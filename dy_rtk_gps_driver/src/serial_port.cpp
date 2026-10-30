#include "rtk_gps_driver_cpp/serial_port.h"
#include <cerrno>
#include <chrono>
#include <iostream>
#include <cstring>
#include <poll.h>
#include <sys/ioctl.h>

namespace rtk_gps_driver_cpp
{
  namespace
  {

    constexpr size_t kMaxBufferSize = 65536;

  } // namespace

  SerialPort::SerialPort()
      : serial_(io_service_), is_open_(false), running_(false) {}

  SerialPort::~SerialPort()
  {
    close();
  }

  bool SerialPort::open(const std::string &port, unsigned int baudrate)
  {
    try
    {
      serial_.open(port);
      serial_.set_option(boost::asio::serial_port::baud_rate(baudrate));
      serial_.set_option(boost::asio::serial_port::character_size(8));
      serial_.set_option(boost::asio::serial_port::parity(
          boost::asio::serial_port::parity::none));
      serial_.set_option(boost::asio::serial_port::stop_bits(
          boost::asio::serial_port::stop_bits::one));
      serial_.set_option(boost::asio::serial_port::flow_control(
          boost::asio::serial_port::flow_control::none));

      int fd = serial_.native_handle();
      struct termios tio;
      if (tcgetattr(fd, &tio) != 0)
      {
        set_last_error(std::string("tcgetattr failed: ") + std::strerror(errno));
        boost::system::error_code ec;
        serial_.close(ec);
        return false;
      }
      tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                       ICRNL | IXON | IXOFF | IXANY);
      tio.c_oflag &= ~OPOST;
      tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
      tio.c_cflag |= (CLOCAL | CREAD);
      tio.c_cc[VMIN] = 0;
      tio.c_cc[VTIME] = 0;
      if (tcsetattr(fd, TCSANOW, &tio) != 0)
      {
        set_last_error(std::string("tcsetattr failed: ") + std::strerror(errno));
        boost::system::error_code ec;
        serial_.close(ec);
        return false;
      }

      set_last_error("");
      {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        rx_buffer_.clear();
      }
      is_open_ = true;
      running_ = true;

      reader_thread_ = std::thread(&SerialPort::reader_thread_func, this);
      return true;
    }
    catch (const boost::system::system_error &e)
    {
      std::cerr << "Failed to open serial port " << port << ": " << e.what() << std::endl;
      set_last_error(e.what());
      is_open_ = false;
      return false;
    }
  }

  void SerialPort::close()
  {
    running_ = false;
    is_open_ = false;

    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      if (serial_.is_open())
      {
        boost::system::error_code ec;
        serial_.close(ec);
      }
    }

    if (reader_thread_.joinable())
    {
      reader_thread_.join();
    }
  }

  std::string SerialPort::last_error() const
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
  }

  void SerialPort::set_last_error(const std::string &error)
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = error;
  }

  void SerialPort::fail_reader(const std::string &error)
  {
    set_last_error(error);
    is_open_ = false;
    running_ = false;
  }

  void SerialPort::reader_thread_func()
  {
    int fd = -1;
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      if (!serial_.is_open())
        return;
      fd = serial_.native_handle();
    }

    uint8_t read_buf[512];

    while (running_.load() && is_open_.load())
    {
      struct pollfd poll_fd = {fd, POLLIN, 0};
      int poll_result;
      do
      {
        poll_result = ::poll(&poll_fd, 1, 100);
      } while (poll_result < 0 && errno == EINTR);

      if (!running_.load() || !is_open_.load())
        break;
      if (poll_result == 0)
        continue;
      if (poll_result < 0)
      {
        fail_reader(std::string("serial read poll failed: ") + std::strerror(errno));
        break;
      }
      if ((poll_fd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0)
      {
        fail_reader("serial read endpoint closed or invalid");
        break;
      }
      if ((poll_fd.revents & POLLIN) == 0)
        continue;

      ssize_t read_size;
      do
      {
        read_size = ::read(fd, read_buf, sizeof(read_buf));
      } while (read_size < 0 && errno == EINTR);
      if (read_size < 0)
      {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
        {
          fail_reader(std::string("serial read failed: ") + std::strerror(errno));
          break;
        }
        continue;
      }
      if (read_size == 0)
        continue;

      std::lock_guard<std::mutex> lock(buffer_mutex_);
      for (ssize_t i = 0; i < read_size; ++i)
      {
        rx_buffer_.push_back(read_buf[i]);
      }
      if (rx_buffer_.size() > kMaxBufferSize)
      {
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + (rx_buffer_.size() - kMaxBufferSize));
      }
    }
  }

  std::vector<uint8_t> SerialPort::drain_bytes()
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    std::vector<uint8_t> result(rx_buffer_.begin(), rx_buffer_.end());
    rx_buffer_.clear();
    return result;
  }

  bool SerialPort::write_with_drain(const std::vector<uint8_t> &data)
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (!is_open_ || !serial_.is_open() || data.empty())
    {
      set_last_error(data.empty() ? "serial write data is empty" : "serial port is closed");
      return false;
    }

    int fd = serial_.native_handle();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);

    size_t total_written = 0;
    while (total_written < data.size())
    {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline)
      {
        set_last_error("serial write timed out waiting for POLLOUT");
        return false;
      }
      const int timeout_ms = static_cast<int>(std::chrono::duration_cast<
                                                  std::chrono::milliseconds>(deadline - now)
                                                  .count());
      struct pollfd poll_fd = {fd, POLLOUT, 0};
      int poll_result;
      do
      {
        poll_result = ::poll(&poll_fd, 1, timeout_ms);
      } while (poll_result < 0 && errno == EINTR);
      if (poll_result <= 0 || (poll_fd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0)
      {
        set_last_error(poll_result == 0 ? "serial write timed out waiting for POLLOUT"
                                        : "serial write endpoint closed or invalid");
        return false;
      }

      ssize_t n = ::write(fd, data.data() + total_written, data.size() - total_written);
      if (n < 0)
      {
        if (errno == EINTR)
        {
          continue;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
          continue;
        }
        set_last_error(std::string("serial write failed: ") + std::strerror(errno));
        return false;
      }
      if (n == 0)
        continue;
      total_written += n;
    }

    while (std::chrono::steady_clock::now() < deadline)
    {
      int queued_bytes = 0;
      if (::ioctl(fd, TIOCOUTQ, &queued_bytes) != 0)
      {
        if (errno == ENOTTY)
        {
          set_last_error("");
          return true;
        }
        set_last_error(std::string("serial output-queue check failed: ") + std::strerror(errno));
        return false;
      }
      if (queued_bytes == 0)
      {
        set_last_error("");
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    set_last_error("serial output queue did not drain within one second");
    return false;
  }

} // namespace rtk_gps_driver_cpp

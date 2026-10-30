#ifndef RTK_GPS_DRIVER_CPP_SERIAL_PORT_H
#define RTK_GPS_DRIVER_CPP_SERIAL_PORT_H

#include <string>
#include <cstdint>
#include <mutex>
#include <vector>
#include <thread>
#include <atomic>
#include <deque>
#include <boost/asio.hpp>
#include <termios.h>
#include <unistd.h>

namespace rtk_gps_driver_cpp
{

  class SerialPort
  {
  public:
    SerialPort();
    ~SerialPort();

    bool open(const std::string &port, unsigned int baudrate);
    void close();
    bool is_open() const { return is_open_.load(); }
    std::string last_error() const;

    std::vector<uint8_t> drain_bytes();

    bool write_with_drain(const std::vector<uint8_t> &data);

  private:
    void reader_thread_func();
    void fail_reader(const std::string &error);
    void set_last_error(const std::string &error);

    boost::asio::io_service io_service_;
    boost::asio::serial_port serial_;
    std::mutex serial_mutex_;
    std::mutex buffer_mutex_;
    mutable std::mutex error_mutex_;
    std::deque<uint8_t> rx_buffer_;
    std::string last_error_;

    std::atomic<bool> is_open_;
    std::atomic<bool> running_;
    std::thread reader_thread_;
  };

} // namespace rtk_gps_driver_cpp

#endif // RTK_GPS_DRIVER_CPP_SERIAL_PORT_H

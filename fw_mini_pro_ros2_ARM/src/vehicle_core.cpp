#include "fw_mini_pro.hpp"
#include "command_watchdog.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

// ─────────────────────────────────────────────
// 位域打包/解包（Intel 小端位域，见头文件说明）
// ─────────────────────────────────────────────

uint32_t FwMiniProVehicle::get_bits(const uint8_t* data, int start_bit, int length)
{
    uint32_t value = 0;
    for (int i = 0; i < length; ++i) {
        int bit_index   = start_bit + i;
        int byte_index  = bit_index / 8;
        int bit_in_byte = bit_index % 8;
        uint32_t bit = (data[byte_index] >> bit_in_byte) & 0x01;
        value |= (bit << i);
    }
    return value;
}

void FwMiniProVehicle::set_bits(uint8_t* data, int start_bit, int length, uint32_t value)
{
    for (int i = 0; i < length; ++i) {
        int bit_index   = start_bit + i;
        int byte_index  = bit_index / 8;
        int bit_in_byte = bit_index % 8;
        uint8_t bit = (value >> i) & 0x01;
        data[byte_index] = (uint8_t)((data[byte_index] & ~(1 << bit_in_byte)) | (bit << bit_in_byte));
    }
}

int32_t FwMiniProVehicle::sign_extend(uint32_t value, int length)
{
    uint32_t mask = 1u << (length - 1);
    return (int32_t)((value ^ mask) - mask);
}

uint8_t FwMiniProVehicle::bcc(const uint8_t* data)
{
    uint8_t c = 0;
    for (int i = 0; i < 7; ++i) c ^= data[i];
    return c;
}

// ─────────────────────────────────────────────
// 构造/析构
// ─────────────────────────────────────────────

FwMiniProVehicle::FwMiniProVehicle(rclcpp::Node& node) : node_(&node)
{
    feedback_pub_ = node_->create_publisher<can_client::msg::VehicleFeedback>("vehicle_feedback", 10);
    fault_reset_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
        "fault_reset", 10,
        std::bind(&FwMiniProVehicle::callbackFaultReset, this, std::placeholders::_1));

    can_iface_ = node_->declare_parameter<std::string>("can_iface", "can0");
    // 整车运行速度 0~5.4km/h ≈ 1.5m/s（见用户手册 §2.2 性能参数）
    max_linear_speed_ = node_->declare_parameter<double>("max_linear_speed", 1.5);
    max_angular_speed_ = node_->declare_parameter<double>("max_angular_speed", 3.0);

    if (open_can_socket(can_iface_, can_sock_)) {
        recv_thread_ = std::thread(&FwMiniProVehicle::recv_loop, this);
        RCLCPP_INFO(node_->get_logger(), "[fw_mini_pro] %s opened", can_iface_.c_str());
    } else {
        RCLCPP_WARN(node_->get_logger(), "[fw_mini_pro] failed to open %s", can_iface_.c_str());
    }
}

FwMiniProVehicle::~FwMiniProVehicle()
{
    running_ = false;
    if (can_sock_ >= 0) {
        ::shutdown(can_sock_, SHUT_RDWR);
        ::close(can_sock_);
        can_sock_ = -1;
    }
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
}

// ─────────────────────────────────────────────
// ROS 回调
// ─────────────────────────────────────────────

void FwMiniProVehicle::vehicle_cmd(const can_client::msg::VehicleCmd& msg)
{
    std::lock_guard<std::mutex> lock(mutex_);

    enable_ = msg.enable;

    double lx = std::max(-max_linear_speed_,  std::min(max_linear_speed_,  (double)msg.linear_x));
    double ly = std::max(-max_linear_speed_,  std::min(max_linear_speed_,  (double)msg.linear_y));
    double az = std::max(-max_angular_speed_, std::min(max_angular_speed_, (double)msg.angular_z));

    x_vel_mmps_  = (int16_t)std::lround(lx * 1000.0);
    y_vel_mmps_  = (int16_t)std::lround(ly * 1000.0);
    z_avel_cdps_ = (int16_t)std::lround(az * (180.0 / M_PI) * 100.0);

    switch (msg.gear) {
        case can_client::msg::VehicleCmd::GEAR_PARK:    target_gear_ = GEAR_PARK;    break;
        case can_client::msg::VehicleCmd::GEAR_NEUTRAL: target_gear_ = GEAR_NEUTRAL; break;
        case can_client::msg::VehicleCmd::GEAR_CRAB:    target_gear_ = GEAR_CRAB;    break;
        case can_client::msg::VehicleCmd::GEAR_4WD4WS:
        default:                                   target_gear_ = GEAR_4WD4WS; break;
    }
    if (!enable_) target_gear_ = GEAR_DISABLE;
}

void FwMiniProVehicle::callbackFaultReset(const std_msgs::msg::UInt8& msg)
{
    if (msg.data == 1) {
        std::lock_guard<std::mutex> lock(mutex_);
        unlock_hold_frames_ = 3;   // 保持几帧后自然释放，形成"安全停车解锁"所需的下降沿
        RCLCPP_INFO(node_->get_logger(), "[fw_mini_pro] safe-stop unlock requested");
    }
}

void FwMiniProVehicle::sendDisableMessage()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        enable_ = false;
        target_gear_ = GEAR_DISABLE;
        x_vel_mmps_ = z_avel_cdps_ = y_vel_mmps_ = 0;
    }
    // Shutdown must actually transmit the zero command, not just update memory.
    ctrl_cmd_send();
}

// ─────────────────────────────────────────────
// 周期发送（心跳重发，由外部按固定频率调用）
// ─────────────────────────────────────────────

void FwMiniProVehicle::run()
{
    ctrl_cmd_send();
    io_cmd_send();

    can_client::msg::VehicleFeedback fb_copy;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        fb_copy = feedback_;
    }
    fb_copy.header.stamp = node_->get_clock()->now();
    feedback_pub_->publish(fb_copy);
}

// ─────────────────────────────────────────────
// 0x18C4D1D0 ctrl_cmd 发送
// ─────────────────────────────────────────────

void FwMiniProVehicle::ctrl_cmd_send()
{
    uint8_t gear;
    int16_t xv, zv, yv;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // 急停触发时强制驻车并清零速度指令，避免顶着急停请求继续下发运动指令
        gear = feedback_.estop ? (uint8_t)GEAR_PARK : target_gear_;
        xv = (gear == GEAR_4WD4WS || gear == GEAR_CRAB) ? x_vel_mmps_ : 0;
        zv = (gear == GEAR_4WD4WS) ? z_avel_cdps_ : 0;   // 仅 4T4D 档有效
        yv = (gear == GEAR_CRAB)   ? y_vel_mmps_  : 0;   // 仅横移档有效
        if (feedback_.estop) { xv = 0; zv = 0; yv = 0; }
        ctrl_counter_ = (ctrl_counter_ + 1) & 0x0F;
    }

    uint8_t data[8] = {0};
    set_bits(data, 0,  4,  gear & 0x0F);
    set_bits(data, 4,  16, (uint16_t)xv);
    set_bits(data, 20, 16, (uint16_t)zv);
    set_bits(data, 36, 16, (uint16_t)yv);
    set_bits(data, 52, 4,  ctrl_counter_);
    data[7] = bcc(data);

    send_frame(0x18C4D1D0, data);
}

// ─────────────────────────────────────────────
// 0x18C4D7D0 io_cmd 发送（灯光自动模式 + 安全停车解锁脉冲）
// ─────────────────────────────────────────────

void FwMiniProVehicle::io_cmd_send()
{
    uint8_t unlock_bit = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (unlock_hold_frames_ > 0) {
            unlock_bit = 1;
            unlock_hold_frames_--;
        }
        io_counter_ = (io_counter_ + 1) & 0x0F;
    }

    uint8_t data[8] = {0};
    set_bits(data, 0, 1, 0);            // 灯控制权模式：0=按状态自动控制
    set_bits(data, 1, 1, unlock_bit);   // 安全停车解锁开关：下降沿有效
    set_bits(data, 2, 1, 0);            // 低功耗模式启用开关：常闭，避免自动驻车打断自动驾驶
    set_bits(data, 52, 4, io_counter_);
    data[7] = bcc(data);

    send_frame(0x18C4D7D0, data);
}

// ─────────────────────────────────────────────
// 反馈帧解析
// ─────────────────────────────────────────────

void FwMiniProVehicle::parse_ctrl_fb(const uint8_t* data)
{
    uint32_t gear = get_bits(data, 0, 4);
    int32_t  xv   = sign_extend(get_bits(data, 4,  16), 16);
    int32_t  zv   = sign_extend(get_bits(data, 20, 16), 16);
    int32_t  yv   = sign_extend(get_bits(data, 36, 16), 16);

    std::lock_guard<std::mutex> lock(mutex_);
    feedback_.gear      = (uint8_t)gear;
    feedback_.linear_x  = xv / 1000.0f;
    feedback_.linear_y  = yv / 1000.0f;
    feedback_.angular_z = (zv / 100.0f) * (float)(M_PI / 180.0);   // 0.01°/s -> rad/s
}

void FwMiniProVehicle::parse_io_fb(const uint8_t* data)
{
    bool estop         = get_bits(data, 40, 1);
    bool remote_ctrl   = get_bits(data, 41, 1);
    bool charge_dock   = get_bits(data, 42, 1);
    bool manual_charge = get_bits(data, 43, 1);

    bool bump_fl = get_bits(data, 24, 1);
    bool bump_fc = get_bits(data, 25, 1);
    bool bump_fr = get_bits(data, 26, 1);
    bool bump_rl = get_bits(data, 27, 1);
    bool bump_rc = get_bits(data, 28, 1);
    bool bump_rr = get_bits(data, 29, 1);

    std::lock_guard<std::mutex> lock(mutex_);
    feedback_.estop               = estop;
    feedback_.remote_control       = remote_ctrl;
    feedback_.charging              = charge_dock || manual_charge;
    feedback_.bumper_front_left    = bump_fl;
    feedback_.bumper_front_center  = bump_fc;
    feedback_.bumper_front_right   = bump_fr;
    feedback_.bumper_rear_left     = bump_rl;
    feedback_.bumper_rear_center   = bump_rc;
    feedback_.bumper_rear_right    = bump_rr;
}

void FwMiniProVehicle::parse_bms_fb(const uint8_t* data)
{
    uint32_t voltage = get_bits(data, 0, 16);
    int32_t  current = sign_extend(get_bits(data, 16, 16), 16);

    std::lock_guard<std::mutex> lock(mutex_);
    feedback_.battery_voltage = voltage / 100.0f;
    feedback_.battery_current = current / 100.0f;
}

void FwMiniProVehicle::parse_bms_flag_fb(const uint8_t* data)
{
    uint32_t percent      = get_bits(data, 0, 8);
    uint32_t protect_bits = get_bits(data, 8, 14);   // 单体/整组过欠压、充放电过温欠温、过流、短路等保护位
    int32_t  temp_max     = sign_extend(get_bits(data, 28, 12), 12);
    int32_t  temp_min     = sign_extend(get_bits(data, 40, 12), 12);

    std::lock_guard<std::mutex> lock(mutex_);
    feedback_.battery_percent  = (float)percent;
    feedback_.battery_temp_max = temp_max / 10.0f;
    feedback_.battery_temp_min = temp_min / 10.0f;
    feedback_.battery_fault    = (protect_bits != 0);
}

void FwMiniProVehicle::parse_error_fb(const uint8_t* data)
{
    uint32_t level = get_bits(data, 0,  8);
    uint32_t comp  = get_bits(data, 8,  8);
    uint32_t id    = get_bits(data, 16, 8);
    uint32_t code  = get_bits(data, 24, 24);

    std::lock_guard<std::mutex> lock(mutex_);
    feedback_.fault_level     = (uint8_t)level;
    feedback_.fault_component = (uint8_t)comp;
    feedback_.fault_id        = (uint8_t)id;
    feedback_.fault_code      = code;
}

// ─────────────────────────────────────────────
// SocketCAN 直连
// ─────────────────────────────────────────────

bool FwMiniProVehicle::open_can_socket(const std::string& iface, int& fd)
{
    fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) return false;

    struct ifreq ifr{};
    strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        ::close(fd); fd = -1; return false;
    }

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        ::close(fd); fd = -1; return false;
    }
    return true;
}

void FwMiniProVehicle::send_frame(uint32_t ext_id, const uint8_t data[8])
{
    if (can_sock_ < 0) return;
    struct can_frame frame{};
    frame.can_id  = (ext_id & CAN_EFF_MASK) | CAN_EFF_FLAG;   // 29位扩展帧
    frame.can_dlc = 8;
    memcpy(frame.data, data, 8);
    ::write(can_sock_, &frame, sizeof(frame));
}

void FwMiniProVehicle::recv_loop()
{
    struct can_frame frame;
    while (running_) {
        int n = ::read(can_sock_, &frame, sizeof(frame));
        if (n != (int)sizeof(frame)) continue;

        uint32_t id = frame.can_id & CAN_EFF_MASK;
        switch (id) {
            case 0x18C4D1EF: parse_ctrl_fb(frame.data);     break;
            case 0x18C4DAEF: parse_io_fb(frame.data);       break;
            case 0x18C4E1EF: parse_bms_fb(frame.data);      break;
            case 0x18C4E2EF: parse_bms_flag_fb(frame.data); break;
            case 0x18C4EAEF: parse_error_fb(frame.data);    break;
            default: break;
        }
    }
}

// ─────────────────────────────────────────────
// vehicle_core：订阅 /vehicle_cmd，以固定频率驱动车辆驱动的心跳重发
// ─────────────────────────────────────────────

class VehicleCore : public rclcpp::Node
{
public:
    VehicleCore() : rclcpp::Node("vehicle_core"),
        command_watchdog_(declare_parameter<double>("command_timeout_sec", 0.25))
    {
        update_rate_ = declare_parameter<double>("update_rate", 20.0);
        if (!std::isfinite(update_rate_) || update_rate_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "update_rate must be positive; using 20 Hz");
            update_rate_ = 20.0;
        }

        vehicle_ = std::make_unique<FwMiniProVehicle>(*this);
        vehicle_cmd_sub_ = create_subscription<can_client::msg::VehicleCmd>(
            "vehicle_cmd", rclcpp::QoS(1).reliable(),
            std::bind(&VehicleCore::vehicleCmdCallback, this, std::placeholders::_1));
        timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / update_rate_)),
            [this]() {
                if (!command_watchdog_.fresh(std::chrono::steady_clock::now())) {
                    // Continually send disabled heartbeat frames until a fresh command arrives.
                    vehicle_->vehicle_cmd(can_client::msg::VehicleCmd{});
                }
                vehicle_->run();
            });
    }

    void shutdown_vehicle()
    {
        if (vehicle_) {
            vehicle_->sendDisableMessage();
        }
    }

private:
    rclcpp::Subscription<can_client::msg::VehicleCmd>::SharedPtr vehicle_cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double update_rate_;
    CommandWatchdog command_watchdog_;
    std::unique_ptr<VehicleInferface> vehicle_;

    void vehicleCmdCallback(const can_client::msg::VehicleCmd::SharedPtr msg)
    {
        if (!std::isfinite(msg->linear_x) || !std::isfinite(msg->linear_y) ||
            !std::isfinite(msg->angular_z)) {
            command_watchdog_.invalidate();
            vehicle_->sendDisableMessage();
            RCLCPP_ERROR(get_logger(), "Invalid vehicle command; chassis disabled");
            return;
        }
        command_watchdog_.received(std::chrono::steady_clock::now());
        vehicle_->vehicle_cmd(*msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VehicleCore>();
    rclcpp::spin(node);
    node->shutdown_vehicle();
    RCLCPP_INFO(node->get_logger(), "[vehicle_core] node exiting.");
    rclcpp::shutdown();
    return 0;
}

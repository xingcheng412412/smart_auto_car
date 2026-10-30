#ifndef _FW_MINI_PRO_H_
#define _FW_MINI_PRO_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <cstdint>
#include <cstring>
#include <string>
#include <mutex>
#include <atomic>
#include <thread>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <can_client/msg/vehicle_cmd.hpp>
#include <can_client/msg/vehicle_feedback.hpp>

// 车辆驱动抽象接口：vehicle_core.cpp 通过该接口驱动具体车型的实现
class VehicleInferface
{
public:
    VehicleInferface(){};
    virtual ~VehicleInferface(){};

    virtual void vehicle_cmd(const can_client::msg::VehicleCmd& msg) = 0;
    virtual void sendDisableMessage() = 0;
    virtual void run() = 0;
};

// FW-mini-pro 全向型智能线控底盘 CAN 通讯协议
// 深圳煜禾森科技有限公司《FW-mini-pro 用户使用手册》V1.0.0 §4.4/4.5
//
// CAN2.0B 扩展帧（29位ID），波特率 500kbps，信号排列格式 Intel（小端位域）。
// 多字节信号按小端整数跨位打包：起始位 = 字节序号*8 + 字节内偏移，
// 从起始位开始的 N 个比特位为该信号的 bit0..bit(N-1)。
//
// 发送帧（ADAS → VCU）：
//   0x18C4D1D0 ctrl_cmd  10~50ms  目标档位(4bit)+X轴线速度(16bit,0.001m/s)+
//                                  Z轴角速度(16bit,0.01°/s)+Y轴线速度(16bit,0.001m/s)+
//                                  心跳(4bit)+BCC校验(8bit)
//   0x18C4D7D0 io_cmd    10ms     灯控制权/安全停车解锁(下降沿有效)/低功耗使能+心跳+BCC校验
//
// 接收帧（VCU → ADAS）：
//   0x18C4D1EF ctrl_fb   10ms     当前档位+X/Z/Y反馈+心跳+BCC校验
//   0x18C4DAEF io_fb     50ms     灯光/急停/遥控在线/充电/防撞条状态反馈
//   0x18C4E1EF bms_fb    100ms    电池电压(0.01V)/电流(0.01A)/剩余容量(0.01Ah)
//   0x18C4E2EF bms_flag_fb 100ms  电池剩余百分比/保护标志位/最高最低温度(0.1℃)
//   0x18C4EAEF error_fb  100ms    故障等级/部件类型/部件id/故障编码
//
// BCC 校验：Checksum = Byte0 XOR Byte1 XOR ... XOR Byte6，存于 Byte7。

class FwMiniProVehicle : public VehicleInferface
{
public:
    explicit FwMiniProVehicle(rclcpp::Node& node);
    ~FwMiniProVehicle() override;

    void vehicle_cmd(const can_client::msg::VehicleCmd& msg) override;
    void sendDisableMessage() override;
    void run() override;

private:
    // ctrl_cmd/ctrl_fb 目标档位编码
    enum Gear : uint8_t
    {
        GEAR_DISABLE = 0x00,
        GEAR_PARK    = 0x01,
        GEAR_NEUTRAL = 0x02,
        GEAR_4WD4WS  = 0x06,
        GEAR_CRAB    = 0x08,
    };

    rclcpp::Node* node_;
    rclcpp::Publisher<can_client::msg::VehicleFeedback>::SharedPtr feedback_pub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr fault_reset_sub_;

    std::string can_iface_;
    double max_linear_speed_;    // m/s，限制 linear_x/linear_y
    double max_angular_speed_;   // rad/s，限制 angular_z

    // 下发状态与反馈状态共用一把锁：驱动本身速率低（10~100Hz），无需分离
    std::mutex mutex_;

    bool    enable_       = false;
    uint8_t target_gear_  = GEAR_DISABLE;
    int16_t x_vel_mmps_   = 0;   // 0.001 m/s/bit
    int16_t z_avel_cdps_  = 0;   // 0.01 °/s/bit
    int16_t y_vel_mmps_   = 0;   // 0.001 m/s/bit

    uint8_t ctrl_counter_ = 0;   // 0~15 循环
    uint8_t io_counter_   = 0;   // 0~15 循环
    int     unlock_hold_frames_ = 0;   // >0 时 io_cmd 解锁位置1，倒计时到0后置0形成下降沿

    can_client::msg::VehicleFeedback feedback_;

    int can_sock_ = -1;
    std::thread recv_thread_;
    std::atomic<bool> running_{true};

    bool open_can_socket(const std::string& iface, int& fd);
    void send_frame(uint32_t ext_id, const uint8_t data[8]);
    void recv_loop();

    void callbackFaultReset(const std_msgs::msg::UInt8& msg);

    void ctrl_cmd_send();   // 0x18C4D1D0
    void io_cmd_send();     // 0x18C4D7D0

    void parse_ctrl_fb(const uint8_t* data);      // 0x18C4D1EF
    void parse_io_fb(const uint8_t* data);        // 0x18C4DAEF
    void parse_bms_fb(const uint8_t* data);       // 0x18C4E1EF
    void parse_bms_flag_fb(const uint8_t* data);  // 0x18C4E2EF
    void parse_error_fb(const uint8_t* data);     // 0x18C4EAEF

    static uint32_t get_bits(const uint8_t* data, int start_bit, int length);
    static void     set_bits(uint8_t* data, int start_bit, int length, uint32_t value);
    static int32_t  sign_extend(uint32_t value, int length);
    static uint8_t  bcc(const uint8_t* data);   // Byte0^...^Byte6
};

#endif

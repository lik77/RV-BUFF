// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"
// 命名空间声明，用于封装RM串口驱动相关功能
namespace rm_serial_driver
{
  /**
   * @brief RM串口驱动节点构造函数
   * 
   * @param options 节点配置选项，用于初始化节点
   */
  RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
    : Node("rm_serial_driver", options),
      owned_ctx_{new IoContext(2)},
      serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
  {
    RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

    // 初始化参数获取
    getParams();

    // 声明时间戳偏移参数，并创建tf广播器
    timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 创建延迟和瞄准点标记的发布者
    latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);

    // 创建装甲检测器参数客户端
    detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

    // 创建重置追踪器的客户端
    reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

    try {
      // 初始化串口并尝试打开
      serial_driver_->init_port(device_name_, *device_config_);
      if (!serial_driver_->port()->is_open()) {
        serial_driver_->port()->open();
        // 启动接收数据线程
        receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
      throw ex;
    }

    // 初始化瞄准点标记消息
    aiming_point_.header.frame_id = "odom";
    aiming_point_.ns = "aiming_point";
    aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
    aiming_point_.action = visualization_msgs::msg::Marker::ADD;
    aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
    aiming_point_.color.r = 1.0;
    aiming_point_.color.g = 1.0;
    aiming_point_.color.b = 1.0;
    aiming_point_.color.a = 1.0;
    aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

    // 创建目标订阅者，用于接收追踪目标数据
    target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
      "/tracker/target", rclcpp::SensorDataQoS(),
      std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));
  }

// RMSerialDriver类的析构函数
RMSerialDriver::~RMSerialDriver()
{
  // 确保接收线程存在且可以加入，然后加入接收线程
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  // 检查串口是否打开，如果是，则关闭串口
  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  // 检查是否拥有上下文对象，如果是，则等待上下文对象退出
  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}
/**
 * @brief 接收串口数据并处理
 * 
 * 该函数负责从串口接收数据，并根据接收到的数据进行处理
 * 它首先接收一个字节的头部，如果头部符合预期，则继续接收剩余的数据
 * 接收到完整数据包后，它会验证CRC校验和，如果校验通过，则更新参数、重置跟踪器、广播变换和发布瞄准点
 * 如果CRC校验失败，则输出错误日志
 * 如果接收到无效的头部，则输出警告日志
 * 如果在接收数据过程中发生异常，则重新打开串口
 */
void RMSerialDriver::receiveData()
{
  // 初始化头部缓冲区
  std::vector<uint8_t> header(1);
  // 初始化数据缓冲区，并预留足够空间
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacket));

  // 循环接收数据
  while (rclcpp::ok()) {
    try {
      // 接收一个字节的头部
      serial_driver_->port()->receive(header);

      // 如果头部符合预期，则继续接收剩余的数据
      if (header[0] == 0x5A) {
        data.resize(sizeof(ReceivePacket) - 1);
        serial_driver_->port()->receive(data);

        // 将头部插入数据缓冲区
        data.insert(data.begin(), header[0]);
        // 将接收到的数据转换为ReceivePacket对象
        ReceivePacket packet = fromVector(data);

        // 验证CRC校验和
        bool crc_ok =
          crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
        // 如果CRC校验通过，则处理数据
        if (crc_ok) {
          // 更新检测颜色参数
          if (!initial_set_param_ || packet.detect_color != previous_receive_color_) {
            setParam(rclcpp::Parameter("detect_color", packet.detect_color));
            previous_receive_color_ = packet.detect_color;
          }

          // 如果需要重置跟踪器，则调用resetTracker函数
          if (packet.reset_tracker) {
            resetTracker();
          }

          // 创建并广播变换
          geometry_msgs::msg::TransformStamped t;
          timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
          t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
          t.header.frame_id = "odom";
          t.child_frame_id = "gimbal_link";
          tf2::Quaternion q;
          q.setRPY(packet.roll, packet.pitch, packet.yaw);
          t.transform.rotation = tf2::toMsg(q);
          tf_broadcaster_->sendTransform(t);

          // 如果瞄准点的x坐标大于阈值，则发布瞄准点
          if (abs(packet.aim_x) > 0.01) {
            aiming_point_.header.stamp = this->now();
            aiming_point_.pose.position.x = packet.aim_x;
            aiming_point_.pose.position.y = packet.aim_y;
            aiming_point_.pose.position.z = packet.aim_z;
            marker_pub_->publish(aiming_point_);
          }
        } else {
          // 如果CRC校验失败，则输出错误日志
          RCLCPP_ERROR(get_logger(), "CRC error!");
        }
      } else {
        // 如果接收到无效的头部，则输出警告日志
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
      }
    } catch (const std::exception & ex) {
      // 如果在接收数据过程中发生异常，则输出错误日志并重新打开串口
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

/**
 * @brief 发送目标数据到串口
 * 
 * 本函数负责将接收到的目标数据打包并通过串口发送出去。它首先根据目标的ID查找对应的uint8_t值，
 * 然后将数据填充到SendPacket结构体中，并计算CRC16校验和。最后，将数据转换为字节流并通过串口发送。
 * 如果在发送过程中遇到异常，将重新打开串口连接。
 * 
 * @param msg 共享指针，指向要发送的目标消息，包含目标的跟踪状态、ID、装甲板数量、位置、速度等信息
 */
void RMSerialDriver::sendData(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  // 静态映射表，将字符串ID映射到uint8_t类型
  const static std::map<std::string, uint8_t> id_unit8_map{
    {"", 0},  {"outpost", 0}, {"1", 1}, {"1", 1},     {"2", 2},
    {"3", 3}, {"4", 4},       {"5", 5}, {"guard", 6}, {"base", 7}};

  try {
    // 创建数据包实例
    SendPacket packet;
    // 填充跟踪状态
    packet.tracking = msg->tracking;
    // 根据消息中的ID查找对应的uint8_t值，并填充到数据包中
    packet.id = id_unit8_map.at(msg->id);
    // 填充装甲板数量
    packet.armors_num = msg->armors_num;
    // 填充目标位置坐标
    packet.x = msg->position.x;
    packet.y = msg->position.y;
    packet.z = msg->position.z;
    // 填充目标 yaw 角度
    packet.yaw = msg->yaw;
    // 填充目标速度
    packet.vx = msg->velocity.x;
    packet.vy = msg->velocity.y;
    packet.vz = msg->velocity.z;
    // 填充目标 yaw 速度
    packet.v_yaw = msg->v_yaw;
    // 填充目标的两个半径和高度差
    packet.r1 = msg->radius_1;
    packet.r2 = msg->radius_2;
    packet.dz = msg->dz;
    // 计算并附加CRC16校验和
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

    // 将数据包转换为字节流
    std::vector<uint8_t> data = toVector(packet);

    // 通过串口发送数据
    serial_driver_->port()->send(data);

    // 计算并发布总延迟
    std_msgs::msg::Float64 latency;
    latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
    RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
    latency_pub_->publish(latency);
  } catch (const std::exception & ex) {
    // 捕获异常并重新打开串口
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

/**
 * @brief 获取并设置串口通信参数
 * 
 * 本函数从参数服务器获取串口通信所需的各项配置，包括设备名称、波特率、流控制、校验位和停止位，
 * 并根据这些参数初始化串口配置对象device_config_。函数使用了参数类型别名和条件语句来解析和验证参数，
 * 确保参数的有效性和适用性。若参数无效或不满足要求，函数将抛出异常并记录错误信息。
 */
void RMSerialDriver::getParams()
{
  // 类型别名，用于简化枚举类型的使用
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  // 初始化串口配置变量
  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  // 尝试获取设备名称参数，若无效则抛出异常
  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  // 尝试获取波特率参数，若无效则抛出异常
  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  // 尝试获取流控制参数并根据其值设置相应的枚举值，若无效则抛出异常
  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  // 尝试获取校验位参数并根据其值设置相应的枚举值，若无效则抛出异常
  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  // 尝试获取停止位参数并根据其值设置相应的枚举值，若无效则抛出异常
  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  // 根据获取的参数创建并初始化串口配置对象
  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

/**
 * 重新打开串口连接
 * 
 * 本函数尝试重新打开串口连接如果串口当前是打开状态，会先关闭再重新打开
 * 如果重新打开过程中发生异常，会记录错误信息，并在1秒后再次尝试重新打开串口
 */
void RMSerialDriver::reopenPort()
{
  // 记录重新打开串口的尝试
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    // 如果串口是打开状态，则先关闭串口
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    // 重新打开串口
    serial_driver_->port()->open();
    // 记录成功重新打开串口的信息
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    // 记录重新打开串口时发生的错误
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    // 如果rclcpp状态正常，等待1秒后再次尝试重新打开串口
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

/**
 * @brief 设置参数到检测器
 * 
 * 本函数负责将给定的参数发送到检测器节点，以更新其配置
 * 它首先检查服务是否准备就绪，然后发送参数更新请求
 * 如果参数设置成功，它将标记初始参数设置完成
 * 
 * @param param 要设置的参数，通常是一个整数类型
 */
void RMSerialDriver::setParam(const rclcpp::Parameter & param)
{
  // 检查服务是否准备就绪，如果没有准备好，则记录警告并返回
  if (!detector_param_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    return;
  }

  // 检查是否有未完成的参数设置操作，如果没有或者已经完成，则开始设置新参数
  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    // 记录日志，开始设置参数
    RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
    // 发送参数设置请求，并在完成后调用回调函数
    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        // 遍历参数设置结果，如果有任何参数设置失败，则记录错误
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        // 如果所有参数都设置成功，则记录日志并标记初始参数设置完成
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }
}

void RMSerialDriver::resetTracker()
{
  if (!reset_tracker_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_tracker_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Reset tracker!");
}

}  

#include "rclcpp_components/register_node_macro.hpp"


RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)

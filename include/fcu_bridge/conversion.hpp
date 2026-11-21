#pragma once

#include <string>

#include <rclcpp/serialized_message.hpp>
#include <std_msgs/msg/header.hpp>

namespace fcu_bridge
{
/**
 * @brief 判断是否已实现指定的跨类型转换；同类型消息由节点直接转发。
 * @param input_type 输入 ROS 类型全名。
 * @param output_type 输出 ROS 类型全名。
 * @return 支持时为 true。
 */
bool CanConvert(const std::string& input_type, const std::string& output_type);

/**
 * @brief 转换一条消息，不执行坐标变换或轨迹跟踪。
 * @param input_type 输入 ROS 类型全名。
 * @param output_type 输出 ROS 类型全名。
 * @param input 输入序列化消息。
 * @param fallback_header 无 Header 的输入使用的时间戳和坐标系。
 * @param output 转换后的序列化消息。
 * @return 输入含有所需字段且转换成功时为 true。
 */
bool Convert(const std::string& input_type, const std::string& output_type,
    const rclcpp::SerializedMessage& input, const std_msgs::msg::Header& fallback_header,
    rclcpp::SerializedMessage& output);
}  // namespace fcu_bridge

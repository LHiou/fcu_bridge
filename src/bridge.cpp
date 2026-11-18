#include <exception>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "message_bridge/conversion.hpp"

namespace message_bridge
{
class Bridge : public rclcpp::Node
{
public:
    /** @brief 创建消息桥节点，连接工作由 Start 完成。 */
    Bridge() : Node("message_bridge")
    {
        // 参数在启动时读取，运行期间不切换消息类型。
    }

    /** @brief 读取配置并连接话题。
     * @return 配置和类型组合有效时为 true。 */
    bool Start()
    {
        // 一条链路只负责一个输入和一个输出，使用命名空间扩展到多机。
        const auto input_topic = Parameter<std::string>("input_topic", "control/manual");
        const auto output_topic =
            Parameter<std::string>("output_topic", "mavros/setpoint_raw/local");
        input_type_ = Parameter<std::string>(
            "input_type", "trajectory_msgs/msg/MultiDOFJointTrajectoryPoint");
        output_type_ = Parameter<std::string>("output_type", "mavros_msgs/msg/PositionTarget");
        frame_id_ = Parameter<std::string>("frame_id", "map");
        relay_ = input_type_ == output_type_;
        if (!relay_ && !CanConvert(input_type_, output_type_))
        {
            RCLCPP_ERROR(
                get_logger(), "不支持此转换：%s -> %s", input_type_.c_str(), output_type_.c_str());
            return false;
        }

        const auto topics = get_node_topics_interface();
        if (topics->resolve_topic_name(input_topic) == topics->resolve_topic_name(output_topic))
        {
            RCLCPP_ERROR(get_logger(), "输入和输出解析后为同一话题，会产生循环转发");
            return false;
        }
        const int depth = Parameter<int>("queue_depth", 10);
        if (depth < 1)
        {
            RCLCPP_ERROR(get_logger(), "queue_depth 必须大于零");
            return false;
        }
        rclcpp::QoS input_qos(depth);
        rclcpp::QoS output_qos(depth);
        if (!ConfigureQos("input", "best_effort", input_qos) ||
            !ConfigureQos("output", "reliable", output_qos))
        {
            return false;
        }

        publisher_ = create_generic_publisher(output_topic, output_type_, output_qos);
        subscription_ = create_generic_subscription(input_topic, input_type_, input_qos,
            std::bind(&Bridge::OnMessage, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "%s [%s] -> %s [%s] (%s)", subscription_->get_topic_name(),
            input_type_.c_str(), publisher_->get_topic_name(), output_type_.c_str(),
            relay_ ? "原样转发" : "字段转换");
        return true;
    }

private:
    /** @brief 声明启动参数。
     * @param name 参数名。
     * @param value 默认值。
     * @return 实际配置值。 */
    template <typename Value>
    Value Parameter(const std::string& name, const Value& value)
    {
        // 禁止运行期间修改而不生效，修改配置后应重新启动节点。
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        return declare_parameter<Value>(name, value, descriptor);
    }

    /**
     * @brief 配置一个方向的 DDS 服务质量。
     * @param prefix input 或 output。
     * @param default_reliability 默认可靠性。
     * @param qos 待设置的 QoS。
     * @return 配置值有效时为 true。
     */
    bool ConfigureQos(
        const std::string& prefix, const std::string& default_reliability, rclcpp::QoS& qos)
    {
        // 输入和输出独立配置，兼容传感器数据及需要保留的地图数据。
        const auto reliability =
            Parameter<std::string>(prefix + "_reliability", default_reliability);
        const auto durability = Parameter<std::string>(prefix + "_durability", "volatile");
        if (reliability == "reliable")
        {
            qos.reliable();
        }
        else if (reliability == "best_effort")
        {
            qos.best_effort();
        }
        else
        {
            RCLCPP_ERROR(
                get_logger(), "%s_reliability 只能为 reliable 或 best_effort", prefix.c_str());
            return false;
        }
        if (durability == "transient_local")
        {
            qos.transient_local();
        }
        else if (durability == "volatile")
        {
            qos.durability_volatile();
        }
        else
        {
            RCLCPP_ERROR(
                get_logger(), "%s_durability 只能为 volatile 或 transient_local", prefix.c_str());
            return false;
        }
        return true;
    }

    /** @brief 处理一条输入。
     * @param input ROS 序列化消息。 */
    void OnMessage(std::shared_ptr<rclcpp::SerializedMessage> input)
    {
        // 同类型直接转发，跨类型只发布具备所需字段的转换结果。
        if (relay_)
        {
            publisher_->publish(*input);
            return;
        }
        std_msgs::msg::Header fallback;
        fallback.stamp = now();
        fallback.frame_id = frame_id_;
        rclcpp::SerializedMessage output;
        if (Convert(input_type_, output_type_, *input, fallback, output))
        {
            publisher_->publish(output);
        }
        else
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "跳过消息：缺少目标类型所需字段，或轨迹点包含多个关节");
        }
    }

    std::string input_type_;
    std::string output_type_;
    std::string frame_id_;
    bool relay_ = false;
    rclcpp::GenericPublisher::SharedPtr publisher_;
    rclcpp::GenericSubscription::SharedPtr subscription_;
};
}  // namespace message_bridge

/** @brief 启动消息桥。
 * @param argc 参数数量。
 * @param argv ROS 参数。
 * @return 成功为 0，失败为 1。 */
int main(int argc, char** argv)
{
    // 业务代码使用返回值报告错误；只在进程入口接收 ROS 库自身的异常。
    int result = 0;
    try
    {
        rclcpp::init(argc, argv);
        auto bridge = std::make_shared<message_bridge::Bridge>();
        if (bridge->Start())
        {
            rclcpp::spin(bridge);
        }
        else
        {
            result = 1;
        }
    }
    catch (const std::exception& error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("message_bridge"), "%s", error.what());
        result = 1;
    }
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
    return result;
}

#include <chrono>
#include <cmath>
#include <exception>
#include <optional>
#include <functional>
#include <memory>
#include <string>

#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "fcu_bridge/conversion.hpp"

namespace fcu_bridge
{
class Bridge : public rclcpp::Node
{
public:
    /** @brief 创建消息桥节点，连接工作由 Start 完成。 */
    Bridge() : Node("fcu_bridge")
    {
        // 参数在启动时读取，运行期间不切换消息类型。
    }

    /** @brief 读取配置并连接话题。
     * @return 配置和类型组合有效时为 true。 */
    bool Start()
    {
        // 纯手控配置只创建原有输入。
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
        manual_ignore_ = IgnoreFields(Parameter<std::vector<std::string>>("active_fields", {}));
        const auto auto_topic = Parameter<std::string>("autonomous_input_topic", "");
        if ((manual_ignore_ || !auto_topic.empty()) && output_type_ != kTarget)
            throw std::invalid_argument("控制字段和自主输入要求 PositionTarget 输出");
        publisher_ = create_generic_publisher(output_topic, output_type_, rclcpp::QoS(10).reliable());
        subscription_ = create_generic_subscription(input_topic, input_type_, rclcpp::QoS(10).best_effort(),
            [this](std::shared_ptr<rclcpp::SerializedMessage> msg) { if (!active_) OnMessage(msg); });
        if (!auto_topic.empty())
        {
            if (topics->resolve_topic_name(auto_topic) == publisher_->get_topic_name() ||
                topics->resolve_topic_name(auto_topic) == subscription_->get_topic_name())
                throw std::invalid_argument("手控、自主和输出话题必须不同");
            auto_type_ = Parameter<std::string>("autonomous_input_type", kTarget);
            if (auto_type_ != kTarget && !CanConvert(auto_type_, kTarget))
                throw std::invalid_argument("不支持的自主输入类型: " + auto_type_);
            auto_ignore_ = IgnoreFields(Parameter<std::vector<std::string>>("autonomous_active_fields", {}));
            auto_sub_ = create_generic_subscription(auto_topic, auto_type_, rclcpp::QoS(1).best_effort(),
                std::bind(&Bridge::OnAutonomous, this, std::placeholders::_1));
            StartControl();
        }
        RCLCPP_INFO(get_logger(), "%s -> %s", subscription_->get_topic_name(), publisher_->get_topic_name());
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

    using Request = std_msgs::msg::UInt64;
    using Trigger = std_srvs::srv::Trigger;
    using Clock = std::chrono::steady_clock;
    static constexpr const char* kTarget = "mavros_msgs/msg/PositionTarget";

    bool AirborneOffboard() const
    {
        return offboard_ && airborne_ &&
            Clock::now() - last_state_ < std::chrono::seconds(3) &&
            Clock::now() - last_odom_ < std::chrono::seconds(3);
    }

    void PublishActive(bool value)
    {
        active_ = value;
        std_msgs::msg::Bool msg;
        msg.data = value;
        active_pub_->publish(msg);
    }

    void SetRequest(uint64_t id)
    {
        request_.data = id;
        ready_ = false;
        request_pub_->publish(request_);
        if (!id) PublishActive(false);
    }

    void StartControl()
    {
        const auto retained = rclcpp::QoS(1).transient_local();
        request_pub_ = create_publisher<Request>("control/autonomous_requested", retained);
        active_pub_ = create_publisher<std_msgs::msg::Bool>("control/autonomous_active", retained);
        SetRequest(0);
        ready_sub_ = create_subscription<Request>("control/autonomous_ready", retained,
            [this](Request::ConstSharedPtr msg) {
                if (request_.data && msg->data == request_.data && !ready_) {
                    ready_ = true;
                    ready_stamp_ = now().nanoseconds();
                }
            });
        state_sub_ = create_subscription<mavros_msgs::msg::State>("mavros/state", rclcpp::SensorDataQoS(),
            [this](mavros_msgs::msg::State::ConstSharedPtr msg) {
                last_state_ = Clock::now();
                disarmed_ = msg->connected && !msg->armed;
                offboard_ = msg->connected && msg->armed && msg->mode == "OFFBOARD";
                if (!offboard_ && request_.data) SetRequest(0);
            });
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("mavros/local_position/odom", rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                const double z = msg->pose.pose.position.z;
                if (!std::isfinite(z)) return;
                last_odom_ = Clock::now();
                if (disarmed_) ground_ = z;
                airborne_ = ground_ && z - *ground_ >= (airborne_ ? 0.2 : 0.5);
            });
        toggle_ = create_service<Trigger>("control/toggle",
            [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
                if (request_.data) {
                    SetRequest(0);
                    res->message = "已切换到手控";
                } else if (count_publishers(publisher_->get_topic_name()) > 1) {
                    res->success = false;
                    res->message = "飞控设定点话题存在其他发布者";
                    return;
                } else if (!AirborneOffboard()) {
                    res->success = false;
                    res->message = "请先解锁、进入 OFFBOARD 并起飞";
                    return;
                } else {
                    SetRequest(static_cast<uint64_t>(Clock::now().time_since_epoch().count()));
                    res->message = "已请求自主，等待新轨迹，当前仍可手控";
                }
                res->success = true;
            });
        watchdog_ = create_wall_timer(std::chrono::milliseconds(50), [this] {
            if (request_.data && (!AirborneOffboard() ||
                (active_ && Clock::now() - last_command_ > std::chrono::milliseconds(500))))
                SetRequest(0);
        });
    }

    void OnAutonomous(std::shared_ptr<rclcpp::SerializedMessage> msg)
    {
        if (!ready_ || !AirborneOffboard()) return;
        if (active_ && Clock::now() - last_command_ > std::chrono::milliseconds(500)) {
            SetRequest(0);
            return;
        }
        std_msgs::msg::Header fallback;
        fallback.stamp = now();
        fallback.frame_id = frame_id_;
        mavros_msgs::msg::PositionTarget target;
        if (!MakeTarget(auto_type_, *msg, fallback, auto_ignore_, target)) return;
        const auto stamp = rclcpp::Time(target.header.stamp).nanoseconds();
        const auto age = now().nanoseconds() - stamp;
        if (stamp <= ready_stamp_ || age < 0 || age > 500000000) return;
        PublishTarget(target);
        last_command_ = Clock::now();
        if (!active_) PublishActive(true);
    }

    void PublishTarget(const mavros_msgs::msg::PositionTarget& target)
    {
        rclcpp::SerializedMessage output;
        rclcpp::Serialization<mavros_msgs::msg::PositionTarget>().serialize_message(&target, &output);
        publisher_->publish(output);
    }

    /** @brief 处理一条输入。
     * @param input ROS 序列化消息。 */
    void OnMessage(std::shared_ptr<rclcpp::SerializedMessage> input)
    {
        // 同类型直接转发，跨类型只发布具备所需字段的转换结果。
        if (relay_ && !manual_ignore_)
        {
            publisher_->publish(*input);
            return;
        }
        std_msgs::msg::Header fallback;
        fallback.stamp = now();
        fallback.frame_id = frame_id_;
        if (manual_ignore_)
        {
            mavros_msgs::msg::PositionTarget target;
            if (MakeTarget(input_type_, *input, fallback, manual_ignore_, target)) PublishTarget(target);
            return;
        }
        rclcpp::SerializedMessage output;
        if (Convert(input_type_, output_type_, *input, fallback, output))
        {
            publisher_->publish(output);
        }
        else
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "跳过无效的输入消息：%s -> %s", input_type_.c_str(), output_type_.c_str());
        }
    }

    uint16_t manual_ignore_ = 0, auto_ignore_ = 0;
    std::string auto_type_;
    Request request_;
    bool ready_ = false, active_ = false, disarmed_ = false, offboard_ = false, airborne_ = false;
    int64_t ready_stamp_ = 0;
    std::optional<double> ground_;
    Clock::time_point last_state_, last_odom_, last_command_;
    rclcpp::GenericSubscription::SharedPtr auto_sub_;
    rclcpp::Publisher<Request>::SharedPtr request_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
    rclcpp::Subscription<Request>::SharedPtr ready_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Service<Trigger>::SharedPtr toggle_;
    rclcpp::TimerBase::SharedPtr watchdog_;
    std::string input_type_;
    std::string output_type_;
    std::string frame_id_;
    bool relay_ = false;
    rclcpp::GenericPublisher::SharedPtr publisher_;
    rclcpp::GenericSubscription::SharedPtr subscription_;
};
}  // namespace fcu_bridge

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
        auto bridge = std::make_shared<fcu_bridge::Bridge>();
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
        RCLCPP_ERROR(rclcpp::get_logger("fcu_bridge"), "%s", error.what());
        result = 1;
    }
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
    return result;
}

#include "fcu_bridge/conversion.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>

#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/serialization.hpp>
#include <rosidl_runtime_cpp/traits.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>

namespace fcu_bridge {
namespace {
namespace gm          = geometry_msgs::msg;
using PositionCommand = quadrotor_msgs::msg::PositionCommand;
using PositionTarget  = mavros_msgs::msg::PositionTarget;
using TrajectoryPoint = trajectory_msgs::msg::MultiDOFJointTrajectoryPoint;

// optional 区分“没有提供字段”和“明确发送零值”，防止生成不存在的控制指令
struct Motion {
    std::optional<std_msgs::msg::Header> header;
    std::optional<gm::Point> position;
    std::optional<gm::Quaternion> orientation;
    std::optional<gm::Twist> velocity;
    std::optional<gm::Accel> acceleration;
};

/**
 * @brief 提取位置
 * @param input 输入消息
 * @return 已提供的运动字段
 */
Motion read(const gm::Point &input) {
    // 点消息只描述位置，不附加默认姿态
    Motion motion;
    motion.position = input;
    return motion;
}

/**
 * @brief 提取带时间戳的位置
 * @param input 输入消息
 * @return 运动字段及 Header
 */
Motion read(const gm::PointStamped &input) {
    // 保留原始时间戳和参考系
    Motion motion = read(input.point);
    motion.header = input.header;
    return motion;
}

/**
 * @brief 提取位姿
 * @param input 输入消息
 * @return 位置及完整四元数
 */
Motion read(const gm::Pose &input) {
    // 普通位姿转换保留完整姿态，只有输出设定点时才提取 yaw
    Motion motion      = read(input.position);
    motion.orientation = input.orientation;
    return motion;
}

/**
 * @brief 提取带时间戳的位姿
 * @param input 输入消息
 * @return 运动字段及 Header
 */
Motion read(const gm::PoseStamped &input) {
    // 将位姿和所属坐标系一同传递
    Motion motion = read(input.pose);
    motion.header = input.header;
    return motion;
}

/**
 * @brief 提取协方差消息中的位姿
 * @param input 输入消息
 * @return 不含协方差的位姿
 */
Motion read(const gm::PoseWithCovariance &input) {
    // 目标控制消息不使用协方差
    return read(input.pose);
}

/**
 * @brief 提取带 Header 的协方差位姿
 * @param input 输入消息
 * @return 位姿及 Header
 */
Motion read(const gm::PoseWithCovarianceStamped &input) {
    // 保留 Header，提取内部位姿
    Motion motion = read(input.pose);
    motion.header = input.header;
    return motion;
}

/**
 * @brief 提取六维速度
 * @param input 输入消息
 * @return 线速度和角速度
 */
Motion read(const gm::Twist &input) {
    // 在中间数据中保留三个角速度分量
    Motion motion;
    motion.velocity = input;
    return motion;
}

/**
 * @brief 提取带时间戳的速度
 * @param input 输入消息
 * @return 速度及 Header
 */
Motion read(const gm::TwistStamped &input) {
    // 只传递坐标系名称，不执行坐标变换
    Motion motion = read(input.twist);
    motion.header = input.header;
    return motion;
}

/**
 * @brief 提取协方差消息中的速度
 * @param input 输入消息
 * @return 不含协方差的速度
 */
Motion read(const gm::TwistWithCovariance &input) {
    // 控制输出只需要速度值
    return read(input.twist);
}

/**
 * @brief 提取带 Header 的协方差速度
 * @param input 输入消息
 * @return 速度及 Header
 */
Motion read(const gm::TwistWithCovarianceStamped &input) {
    // 保留 Header，提取内部速度
    Motion motion = read(input.twist);
    motion.header = input.header;
    return motion;
}

/**
 * @brief 提取六维加速度
 * @param input 输入消息
 * @return 线加速度和角加速度
 */
Motion read(const gm::Accel &input) {
    // 加速度按 m/s² 和 rad/s² 传递，不解释为力
    Motion motion;
    motion.acceleration = input;
    return motion;
}

/**
 * @brief 提取带时间戳的加速度
 * @param input 输入消息
 * @return 加速度及 Header
 */
Motion read(const gm::AccelStamped &input) {
    // 保留加速度对应的时间和参考系
    Motion motion = read(input.accel);
    motion.header = input.header;
    return motion;
}

/**
 * @brief 提取里程计位姿
 * @param input 输入消息
 * @return header 坐标系下的位姿
 */
Motion read(const nav_msgs::msg::Odometry &input) {
    // twist 属于 child_frame_id，不能直接与 header.frame_id 下的位姿合并
    Motion motion = read(input.pose);
    motion.header = input.header;
    return motion;
}

/**
 * @brief 提取单关节轨迹点
 * @param input 当前设定点
 * @return 有效字段；多关节时为空
 */
Motion read(const TrajectoryPoint &input) {
    // 此节点不执行整段轨迹，也不选择多个关节中的某一个
    Motion motion;
    if (input.transforms.size() > 1 || input.velocities.size() > 1 || input.accelerations.size() > 1) {
        return motion;
    }
    if (!input.transforms.empty()) {
        const auto &transform = input.transforms.front();
        gm::Point point;
        point.x            = transform.translation.x;
        point.y            = transform.translation.y;
        point.z            = transform.translation.z;
        motion.position    = point;
        motion.orientation = transform.rotation;
    }
    if (!input.velocities.empty()) {
        motion.velocity = input.velocities.front();
    }
    if (!input.accelerations.empty()) {
        // ROS 的轨迹点用 Twist 结构存放加速度，需按字段语义转换为 Accel
        gm::Accel acceleration;
        acceleration.linear  = input.accelerations.front().linear;
        acceleration.angular = input.accelerations.front().angular;
        motion.acceleration  = acceleration;
    }
    return motion;
}

/**
 * @brief 写入位置
 * @param motion 运动字段
 * @param output 输出消息
 * @return 是否具有位置
 */
bool write(const Motion &motion, gm::Point &output) {
    // 未提供的位置不能用零位置代替
    if (!motion.position) {
        return false;
    }
    output = *motion.position;
    return true;
}

/**
 * @brief 写入带 Header 的位置
 * @param motion 运动字段
 * @param output 输出
 * @return 是否成功
 */
bool write(const Motion &motion, gm::PointStamped &output) {
    // Header 由转换入口统一选择
    output.header = *motion.header;
    return write(motion, output.point);
}

/**
 * @brief 写入位姿
 * @param motion 运动字段
 * @param output 输出
 * @return 是否同时具有位置和姿态
 */
bool write(const Motion &motion, gm::Pose &output) {
    // 不为只有位置的输入虚构朝向
    if (!motion.orientation || !write(motion, output.position)) {
        return false;
    }
    output.orientation = *motion.orientation;
    return true;
}

/**
 * @brief 写入带 Header 的位姿
 * @param motion 运动字段
 * @param output 输出
 * @return 是否成功
 */
bool write(const Motion &motion, gm::PoseStamped &output) {
    // 保持位姿时间戳和参考系一致
    output.header = *motion.header;
    return write(motion, output.pose);
}

/**
 * @brief 写入六维速度
 * @param motion 运动字段
 * @param output 输出
 * @return 是否具有速度
 */
bool write(const Motion &motion, gm::Twist &output) {
    // 完整保留线速度、角速度
    if (!motion.velocity) {
        return false;
    }
    output = *motion.velocity;
    return true;
}

/**
 * @brief 写入带 Header 的速度
 * @param motion 运动字段
 * @param output 输出
 * @return 是否成功
 */
bool write(const Motion &motion, gm::TwistStamped &output) {
    // 只增加消息封装，不修改速度含义
    output.header = *motion.header;
    return write(motion, output.twist);
}

/**
 * @brief 写入六维加速度
 * @param motion 运动字段
 * @param output 输出
 * @return 是否具有加速度
 */
bool write(const Motion &motion, gm::Accel &output) {
    // 未提供加速度时不生成零加速度指令
    if (!motion.acceleration) {
        return false;
    }
    output = *motion.acceleration;
    return true;
}

/**
 * @brief 写入带 Header 的加速度
 * @param motion 运动字段
 * @param output 输出
 * @return 是否成功
 */
bool write(const Motion &motion, gm::AccelStamped &output) {
    // 保留或补充转换入口选择的 Header
    output.header = *motion.header;
    return write(motion, output.accel);
}

/**
 * @brief 生成 MAVROS 设定点
 * @param motion 运动字段
 * @param output 输出
 * @return 是否有控制字段
 */
bool write(const Motion &motion, PositionTarget &output) {
    // 从全部忽略开始，仅启用输入实际提供的字段；ROS 端仍使用 ENU
    if (!motion.position && !motion.velocity && !motion.acceleration) {
        return false;
    }
    constexpr uint16_t kIgnorePosition =
            PositionTarget::IGNORE_PX | PositionTarget::IGNORE_PY | PositionTarget::IGNORE_PZ;
    constexpr uint16_t kIgnoreVelocity =
            PositionTarget::IGNORE_VX | PositionTarget::IGNORE_VY | PositionTarget::IGNORE_VZ;
    constexpr uint16_t kIgnoreAcceleration =
            PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ;
    output.header           = *motion.header;
    output.coordinate_frame = PositionTarget::FRAME_LOCAL_NED;
    output.type_mask        = kIgnorePosition | kIgnoreVelocity | kIgnoreAcceleration | PositionTarget::IGNORE_YAW |
                              PositionTarget::IGNORE_YAW_RATE;
    if (motion.position) {
        output.position = *motion.position;
        output.type_mask &= ~kIgnorePosition;
    }
    if (motion.orientation) {
        const auto &q = *motion.orientation;
        output.yaw    = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        output.type_mask &= ~PositionTarget::IGNORE_YAW;
    }
    if (motion.velocity) {
        output.velocity = motion.velocity->linear;
        output.yaw_rate = motion.velocity->angular.z;
        output.type_mask &= ~(kIgnoreVelocity | PositionTarget::IGNORE_YAW_RATE);
    }
    if (motion.acceleration) {
        output.acceleration_or_force = motion.acceleration->linear;
        output.type_mask &= ~kIgnoreAcceleration;
    }
    return true;
}

/**
 * @brief 解码并提取字段
 * @param input 序列化消息
 * @return 已提供的运动字段
 */
template<typename Message>
Motion decode(const rclcpp::SerializedMessage &input) {
    // 使用 ROS 自带序列化接口，消息定义由已安装的 ROS 包提供
    Message message;
    rclcpp::Serialization<Message>().deserialize_message(&input, &message);
    return read(message);
}

/**
 * @brief 编码输出
 * @param motion 运动字段
 * @param output 序列化输出
 * @return 是否成功
 */
template<typename Message>
bool encode(const Motion &motion, rclcpp::SerializedMessage &output) {
    // 只有字段足够生成目标类型时才序列化
    Message message;
    if (!write(motion, message)) {
        return false;
    }
    rclcpp::Serialization<Message>().serialize_message(&message, &output);
    return true;
}

enum Field {
    kPosition     = 1,
    kOrientation  = 2,
    kVelocity     = 4,
    kAcceleration = 8,
    kPose         = kPosition | kOrientation,
    kAll          = kPose | kVelocity | kAcceleration
};

struct InputType {
    const char *name;
    int fields;
    Motion (*decode)(const rclcpp::SerializedMessage &);
};

struct OutputType {
    const char *name;
    int required_fields;
    bool (*encode)(const Motion &, rclcpp::SerializedMessage &);
};

/**
 * @brief 声明一种输入类型
 * @param fields 可提取字段
 * @return 类型信息及解码函数
 */
template<typename Message>
InputType input(int fields) {
    // 类型名称直接取自 ROS 消息，避免手写名称与模板类型不一致
    return {rosidl_generator_traits::name<Message>(), fields, decode<Message>};
}

/**
 * @brief 声明一种输出类型
 * @param fields 必需字段
 * @return 类型信息及编码函数
 */
template<typename Message>
OutputType output(int fields) {
    // 必需字段用于启动时判断类型组合是否有意义
    return {rosidl_generator_traits::name<Message>(), fields, encode<Message>};
}

const InputType kInputs[] = {
        input<gm::Point>(kPosition),
        input<gm::PointStamped>(kPosition),
        input<gm::Pose>(kPose),
        input<gm::PoseStamped>(kPose),
        input<gm::PoseWithCovariance>(kPose),
        input<gm::PoseWithCovarianceStamped>(kPose),
        input<gm::Twist>(kVelocity),
        input<gm::TwistStamped>(kVelocity),
        input<gm::TwistWithCovariance>(kVelocity),
        input<gm::TwistWithCovarianceStamped>(kVelocity),
        input<gm::Accel>(kAcceleration),
        input<gm::AccelStamped>(kAcceleration),
        input<nav_msgs::msg::Odometry>(kPose),
        input<TrajectoryPoint>(kAll),
};

const OutputType kOutputs[] = {
        output<gm::Point>(kPosition),     output<gm::PointStamped>(kPosition),
        output<gm::Pose>(kPose),          output<gm::PoseStamped>(kPose),
        output<gm::Twist>(kVelocity),     output<gm::TwistStamped>(kVelocity),
        output<gm::Accel>(kAcceleration), output<gm::AccelStamped>(kAcceleration),
        output<PositionTarget>(0),
};

/**
 * @brief 查找消息类型
 * @param types 支持表
 * @param name 类型全名
 * @return 条目或 nullptr
 */
template<typename Type, size_t Size>
const Type *find(const Type (&types)[Size], const std::string &name) {
    // 支持表很小，顺序查找即可
    for (const auto &type: types) {
        if (name == type.name) {
            return &type;
        }
    }
    return nullptr;
}
} // namespace

// These eleven IGNORE bits are independent of the MAVLink FORCE bit.
constexpr uint16_t kControlFields = 0xDFF;
uint16_t ignoreFields(const std::vector<std::string> &fields) {
    if (fields.empty()) {
        return 0;
    }
    static const std::pair<const char *, uint16_t> bits[] = {{"px", PositionTarget::IGNORE_PX},
                                                             {"py", PositionTarget::IGNORE_PY},
                                                             {"pz", PositionTarget::IGNORE_PZ},
                                                             {"vx", PositionTarget::IGNORE_VX},
                                                             {"vy", PositionTarget::IGNORE_VY},
                                                             {"vz", PositionTarget::IGNORE_VZ},
                                                             {"ax", PositionTarget::IGNORE_AFX},
                                                             {"ay", PositionTarget::IGNORE_AFY},
                                                             {"az", PositionTarget::IGNORE_AFZ},
                                                             {"yaw", PositionTarget::IGNORE_YAW},
                                                             {"yaw_rate", PositionTarget::IGNORE_YAW_RATE}};
    uint16_t ignore                                       = kControlFields;
    for (const auto &field: fields) {
        auto bit = std::find_if(std::begin(bits), std::end(bits), [&](const auto &entry) {
            return field == entry.first;
        });
        if (bit == std::end(bits)) {
            throw std::invalid_argument("未知控制字段: " + field);
        }
        ignore &= ~bit->second;
    }
    return ignore;
}

bool makeTarget(const std::string &input_type, const rclcpp::SerializedMessage &input,
                const std_msgs::msg::Header &fallback, uint16_t ignore, PositionTarget &target) {
    if (input_type == rosidl_generator_traits::name<PositionTarget>()) {
        rclcpp::Serialization<PositionTarget>().deserialize_message(&input, &target);
    } else if (input_type == rosidl_generator_traits::name<PositionCommand>()) {
        PositionCommand cmd;
        rclcpp::Serialization<PositionCommand>().deserialize_message(&input, &cmd);
        if (cmd.trajectory_flag != PositionCommand::TRAJECTORY_STATUS_READY) {
            return false;
        }
        target.header                = cmd.header;
        target.coordinate_frame      = PositionTarget::FRAME_LOCAL_NED;
        target.type_mask             = 0;
        target.position              = cmd.position;
        target.velocity              = cmd.velocity;
        target.acceleration_or_force = cmd.acceleration;
        target.yaw                   = cmd.yaw;
        target.yaw_rate              = cmd.yaw_dot;
    } else {
        const auto *type = find(kInputs, input_type);
        if (!type) {
            return false;
        }
        Motion motion = type->decode(input);
        if (!motion.header) {
            motion.header = fallback;
        }
        if (!write(motion, target)) {
            return false;
        }
    }
    target.type_mask |= ignore;
    if ((target.type_mask & kControlFields) == kControlFields) {
        return false;
    }
    const auto valid = [&](uint16_t bit, double value) {
        return (target.type_mask & bit) || std::isfinite(value);
    };
    const auto valid_xyz = [&](uint16_t first, const auto &v) {
        return valid(first, v.x) && valid(first << 1, v.y) && valid(first << 2, v.z);
    };
    return valid_xyz(PositionTarget::IGNORE_PX, target.position) &&
           valid_xyz(PositionTarget::IGNORE_VX, target.velocity) &&
           valid_xyz(PositionTarget::IGNORE_AFX, target.acceleration_or_force) &&
           valid(PositionTarget::IGNORE_YAW, target.yaw) && valid(PositionTarget::IGNORE_YAW_RATE, target.yaw_rate);
}

bool canConvert(const std::string &input_type, const std::string &output_type) {
    if (input_type == rosidl_generator_traits::name<PositionCommand>()) {
        return output_type == rosidl_generator_traits::name<PositionTarget>();
    }
    // 排除类型本身无法提供目标字段的组合，轨迹点还需逐条检查数组内容
    const auto *input  = find(kInputs, input_type);
    const auto *output = find(kOutputs, output_type);
    return input && output && (input->fields & output->required_fields) == output->required_fields;
}

bool convert(const std::string &input_type, const std::string &output_type, const rclcpp::SerializedMessage &input,
             const std_msgs::msg::Header &fallback_header, rclcpp::SerializedMessage &output) {
    // 先提取语义相同的字段，再按目标消息封装；无 Header 的输入才补时间和参考系
    if (!canConvert(input_type, output_type)) {
        return false;
    }
    if (output_type == rosidl_generator_traits::name<PositionTarget>()) {
        PositionTarget target;
        if (!makeTarget(input_type, input, fallback_header, 0, target)) {
            return false;
        }
        rclcpp::Serialization<PositionTarget>().serialize_message(&target, &output);
        return true;
    }
    Motion motion = find(kInputs, input_type)->decode(input);
    if (!motion.header) {
        motion.header = fallback_header;
    }
    return find(kOutputs, output_type)->encode(motion, output);
}
} // namespace fcu_bridge

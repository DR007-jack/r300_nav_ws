#include <memory>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

// 引入ugv_sdk核心头文件（包含ProtocolVersion定义的头文件）
#include "ugv_sdk/details/interface/parser_interface.hpp"  // 直接引入ProtocolVersion所在头文件
#include "ugv_sdk/mobile_robot/scout_robot.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"
#include "scout_base/scout_messenger.hpp"

using namespace westonrobot;

// 全局机器人对象指针
std::unique_ptr<ScoutRobot> robot;

int main(int argc, char **argv) {
  // 初始化ROS节点
  ros::init(argc, argv, "scout_odom");
  ros::NodeHandle node(""), private_node("~");

  // 读取机器人类型参数（scout mini/omni）
  bool is_scout_mini = false;
  static bool is_scout_omni = false;
  private_node.getParam("is_scout_mini", is_scout_mini);
  ROS_INFO("Working as scout mini: %d", is_scout_mini);
  private_node.getParam("is_scout_omni", is_scout_omni);
  ROS_INFO("Working as scout omni: %d", is_scout_omni);

  // 协议检测（使用正确拼写的ProtocolDectctor）
  westonrobot::ProtocolDectctor detector;
  try {
    // 连接CAN0端口
    detector.Connect("can0");
    // 检测协议版本（直接使用返回值，无需显式声明ProtocolVersion类型）
    auto proto = detector.DetectProtocolVersion(5);
    std::cout << "Detected protocol version: " << static_cast<int>(proto) << std::endl;

    // 根据机器人类型创建机器人对象（直接使用检测到的proto构造，避免显式引用ProtocolVersion::AGX_Vx）
    if (is_scout_mini && is_scout_omni) {
      // Scout Mini Omni机器人
      std::cout << "Creating Scout Mini Omni Robot object" << std::endl;
      robot = std::unique_ptr<ScoutMiniOmniRobot>(new ScoutMiniOmniRobot(proto));
    } else {
      // 普通Scout机器人
      std::cout << "Creating Scout Robot object (mini: " << is_scout_mini << ")" << std::endl;
      robot = std::unique_ptr<ScoutRobot>(new ScoutRobot(proto, is_scout_mini));
    }

    // 检查机器人对象是否创建成功
    if (robot == nullptr) {
      ROS_ERROR("Failed to create robot object");
      ros::shutdown();
      return -1;
    }
  } catch (const std::exception &error) {
    ROS_ERROR("Please bring up CAN bus or make sure CAN port exists. Error: %s", error.what());
    ros::shutdown();
    return -1;
  }

  // 创建ROS消息发布器对象
  ScoutROSMessenger messenger(robot.get(), &node, is_scout_omni);

  // 读取配置参数
  std::string port_name;
  private_node.param<std::string>("port_name", port_name, std::string("can0"));
  private_node.param<std::string>("odom_frame", messenger.odom_frame_, std::string("odom"));
  private_node.param<std::string>("base_frame", messenger.base_frame_, std::string("base_link"));
  private_node.param<bool>("simulated_robot", messenger.simulated_robot_, false);
  private_node.param<int>("control_rate", messenger.sim_control_rate_, 50);
  private_node.param<std::string>("odom_topic_name", messenger.odom_topic_name_, std::string("odom"));
  private_node.param<bool>("pub_tf", messenger.pub_tf, true);

  // 非仿真模式下连接机器人
  if (!messenger.simulated_robot_) {
    if (port_name.find("can") != std::string::npos) {
      robot->Connect(port_name);
      robot->EnableCommandedMode();
      ROS_INFO("Using CAN bus to talk with the robot");
    } else {
      ROS_INFO("Only CAN bus interface is supported for now");
    }
  }

  // 设置ROS订阅器
  messenger.SetupSubscription();

  // 主循环：50Hz发布机器人状态
  ros::Rate rate(50);
  while (ros::ok()) {
    if (!messenger.simulated_robot_) {
      messenger.PublishStateToROS();
    } else {
      double linear, angular;
      messenger.GetCurrentMotionCmdForSim(linear, angular);
      messenger.PublishSimStateToROS(linear, angular);
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

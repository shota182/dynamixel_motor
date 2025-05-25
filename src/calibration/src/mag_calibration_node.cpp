#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

class MagCalibrationNode
{
public:
  MagCalibrationNode(ros::NodeHandle& nh)
    : nh_(nh)
  {
    // パラメータ取得
    nh_.param("sensor_count", sensor_count_, 0);
    nh_.param("spring_constants", spring_constant_, 1.0);

    nh_.getParam("motor_side_wire_angles", motor_angles_);
    nh_.getParam("manipulator_side_wire_angles", manipulator_angles_);
    nh_.getParam("magnet_strength", magnet_strength_);
    nh_.getParam("magnet_distances", magnet_distances_);

    if (magnet_strength_.size() != magnet_distances_.size()) {
      ROS_ERROR("magnet_strength and magnet_distances must be same size");
      ros::shutdown();
    }

    if (motor_angles_.size() != (size_t)sensor_count_) {
      ROS_WARN("motor_side_wire_angles size does not match sensor_count");
    }

    // 磁気センサ値購読
    sub_mag_ = nh_.subscribe("/sensor/mag", 1, &MagCalibrationNode::tensionCallback, this);

    ROS_INFO("MagCalibrationNode initialized.");
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_mag_;

  int sensor_count_;
  double spring_constant_;
  std::vector<double> motor_angles_;
  std::vector<double> manipulator_angles_;
  std::vector<double> magnet_strength_;   // 磁束密度 [mT]
  std::vector<double> magnet_distances_;  // 距離 [mm]

  // 線形補間で磁束密度→距離を推定
  double estimateDistance(double B_measured) {
    for (size_t i = 1; i < magnet_strength_.size(); ++i) {
      if (B_measured > magnet_strength_[i]) {
        double B1 = magnet_strength_[i - 1];
        double B2 = magnet_strength_[i];
        double d1 = magnet_distances_[i - 1];
        double d2 = magnet_distances_[i];

        double ratio = (B_measured - B2) / (B1 - B2);
        return d2 + (d1 - d2) * ratio;  // 線形補間
      }
    }
    return magnet_distances_.back(); // 範囲外は最大距離
  }

  void tensionCallback(const sensor_msgs::MagneticField::ConstPtr& msg)
  {
    // 仮に msg->magnetic_field.x 〜 x方向成分を使うと仮定（センサ数に対応づけるには変更必要）
    double B = msg->magnetic_field.x * 1e3; // [T] → [mT]

    double distance = estimateDistance(B); // [mm]
    double deflection = distance / 1000.0; // [m]

    double tension = spring_constant_ * deflection;

    ROS_INFO("B = %.2f mT, distance = %.2f mm, tension = %.4f N", B, distance, tension);

    // ばね方向成分の計算（例：wire角度に対してcos成分）
    for (size_t i = 0; i < motor_angles_.size(); ++i) {
      double angle = motor_angles_[i];
      double axial_tension = tension * std::cos(angle);
      ROS_INFO("Wire %lu: Axial Tension = %.4f N (angle = %.2f rad)", i, axial_tension, angle);
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mag_calibration_node");
  ros::NodeHandle nh("~");
  MagCalibrationNode node(nh);
  ros::spin();
  return 0;
}

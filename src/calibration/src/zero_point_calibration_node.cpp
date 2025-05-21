#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/MagneticField.h>
#include <deque>

class ZeroPointCalibrationNode {
public:
  ZeroPointCalibrationNode(ros::NodeHandle& nh) : nh_(nh), calibrated_(false), initialized_(false) {
    // YAMLからパラメータ読み込み（デフォルト値10）
    nh_.param("calibration_samples", mag_window_size_, 3);

    sub_pos_ = nh_.subscribe("/sensor/motor/output/position", 1, &ZeroPointCalibrationNode::positionCallback, this);
    sub_mag_ = nh_.subscribe("/sensor/mag", 1, &ZeroPointCalibrationNode::magCallback, this);
    pub_cmd_ = nh_.advertise<std_msgs::Int32MultiArray>("/sensor/motor/input/position", 10);

    ROS_INFO("Zero Point Calibration Node initialized. calibration_samples = %d", mag_window_size_);
  }

  void spin() {
    ros::spin();
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_pos_;
  ros::Subscriber sub_mag_;
  ros::Publisher pub_cmd_;

  std::deque<sensor_msgs::MagneticField> mag_buffer_;
  int mag_window_size_;
  bool calibrated_;
  bool initialized_;

  std_msgs::Int32MultiArray initial_position_;

  void positionCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    // 初回のみ位置を保存
    if (!initialized_) {
      initial_position_ = *msg;
      initialized_ = true;
      ROS_INFO("Initial motor position received and stored.");
    }
  }

  void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg) {
    if (!initialized_) return; // 位置初期化が完了してから処理

    mag_buffer_.push_back(*msg);
    if (mag_buffer_.size() > mag_window_size_) {
      mag_buffer_.pop_front();
    }

    if (!calibrated_ && mag_buffer_.size() == mag_window_size_) {
      sensor_msgs::MagneticField avg_mag = computeAverageMag();
      ROS_INFO("Calibration complete: avg B = [%f, %f, %f]",
               avg_mag.magnetic_field.x, avg_mag.magnetic_field.y, avg_mag.magnetic_field.z);

      // 例：initial_position_をそのままゼロ基準として使用
      std_msgs::Int32MultiArray cmd;
      cmd.data = std::vector<int>(initial_position_.data.size(), 0); // モータ指令ゼロに設定
      pub_cmd_.publish(cmd);

      calibrated_ = true;
    }
  }

  sensor_msgs::MagneticField computeAverageMag() {
    sensor_msgs::MagneticField avg;
    for (const auto& mag : mag_buffer_) {
      avg.magnetic_field.x += mag.magnetic_field.x;
      avg.magnetic_field.y += mag.magnetic_field.y;
      avg.magnetic_field.z += mag.magnetic_field.z;
    }
    int n = mag_buffer_.size();
    avg.magnetic_field.x /= n;
    avg.magnetic_field.y /= n;
    avg.magnetic_field.z /= n;
    return avg;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "zero_point_calibration_node");
  ros::NodeHandle nh("~");  // プライベート名前空間から読み込む
  ZeroPointCalibrationNode node(nh);
  node.spin();
  return 0;
}

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <deque>

class ForceControlNode {
public:
  ForceControlNode(ros::NodeHandle& nh)
    : nh_(nh)
  {
    nh_.param("tension_threshold", tension_threshold_, 10);
    nh_.param("pull_value_gain", pull_value_gain_, 10);
    nh_.param("Kp", Kp_, 10.0);
    nh_.param("index_sensor_1", index_sensor_1_, 0);
    nh_.param("index_sensor_2", index_sensor_2_, 1);
    nh_.param("tension_margin", tension_margin_, 30);
    nh_.param("avg_count", avg_count_, 5);

    initialized_position_ = false;
    initialized_tension_ = false;

    sub_pos_ = nh_.subscribe("/sensor/motor/output/position", 1, &ForceControlNode::positionCallback, this);
    sub_mag_ = nh_.subscribe("/sensor/mag", 1, &ForceControlNode::tensionCallback, this);
    pub_cmd_ = nh_.advertise<std_msgs::Int32MultiArray>("/sensor/motor/input/position", 10);
    timer_ = nh_.createTimer(ros::Duration(0.02), &ForceControlNode::controlLoop, this);
  }

private:
  void positionCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    latest_position_ = *msg;
    if (!initialized_position_) {
      initial_position_ = *msg;
      initialized_position_ = true;
    }
  }

  void tensionCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    latest_tension_ = *msg;
    if (!initialized_tension_) {
      tension_buffer_.push_back(*msg);
      if (tension_buffer_.size() >= avg_count_) {
        initial_tension_.data.resize(msg->data.size());
        for (size_t i = 0; i < msg->data.size(); ++i) {
          int sum = 0;
          for (const auto& m : tension_buffer_) sum += m.data[i];
          initial_tension_.data[i] = sum / avg_count_ + tension_threshold_;
        }
        T_max_1_ = tension_margin_ + initial_tension_.data[index_sensor_1_];
        T_max_2_ = tension_margin_ + initial_tension_.data[index_sensor_2_];
        initialized_tension_ = true;
        ROS_INFO_STREAM("Initial tension calibrated over " << avg_count_ << " samples.");
      }
    }
  }

  void controlLoop(const ros::TimerEvent&) {
    if (!(initialized_position_ && initialized_tension_)) return;

    std_msgs::Int32MultiArray cmd;
    cmd.data.resize(2);

    // θ（変位量）
    int theta_1 = latest_position_.data[0] - initial_position_.data[0];
    int theta_2 = latest_position_.data[1] - initial_position_.data[1];

    // T（張力変化量）
    int T_1 = latest_tension_.data[index_sensor_1_] - initial_tension_.data[index_sensor_1_];
    int T_2 = latest_tension_.data[index_sensor_2_] - initial_tension_.data[index_sensor_2_];

    // --- モータ1制御 ---
    if (latest_tension_.data[index_sensor_1_] < initial_tension_.data[index_sensor_1_]) {
      cmd.data[0] = latest_position_.data[0] + pull_value_gain_ * T_1;
      ROS_INFO_STREAM_THROTTLE(1.0, "[motor1] tension low → pull: " << cmd.data[0]);
    } else if (latest_tension_.data[index_sensor_1_] > T_max_1_) {
      cmd.data[0] = latest_position_.data[0] + pull_value_gain_ * (T_1 - tension_margin_);
      ROS_INFO_STREAM_THROTTLE(1.0, "[motor1] tension high → release: " << cmd.data[0]);
    } else {
      cmd.data[0] = latest_position_.data[0] - static_cast<int>(Kp_ * theta_1);
      ROS_INFO_STREAM_THROTTLE(1.0, "[motor1] holding: " << cmd.data[0]);
    }

    // --- モータ2制御 ---
    if (latest_tension_.data[index_sensor_2_] < initial_tension_.data[index_sensor_2_]) {
      cmd.data[1] = latest_position_.data[1] + pull_value_gain_ * T_2;
      ROS_INFO_STREAM_THROTTLE(1.0, "[motor2] tension low → pull: " << cmd.data[1]);
    } else if (latest_tension_.data[index_sensor_2_] > T_max_2_) {
      cmd.data[1] = latest_position_.data[1] + pull_value_gain_ * (T_2 - tension_margin_);
      ROS_INFO_STREAM_THROTTLE(1.0, "[motor2] tension high → release: " << cmd.data[1]);
    } else {
      cmd.data[1] = latest_position_.data[1] - static_cast<int>(Kp_ * theta_2);
      ROS_INFO_STREAM_THROTTLE(1.0, "[motor2] holding: " << cmd.data[1]);
    }

    pub_cmd_.publish(cmd);
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_pos_, sub_mag_;
  ros::Publisher pub_cmd_;
  ros::Timer timer_;

  std_msgs::Int32MultiArray initial_position_;
  std_msgs::Int32MultiArray initial_tension_;
  std_msgs::Int32MultiArray latest_position_;
  std_msgs::Int32MultiArray latest_tension_;

  std::deque<std_msgs::Int32MultiArray> tension_buffer_;

  bool initialized_position_, initialized_tension_;
  int tension_threshold_, pull_value_gain_;
  double Kp_;
  int index_sensor_1_, index_sensor_2_;
  int T_max_1_, T_max_2_, tension_margin_;
  int avg_count_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "force_control_node");
  ros::NodeHandle nh("~");
  ForceControlNode node(nh);
  ros::spin();
  return 0;
}

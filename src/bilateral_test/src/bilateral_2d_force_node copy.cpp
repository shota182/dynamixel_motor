// bilateral_test_node.cpp (4-motor version)
// positionがinitialベースで記載していたが，スレーブ側の張力保証を入れると干渉するためコピーを作成，
// latestベースをもとの方に記載するようにする
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <deque>
#include <iomanip>  // std::setprecision を使用するために必要

class BilateralTestNode {
public:
  BilateralTestNode(ros::NodeHandle& nh)
    : nh_(nh) {
    nh_.param("tension_threshold", tension_threshold_, 10);
    nh_.param("pull_value_gain", pull_value_gain_, 10);
    nh_.param("Kp", Kp_, 10.0);
    nh_.param("Kf", Kf_, 1.0);
    nh_.param("Kd", Kd_, 5.0);
    nh_.param("index_master_1", index_master_1_, 0);
    nh_.param("index_master_2", index_master_2_, 1);
    nh_.param("index_slave_1", index_slave_1_, 2);
    nh_.param("index_slave_2", index_slave_2_, 3);
    nh_.param<std::vector<int>>("motor_inverse", motor_inverse_, std::vector<int>{0, 0, 0, 0});
    nh_.param("tension_margin", tension_margin_, 1.0);
    nh_.param("avg_count", avg_count_, 5);

    initialized_position_ = false;
    initialized_tension_ = false;
    initialized_position_second_ = false;
    initialized_tension_second_ = false;
    sub_pos_ = nh_.subscribe("/sensor/motor/output/position", 1, &BilateralTestNode::positionCallback, this);
    sub_mag_ = nh_.subscribe("/force/input/tension", 1, &BilateralTestNode::tensionCallback, this);
    pub_cmd_ = nh_.advertise<std_msgs::Int32MultiArray>("/sensor/motor/input/position", 10);
    timer_ = nh_.createTimer(ros::Duration(0.02), &BilateralTestNode::controlLoop, this);
  }

private:
  void positionCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    latest_position_ = *msg;
    if (!initialized_position_) {
      initial_position_ = *msg;
      initialized_position_ = true;
    } else if (initialized_tension_second_ && !initialized_position_second_){
      initial_position_ = *msg;
      initialized_position_second_ = true;
    }
  }

  void tensionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    latest_tension_ = *msg;
    ROS_INFO_STREAM("Received tension data with size: " << msg->data.size());

    if (msg->data.empty()) {
        ROS_WARN("Received empty tension data. Skipping processing.");
        return;
    }

    if (!initialized_tension_) {
        tension_buffer_.push_back(*msg);
        ROS_INFO_STREAM("Tension buffer size: " << tension_buffer_.size());

        if (tension_buffer_.size() >= avg_count_) {
            initial_tension_.data.resize(msg->data.size());
            for (size_t i = 0; i < msg->data.size(); ++i) {
                int sum = 0;
                for (const auto& m : tension_buffer_) {
                    sum += m.data[i];
                }
                initial_tension_.data[i] = sum / avg_count_ + tension_threshold_;
            }
            T_max_1_ = tension_margin_ + initial_tension_.data[index_master_1_];
            T_max_2_ = tension_margin_ + initial_tension_.data[index_master_2_];
            initialized_tension_ = true;
            ROS_INFO_STREAM("Initial tension averaged over " << avg_count_ << " samples.");
        }
    }
  }

  void controlLoop(const ros::TimerEvent&) {
    if (!(initialized_position_ && initialized_tension_)) {
        ROS_WARN("Position or tension not initialized. Skipping control loop.");
        return;
    }

    std_msgs::Int32MultiArray cmd;
    cmd.data.resize(latest_position_.data.size());
    ROS_INFO_STREAM("Latest position size: " << latest_position_.data.size());
    ROS_INFO_STREAM("Latest tension size: " << latest_tension_.data.size());

    if (!initialized_tension_second_) {
      ROS_INFO_STREAM("Initializing tension.");

      // マスタ側の張力
      double tension_master_1 = latest_tension_.data[index_master_1_];
      double tension_master_2 = latest_tension_.data[index_master_2_];
      // スレーブ側の張力
      double tension_slave_1 = latest_tension_.data[index_slave_1_];
      double tension_slave_2 = latest_tension_.data[index_slave_2_];

      // 張力の初期化
      cmd.data[0] = static_cast<int>(motor_inverse_[0] * Kf_ * (tension_margin_ - tension_master_1) + latest_position_.data[0]);
      cmd.data[1] = static_cast<int>(motor_inverse_[1] * Kf_ * (tension_margin_ - tension_slave_1) + latest_position_.data[1]);
      cmd.data[2] = static_cast<int>(motor_inverse_[2] * Kf_ * (tension_margin_ - tension_master_2) + latest_position_.data[2]);
      cmd.data[3] = static_cast<int>(motor_inverse_[3] * Kf_ * (tension_margin_ - tension_slave_2) + latest_position_.data[3]);

      pub_cmd_.publish(cmd);

      // 精度を設定してログを出力
      ROS_INFO_STREAM(std::fixed << std::setprecision(6)
                      << "master1 tension: " << (tension_master_1)
                      << ", master2 tension: " << (tension_master_2)
                      << ", slave1 tension: " << (tension_slave_1)
                      << ", slave2 tension: " << (tension_slave_2));
      if () {
          initialized_tension_second_ = true;
          ROS_INFO("Initial tension set successfully, proceeding to control loop.");
      }
      return;
    }

    // マスタ側の位置
    int position_master_1 = motor_inverse_[0] * (latest_position_.data[0] - initial_position_.data[0]);
    int position_master_2 = motor_inverse_[2] * (latest_position_.data[2] - initial_position_.data[2]);
    ROS_INFO_STREAM("Position master 1: " << position_master_1 << ", Position master 2: " << position_master_2);

    // マスタ側の張力
    double tension_master_1 = latest_tension_.data[index_master_1_];
    double tension_master_2 = latest_tension_.data[index_master_2_];
    ROS_INFO_STREAM("Tension master 1: " << tension_master_1*1000 << ", Tension master 2: " << tension_master_2*1000);

    // スレーブ側の位置
    int position_slave_1 = motor_inverse_[1] * (latest_position_.data[1] - initial_position_.data[1]);
    int position_slave_2 = motor_inverse_[3] * (latest_position_.data[3] - initial_position_.data[3]);
    ROS_INFO_STREAM("Position slave 1: " << position_slave_1 << ", Position slave 2: " << position_slave_2);

    // スレーブ側の張力
    double tension_slave_1 = latest_tension_.data[index_slave_1_];
    double tension_slave_2 = latest_tension_.data[index_slave_2_];
    ROS_INFO_STREAM("Tension slave 1: " << tension_slave_1*1000 << ", Tension slave 2: " << tension_slave_2*1000);

    // コマンド計算
    cmd.data[1] = static_cast<int>(motor_inverse_[1] * Kp_ * (position_master_1 - position_slave_1) + initial_position_.data[1]);
    cmd.data[3] = static_cast<int>(motor_inverse_[3] * Kp_ * (position_master_2 - position_slave_2) + initial_position_.data[3]);
    cmd.data[0] = static_cast<int>(motor_inverse_[0] * Kf_ * (tension_slave_2 - tension_master_1) + latest_position_.data[0]);
    cmd.data[2] = static_cast<int>(motor_inverse_[2] * Kf_ * (tension_slave_1 - tension_master_2) + latest_position_.data[2]);

    pub_cmd_.publish(cmd);
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_pos_, sub_mag_;
  ros::Publisher pub_cmd_;
  ros::Timer timer_;

  std_msgs::Int32MultiArray initial_position_;
  std_msgs::Float64MultiArray initial_tension_;
  std_msgs::Int32MultiArray latest_position_;
  std_msgs::Float64MultiArray latest_tension_;

  std::deque<std_msgs::Float64MultiArray> tension_buffer_;

  bool initialized_position_, initialized_tension_, initialized_position_second_, initialized_tension_second_;
  int tension_threshold_;
  int pull_value_gain_;
  double Kp_, Kf_, Kd_;
  int index_master_1_, index_master_2_, index_slave_1_, index_slave_2_;
  int T_max_1_, T_max_2_;
  double tension_margin_;
  std::vector<int> motor_inverse_;
  int avg_count_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "bilateral_force_node");
  ros::NodeHandle nh("~");
  BilateralTestNode node(nh);
  ros::spin();
  return 0;
}

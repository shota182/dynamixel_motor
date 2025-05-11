// bilateral_test_node.cpp
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <deque>

class BilateralTestNode {
public:
  BilateralTestNode(ros::NodeHandle& nh)
    : nh_(nh){
    // パラメータ読み込み
    nh_.param("pull_value_gain", pull_value_gain_, 10);
    nh_.param("Kp", Kp_, 10.0);
    nh_.param("Kf", Kf_, 1.0);
    nh_.param("Kd", Kd_, 5.0);
    nh_.param("index_master", index_master_, 0);
    nh_.param("index_slave", index_slave_, 2);
    nh_.param("tension_margin", tension_margin_, 30);
    nh_.param("avg_count", avg_count_, 5);

    initialized_position_ = false;
    initialized_tension_ = false;
    sub_pos_ = nh_.subscribe("/sensor/motor/output/position", 1, &BilateralTestNode::positionCallback, this);
    sub_mag_ = nh_.subscribe("/sensor/mag", 1, &BilateralTestNode::tensionCallback, this);
    pub_cmd_ = nh_.advertise<std_msgs::Int32MultiArray>("/sensor/motor/input/position", 10);
    timer_ = nh_.createTimer(ros::Duration(0.02), &BilateralTestNode::controlLoop, this);
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
          initial_tension_.data[i] = sum / avg_count_;
        }
        T_max_ = tension_margin_ + initial_tension_.data[index_master_];
        initialized_tension_ = true;
        ROS_INFO_STREAM("Initial tension averaged over " << avg_count_ << " samples.");
      }
    }
  }

  void controlLoop(const ros::TimerEvent&) {
    if (!(initialized_position_ && initialized_tension_)) return;
    std_msgs::Int32MultiArray cmd;
    cmd.data.resize(latest_position_.data.size());
    // マスタ側モータ位置の変化量
    int theta_m = latest_position_.data[0] - initial_position_.data[0];
    // マスタ側磁気センサ値の変化量
    int Tm = latest_tension_.data[index_master_] - initial_tension_.data[index_master_];

    // スレーブ側モータ位置の変化量
    int theta_s = latest_position_.data[1] - initial_position_.data[1];
    // スレーブ側磁気センサ値の変化量
    int Ts = latest_tension_.data[index_slave_] - initial_tension_.data[index_slave_];

    // ログ出力
    // ROS_INFO_STREAM_THROTTLE(1.0,
    //     "\n--- --- --- master --- --- ---\n" <<
    //     "latest_position_.data[0]:" << latest_position_.data[0] << "\n" <<
    //     "initial_position_.data[0]:" << initial_position_.data[0] << "\n" <<
    //     "latest_tension_.data[index_master_]:" << latest_tension_.data[index_master_] << "\n" <<
    //     "initial_tension_.data[index_master_]:" << initial_tension_.data[index_master_] << "\n" <<
    //     "--- --- --- slave --- --- ---\n" <<
    //     "latest_position_.data[1]:" << latest_position_.data[1] << "\n" <<
    //     "initial_position_.data[1]:" << initial_position_.data[1] << "\n" <<
    //     "latest_tension_.data[index_slave_]:" << latest_tension_.data[index_slave_] << "\n" <<
    //     "initial_tension_.data[index_slave_]:" << initial_tension_.data[index_slave_] << "\n");

    // スレーブ
    if (latest_tension_.data[index_slave_] < initial_tension_.data[index_slave_]){
      // tension is low
      cmd.data[1] = latest_position_.data[1] + pull_value_gain_*(latest_tension_.data[index_slave_] - initial_tension_.data[index_slave_]);
    } else{
      // tension is good
      cmd.data[1] = theta_m + initial_position_.data[1];
    }
    // マスタ
    if (latest_tension_.data[index_master_] < initial_tension_.data[index_master_]){
        // tension is low
        cmd.data[0] = latest_position_.data[0] + pull_value_gain_*(latest_tension_.data[index_master_] - initial_tension_.data[index_master_]);
        // ROS_INFO_STREAM_THROTTLE(1.0, "low (cmd[0] = " << cmd.data[0] << ")\n");
    } else if (latest_tension_.data[index_master_] > T_max_){
        // tension is high
        cmd.data[0] = latest_position_.data[0] + pull_value_gain_*(latest_tension_.data[index_master_] - T_max_);  // 張りすぎ → 緩める
        // ROS_INFO_STREAM_THROTTLE(1.0, "high (cmd[0] = " << cmd.data[0] << ")\n");
    } else{
        // tension is good
        cmd.data[0] = latest_position_.data[0];                // 範囲内 → 維持
        // ROS_INFO_STREAM_THROTTLE(1.0, "good: (cmd[0] = " << cmd.data[0] << ")\n");
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
  int pull_value_gain_;
  double Kp_, Kf_, Kd_;
  int index_master_, index_slave_;
  int T_max_, tension_margin_;
  int avg_count_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "bilateral_test_node");
  ros::NodeHandle nh("~");
  BilateralTestNode node(nh);
  ros::spin();
  return 0;
}

// 7/9
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <deque>
#include <iomanip>
#include <algorithm>

class BilateralTestNode {
public:
  BilateralTestNode(ros::NodeHandle& nh)
    : nh_(nh) {
    // settings.yamlからパラメータを取得
    nh_.getParam("tension_threshold", tension_threshold_);
    nh_.getParam("pull_value_gain", pull_value_gain_);
    nh_.getParam("Kf", Kf_);
    nh_.getParam("Kp", Kp_);
    nh_.getParam("Ki", Ki_);
    nh_.getParam("Kd", Kd_);
    nh_.getParam("Ksm", Ksm_);
    nh_.getParam("index_master_1", index_master_1_);
    nh_.getParam("index_master_2", index_master_2_);
    nh_.getParam("index_slave_1", index_slave_1_);
    nh_.getParam("index_slave_2", index_slave_2_);
    nh_.getParam("motor_inverse", motor_inverse_);
    nh_.getParam("tension_margin", tension_margin_);
    nh_.getParam("avg_count", avg_count_);
    nh_.getParam("output_log", output_log_);
    nh_.getParam("control_loop_freq", control_loop_freq_); // 制御周期を取得

    initialized_position_ = false;
    initialized_tension_ = false;
    updated_position = false;
    updated_tension = false;

    control_interval_ = 1.0 / control_loop_freq_; // 制御周期を計算

    sub_pos_ = nh_.subscribe("/sensor/motor/output/position", 1, &BilateralTestNode::positionCallback, this);
    sub_mag_ = nh_.subscribe("/force/input/tension", 1, &BilateralTestNode::tensionCallback, this);
    pub_cmd_ = nh_.advertise<std_msgs::Int32MultiArray>("/sensor/motor/input/position", 10);
    timer_ = nh_.createTimer(ros::Duration(1/control_loop_freq_), &BilateralTestNode::controlLoop, this);

    ROS_INFO("BilateralTestNode initialized with control loop interval: %.5f seconds", 1/control_loop_freq_);
  }

private:
  void positionCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    previous_position_ = latest_position_;
    latest_position_ = *msg;
    updated_position = true;
    if (!initialized_position_) {
      initial_position_ = *msg;
      initialized_position_ = true;
    }
  }

  void tensionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    previous_tension_ = latest_tension_;
    latest_tension_ = *msg;
    updated_tension = true;
    // ROS_INFO_STREAM("Received tension data with size: " << msg->data.size());

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
        initialized_tension_ = true;
        ROS_INFO_STREAM("Initial tension averaged over " << avg_count_ << " samples.");
      }
    }
  }

  void controlLoop(const ros::TimerEvent&) {
    if(output_log_){
      ROS_INFO_STREAM("\nLoop Start");
    }
    if (!(initialized_position_ && initialized_tension_)) {
      ROS_WARN_ONCE("Position or tension not initialized. Skipping control loop.");
      return;
    }

    if(!updated_position || !updated_tension) {
      return;
    }

    std_msgs::Int32MultiArray cmd;
    cmd.data = latest_position_.data; // 最新の位置データをコマンドに設定

    // マスタ側の位置
    int position_master_1 = motor_inverse_[index_master_1_] * (latest_position_.data[index_master_1_] - initial_position_.data[index_master_1_]);
    int position_master_2 = motor_inverse_[index_master_2_] * (latest_position_.data[index_master_2_] - initial_position_.data[index_master_2_]);

    // マスタ側の張力
    double tension_master_1 = latest_tension_.data[index_master_1_];
    double tension_master_2 = latest_tension_.data[index_master_2_];

    // スレーブ側の位置
    int position_slave_1 = motor_inverse_[index_slave_1_] * (latest_position_.data[index_slave_1_] - initial_position_.data[index_slave_1_]);
    int position_slave_2 = motor_inverse_[index_slave_2_] * (latest_position_.data[index_slave_2_] - initial_position_.data[index_slave_2_]);

    // スレーブ側の張力
    double tension_slave_1 = latest_tension_.data[index_slave_1_];
    double tension_slave_2 = latest_tension_.data[index_slave_2_];

    // 張力を比較して低い方をmarginに
    if(tension_master_1 < tension_master_2) {
      target_tension_master_1_ = tension_margin_;
      target_tension_master_2_ = tension_slave_1;
    } else {
      target_tension_master_1_ = tension_slave_2;
      target_tension_master_2_ = tension_margin_;
    }

    // マスタ1の計算
    error[0] = target_tension_master_1_ - tension_master_1;
    derivative[0] = (tension_master_1 - previous_tension_.data[index_master_1_]) / control_interval_;
    integral_error_[0] += error[0] * control_interval_;
    cmd.data[index_master_1_] = static_cast<int>(motor_inverse_[index_master_1_] * (Kp_ * error[0] + Kd_ * derivative[0] + Ki_ * integral_error_[0]) + latest_position_.data[index_master_1_]);
    // マスタ2の計算
    error[1] = target_tension_master_2_ - tension_master_2;
    derivative[1] = (tension_master_2 - previous_tension_.data[index_master_2_]) / control_interval_;
    integral_error_[1] += error[1] * control_interval_;
    cmd.data[index_master_2_] = static_cast<int>(motor_inverse_[index_master_2_] * (Kp_ * error[1] + Kd_ * derivative[1] + Ki_ * integral_error_[1]) + latest_position_.data[index_master_2_]);

    // masterのposition拮抗量から計算
    target_position_slave_1_ = (position_master_1-position_master_2) / 2;
    target_position_slave_2_ = (position_master_2-position_master_1) / 2;
    // スレーブの計算
    cmd.data[index_slave_1_] = static_cast<int>(motor_inverse_[index_slave_1_] * (Kf_ * (target_position_slave_1_) + pull_value_gain_) + initial_position_.data[index_slave_1_]);
    cmd.data[index_slave_2_] = static_cast<int>(motor_inverse_[index_slave_2_] * (Kf_ * (target_position_slave_2_) + pull_value_gain_) + initial_position_.data[index_slave_2_]);

    // 入力位置の更新
    // for (size_t i = 0; i < latest_position_.data.size(); ++i) {
    //   input_position_.data[i] = cmd.data[i];
    // }

    pub_cmd_.publish(cmd);

    // log
    if(output_log_){
      ROS_INFO_STREAM("Position master 1: " << position_master_1 << ", Position master 2: " << position_master_2);
      ROS_INFO_STREAM("Position slave 1: " << position_slave_1 << ", Position slave 2: " << position_slave_2);
      ROS_INFO_STREAM("Tension master 1: " << tension_master_1 << ", Tension master 2: " << tension_master_2);
      ROS_INFO_STREAM("Tension slave 1: " << tension_slave_1 << ", Tension slave 2: " << tension_slave_2);
      ROS_INFO_STREAM("slave1 is: " << cmd.data[1] << "=" << motor_inverse_[1] << " * (" << Kf_ << " * (" << position_master_1 << " - " << position_slave_1 << ")) + " << initial_position_.data[1]);
      ROS_INFO_STREAM("slave2 is: " << cmd.data[3] << "=" << motor_inverse_[3] << " * (" << Kf_ << " * (" << position_master_2 << " - " << position_slave_2 << ")) + " << initial_position_.data[3]);
      ROS_INFO_STREAM("master1 is: " << cmd.data[0] << "=" << motor_inverse_[0] << " * (" << Kp_ << " * " << error << " + " << Kd_ << " * " << derivative << " + " << Ki_ << " * " << integral_error_[0] << ") + " << latest_position_.data[0]);
      ROS_INFO_STREAM("master2 is: " << cmd.data[2] << "=" << motor_inverse_[2] << " * (" << Kp_ << " * " << error << " + " << Kd_ << " * " << derivative << " + " << Ki_ << " * " << integral_error_[1] << ") + " << latest_position_.data[2]);
    }
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_pos_, sub_mag_;
  ros::Publisher pub_cmd_;
  ros::Timer timer_;

  std_msgs::Int32MultiArray initial_position_;
  std_msgs::Float64MultiArray initial_tension_, previous_tension_;
  std_msgs::Int32MultiArray latest_position_, previous_position_, input_position_;
  std_msgs::Float64MultiArray latest_tension_;

  std::deque<std_msgs::Float64MultiArray> tension_buffer_;

  bool initialized_position_, initialized_tension_;
  bool updated_position, updated_tension;
  int tension_threshold_;
  int pull_value_gain_;
  double Kf_, Kp_, Ki_, Kd_, Ksm_;
  double error[2] = {0.0, 0.0}; // エラー項を保持する配列
  double derivative[2] = {0.0, 0.0}; // 微分項を保持する配列
  double integral_error_[2] = {0.0, 0.0}; // 積分項を保持する配列
  double control_interval_;
  int index_master_1_, index_master_2_, index_slave_1_, index_slave_2_;
  double target_tension_master_1_, target_tension_master_2_;
  int target_position_slave_1_, target_position_slave_2_;
  double tension_margin_;
  std::vector<int> motor_inverse_;
  int avg_count_;
  bool output_log_;
  double control_loop_freq_; // 制御周期
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "bilateral_force_node");
  ros::NodeHandle nh("~");
  BilateralTestNode node(nh);
  ros::spin();
  return 0;
}

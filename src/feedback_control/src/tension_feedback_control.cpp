#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64.h>
#include <vector>

class PDControlNode {
public:
  PDControlNode(ros::NodeHandle& nh)
    : nh_(nh), initialized_tension_(false), initialized_position_(false) {
    nh_.param("Kp", Kp_, 1.0);
    nh_.param("Kd", Kd_, 0.1);
    nh_.param("publish_index", publish_index_, 0); // 配列の何番目をpubするか指定
    nh_.param("motor_inverse", motor_inverse_, std::vector<int>{-1, 1}); // モーターの反転設定（1: 正方向, -1: 逆方向）

    sub_tension_ = nh_.subscribe("/force/input/tension", 1, &PDControlNode::tensionCallback, this);
    sub_target_tension_ = nh_.subscribe("/sensor/motor/input/tension", 1, &PDControlNode::targetTensionCallback, this);
    sub_motor_position_ = nh_.subscribe("/sensor/motor/output/position", 1, &PDControlNode::motorPositionCallback, this);
    pub_position_ = nh_.advertise<std_msgs::Int32MultiArray>("/sensor/motor/input/position", 10);

    // デバッグ用トピック
    pub_selected_position_ = nh_.advertise<std_msgs::Float64>("/debug/selected_position", 10);
    pub_selected_target_tension_ = nh_.advertise<std_msgs::Float64>("/debug/selected_target_tension", 10);
  }

private:
  void tensionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.empty() || target_tension_.empty() || !initialized_position_) {
    //   ROS_WARN("Received empty tension data or target tension is not initialized. Skipping.");
      return;
    }

    if (!initialized_tension_) {
      previous_tension_ = msg->data;
      initialized_tension_ = true;
      return;
    }

    // target_tension_のサイズをtensionのサイズに合わせる
    if (target_tension_.size() < motor_positions_.size()) {
      target_tension_.resize(motor_positions_.size(), 0.0); // 不足分を0.0で埋める
    }

    // motor_inverseのサイズをposition_cmdのサイズに合わせる
    if (motor_inverse_.size() < motor_positions_.size()) {
      motor_inverse_.resize(motor_positions_.size(), 1); // 不足分を1（正方向）で埋める
    }

    std_msgs::Int32MultiArray position_cmd;

    for (size_t i = 0; i < motor_positions_.size(); ++i) {
      double error = target_tension_[i] - msg->data[i];
      double derivative = (msg->data[i] - previous_tension_[i]) / control_interval_;
      // PD制御の計算（モーターの反転を考慮）
      position_cmd.data[i] = static_cast<int>(motor_inverse_[i] * (Kp_ * error - Kd_ * derivative) + motor_positions_[i]);

      // 計算結果のデバッグログ
      // ROS_INFO("Index: %zu, Target Tension: %.2f, Current Tension: %.2f, Error: %.2f, Derivative: %.2f", i, target_tension_[i], msg->data[i], error, derivative);
      // ROS_INFO("Motor Inverse: %d, Motor Position: %d, Position Command: %d", motor_inverse_[i], motor_positions_[i], position_cmd.data[i]);
    }

    previous_tension_ = msg->data;
    pub_position_.publish(position_cmd);

    // ROS_INFO_STREAM("publish_index" << publish_index_);
    // ROS_INFO_STREAM("msg->data[tension_id_[publish_index_]" << msg->data[tension_id_[publish_index_]]);
    // ROS_INFO_STREAM("target_tension_[publish_index_]" << target_tension_[publish_index_]);
    // 指定されたインデックスの値をpub
    if (publish_index_ >= 0 && publish_index_ < msg->data.size()) {
      std_msgs::Float64 selected_position_msg;
      selected_position_msg.data = msg->data[publish_index_]; // tension_idに基づいて選択された位置を取得
      pub_selected_position_.publish(selected_position_msg);

      std_msgs::Float64 selected_target_tension_msg;
      selected_target_tension_msg.data = target_tension_[publish_index_];
      pub_selected_target_tension_.publish(selected_target_tension_msg);
    } else {
      ROS_WARN("publish_index is out of range. Skipping publishing selected values.");
    }
  }

  void targetTensionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.empty()) {
      ROS_WARN("Received empty target tension data. Skipping.");
      return;
    }

    target_tension_ = msg->data; // target_tension_を更新
    ROS_INFO("Updated target tension.");
  }

  void motorPositionCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    if (msg->data.empty()) {
      ROS_WARN("Received empty motor position data. Skipping.");
      return;
    }
    if (!initialized_position_) {
        initialized_position_ = true; // 初期化フラグを立てる
        initial_position_ = msg->data; // 初期位置を保存
    }
    motor_positions_ = msg->data; // モーターの位置データを更新
    ROS_INFO("Updated motor positions.");
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_tension_;
  ros::Subscriber sub_target_tension_; // target_tensionを購読するトピック
  ros::Subscriber sub_motor_position_; // motor/output/positionを購読するトピック
  ros::Publisher pub_position_;
  ros::Publisher pub_selected_position_;       // 指定されたインデックスのposition_cmdを発行
  ros::Publisher pub_selected_target_tension_; // 指定されたインデックスのtarget_tensionを発行

  std::vector<double> previous_tension_;
  std::vector<double> target_tension_;
  std::vector<int> motor_inverse_; // モーターの反転設定（1: 正方向, -1: 逆方向）
  std::vector<int> motor_positions_, initial_position_; // モーターの位置データ
  bool initialized_tension_, initialized_position_;
  double Kp_, Kd_;
  int publish_index_; // 配列のインデックスを指定
  const double control_interval_ = 0.02; // 制御周期（秒）
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pd_control_node");
  ros::NodeHandle nh("~");
  PDControlNode node(nh);
  ros::spin();
  return 0;
}
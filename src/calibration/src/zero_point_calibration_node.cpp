#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/MagneticField.h>
#include <deque>

class ZeroPointCalibrationNode {
public:
  ZeroPointCalibrationNode(ros::NodeHandle& nh) : nh_(nh), calibrated_(false), initialized_(false) {
    // YAMLからパラメータ読み込み
    nh_.param("calibration_samples", mag_window_size_, 5);
    nh_.param("pull_time", pull_time_, 0.3);
    nh_.param("stop_time", stop_time_, 1);
    nh_.param("pull_step_max", pull_step_max_, 512); // [step]pull_time_中に引かれるステップ数 // 512で45[deg]


    sub_pos_ = nh_.subscribe("/sensor/motor/output/position", 1, &ZeroPointCalibrationNode::positionCallback, this);
    sub_mag_ = nh_.subscribe("/sensor/mag", 1, &ZeroPointCalibrationNode::magCallback, this);
    pub_cmd_ = nh_.advertise<std_msgs::Int32MultiArray>("/sensor/motor/input/position", 10);
    timer_ = nh_.createTimer(ros::Duration(0.02), &ForceControlNode::execute, this);

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

  std_msgs::Int32MultiArray initial_position_, current_position_;

  std::deque<sensor_msgs::Int32MultiArray> mag_buffer_;
  std_msgs::Int32MultiArray mag_not_changed_;
  int mag_window_size_;
  double pull_time_;
  double stop_time_;
  int pull_step_max_;
  bool calibrated_;
  bool initialized_position_ = false;
  bool initialized_mag_ = false;

  enum class State {
    INITIALIZING,  // 初期化中
    PUBLISHING,     // → 0.3秒後 → NON_PUBLISHING
    NON_PUBLISHING, // → 1.0秒後 → CALCULATING
    CALCULATING     // → 計算完了 → PUBLISHING
  };

  State current_state_ = State::INITIALIZING;
  ros::Time state_start_time_;
  ros::Publisher pub_;
  std::deque<std::vector<int32_t>> buffer_;
  int buffer_size_ = 5;


  void positionCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    // 初回のみ位置を保存
    if (!initialized_position_) {
      initial_position_ = *msg;
      initialized_position_ = true;
      ROS_INFO("Initial motor position received and stored.");
    }
    // 最新の位置を保存
    current_position_ = *msg;
  }


  void magCallback(const sensor_msgs::Int32MultiArray::ConstPtr& msg) {
    addDataToBuffer(mag_buffer_, msg->data, mag_window_size_);
    initialized_mag_ = true;
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

  void addDataToBuffer(std::deque<std::vector<int>>& buffer,
                      const std::vector<int>& new_data,
                      int data_length)  // データ数の最大値
  {
    if (data_length > 0 && static_cast<int>(buffer.size()) >= data_length) {
      buffer.pop_front();
    }データ数
    buffer.push_back(new_data);
  }

  void inputMotorPosition(const std::vector<int>& position) {
    std_msgs::Int32MultiArray msg;
    msg.data = position;
    pub_cmd_.publish(msg);
  }

void execute(const ros::TimerEvent&)
{
    // 速度vで0.3[sec]くらい引く
      // vは1[sec]あたりのモータステップ数
        // 0.3[sec]の間pub
      // 都度1[sec]止める
      // 傾きを取得して変化点を算出
        // window slidingでデータ点の変化を計算
        // しきい値を超えたタイミングを変化点とする
      // 遅延を考慮して数[sec]のモータステップを取得する
    // 速度を下げて同じ作業を繰り返す
  if(!initialized_position_ || !initialized_mag_) {
    // 初期位置が取得できていない場合
    ROS_WARN("Motor position or Mag not initialized yet.");
    return;
  }
  ros::Time now = ros::Time::now();
  ros::Duration elapsed = now - state_start_time_;

  // 状態遷移
  switch (current_state_) {
    case State::INITIALIZING: 
      {
        // 初期化中
        if (initialized_position_ && initialized_mag_) {
          current_state_ = State::PUBLISHING;
          state_start_time_ = now;
        }
        break;
      }
    case State::PUBLISHING:
      {
        if (elapsed.toSec() >= pull_time_) {
          current_state_ = State::NON_PUBLISHING;
          state_start_time_ = now;
        }
        break;
      }

    case State::NON_PUBLISHING:
      {
        if (elapsed.toSec() >= stop_time_) {
          current_state_ = State::CALCULATING;
          state_start_time_ = now;
        }
        break;
      }

    case State::CALCULATING:
      {
        std::vector<int32_t> avg = computeAverageFromBuffer(buffer_);
        sensor_msgs::Int32MultiArray msg;
        msg.data = avg;
        pub_.publish(msg);

        current_state_ = State::INITIALIZING;
        state_start_time_ = now;
        break;
      }
  }

  // 処理を記述
  switch (current_state_) {
    case State::INITIALIZING: 
      {
        if (initialized_position_ && initialized_mag_) {
          // モータの初期位置を更新
          initial_position_ = current_position_;
        }
        else {
          ROS_WARN("Motor position or Mag not initialized yet.");
        }
        break;
      }
    case State::PUBLISHING:
      {
        // ステップと時間から引く量を計算
        // mag_not_changed_ = false
        sensor_msgs::Int32MultiArray msg;
        msg.data = current_position_.data;
        for (size_t i = 0; i < msg.data.size(); ++i) {
          msg.data[i] = initial_position_.data[i] + mag_not_changed_[i] * pull_step_max_ * ((elapsed.toSec()) / pull_time_);
        }
        // 差分をとってpublish
        if (!buffer_.empty()) {
          msg.data = buffer_.back();  // 最新データを送信（例）
          pub_.publish(msg);
        }
        break;
      }

    case State::NON_PUBLISHING:
      {
        // 受信のみ（何もしない）
        if (elapsed.toSec() >= 1.0) {
          current_state_ = State::CALCULATING;
          state_start_time_ = now;
        }
        break;
      }

    case State::CALCULATING:
      {
        // 計算して即PUBLISHINGへ
        std::vector<int32_t> avg = computeAverageFromBuffer(buffer_);
        sensor_msgs::Int32MultiArray msg;
        mag_not_changed_を0にする
        msg.data = avg;
        pub_.publish(msg);

        current_state_ = State::PUBLISHING;
        state_start_time_ = now;
        break;
      }
  }
}

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "zero_point_calibration_node");
  ros::NodeHandle nh("~");  // プライベート名前空間から読み込む
  ZeroPointCalibrationNode node(nh);
  node.spin();
  return 0;
}

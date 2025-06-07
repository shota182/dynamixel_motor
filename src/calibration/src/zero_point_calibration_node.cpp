#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/MagneticField.h>
#include <deque>
#include <vector>
#include <cmath>

class ZeroPointCalibrationNode {
public:
  ZeroPointCalibrationNode(ros::NodeHandle& nh) : nh_(nh) {
    // YAMLからパラメータ読み込み
    nh_.param("calibration_samples", mag_calibration_samples_, 20);
    nh_.param("mag_window_size", mag_window_size_, 5);
    nh_.param("pull_time", pull_time_, 0.3);
    nh_.param("stop_time", stop_time_, 1.0);
    nh_.param("repeat", repeat_, 3);
    nh_.param("pull_step_max", pull_step_max_, 512);
    nh_.param("dist_threshold", dist_threshold_, 10);
    nh_.getParam("mag_target_indices", mag_target_indices_);
    nh_.getParam("motor_direction", motor_direction_);

    sub_pos_ = nh_.subscribe("/sensor/motor/output/position", 1, &ZeroPointCalibrationNode::positionCallback, this);
    sub_mag_ = nh_.subscribe("/sensor/mag", 1, &ZeroPointCalibrationNode::magCallback, this);
    pub_cmd_ = nh_.advertise<std_msgs::Int32MultiArray>("/sensor/motor/input/position", 10);
    timer_ = nh_.createTimer(ros::Duration(0.02), &ZeroPointCalibrationNode::execute, this);

    ROS_INFO("Zero Point Calibration Node initialized.");
  }

  void spin() {
    ros::spin();
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_pos_;
  ros::Subscriber sub_mag_;
  ros::Publisher pub_cmd_;
  ros::Timer timer_;

  std_msgs::Int32MultiArray initial_position_, pull_start_position_, current_position_, changed_position_;
  std::deque<std_msgs::Int32MultiArray> mag_buffer_;
  std::vector<bool> mag_not_changed_;
  int mag_calibration_samples_, mag_window_size_;
  double pull_time_, stop_time_;
  int repeat_, pull_step_max_, dist_threshold_;
  std::vector<int> mag_target_indices_, motor_direction_;
  bool initialized_position_ = false, initialized_mag_ = false;

  struct MeanVarianceResult {
    std::vector<double> mean;
    std::vector<double> variance;
  };
  MeanVarianceResult initial_mag_, current_mag_mv_;

  enum class State {
    INITIALIZING,  // 初期化中
    PUBLISHING,     // → 0.3秒後 → NON_PUBLISHING
    NON_PUBLISHING, // → 1.0秒後 → CALCULATING
    CALCULATING,     // → 計算完了 → PUBLISHING
    FINISHED        // 完了
  };

  State current_state_ = State::INITIALIZING;
  ros::Time state_start_time_ = ros::Time::now();

  void positionCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    if (!initialized_position_) {
      initial_position_ = *msg;
      initialized_position_ = true;
      ROS_INFO("Initial motor position received and stored.");
    }
    current_position_ = *msg;
  }

  void magCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    std::vector<int32_t> selected_data;
    for (int idx : mag_target_indices_) {
      if (idx >= 0 && idx < static_cast<int>(msg->data.size())) {
        selected_data.push_back(msg->data[idx]);
      } else {
        ROS_WARN("Mag index %d out of range in received message.", idx);
        selected_data.push_back(0);
      }
    }
    if (!initialized_mag_) {
      addDataToBuffer(mag_buffer_, selected_data, mag_calibration_samples_);
      if (mag_buffer_.size() >= mag_calibration_samples_) {
        initial_mag_ = computeMeanAndVariance(mag_buffer_);
        initialized_mag_ = true;
        clearBuffer(mag_buffer_);
      }
    } else if (current_state_ == State::NON_PUBLISHING) {
      addDataToBuffer(mag_buffer_, selected_data, mag_calibration_samples_);
    }
  }


  // bufferを引数として，中身の平均と分散を取る
  MeanVarianceResult computeMeanAndVariance(const std::deque<std_msgs::Int32MultiArray>& buffer) {
    MeanVarianceResult result;
    if (buffer.empty() || buffer.front().data.empty()) {
      ROS_WARN("Buffer is empty.");
      return result;
    }
    size_t N = buffer.size();
    size_t D = buffer.front().data.size();
    result.mean.resize(D, 0.0);
    result.variance.resize(D, 0.0);

    // 平均の計算
    for (const auto& vec : buffer) {
      if (vec.data.size() != D) {
        ROS_WARN("Data size mismatch.");
        return MeanVarianceResult{};
      }
      for (size_t i = 0; i < D; ++i) {
        result.mean[i] += vec.data[i];
      }
    }
    for (size_t i = 0; i < D; ++i) {
      result.mean[i] /= static_cast<double>(N);
    }
    // 分散の計算
    for (const auto& vec : buffer) {
      for (size_t i = 0; i < D; ++i) {
        double diff = static_cast<double>(vec.data[i]) - result.mean[i];
        result.variance[i] += diff * diff;
      }
    }
    for (size_t i = 0; i < D; ++i) {
      result.variance[i] /= static_cast<double>(N);
    }

    return result;
  }

  void addDataToBuffer(std::deque<std_msgs::Int32MultiArray>& buffer, const std::vector<int32_t>& new_data, int data_length) {
    if (data_length > 0 && static_cast<int>(buffer.size()) >= data_length) {
      buffer.pop_front();
    }
    std_msgs::Int32MultiArray msg;
    msg.data = new_data;
    buffer.push_back(msg);
  }

  // バッファをクリアする
  void clearBuffer(std::deque<std_msgs::Int32MultiArray>& buffer) {
    buffer.clear();
  }

  void checkMagDistanceElementwise(const std::vector<double>& current_mean, const std::vector<double>& initial_mean) {
    if (current_mean.size() != initial_mean.size()) {
        ROS_WARN("Size mismatch between current_mean and initial_mean.");
        return;
    }

    if (mag_not_changed_.size() != current_mean.size()) {
        mag_not_changed_.resize(current_mean.size(), true);
    }

    for (size_t i = 0; i < current_mean.size(); ++i) {
        double diff = std::abs(current_mean[i] - initial_mean[i]);
        if (diff > dist_threshold_) {
            mag_not_changed_[i] = false;
        } else {
            mag_not_changed_[i] = true;
        }
    }
}

  void execute(const ros::TimerEvent&) {
    if (!initialized_position_ || !initialized_mag_) {
      ROS_WARN("Motor position or Mag not initialized yet.");
      return;
    }
    mag_not_changed_.assign(initial_mag_.mean.size(), true);
    ros::Time now = ros::Time::now();
    ros::Duration elapsed = now - state_start_time_;

    // 状態遷移
    switch (current_state_) {
      case State::INITIALIZING: 
        {
          // 初期化中
          if (initialized_position_ && initialized_mag_) {
            if (elapsed.toSec() >= 1.0) {
              current_state_ = State::PUBLISHING;
              // モータの初期位置を更新
              pull_start_position_ = current_position_;
              state_start_time_ = now;
            }
          }
          break;
        }
      case State::PUBLISHING:
        {
          if (elapsed.toSec() >= pull_time_) {
            current_state_ = State::NON_PUBLISHING;
            clearBuffer(mag_buffer_);
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
          if(repeat_ <= 0) {
            // 繰り返し回数が0以下になったら終了
            ROS_INFO("Zero Point Calibration completed.");
            current_state_ = State::FINISHED;
            break;
          }
          current_state_ = State::INITIALIZING;
          state_start_time_ = now;
          break;
        }
      case State::FINISHED:
        {
          return;
        }
    }

    // 処理を記述
    switch (current_state_) {
      case State::INITIALIZING: 
        {
          if (initialized_position_ && initialized_mag_) {
            // モータの初期位置を更新
            pull_start_position_ = current_position_;
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
          std_msgs::Int32MultiArray msg;
          msg.data = current_position_.data;
          for (size_t i = 0; i < msg.data.size(); i++) {
            if(mag_not_changed_[i]) {
              // 変化がなかった場合は、pull_start_position_から引く
              msg.data[i] = int(pull_start_position_.data[i] + motor_direction_[i] * pull_step_max_ * ((elapsed.toSec()) / pull_time_));
            }
          }
          pub_cmd_.publish(msg);
          break;
        }

      case State::NON_PUBLISHING:
        {
          // 受信のみ（何もしない）
          break;
        }
      case State::CALCULATING:
        {
          // magデータの平均を計算
          current_mag_mv_ = computeMeanAndVariance(mag_buffer_);
          // 初期化されたmagデータと比較
          checkMagDistanceElementwise(current_mag_mv_.mean, initial_mag_.mean);
          // すべてfalseならばステップを小さくして再開
          if (std::all_of(mag_not_changed_.begin(), mag_not_changed_.end(), [](bool v) { return v == 0; })) {
            // 変化点を回転前後の間と定義
            ROS_INFO("Magnetic data did not change significantly, reducing pull step and repeating.");
            // changed_position_ = current_position_;
            size_t n = current_position_.data.size();
            changed_position_.data.resize(n);
            for (size_t i = 0; i < n; ++i) {
              changed_position_.data[i] = int(current_position_.data[i] + pull_start_position_.data[i])/2;
              pull_start_position_.data[i] = int((current_position_.data[i] + pull_start_position_.data[i])/2 - motor_direction_[i] * pull_step_max_);
            }
            pub_cmd_.publish(pull_start_position_);
            // すべて false のときの処理
            pull_step_max_ /= 2;
            repeat_--;
          }
          break;
        }
      case State::FINISHED:
        {
          pub_cmd_.publish(changed_position_);
          // 磁気センサ値とモータ位置の初期化が完了したら終了
          ROS_INFO("Zero Point Calibration finished.");
          // キャリブレーション結果の表示
          std::string pos_str = "[";
          for (size_t i = 0; i < changed_position_.data.size(); ++i) {
            pos_str += std::to_string(changed_position_.data[i]);
            if (i != changed_position_.data.size() - 1) pos_str += ", ";
          }
          pos_str += "]";

          std::string mag_str = "[";
          for (size_t i = 0; i < initial_mag_.mean.size(); ++i) {
            mag_str += std::to_string(initial_mag_.mean[i]);
            if (i != initial_mag_.mean.size() - 1) mag_str += ", ";
          }
          mag_str += "]";

          ROS_INFO("\nCalibration completed with \nposition: %s, \nmag: %s\n", pos_str.c_str(), mag_str.c_str());
          return;  // ループを抜ける
        }
    }
  }

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "zero_point_calibration_node");
  ros::NodeHandle nh("~");
  ZeroPointCalibrationNode node(nh);
  node.spin();
  return 0;
}

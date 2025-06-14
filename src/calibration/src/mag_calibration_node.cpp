#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <deque>

class MagCalibrationNode {
public:
  MagCalibrationNode() : nh_("~"), initialized_mag_(false), initialized_motor_(false) {
    // パラメータを取得
    nh_.getParam("csv_path", csv_path_);
    nh_.getParam("mag_avg_count", mag_avg_count_);
    nh_.getParam("mag_save_index", mag_save_index_);
    nh_.getParam("motor_inverse", motor_inverse_);
    nh_.getParam("save_dir", save_dir_);

    sub_mag_ = nh_.subscribe("/sensor/mag", 1, &MagCalibrationNode::magCallback, this);
    sub_pos_ = nh_.subscribe("/sensor/motor/output/position", 1, &MagCalibrationNode::positionCallback, this);
    pub_cmd_ = nh_.advertise<std_msgs::Int32MultiArray>("/sensor/motor/input/position", 10);
  }

  void spin() {
    if (csv_path_.empty()) {
      ROS_ERROR("csv_path parameter is empty");
      return;
    }

    loadCSV(csv_path_);

    ros::Rate rate(10);
    while (ros::ok()) {
      ros::spinOnce();
      rate.sleep();
      if (initialized_mag_ && initialized_motor_) break;
    }

    // 現在時刻を取得
    std::time_t t = std::time(nullptr);
    char time_str[32];
    std::strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", std::localtime(&t));

    // ファイル名を生成
    std::string filename = "spring_vs_mag_" + std::string(time_str) + ".csv";
    std::string save_path = save_dir_ + "/" + filename;

    std::ofstream ofs(save_path);
    if (!ofs.is_open()) {
      ROS_ERROR_STREAM("Failed to open " << save_path << " for writing.");
      return;
    }
    ROS_INFO_STREAM("Saving CSV to: " << save_path);

    writeCSVHeader(ofs);

    for (int step = motor_step_min_; step <= motor_step_max_; ++step) {
      std_msgs::Int32MultiArray cmd;
      cmd.data.resize(latest_motor_.data.size(), 0);
      if (motor_inverse_) {
        cmd.data[0] = initial_motor_.data[0] - step;  // 逆方向に動かす
      } else {
        cmd.data[0] = initial_motor_.data[0] + step;  // 正方向に動かす
      }
      pub_cmd_.publish(cmd);

      ros::spinOnce();
      ros::Duration(1.0).sleep();  // 必要に応じて調整

      double spring_disp = interpolate(static_cast<double>(step));
      writeCSVRow(ofs, step, spring_disp);

      ROS_INFO_STREAM("Step: " << cmd.data[0] << ", SpringDisp: " << spring_disp << ", mag: " << latest_mag_.data[mag_save_index_]);
    }

    ofs.close();
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_mag_;
  ros::Subscriber sub_pos_;
  ros::Publisher pub_cmd_;

  std::string csv_path_;
  std::vector<double> csv_motor_steps_;
  std::vector<double> csv_spring_disp_;
  std::vector<double> csv_tension_;
  int motor_step_min_;
  int motor_step_max_;

  std_msgs::Int32MultiArray latest_mag_;
  std_msgs::Int32MultiArray latest_motor_;
  std_msgs::Int32MultiArray initial_mag_;
  std_msgs::Int32MultiArray initial_motor_;
  std::deque<std_msgs::Int32MultiArray> mag_buffer_;
  int mag_save_index_;
  bool motor_inverse_;

  int mag_avg_count_;
  bool initialized_mag_;
  bool initialized_motor_;
  std::string save_dir_;

  void loadCSV(const std::string& filename) {
    ROS_INFO_STREAM("Trying to load CSV from: " << filename);

    std::ifstream file(filename);
    if (!file.is_open()) {
      ROS_ERROR_STREAM("Failed to open CSV file: " << filename);
      return;
    }

    std::string line;
    std::getline(file, line);  // skip header
    int count = 0;
    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string step_str, disp_str, tension_str;
      std::getline(ss, step_str, ',');
      std::getline(ss, disp_str, ',');
      std::getline(ss, tension_str, ',');

      if (step_str.empty() || disp_str.empty() || tension_str.empty()) {
        ROS_WARN_STREAM("Skipping invalid line: " << line);
        continue;
      }

      csv_motor_steps_.push_back(std::stod(step_str));
      csv_spring_disp_.push_back(std::stod(disp_str));
      csv_tension_.push_back(std::stod(tension_str));

      if (!csv_motor_steps_.empty()) {
        auto minmax = std::minmax_element(csv_motor_steps_.begin(), csv_motor_steps_.end());
        motor_step_min_ = static_cast<int>(*minmax.first);
        motor_step_max_ = static_cast<int>(*minmax.second);
      } else {
        motor_step_min_ = motor_step_max_ = 0;
        ROS_WARN("csv_motor_steps_ is empty. min/max set to 0.");
      }
    }
  }

  void writeCSVHeader(std::ofstream& ofs) {
    ofs << "spring_displacement";
    ofs << ",mag" << mag_save_index_;
    ofs << ",interpolated_tension";
    ofs << "\n";
  }

  void writeCSVRow(std::ofstream& ofs, double step, double spring_disp) {
    ofs << spring_disp;

    if (mag_save_index_ >= 0 && mag_save_index_ < static_cast<int>(latest_mag_.data.size())) {
      int mag_val = latest_mag_.data[mag_save_index_];
      ofs << "," << mag_val;
    } else {
      ofs << ",0";
    }

    double tension = interpolateTension(step);
    ofs << "," << tension;

    ofs << "\n";
  }

  void positionCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    latest_motor_ = *msg;

    if (!initialized_motor_) {
      initial_motor_ = *msg;
      initialized_motor_ = true;
      ROS_INFO("Motor position initialized.");
    }
  }

  void magCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    if (!initialized_mag_) {
      if (msg->data.empty()) {
        ROS_WARN("Received empty mag data. Skipping.");
        return;
      }

      mag_buffer_.push_back(*msg);
      ROS_DEBUG_STREAM("mag_buffer_ size: " << mag_buffer_.size());

      if (mag_buffer_.size() >= mag_avg_count_) {
        size_t N = msg->data.size();
        initial_mag_.data.resize(N, 0);

        for (size_t i = 0; i < N; ++i) {
          int sum = 0;
          int valid_count = 0;

          for (const auto& m : mag_buffer_) {
            if (i < m.data.size()) {
              sum += m.data[i];
              valid_count++;
            }
          }

          if (valid_count > 0) {
            initial_mag_.data[i] = sum / valid_count;
          } else {
            initial_mag_.data[i] = 0;
            ROS_WARN_STREAM("Index " << i << " has no valid samples.");
          }
        }

        initialized_mag_ = true;
        ROS_INFO_STREAM("Initial mag computed. Sample count: " << mag_avg_count_ << ", element count: " << initial_mag_.data.size());
      }
    } else {
        if (msg->data.empty()) {
          ROS_WARN("Received empty mag data. Skipping.");
          return;
        }

        // バッファに追加
        mag_buffer_.push_back(*msg);
        if (mag_buffer_.size() > static_cast<size_t>(mag_avg_count_)) {
          mag_buffer_.pop_front();
        }

        // latest_mag_ を移動平均で更新
        size_t N = msg->data.size();
        latest_mag_.data.resize(N, 0);

        for (size_t i = 0; i < N; ++i) {
          int sum = 0;
          int count = 0;

          for (const auto& m : mag_buffer_) {
            if (i < m.data.size()) {
              sum += m.data[i];
              count++;
            }
          }

          latest_mag_.data[i] = (count > 0) ? sum / count : 0;
        }

        ROS_DEBUG_STREAM("Updated latest_mag_ with moving average over " << mag_buffer_.size() << " samples.");
    }
  }

  double interpolate(double step) {
    if (csv_motor_steps_.size() < 2 || csv_spring_disp_.size() < 2) {
      ROS_ERROR_STREAM("CSV data too small for interpolation.");
      return 0.0;
    }

    for (size_t i = 0; i < csv_motor_steps_.size() - 1; ++i) {
      double s0 = csv_motor_steps_[i];
      double s1 = csv_motor_steps_[i + 1];
      double d0 = csv_spring_disp_[i];
      double d1 = csv_spring_disp_[i + 1];

      if (step >= s0 && step <= s1) {
        double t = (step - s0) / (s1 - s0);
        return d0 + t * (d1 - d0);
      }
    }

    ROS_WARN_STREAM("Step " << step << " outside CSV range.");
    return 0.0;
  }

  double interpolateTension(double step) {
    if (csv_motor_steps_.size() < 2 || csv_tension_.size() < 2) {
      ROS_ERROR_STREAM("CSV data too small for tension interpolation.");
      return 0.0;
    }

    for (size_t i = 0; i < csv_motor_steps_.size() - 1; ++i) {
      double s0 = csv_motor_steps_[i];
      double s1 = csv_motor_steps_[i + 1];
      double t0 = csv_tension_[i];
      double t1 = csv_tension_[i + 1];

      if (step >= s0 && step <= s1) {
        double t = (step - s0) / (s1 - s0);
        return t0 + t * (t1 - t0);
      }
    }

    ROS_WARN_STREAM("Step " << step << " outside tension range.");
    return 0.0;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mag_calibration_node");
  MagCalibrationNode node;
  node.spin();
  return 0;
}

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <deque>

class ZeroPointCalibrationNode {
public:
  ZeroPointCalibrationNode(ros::NodeHandle& nh) : nh_(nh),
    initialized_mag_(false),
    initialized_motor_(false)
  {
    sub_mag_ = nh_.subscribe("/sensor/mag", 1, &ZeroPointCalibrationNode::magCallback, this);
    sub_pos_ = nh_.subscribe("/sensor/motor/output/position", 1, &ZeroPointCalibrationNode::positionCallback, this);
    pub_cmd_ = nh_.advertise<std_msgs::Int32MultiArray>("/sensor/motor/input/position", 10);

    nh_.param<std::string>("csv_path", csv_path_, "calibration_table.csv");
    nh_.param<int>("mag_avg_count", mag_avg_count_, 10);
    nh_.param<int>("mag_use_index", mag_save_index_, 0);


    loadCSV(csv_path_);
  }

  void spin() {
    ROS_INFO("spin0");
    ros::Rate rate(10);
    while (ros::ok()) {
      ros::spinOnce();
      rate.sleep();

      if (initialized_mag_ && initialized_motor_) break;
    }

    ROS_INFO("All initializations completed. Starting data collection...");

    std::ofstream ofs("spring_vs_mag.csv");
    ofs << "spring_displacement";
    for (size_t i = 0; i < latest_mag_.data.size(); ++i) {
      ofs << ",mag" << i;
    }
    ofs << "\n";

    ros::Rate loop_rate(10);
    while (ros::ok()) {
      ros::spinOnce();
      if (!latest_motor_.data.empty() && !initial_motor_.data.empty()) {
        // 相対ステップ角（初期値との差）
        double step = static_cast<double>(latest_motor_.data[0] - initial_motor_.data[0]);
        double spring_disp = interpolate(step);
        ofs << spring_disp;

        if (mag_save_index_ >= 0 && mag_save_index_ < static_cast<int>(latest_mag_.data.size())) {
          ofs << "," << latest_mag_.data[mag_save_index_];
        } else {
          ROS_WARN("mag_save_index %d out of range, writing 0.", mag_save_index_);
          ofs << ",0";
        }
        ofs << "\n";
        ROS_INFO_STREAM("Saved: step=" << step << ", spring=" << spring_disp);
        break;
      }
      loop_rate.sleep();
    }

    ROS_INFO("Data saved to spring_vs_mag.csv");
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_mag_;
  ros::Subscriber sub_pos_;
  ros::Publisher pub_cmd_;

  std::string csv_path_;
  std::vector<double> csv_motor_steps_;
  std::vector<double> csv_spring_disp_;

  std_msgs::Int32MultiArray latest_mag_;
  std_msgs::Int32MultiArray latest_motor_;
  std_msgs::Int32MultiArray initial_mag_;
  std_msgs::Int32MultiArray initial_motor_;
  std::deque<std_msgs::Int32MultiArray> mag_buffer_;
  int mag_save_index_;

  int mag_avg_count_;
  bool initialized_mag_;
  bool initialized_motor_;

  void loadCSV(const std::string& filename) {
    std::ifstream file(filename);
    std::string line;
    std::getline(file, line);  // skip header

    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string step_str, disp_str;
      std::getline(ss, step_str, ',');
      std::getline(ss, disp_str, ',');

      csv_motor_steps_.push_back(std::stod(step_str));
      csv_spring_disp_.push_back(std::stod(disp_str));
    }

    ROS_INFO("Loaded CSV with %lu entries.", csv_motor_steps_.size());
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
    latest_mag_ = *msg;
    ROS_DEBUG_STREAM("Received mag data: size = " << msg->data.size());

    if (!initialized_mag_) {
      if (msg->data.empty()) {
        ROS_WARN("Received empty mag data. Skipping.");
        return;
      }

      mag_buffer_.push_back(*msg);
      ROS_DEBUG_STREAM("mag_buffer_ size: " << mag_buffer_.size());

      // バッファが溜まったら初期化
      if (mag_buffer_.size() >= mag_avg_count_) {
        size_t N = msg->data.size();
        initial_mag_.data.resize(N, 0);

        for (size_t i = 0; i < N; ++i) {
          int valid_count = 0;
          int sum = 0;

          for (size_t j = 0; j < mag_buffer_.size(); ++j) {
            const auto& m = mag_buffer_[j];
            if (i < m.data.size()) {
              sum += m.data[i];
              valid_count++;
            } else {
              ROS_WARN_STREAM("mag_buffer_[" << j << "] has insufficient size: " << m.data.size() << " < " << i + 1);
            }
          }

          if (valid_count > 0) {
            initial_mag_.data[i] = sum / valid_count;
          } else {
            ROS_ERROR_STREAM("Index " << i << " had no valid samples. Setting to 0.");
            initial_mag_.data[i] = 0;
          }
        }

        initialized_mag_ = true;
        ROS_INFO_STREAM("Initial mag computed. Sample count: " << mag_avg_count_ << ", element count: " << initial_mag_.data.size());
      }
    }
  }


  double interpolate(double step) {
    for (size_t i = 0; i < csv_motor_steps_.size() - 1; ++i) {
      if (step >= csv_motor_steps_[i] && step <= csv_motor_steps_[i + 1]) {
        double t = (step - csv_motor_steps_[i]) / (csv_motor_steps_[i + 1] - csv_motor_steps_[i]);
        return csv_spring_disp_[i] + t * (csv_spring_disp_[i + 1] - csv_spring_disp_[i]);
      }
    }
    return 0.0;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mag_calibration_node");
  ros::NodeHandle nh;

  ZeroPointCalibrationNode node(nh);
  node.spin();

  return 0;
}

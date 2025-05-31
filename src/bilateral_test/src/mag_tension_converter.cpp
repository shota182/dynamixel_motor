#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iostream>


class MagTensionConverter {
public:
  MagTensionConverter() : nh_("~"), initialized_mag_(false) {
    sub_mag_ = nh_.subscribe("/sensor/mag", 1, &MagTensionConverter::magCallback, this);
    pub_tension_ = nh_.advertise<std_msgs::Float64MultiArray>("/force/input/tension", 10);

    // csv_paths_sm を取得
    std::vector<std::string> csv_paths_sm;
    if (nh_.getParam("csv_paths_sm", csv_paths_sm)) {
      ROS_INFO_STREAM("CSV paths (sm) received. Count: " << csv_paths_sm.size());
      loadMagTensionTables(csv_paths_sm, mag_tables_sm_, true);
    } else {
      ROS_WARN("No csv_paths_sm parameter found. All tension will default to -1.");
      mag_tables_sm_.clear();  // 念のため空にしておく
    }

    // csv_paths_st を取得
    std::vector<std::string> csv_paths_st;
    if (nh_.getParam("csv_paths_st", csv_paths_st)) {
      ROS_INFO_STREAM("CSV paths (st) received. Count: " << csv_paths_st.size());
      loadMagTensionTables(csv_paths_st, mag_tables_st_, false);
    } else {
      ROS_WARN("No csv_paths_st parameter found. All tension will default to -1.");
      mag_tables_st_.clear();  // 念のため空にしておく
    }
  }

  struct MagTensionTable {
    std::vector<double> mag2;  // mag2 を double 型に変更
    std::vector<double> tension;
  };

  void magCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    if (msg->data.empty()) {
      ROS_WARN("Received empty mag data. Skipping.");
      return;
    }

    if (!initialized_mag_) {
      initial_mag_ = *msg;

      // 初期値から spring_displacement を取得
      for (size_t i = 0; i < initial_mag_.data.size(); ++i) {
        if (i < mag_tables_sm_.size() && !mag_tables_sm_[i].mag2.empty()) {
          initial_spring_displacement_.push_back(interpolateSpringDisplacement(mag_tables_sm_[i], initial_mag_.data[i]));
        } else {
          initial_spring_displacement_.push_back(0.0);  // 初期値が取得できない場合は 0.0
        }
      }

      initialized_mag_ = true;
      ROS_INFO("Initial mag and spring displacement saved.");
      return;
    }

    latest_mag_ = *msg;
  }

  void run() {
    ros::Rate rate(100);  // 任意の周期

    while (ros::ok()) {
        ros::spinOnce();

        if (!initialized_mag_) {
            rate.sleep();
            continue;
        }

        std_msgs::Float64MultiArray tension_msg;
        size_t N = latest_mag_.data.size();
        tension_msg.data.resize(N);

        for (size_t i = 0; i < N; ++i) {
            // 条件1: 対応する CSV テーブルが存在しない
            if (i >= mag_tables_sm_.size() || i >= mag_tables_st_.size()) {
                tension_msg.data[i] = -1;
                continue;
            }

            const auto& sm_table = mag_tables_sm_[i];
            const auto& st_table = mag_tables_st_[i];

            // 条件2: テーブルが空（読み込み失敗または無効）
            if (sm_table.mag2.empty() || st_table.mag2.empty()) {
                tension_msg.data[i] = -1;
                continue;
            }

            // 相対変化を計算
            double current_spring_displacement = interpolateSpringDisplacement(sm_table, latest_mag_.data[i]);
            double relative_change = current_spring_displacement - initial_spring_displacement_[i];

            // st の spring_displacement に相対変化を当てはめて tension を取得
            double tension = interpolateTension(st_table, relative_change);
            tension_msg.data[i] = tension;
        }

        pub_tension_.publish(tension_msg);
        rate.sleep();
    }
  }

  void loadMagTensionTables(const std::vector<std::string>& paths, std::vector<MagTensionTable>& mag_tables, bool is_sm) {
    mag_tables.clear();

    for (const auto& path : paths) {
        std::ifstream file(path);
        MagTensionTable table;

        if (!file.is_open()) {
            ROS_WARN_STREAM("Failed to open CSV file: " << path);
            mag_tables.push_back(table);  // 空テーブル
            continue;
        }

        std::string line;
        if (!std::getline(file, line)) {
            ROS_WARN_STREAM("Empty or malformed CSV file: " << path);
            mag_tables.push_back(table);
            continue;
        }

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string val;
            std::vector<std::string> tokens;
            while (std::getline(ss, val, ',')) {
                tokens.push_back(val);
            }

            try {
                if (is_sm && tokens.size() == 2) {  // sm形式の場合
                    int mag2 = std::stoi(tokens[0]);
                    double spring_displacement = std::stod(tokens[1]);
                    table.mag2.push_back(mag2);
                    table.tension.push_back(spring_displacement);  // spring_displacement を tension に保存
                } else if (!is_sm && tokens.size() == 3) {  // st形式の場合
                    double spring_displacement = std::stod(tokens[1]);
                    double tension = std::stod(tokens[2]);
                    table.mag2.push_back(tension);  // tension を保存
                    table.tension.push_back(spring_displacement);  // spring_displacement を保存
                }
            } catch (const std::exception& e) {
                ROS_WARN_STREAM("Failed to parse line: " << line << " → " << e.what());
            }
        }

        mag_tables.push_back(table);
    }
}

double interpolateTension(const MagTensionTable& table, double relative_change) {
    const auto& spring_displacement = table.tension;  // 正しく spring_displacement を参照
    const auto& tension = table.mag2;  // 正しく tension を参照

    if (spring_displacement.empty()) {
        ROS_WARN("Spring displacement table is empty. Returning 0.0.");
        return 0.0;
    }

    if (relative_change <= spring_displacement.front()) {
        return tension.front();
    }
    if (relative_change >= spring_displacement.back()) {
        return tension.back();
    }

    for (size_t i = 0; i < spring_displacement.size() - 1; ++i) {
        if (relative_change >= spring_displacement[i] && relative_change <= spring_displacement[i + 1]) {
            double ratio = (relative_change - spring_displacement[i]) / 
                           (spring_displacement[i + 1] - spring_displacement[i]);
            double interpolated_tension = tension[i] + ratio * (tension[i + 1] - tension[i]);
            return interpolated_tension;
        }
    }

    ROS_WARN_STREAM("Relative change (" << relative_change << ") did not match any spring displacement range. Returning 0.0.");
    return 0.0;
}

double interpolateSpringDisplacement(const MagTensionTable& table, int mag_value) {
    const auto& mag2 = table.mag2;
    const auto& tension = table.tension;

    if (mag2.empty()) return 0.0;
    if (mag_value <= mag2.front()) return tension.front();
    if (mag_value >= mag2.back()) return tension.back();

    for (size_t i = 0; i < mag2.size() - 1; ++i) {
      if (mag_value >= mag2[i] && mag_value <= mag2[i + 1]) {
        double ratio = (mag_value - mag2[i]) / static_cast<double>(mag2[i + 1] - mag2[i]);
        return tension[i] + ratio * (tension[i + 1] - tension[i]);
      }
    }

    return 0.0;
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_mag_;
  ros::Publisher pub_tension_;
  bool initialized_mag_;
  std_msgs::Int32MultiArray initial_mag_;
  std_msgs::Int32MultiArray latest_mag_;
  std::vector<double> initial_spring_displacement_;  // 初期 spring_displacement を保存
  std::vector<MagTensionTable> mag_tables_sm_;
  std::vector<MagTensionTable> mag_tables_st_;
  std::vector<MagTensionTable> mag_tables_;


};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mag_tension_converter");
  MagTensionConverter converter;
  converter.run();
  return 0;
}

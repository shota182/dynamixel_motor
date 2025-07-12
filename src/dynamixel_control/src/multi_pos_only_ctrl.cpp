#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

/* -------- Control Table (Protocol 2.0) ---------- */
constexpr uint16_t ADDR_OPERATING_MODE   = 11;
constexpr uint16_t ADDR_TORQUE_ENABLE    = 64;
constexpr uint16_t ADDR_GOAL_POSITION    = 116;
constexpr uint8_t  LEN_GOAL_POSITION     = 4;
constexpr uint16_t ADDR_PRESENT_POSITION = 132;  // 4 byte
constexpr uint8_t  LEN_PRESENT_POSITION  = 4;

constexpr uint8_t  MODE_EXTENDED_POSITION_CONTROL = 4;
constexpr uint8_t  TORQUE_ENABLE         = 1;
/* ------------------------------------------------ */

/* ---- 追加する定数 ---- */
constexpr uint16_t ADDR_POS_D_GAIN = 80;   // 2 byte
constexpr uint16_t ADDR_POS_I_GAIN = 82;   // 2 byte
constexpr uint16_t ADDR_POS_P_GAIN = 84;   // 2 byte
constexpr uint16_t ADDR_PROFILE_ACCELERATION = 108; // 4 byte
constexpr uint16_t ADDR_PROFILE_VELOCITY = 112;    // 4 byte

class XL330PositionOnly
{
public:
  explicit XL330PositionOnly(ros::NodeHandle& nh) : nh_(nh)
  {
    /* ---- パラメータ ---- */
    nh_.param("device_name", device_, std::string("/dev/ttyUSB0"));
    nh_.param("baud_rate", baud_, 57600);
    nh_.param("motor_count", motor_cnt_, 2);
    nh_.param("freq", freq_, 200.0);
    nh_.param("goal_topic", goal_topic_, std::string("/sensor/motor/input/position"));
    nh_.param("pos_topic", pos_topic_, std::string("/sensor/motor/output/position"));

    /* ---- SDK ---- */
    port_   = dynamixel::PortHandler::getPortHandler(device_.c_str());
    packet_ = dynamixel::PacketHandler::getPacketHandler(2.0);
    if (!port_->openPort()) {
      ROS_FATAL("Failed to open port: %s. Please check the device connection and permissions.", device_.c_str());
      throw std::runtime_error("Port open failed");
    }
    if (!port_->setBaudRate(baud_)) {
      ROS_FATAL("Failed to set baud rate: %d. Please check the device capabilities.", baud_);
      throw std::runtime_error("Baud rate set failed");
    }

    initMotors();

    /* ---- SyncWrite / SyncRead ---- */
    sw_goal_ = new dynamixel::GroupSyncWrite(port_, packet_, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);

    sr_pos_ = new dynamixel::GroupSyncRead(port_, packet_, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);

    for (int id = 1; id <= motor_cnt_; ++id)
    {
      sr_pos_->addParam(id);
    }

    /* ---- ROS 通信 ---- */
    sub_goal_ = nh_.subscribe(goal_topic_, 10, &XL330PositionOnly::goalCB, this);
    pub_pos_  = nh_.advertise<std_msgs::Int32MultiArray>(pos_topic_, 10);
    timer_ = nh_.createTimer(ros::Duration(1.0 / freq_), &XL330PositionOnly::timerCB, this);

    ROS_INFO("XL330 position-only ready (motors=%d, %.0f Hz)", motor_cnt_, freq_);
  }

  ~XL330PositionOnly()
  {
    disableTorque();
    delete sw_goal_;  delete sr_pos_;
    port_->closePort();
  }

private:
  /* ------------- 初期化 ------------- */
  void initMotors()
  {
    int kp, ki, kd; // 一時的に int 型を使用
    nh_.param("kp", kp, 400); // デフォルト値を int 型で指定
    nh_.param("ki", ki, 0);
    nh_.param("kd", kd, 0);

    // uint16_t 型にキャストして設定
    setGains(static_cast<uint16_t>(kp), static_cast<uint16_t>(ki), static_cast<uint16_t>(kd));
    ROS_INFO("All motors gains set: P=%d, I=%d, D=%d", kp, ki, kd);

    // Profile Acceleration / Velocity をパラメータから取得
    int profile_acceleration, profile_velocity;
    nh_.param("profile_acceleration", profile_acceleration, 0); // デフォルト値 0
    nh_.param("profile_velocity", profile_velocity, 0);         // デフォルト値 0

    for (int id = 1; id <= motor_cnt_; ++id)
    {
        uint8_t err = 0;

        // トルクを有効化
        packet_->write1ByteTxRx(port_, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &err);
        if (err) ROS_WARN("Torque enable error ID %d (err=%d)", id, err);

        // Profile Acceleration を設定
        packet_->write4ByteTxRx(port_, id, ADDR_PROFILE_ACCELERATION, static_cast<uint32_t>(profile_acceleration), &err);
        if (err) ROS_WARN("Profile Acceleration set error ID %d (err=%d)", id, err);

        // Profile Velocity を設定
        packet_->write4ByteTxRx(port_, id, ADDR_PROFILE_VELOCITY, static_cast<uint32_t>(profile_velocity), &err);
        if (err) ROS_WARN("Profile Velocity set error ID %d (err=%d)", id, err);
    }

    ROS_INFO("Torque enabled and Profile Acceleration/Velocity set for all motors.");
  }
  void disableTorque()
  {
    for (int id = 1; id <= motor_cnt_; ++id)
      packet_->write1ByteTxRx(port_, id, ADDR_TORQUE_ENABLE, 0);
  }

  void setGains(uint16_t kp, uint16_t ki, uint16_t kd)
  {
    for (int id = 1; id <= motor_cnt_; ++id)
    {
      uint8_t err = 0;
      packet_->write2ByteTxRx(port_, id, ADDR_POS_P_GAIN, kp, &err);
      packet_->write2ByteTxRx(port_, id, ADDR_POS_I_GAIN, ki, &err);
      packet_->write2ByteTxRx(port_, id, ADDR_POS_D_GAIN, kd, &err);
      if (err) ROS_WARN("Gain set error ID %d (err=%d)", id, err);
    }
  }

  /* ------ goal_positions callback ------ */
  void goalCB(const std_msgs::Int32MultiArray::ConstPtr& msg)
  {
    if (msg->data.size() != static_cast<size_t>(motor_cnt_))
    {
      ROS_WARN_THROTTLE(1.0, "goal len %zu != motor_count %d → ignore", msg->data.size(), motor_cnt_);
      return;
    }
    sw_goal_->clearParam();
    for (int i = 0; i < motor_cnt_; ++i)
    {
      uint8_t param[LEN_GOAL_POSITION];
      int32_t v = msg->data[i];
      param[0] = DXL_LOBYTE(DXL_LOWORD(v));
      param[1] = DXL_HIBYTE(DXL_LOWORD(v));
      param[2] = DXL_LOBYTE(DXL_HIWORD(v));
      param[3] = DXL_HIBYTE(DXL_HIWORD(v));
      sw_goal_->addParam(i + 1, param);
    }
    if (sw_goal_->txPacket() != COMM_SUCCESS)
      ROS_ERROR_THROTTLE(1.0, "SyncWrite GoalPosition fail");
  }

  /* ------ timer: SyncRead + publish ------ */
  void timerCB(const ros::TimerEvent&)
  {
    if (sr_pos_->txRxPacket() != COMM_SUCCESS)
    {
      ROS_ERROR_THROTTLE(1.0, "SyncRead fail");
      return;
    }

    std_msgs::Int32MultiArray pos_msg;
    pos_msg.data.resize(motor_cnt_);

    for (int i = 0; i < motor_cnt_; ++i)
    {
      int id = i + 1;

      // ---- position (4 byte) ----
      int32_t raw_pos = static_cast<int32_t>(sr_pos_->getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION));
      pos_msg.data[i] = raw_pos;                 // 0‑4095
    }
    pub_pos_.publish(pos_msg);
  }

  /* ---------- members ---------- */
  ros::NodeHandle nh_;
  ros::Subscriber sub_goal_;
  ros::Publisher  pub_pos_;
  ros::Timer      timer_;

  std::string device_;
  int baud_, motor_cnt_;
  double freq_;
  std::string goal_topic_, pos_topic_;

  dynamixel::PortHandler*  port_;
  dynamixel::PacketHandler* packet_;
  dynamixel::GroupSyncWrite* sw_goal_;
  dynamixel::GroupSyncRead*  sr_pos_;
};

/* ------------ main ------------ */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_pos_only_ctrl");
  ros::NodeHandle nh("~");
  XL330PositionOnly node(nh);
  ros::spin();
  return 0;
}
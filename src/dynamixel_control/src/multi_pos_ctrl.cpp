#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

/* -------- Control Table (Protocol 2.0) ---------- */
constexpr uint16_t ADDR_OPERATING_MODE   = 11;
constexpr uint16_t ADDR_TORQUE_ENABLE    = 64;
constexpr uint16_t ADDR_GOAL_POSITION    = 116;
constexpr uint8_t  LEN_GOAL_POSITION     = 4;
constexpr uint16_t ADDR_PRESENT_CURRENT  = 126;  // 2 byte
constexpr uint8_t  LEN_PRESENT_CURRENT   = 2;
constexpr uint16_t ADDR_PRESENT_POSITION = 132;  // 4 byte
constexpr uint8_t  LEN_PRESENT_POSITION  = 4;

constexpr uint8_t  MODE_EXTENDED_POSITION_CONTROL = 4;
constexpr uint8_t  TORQUE_ENABLE         = 1;
/* ------------------------------------------------ */

class XL330MultiIO
{
public:
  explicit XL330MultiIO(ros::NodeHandle& nh) : nh_(nh)
  {
    /* ---- パラメータ ---- */
    nh_.param("device_name", device_, std::string("/dev/ttyUSB0"));
    nh_.param("baud_rate",   baud_,   57600);
    nh_.param("motor_count", motor_cnt_, 3);
    nh_.param("freq",        freq_,   50.0);

    /* ---- SDK ---- */
    port_   = dynamixel::PortHandler::getPortHandler(device_.c_str());
    packet_ = dynamixel::PacketHandler::getPacketHandler(2.0);
    if (!port_->openPort())            ROS_FATAL("openPort fail");
    if (!port_->setBaudRate(baud_))    ROS_FATAL("setBaud fail");

    initMotors();

    /* ---- SyncWrite / SyncRead ---- */
    sw_goal_ = new dynamixel::GroupSyncWrite(
        port_, packet_, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);

    sr_cur_ = new dynamixel::GroupSyncRead(
        port_, packet_, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT);
    sr_pos_ = new dynamixel::GroupSyncRead(
        port_, packet_, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);

    for (int id = 1; id <= motor_cnt_; ++id)
    {
      sr_cur_->addParam(id);
      sr_pos_->addParam(id);
    }

    /* ---- ROS 通信 ---- */
    sub_goal_ = nh_.subscribe("/goal_positions", 10,
                              &XL330MultiIO::goalCB, this);
    pub_pos_  = nh_.advertise<std_msgs::Int32MultiArray>(
                    "/present_positions", 10);
    pub_cur_  = nh_.advertise<std_msgs::Int32MultiArray>(
                    "/present_currents", 10);

    timer_ = nh_.createTimer(ros::Duration(1.0 / freq_),
                             &XL330MultiIO::timerCB, this);

    ROS_INFO("XL330 multi‑IO ready (motors=%d, %.0f Hz)", motor_cnt_, freq_);
  }

  ~XL330MultiIO()
  {
    disableTorque();
    delete sw_goal_;  delete sr_cur_;  delete sr_pos_;
    port_->closePort();
  }

private:
  /* ------------- 初期化 ------------- */
  void initMotors()
  {
    for (int id = 1; id <= motor_cnt_; ++id)
    {
      uint8_t err = 0;
      packet_->write1ByteTxRx(port_, id, ADDR_OPERATING_MODE,
                              MODE_EXTENDED_POSITION_CONTROL, &err);
      packet_->write1ByteTxRx(port_, id, ADDR_TORQUE_ENABLE,
                              TORQUE_ENABLE, &err);
      if (err) ROS_WARN("Init error ID %d (err=%d)", id, err);
    }
  }
  void disableTorque()
  {
    for (int id = 1; id <= motor_cnt_; ++id)
      packet_->write1ByteTxRx(port_, id, ADDR_TORQUE_ENABLE, 0);
  }

  /* ------ goal_positions callback ------ */
  void goalCB(const std_msgs::Int32MultiArray::ConstPtr& msg)
  {
    if (msg->data.size() != static_cast<size_t>(motor_cnt_))
    {
      ROS_WARN_THROTTLE(1.0,
        "goal len %zu != motor_count %d → ignore", msg->data.size(), motor_cnt_);
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
    if (sr_cur_->txRxPacket() != COMM_SUCCESS ||
        sr_pos_->txRxPacket() != COMM_SUCCESS)
    {
      ROS_ERROR_THROTTLE(1.0, "SyncRead fail");
      return;
    }

    std_msgs::Int32MultiArray pos_msg, cur_msg;
    pos_msg.data.resize(motor_cnt_);
    cur_msg.data.resize(motor_cnt_);

    for (int i = 0; i < motor_cnt_; ++i)
    {
      int id = i + 1;

      // ---- current (2 byte, signed) ----
      int16_t raw_cur = static_cast<int16_t>(
        sr_cur_->getData(id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT));
      cur_msg.data[i] = raw_cur;                 // raw [LSB] (≒2.69 mA)

      // ---- position (4 byte) ----
      int32_t raw_pos = static_cast<int32_t>(
        sr_pos_->getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION));
      pos_msg.data[i] = raw_pos;                 // 0‑4095
    }
    pub_cur_.publish(cur_msg);
    pub_pos_.publish(pos_msg);
  }

  /* ---------- members ---------- */
  ros::NodeHandle nh_;
  ros::Subscriber sub_goal_;
  ros::Publisher  pub_pos_, pub_cur_;
  ros::Timer      timer_;

  std::string device_;
  int baud_, motor_cnt_;
  double freq_;

  dynamixel::PortHandler*  port_;
  dynamixel::PacketHandler* packet_;
  dynamixel::GroupSyncWrite* sw_goal_;
  dynamixel::GroupSyncRead*  sr_cur_;
  dynamixel::GroupSyncRead*  sr_pos_;
};

/* ------------ main ------------ */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "xl330_multi_io");
  ros::NodeHandle nh("~");
  XL330MultiIO node(nh);
  ros::spin();
  return 0;
}

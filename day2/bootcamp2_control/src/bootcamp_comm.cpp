#include <bootcamp2_control/bootcamp_control.h>

class TocabiController : public rclcpp::Node
{
  public:
    TocabiController() : Node("tocabi_controller")
    {
      motor_thread_ = std::thread(&TocabiController::motor, this);
      motor_thread_.join();
    }

    ~TocabiController()
    {
      RCLCPP_INFO(this->get_logger(), "Torque Mode off!");
      motorTorqueOff(portHandler, packetHandler);
      RCLCPP_INFO(this->get_logger(), "close the port!");
      portHandler->closePort();
    }

  private:
    std::thread motor_thread_;
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;


    void motor()
    {
      rclcpp::Rate loop_rate(Hz);

      // Dynamixel
      portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
      packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
      dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
      dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_RESENT_POSITION);

      if(!serialPortCheck(portHandler)) return;
      motorIDSet();
      // motorTorqueOn(portHandler, packetHandler);
      if(!storagePosition(&groupSyncRead)) return;

      while (rclcpp::ok())
      {
        readPosition(&groupSyncRead, packetHandler);
        if(!checkData(&groupSyncRead)) return;
        getPosition(&groupSyncRead);

        for (int i = 0; i < motorID.size(); i++)
        {
          printf("motor ID: %d position [deg]: %lf\n", motorID[i], present_postiion[i]*RAD2DEG);
        }

        loop_rate.sleep();
      }
      motorTorqueOff(portHandler, packetHandler);
    }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TocabiController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
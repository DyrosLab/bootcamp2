#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "dynamixel_sdk/dynamixel_sdk.h"

/********* DYNAMIXEL Model definition *********
***** (Use only one definition at a time) *****/
#define X_SERIES // X330, X430, X540, 2X430
// #define PRO_SERIES // H54, H42, M54, M42, L54, L42
// #define PRO_A_SERIES // PRO series with (A) firmware update.
// #define P_SERIES  // PH54, PH42, PM54
// #define XL320  // [WARNING] Operating Voltage : 7.4V
// #define MX_SERIES // MX series with 2.0 firmware update.
// Control table address
#if defined(X_SERIES) || defined(MX_SERIES)
  #define ADDR_TORQUE_ENABLE          64
  #define ADDR_GOAL_POSITION          116
  #define ADDR_PRESENT_POSITION       132
  #define MINIMUM_POSITION_LIMIT      0  // Refer to the Minimum Position Limit of product eManual
  #define MAXIMUM_POSITION_LIMIT      4095  // Refer to the Maximum Position Limit of product eManual
  #define BAUDRATE                    1000000
#elif defined(PRO_SERIES)
  #define ADDR_TORQUE_ENABLE          562  // Control table address is different in DYNAMIXEL model
  #define ADDR_GOAL_POSITION          596
  #define ADDR_PRESENT_POSITION       611
  #define MINIMUM_POSITION_LIMIT      -150000  // Refer to the Minimum Position Limit of product eManual
  #define MAXIMUM_POSITION_LIMIT      150000  // Refer to the Maximum Position Limit of product eManual
  #define BAUDRATE                    57600
#elif defined(P_SERIES) ||defined(PRO_A_SERIES)
  #define ADDR_TORQUE_ENABLE          512  // Control table address is different in DYNAMIXEL model
  #define ADDR_GOAL_POSITION          564
  #define ADDR_PRESENT_POSITION       580
  #define MINIMUM_POSITION_LIMIT      -150000  // Refer to the Minimum Position Limit of product eManual
  #define MAXIMUM_POSITION_LIMIT      150000  // Refer to the Maximum Position Limit of product eManual
  #define BAUDRATE                    57600
#elif defined(XL320)
  #define ADDR_TORQUE_ENABLE          24
  #define ADDR_GOAL_POSITION          30
  #define ADDR_PRESENT_POSITION       37
  #define MINIMUM_POSITION_LIMIT      0  // Refer to the CW Angle Limit of product eManual
  #define MAXIMUM_POSITION_LIMIT      1023  // Refer to the CCW Angle Limit of product eManual
  #define BAUDRATE                    1000000  // Default Baudrate of XL-320 is 1Mbps
#endif

// DYNAMIXEL Protocol Version (1.0 / 2.0)
// https://emanual.robotis.com/docs/en/dxl/protocol2/
#define PROTOCOL_VERSION  2.0

// Use the actual port assigned to the U2D2.
// ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
#define DEVICENAME  "/dev/ttyUSB0"

// Factory default ID of all DYNAMIXEL is 1
#define DXL1_ID 1
#define DXL2_ID 2

// Data Byte Length
#define LEN_GOAL_POSITION 4  
#define LEN_RESENT_POSITION 4

#define TORQUE_ENABLE 1  // Value for enabling the torque
#define TORQUE_DISABLE 0 // Value for disabling the torque

#define DEG2RAD (0.01745329251994329576923690768489)
#define RAD2DEG (1 / DEG2RAD)

int dxl_comm_result = COMM_TX_FAIL; // Communication result
bool dxl_addparam_result = false;   // addParam result
bool dxl_getdata_result = false;    // GetParam result
uint8_t dxl_error = 0;              // Dynamixel error
int32_t dxl_present_position = 0;
int dxl_goal_position = 0;
uint8_t param_goal_position[4];
double present_postiion[2];

std::vector<int> motorID;
double pulse2deg = 0.087890625; // 360deg/4096pulse
double Hz = 100;

bool serialPortCheck(dynamixel::PortHandler *portHandler)
{
    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        return false;
    }

    if (portHandler->setBaudRate(BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
        return true;
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        return false;
    }
}

void motorIDSet()
{
    motorID.push_back(DXL1_ID);
    motorID.push_back(DXL2_ID);
}

void motorTorqueOn(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler)
{
    for (int i = 0; i < motorID.size(); i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, motorID[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            printf("Dynamixel#%d has been successfully Torque On! \n", motorID[i]);
        }
    }
}

void motorTorqueOff(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler)
{
    for (int i = 0; i < motorID.size(); i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, motorID[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            printf("Dynamixel#%d has been successfully Torque Off! \n", motorID[i]);
        }
    }
}

bool storagePosition(dynamixel::GroupSyncRead *groupSyncRead)    
{
    for (int i = 0; i < motorID.size(); i++)
    {   
        dxl_addparam_result = groupSyncRead->addParam(motorID[i]);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", motorID[i]);
            return false;
        }
    }
    return true;
}

void readPosition(dynamixel::GroupSyncRead *groupSyncRead, dynamixel::PacketHandler *packetHandler)
{
    
    dxl_comm_result = groupSyncRead->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }

    for (int i = 0; i < motorID.size(); i++)
    {
        if (groupSyncRead->getError(motorID[i], &dxl_error))
        {
            printf("[ID:%03d] %s\n", motorID[i], packetHandler->getRxPacketError(dxl_error));
        }
    }
}

bool checkData(dynamixel::GroupSyncRead *groupSyncRead)
{
    for (int i = 0; i < motorID.size(); i++)
    {   
        dxl_getdata_result = groupSyncRead->isAvailable(motorID[i], ADDR_PRESENT_POSITION, LEN_RESENT_POSITION);
        if (dxl_getdata_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", motorID[i]);
            return false;
        }
    }
    return true;
}

void getPosition(dynamixel::GroupSyncRead *groupSyncRead)
{
    for (int i = 0; i < motorID.size(); i++)
    {
        dxl_present_position = groupSyncRead->getData(motorID[i], ADDR_PRESENT_POSITION, LEN_RESENT_POSITION);
        present_postiion[i] = (double)dxl_present_position * pulse2deg * DEG2RAD;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Rate loop_rate(Hz);

    auto node = std::make_shared<rclcpp::Node>("bootcamp_dxl_sync");

    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;

    // Dynamixel
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_RESENT_POSITION);

    if(!serialPortCheck(portHandler)) return -1;
    motorIDSet();
    motorTorqueOn(portHandler, packetHandler);
    if(!storagePosition(&groupSyncRead)) return -1;

    // center tick
    readPosition(&groupSyncRead, packetHandler);
    checkData(&groupSyncRead);
    getPosition(&groupSyncRead);
    std::vector<int32_t> home_ticks(motorID.size());
    for (size_t i = 0; i < motorID.size(); i++)
    {
        home_ticks[i] = groupSyncRead.getData(motorID[i], ADDR_PRESENT_POSITION, LEN_RESENT_POSITION);
        printf("Motor ID: %d Home Tick: %d (%.2f deg)\n", motorID[i], home_ticks[i], present_postiion[i] * RAD2DEG);
    }

    const int amplitude_tick = (int)(30.0 / pulse2deg);  // 30 deg -> 341 tick
    int direction = 1;
    int step_tick = 1; // 10 tick ≈ 0.88 deg

    while (rclcpp::ok())
    {
        readPosition(&groupSyncRead, packetHandler);
        if(!checkData(&groupSyncRead)) return -1;
        getPosition(&groupSyncRead);

        for (int i = 0; i < motorID.size(); i++)
        {
            printf("motor ID: %d position [deg]: %lf\n", motorID[i], present_postiion[i]*RAD2DEG);
        }

        for (size_t i = 0; i < motorID.size(); i++)
        {
            static int goal_tick_offset = 0;
            goal_tick_offset += direction * step_tick;

            if (goal_tick_offset >= amplitude_tick) { goal_tick_offset = amplitude_tick; direction = -1; }
            else if (goal_tick_offset <= -amplitude_tick) { goal_tick_offset = -amplitude_tick; direction = 1; }

            int32_t goal_tick = home_ticks[i] + goal_tick_offset;

            uint8_t param_goal_position[4];
            param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal_tick));
            param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal_tick));
            param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal_tick));
            param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal_tick));

            dxl_addparam_result = groupSyncWrite.addParam(motorID[i], param_goal_position);
            if (!dxl_addparam_result)
            {
                fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed\n", motorID[i]);
            }
        }

        dxl_comm_result = groupSyncWrite.txPacket();
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        groupSyncWrite.clearParam();

        loop_rate.sleep();
    }
    motorTorqueOff(portHandler, packetHandler);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
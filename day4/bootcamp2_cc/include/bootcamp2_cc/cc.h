#include <iostream>
#include <stdio.h>
#include <vector>
#include <thread>
#include <mutex>
#include "dynamixel_sdk/dynamixel_sdk.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <string>

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>


#define EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, Size, SizeSuffix)      \
    typedef Matrix<Type, Size, Size> Matrix##SizeSuffix##TypeSuffix; \
    typedef Matrix<Type, Size, 1> Vector##SizeSuffix##TypeSuffix;    \
    typedef Matrix<Type, 1, Size> RowVector##SizeSuffix##TypeSuffix;

typedef double rScalar;
typedef Eigen::Matrix<rScalar, 6, 1> Vector6d;

#define DEVICENAME "/dev/ttyUSB0"
#define PROTOCOL_VERSION 2.0
#define BAUDRATE 1000000

#define DXL1_ID 1
#define DXL2_ID 2
#define DXL3_ID 3
#define DXL4_ID 4
#define DXL5_ID 5
#define DXL6_ID 6

#define ADDR_PRO_GOAL_POSITION 116
#define LEN_PRO_GOAL_POSITION 4
#define ADDR_PRO_PRESENT_POSITION 132
#define LEN_PRO_PRESENT_POSITION 4
#define ADDR_PRO_TORQUE_ENABLE 64
#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0

#define DEG2RAD (0.01745329251994329576923690768489)
#define RAD2DEG (1 / DEG2RAD)

int dxl_comm_result = COMM_TX_FAIL; // Communication result
bool dxl_addparam_result = false;   // addParam result
bool dxl_getdata_result = false;    // GetParam result
uint8_t dxl_error = 0; // Dynamixel error
int32_t dxl_present_position = 0;
int dxl_goal_position = 0;
uint8_t param_goal_position[4];
double present_postiion[6];

std::vector<int> motorID;
double pulse2deg = 0.087890625;
double Hz = 100;

bool sw_on;
bool control_on;
bool init_pos = true;
bool control_pos = true;
bool torque_off = false;
std::mutex global_mutex;
Vector6d desired_q_;
Vector6d current_q_;

Vector6d end_eff;
Vector6d Q_init, Q_dot;
Vector6d Q_;
Vector6d Q_desired, pulse_desired;
Vector6d x_dot, x_init, x_desired;
int tick = 0;
int tick_init = 0;
double trajectory_time;

bool real_robot;

sensor_msgs::msg::JointState joint_setmsg;
sensor_msgs::msg::JointState jointmsg;

rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_set;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointReal_set;

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
    motorID.push_back(DXL3_ID);
    motorID.push_back(DXL4_ID);
    motorID.push_back(DXL5_ID);
    motorID.push_back(DXL6_ID);
}

void motorTorqueOn(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler)
{
    for (int i = 0; i < motorID.size(); i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, motorID[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

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
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, motorID[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
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
        dxl_getdata_result = groupSyncRead->isAvailable(motorID[i], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
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
        dxl_present_position = groupSyncRead->getData(motorID[i], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        present_postiion[i] = (double)dxl_present_position * pulse2deg * DEG2RAD;
    }
}

void setPosition(dynamixel::GroupSyncWrite *groupSyncWrite, dynamixel::PacketHandler *packetHandler, Vector6d desired_pulse)
{
    for (int i = 0; i < motorID.size(); i++)
    {
        dxl_goal_position = (int)desired_pulse(i);
        param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position));
        param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position));
        param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position));
        param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position));

        dxl_addparam_result = groupSyncWrite->addParam(motorID[i], param_goal_position);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", motorID[i]);
        }

        // Syncwrite goal position
        dxl_comm_result = groupSyncWrite->txPacket();
        if (dxl_comm_result != COMM_SUCCESS)
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
}

static double cubicDot(double time,    ///< Current time
                       double time_0,  ///< Start time
                       double time_f,  ///< End time
                       double x_0,     ///< Start state
                       double x_f,     ///< End state
                       double x_dot_0, ///< Start state dot
                       double x_dot_f  ///< End state dot
)
{
    double x_t;

    if (time < time_0)
    {
        x_t = x_dot_0;
    }
    else if (time > time_f)
    {
        x_t = x_dot_f;
    }
    else
    {
        double elapsed_time = time - time_0;
        double total_time = time_f - time_0;
        double total_time2 = total_time * total_time;  // pow(t,2)
        double total_time3 = total_time2 * total_time; // pow(t,3)
        double total_x = x_f - x_0;

        x_t = x_dot_0

              + 2 * (3 * total_x / total_time2 - 2 * x_dot_0 / total_time - x_dot_f / total_time) * elapsed_time

              + 3 * (-2 * total_x / total_time3 + (x_dot_0 + x_dot_f) / total_time2) * elapsed_time * elapsed_time;
    }

    return x_t;
}

static double cubic(double time,    ///< Current time
                    double time_0,  ///< Start time
                    double time_f,  ///< End time
                    double x_0,     ///< Start state
                    double x_f,     ///< End state
                    double x_dot_0, ///< Start state dot
                    double x_dot_f  ///< End state dot
)
{
    double x_t;

    if (time < time_0)
    {
        x_t = x_0;
    }
    else if (time > time_f)
    {
        x_t = x_f;
    }
    else
    {
        double elapsed_time = time - time_0;
        double total_time = time_f - time_0;
        double total_time2 = total_time * total_time;  // pow(t,2)
        double total_time3 = total_time2 * total_time; // pow(t,3)
        double total_x = x_f - x_0;

        x_t = x_0 + x_dot_0 * elapsed_time

              + (3 * total_x / total_time2 - 2 * x_dot_0 / total_time - x_dot_f / total_time) * elapsed_time * elapsed_time

              + (-2 * total_x / total_time3 +
                 (x_dot_0 + x_dot_f) / total_time2) *
                    elapsed_time * elapsed_time * elapsed_time;
    }

    return x_t;
}




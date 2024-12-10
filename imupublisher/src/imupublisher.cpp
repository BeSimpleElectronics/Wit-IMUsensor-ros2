#include <chrono>
#include <functional>
#include <memory>
#include <cstdint>
#include <unistd.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

extern "C"
{
#include "serial.h"
#include "REG.h"
#include "wit_c_sdk.h"
}
#define ACC_UPDATE 0x01
#define GYRO_UPDATE 0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE 0x08
#define READ_UPDATE 0x80

static int fd, s_iCurBaud = 9600;
static volatile char s_cDataUpdate = 0;

float fAcc[3], fGyro[3], fAngle[3];
unsigned char cBuff[1];
const int c_uiBaud[] = {2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
using namespace std::chrono_literals;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> publisher_;
std::shared_ptr<rclcpp::TimerBase> timer_;

static void AutoScanSensor(const std::string &dev);
static void SensorDataUpdate(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
int main(int argc, char *argv[])
{
    char deviceName[256] = "/dev/ttyUSB0";
    // strncpy(deviceName, "/dev/ttyUSB0", sizeof(deviceName) - 1);
    // deviceName[sizeof(deviceName) - 1] = '\0';

    if ((fd = serial_open(deviceName, 9600)) < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("string"), "Failed to open %s", deviceName);
        rclcpp::shutdown();
        return 0;
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("string"), "Opened %s successfully.", deviceName);
    }

    // Initialize WIT-Motion sensor
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    RCLCPP_INFO(rclcpp::get_logger("string"), "Status %i .", WitRegisterCallBack(SensorDataUpdate));
    AutoScanSensor("/dev/ttyUSB0");
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("simple_publisher");
    publisher_ = node->create_publisher<std_msgs::msg::String>("topic", 10);
    // timer_ = node->create_wall_timer(500ms, timer_callback);
    while (true)
    {
        while (serial_read_data(fd, cBuff, 1))
        {
            WitSerialDataIn(cBuff[0]);
        }
        std::cout << "\n";
        Delayms(500);

        if (s_cDataUpdate)
        {
            for (int i = 0; i < 3; i++) {
                fAcc[i] = sReg[AX + i] / 32768.0f * 16.0f;
                fGyro[i] = sReg[GX + i] / 32768.0f * 2000.0f;
                fAngle[i] = sReg[Roll + i] / 32768.0f * 180.0f;
            }
            std_msgs::msg::String msg;

            // Extract and format sensor data
            if (s_cDataUpdate & ACC_UPDATE)
            {
                msg.data += "Acc: ";
                msg.data += std::to_string(fAcc[0]) + " " + std::to_string(fAcc[1]) + " " + std::to_string(fAcc[2]) + "\n";
                s_cDataUpdate &= ~ACC_UPDATE;
            }
            if (s_cDataUpdate & GYRO_UPDATE)
            {
                msg.data += "Gyro: ";
                msg.data += std::to_string(fGyro[0]) + " " + std::to_string(fGyro[1]) + " " + std::to_string(fGyro[2]) + "\n";
                s_cDataUpdate &= ~GYRO_UPDATE;
            }
            if (s_cDataUpdate & ANGLE_UPDATE)
            {
                msg.data += "Angle: ";
                msg.data += std::to_string(fAngle[0]) + " " + std::to_string(fAngle[1]) + " " + std::to_string(fAngle[2]) + "\n";
                s_cDataUpdate &= ~ANGLE_UPDATE;
            }
            if (s_cDataUpdate & MAG_UPDATE)
            {
                msg.data += "Mag: ";
                msg.data += std::to_string(sReg[HX]) + " " + std::to_string(sReg[HY]) + " " + std::to_string(sReg[HZ]) + "\n";
                s_cDataUpdate &= ~MAG_UPDATE;
            }

            RCLCPP_INFO(rclcpp::get_logger("string"), "Publishing: '%s'", msg.data.c_str());
            publisher_->publish(msg);
        }
    }

    serial_close(fd);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

static void SensorDataUpdate(uint32_t uiReg, uint32_t uiRegNum)
{
    for (uint32_t i = 0; i < uiRegNum; i++)
    {
        switch (uiReg)
        {
        case AZ:
            s_cDataUpdate |= ACC_UPDATE;
            break;
        case GZ:
            s_cDataUpdate |= GYRO_UPDATE;
            break;
        case HZ:
            s_cDataUpdate |= MAG_UPDATE;
            break;
        case Yaw:
            s_cDataUpdate |= ANGLE_UPDATE;
            break;
        default:
            s_cDataUpdate |= READ_UPDATE;
            break;
        }
        uiReg++;
    }
}

static void Delayms(uint16_t ucMs)
{
    usleep(ucMs * 1000);
}

static void AutoScanSensor(const std::string &dev)
{
    unsigned char cBuff[1];

    char deviceName[256];
    strncpy(reinterpret_cast<char *>(deviceName), dev.c_str(), sizeof(deviceName) - 1);
    deviceName[sizeof(deviceName) - 1] = '\0';

    for (size_t i = 1; i < 10; i++)
    {
        serial_close(fd);
        s_iCurBaud = c_uiBaud[i];
        fd = serial_open(deviceName, c_uiBaud[i]);

        int iRetry = 2;
        do
        {
            s_cDataUpdate = 0;
            WitReadReg(AX, 3);
            Delayms(200);
            while (serial_read_data(fd, cBuff, 1))
            {
                WitSerialDataIn(cBuff[0]);
            }
            if (s_cDataUpdate != 0)
            {
                std::cout << c_uiBaud[i] << " baud sensor found.\n"
                          << std::endl;
                return;
            }
            iRetry--;
        } while (iRetry);
    }

    std::cerr << "Sensor not found. Please check your connection." << std::endl;
    // system("exit");
}
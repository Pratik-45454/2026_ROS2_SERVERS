#ifndef BNO08_HPP_
#define BNO08_HPP_

#include <string>
#include <libserial/SerialPort.h>

#define BNO_PACKET_SIZE 19
#define BNO_HEADER 0xAAAA
#define BNO_TIME_PERIOD 10 // milliseconds for 100Hz
#define _g 9.80665f

#pragma pack(push, 1)
struct BnoData
{
    uint8_t index;
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    uint8_t mi;
    uint8_t mr;
    uint8_t rsvd;
    uint8_t csum;
};
#pragma pack(pop)

struct ImuData
{
    float yaw;
    float pitch;
    float roll;
    float accel_x;
    float accel_y;
    float accel_z;
};

class Bno08
{
public:
    enum BnoRecvStatus
    {
        CPLT_DATA_NOT_AVAILABLE,
        HEADER_MATCHED,
        HEADER_ERROR,
        CHECKSUM_MATCHED,
        CHECKSUM_ERROR
    };

    std::string portName;
    LibSerial::SerialPort port;
    LibSerial::DataBuffer buffer;
    ImuData data;
    uint32_t lastReceive;

    Bno08() = default;
    Bno08(std::string _portName) : portName(_portName) {}
    ~Bno08()
    {
        port.Close();
    }

    void init();
    void receive();
    bool isConnected();
};

#endif // BNO08_HPP_
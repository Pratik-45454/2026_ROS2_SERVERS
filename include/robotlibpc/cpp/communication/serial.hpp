#ifndef ROBOTLIBPC_SERIAL_HPP_
#define ROBOTLIBPC_SERIAL_HPP_

#include <string>
#include <stdint.h>
#include <libserial/SerialPort.h>

#ifndef SERIAL_START_BYTE
#define SERAL_START_BYTE 0xA5
#endif

#ifndef SERIAL_TIMEOUT
#define SERIAL_TIMEOUT 100
#endif

enum SerialOpenStatus
{
    SERIAL_OK,
    SERIAL_ALREADY_OPEN,
    SERIAL_OPEN_FAILED,
    SERIAL_NOT_OPEN,
};

enum SerialReceiveStatus
{
    SERIAL_RECEIVE_OK,
    SERIAL_DATA_NOT_AVAILABLE,
    SERIAL_HEAD_ERROR,
    SERIAL_HEAD_RECEIVED,
    SERILA_CHECK_ERROR,
    SERIAL_RECEIVE_TIMEOUT
};

enum SerialCheckType
{
    SERIAL_PLUS_SUM,
    SERIAL_XOR_SUM,
    SERIAL_CRC8
};

class Serial : public LibSerial::SerialPort
{
public:
    std::string portName;
    uint16_t rxSize;
    uint16_t txSize;
    uint8_t head;
    uint8_t headSize;
    SerialCheckType checkType;
    std::ios_base::openmode mode;
    uint8_t headReceivedNum = 0;
    uint32_t rxSeq = 0;
    uint32_t txSeq = 0;
    uint32_t lastRxTime = 0;
    uint32_t lastTxTime = 0;

    Serial() = default;

    Serial(const std::string &_portName, uint16_t _rxSize, uint16_t _txSize = 0,
           uint8_t _head = SERAL_START_BYTE, uint8_t _headSize = 1, SerialCheckType _checkType = SERIAL_CRC8);

    Serial(const Serial &) = default;

    bool init();
    SerialReceiveStatus receive(uint8_t *data);
    bool receiveBlocking(uint8_t *data, uint32_t timeOut = 0);
    void transmit(uint8_t *data);

    ~Serial();
};

#endif // ROBOTLIBPC_SERIAL_HPP_
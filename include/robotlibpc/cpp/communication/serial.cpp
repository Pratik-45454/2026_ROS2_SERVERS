#include <vector>
#include <cstring>
#include <iostream>

#include "serial.hpp"
#include "../crypto/crc8.hpp"
#include "../common/time.hpp"

#define RX_PACKET_SIZE (headSize + rxSize + 1)

uint8_t getPlusChecksum(uint8_t *data, uint16_t size)
{
    uint8_t check = 0;
    for (int i = 0; i < size; i++)
    {
        check += data[i];
    }
    return check;
}

uint8_t getXORChecksum(uint8_t *data, uint16_t size)
{
    uint8_t check = 0;
    for (int i = 0; i < size; i++)
    {
        check ^= data[i];
    }
    return check;
}

Serial::Serial(const std::string &_portName, uint16_t _rxSize, uint16_t _txSize,
               uint8_t _head, uint8_t _headSize, SerialCheckType _checkType)
    : portName(_portName), rxSize(_rxSize), txSize(_txSize), head(_head), headSize(_headSize), checkType(_checkType)
{
    if (rxSize > 0)
    {
        mode = std::ios_base::in;
    }
    if (txSize > 0)
    {
        mode = std::ios_base::out;
    }
    if (rxSize > 0 && txSize > 0)
    {
        mode = std::ios_base::in | std::ios_base::out;
    }
}

bool Serial::init()
{
    if (rxSize == 0 && txSize == 0)
        return false;

    try
    {
        SerialPort::Open(portName, mode);
    }
    catch (const LibSerial::AlreadyOpen &ex)
    {
        std::cout << "Serial::Port `" << portName << "` is already open" << std::endl;
    }
    catch (const LibSerial::OpenFailed &ex)
    {
        std::cerr << "Serial::Failed to open `" << portName << "`" << std::endl;
        std::cerr << ex.what() << std::endl;
        return false;
    }
    catch (const LibSerial::NotOpen &ex)
    {
        std::cerr << "Serial::Not open `" << portName << "`" << std::endl;
        std::cerr << ex.what() << std::endl;
        return false;
    }

    return true;
}

SerialReceiveStatus Serial::receive(uint8_t *data)
{
    std::vector<uint8_t> rxBuffer;
    while (GetNumberOfBytesAvailable() < RX_PACKET_SIZE)
        return SERIAL_DATA_NOT_AVAILABLE;

    uint8_t bytesReadCount = 0;
    while ((headReceivedNum < headSize) && (bytesReadCount < RX_PACKET_SIZE))
    {
        Read(rxBuffer, 1);
        bytesReadCount++;

        if (rxBuffer.front() == head)
        {
            headReceivedNum++;
        }
        else
        {
            headReceivedNum = 0;
            std::cerr << "Serial::Head didn't match." << std::endl;
        }
    }

    if (headReceivedNum < headSize)
        return SERIAL_HEAD_ERROR;

    if (GetNumberOfBytesAvailable() < rxSize + 1)
        return SERIAL_HEAD_RECEIVED;

    headReceivedNum = 0;
    Read(rxBuffer, rxSize + 1);

    uint8_t check;
    switch (checkType)
    {
    case SERIAL_PLUS_SUM:
        check = getPlusChecksum(rxBuffer.data(), rxSize);
        break;

    case SERIAL_XOR_SUM:
        check = getXORChecksum(rxBuffer.data(), rxSize);
        break;

    case SERIAL_CRC8:
        check = CRC8::Instance.get_hash(rxBuffer.data(), rxSize);
    }

    if (check == rxBuffer.back())
    {
        memcpy(data, rxBuffer.data(), rxSize);
        lastRxTime = get_tick_ms();
        rxSeq++;
        return SERIAL_RECEIVE_OK;
    }
    else
    {
        std::cerr << "Serial::Check didn't match." << std::endl;
    }

    return SERILA_CHECK_ERROR;
}

bool Serial::receiveBlocking(uint8_t *data, uint32_t timeOut)
{
    uint32_t startTime = get_tick_ms();
    while ((timeOut == 0) || (get_tick_ms() - startTime < timeOut))
    {
        if (receive(data) == SERIAL_RECEIVE_OK)
            return true;
    }
    return false;
}

void Serial::transmit(uint8_t *data)
{
    std::vector<uint8_t> txBuffer(data, data + txSize);
    Write(txBuffer);
    lastTxTime = get_tick_ms();
    txSeq++;
}

Serial::~Serial()
{
    if (IsOpen())
        Close();
}
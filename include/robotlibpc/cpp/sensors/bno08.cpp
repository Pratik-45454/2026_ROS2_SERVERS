#include <iostream>
#include <cstring>
#include "../common/time.hpp"
#include "bno08.hpp"

void Bno08::init()
{
    try
    {
        port.Open(portName, std::ios_base::in);
    }
    catch (const LibSerial::AlreadyOpen &ex)
    {
    }
    catch (const LibSerial::OpenFailed &ex)
    {

        std::cerr << "Serial::Failed to open `" << portName << "`" << std::endl;
        std::cerr << ex.what() << std::endl;
        throw;
    }
    catch (const LibSerial::NotOpen &ex)
    {
        std::cerr << "Serial::Not open `" << portName << "`" << std::endl;
        std::cerr << ex.what() << std::endl;
        throw;
    }
}

void Bno08::receive()
{
    BnoRecvStatus status = CPLT_DATA_NOT_AVAILABLE;
    bool isWaitingForHeader = true;
    while (status != CHECKSUM_MATCHED)
    {
        if (port.GetNumberOfBytesAvailable() >= 19)
        {
            if (isWaitingForHeader)
            {
                port.Read(buffer, 2);
                uint16_t header = static_cast<uint16_t>(buffer.at(0)) | static_cast<uint16_t>(buffer.at(1)) << 8;
                if (header == BNO_HEADER)
                {
                    isWaitingForHeader = false;
                    status = HEADER_MATCHED;
                }
                else
                {
                    std::cerr << "Bno08::Start byte didn't match." << std::endl;
                    status = HEADER_ERROR;
                }
            }
            else
            {
                port.Read(buffer, 17);

                uint8_t csum = 0;
                for (int i = 0; i < 16; i++)
                {
                    csum += buffer.at(i);
                }

                if (csum == buffer.at(16))
                {
                    BnoData *pBuffer = (BnoData *)buffer.data();
                    data.yaw = -static_cast<float>(pBuffer->yaw / 100) + 0.01f * static_cast<float>(pBuffer->yaw % 100);
                    data.pitch = static_cast<float>(pBuffer->pitch / 100) + 0.01f * static_cast<float>(pBuffer->pitch % 100);
                    data.roll = static_cast<float>(pBuffer->roll / 100) + 0.01f * static_cast<float>(pBuffer->roll % 100);
                    data.accel_x = pBuffer->accel_x * _g / 1000;
                    data.accel_y = pBuffer->accel_y * _g / 1000;
                    data.accel_z = pBuffer->accel_z * _g / 1000;
                    uint32_t lastReceive = get_tick_ms();
                    status = CHECKSUM_MATCHED;
                }
                else
                {
                    std::cerr << "Bno08:: Checksum didn't match." << std::endl;
                    status = CHECKSUM_ERROR;
                }
                isWaitingForHeader = true;
            }
        }
    }
}

/**
 ****************************************************************************
 * @file    tfmini.cpp
 * @author  Robotics Team, IOE Pulchowk Campus
 * @brief   This file implements the TFmini class for receiving and parsing TFmini data
 * @details
 * - Created: 2023 by Kundan Siwakoti(078bei018)
 * - Modified: Jan 15, 2024 by Sagar Chaudhary(078bei031)
 *   - Improved callback and parsing algorithm
 *   - Added restore settings function
 *   - Added reset device
 *   - Documented file
 ****************************************************************************
 */

#include <iostream>
#include <memory.h>

#include "tfmini.hpp"
#include "../common/time.hpp"

#define serial_port_address_black "/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0"
#define PI_UART "/dev/serial0"

bool TFmini::init()
{
    try
    {
        serial_port.Open(port_name);
    }
    catch (const LibSerial::AlreadyOpen &ex)
    {
    }
    catch (const LibSerial::OpenFailed &ex)
    {
        std::cerr << "Serial::Failed to open `" << port_name << "`" << std::endl;
        std::cerr << ex.what() << std::endl;
        return false;
    }
    catch (const LibSerial::NotOpen &ex)
    {
        std::cerr << "Serial::Not open `" << port_name << "`" << std::endl;
        std::cerr << ex.what() << std::endl;
        return false;
    }

    head_received_num = 0;
    last_received = 0;

    return true;
}

bool TFmini::receive()
{
    std::vector<uint8_t> input_buffer;
    while (serial_port.GetNumberOfBytesAvailable() < TFMINI_FRAME_SIZE)
        return false;

    uint8_t bytes_read_count = 0;
    while ((head_received_num < TFMINI_HEAD_SIZE) && (bytes_read_count < TFMINI_FRAME_SIZE))
    {
        serial_port.Read(input_buffer, 1);
        bytes_read_count++;

        if (input_buffer.front() == TF_HEAD)
        {
            head_received_num++;
        }
        else
        {
            head_received_num = 0;
            std::cerr << "TFmini::Head not matched." << std::endl;
        }
    }

    if (head_received_num < TFMINI_HEAD_SIZE)
        return false;

    if (serial_port.GetNumberOfBytesAvailable() < TFMINI_DATA_SIZE + 1)
        return false;

    head_received_num = 0;
    serial_port.Read(input_buffer, TFMINI_DATA_SIZE + 1);

    uint8_t check = TF_HEAD + TF_HEAD;
    for (int i = 0; i < TFMINI_DATA_SIZE; i++)
    {
        check += input_buffer.at(i);
    }

    if (check == input_buffer.back())
    {
        distance = (uint16_t)input_buffer.at(0) | (uint16_t)input_buffer.at(1) << 8;
        strength = (uint16_t)input_buffer.at(2) | (uint16_t)input_buffer.at(3) << 8;
        temperature = (uint16_t)input_buffer.at(4) | (uint16_t)input_buffer.at(5) << 8;
        last_received = get_tick_ms();
        return true;
    }
    else
    {
        std::cerr << "TFmini::Checksum not matched." << std::endl;
    }

    return false;
}

uint16_t TFmini::get_distance()
{
    return distance;
}

uint16_t TFmini::get_strength()
{
    return strength;
}

uint16_t TFmini::get_temperature()
{
    return temperature;
}

bool TFmini::is_reliable()
{
    if ((strength > 200U) && (strength != 65535U))
        return true;

    return false;
}

bool TFmini::is_connected()
{
    return (get_tick_ms() - last_received < TFMINI_TIMEOUT);
}

void TFmini::print_data()
{
    std::cout << "TFmini:: distance:" << distance << " strength:" << strength << " temperature:" << temperature << std::endl;
}

void TFmini::close()
{
    serial_port.Close();
}

TFmini::~TFmini()
{
    if (serial_port.IsOpen())
        serial_port.Close();
}
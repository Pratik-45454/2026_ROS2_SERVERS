/**
 ****************************************************************************
 * @file    tfmini.hpp
 * @brief   Header for interfacing TFmini plus Lidar with rasb-pi
 * @author  Robotics Team, IOE Pulchowk Campus
 ****************************************************************************
 * @note
 * @brief   Frame Format:
 * - Byte0: 0x59, frame header, same for each frame
 * - Byte1: 0x59, frame header, same for each frame
 * - Byte2: Dist_L distance value lower by 8 bits
 * - Byte3: Dist_H distance value higher by 8 bits
 * - Byte4: Strength_L low 8 bits
 * - Byte5: Strength_H high 8 bits
 * - Byte6: Temp_L low 8 bits (suitable for version later than V1.3.0)
 * - Byte7: Temp_H high 8 bits (suitable for version later than V1.3.0)
 * - Byte8: Checksum is the lower 8 bits of the cumulative sum of the numbers of the first 8 bytes.
 ****************************************************************************
 * @attention
 * @brief Data Communication Protocol:
 * - Communication interface  : UART
 * - Baud rate                : 115200
 * - Data bit                 : 8
 * - Stop bit                 : 1
 * - Parity check             : None
 *****************************************************************************
 */

#ifndef TFMINI_HPP__
#define TFMINI_HPP__

#include <stdint.h>
#include <string>
#include <string>
#include <libserial/SerialPort.h>
#include "../msgs/sensor_msg.hpp"

#define TF_HEAD 0x59
#define TFMINI_HEAD_SIZE 2
#define TFMINI_DATA_SIZE 6
#define TFMINI_FRAME_SIZE 9

#ifndef TFMINI_TIMEOUT
#define TFMINI_TIMEOUT 100
#endif
#define TF_HEAD 0x59
#define TFMINI_HEAD_SIZE 2
#define TFMINI_DATA_SIZE 6
#define TFMINI_FRAME_SIZE 9

#ifndef TFMINI_TIMEOUT
#define TFMINI_TIMEOUT 100
#endif

/**
 * @class   TFmini
 * @brief   Class for interfacing TFmini LiDAR
 */
class TFmini
{
public:
  LibSerial::SerialPort serial_port;
  std::string port_name;
  uint16_t distance;    /**< Measured distance */
  uint16_t strength;    /**< Measured signal strength */
  uint16_t temperature; /**< Measured temperature */
  uint8_t head_received_num;
  uint32_t last_received;

  /**
   * @brief   Default constructor
   */
  TFmini() = default;

  /**
   * @brief Constructor
   * @param fileName: name of serial port
   */
  TFmini(const std::string &_port_name) : port_name(_port_name) {}

  bool init();

  /**
   * @brief Recieve from Serial Port
   */
  bool receive();

  /**
   * @brief   Get the measured distance
   * @return  Measured distance in centimeters
   */
  uint16_t get_distance();

  /**
   * @brief   Get the measured temperature
   * @return  Measured temperature in degrees Celsius
   */
  uint16_t get_temperature();

  /**
   * @brief   Get the measured signal strength
   * @return  Measured signal strength
   */
  uint16_t get_strength();

  /**
   * @brief Check if strenth is reliable or not
   *
   * @return true   Reliable
   * @return false  Unreliable
   */
  bool is_reliable();

  /**
   * @brief Check if the sensor is connected
   *
   * @return true   Connected
   * @return false  Not connected
   */
  bool is_connected();

  /**
   * @brief   Print the measured data
   */
  void print_data();

  void close();

  /**
   * @brief   Default destructor
   */
  ~TFmini();
};

#endif // TFMINI_HPP__
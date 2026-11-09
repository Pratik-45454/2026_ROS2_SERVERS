#ifndef __ROBOT__MSG__
#define __ROBOT__MSG__

#include <stdint.h>

/**
 * @struct  TF_Frame
 * @brief   Structure to represent the TFmini packet format
 */
struct TF_Frame
{
  struct
  {
    uint8_t head0; /**< Byte 0: Frame header, should be 0x59 */
    uint8_t head1; /**< Byte 1: Frame header, should be 0x59 */
  } head;

  struct
  {
    uint8_t distance_L;    /**< Byte 2: Distance value lower 8 bits */
    uint8_t distance_H;    /**< Byte 3: Distance value higher 8 bits */
    uint8_t strength_L;    /**< Byte 4: Strength low 8 bits */
    uint8_t strength_H;    /**< Byte 5: Strength high 8 bits */
    uint8_t temperature_L; /**< Byte 6: Temperature low 8 bits (for version later than V1.3.0) */
    uint8_t temperature_H; /**< Byte 7: Temperature high 8 bits (for version later than V1.3.0) */
  } data;

  uint8_t checksum; /**< Byte 8: Checksum is the lower 8 bits of the cumulative sum of the numbers of the first 8 bytes. */
};

#endif

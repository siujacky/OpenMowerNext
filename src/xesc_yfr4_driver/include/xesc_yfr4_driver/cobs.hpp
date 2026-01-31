/**
 * @file cobs.hpp
 * @brief Consistent Overhead Byte Stuffing (COBS) Encoder/Decoder
 *
 * Original copyright: Christopher Baker, Jacques Fortier (MIT License)
 * Ported to ROS2 for OpenMowerNext
 */

#ifndef XESC_YFR4_DRIVER__COBS_HPP_
#define XESC_YFR4_DRIVER__COBS_HPP_

#include <cstddef>
#include <cstdint>

namespace xesc_yfr4_driver
{

/**
 * @brief A Consistent Overhead Byte Stuffing (COBS) Encoder/Decoder
 */
class COBS
{
public:
  /**
   * @brief Encode a byte buffer with the COBS encoder
   */
  static std::size_t encode(const uint8_t* buffer, std::size_t size, uint8_t* encoded_buffer)
  {
    std::size_t read_index = 0;
    std::size_t write_index = 1;
    std::size_t code_index = 0;
    uint8_t code = 1;

    while (read_index < size)
    {
      if (buffer[read_index] == 0)
      {
        encoded_buffer[code_index] = code;
        code = 1;
        code_index = write_index++;
        read_index++;
      }
      else
      {
        encoded_buffer[write_index++] = buffer[read_index++];
        code++;

        if (code == 0xFF)
        {
          encoded_buffer[code_index] = code;
          code = 1;
          code_index = write_index++;
        }
      }
    }

    encoded_buffer[code_index] = code;

    return write_index;
  }

  /**
   * @brief Decode a COBS-encoded buffer
   */
  static std::size_t decode(const uint8_t* encoded_buffer, std::size_t size, uint8_t* decoded_buffer)
  {
    if (size == 0)
    {
      return 0;
    }

    std::size_t read_index = 0;
    std::size_t write_index = 0;
    uint8_t code = 0;
    uint8_t i = 0;

    while (read_index < size)
    {
      code = encoded_buffer[read_index];

      if (read_index + code > size && code != 1)
      {
        return 0;
      }

      read_index++;

      for (i = 1; i < code; i++)
      {
        decoded_buffer[write_index++] = encoded_buffer[read_index++];
      }

      if (code != 0xFF && read_index != size)
      {
        decoded_buffer[write_index++] = '\0';
      }
    }

    return write_index;
  }

  /**
   * @brief Get the maximum encoded buffer size
   */
  static constexpr std::size_t getEncodedBufferSize(std::size_t unencoded_buffer_size)
  {
    return unencoded_buffer_size + unencoded_buffer_size / 254 + 1;
  }
};

}  // namespace xesc_yfr4_driver

#endif  // XESC_YFR4_DRIVER__COBS_HPP_

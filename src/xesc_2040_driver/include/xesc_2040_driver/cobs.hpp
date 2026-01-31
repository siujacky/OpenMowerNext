/**
 * @file cobs.hpp
 * @brief Consistent Overhead Byte Stuffing (COBS) Encoder/Decoder
 *
 * Original copyright: Christopher Baker, Jacques Fortier (MIT License)
 * Ported to ROS2 for OpenMowerNext
 */

#ifndef XESC_2040_DRIVER__COBS_HPP_
#define XESC_2040_DRIVER__COBS_HPP_

#include <cstddef>
#include <cstdint>

namespace xesc_2040_driver
{

/**
 * @brief A Consistent Overhead Byte Stuffing (COBS) Encoder/Decoder
 *
 * Consistent Overhead Byte Stuffing (COBS) is an encoding that removes all 0
 * bytes from arbitrary binary data. The encoded data consists only of bytes
 * with values from 0x01 to 0xFF. This is useful for preparing data for
 * transmission over a serial link (RS-232 or RS-485 for example), as the 0
 * byte can be used to unambiguously indicate packet boundaries. COBS also has
 * the advantage of adding very little overhead (at least 1 byte, plus up to an
 * additional byte per 254 bytes of data). For messages smaller than 254 bytes,
 * the overhead is constant.
 *
 * @sa http://conferences.sigcomm.org/sigcomm/1997/papers/p062.pdf
 * @sa http://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
 */
class COBS
{
public:
  /**
   * @brief Encode a byte buffer with the COBS encoder
   * @param buffer A pointer to the unencoded buffer to encode
   * @param size The number of bytes in the buffer
   * @param encoded_buffer The buffer for the encoded bytes
   * @returns The number of bytes written to the encoded_buffer
   * @warning The encoded_buffer must have at least getEncodedBufferSize() allocated
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
   * @param encoded_buffer A pointer to the encoded_buffer to decode
   * @param size The number of bytes in the encoded_buffer
   * @param decoded_buffer The target buffer for the decoded bytes
   * @returns The number of bytes written to the decoded_buffer
   * @warning decoded_buffer must have a minimum capacity of size
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
   * @brief Get the maximum encoded buffer size for an unencoded buffer size
   * @param unencoded_buffer_size The size of the buffer to be encoded
   * @returns The maximum size of the required encoded buffer
   */
  static constexpr std::size_t getEncodedBufferSize(std::size_t unencoded_buffer_size)
  {
    return unencoded_buffer_size + unencoded_buffer_size / 254 + 1;
  }
};

}  // namespace xesc_2040_driver

#endif  // XESC_2040_DRIVER__COBS_HPP_

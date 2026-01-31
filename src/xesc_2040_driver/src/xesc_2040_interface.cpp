/**
 * @file xesc_2040_interface.cpp
 * @brief Serial interface implementation for XESC 2040 motor controller
 *
 * Ported from open_mower_ros (ROS1) to ROS2
 * Original author: Clemens Elflein
 *
 * Changes from ROS1:
 * - pthread replaced with std::thread
 * - nanosleep replaced with std::this_thread::sleep_for
 * - Modern C++17 style
 */

#include "xesc_2040_driver/xesc_2040_interface.hpp"

#include <sstream>

namespace xesc_2040_driver
{

Xesc2040Interface::Xesc2040Interface(const ErrorHandlerFunction& error_handler)
  : serial_(std::string(), 115200, serial::Timeout::simpleTimeout(100), serial::eightbits, serial::parity_none,
            serial::stopbits_one, serial::flowcontrol_none)
  , error_handler_(error_handler)
{
}

Xesc2040Interface::~Xesc2040Interface()
{
  stop();
}

void Xesc2040Interface::rxThread()
{
  COBS cobs;
  boost::crc_ccitt_type crc;

  constexpr std::size_t BUFFER_SIZE = 1000;
  uint8_t buffer[BUFFER_SIZE];
  uint8_t buffer_decoded[BUFFER_SIZE];
  std::size_t read_pos = 0;

  {
    std::unique_lock<std::mutex> lk(status_mutex_);
    status_ = Xesc2040StatusStruct{};
    status_.connection_state = Xesc2040ConnectionState::DISCONNECTED;
  }

  while (rx_thread_run_)
  {
    if (!serial_.isOpen())
    {
      try
      {
        status_.connection_state = Xesc2040ConnectionState::DISCONNECTED;

        serial_.setPort(port_);
        serial_.setBaudrate(115200);
        auto timeout = serial::Timeout::simpleTimeout(100);
        serial_.setTimeout(timeout);
        serial_.open();

        // Wait for controller to boot
        std::this_thread::sleep_for(std::chrono::seconds(1));
        status_.connection_state = Xesc2040ConnectionState::CONNECTED;
        sendSettings();
      }
      catch (const std::exception& e)
      {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::ostringstream ss;
        ss << "Error connecting to xESC on Port: " << serial_.getPort();
        error_handler_(ss.str());
      }
    }

    std::size_t bytes_read = 0;
    try
    {
      bytes_read = serial_.read(buffer + read_pos, 1);
    }
    catch (const std::exception& e)
    {
      std::ostringstream ss;
      ss << "Error reading serial_port. Closing Connection. Port: " << serial_.getPort();
      error_handler_(ss.str());
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if (read_pos + bytes_read >= BUFFER_SIZE)
    {
      read_pos = 0;
      bytes_read = 0;
      std::ostringstream ss;
      ss << "Prevented buffer overflow. There is a problem with the serial comms. Port: " << serial_.getPort();
      error_handler_(ss.str());
    }

    if (bytes_read)
    {
      if (buffer[read_pos] == 0)
      {
        // End of packet found
        std::size_t data_size = cobs.decode(buffer, read_pos, buffer_decoded);

        // First, check the CRC
        if (data_size < 3)
        {
          // We don't even have one byte of data
          // (type + crc = 3 bytes already)
          std::ostringstream ss;
          ss << "Got empty packet from xESC. Port: " << serial_.getPort();
          error_handler_(ss.str());
        }
        else
        {
          // We have at least 1 byte of data, check the CRC
          crc.reset();
          // Process (data_size - 2 bytes for CRC) bytes
          crc.process_bytes(buffer_decoded, data_size - 2);
          uint16_t checksum = crc.checksum();
          uint16_t received_checksum = *reinterpret_cast<uint16_t*>(buffer_decoded + data_size - 2);

          if (checksum == received_checksum)
          {
            // Packet checksum is OK, process it
            switch (buffer_decoded[0])
            {
              case XESC2040_MSG_TYPE_STATUS: {
                if (data_size == sizeof(Xesc2040StatusPacket))
                {
                  handlePacket(reinterpret_cast<Xesc2040StatusPacket*>(buffer_decoded));
                }
                else
                {
                  std::ostringstream ss;
                  ss << "Got packet with wrong size on port: " << serial_.getPort()
                     << ". id was: " << static_cast<int>(buffer_decoded[0]);
                  error_handler_(ss.str());
                }
              }
              break;

              default: {
                std::ostringstream ss;
                ss << "Got unknown valid packet from xESC on Port: " << serial_.getPort()
                   << ". id was: " << static_cast<int>(buffer_decoded[0]);
                error_handler_(ss.str());
              }
              break;
            }
          }
          else
          {
            std::ostringstream ss;
            ss << "Got invalid checksum from xESC on Port: " << serial_.getPort();
            error_handler_(ss.str());
          }
        }

        read_pos = 0;
      }
      else
      {
        read_pos += bytes_read;
      }
    }
  }

  serial_.close();
}

void Xesc2040Interface::handlePacket(Xesc2040StatusPacket* packet)
{
  // Only update the state if connection state is connected
  if (status_.connection_state == Xesc2040ConnectionState::CONNECTED)
  {
    std::lock_guard<std::mutex> lk(status_mutex_);
    status_.seq = packet->seq;
    status_.fw_version_major = packet->fw_version_major;
    status_.fw_version_minor = packet->fw_version_minor;
    status_.voltage_input = packet->voltage_input;
    status_.temperature_pcb = packet->temperature_pcb;
    status_.temperature_motor = packet->temperature_motor;
    status_.current_input = packet->current_input;
    status_.duty_cycle = packet->duty_cycle;
    status_.tacho = packet->tacho;
    status_.tacho_absolute = packet->tacho_absolute;
    status_.direction = packet->direction;
    status_.fault_code = packet->fault_code;

    // If unconfigured, send settings
    if (packet->fault_code & FAULT_UNINITIALIZED)
    {
      sendSettings();
    }

    status_cv_.notify_all();
  }
  else
  {
    std::ostringstream ss;
    ss << "Got packet with status != connected on port: " << serial_.getPort();
    error_handler_(ss.str());
  }
}

bool Xesc2040Interface::send(uint8_t* packet, std::size_t length)
{
  std::unique_lock<std::mutex> lk(serial_tx_mutex_);
  if (!serial_.isOpen())
  {
    return false;
  }

  tx_crc_.reset();
  tx_crc_.process_bytes(packet, length - 2);
  uint16_t checksum = tx_crc_.checksum();
  *reinterpret_cast<uint16_t*>(packet + length - 2) = checksum;

  std::size_t encoded_size = COBS::encode(packet, length, tx_buffer_);
  tx_buffer_[encoded_size] = 0;
  encoded_size++;

  std::size_t written = serial_.write(tx_buffer_, encoded_size);
  return written == encoded_size;
}

void Xesc2040Interface::setDutyCycle(double duty_cycle)
{
  Xesc2040ControlPacket control_packet;
  control_packet.message_type = XESC2040_MSG_TYPE_CONTROL;
  control_packet.duty_cycle = duty_cycle;
  control_packet.crc = 0;
  send(reinterpret_cast<uint8_t*>(&control_packet), sizeof(control_packet));
}

void Xesc2040Interface::start(const std::string& port)
{
  port_ = port;
  rx_thread_run_ = true;
  rx_thread_ = std::thread(&Xesc2040Interface::rxThread, this);
}

void Xesc2040Interface::stop()
{
  // Stop the motor
  setDutyCycle(0.0);

  // Tell the IO thread to stop
  rx_thread_run_ = false;

  // Wait for IO thread to actually exit
  if (rx_thread_.joinable())
  {
    rx_thread_.join();
  }
}

void Xesc2040Interface::getStatus(Xesc2040StatusStruct* status)
{
  std::unique_lock<std::mutex> lk(status_mutex_);
  *status = status_;
}

void Xesc2040Interface::waitForStatus(Xesc2040StatusStruct* status)
{
  std::unique_lock<std::mutex> lk(status_mutex_);
  status_cv_.wait(lk);
  *status = status_;
}

void Xesc2040Interface::sendSettings()
{
  if (!settings_valid_)
  {
    error_handler_("Error sending xESC settings: Settings invalid. Call updateSettings first!");
    return;
  }
  settings_.message_type = XESC2040_MSG_TYPE_SETTINGS;
  send(reinterpret_cast<uint8_t*>(&settings_), sizeof(settings_));
}

void Xesc2040Interface::updateSettings(uint8_t* hall_table, float motor_current_limit, float acceleration,
                                       bool has_motor_temp, float min_motor_temp, float max_motor_temp,
                                       float min_pcb_temp, float max_pcb_temp)
{
  for (int i = 0; i < 8; i++)
  {
    settings_.hall_table[i] = hall_table[i];
  }
  settings_.motor_current_limit = motor_current_limit;
  settings_.acceleration = acceleration;
  settings_.has_motor_temp = has_motor_temp;
  settings_.min_motor_temp = min_motor_temp;
  settings_.max_motor_temp = max_motor_temp;
  settings_.min_pcb_temp = min_pcb_temp;
  settings_.max_pcb_temp = max_pcb_temp;
  settings_valid_ = true;
  sendSettings();
}

}  // namespace xesc_2040_driver

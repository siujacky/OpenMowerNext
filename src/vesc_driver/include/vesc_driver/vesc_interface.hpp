/**
 * @file vesc_interface.hpp
 * @brief VESC Serial Interface - Communicates with VESC motor controllers over serial
 *
 * Ported from open_mower_ros (ROS1) to ROS2
 *
 * Copyright (c) 2019, SoftBank Corp.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Softbank Corp. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Originally developed by Michael T. Boulet at MIT under BSD 3-clause License.
 */

#ifndef VESC_DRIVER__VESC_INTERFACE_HPP_
#define VESC_DRIVER__VESC_INTERFACE_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <serial/serial.h>
#include <boost/crc.hpp>

#include "vesc_driver/vesc_packet.hpp"
#include "vesc_driver/vesc_packet_factory.hpp"

namespace vesc_driver
{

/**
 * @brief VESC connection state enumeration
 */
enum class VescConnectionState
{
  DISCONNECTED = 0,
  WAITING_FOR_FW = 1,
  CONNECTED_INCOMPATIBLE_FW = 2,
  CONNECTED = 3,
};

/**
 * @brief Structure containing VESC status information
 */
struct VescStatusStruct
{
  uint32_t seq = 0;
  uint8_t fw_version_major = 0;
  uint8_t fw_version_minor = 0;
  VescConnectionState connection_state = VescConnectionState::DISCONNECTED;
  double voltage_input = 0.0;      ///< Input voltage (volt)
  double temperature_pcb = 0.0;    ///< Temperature of PCB (degrees Celsius)
  double temperature_motor = 0.0;  ///< Temperature of motor (degrees Celsius)
  double current_motor = 0.0;      ///< Motor current (ampere)
  double current_input = 0.0;      ///< Input current (ampere)
  double speed_erpm = 0.0;         ///< Motor velocity (electrical RPM)
  double duty_cycle = 0.0;         ///< Duty cycle (0 to 1)
  double charge_drawn = 0.0;       ///< Electric charge drawn from input (ampere-hour)
  double charge_regen = 0.0;       ///< Electric charge regenerated to input (ampere-hour)
  double energy_drawn = 0.0;       ///< Energy drawn from input (watt-hour)
  double energy_regen = 0.0;       ///< Energy regenerated to input (watt-hour)
  double displacement = 0.0;       ///< Net tachometer (counts)
  double distance_traveled = 0.0;  ///< Total tachometer (counts)
  uint32_t tacho = 0;
  uint32_t tacho_absolute = 0;
  bool direction = false;
  int32_t fault_code = 0;
};

/**
 * @class VescInterface
 * @brief Class providing an interface to the Vedder VESC motor controller via serial port
 *
 * This class manages the serial connection to a VESC motor controller,
 * handles packet parsing, and provides methods to send commands and
 * receive status updates.
 */
class VescInterface
{
public:
  /**
   * @brief Error handler function type
   */
  using ErrorHandlerFunction = std::function<void(const std::string&)>;

  // Delete copy/move operations (non-copyable due to threads/mutexes)
  VescInterface(const VescInterface&) = delete;
  VescInterface& operator=(const VescInterface&) = delete;
  VescInterface(VescInterface&&) = delete;
  VescInterface& operator=(VescInterface&&) = delete;

  /**
   * @brief Constructs a VescInterface object
   * @param error_handler Callback function for error messages
   * @param state_request_millis Interval between state requests in milliseconds
   */
  explicit VescInterface(const ErrorHandlerFunction& error_handler, uint32_t state_request_millis = 20);

  /**
   * @brief Destructor - stops threads and closes connection
   */
  ~VescInterface();

  /**
   * @brief Set the motor duty cycle
   * @param duty_cycle Duty cycle value (-1.0 to 1.0)
   */
  void setDutyCycle(double duty_cycle);

  /**
   * @brief Set the motor current
   * @param current Current in amperes
   */
  void setCurrent(double current);

  /**
   * @brief Set the brake current
   * @param brake Brake current in amperes
   */
  void setBrake(double brake);

  /**
   * @brief Set the motor speed (electrical RPM)
   * @param speed Speed in electrical RPM
   */
  void setSpeed(double speed);

  /**
   * @brief Set the motor position
   * @param position Position value
   */
  void setPosition(double position);

  /**
   * @brief Start the interface and connect to the specified port
   * @param port Serial port path (e.g., "/dev/ttyUSB0")
   */
  void start(const std::string& port);

  /**
   * @brief Stop the interface and close the connection
   */
  void stop();

  /**
   * @brief Get the current status (non-blocking)
   * @param[out] status Pointer to status structure to fill
   */
  void getStatus(VescStatusStruct* status);

  /**
   * @brief Wait for and get a new status update (blocking)
   * @param[out] status Pointer to status structure to fill
   */
  void waitForStatus(VescStatusStruct* status);

  /**
   * @brief Request firmware version from VESC
   */
  void requestFWVersion();

  /**
   * @brief Request current state/values from VESC
   */
  void requestState();

private:
  /**
   * @brief Send a VESC packet
   * @param packet The packet to send
   * @return true if successful, false otherwise
   */
  bool send(const VescPacket& packet);

  /**
   * @brief RX thread function - reads and parses incoming data
   */
  void rxThread();

  /**
   * @brief Update thread function - periodically requests state
   */
  void updateThread();

  /**
   * @brief Handle a received packet
   * @param packet The received packet
   */
  void handlePacket(VescPacketConstPtr packet);

  // Thread handles
  std::thread rx_thread_;
  std::thread update_thread_;
  std::atomic<bool> rx_thread_run_{ false };
  std::atomic<bool> update_thread_run_{ false };

  // Callback and serial interface
  ErrorHandlerFunction error_handler_;
  serial::Serial serial_;
  std::string port_;

  // Thread synchronization
  std::mutex status_mutex_;
  std::mutex serial_tx_mutex_;
  std::condition_variable status_cv_;

  // Current status
  VescStatusStruct status_;

  // Configuration
  uint32_t state_request_millis_;
};

}  // namespace vesc_driver

#endif  // VESC_DRIVER__VESC_INTERFACE_HPP_

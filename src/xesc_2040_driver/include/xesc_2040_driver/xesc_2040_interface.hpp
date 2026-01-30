/**
 * @file xesc_2040_interface.hpp
 * @brief Serial interface for XESC 2040 motor controller
 * 
 * Ported from open_mower_ros (ROS1) to ROS2
 * Original author: Clemens Elflein
 * 
 * Changes from ROS1:
 * - pthread replaced with std::thread
 * - Modern C++17 style
 */

#ifndef XESC_2040_DRIVER__XESC_2040_INTERFACE_HPP_
#define XESC_2040_DRIVER__XESC_2040_INTERFACE_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <boost/crc.hpp>
#include <serial/serial.h>

#include "xesc_2040_driver/cobs.hpp"
#include "xesc_2040_driver/xesc_2040_datatypes.hpp"

namespace xesc_2040_driver
{

/**
 * @brief Connection state for XESC 2040
 */
enum class Xesc2040ConnectionState
{
  DISCONNECTED = 0,
  CONNECTED = 3
};

/**
 * @brief Status structure for XESC 2040
 */
struct Xesc2040StatusStruct
{
  uint32_t seq{0};
  uint8_t fw_version_major{0};
  uint8_t fw_version_minor{0};
  Xesc2040ConnectionState connection_state{Xesc2040ConnectionState::DISCONNECTED};
  double voltage_input{0.0};       ///< Input voltage (volt)
  double temperature_pcb{0.0};     ///< PCB temperature (degrees Celsius)
  double temperature_motor{0.0};   ///< Motor temperature (degrees Celsius)
  double current_input{0.0};       ///< Input current (ampere)
  double duty_cycle{0.0};          ///< Duty cycle (0 to 1)
  uint32_t tacho{0};               ///< Tachometer count
  uint32_t tacho_absolute{0};      ///< Wheel ticks absolute
  bool direction{false};           ///< Direction CW/CCW
  int32_t fault_code{0};           ///< Fault code bitmask
};

/**
 * @brief Interface class for communicating with XESC 2040 motor controller
 */
class Xesc2040Interface
{
public:
  using ErrorHandlerFunction = std::function<void(const std::string &)>;

  /**
   * @brief Constructor
   * @param error_handler Callback function for error handling
   */
  explicit Xesc2040Interface(const ErrorHandlerFunction & error_handler);

  /**
   * @brief Destructor - stops the interface
   */
  ~Xesc2040Interface();

  // Non-copyable
  Xesc2040Interface(const Xesc2040Interface &) = delete;
  Xesc2040Interface & operator=(const Xesc2040Interface &) = delete;

  /**
   * @brief Set the motor duty cycle
   * @param duty_cycle Duty cycle value (-1.0 to 1.0)
   */
  void setDutyCycle(double duty_cycle);

  /**
   * @brief Start the interface
   * @param port Serial port path (e.g., "/dev/ttyUSB0")
   */
  void start(const std::string & port);

  /**
   * @brief Stop the interface
   */
  void stop();

  /**
   * @brief Get current status (non-blocking)
   * @param[out] status Status structure to populate
   */
  void getStatus(Xesc2040StatusStruct * status);

  /**
   * @brief Wait for and get new status (blocking)
   * @param[out] status Status structure to populate
   */
  void waitForStatus(Xesc2040StatusStruct * status);

  /**
   * @brief Update controller settings
   */
  void updateSettings(
    uint8_t * hall_table,
    float motor_current_limit,
    float acceleration,
    bool has_motor_temp,
    float min_motor_temp,
    float max_motor_temp,
    float min_pcb_temp,
    float max_pcb_temp);

private:
  /**
   * @brief Send a packet to the controller
   * @param packet Pointer to packet data
   * @param size Size of packet in bytes
   * @return true if successful
   */
  bool send(uint8_t * packet, std::size_t size);

  /**
   * @brief Send settings packet
   */
  void sendSettings();

  /**
   * @brief Receive thread function
   */
  void rxThread();

  /**
   * @brief Handle received status packet
   * @param packet Pointer to status packet
   */
  void handlePacket(Xesc2040StatusPacket * packet);

  // Thread management
  std::thread rx_thread_;
  std::atomic<bool> rx_thread_run_{false};

  // Serial communication
  ErrorHandlerFunction error_handler_;
  serial::Serial serial_;
  std::string port_;

  // Status synchronization
  std::mutex status_mutex_;
  std::mutex serial_tx_mutex_;
  std::condition_variable status_cv_;
  Xesc2040StatusStruct status_;

  // Settings
  Xesc2040SettingsPacket settings_;
  bool settings_valid_{false};

  // Transmit buffer
  static constexpr std::size_t TX_BUFFER_SIZE = 1000;
  uint8_t tx_buffer_[TX_BUFFER_SIZE];
  boost::crc_ccitt_type tx_crc_;
};

}  // namespace xesc_2040_driver

#endif  // XESC_2040_DRIVER__XESC_2040_INTERFACE_HPP_

/**
 * @file xesc_yfr4_interface.hpp
 * @brief Serial interface for XESC YardForce R4 motor controller
 * 
 * Ported from open_mower_ros (ROS1) to ROS2
 * Original author: Clemens Elflein
 * 
 * Changes from ROS1:
 * - pthread replaced with std::thread
 * - Modern C++17 style
 */

#ifndef XESC_YFR4_DRIVER__XESC_YFR4_INTERFACE_HPP_
#define XESC_YFR4_DRIVER__XESC_YFR4_INTERFACE_HPP_

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

#include "xesc_yfr4_driver/cobs.hpp"
#include "xesc_yfr4_driver/xesc_yfr4_datatypes.hpp"

namespace xesc_yfr4_driver
{

/**
 * @brief Connection state for XESC YFR4
 */
enum class XescYFR4ConnectionState
{
  DISCONNECTED = 0,
  CONNECTED = 3
};

/**
 * @brief Status structure for XESC YFR4
 */
struct XescYFR4StatusStruct
{
  uint32_t seq{0};
  uint8_t fw_version_major{0};
  uint8_t fw_version_minor{0};
  XescYFR4ConnectionState connection_state{XescYFR4ConnectionState::DISCONNECTED};
  double temperature_pcb{0.0};     ///< PCB temperature (degrees Celsius)
  double current_input{0.0};       ///< Input current (ampere)
  double duty_cycle{0.0};          ///< Duty cycle (0 to 1)
  bool direction{false};           ///< Direction CW/CCW
  uint32_t tacho{0};               ///< Wheel ticks
  uint32_t tacho_absolute{0};      ///< Wheel ticks absolute
  uint16_t rpm{0};                 ///< Revolutions per minute
  int32_t fault_code{0};           ///< Fault code bitmask
};

/**
 * @brief Interface class for communicating with XESC YFR4 motor controller
 */
class XescYFR4Interface
{
public:
  using ErrorHandlerFunction = std::function<void(const std::string &)>;

  /**
   * @brief Constructor
   * @param error_handler Callback function for error handling
   */
  explicit XescYFR4Interface(const ErrorHandlerFunction & error_handler);

  /**
   * @brief Destructor - stops the interface
   */
  ~XescYFR4Interface();

  // Non-copyable
  XescYFR4Interface(const XescYFR4Interface &) = delete;
  XescYFR4Interface & operator=(const XescYFR4Interface &) = delete;

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
  void getStatus(XescYFR4StatusStruct * status);

  /**
   * @brief Wait for and get new status (blocking)
   * @param[out] status Status structure to populate
   */
  void waitForStatus(XescYFR4StatusStruct * status);

  /**
   * @brief Update controller settings
   */
  void updateSettings(
    float motor_current_limit,
    float min_pcb_temp,
    float max_pcb_temp);

private:
  /**
   * @brief Send a packet to the controller
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
   */
  void handlePacket(XescYFR4StatusPacket * packet);

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
  XescYFR4StatusStruct status_;

  // Settings
  XescYFR4SettingsPacket settings_;
  bool settings_valid_{false};

  // Transmit buffer
  static constexpr std::size_t TX_BUFFER_SIZE = 1000;
  uint8_t tx_buffer_[TX_BUFFER_SIZE];
  boost::crc_ccitt_type tx_crc_;
};

}  // namespace xesc_yfr4_driver

#endif  // XESC_YFR4_DRIVER__XESC_YFR4_INTERFACE_HPP_

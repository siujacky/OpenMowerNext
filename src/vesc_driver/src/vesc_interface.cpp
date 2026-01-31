/**
 * @file vesc_interface.cpp
 * @brief VESC Serial Interface implementation
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

#include "vesc_driver/vesc_interface.hpp"

#include <algorithm>

namespace vesc_driver
{

VescInterface::VescInterface(const ErrorHandlerFunction& error_handler, uint32_t state_request_millis)
  : serial_(std::string(), 115200, serial::Timeout::simpleTimeout(100), serial::eightbits, serial::parity_none,
            serial::stopbits_one, serial::flowcontrol_none)
  , state_request_millis_(state_request_millis)
{
  error_handler_ = error_handler;
}

VescInterface::~VescInterface()
{
  stop();
}

void VescInterface::updateThread()
{
  auto last_fw_request = std::chrono::steady_clock::now();

  while (update_thread_run_)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(state_request_millis_));

    auto now = std::chrono::steady_clock::now();
    VescConnectionState state;
    {
      std::unique_lock<std::mutex> lk(status_mutex_);
      state = status_.connection_state;
    }

    if (state == VescConnectionState::WAITING_FOR_FW)
    {
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fw_request);
      if (elapsed.count() > 1000)
      {
        last_fw_request = now;
        requestFWVersion();
      }
      continue;
    }
    else if (state == VescConnectionState::CONNECTED || state == VescConnectionState::CONNECTED_INCOMPATIBLE_FW)
    {
      requestState();
    }
  }
}

void VescInterface::rxThread()
{
  Buffer buffer;
  buffer.reserve(4096);

  {
    std::unique_lock<std::mutex> lk(status_mutex_);
    status_ = VescStatusStruct{};
    status_.connection_state = VescConnectionState::DISCONNECTED;
  }

  while (rx_thread_run_)
  {
    // Check if the serial port is connected. If not, connect to it.
    if (!serial_.isOpen())
    {
      {
        std::unique_lock<std::mutex> lk(status_mutex_);
        status_ = VescStatusStruct{};
        status_.connection_state = VescConnectionState::DISCONNECTED;
      }
      try
      {
        serial_.setPort(port_);
        serial_.open();
      }
      catch (const std::exception& e)
      {
        // Retry later
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      {
        std::unique_lock<std::mutex> lk(status_mutex_);
        status_.connection_state = VescConnectionState::WAITING_FOR_FW;
      }
    }

    int bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;
    if (!buffer.empty())
    {
      // Search buffer for valid packet(s)
      auto iter = buffer.begin();
      auto iter_begin = buffer.begin();
      while (iter != buffer.end())
      {
        // Check if valid start-of-frame character
        if (*iter == VescFrame::VESC_SOF_VAL_SMALL_FRAME || *iter == VescFrame::VESC_SOF_VAL_LARGE_FRAME)
        {
          // Good start, now attempt to create packet
          std::string error;
          VescPacketConstPtr packet = VescPacketFactory::createPacket(iter, buffer.end(), &bytes_needed, &error);

          if (packet)
          {
            // Good packet, check if we skipped any data
            if (std::distance(iter_begin, iter) > 0)
            {
              std::ostringstream ss;
              ss << "Out-of-sync with VESC, unknown data leading valid frame. Discarding "
                 << std::distance(iter_begin, iter) << " bytes.";
              error_handler_(ss.str());
            }
            // Call packet handler
            handlePacket(packet);
            // Update state
            iter = iter + static_cast<ptrdiff_t>(packet->getFrame().size());
            iter_begin = iter;
            // Continue to look for another frame in buffer
            continue;
          }
          else if (bytes_needed > 0)
          {
            // Need more data, break out of while loop
            break;
          }
          else
          {
            // This was not a packet, move on to next byte
            error_handler_(error);
          }
        }

        ++iter;
      }

      // If iter is at the end of the buffer, more bytes are needed
      if (iter == buffer.end())
      {
        bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;
      }

      // Erase "used" buffer
      if (std::distance(iter_begin, iter) > 0)
      {
        std::ostringstream ss;
        ss << "Out-of-sync with VESC, discarding " << std::distance(iter_begin, iter) << " bytes.";
        error_handler_(ss.str());
      }
      buffer.erase(buffer.begin(), iter);
    }

    // Attempt to read at least bytes_needed bytes from the serial port
    int bytes_to_read = std::max(bytes_needed, std::min(4096, static_cast<int>(serial_.available())));

    try
    {
      size_t bytes_read = serial_.read(buffer, static_cast<size_t>(bytes_to_read));
      if (bytes_needed > 0 && bytes_read == 0 && !buffer.empty())
      {
        error_handler_("Possibly out-of-sync with VESC, read timeout in the middle of a frame.");
      }
    }
    catch (const std::exception& e)
    {
      error_handler_("Error during serial read. Reconnecting.");
      {
        std::unique_lock<std::mutex> lk(status_mutex_);
        status_.connection_state = VescConnectionState::DISCONNECTED;
      }
      serial_.close();
    }
  }

  serial_.close();
}

void VescInterface::handlePacket(VescPacketConstPtr packet)
{
  VescConnectionState current_state;
  {
    std::lock_guard<std::mutex> lk(status_mutex_);
    current_state = status_.connection_state;
  }

  // Only update the state if connection state is connected
  if ((current_state == VescConnectionState::CONNECTED ||
       current_state == VescConnectionState::CONNECTED_INCOMPATIBLE_FW) &&
      packet->getName() == "Values")
  {
    std::lock_guard<std::mutex> lk(status_mutex_);
    auto values = std::dynamic_pointer_cast<VescPacketValues const>(packet);

    status_.seq++;
    status_.voltage_input = values->getInputVoltage();
    status_.temperature_pcb = values->getMosTemp();
    status_.temperature_motor = values->getMotorTemp();
    status_.current_motor = values->getMotorCurrent();
    status_.current_input = values->getInputCurrent();
    status_.speed_erpm = values->getVelocityERPM();
    status_.duty_cycle = values->getDuty();
    status_.charge_drawn = values->getConsumedCharge();
    status_.charge_regen = values->getInputCharge();
    status_.energy_drawn = values->getConsumedPower();
    status_.energy_regen = values->getInputPower();
    status_.displacement = values->getPosition();
    status_.distance_traveled = values->getDisplacement();
    status_.fault_code = values->getFaultCode();
    status_.tacho = values->getPosition();
    status_.tacho_absolute = values->getDisplacement();
    status_.direction = values->getVelocityERPM() < 0;
    status_cv_.notify_all();
  }
  else if (packet->getName() == "FWVersion")
  {
    std::lock_guard<std::mutex> lk(status_mutex_);
    auto fw_version = std::dynamic_pointer_cast<VescPacketFWVersion const>(packet);

    status_.seq++;
    status_.fw_version_major = static_cast<uint8_t>(fw_version->fwMajor());
    status_.fw_version_minor = static_cast<uint8_t>(fw_version->fwMinor());

    // Check for fully compatible FW here
    if (status_.fw_version_major == 5 && status_.fw_version_minor == 3)
    {
      status_.connection_state = VescConnectionState::CONNECTED;
    }
    else
    {
      status_.connection_state = VescConnectionState::CONNECTED_INCOMPATIBLE_FW;
    }

    status_cv_.notify_all();
  }
}

bool VescInterface::send(const VescPacket& packet)
{
  std::unique_lock<std::mutex> lk(serial_tx_mutex_);
  if (!serial_.isOpen())
  {
    return false;
  }
  size_t written = serial_.write(packet.getFrame());
  return written == packet.getFrame().size();
}

void VescInterface::requestFWVersion()
{
  send(VescPacketRequestFWVersion());
}

void VescInterface::requestState()
{
  send(VescPacketRequestValues());
}

void VescInterface::setDutyCycle(double duty_cycle)
{
  send(VescPacketSetDuty(duty_cycle));
}

void VescInterface::setCurrent(double current)
{
  send(VescPacketSetCurrent(current));
}

void VescInterface::setBrake(double brake)
{
  send(VescPacketSetCurrentBrake(brake));
}

void VescInterface::setSpeed(double speed)
{
  send(VescPacketSetVelocityERPM(speed));
}

void VescInterface::setPosition(double position)
{
  send(VescPacketSetPos(position));
}

void VescInterface::start(const std::string& port)
{
  port_ = port;
  // Start up monitoring threads
  rx_thread_run_ = true;
  update_thread_run_ = true;
  rx_thread_ = std::thread(&VescInterface::rxThread, this);
  update_thread_ = std::thread(&VescInterface::updateThread, this);
}

void VescInterface::stop()
{
  // Stop the motor
  setDutyCycle(0.0);

  // Tell the threads to stop
  rx_thread_run_ = false;
  update_thread_run_ = false;

  // Wait for threads to actually exit
  if (rx_thread_.joinable())
  {
    rx_thread_.join();
  }
  if (update_thread_.joinable())
  {
    update_thread_.join();
  }
}

void VescInterface::getStatus(VescStatusStruct* status)
{
  std::unique_lock<std::mutex> lk(status_mutex_);
  *status = status_;
}

void VescInterface::waitForStatus(VescStatusStruct* status)
{
  std::unique_lock<std::mutex> lk(status_mutex_);
  // Wait for new data
  status_cv_.wait(lk);
  *status = status_;
}

}  // namespace vesc_driver

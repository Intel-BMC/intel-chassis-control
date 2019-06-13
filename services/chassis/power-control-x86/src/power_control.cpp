/*
// Copyright (c) 2018-2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/
#include "i2c.hpp"

#include <gpiod.h>
#include <linux/aspeed-lpc-sio.h>
#include <sys/sysinfo.h>
#include <systemd/sd-journal.h>

#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/container/flat_map.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sdbusplus/asio/object_server.hpp>
#include <string_view>

namespace power_control
{
static boost::asio::io_service io;
std::shared_ptr<sdbusplus::asio::connection> conn;
static std::shared_ptr<sdbusplus::asio::dbus_interface> hostIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> chassisIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> powerButtonIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> resetButtonIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> nmiButtonIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> osIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> idButtonIface;

static struct gpiod_line* powerButtonMask = nullptr;
static struct gpiod_line* resetButtonMask = nullptr;
static bool nmiButtonMasked = false;

const static constexpr int powerPulseTimeMs = 200;
const static constexpr int forceOffPulseTimeMs = 15000;
const static constexpr int resetPulseTimeMs = 500;
const static constexpr int powerCycleTimeMs = 1000;
const static constexpr int sioPowerGoodWatchdogTimeMs = 1000;
const static constexpr int psPowerOKWatchdogTimeMs = 8000;
const static constexpr int gracefulPowerOffTimeMs = 60000;
const static constexpr int buttonMaskTimeMs = 60000;

const static std::filesystem::path powerControlDir = "/var/lib/power-control";
const static constexpr std::string_view powerDropFile = "power-drop";

// Timers
// Time holding GPIOs asserted
static boost::asio::steady_timer gpioAssertTimer(io);
// Time between off and on during a power cycle
static boost::asio::steady_timer powerCycleTimer(io);
// Time OS gracefully powering off
static boost::asio::steady_timer gracefulPowerOffTimer(io);
// Time power supply power OK assertion on power-on
static boost::asio::steady_timer psPowerOKWatchdogTimer(io);
// Time SIO power good assertion on power-on
static boost::asio::steady_timer sioPowerGoodWatchdogTimer(io);

// Event Descriptors
static boost::asio::posix::stream_descriptor psPowerOKEvent(io);
static boost::asio::posix::stream_descriptor sioPowerGoodEvent(io);
static boost::asio::posix::stream_descriptor sioOnControlEvent(io);
static boost::asio::posix::stream_descriptor sioS5Event(io);
static boost::asio::posix::stream_descriptor powerButtonEvent(io);
static boost::asio::posix::stream_descriptor resetButtonEvent(io);
static boost::asio::posix::stream_descriptor nmiButtonEvent(io);
static boost::asio::posix::stream_descriptor idButtonEvent(io);
static boost::asio::posix::stream_descriptor postCompleteEvent(io);

enum class PowerState
{
    on,
    waitForPSPowerOK,
    waitForSIOPowerGood,
    failedTransitionToOn,
    off,
    acLossOff,
    transitionToOff,
    gracefulTransitionToOff,
    cycleOff,
    transitionToCycleOff,
    gracefulTransitionToCycleOff,
};
static PowerState powerState;
static std::string getPowerStateName(PowerState state)
{
    switch (state)
    {
        case PowerState::on:
            return "On";
            break;
        case PowerState::waitForPSPowerOK:
            return "Wait for Power Supply Power OK";
            break;
        case PowerState::waitForSIOPowerGood:
            return "Wait for SIO Power Good";
            break;
        case PowerState::failedTransitionToOn:
            return "Failed Transition to On";
            break;
        case PowerState::off:
            return "Off";
            break;
        case PowerState::acLossOff:
            return "Off After AC Loss";
            break;
        case PowerState::transitionToOff:
            return "Transition to Off";
            break;
        case PowerState::gracefulTransitionToOff:
            return "Graceful Transition to Off";
            break;
        case PowerState::cycleOff:
            return "Power Cycle Off";
            break;
        case PowerState::transitionToCycleOff:
            return "Transition to Power Cycle Off";
            break;
        case PowerState::gracefulTransitionToCycleOff:
            return "Graceful Transition to Power Cycle Off";
            break;
        default:
            return "unknown state: " + std::to_string(static_cast<int>(state));
            break;
    }
}
static void logStateTransition(const PowerState state)
{
    std::cerr << "Moving to \"" << getPowerStateName(state) << "\" state.\n";
}

enum class Event
{
    psPowerOKAssert,
    psPowerOKDeAssert,
    sioPowerGoodAssert,
    sioPowerGoodDeAssert,
    sioS5Assert,
    sioS5DeAssert,
    powerButtonPressed,
    powerCycleTimerExpired,
    psPowerOKWatchdogTimerExpired,
    sioPowerGoodWatchdogTimerExpired,
    gracefulPowerOffTimerExpired,
    powerOnRequest,
    powerOffRequest,
    powerCycleRequest,
    resetRequest,
    gracefulPowerOffRequest,
    gracefulPowerCycleRequest,
};
static std::string getEventName(Event event)
{
    switch (event)
    {
        case Event::psPowerOKAssert:
            return "power supply power OK assert";
            break;
        case Event::psPowerOKDeAssert:
            return "power supply power OK de-assert";
            break;
        case Event::sioPowerGoodAssert:
            return "SIO power good assert";
            break;
        case Event::sioPowerGoodDeAssert:
            return "SIO power good de-assert";
            break;
        case Event::sioS5Assert:
            return "SIO S5 assert";
            break;
        case Event::sioS5DeAssert:
            return "SIO S5 de-assert";
            break;
        case Event::powerButtonPressed:
            return "power button pressed";
            break;
        case Event::powerCycleTimerExpired:
            return "power cycle timer expired";
            break;
        case Event::psPowerOKWatchdogTimerExpired:
            return "power supply power OK watchdog timer expired";
            break;
        case Event::sioPowerGoodWatchdogTimerExpired:
            return "SIO power good watchdog timer expired";
            break;
        case Event::gracefulPowerOffTimerExpired:
            return "graceful power-off timer expired";
            break;
        case Event::powerOnRequest:
            return "power-on request";
            break;
        case Event::powerOffRequest:
            return "power-off request";
            break;
        case Event::powerCycleRequest:
            return "power-cycle request";
            break;
        case Event::resetRequest:
            return "reset request";
            break;
        case Event::gracefulPowerOffRequest:
            return "graceful power-off request";
            break;
        case Event::gracefulPowerCycleRequest:
            return "graceful power-cycle request";
            break;
        default:
            return "unknown event: " + std::to_string(static_cast<int>(event));
            break;
    }
}
static void logEvent(const std::string_view stateHandler, const Event event)
{
    std::cerr << stateHandler << ": " << getEventName(event)
              << " event received.\n";
}

// Power state handlers
static void powerStateOn(const Event event);
static void powerStateWaitForPSPowerOK(const Event event);
static void powerStateWaitForSIOPowerGood(const Event event);
static void powerStateFailedTransitionToOn(const Event event);
static void powerStateOff(const Event event);
static void powerStateACLossOff(const Event event);
static void powerStateTransitionToOff(const Event event);
static void powerStateGracefulTransitionToOff(const Event event);
static void powerStateCycleOff(const Event event);
static void powerStateTransitionToCycleOff(const Event event);
static void powerStateGracefulTransitionToCycleOff(const Event event);

static std::function<void(const Event)> getPowerStateHandler(PowerState state)
{
    switch (state)
    {
        case PowerState::on:
            return powerStateOn;
            break;
        case PowerState::waitForPSPowerOK:
            return powerStateWaitForPSPowerOK;
            break;
        case PowerState::waitForSIOPowerGood:
            return powerStateWaitForSIOPowerGood;
            break;
        case PowerState::failedTransitionToOn:
            return powerStateFailedTransitionToOn;
            break;
        case PowerState::off:
            return powerStateOff;
            break;
        case PowerState::acLossOff:
            return powerStateACLossOff;
            break;
        case PowerState::transitionToOff:
            return powerStateTransitionToOff;
            break;
        case PowerState::gracefulTransitionToOff:
            return powerStateGracefulTransitionToOff;
            break;
        case PowerState::cycleOff:
            return powerStateCycleOff;
            break;
        case PowerState::transitionToCycleOff:
            return powerStateTransitionToCycleOff;
            break;
        case PowerState::gracefulTransitionToCycleOff:
            return powerStateGracefulTransitionToCycleOff;
            break;
        default:
            return std::function<void(const Event)>{};
            break;
    }
};

static void sendPowerControlEvent(const Event event)
{
    std::function<void(const Event)> handler = getPowerStateHandler(powerState);
    if (handler == nullptr)
    {
        std::cerr << "Failed to find handler for power state: "
                  << static_cast<int>(powerState) << "\n";
        return;
    }
    handler(event);
}

static constexpr std::string_view getHostState(const PowerState state)
{
    switch (state)
    {
        case PowerState::on:
        case PowerState::transitionToOff:
        case PowerState::gracefulTransitionToOff:
        case PowerState::transitionToCycleOff:
        case PowerState::gracefulTransitionToCycleOff:
            return "xyz.openbmc_project.State.Host.HostState.Running";
            break;
        case PowerState::waitForPSPowerOK:
        case PowerState::waitForSIOPowerGood:
        case PowerState::failedTransitionToOn:
        case PowerState::off:
        case PowerState::cycleOff:
        case PowerState::acLossOff:
            return "xyz.openbmc_project.State.Host.HostState.Off";
            break;
        default:
            return "";
            break;
    }
};
static constexpr std::string_view getChassisState(const PowerState state)
{
    switch (state)
    {
        case PowerState::on:
        case PowerState::transitionToOff:
        case PowerState::gracefulTransitionToOff:
        case PowerState::transitionToCycleOff:
        case PowerState::gracefulTransitionToCycleOff:
            return "xyz.openbmc_project.State.Chassis.PowerState.On";
            break;
        case PowerState::waitForPSPowerOK:
        case PowerState::waitForSIOPowerGood:
        case PowerState::failedTransitionToOn:
        case PowerState::off:
        case PowerState::cycleOff:
        case PowerState::acLossOff:
            return "xyz.openbmc_project.State.Chassis.PowerState.Off";
            break;
        default:
            return "";
            break;
    }
};
static void setPowerState(const PowerState state)
{
    powerState = state;
    logStateTransition(state);

    hostIface->set_property("CurrentHostState",
                            std::string(getHostState(powerState)));

    chassisIface->set_property("CurrentPowerState",
                               std::string(getChassisState(powerState)));
}

static void acOnLog()
{
    sd_journal_send("MESSAGE=PowerControl: AC lost PowerOn", "PRIORITY=%i",
                    LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.DCPowerOnAfterACLost", NULL);
}

static int initializePowerDropStorage()
{
    // create the power control directory if it doesn't exist
    std::error_code ec;
    if (!(std::filesystem::create_directories(power_control::powerControlDir,
                                              ec)))
    {
        if (ec.value() != 0)
        {
            std::cerr << "failed to create " << power_control::powerControlDir
                      << ": " << ec.message() << "\n";
            return -1;
        }
    }
    // Create the power drop file if it doesn't exist
    if (!std::filesystem::exists(power_control::powerControlDir /
                                 power_control::powerDropFile))
    {
        std::ofstream powerDrop(power_control::powerControlDir /
                                power_control::powerDropFile);
        powerDrop << "No";
    }
    return 0;
}

static void storePowerDrop()
{
    std::ofstream powerDropStream(powerControlDir / powerDropFile);
    powerDropStream << "Yes";
}

static void clearPowerDrop()
{
    std::ofstream powerDropStream(powerControlDir / powerDropFile);
    powerDropStream << "No";
}

static bool wasPowerDropped()
{
    std::ifstream powerDropStream(powerControlDir / powerDropFile);
    if (!powerDropStream.is_open())
    {
        std::cerr << "Failed to open power drop file\n";
        return false;
    }

    std::string drop;
    std::getline(powerDropStream, drop);
    return drop == "Yes";
}

static void invokePowerRestorePolicy(const std::string& policy)
{
    // Async events may call this twice, but we only want to run once
    static bool policyInvoked = false;
    if (policyInvoked)
    {
        return;
    }
    policyInvoked = true;

    std::cerr << "Power restore delay expired, invoking " << policy << "\n";
    if (policy ==
        "xyz.openbmc_project.Control.Power.RestorePolicy.Policy.AlwaysOn")
    {
        sendPowerControlEvent(Event::powerOnRequest);
    }
    else if (policy == "xyz.openbmc_project.Control.Power.RestorePolicy."
                       "Policy.Restore")
    {
        if (wasPowerDropped())
        {
            std::cerr << "Power was dropped, restoring Host On state\n";
            sendPowerControlEvent(Event::powerOnRequest);
        }
        else
        {
            std::cerr << "No power drop, restoring Host Off state\n";
        }
    }
}

static void powerRestorePolicyDelay(int delay)
{
    // Async events may call this twice, but we only want to run once
    static bool delayStarted = false;
    if (delayStarted)
    {
        return;
    }
    delayStarted = true;
    // Calculate the delay from now to meet the requested delay
    // Subtract the approximate uboot time
    static constexpr const int ubootSeconds = 20;
    delay -= ubootSeconds;
    // Subtract the time since boot
    struct sysinfo info = {};
    if (sysinfo(&info) == 0)
    {
        delay -= info.uptime;
    }
    // 0 is the minimum delay
    delay = std::max(delay, 0);

    static boost::asio::steady_timer powerRestorePolicyTimer(io);
    powerRestorePolicyTimer.expires_after(std::chrono::seconds(delay));
    std::cerr << "Power restore delay of " << delay << " seconds started\n";
    powerRestorePolicyTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                std::cerr << "power restore policy async_wait failed: "
                          << ec.message() << "\n";
            }
            return;
        }
        // Get Power Restore Policy
        // In case PowerRestorePolicy is not available, set a match for it
        static std::unique_ptr<sdbusplus::bus::match::match>
            powerRestorePolicyMatch = std::make_unique<
                sdbusplus::bus::match::match>(
                *conn,
                "type='signal',interface='org.freedesktop.DBus.Properties',"
                "member='PropertiesChanged',arg0namespace='xyz.openbmc_"
                "project.Control.Power.RestorePolicy'",
                [](sdbusplus::message::message& msg) {
                    std::string interfaceName;
                    boost::container::flat_map<std::string,
                                               std::variant<std::string>>
                        propertiesChanged;
                    std::string policy;
                    try
                    {
                        msg.read(interfaceName, propertiesChanged);
                        policy = std::get<std::string>(
                            propertiesChanged.begin()->second);
                    }
                    catch (std::exception& e)
                    {
                        std::cerr
                            << "Unable to read power restore policy value\n";
                        powerRestorePolicyMatch.reset();
                        return;
                    }
                    invokePowerRestorePolicy(policy);
                    powerRestorePolicyMatch.reset();
                });

        // Check if it's already on DBus
        conn->async_method_call(
            [](boost::system::error_code ec,
               const std::variant<std::string>& policyProperty) {
                if (ec)
                {
                    return;
                }
                powerRestorePolicyMatch.reset();
                const std::string* policy =
                    std::get_if<std::string>(&policyProperty);
                if (policy == nullptr)
                {
                    std::cerr << "Unable to read power restore policy value\n";
                    return;
                }
                invokePowerRestorePolicy(*policy);
            },
            "xyz.openbmc_project.Settings",
            "/xyz/openbmc_project/control/host0/power_restore_policy",
            "org.freedesktop.DBus.Properties", "Get",
            "xyz.openbmc_project.Control.Power.RestorePolicy",
            "PowerRestorePolicy");
    });
}

static void powerRestorePolicyStart()
{
    std::cerr << "Power restore policy started\n";

    // Get the desired delay time
    // In case PowerRestoreDelay is not available, set a match for it
    static std::unique_ptr<sdbusplus::bus::match::match>
        powerRestoreDelayMatch = std::make_unique<sdbusplus::bus::match::match>(
            *conn,
            "type='signal',interface='org.freedesktop.DBus.Properties',member='"
            "PropertiesChanged',arg0namespace='xyz.openbmc_project.Control."
            "Power.RestoreDelay'",
            [](sdbusplus::message::message& msg) {
                std::string interfaceName;
                boost::container::flat_map<std::string, std::variant<uint16_t>>
                    propertiesChanged;
                int delay = 0;
                try
                {
                    msg.read(interfaceName, propertiesChanged);
                    delay =
                        std::get<uint16_t>(propertiesChanged.begin()->second);
                }
                catch (std::exception& e)
                {
                    std::cerr << "Unable to read power restore delay value\n";
                    powerRestoreDelayMatch.reset();
                    return;
                }
                powerRestorePolicyDelay(delay);
                powerRestoreDelayMatch.reset();
            });

    // Check if it's already on DBus
    conn->async_method_call(
        [](boost::system::error_code ec,
           const std::variant<uint16_t>& delayProperty) {
            if (ec)
            {
                return;
            }
            powerRestoreDelayMatch.reset();
            const uint16_t* delay = std::get_if<uint16_t>(&delayProperty);
            if (delay == nullptr)
            {
                std::cerr << "Unable to read power restore delay value\n";
                return;
            }
            powerRestorePolicyDelay(*delay);
        },
        "xyz.openbmc_project.Settings",
        "/xyz/openbmc_project/control/power_restore_delay",
        "org.freedesktop.DBus.Properties", "Get",
        "xyz.openbmc_project.Control.Power.RestoreDelay", "PowerRestoreDelay");
}

static gpiod_line* requestGPIOEvents(
    const std::string_view name, const std::function<void()>& handler,
    boost::asio::posix::stream_descriptor& gpioEventDescriptor)
{
    // Find the GPIO line
    struct gpiod_line* gpioLine = gpiod_line_find(name.data());
    if (gpioLine == NULL)
    {
        std::cerr << "Failed to find the " << name << " line\n";
        return nullptr;
    }

    if (gpiod_line_request_both_edges_events(gpioLine, "power_control") < 0)
    {
        std::cerr << "Failed to request events for " << name << "\n";
        return nullptr;
    }

    int gpioLineFd = gpiod_line_event_get_fd(gpioLine);
    if (gpioLineFd < 0)
    {
        std::cerr << "Failed to get " << name << " fd\n";
        return nullptr;
    }

    gpioEventDescriptor.assign(gpioLineFd);

    gpioEventDescriptor.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [name, handler](const boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << name << " fd handler error: " << ec.message()
                          << "\n";
                // TODO: throw here to force power-control to restart?
                return;
            }
            handler();
        });
    return gpioLine;
}

static struct gpiod_line* setGPIOOutput(const std::string_view name,
                                        const int value)
{
    // Find the GPIO line
    struct gpiod_line* gpioLine = gpiod_line_find(name.data());
    if (gpioLine == NULL)
    {
        std::cerr << "Failed to find the " << name << " line.\n";
        return nullptr;
    }

    // Request GPIO output to specified value
    if (gpiod_line_request_output(gpioLine, __FUNCTION__, value) < 0)
    {
        std::cerr << "Failed to request " << name << " output\n";
        return nullptr;
    }

    std::cerr << name << " set to " << std::to_string(value) << "\n";
    return gpioLine;
}

static int setMaskedGPIOOutputForMs(struct gpiod_line* maskedGPIOLine,
                                    const std::string_view name,
                                    const int value, const int durationMs)
{
    // Set the masked GPIO line to the specified value
    if (gpiod_line_set_value(maskedGPIOLine, value) < 0)
    {
        return -1;
    }
    std::cerr << name << " set to " << std::to_string(value);
    gpioAssertTimer.expires_after(std::chrono::milliseconds(durationMs));
    gpioAssertTimer.async_wait(
        [maskedGPIOLine, value, name](const boost::system::error_code ec) {
            // Set the masked GPIO line back to the opposite value
            if (gpiod_line_set_value(maskedGPIOLine, !value) < 0)
            {
                std::cerr << name << " failed to release\n";
                return;
            }
            std::cerr << name << " released\n";
            if (ec)
            {
                // operation_aborted is expected if timer is canceled before
                // completion.
                if (ec != boost::asio::error::operation_aborted)
                {
                    std::cerr << name << " async_wait failed: " + ec.message()
                              << "\n";
                }
            }
        });
    return 0;
}

static int setGPIOOutputForMs(const std::string_view name, const int value,
                              const int durationMs)
{
    // If the requested GPIO is masked, use the mask line to set the output
    if (powerButtonMask != nullptr && name == "POWER_OUT")
    {
        return setMaskedGPIOOutputForMs(powerButtonMask, name, value,
                                        durationMs);
    }
    if (resetButtonMask != nullptr && name == "RESET_OUT")
    {
        return setMaskedGPIOOutputForMs(resetButtonMask, name, value,
                                        durationMs);
    }

    // No mask set, so request and set the GPIO normally
    struct gpiod_line* gpioLine = setGPIOOutput(name, value);
    if (gpioLine == nullptr)
    {
        return -1;
    }
    gpioAssertTimer.expires_after(std::chrono::milliseconds(durationMs));
    gpioAssertTimer.async_wait(
        [gpioLine, name](const boost::system::error_code ec) {
            gpiod_line_close_chip(gpioLine);
            std::cerr << name << " released\n";
            if (ec)
            {
                // operation_aborted is expected if timer is canceled before
                // completion.
                if (ec != boost::asio::error::operation_aborted)
                {
                    std::cerr << name << " async_wait failed: " << ec.message()
                              << "\n";
                }
            }
        });
    return 0;
}

static void powerOn()
{
    setGPIOOutputForMs("POWER_OUT", 0, powerPulseTimeMs);
}

static void gracefulPowerOff()
{
    setGPIOOutputForMs("POWER_OUT", 0, powerPulseTimeMs);
}

static void forcePowerOff()
{
    if (setGPIOOutputForMs("POWER_OUT", 0, forceOffPulseTimeMs) < 0)
    {
        return;
    }

    // If the force off timer expires, then the PCH power-button override
    // failed, so attempt the Unconditional Powerdown SMBus command.
    gpioAssertTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                std::cerr << "Force power off async_wait failed: "
                          << ec.message() << "\n";
            }
            return;
        }
        std::cerr << "PCH Power-button override failed. Issuing Unconditional "
                     "Powerdown SMBus command.\n";
        const static constexpr size_t pchDevBusAddress = 3;
        const static constexpr size_t pchDevSlaveAddress = 0x44;
        const static constexpr size_t pchCmdReg = 0;
        const static constexpr size_t pchPowerDownCmd = 0x02;
        if (i2cSet(pchDevBusAddress, pchDevSlaveAddress, pchCmdReg,
                   pchPowerDownCmd) < 0)
        {
            std::cerr << "Unconditional Powerdown command failed! Not sure "
                         "what to do now.\n";
        }
    });
}

static void reset()
{
    setGPIOOutputForMs("RESET_OUT", 0, resetPulseTimeMs);
}

static void gracefulPowerOffTimerStart()
{
    std::cerr << "Graceful power-off timer started\n";
    gracefulPowerOffTimer.expires_after(
        std::chrono::milliseconds(gracefulPowerOffTimeMs));
    gracefulPowerOffTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                std::cerr << "Graceful power-off async_wait failed: "
                          << ec.message() << "\n";
            }
            std::cerr << "Graceful power-off timer canceled\n";
            return;
        }
        std::cerr << "Graceful power-off timer completed\n";
        sendPowerControlEvent(Event::gracefulPowerOffTimerExpired);
    });
}

static void powerCycleTimerStart()
{
    std::cerr << "Power-cycle timer started\n";
    powerCycleTimer.expires_after(std::chrono::milliseconds(powerCycleTimeMs));
    powerCycleTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                std::cerr << "Power-cycle async_wait failed: " << ec.message()
                          << "\n";
            }
            std::cerr << "Power-cycle timer canceled\n";
            return;
        }
        std::cerr << "Power-cycle timer completed\n";
        sendPowerControlEvent(Event::powerCycleTimerExpired);
    });
}

static void psPowerOKWatchdogTimerStart()
{
    std::cerr << "power supply power OK watchdog timer started\n";
    psPowerOKWatchdogTimer.expires_after(
        std::chrono::milliseconds(psPowerOKWatchdogTimeMs));
    psPowerOKWatchdogTimer.async_wait(
        [](const boost::system::error_code ec) {
            if (ec)
            {
                // operation_aborted is expected if timer is canceled before
                // completion.
                if (ec != boost::asio::error::operation_aborted)
                {
                    std::cerr
                        << "power supply power OK watchdog async_wait failed: "
                        << ec.message() << "\n";
                }
                std::cerr << "power supply power OK watchdog timer canceled\n";
                return;
            }
            std::cerr << "power supply power OK watchdog timer expired\n";
            sendPowerControlEvent(Event::psPowerOKWatchdogTimerExpired);
        });
}

static void sioPowerGoodWatchdogTimerStart()
{
    std::cerr << "SIO power good watchdog timer started\n";
    sioPowerGoodWatchdogTimer.expires_after(
        std::chrono::milliseconds(sioPowerGoodWatchdogTimeMs));
    sioPowerGoodWatchdogTimer.async_wait(
        [](const boost::system::error_code ec) {
            if (ec)
            {
                // operation_aborted is expected if timer is canceled before
                // completion.
                if (ec != boost::asio::error::operation_aborted)
                {
                    std::cerr << "SIO power good watchdog async_wait failed: "
                              << ec.message() << "\n";
                }
                std::cerr << "SIO power good watchdog timer canceled\n";
                return;
            }
            std::cerr << "SIO power good watchdog timer completed\n";
            sendPowerControlEvent(Event::sioPowerGoodWatchdogTimerExpired);
        });
}

static void powerStateOn(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKDeAssert:
            storePowerDrop();
            setPowerState(PowerState::off);
            break;
        case Event::sioS5Assert:
            setPowerState(PowerState::transitionToOff);
            break;
        case Event::powerButtonPressed:
            setPowerState(PowerState::gracefulTransitionToOff);
            gracefulPowerOffTimerStart();
            break;
        case Event::powerOffRequest:
            setPowerState(PowerState::transitionToOff);
            forcePowerOff();
            break;
        case Event::gracefulPowerOffRequest:
            setPowerState(PowerState::gracefulTransitionToOff);
            gracefulPowerOffTimerStart();
            gracefulPowerOff();
            break;
        case Event::powerCycleRequest:
            setPowerState(PowerState::transitionToCycleOff);
            forcePowerOff();
            break;
        case Event::gracefulPowerCycleRequest:
            setPowerState(PowerState::gracefulTransitionToCycleOff);
            gracefulPowerOffTimerStart();
            gracefulPowerOff();
            break;
        case Event::resetRequest:
            reset();
            break;
        default:
            std::cerr << "No action taken.\n";
            break;
    }
}

static void powerStateWaitForPSPowerOK(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKAssert:
            // Cancel any GPIO assertions held during the transition
            gpioAssertTimer.cancel();
            psPowerOKWatchdogTimer.cancel();
            sioPowerGoodWatchdogTimerStart();
            setPowerState(PowerState::waitForSIOPowerGood);
            break;
        case Event::psPowerOKWatchdogTimerExpired:
            setPowerState(PowerState::failedTransitionToOn);
            break;
        default:
            std::cerr << "No action taken.\n";
            break;
    }
}

static void powerStateWaitForSIOPowerGood(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::sioPowerGoodAssert:
            sioPowerGoodWatchdogTimer.cancel();
            setPowerState(PowerState::on);
            break;
        case Event::sioPowerGoodWatchdogTimerExpired:
            setPowerState(PowerState::failedTransitionToOn);
            forcePowerOff();
            break;
        default:
            std::cerr << "No action taken.\n";
            break;
    }
}

static void powerStateFailedTransitionToOn(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKAssert:
            // We're in a failure state, so don't allow the system to turn on
            // without a user request
            forcePowerOff();
            break;
        case Event::psPowerOKDeAssert:
            // Cancel any GPIO assertions held during the transition
            gpioAssertTimer.cancel();
            break;
        case Event::powerButtonPressed:
            psPowerOKWatchdogTimerStart();
            setPowerState(PowerState::waitForPSPowerOK);
            break;
        case Event::powerOnRequest:
            psPowerOKWatchdogTimerStart();
            setPowerState(PowerState::waitForPSPowerOK);
            powerOn();
            break;
        default:
            std::cerr << "No action taken.\n";
            break;
    }
}

static void powerStateOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKAssert:
            clearPowerDrop();
            setPowerState(PowerState::waitForSIOPowerGood);
            break;
        case Event::powerButtonPressed:
            clearPowerDrop();
            psPowerOKWatchdogTimerStart();
            setPowerState(PowerState::waitForPSPowerOK);
            break;
        case Event::powerOnRequest:
            clearPowerDrop();
            psPowerOKWatchdogTimerStart();
            setPowerState(PowerState::waitForPSPowerOK);
            powerOn();
            break;
        default:
            std::cerr << "No action taken.\n";
            break;
    }
}

static void powerStateACLossOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKAssert:
            acOnLog();
            clearPowerDrop();
            setPowerState(PowerState::waitForSIOPowerGood);
            break;
        case Event::powerButtonPressed:
            acOnLog();
            psPowerOKWatchdogTimerStart();
            clearPowerDrop();
            setPowerState(PowerState::waitForPSPowerOK);
            break;
        case Event::powerOnRequest:
            acOnLog();
            psPowerOKWatchdogTimerStart();
            clearPowerDrop();
            setPowerState(PowerState::waitForPSPowerOK);
            powerOn();
            break;
        default:
            std::cerr << "No action taken.\n";
            break;
    }
}

static void powerStateTransitionToOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKDeAssert:
            // Cancel any GPIO assertions held during the transition
            gpioAssertTimer.cancel();
            setPowerState(PowerState::off);
            break;
        default:
            std::cerr << "No action taken.\n";
            break;
    }
}

static void powerStateGracefulTransitionToOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKDeAssert:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::off);
            break;
        case Event::gracefulPowerOffTimerExpired:
            setPowerState(PowerState::on);
            break;
        default:
            std::cerr << "No action taken.\n";
            break;
    }
}

static void powerStateCycleOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::powerCycleTimerExpired:
            psPowerOKWatchdogTimerStart();
            setPowerState(PowerState::waitForPSPowerOK);
            powerOn();
            break;
        default:
            std::cerr << "No action taken.\n";
            break;
    }
}

static void powerStateTransitionToCycleOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKDeAssert:
            // Cancel any GPIO assertions held during the transition
            gpioAssertTimer.cancel();
            setPowerState(PowerState::cycleOff);
            powerCycleTimerStart();
            break;
        default:
            std::cerr << "No action taken.\n";
            break;
    }
}

static void powerStateGracefulTransitionToCycleOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKDeAssert:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::cycleOff);
            powerCycleTimerStart();
            break;
        case Event::gracefulPowerOffTimerExpired:
            setPowerState(PowerState::on);
            break;
        default:
            std::cerr << "No action taken.\n";
            break;
    }
}

bool isACBoot()
{
    struct sio_ioctl_data sioData = {};

    int fd = open("/dev/lpc-sio", O_RDWR | O_CLOEXEC);
    if (fd < 0)
    {
        std::cerr << "Failed to open lpc-sio\n";
        return false;
    }

    sioData.sio_cmd = SIO_GET_PFAIL_STATUS;
    sioData.param = 0;
    if (ioctl(fd, SIO_IOC_COMMAND, &sioData) < 0)
    {

        std::cerr << "ioctl SIO_GET_PFAIL_STATUS error\n";
        sioData.data = 0;
    }

    close(fd);
    return (sioData.data != 0);
}

static void psPowerOKHandler()
{
    struct gpiod_line_event gpioLineEvent;

    if (gpiod_line_event_read_fd(psPowerOKEvent.native_handle(),
                                 &gpioLineEvent) < 0)
    {
        std::cerr << "failed to read gpioLineEvent from fd: "
                  << psPowerOKEvent.native_handle() << "\n";
        return;
    }
    Event powerControlEvent =
        gpioLineEvent.event_type == GPIOD_LINE_EVENT_RISING_EDGE
            ? Event::psPowerOKAssert
            : Event::psPowerOKDeAssert;

    sendPowerControlEvent(powerControlEvent);
    psPowerOKEvent.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [](const boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << "power supply power OK handler error: "
                          << ec.message() << "\n";
                return;
            }
            psPowerOKHandler();
        });
}

static void sioPowerGoodHandler()
{
    struct gpiod_line_event gpioLineEvent;

    if (gpiod_line_event_read_fd(sioPowerGoodEvent.native_handle(),
                                 &gpioLineEvent) < 0)
    {
        std::cerr << "failed to read gpioLineEvent from fd: "
                  << sioPowerGoodEvent.native_handle() << "\n";
        return;
    }
    Event powerControlEvent =
        gpioLineEvent.event_type == GPIOD_LINE_EVENT_RISING_EDGE
            ? Event::sioPowerGoodAssert
            : Event::sioPowerGoodDeAssert;

    sendPowerControlEvent(powerControlEvent);
    sioPowerGoodEvent.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [](const boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << "SIO power good handler error: " << ec.message()
                          << "\n";
                return;
            }
            sioPowerGoodHandler();
        });
}

static void sioOnControlHandler()
{
    struct gpiod_line_event gpioLineEvent;

    if (gpiod_line_event_read_fd(sioOnControlEvent.native_handle(),
                                 &gpioLineEvent) < 0)
    {
        std::cerr << "failed to read gpioLineEvent from fd: "
                  << sioOnControlEvent.native_handle() << "\n";
        return;
    }
    bool sioOnControl =
        gpioLineEvent.event_type == GPIOD_LINE_EVENT_RISING_EDGE;
    std::cerr << "SIO_ONCONTROL value changed: " << sioOnControl << "\n";
    sioOnControlEvent.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [](const boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << "SIO ONCONTROL handler error: " << ec.message()
                          << "\n";
                return;
            }
            sioOnControlHandler();
        });
}

static void sioS5Handler()
{
    struct gpiod_line_event gpioLineEvent;

    if (gpiod_line_event_read_fd(sioS5Event.native_handle(), &gpioLineEvent) <
        0)
    {
        std::cerr << "failed to read gpioLineEvent from fd: "
                  << sioS5Event.native_handle() << "\n";
        return;
    }
    Event powerControlEvent =
        gpioLineEvent.event_type == GPIOD_LINE_EVENT_FALLING_EDGE
            ? Event::sioS5Assert
            : Event::sioS5DeAssert;

    sendPowerControlEvent(powerControlEvent);
    sioS5Event.async_wait(boost::asio::posix::stream_descriptor::wait_read,
                          [](const boost::system::error_code ec) {
                              if (ec)
                              {
                                  std::cerr << "SIO S5 handler error: "
                                            << ec.message() << "\n";
                                  return;
                              }
                              sioS5Handler();
                          });
}

static void powerButtonHandler()
{
    struct gpiod_line_event gpioLineEvent;

    if (gpiod_line_event_read_fd(powerButtonEvent.native_handle(),
                                 &gpioLineEvent) < 0)
    {
        std::cerr << "failed to read gpioLineEvent from fd: "
                  << powerButtonEvent.native_handle() << "\n";
        return;
    }
    if (gpioLineEvent.event_type == GPIOD_LINE_EVENT_FALLING_EDGE)
    {
        powerButtonIface->set_property("ButtonPressed", true);
        if (powerButtonMask == nullptr)
        {
            sendPowerControlEvent(Event::powerButtonPressed);
        }
        else
        {
            std::cerr << "power button press masked\n";
        }
    }
    else if (gpioLineEvent.event_type == GPIOD_LINE_EVENT_RISING_EDGE)
    {
        powerButtonIface->set_property("ButtonPressed", false);
    }
    powerButtonEvent.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [](const boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << "power button handler error: " << ec.message()
                          << "\n";
                return;
            }
            powerButtonHandler();
        });
}

static void resetButtonHandler()
{
    struct gpiod_line_event gpioLineEvent;

    if (gpiod_line_event_read_fd(resetButtonEvent.native_handle(),
                                 &gpioLineEvent) < 0)
    {
        std::cerr << "failed to read gpioLineEvent from fd: "
                  << resetButtonEvent.native_handle() << "\n";
        return;
    }
    if (gpioLineEvent.event_type == GPIOD_LINE_EVENT_FALLING_EDGE)
    {
        resetButtonIface->set_property("ButtonPressed", true);
        if (resetButtonMask != nullptr)
        {
            std::cerr << "reset button press masked\n";
        }
    }
    else if (gpioLineEvent.event_type == GPIOD_LINE_EVENT_RISING_EDGE)
    {
        resetButtonIface->set_property("ButtonPressed", false);
    }
    resetButtonEvent.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [](const boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << "reset button handler error: " << ec.message()
                          << "\n";
                return;
            }
            resetButtonHandler();
        });
}

static void nmiButtonHandler()
{
    struct gpiod_line_event gpioLineEvent;

    if (gpiod_line_event_read_fd(nmiButtonEvent.native_handle(),
                                 &gpioLineEvent) < 0)
    {
        std::cerr << "failed to read gpioLineEvent from fd: "
                  << nmiButtonEvent.native_handle() << "\n";
        return;
    }
    if (gpioLineEvent.event_type == GPIOD_LINE_EVENT_FALLING_EDGE)
    {
        nmiButtonIface->set_property("ButtonPressed", true);
        if (nmiButtonMasked)
        {
            std::cerr << "NMI button press masked\n";
        }
    }
    else if (gpioLineEvent.event_type == GPIOD_LINE_EVENT_RISING_EDGE)
    {
        nmiButtonIface->set_property("ButtonPressed", false);
    }
    nmiButtonEvent.async_wait(boost::asio::posix::stream_descriptor::wait_read,
                              [](const boost::system::error_code ec) {
                                  if (ec)
                                  {
                                      std::cerr << "NMI button handler error: "
                                                << ec.message() << "\n";
                                      return;
                                  }
                                  nmiButtonHandler();
                              });
}

static void idButtonHandler()
{
    struct gpiod_line_event gpioLineEvent;

    if (gpiod_line_event_read_fd(idButtonEvent.native_handle(),
                                 &gpioLineEvent) < 0)
    {
        std::cerr << "failed to read gpioLineEvent from fd: "
                  << idButtonEvent.native_handle() << "\n";
        return;
    }
    if (gpioLineEvent.event_type == GPIOD_LINE_EVENT_FALLING_EDGE)
    {
        idButtonIface->set_property("ButtonPressed", true);
    }
    else if (gpioLineEvent.event_type == GPIOD_LINE_EVENT_RISING_EDGE)
    {
        idButtonIface->set_property("ButtonPressed", false);
    }
    idButtonEvent.async_wait(boost::asio::posix::stream_descriptor::wait_read,
                             [](const boost::system::error_code& ec) {
                                 if (ec)
                                 {
                                     std::cerr << "ID button handler error: "
                                               << ec.message() << "\n";
                                     return;
                                 }
                                 idButtonHandler();
                             });
}

static void postCompleteHandler()
{
    struct gpiod_line_event event;

    if (gpiod_line_event_read_fd(postCompleteEvent.native_handle(), &event) < 0)
    {
        std::cerr << "failed to read event from fd: "
                  << postCompleteEvent.native_handle() << "\n";
        return;
    }
    bool postComplete = event.event_type == GPIOD_LINE_EVENT_FALLING_EDGE;
    std::cerr << "POST complete value changed: " << postComplete << "\n";
    if (postComplete)
    {
        osIface->set_property("OperatingSystemState", std::string("Standby"));
    }
    else
    {
        osIface->set_property("OperatingSystemState", std::string("Inactive"));
    }
    postCompleteEvent.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [](const boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << "POST complete handler error: " << ec.message()
                          << "\n";
                return;
            }
            postCompleteHandler();
        });
}
} // namespace power_control

int main(int argc, char* argv[])
{
    std::cerr << "Start Chassis power control service...\n";
    power_control::conn =
        std::make_shared<sdbusplus::asio::connection>(power_control::io);

    // Request all the dbus names
    power_control::conn->request_name("xyz.openbmc_project.State.Host");
    power_control::conn->request_name("xyz.openbmc_project.State.Chassis");
    power_control::conn->request_name(
        "xyz.openbmc_project.State.OperatingSystem");
    power_control::conn->request_name("xyz.openbmc_project.Chassis.Buttons");

    // Initialize the power drop storage
    if (power_control::initializePowerDropStorage() < 0)
    {
        return -1;
    }

    // Request PS_PWROK GPIO events
    struct gpiod_line* powerGoodLine = power_control::requestGPIOEvents(
        "PS_PWROK", power_control::psPowerOKHandler,
        power_control::psPowerOKEvent);
    if (powerGoodLine == nullptr)
    {
        return -1;
    }

    // Request SIO_POWER_GOOD GPIO events
    struct gpiod_line* sioPowerGoodLine = power_control::requestGPIOEvents(
        "SIO_POWER_GOOD", power_control::sioPowerGoodHandler,
        power_control::sioPowerGoodEvent);
    if (sioPowerGoodLine == nullptr)
    {
        return -1;
    }

    // Request SIO_ONCONTROL GPIO events
    struct gpiod_line* sioOnControlLine = power_control::requestGPIOEvents(
        "SIO_ONCONTROL", power_control::sioOnControlHandler,
        power_control::sioOnControlEvent);
    if (sioOnControlLine == nullptr)
    {
        return -1;
    }

    // Request SIO_S5 GPIO events
    struct gpiod_line* sioS5Line = power_control::requestGPIOEvents(
        "SIO_S5", power_control::sioS5Handler, power_control::sioS5Event);
    if (sioS5Line == nullptr)
    {
        return -1;
    }

    // Request POWER_BUTTON GPIO events
    struct gpiod_line* powerButtonLine = power_control::requestGPIOEvents(
        "POWER_BUTTON", power_control::powerButtonHandler,
        power_control::powerButtonEvent);
    if (powerButtonLine == nullptr)
    {
        return -1;
    }

    // Request RESET_BUTTON GPIO events
    struct gpiod_line* resetButtonLine = power_control::requestGPIOEvents(
        "RESET_BUTTON", power_control::resetButtonHandler,
        power_control::resetButtonEvent);
    if (resetButtonLine == nullptr)
    {
        return -1;
    }

    // Request NMI_BUTTON GPIO events
    struct gpiod_line* nmiButtonLine = power_control::requestGPIOEvents(
        "NMI_BUTTON", power_control::nmiButtonHandler,
        power_control::nmiButtonEvent);
    if (nmiButtonLine == nullptr)
    {
        return -1;
    }

    // Request ID_BUTTON GPIO events
    struct gpiod_line* idButtonLine = power_control::requestGPIOEvents(
        "ID_BUTTON", power_control::idButtonHandler,
        power_control::idButtonEvent);
    if (idButtonLine == nullptr)
    {
        return -1;
    }

    // Request POST_COMPLETE GPIO events
    struct gpiod_line* postCompleteLine = power_control::requestGPIOEvents(
        "POST_COMPLETE", power_control::postCompleteHandler,
        power_control::postCompleteEvent);
    if (postCompleteLine == nullptr)
    {
        return -1;
    }

    // Initialize the power state
    power_control::powerState = power_control::PowerState::off;
    // Check power good
    if (gpiod_line_get_value(powerGoodLine) > 0)
    {
        power_control::powerState = power_control::PowerState::on;
    }

    // Check if this is an AC boot
    if (power_control::isACBoot())
    {
        // This is an AC boot, so log the AC boot event on the next boot
        // If we're on, log the AC boot event now
        if (power_control::powerState == power_control::PowerState::on)
        {
            power_control::acOnLog();
        }
        else
        {
            // Since we're off, log the AC boot event at power on, by starting
            // in the AC Loss Off state
            power_control::powerState = power_control::PowerState::acLossOff;
        }

        // Start the Power Restore policy
        power_control::powerRestorePolicyStart();
    }
    std::cerr << "Initializing power state. ";
    power_control::logStateTransition(power_control::powerState);

    // Power Control Service
    sdbusplus::asio::object_server hostServer =
        sdbusplus::asio::object_server(power_control::conn);

    // Power Control Interface
    power_control::hostIface = hostServer.add_interface(
        "/xyz/openbmc_project/state/host0", "xyz.openbmc_project.State.Host");

    power_control::hostIface->register_property(
        "RequestedHostTransition",
        std::string("xyz.openbmc_project.State.Host.Transition.Off"),
        [](const std::string& requested, std::string& resp) {
            if (requested == "xyz.openbmc_project.State.Host.Transition.Off")
            {
                sendPowerControlEvent(
                    power_control::Event::gracefulPowerOffRequest);
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Host.Transition.On")
            {
                sendPowerControlEvent(power_control::Event::powerOnRequest);
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Host.Transition.Reboot")
            {
                sendPowerControlEvent(
                    power_control::Event::gracefulPowerCycleRequest);
            }
            else
            {
                std::cerr << "Unrecognized host state transition request.\n";
                throw std::invalid_argument("Unrecognized Transition Request");
                return 0;
            }
            resp = requested;
            return 1;
        });
    power_control::hostIface->register_property(
        "CurrentHostState",
        std::string(power_control::getHostState(power_control::powerState)));

    power_control::hostIface->initialize();

    // Chassis Control Service
    sdbusplus::asio::object_server chassisServer =
        sdbusplus::asio::object_server(power_control::conn);

    // Chassis Control Interface
    power_control::chassisIface =
        chassisServer.add_interface("/xyz/openbmc_project/state/chassis0",
                                    "xyz.openbmc_project.State.Chassis");

    power_control::chassisIface->register_property(
        "RequestedPowerTransition",
        std::string("xyz.openbmc_project.State.Chassis.Transition.Off"),
        [](const std::string& requested, std::string& resp) {
            if (requested == "xyz.openbmc_project.State.Chassis.Transition.Off")
            {
                sendPowerControlEvent(power_control::Event::powerOffRequest);
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Chassis.Transition.On")
            {
                sendPowerControlEvent(power_control::Event::powerOnRequest);
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Chassis.Transition.PowerCycle")
            {
                sendPowerControlEvent(power_control::Event::powerCycleRequest);
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Chassis.Transition.Reset")
            {
                sendPowerControlEvent(power_control::Event::resetRequest);
            }
            else
            {
                std::cerr << "Unrecognized chassis state transition request.\n";
                throw std::invalid_argument("Unrecognized Transition Request");
                return 0;
            }
            resp = requested;
            return 1;
        });
    power_control::chassisIface->register_property(
        "CurrentPowerState",
        std::string(power_control::getChassisState(power_control::powerState)));

    power_control::chassisIface->initialize();

    // Buttons Service
    sdbusplus::asio::object_server buttonsServer =
        sdbusplus::asio::object_server(power_control::conn);

    // Power Button Interface
    power_control::powerButtonIface = buttonsServer.add_interface(
        "/xyz/openbmc_project/chassis/buttons/power",
        "xyz.openbmc_project.Chassis.Buttons");

    power_control::powerButtonIface->register_property(
        "ButtonMasked", false, [](const bool requested, bool& current) {
            if (requested)
            {
                if (power_control::powerButtonMask != nullptr)
                {
                    return 1;
                }
                power_control::powerButtonMask =
                    power_control::setGPIOOutput("POWER_OUT", 1);
                if (power_control::powerButtonMask == nullptr)
                {
                    throw std::runtime_error("Failed to request GPIO");
                    return 0;
                }
                std::cerr << "Power Button Masked.\n";
            }
            else
            {
                if (power_control::powerButtonMask == nullptr)
                {
                    return 1;
                }
                std::cerr << "Power Button Un-masked\n";
                gpiod_line_close_chip(power_control::powerButtonMask);
                power_control::powerButtonMask = nullptr;
            }
            // Update the mask setting
            current = requested;
            return 1;
        });

    // Check power button state
    bool powerButtonPressed = false;
    if (gpiod_line_get_value(powerButtonLine) == 0)
    {
        powerButtonPressed = true;
    }
    power_control::powerButtonIface->register_property("ButtonPressed",
                                                       powerButtonPressed);

    power_control::powerButtonIface->initialize();

    // Reset Button Interface
    power_control::resetButtonIface = buttonsServer.add_interface(
        "/xyz/openbmc_project/chassis/buttons/reset",
        "xyz.openbmc_project.Chassis.Buttons");

    power_control::resetButtonIface->register_property(
        "ButtonMasked", false, [](const bool requested, bool& current) {
            if (requested)
            {
                if (power_control::resetButtonMask != nullptr)
                {
                    return 1;
                }
                power_control::resetButtonMask =
                    power_control::setGPIOOutput("RESET_OUT", 1);
                if (power_control::resetButtonMask == nullptr)
                {
                    throw std::runtime_error("Failed to request GPIO");
                    return 0;
                }
                std::cerr << "Reset Button Masked.\n";
            }
            else
            {
                if (power_control::resetButtonMask == nullptr)
                {
                    return 1;
                }
                std::cerr << "Reset Button Un-masked\n";
                gpiod_line_close_chip(power_control::resetButtonMask);
                power_control::resetButtonMask = nullptr;
            }
            // Update the mask setting
            current = requested;
            return 1;
        });

    // Check reset button state
    bool resetButtonPressed = false;
    if (gpiod_line_get_value(powerButtonLine) == 0)
    {
        resetButtonPressed = true;
    }
    power_control::resetButtonIface->register_property("ButtonPressed",
                                                       resetButtonPressed);

    power_control::resetButtonIface->initialize();

    // NMI Button Interface
    power_control::nmiButtonIface =
        buttonsServer.add_interface("/xyz/openbmc_project/chassis/buttons/nmi",
                                    "xyz.openbmc_project.Chassis.Buttons");

    power_control::nmiButtonIface->register_property(
        "ButtonMasked", false, [](const bool requested, bool& current) {
            if (power_control::nmiButtonMasked == requested)
            {
                // NMI button mask is already set as requested, so no change
                return 1;
            }
            if (requested)
            {
                std::cerr << "NMI Button Masked.\n";
                power_control::nmiButtonMasked = true;
            }
            else
            {
                std::cerr << "NMI Button Un-masked.\n";
                power_control::nmiButtonMasked = false;
            }
            // Update the mask setting
            current = power_control::nmiButtonMasked;
            return 1;
        });

    // Check NMI button state
    bool nmiButtonPressed = false;
    if (gpiod_line_get_value(nmiButtonLine) == 0)
    {
        nmiButtonPressed = true;
    }
    power_control::nmiButtonIface->register_property("ButtonPressed",
                                                     nmiButtonPressed);

    power_control::nmiButtonIface->initialize();

    // ID Button Interface
    power_control::idButtonIface =
        buttonsServer.add_interface("/xyz/openbmc_project/chassis/buttons/id",
                                    "xyz.openbmc_project.Chassis.Buttons");

    // Check ID button state
    bool idButtonPressed = false;
    if (gpiod_line_get_value(idButtonLine) == 0)
    {
        idButtonPressed = true;
    }
    power_control::idButtonIface->register_property("ButtonPressed",
                                                    idButtonPressed);

    power_control::idButtonIface->initialize();

    // OS State Service
    sdbusplus::asio::object_server osServer =
        sdbusplus::asio::object_server(power_control::conn);

    // OS State Interface
    power_control::osIface = osServer.add_interface(
        "/xyz/openbmc_project/state/os",
        "xyz.openbmc_project.State.OperatingSystem.Status");

    // Get the initial OS state based on POST complete
    //      0: Asserted, OS state is "Standby" (ready to boot)
    //      1: De-Asserted, OS state is "Inactive"
    std::string osState =
        gpiod_line_get_value(postCompleteLine) > 0 ? "Inactive" : "Standby";

    power_control::osIface->register_property("OperatingSystemState",
                                              std::string(osState));

    power_control::osIface->initialize();

    power_control::io.run();

    return 0;
}
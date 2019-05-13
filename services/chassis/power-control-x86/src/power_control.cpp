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
#include <filesystem>
#include <gpiod.h>
#include <systemd/sd-journal.h>
#include "power_control.hpp"

class LpcSioDevFile
{
  private:
    int fd = -1;
    struct sio_ioctl_data sioData;

  public:
    LpcSioDevFile()
    {
        fd = ::open(LPC_SIO_DEVPATH, O_RDWR);
        if (fd < 0)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "Open LPC-SIO error!");
            return;
        }
    }

    int getData(unsigned short sioCmd, unsigned int& data)
    {
        if (fd < 0)
        {
            return -1;
        }
        sioData.sio_cmd = sioCmd;
        sioData.param = 0;
        if (::ioctl(fd, SIO_IOC_COMMAND, &sioData) == 0)
        {
            data = sioData.data;
            return 0;
        }
        else
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "ioctl SIO_GET_PFAIL_STATUS error!");
            return -1;
        }
    }

    ~LpcSioDevFile()
    {
        if (fd > 0)
        {
            ::close(fd);
        }
    }
};

void PowerControl::timeOutHandler()
{
    LpcSioDevFile devfile;
    unsigned int data = 0;
    int ret = 0;

    ret = devfile.getData(SIO_GET_ACPI_STATE, data);
    if (ret)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "ioctl SIO_GET_ACPI_STATE error!");
    }
    else
    {
        if (s4s5State() != data)
        {
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "ACPI state changed\n",
                phosphor::logging::entry("OLD=%d", s4s5State()),
                phosphor::logging::entry("NEW=%d", data));
            s4s5State(data);
        }
    }

    ret = devfile.getData(SIO_GET_PWRGD_STATUS, data);
    if (ret)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "ioctl SIO_GET_PWRGD_STATUS error!");
    }
    else
    {
        if (vrdGood() != data)
        {
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "VRD_PWR_GOOD changed\n",
                phosphor::logging::entry("OLD=%d", vrdGood()),
                phosphor::logging::entry("NEW=%d", data));
            vrdGood(data);
        }
    }

    this->timer.start(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::milliseconds(pollingIntervalMs)));
    this->timer.setEnabled<std::true_type>();
}

PowerControl::PowerControl(sdbusplus::bus::bus& bus, const char* path,
                           const bool pgood,
                           phosphor::watchdog::EventPtr event) :
    sdbusplus::server::object_t<pwr_control>(bus, path),
    bus(bus), timer(event, std::bind(&PowerControl::timeOutHandler, this)),
    powerButtonPressedSignal(
        bus,
        sdbusplus::bus::match::rules::type::signal() +
            sdbusplus::bus::match::rules::member("Pressed") +
            sdbusplus::bus::match::rules::path(powerButtonPath) +
            sdbusplus::bus::match::rules::interface(powerButtonIntf),
        [this](sdbusplus::message::message& msg) {
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "powerButtonPressed callback function is called...");
            this->powerButtonPressed = true;
            return;
        }),
    resetButtonPressedSignal(
        bus,
        sdbusplus::bus::match::rules::type::signal() +
            sdbusplus::bus::match::rules::member("Pressed") +
            sdbusplus::bus::match::rules::path(resetButtonPath) +
            sdbusplus::bus::match::rules::interface(resetButtonIntf),
        [this](sdbusplus::message::message& msg) {
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "resetButtonPressed callback function is called...");
            this->resetButtonPressed = true;
            return;
        }),
    pgoodChangedSignal(
        bus,
        sdbusplus::bus::match::rules::type::signal() +
            sdbusplus::bus::match::rules::member("PropertiesChanged") +
            sdbusplus::bus::match::rules::path(gpioDaemonPowerGoodPath) +
            sdbusplus::bus::match::rules::interface(propertiesIntf),
        [this](sdbusplus::message::message& msg) {
            std::string objectName;
            std::map<std::string, BasicVariantType> propertyMap;
            msg.read(objectName, propertyMap);
            powerGoodPropertyHandler(propertyMap);
        }),
    postCompleteChangedSignal(
        bus,
        sdbusplus::bus::match::rules::type::signal() +
            sdbusplus::bus::match::rules::member("PropertiesChanged") +
            sdbusplus::bus::match::rules::path(gpioDaemonPostCompletePath) +
            sdbusplus::bus::match::rules::interface(propertiesIntf),
        [this](sdbusplus::message::message& msg) {
            std::string objectName;
            std::map<std::string, BasicVariantType> propertyMap;
            msg.read(objectName, propertyMap);
            postCompletePropertyHandler(propertyMap);
        })
{
    LpcSioDevFile devfile;
    unsigned int data = 0;
    int ret = 0;

    this->ACOnLogged = false;
    powerButtonPressed = false;
    resetButtonPressed = false;
    this->pgood(pgood);

    if (pgood)
    {
        if (this->postComplete())
        {
            this->state(powerStateOn);
        }
        else
        {
            this->state(powerStateReset);
        }
    }
    else
    {
        this->state(powerStateOff);
    }

    timer.start(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::milliseconds(pollingIntervalMs)));
    timer.setEnabled<std::true_type>();
    phosphor::logging::log<phosphor::logging::level::INFO>(
        "Enable SIO polling timer");

    ret = devfile.getData(SIO_GET_PFAIL_STATUS, data);
    if (ret)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "ioctl SIO_GET_PFAIL_STATUS error!");
        return;
    }

    if (data)
    {
        phosphor::logging::log<phosphor::logging::level::INFO>("AC lost on\n");
        pFail(true);

        if (pgood)
        {
            this->ACOnLog();
        }
    }
    else
    {
        phosphor::logging::log<phosphor::logging::level::INFO>("!AC lost on\n");
        pFail(false);
    }

    // init post_complete
    sdbusplus::message::variant<bool, std::string> propValue;
    if (getGpioDaemonProperty(bus, gpioDaemonPostCompletePath, "Value",
                              propValue) == 0)
    {
        bool post = sdbusplus::message::variant_ns::get<bool>(propValue);
        postComplete(post);
    }
    else
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Init Post_Complete error!");
    }
}

void PowerControl::powerGoodPropertyHandler(
    const std::map<std::string, BasicVariantType>& propertyMap)
{
    // Check if it was the Value property that changed.
    auto valPropMap = propertyMap.find("Value");
    if (valPropMap != propertyMap.end())
    {
        phosphor::logging::log<phosphor::logging::level::INFO>(
            "PowerControl: Power_Good property handler is called");
        auto value =
            sdbusplus::message::variant_ns::get<bool>(valPropMap->second);

        if (value && this->pFail() && !this->ACOnLogged)
        {
            // for first power on after AC lost, log to redfish
            this->ACOnLog();
        }

        if (this->pgood() != value)
        {
            phosphor::logging::log<phosphor::logging::level::INFO>(
                value ? "PSGOOD" : "!PSGOOD");

            phosphor::logging::log<phosphor::logging::level::INFO>(
                this->postComplete() ? "POST" : "!POST");

            this->pgood(value);

            if (value)
            {
                this->powerGood();
            }
            else
            {
                this->powerLost();
            }
        }
    }
}

void PowerControl::postCompletePropertyHandler(
    const std::map<std::string, BasicVariantType>& propertyMap)
{
    // Check if it was the Value property that changed.
    auto valPropMap = propertyMap.find("Value");
    if (valPropMap != propertyMap.end())
    {
        phosphor::logging::log<phosphor::logging::level::INFO>(
            "PowerControl: Post_Complete property handler is called");
        auto value =
            sdbusplus::message::variant_ns::get<bool>(valPropMap->second);
        if (this->postComplete() != value)
        {
            phosphor::logging::log<phosphor::logging::level::INFO>(
                value ? "POST" : "!POST");
            phosphor::logging::log<phosphor::logging::level::INFO>(
                this->pgood() ? "PSGOOD" : "!PSGOOD");

            this->postComplete(value);
        }
    }
}

int32_t PowerControl::forcePowerOff()
{
    int ret = 0;

    ret = i2cSet(pchDevBusAddress, pchDevSlaveAddress, pchCmdReg,
                 pchPowerDownCmd);
    if (ret < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>("i2cSet error!");
    }
    return ret;
}

int32_t PowerControl::triggerReset()
{
    phosphor::logging::log<phosphor::logging::level::INFO>("triggerReset");

    // Find the RESET_OUT GPIO line
    struct gpiod_line* resetOutTmp = gpiod_line_find("RESET_OUT");
    if (resetOutTmp == NULL)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Failed to find the RESET_OUT line.");
        return -1;
    }
    std::unique_ptr<gpiod_line, decltype(&gpiod_line_close_chip)> resetOut(
        resetOutTmp, gpiod_line_close_chip);
    resetOutTmp = nullptr;

    // Request RESET_OUT as asserted
    if (gpiod_line_request_output(resetOut.get(), __FUNCTION__, 0) < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Failed to request RESET_OUT output\n");
        return -1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(resetPulseTimeMs));

    return 0;
}

int32_t PowerControl::setPowerState(int32_t newState)
{
    bool forcePowerOff = false;
    int ret = 0;
    int count = 0;

    if (newState < 0 || newState >= powerStateMax)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "error! invalid parameter!");
        return -1;
    }

    phosphor::logging::log<phosphor::logging::level::INFO>(
        "setPowerState", phosphor::logging::entry("NEWSTATE=%d", newState));

    if (std::filesystem::exists(forceOffFlagPath))
    {
        phosphor::logging::log<phosphor::logging::level::INFO>("ForceOffFlag");
        forcePowerOff = true;
    }
    else
    {
        phosphor::logging::log<phosphor::logging::level::INFO>("!ForceOffFlag");
        forcePowerOff = false;
    }

    switch (newState)
    {
        case powerStateReset:
            if (this->resetButtonPressed)
            {
                phosphor::logging::log<phosphor::logging::level::INFO>(
                    "RESET button pressed, using pass-through");
            }
            else
            {
                // trigger the reset gpio signal
                phosphor::logging::log<phosphor::logging::level::INFO>(
                    "setPowerState system reset");

                // make sure the state property is changed, host-state-manager
                // use this dbus property changed signal to get restart cause
                if (newState == state())
                {
                    phosphor::logging::log<phosphor::logging::level::INFO>(
                        "Make sure reset state change");
                    state(powerStateMax);
                }
                state(newState);
                triggerReset();
            }
            break;
        case powerStateOn:
        case powerStateOff:
            if (this->powerButtonPressed)
            {
                phosphor::logging::log<phosphor::logging::level::INFO>(
                    "POWER button pressed, using pass-through");
            }
            else
            {
                // trigger the power_up gpio signal
                // make sure the state property is changed, host-state-manager
                // use this dbus property changed signal to get restart cause
                if (newState == state())
                {
                    phosphor::logging::log<phosphor::logging::level::INFO>(
                        "Make sure ON/OFF state change");
                    state(powerStateMax);
                }
                state(newState);

                // check if it is on or off
                if (pgood())
                {
                    phosphor::logging::log<phosphor::logging::level::INFO>(
                        "setPowerState PGOOD");
                }
                else
                {
                    phosphor::logging::log<phosphor::logging::level::INFO>(
                        "setPowerState !PGOOD");
                }

                if ((powerStateOff == newState) && (!pgood()))
                {
                    phosphor::logging::log<phosphor::logging::level::INFO>(
                        "OFF already");
                }
                else if ((powerStateOn == newState) && (pgood()))
                {
                    phosphor::logging::log<phosphor::logging::level::INFO>(
                        "ON already");
                }
                else
                {
                    // Find the POWER_OUT GPIO line
                    struct gpiod_line* powerOutTmp =
                        gpiod_line_find("POWER_OUT");
                    if (powerOutTmp == NULL)
                    {
                        phosphor::logging::log<phosphor::logging::level::ERR>(
                            "Failed to find the POWER_OUT line.");
                        return -1;
                    }
                    std::unique_ptr<gpiod_line,
                                    decltype(&gpiod_line_close_chip)>
                        powerOut(powerOutTmp, gpiod_line_close_chip);
                    powerOutTmp = nullptr;

                    // Request POWER_OUT as asserted
                    if (gpiod_line_request_output(powerOut.get(), __FUNCTION__,
                                                  0) < 0)
                    {
                        phosphor::logging::log<phosphor::logging::level::ERR>(
                            "Failed to request POWER_OUT output\n");
                        return -1;
                    }

                    phosphor::logging::log<phosphor::logging::level::INFO>(
                        "setPowerState switch power state");

                    if (forcePowerOff && (powerStateOff == newState))
                    {
                        std::this_thread::sleep_for(
                            std::chrono::milliseconds(forceOffPulseTimeMs));
                    }
                    else
                    {
                        std::this_thread::sleep_for(
                            std::chrono::milliseconds(powerPulseTimeMs));
                    }
                }
            }
            break;
        default:
            break;
    }

    this->resetButtonPressed = false;
    this->powerButtonPressed = false;
    return 0;
}

void setGpioDaemonProperty(sdbusplus::bus::bus& bus, const char* path,
                           const char* property,
                           sdbusplus::message::variant<bool, std::string> value)
{
    sdbusplus::message::message method =
        bus.new_method_call(gpioDaemonService, path, propertiesIntf, "Set");
    method.append(gpioDaemonCtrlIntf, property);
    method.append(value);
    bus.call(method);
}

int getGpioDaemonProperty(sdbusplus::bus::bus& bus, const char* path,
                          const char* property,
                          sdbusplus::message::variant<bool, std::string>& value)
{
    try
    {
        auto method =
            bus.new_method_call(gpioDaemonService, path, propertiesIntf, "Get");
        method.append(gpioDaemonCtrlIntf, property);
        auto reply = bus.call(method);
        reply.read(value);
    }
    catch (const sdbusplus::exception::SdBusError& e)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "getGpioDaemonProperty failed",
            phosphor::logging::entry("OBJPATH:%s", path),
            phosphor::logging::entry("PROP:%s", property));
        return -EIO;
    }
    return 0;
}

int32_t PowerControl::getPowerState()
{
    return state();
}

void PowerControl::ACOnLog()
{
    sd_journal_send("MESSAGE=PowerControl: AC lost PowerOn", "PRIORITY=%i",
                    LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.DCPowerOnAfterACLost", NULL);

    this->ACOnLogged = true;
}

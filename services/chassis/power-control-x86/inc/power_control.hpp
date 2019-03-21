/*
// Copyright (c) 2018 Intel Corporation
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

#pragma once
#include <fcntl.h>
#include <linux/aspeed-lpc-sio.h>
#include <unistd.h>
#include <phosphor-logging/elog-errors.hpp>
#include <xyz/openbmc_project/Chassis/Common/error.hpp>
#include <xyz/openbmc_project/Chassis/Control/Power/server.hpp>
#include <xyz/openbmc_project/Common/error.hpp>
#include "i2c.hpp"
#include <sys/ioctl.h>
#include "timer.hpp"

static constexpr size_t pollingIntervalMs = 500;

const static constexpr char* LPC_SIO_DEVPATH = "/dev/lpc-sio";
const static constexpr char* PGOOD_PIN = "PGOOD";
const static constexpr char* BIOS_POST_CMPLT_PIN = "BIOS_POST_CMPLT";
const static constexpr char* POWER_UP_PIN = "POWER_UP_PIN";
const static constexpr char* RESET_OUT_PIN = "RESET_OUT";

const static constexpr size_t pchDevBusAddress = 3;
const static constexpr size_t pchDevSlaveAddress = 0x44;
const static constexpr size_t pchCmdReg = 0;
const static constexpr size_t pchPowerDownCmd = 0x02;

const static constexpr size_t resetPulseTimeMs = 500;
const static constexpr size_t powerPulseTimeMs = 200;

const static constexpr uint8_t powerStateOff = 0;
const static constexpr uint8_t powerStateOn = 1;
const static constexpr uint8_t powerStateReset = 2;
const static constexpr uint8_t powerStateMax = 3;
const static constexpr char* gpioDaemonPowerUpPath =
    "/xyz/openbmc_project/control/gpio/Power_Up";
const static constexpr char* gpioDaemonCtrlIntf =
    "xyz.openbmc_project.Control.Gpio";
const static constexpr char* propertiesIntf = "org.freedesktop.DBus.Properties";
const static constexpr char* gpioDaemonResetOutPath =
    "/xyz/openbmc_project/control/gpio/Reset_Out";
const static constexpr char* gpioDaemonService = "xyz.openbmc_project.Gpio";
const static constexpr char* gpioDaemonPowerGoodPath =
    "/xyz/openbmc_project/control/gpio/Power_Good";
static bool first_event = true;

using pwr_control =
    sdbusplus::xyz::openbmc_project::Chassis::Control::server::Power;

struct PowerControl : sdbusplus::server::object_t<pwr_control>
{
    PowerControl(sdbusplus::bus::bus& bus, const char* path,
                 phosphor::watchdog::EventPtr event) :
        sdbusplus::server::object_t<pwr_control>(bus, path),
        bus(bus),
        propertiesChangedSignal(
            bus,
            sdbusplus::bus::match::rules::type::signal() +
                sdbusplus::bus::match::rules::member("PropertiesChanged") +
                sdbusplus::bus::match::rules::path(gpioDaemonPowerGoodPath) +
                sdbusplus::bus::match::rules::interface(propertiesIntf),
            [this](sdbusplus::message::message& msg) {
                phosphor::logging::log<phosphor::logging::level::INFO>(
                    "PowerControl propertiesChangedSignal callback function is "
                    "called...");

                std::string objectName;
                std::map<std::string, sdbusplus::message::variant<bool>>
                    msgData;
                msg.read(objectName, msgData);
                // Check if it was the Value property that changed.
                auto valPropMap = msgData.find("Value");
                {
                    if (valPropMap != msgData.end())
                    {
                        if (sdbusplus::message::variant_ns::get<bool>(
                                valPropMap->second))
                        {

                            phosphor::logging::log<
                                phosphor::logging::level::INFO>("PSGOOD");
                            this->state(1);
                            this->pgood(1);
                            if (first_event)
                            {
                                first_event = false;
                            }
                            else
                            {
                                this->powerGood();
                            }
                        }
                        else
                        {
                            phosphor::logging::log<
                                phosphor::logging::level::INFO>("!PSGOOD");
                            this->state(0);
                            this->pgood(0);
                            if (first_event)
                            {
                                first_event = false;
                            }
                            else
                            {
                                this->powerLost();
                            }
                        }
                    }
                }
            })
    {
    }

    ~PowerControl()
    {
    }

    int32_t forcePowerOff() override;
    int32_t setPowerState(int32_t newState) override;
    int32_t getPowerState() override;

  private:
    sdbusplus::bus::bus& bus;
    int32_t triggerReset();

    void setGpioDaemonProperty(
        const char* path, const char* property,
        sdbusplus::message::variant<bool, std::string> value);
    sdbusplus::bus::match_t propertiesChangedSignal;
};

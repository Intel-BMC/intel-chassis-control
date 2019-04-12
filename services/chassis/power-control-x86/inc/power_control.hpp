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
const static constexpr char* gpioDaemonPostCompletePath =
    "/xyz/openbmc_project/control/gpio/Post_Complete";

using pwr_control =
    sdbusplus::xyz::openbmc_project::Chassis::Control::server::Power;

using BasicVariantType =
    sdbusplus::message::variant<std::string, int64_t, uint64_t, double, int32_t,
                                uint32_t, int16_t, uint16_t, uint8_t, bool>;

void setGpioDaemonProperty(
    sdbusplus::bus::bus& bus, const char* path, const char* property,
    sdbusplus::message::variant<bool, std::string> value);
int getGpioDaemonProperty(
    sdbusplus::bus::bus& bus, const char* path, const char* property,
    sdbusplus::message::variant<bool, std::string>& value);

struct PowerControl : sdbusplus::server::object_t<pwr_control>
{
    PowerControl(sdbusplus::bus::bus& bus, const char* path, const bool pgood,
                 phosphor::watchdog::EventPtr event);

    ~PowerControl()
    {
    }

    int32_t forcePowerOff() override;
    int32_t setPowerState(int32_t newState) override;
    int32_t getPowerState() override;
    void powerGoodPropertyHandler(
        const std::map<std::string, BasicVariantType>& propertyMap);
    void postCompletePropertyHandler(
        const std::map<std::string, BasicVariantType>& propertyMap);
    void timeOutHandler();
    void ACOnLog();

  private:
    bool ACOnLogged;
    phosphor::watchdog::Timer timer;
    sdbusplus::bus::bus& bus;
    int32_t triggerReset();
    sdbusplus::bus::match_t pgoodChangedSignal;
    sdbusplus::bus::match_t postCompleteChangedSignal;
};

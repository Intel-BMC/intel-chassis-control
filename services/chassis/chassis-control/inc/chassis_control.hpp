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
#include "xyz/openbmc_project/Chassis/Common/error.hpp"
#include "xyz/openbmc_project/Chassis/Control/Chassis/server.hpp"
#include <phosphor-logging/elog-errors.hpp>
#include <systemd/sd-event.h>

const static constexpr char *POWER_BUTTON_PATH =
    "/xyz/openbmc_project/Chassis/Buttons/Power0";
const static constexpr char *POWER_BUTTON_INTF =
    "xyz.openbmc_project.Chassis.Buttons.Power";
const static constexpr char *RESET_BUTTON_PATH =
    "/xyz/openbmc_project/Chassis/Buttons/Reset0";
const static constexpr char *RESET_BUTTON_INTF =
    "xyz.openbmc_project.Chassis.Buttons.Reset";

const static uint8_t POWER_OFF = 0;
const static uint8_t POWER_ON = 1;

struct EventDeleter
{
    void operator()(sd_event *event) const
    {
        event = sd_event_unref(event);
    }
};

using EventPtr = std::unique_ptr<sd_event, EventDeleter>;

struct ChassisControl
    : sdbusplus::server::object_t<
          sdbusplus::xyz::openbmc_project::Chassis::Control::server::Chassis>
{
    ChassisControl(sdbusplus::bus::bus &bus, const char *path,
                   EventPtr &event) :
        sdbusplus::server::object_t<
            sdbusplus::xyz::openbmc_project::Chassis::Control::server::Chassis>(
            bus, path),
        mBus(bus),
        powerButtonPressedSignal(
            bus,
            sdbusplus::bus::match::rules::type::signal() +
                sdbusplus::bus::match::rules::member("Pressed") +
                sdbusplus::bus::match::rules::path(POWER_BUTTON_PATH) +
                sdbusplus::bus::match::rules::interface(POWER_BUTTON_INTF),
            [this](sdbusplus::message::message &msg) {
                phosphor::logging::log<phosphor::logging::level::INFO>(
                    "powerButtonPressed callback function is called...");
                uint8_t state = static_cast<uint8_t>(this->getPowerState());
                if (POWER_ON == state)
                {
                    this->powerOff();
                }
                else if (POWER_OFF == state)
                {
                    this->powerOn();
                }
                else
                {
                    phosphor::logging::log<phosphor::logging::level::ERR>(
                        "UNKNOWN power state");
                }
                return;
            }),
        resetButtonPressedSignal(
            bus,
            sdbusplus::bus::match::rules::type::signal() +
                sdbusplus::bus::match::rules::member("Pressed") +
                sdbusplus::bus::match::rules::path(RESET_BUTTON_PATH) +
                sdbusplus::bus::match::rules::interface(RESET_BUTTON_INTF),
            [this](sdbusplus::message::message &msg) {
                phosphor::logging::log<phosphor::logging::level::INFO>(
                    "resetButtonPressed callback function is called...");
                this->reboot();
                return;
            })
    {
        phosphor::logging::log<phosphor::logging::level::DEBUG>(
            "ChassisControl is created.");
    }

    int32_t powerOn() override;
    int32_t powerOff() override;
    int32_t softPowerOff() override;
    int32_t reboot() override;
    int32_t softReboot() override;
    int32_t quiesce() override;
    int32_t getPowerState() override;

  private:
    int32_t startSystemdUnit(const std::string &unit);
    sdbusplus::bus::bus &mBus;

    sdbusplus::bus::match_t powerButtonPressedSignal;
    sdbusplus::bus::match_t resetButtonPressedSignal;
};

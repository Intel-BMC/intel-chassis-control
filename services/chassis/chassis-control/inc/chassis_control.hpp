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

#include <systemd/sd-event.h>
#include <systemd/sd-id128.h>
#include <systemd/sd-journal.h>

#include <phosphor-logging/elog-errors.hpp>
#include <xyz/openbmc_project/Common/UUID/server.hpp>

const static constexpr char* chassisPowerOffTarget =
    "obmc-chassis-poweroff@0.target";
const static constexpr char* hostStartTarget = "obmc-host-start@0.target";

const static constexpr char* powerControlPath =
    "/xyz/openbmc_project/Chassis/Control/Power0";
const static constexpr char* powerControlInterface =
    "xyz.openbmc_project.Chassis.Control.Power";
const static constexpr char* idButtonPath =
    "/xyz/openbmc_project/Chassis/Buttons/ID0";
const static constexpr char* idButtonInterface =
    "xyz.openbmc_project.Chassis.Buttons.ID";

const static constexpr char* POWER_BUTTON_PATH =
    "/xyz/openbmc_project/Chassis/Buttons/Power0";
const static constexpr char* POWER_BUTTON_INTF =
    "xyz.openbmc_project.Chassis.Buttons.Power";
const static constexpr char* RESET_BUTTON_PATH =
    "/xyz/openbmc_project/Chassis/Buttons/Reset0";
const static constexpr char* RESET_BUTTON_INTF =
    "xyz.openbmc_project.Chassis.Buttons.Reset";
const static constexpr char* DEVICE_UUID_PATH =
    "/xyz/openbmc_project/inventory/system/chassis/motherboard/bmc";

const static uint8_t POWER_OFF = 0;
const static uint8_t POWER_ON = 1;

struct EventDeleter
{
    void operator()(sd_event* event) const
    {
        event = sd_event_unref(event);
    }
};

using EventPtr = std::unique_ptr<sd_event, EventDeleter>;

struct ChassisControl
    : sdbusplus::server::object::object<
          sdbusplus::xyz::openbmc_project::Common::server::UUID>,
      sdbusplus::server::object_t<
          sdbusplus::xyz::openbmc_project::Chassis::Control::server::Chassis>
{
    ChassisControl(sdbusplus::bus::bus& bus, const char* path,
                   EventPtr& event) :
        sdbusplus::server::object::object<
            sdbusplus::xyz::openbmc_project::Common::server::UUID>(
            bus, DEVICE_UUID_PATH),
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
            [this](sdbusplus::message::message& msg) {
                phosphor::logging::log<phosphor::logging::level::INFO>(
                    "powerButtonPressed callback function is called...");
                int32_t pgood = 0;
                this->getPGOODState(pgood);
                if (pgood)
                {
                    this->powerOff();
                }
                else
                {
                    this->powerOn();
                }

                sd_journal_send("MESSAGE=%s", "Power Button Pressed",
                                "PRIORITY=%i", LOG_INFO,
                                "REDFISH_MESSAGE_ID=%s", "PowerButtonPressed",
                                NULL);
                return;
            }),
        resetButtonPressedSignal(
            bus,
            sdbusplus::bus::match::rules::type::signal() +
                sdbusplus::bus::match::rules::member("Pressed") +
                sdbusplus::bus::match::rules::path(RESET_BUTTON_PATH) +
                sdbusplus::bus::match::rules::interface(RESET_BUTTON_INTF),
            [this](sdbusplus::message::message& msg) {
                phosphor::logging::log<phosphor::logging::level::INFO>(
                    "resetButtonPressed callback function is called...");
                int32_t pgood = 0;
                this->getPGOODState(pgood);
                if (pgood)
                {
                    this->softReboot();
                }
                else
                {
                    phosphor::logging::log<phosphor::logging::level::WARNING>(
                        "OFF state Cannot reset");
                }

                sd_journal_send("MESSAGE=%s", "Reset Button Pressed",
                                "PRIORITY=%i", LOG_INFO,
                                "REDFISH_MESSAGE_ID=%s", "ResetButtonPressed",
                                NULL);
                return;
            }),
        idButtonPressedSignal(
            bus,
            sdbusplus::bus::match::rules::type::signal() +
                sdbusplus::bus::match::rules::member("Pressed") +
                sdbusplus::bus::match::rules::path(idButtonPath) +
                sdbusplus::bus::match::rules::interface(idButtonInterface),
            [this](sdbusplus::message::message& msg) {
                bool status = false;
                int8_t ret = 0;
                phosphor::logging::log<phosphor::logging::level::INFO>(
                    "idButtonPressed callback function is called...");

                /*
                 *TODO needs to support ID blinking state,
                 * and sync with ipmi chassis id command.
                 * Since both of them are changing the same 'Asserted' property
                 */
                ret = this->getIDStatus(&status);
                if (ret != 0)
                {
                    phosphor::logging::log<phosphor::logging::level::ERR>(
                        "idButtonPressed callback getIDStatus error!");
                    return;
                }

                ret = this->setIDStatus(!status);
                if (ret != 0)
                {
                    phosphor::logging::log<phosphor::logging::level::ERR>(
                        "idButtonPressed callback setIDStatus error!");
                    return;
                }
                return;
            }),
        pgoodPropSignal(
            bus,
            sdbusplus::bus::match::rules::propertiesChanged(
                powerControlPath, powerControlInterface),
            [this](sdbusplus::message::message& msg) {
                phosphor::logging::log<phosphor::logging::level::INFO>(
                    "pgoodPropSignal callback function is called...");

                using DbusVariant = sdbusplus::message::variant<
                    std::string, bool, uint8_t, uint16_t, int16_t, uint32_t,
                    int32_t, uint64_t, int64_t, double>;

                std::map<std::string, DbusVariant> props;
                std::vector<std::string> inval;
                std::string iface;
                msg.read(iface, props, inval);

                for (const auto& t : props)
                {
                    auto key = t.first;
                    auto value = t.second;

                    if (key == "pgood")
                    {
                        int32_t pgood =
                            sdbusplus::message::variant_ns::get<int32_t>(value);
                        if (!pgood)
                        {

                            // check if the systemd service is in correct state
                            phosphor::logging::log<
                                phosphor::logging::level::INFO>(
                                "pgood changed to low, check on service");
                            if (!this->stateActive(chassisPowerOffTarget))
                            {
                                phosphor::logging::log<
                                    phosphor::logging::level::INFO>(
                                    "Service is inactive, start poweroff "
                                    "service");
                                this->powerOff();
                            }
                            else
                            {
                                phosphor::logging::log<
                                    phosphor::logging::level::INFO>(
                                    "Service is active already, no need to "
                                    "start");
                            }
                        }
                        else
                        {
                            phosphor::logging::log<
                                phosphor::logging::level::INFO>(
                                "pgood changed to High, check on service");
                            if (!this->stateActive(hostStartTarget))
                            {
                                phosphor::logging::log<
                                    phosphor::logging::level::INFO>(
                                    "Service is inactive, start poweron "
                                    "service");
                                this->powerOn();
                            }
                            else
                            {
                                phosphor::logging::log<
                                    phosphor::logging::level::INFO>(
                                    "Service is active already, no need to "
                                    "start");
                            }
                        }
                        break;
                    }
                }
            })
    {
        sd_id128_t id = SD_ID128_NULL;
        int r = 0;
        char s[SD_ID128_STRING_MAX] = "";

/*
 * Follow the example in API manual to
 * generate this ID using command "journalctl --new-id128"
 * https://www.freedesktop.org/software/systemd/man/sd_id128_get_machine.html
 *
 */
#define MESSAGE_APPID                                                          \
    SD_ID128_MAKE(e0, e1, 73, 76, 64, 61, 47, da, a5, 0c, d0, cc, 64, 12, 45,  \
                  78)

        r = sd_id128_get_machine_app_specific(MESSAGE_APPID, &id);
        if (r < 0)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "Error in sd_id128 call");
            return;
        }

        sd_id128_to_string(id, s);
        uUID(std::string(s));

        phosphor::logging::log<phosphor::logging::level::INFO>(
            "ChassisControl is created.",
            phosphor::logging::entry("UUID=%s", s));
    }

    int32_t powerOn() override;
    int32_t powerOff() override;
    int32_t softPowerOff() override;
    int32_t reboot() override;
    int32_t softReboot() override;
    int32_t quiesce() override;
    int32_t getPowerState() override;
    std::string uUID(std::string value) override;

  private:
    bool stateActive(const std::string& target);
    int32_t getPGOODState(int32_t& pgood);
    int32_t startSystemdUnit(const std::string& unit);
    sdbusplus::bus::bus& mBus;

    sdbusplus::bus::match_t powerButtonPressedSignal;
    sdbusplus::bus::match_t resetButtonPressedSignal;
    sdbusplus::bus::match_t idButtonPressedSignal;
    sdbusplus::bus::match_t pgoodPropSignal;
    int8_t getIDStatus(bool* status);
    int8_t setIDStatus(bool status);
};

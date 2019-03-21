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
#include <phosphor-logging/elog-errors.hpp>
#include <unistd.h>
#include "xyz/openbmc_project/Chassis/Common/error.hpp"
#include "xyz/openbmc_project/Chassis/Buttons/Power/server.hpp"
#include "common.hpp"

const static constexpr char *gpioDaemonPowerButtonPath =
    "/xyz/openbmc_project/control/gpio/Power_Button";

struct PowerButton
    : sdbusplus::server::object::object<
          sdbusplus::xyz::openbmc_project::Chassis::Buttons::server::Power>
{

    PowerButton(sdbusplus::bus::bus &bus, const char *path) :
        sdbusplus::server::object::object<
            sdbusplus::xyz::openbmc_project::Chassis::Buttons::server::Power>(
            bus, path),
        bus(bus),
        propertiesChangedSignal(
            bus,
            sdbusplus::bus::match::rules::type::signal() +
                sdbusplus::bus::match::rules::member("PropertiesChanged") +
                sdbusplus::bus::match::rules::path(gpioDaemonPowerButtonPath) +
                sdbusplus::bus::match::rules::interface(propertiesIntf) +
                sdbusplus::bus::match::rules::argN(
                    0, "xyz.openbmc_project.Control.Gpio"),
            [this](sdbusplus::message::message &msg) {
                phosphor::logging::log<phosphor::logging::level::INFO>(
                    "PowerButton propertiesChangedSignal callback function is "
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
                                phosphor::logging::level::INFO>(
                                "POWER_BUTTON: pressed");
                            // emit pressed signal
                            this->pressed();
                        }
                        else
                        {
                            phosphor::logging::log<
                                phosphor::logging::level::INFO>(
                                "POWER_BUTTON: released");
                            // released
                            this->released();
                        }
                    }
                }
            })
    {
    }

    ~PowerButton()
    {
    }
    void simPress() override;
    void simLongPress() override;

  private:
    sdbusplus::bus::bus &bus;
    sdbusplus::bus::match_t propertiesChangedSignal;
};

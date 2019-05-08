/*
// Copyright (c) 2019 Intel Corporation
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
#include "xyz/openbmc_project/Chassis/Buttons/NMI/server.hpp"
#include "common.hpp"

const static constexpr char *gpioDaemonNmiButtonPath =
    "/xyz/openbmc_project/control/gpio/NMI_Button";

struct NmiButton
    : sdbusplus::server::object::object<
          sdbusplus::xyz::openbmc_project::Chassis::Buttons::server::NMI>
{

    NmiButton(sdbusplus::bus::bus &bus, const char *path) :
        sdbusplus::server::object::object<
            sdbusplus::xyz::openbmc_project::Chassis::Buttons::server::NMI>(
            bus, path),
        bus(bus),
        propertiesChangedSignal(
            bus,
            sdbusplus::bus::match::rules::type::signal() +
                sdbusplus::bus::match::rules::member("PropertiesChanged") +
                sdbusplus::bus::match::rules::path(gpioDaemonNmiButtonPath) +
                sdbusplus::bus::match::rules::interface(propertiesIntf) +
                sdbusplus::bus::match::rules::argN(
                    0, "xyz.openbmc_project.Control.Gpio"),
            [this](sdbusplus::message::message &msg) {
                phosphor::logging::log<phosphor::logging::level::INFO>(
                    "NmiButton propertiesChangedSignal callback function is "
                    "called...");

                std::string objectName;
                std::map<std::string, std::variant<bool>> msgData;
                msg.read(objectName, msgData);
                // Check if it was the Value property that changed.
                auto valPropMap = msgData.find("Value");
                if (valPropMap != msgData.end())
                {
                    if (std::get<bool>(valPropMap->second))
                    {
                        phosphor::logging::log<phosphor::logging::level::INFO>(
                            "NMI_BUTTON: pressed");
                        // emit pressed signal
                        this->pressed();
                    }
                    else
                    {
                        phosphor::logging::log<phosphor::logging::level::INFO>(
                            "NMI_BUTTON: released");
                        // released
                        this->released();
                    }
                }
            })
    {
    }

    ~NmiButton()
    {
    }
    void simPress() override;

  private:
    sdbusplus::bus::bus &bus;
    sdbusplus::bus::match_t propertiesChangedSignal;
};

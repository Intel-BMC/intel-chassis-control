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
#include "common.hpp"
#include "xyz/openbmc_project/Chassis/Buttons/ID/server.hpp"
#include "xyz/openbmc_project/Chassis/Common/error.hpp"

#include <unistd.h>

#include <phosphor-logging/elog-errors.hpp>

const static constexpr char* gpioDaemonIdButtonPath =
    "/xyz/openbmc_project/control/gpio/ID_Button";

struct IDButton
    : sdbusplus::server::object::object<
          sdbusplus::xyz::openbmc_project::Chassis::Buttons::server::ID>
{

    IDButton(sdbusplus::bus::bus& bus, const char* path) :
        sdbusplus::server::object::object<
            sdbusplus::xyz::openbmc_project::Chassis::Buttons::server::ID>(
            bus, path),
        bus(bus),
        propertiesChangedSignal(
            bus,
            sdbusplus::bus::match::rules::type::signal() +
                sdbusplus::bus::match::rules::member("PropertiesChanged") +
                sdbusplus::bus::match::rules::path(gpioDaemonIdButtonPath) +
                sdbusplus::bus::match::rules::interface(propertiesIntf),
            [this](sdbusplus::message::message& msg) {
                phosphor::logging::log<phosphor::logging::level::INFO>(
                    "ID button propertiesChangedSignal callback function is "
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
                                "ID_BUTTON: pressed");
                            // emit pressed signal
                            this->pressed();
                        }
                        else
                        {
                            phosphor::logging::log<
                                phosphor::logging::level::INFO>(
                                "ID_BUTTON: released");
                            // released
                            this->released();
                        }
                    }
                }
            })

    {
    }

    ~IDButton()
    {
    }
    void simPress() override;

  private:
    sdbusplus::bus::bus& bus;
    sdbusplus::bus::match_t propertiesChangedSignal;
};

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

#include "reset_button.hpp"
#include "power_button.hpp"
#include "id_button.hpp"

int main(int argc, char *argv[])
{
    int ret = 0;

    phosphor::logging::log<phosphor::logging::level::INFO>(
        "Start power button service...");

    sd_event *event = nullptr;
    ret = sd_event_default(&event);
    if (ret < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Error creating a default sd_event handler");
        return ret;
    }
    EventPtr eventP{event};
    event = nullptr;

    sdbusplus::bus::bus bus = sdbusplus::bus::new_default();
    sdbusplus::server::manager::manager objManager{
        bus, "/xyz/openbmc_project/Chassis/Buttons"};

    bus.request_name("xyz.openbmc_project.Chassis.Buttons");

    PowerButton powerButton{bus, POWER_DBUS_OBJECT_NAME, std::move(eventP)};

    ResetButton resetButton{bus, RESET_DBUS_OBJECT_NAME, std::move(eventP)};

    IDButton idButton{bus, ID_DBUS_OBJECT_NAME, std::move(eventP)};

    try
    {
        bus.attach_event(eventP.get(), SD_EVENT_PRIORITY_NORMAL);
        ret = sd_event_loop(eventP.get());
        if (ret < 0)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "Error occurred during the sd_event_loop",
                phosphor::logging::entry("RET=%d", ret));
        }
    }
    catch (std::exception &e)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(e.what());
        ret = -1;
    }
    return ret;
}

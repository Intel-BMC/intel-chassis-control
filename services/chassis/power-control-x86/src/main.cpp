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
#include "power_control.hpp"

std::unique_ptr<PowerControl> powerControlObject;
std::unique_ptr<sdbusplus::bus::match_t> interfacesAddedSignal;
std::unique_ptr<sdbusplus::server::manager_t> mgrControlObject;

int main(int argc, char* argv[])
{
    int ret = 0;

    phosphor::logging::log<phosphor::logging::level::INFO>(
        "Start Chassis power control service...");

    sd_event* event = nullptr;
    ret = sd_event_default(&event);
    if (ret < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Error creating a default sd_event handler");
        return ret;
    }
    phosphor::watchdog::EventPtr eventP{event,
                                        phosphor::watchdog::EventDeleter()};
    event = nullptr;

    sdbusplus::bus::bus bus = sdbusplus::bus::new_default();

    sdbusplus::message::variant<bool, std::string> propValue;
    if (getGpioDaemonProperty(bus, gpioDaemonPowerGoodPath, "Value",
                              propValue) == 0)
    {
        bus.request_name(DBUS_INTF_NAME);
        mgrControlObject = std::make_unique<sdbusplus::server::manager_t>(
            bus, DBUS_OBJECT_NAME);
        bool pgood = sdbusplus::message::variant_ns::get<bool>(propValue);
        powerControlObject = std::make_unique<PowerControl>(
            bus, DBUS_OBJECT_NAME, pgood, eventP);
    }
    // Register for InterfacesAddedSignal, to match the state change / object
    // change. This will handle all the cases of entity-manager restart,
    // gpio-daemon restart and still hold the power control state properly.
    interfacesAddedSignal = std::make_unique<sdbusplus::bus::match_t>(
        bus,
        sdbusplus::bus::match::rules::type::signal() +
            sdbusplus::bus::match::rules::member("InterfacesAdded") +
            "arg0path='" + gpioDaemonPowerGoodPath + "'",
        [&](sdbusplus::message::message& msg) {
            sdbusplus::message::object_path objectPath;
            using ObjectTypes =
                std::map<std::string, std::map<std::string, BasicVariantType>>;
            ObjectTypes objects;
            msg.read(objectPath, objects);
            for (const auto& objInterfaces : objects)
            {
                if (gpioDaemonCtrlIntf == objInterfaces.first)
                {
                    auto valPropMap = objInterfaces.second.find("Value");
                    if (valPropMap != objInterfaces.second.end())
                    {
                        auto pgood = sdbusplus::message::variant_ns::get<bool>(
                            valPropMap->second);
                        if (powerControlObject == nullptr)
                        {
                            bus.request_name(DBUS_INTF_NAME);
                            mgrControlObject =
                                std::make_unique<sdbusplus::server::manager_t>(
                                    bus, DBUS_OBJECT_NAME);
                            powerControlObject = std::make_unique<PowerControl>(
                                bus, DBUS_OBJECT_NAME, pgood, eventP);
                        }
                        else
                        {
                            powerControlObject->powerGoodPropertyHandler(
                                objInterfaces.second);
                        }
                    }
                }
            }
        });

    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now());

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
    catch (std::exception& e)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(e.what());
        return -1;
    }
    return 0;
}

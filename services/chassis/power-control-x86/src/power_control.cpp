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
#include <boost/process/child.hpp>
#include "power_control.hpp"

constexpr const char* passthroughPath = "/usr/bin/set-passthrough.sh";

struct DisablePassthrough
{

    DisablePassthrough()
    {
        // todo: use driver instead of /dev/mem
        boost::process::child c(passthroughPath, "0");
        c.wait();
    }
    ~DisablePassthrough()
    {
        boost::process::child c(passthroughPath, "1");
        c.wait();
    }
};

PowerControl::PowerControl(sdbusplus::bus::bus& bus, const char* path,
                           const bool pgood,
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
            std::string objectName;
            std::map<std::string, BasicVariantType> propertyMap;
            msg.read(objectName, propertyMap);
            powerGoodPropertyHandler(propertyMap);
        })
{
    this->state(pgood);
    this->pgood(pgood);
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
        phosphor::logging::log<phosphor::logging::level::INFO>(
            value ? "PSGOOD" : "!PSGOOD");
        this->state(value);
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
    int ret = 0;
    int count = 0;
    char buf = '0';

    phosphor::logging::log<phosphor::logging::level::INFO>("triggerReset");

    auto disable = DisablePassthrough();
    // Set GpipDaemon::Power_UP Value property to change host power state.
    setGpioDaemonProperty(bus, gpioDaemonResetOutPath, "Direction",
                          std::string("out"));
    setGpioDaemonProperty(bus, gpioDaemonResetOutPath, "Value", true);

    std::this_thread::sleep_for(std::chrono::milliseconds(resetPulseTimeMs));
    setGpioDaemonProperty(bus, gpioDaemonResetOutPath, "Value", false);

    return 0;
}

int32_t PowerControl::setPowerState(int32_t newState)
{
    int ret = 0;
    int count = 0;
    char buf = '0';

    if (newState < 0 || newState >= powerStateMax)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "error! invalid parameter!");
        return -1;
    }

    phosphor::logging::log<phosphor::logging::level::INFO>(
        "setPowerState", phosphor::logging::entry("NEWSTATE=%d", newState));

    if (powerStateReset == newState)
    {
        phosphor::logging::log<phosphor::logging::level::INFO>(
            "setPowerState system reset");
        triggerReset();
        return 0;
    }

    if (state() == newState)
    {
        phosphor::logging::log<phosphor::logging::level::INFO>(
            "Same powerstate",
            phosphor::logging::entry("NEWSTATE=%d", newState));
        return 0;
    }

    state(newState);
    auto disable = DisablePassthrough();

    // Set GpipDaemon::Power_UP Value property to change host power state.
    setGpioDaemonProperty(bus, gpioDaemonPowerUpPath, "Direction",
                          std::string("out"));
    setGpioDaemonProperty(bus, gpioDaemonPowerUpPath, "Value", true);

    phosphor::logging::log<phosphor::logging::level::INFO>(
        "setPowerState switch power state");
    std::this_thread::sleep_for(std::chrono::milliseconds(powerPulseTimeMs));
    setGpioDaemonProperty(bus, gpioDaemonPowerUpPath, "Value", false);

    if (0 == newState)
    {

        std::this_thread::sleep_for(
            std::chrono::milliseconds(powerPulseTimeMs));
        if (1 == pgood())
        { // still on, force off!
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "Perform force power off");
            count = 0;
            do
            {
                if (count++ > 5)
                {
                    phosphor::logging::log<phosphor::logging::level::ERR>(
                        "forcePowerOff error!");
                    throw sdbusplus::xyz::openbmc_project::Chassis::Common::
                        Error::IOError();
                }
                ret = forcePowerOff();
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(pollingIntervalMs));
            } while (ret != 0);
        }
    }

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

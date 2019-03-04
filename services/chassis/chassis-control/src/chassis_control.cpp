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

#include "chassis_control.hpp"
#include <chrono>
#include <xyz/openbmc_project/Common/error.hpp>

const static constexpr char *ledService =
    "xyz.openbmc_project.LED.GroupManager";
const static constexpr char *ledIDObj =
    "/xyz/openbmc_project/led/groups/enclosure_identify";
const static constexpr char *ledInterface = "xyz.openbmc_project.Led.Group";
const static constexpr char *ledProp = "Asserted";
const static constexpr char *propInterface = "org.freedesktop.DBus.Properties";

const static constexpr char *SYSTEMD_SERVICE = "org.freedesktop.systemd1";
const static constexpr char *SYSTEMD_OBJ_PATH = "/org/freedesktop/systemd1";
const static constexpr char *SYSTEMD_INTERFACE =
    "org.freedesktop.systemd1.Manager";
const static constexpr char *HOST_START_TARGET = "obmc-host-start@0.target";
const static constexpr char *CHASSIS_HARD_POWER_OFF_TARGET =
    "obmc-chassis-hard-poweroff@0.target";
const static constexpr char *CHASSIS_POWER_OFF_TARGET =
    "obmc-chassis-poweroff@0.target";
const static constexpr char *CHASSIS_POWER_ON_TARGET =
    "obmc-chassis-poweron@0.target";

const static constexpr char *POWER_CONTROL_SERVICE =
    "xyz.openbmc_project.Chassis.Control.Power";
const static constexpr char *POWER_CONTROL_OBJ_PATH =
    "/xyz/openbmc_project/Chassis/Control/Power0";
const static constexpr char *POWER_CONTROL_INTERFACE =
    "xyz.openbmc_project.Chassis.Control.Power";
const static constexpr char *HOST_STATE_REBOOT_TGT =
    "obmc-host-reboot@0.target";
const static constexpr char *HOST_STATE_SHUTDOWN_TGT =
    "obmc-host-shutdown@0.target";
const static constexpr char *HOST_SOFT_REBOOT_TGT =
    "obmc-host-warm-reset@0.target";

int32_t ChassisControl::startSystemdUnit(const std::string &unit)
{
    sdbusplus::message::message method = mBus.new_method_call(
        SYSTEMD_SERVICE, SYSTEMD_OBJ_PATH, SYSTEMD_INTERFACE, "StartUnit");
    method.append(unit, "replace");
    sdbusplus::message::message response = mBus.call(method);

    if (response.is_method_error())
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "ERROR: Failed to start systemd unit",
            phosphor::logging::entry("UNIT=%s", unit.c_str()));
        return -1;
    }

    return 0;
}

int32_t ChassisControl::powerOn()
{
    return startSystemdUnit(HOST_START_TARGET);
}

int32_t ChassisControl::powerOff()
{
    return startSystemdUnit(CHASSIS_POWER_OFF_TARGET);
}

int32_t ChassisControl::softPowerOff()
{

    return startSystemdUnit(HOST_STATE_SHUTDOWN_TGT);
}

int32_t ChassisControl::reboot()
{
    return startSystemdUnit(HOST_STATE_REBOOT_TGT);
}

int32_t ChassisControl::softReboot()
{
    return startSystemdUnit(HOST_SOFT_REBOOT_TGT);
}

int32_t ChassisControl::quiesce()
{
    // TODO
    return 0;
}

int32_t ChassisControl::getPowerState()
{
    int32_t state = 0;
    sdbusplus::message::message method =
        mBus.new_method_call(POWER_CONTROL_SERVICE, POWER_CONTROL_OBJ_PATH,
                             POWER_CONTROL_INTERFACE, "getPowerState");
    sdbusplus::message::message result = mBus.call(method);
    if (result.is_method_error())
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "ERROR: Failed to call power control method getPowerState");
        return -1;
    }
    result.read(state);

    return state;
}

int8_t ChassisControl::getIDStatus(bool *status)
{
    sdbusplus::message::message method =
        mBus.new_method_call(ledService, ledIDObj, propInterface, "Get");

    method.append(ledInterface, ledProp);

    try
    {
        auto reply = mBus.call(method);
        sdbusplus::message::variant<bool> asserted;
        reply.read(asserted);
        *status = sdbusplus::message::variant_ns::get<bool>(asserted);
    }
    catch (sdbusplus::exception_t &)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Failed to get property",
            phosphor::logging::entry("PROPERTY=%s", ledProp),
            phosphor::logging::entry("PATH=%s", ledIDObj),
            phosphor::logging::entry("INTERFACE=%s", ledInterface));
        phosphor::logging::elog<
            sdbusplus::xyz::openbmc_project::Common::Error::InternalFailure>();
        return -1;
    }

    return 0;
}

int8_t ChassisControl::setIDStatus(bool status)
{
    sdbusplus::message::message method =
        mBus.new_method_call(ledService, ledIDObj, propInterface, "Set");

    method.append(ledInterface, ledProp,
                  sdbusplus::message::variant<bool>(status));

    try
    {
        mBus.call(method);
    }
    catch (sdbusplus::exception_t &)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Failed to set property",
            phosphor::logging::entry("PROPERTY=%s", ledProp),
            phosphor::logging::entry("PATH=%s", ledIDObj),
            phosphor::logging::entry("INTERFACE=%s", ledInterface));
        phosphor::logging::elog<
            sdbusplus::xyz::openbmc_project::Common::Error::InternalFailure>();
        return -1;
    }

    return 0;
}

std::string ChassisControl::uUID(std::string value)
{
    return sdbusplus::xyz::openbmc_project::Common::server::UUID::uUID(value);
}

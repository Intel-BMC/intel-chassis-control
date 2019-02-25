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
#include "gpio.hpp"

const static constexpr int32_t powerButtonNum = 34;
const static constexpr char *powerButtonDirection = "both";

struct PowerButton
    : sdbusplus::server::object::object<
          sdbusplus::xyz::openbmc_project::Chassis::Buttons::server::Power>
{

    PowerButton(sdbusplus::bus::bus &bus, const char *path, EventPtr &&event,
                sd_event_io_handler_t handler = PowerButton::EventHandler) :
        sdbusplus::server::object::object<
            sdbusplus::xyz::openbmc_project::Chassis::Buttons::server::Power>(
            bus, path),
        fd(-1), bus(bus), event(event), callbackHandler(handler)
    {

        int ret = -1;

        // config gpio
        ret = ::configGpio(powerButtonNum, powerButtonDirection, &fd, bus);
        if (ret < 0)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "POWER_BUTTON: failed to config GPIO");
            return;
        }

        ret = sd_event_add_io(event.get(), nullptr, fd, EPOLLPRI,
                              callbackHandler, this);
        if (ret < 0)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "POWER_BUTTON: failed to add to event loop");
            ::closeGpio(fd);
            return;
        }
    }

    ~PowerButton()
    {
        ::closeGpio(fd);
    }
    void simPress() override;
    void simLongPress() override;
    static int EventHandler(sd_event_source *es, int fd, uint32_t revents,
                            void *userdata)
    {

        int n = -1;
        char buf = '0';

        if (!userdata)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "POWER_BUTTON: userdata null!");
            return -1;
        }

        PowerButton *powerButton = static_cast<PowerButton *>(userdata);

        if (!powerButton)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "POWER_BUTTON: null pointer!");
            return -1;
        }

        n = ::lseek(fd, 0, SEEK_SET);

        if (n < 0)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "POWER_BUTTON: lseek error!");
            return n;
        }

        n = ::read(fd, &buf, sizeof(buf));
        if (n < 0)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "POWER_BUTTON: read error!");
            return n;
        }

        if (buf == '0')
        {
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "POWER_BUTTON: pressed");
            // emit pressed signal
            powerButton->pressed();
        }
        else
        {
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "POWER_BUTTON: released");
            // released
            powerButton->released();
        }

        return 0;
    }

  private:
    int fd;
    sdbusplus::bus::bus &bus;
    EventPtr &event;
    sd_event_io_handler_t callbackHandler;
};

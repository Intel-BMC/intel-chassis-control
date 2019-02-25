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
#include "xyz/openbmc_project/Chassis/Buttons/Reset/server.hpp"
#include "common.hpp"
#include "gpio.hpp"

const static constexpr int32_t rstButtonNum = 32;
const static constexpr char *rstButtonDirection = "both";

struct ResetButton
    : sdbusplus::server::object::object<
          sdbusplus::xyz::openbmc_project::Chassis::Buttons::server::Reset>
{

    ResetButton(sdbusplus::bus::bus &bus, const char *path, EventPtr &&event,
                sd_event_io_handler_t handler = ResetButton::EventHandler) :
        sdbusplus::server::object::object<
            sdbusplus::xyz::openbmc_project::Chassis::Buttons::server::Reset>(
            bus, path),
        fd(-1), bus(bus), event(event), callbackHandler(handler)
    {

        int ret = -1;

        // config gpio
        ret = ::configGpio(rstButtonNum, rstButtonDirection, &fd, bus);
        if (ret < 0)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "RESET_BUTTON: failed to config GPIO");
            return;
        }

        ret = sd_event_add_io(event.get(), nullptr, fd, EPOLLPRI,
                              callbackHandler, this);
        if (ret < 0)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "RESET_BUTTON: failed to add to event loop");
            ::closeGpio(fd);
            return;
        }
    }

    ~ResetButton()
    {
        ::closeGpio(fd);
    }
    void simPress() override;
    static int EventHandler(sd_event_source *es, int fd, uint32_t revents,
                            void *userdata)
    {

        int n = -1;
        char buf = '0';

        if (!userdata)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "RESET_BUTTON: userdata null!");
            return -1;
        }

        ResetButton *resetButton = static_cast<ResetButton *>(userdata);

        if (!resetButton)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "RESET_BUTTON: null pointer!");
            return -1;
        }

        n = ::lseek(fd, 0, SEEK_SET);

        if (n < 0)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "RESET_BUTTON: lseek error!");
            return n;
        }

        n = ::read(fd, &buf, sizeof(buf));
        if (n < 0)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "RESET_BUTTON: read error!");
            return n;
        }

        if (buf == '0')
        {
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "RESET_BUTTON: pressed");
            // emit pressed signal
            resetButton->pressed();
        }
        else
        {
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "RESET_BUTTON: released");
            // released
            resetButton->released();
        }

        return 0;
    }

  private:
    int fd;
    sdbusplus::bus::bus &bus;
    EventPtr &event;
    sd_event_io_handler_t callbackHandler;
};

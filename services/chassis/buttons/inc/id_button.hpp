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
#include "xyz/openbmc_project/Chassis/Buttons/ID/server.hpp"
#include "common.hpp"
#include "gpio.hpp"

const static constexpr int32_t idButtonNum = 218;
const static constexpr char *idButtonDirection = "both";

struct IDButton
    : sdbusplus::server::object::object<
          sdbusplus::xyz::openbmc_project::Chassis::Buttons::server::ID>
{

    IDButton(sdbusplus::bus::bus &bus, const char *path, EventPtr &&event,
             sd_event_io_handler_t handler = IDButton::EventHandler) :
        sdbusplus::server::object::object<
            sdbusplus::xyz::openbmc_project::Chassis::Buttons::server::ID>(
            bus, path),
        fd(-1), bus(bus), event(event), callbackHandler(handler)
    {

        int ret = -1;

        // config gpio
        ret = ::configGpio(idButtonNum, idButtonDirection, &fd, bus);
        if (ret < 0)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "ID_BUTTON: failed to config GPIO");
            return;
        }

        ret = sd_event_add_io(event.get(), nullptr, fd, EPOLLPRI,
                              callbackHandler, this);
        if (ret < 0)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "ID_BUTTON: failed to add to event loop");
            ::closeGpio(fd);
            return;
        }
    }

    ~IDButton()
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
                "ID_BUTTON: userdata null!");
            return -1;
        }

        IDButton *idButton = static_cast<IDButton *>(userdata);

        if (!idButton)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "ID_BUTTON: null pointer!");
            return -1;
        }

        n = ::lseek(fd, 0, SEEK_SET);

        if (n < 0)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "ID_BUTTON: lseek error!");
            return n;
        }

        n = ::read(fd, &buf, sizeof(buf));
        if (n < 0)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "ID_BUTTON: read error!");
            return n;
        }

        if (buf == '0')
        {
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "ID_BUTTON: pressed");
            // emit pressed signal
            idButton->pressed();
        }
        else
        {
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "ID_BUTTON: released");
            // released
            idButton->released();
        }

        return 0;
    }

  private:
    int fd;
    sdbusplus::bus::bus &bus;
    EventPtr &event;
    sd_event_io_handler_t callbackHandler;
};

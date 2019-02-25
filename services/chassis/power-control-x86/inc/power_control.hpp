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
#include <fcntl.h>
#include <linux/aspeed-lpc-sio.h>
#include <unistd.h>
#include <phosphor-logging/elog-errors.hpp>
#include <xyz/openbmc_project/Chassis/Common/error.hpp>
#include <xyz/openbmc_project/Chassis/Control/Power/server.hpp>
#include <xyz/openbmc_project/Common/error.hpp>
#include "gpio.hpp"
#include "i2c.hpp"
#include <sys/ioctl.h>
#include "timer.hpp"

static constexpr size_t POLLING_INTERVAL_MS = 500;

const static constexpr char* LPC_SIO_DEVPATH = "/dev/lpc-sio";

const static constexpr int32_t pgoodPinNum = 219;
const static constexpr char* pgoodPinDirection = "both";

const static constexpr int32_t biosPostCmpltPinNum = 215;
const static constexpr char* biosPostCmpltPinDirection = "both";

const static constexpr int32_t powerUpPinNum = 35;
const static constexpr char* powerUpPinDirection = "out";

const static constexpr int32_t resetOutPinNum = 33;
const static constexpr char* resetOutPinDirection = "out";

const static constexpr size_t PCH_DEVICE_BUS_ADDRESS = 3;
const static constexpr size_t PCH_DEVICE_SLAVE_ADDRESS = 0x44;
const static constexpr size_t PCH_CMD_REGISTER = 0;
const static constexpr size_t PCH_POWER_DOWN_CMD = 0x02;

const static constexpr size_t RESET_PULSE_TIME_MS = 500;
const static constexpr size_t POWER_PULSE_TIME_MS = 200;

const static constexpr uint8_t powerStateOff = 0;
const static constexpr uint8_t powerStateOn = 1;
const static constexpr uint8_t powerStateReset = 2;
const static constexpr uint8_t powerStateMax = 3;

using pwr_control =
    sdbusplus::xyz::openbmc_project::Chassis::Control::server::Power;

struct PowerControl : sdbusplus::server::object_t<pwr_control>
{
    PowerControl(sdbusplus::bus::bus& bus, const char* path,
                 phosphor::watchdog::EventPtr event,
                 sd_event_io_handler_t handler = PowerControl::EventHandler) :
        sdbusplus::server::object_t<pwr_control>(bus, path),
        bus(bus), callbackHandler(handler),
        timer(event, std::bind(&PowerControl::timeOutHandler, this))
    {
        int ret = -1;
        char buf = '0';

        // config gpio
        ret = configGpio(resetOutPinNum, resetOutPinDirection, &reset_out_fd,
                         bus);
        if (ret < 0)
        {
            throw std::runtime_error("failed to config RESET_OUT_PIN");
        }

        ret = configGpio(pgoodPinNum, pgoodPinDirection, &pgood_fd, bus);
        if (ret < 0)
        {
            closeGpio(reset_out_fd);
            throw std::runtime_error("failed to config PGOOD_PIN");
        }

        ret = configGpio(biosPostCmpltPinNum, biosPostCmpltPinDirection,
                         &bios_post_fd, bus);
        if (ret < 0)
        {
            closeGpio(reset_out_fd);
            closeGpio(pgood_fd);
            throw std::runtime_error("failed to config BIOS_POST_CMPLT_PIN");
        }

        ret = configGpio(powerUpPinNum, powerUpPinDirection, &power_up_fd, bus);
        if (ret < 0)
        {
            closeGpio(reset_out_fd);
            closeGpio(pgood_fd);
            closeGpio(bios_post_fd);
            throw std::runtime_error("failed to config POWER_UP_PIN");
        }

        ret = sd_event_add_io(event.get(), nullptr, pgood_fd, EPOLLPRI,
                              callbackHandler, this);
        if (ret < 0)
        {
            closeGpio(reset_out_fd);
            closeGpio(pgood_fd);
            closeGpio(bios_post_fd);
            closeGpio(power_up_fd);
            throw std::runtime_error("failed to add to event loop");
        }

        ret = sd_event_add_io(event.get(), nullptr, bios_post_fd, EPOLLPRI,
                              callbackHandler, this);
        if (ret < 0)
        {
            closeGpio(reset_out_fd);
            closeGpio(pgood_fd);
            closeGpio(bios_post_fd);
            closeGpio(power_up_fd);
            throw std::runtime_error("failed to add to event loop");
        }

        timer.start(std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::milliseconds(POLLING_INTERVAL_MS)));
        timer.setEnabled<std::true_type>();
        phosphor::logging::log<phosphor::logging::level::DEBUG>("Enable timer");
    }

    ~PowerControl()
    {
        closeGpio(reset_out_fd);
        closeGpio(pgood_fd);
        closeGpio(bios_post_fd);
        closeGpio(power_up_fd);
    }

    void timeOutHandler()
    {
        int fd = -1;
        struct sio_ioctl_data sio_data;

        fd = ::open(LPC_SIO_DEVPATH, O_RDWR);
        if (fd < 0)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "Open LPC-SIO error!");
            return;
        }

        sio_data.sio_cmd = SIO_GET_ACPI_STATE;
        sio_data.param = 0;
        if (::ioctl(fd, SIO_IOC_COMMAND, &sio_data) == 0)
        {
            if (s4s5State() != sio_data.data)
            {
                phosphor::logging::log<phosphor::logging::level::INFO>(
                    "ACPI state change\n",
                    phosphor::logging::entry("OLD=%d", s4s5State()),
                    phosphor::logging::entry("NEW=%d", sio_data.data));
                s4s5State(sio_data.data);
            }
        }
        else
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "ioctl SIO_GET_ACPI_STATE error!");
            ::close(fd);
            return;
        }

        sio_data.sio_cmd = SIO_GET_PWRGD_STATUS;
        sio_data.param = 0;
        if (::ioctl(fd, SIO_IOC_COMMAND, &sio_data) == 0)
        {
            if (vrdGood() != sio_data.data)
            {
                phosphor::logging::log<phosphor::logging::level::INFO>(
                    "VRD_PWR_GOOD change\n",
                    phosphor::logging::entry("OLD=%d", vrdGood()),
                    phosphor::logging::entry("NEW=%d", sio_data.data));
                vrdGood(sio_data.data);
            }
        }
        else
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "ioctl SIO_GET_PWRGD_STATUS error!");
            ::close(fd);
            return;
        }

        ::close(fd);

        this->timer.start(std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::milliseconds(POLLING_INTERVAL_MS)));
        this->timer.setEnabled<std::true_type>();
    }

    static int EventHandler(sd_event_source* es, int fd, uint32_t revents,
                            void* userdata)
    {
        // For the first event, only set the initial status,  do not emit signal
        // since is it not triggered by the real gpio change
        static bool first_event = true;
        int n = -1;
        char buf = '0';

        if (!userdata)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "userdata null!");
            return -1;
        }

        PowerControl* powercontrol = static_cast<PowerControl*>(userdata);

        if (!powercontrol)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "null pointer!");
            return -1;
        }

        n = ::lseek(fd, 0, SEEK_SET);
        if (n < 0)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "lseek error!");
            return n;
        }

        n = ::read(fd, &buf, sizeof(buf));
        if (n < 0)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "read error!");
            return n;
        }

        if (fd == powercontrol->pgood_fd)
        {
            if (buf == '0')
            {
                phosphor::logging::log<phosphor::logging::level::INFO>(
                    "!PSGOOD");
                powercontrol->state(0);
                powercontrol->pgood(0);
                if (first_event)
                {
                    first_event = false;
                }
                else
                {
                    powercontrol->powerLost();
                }
            }
            else
            {
                phosphor::logging::log<phosphor::logging::level::INFO>(
                    "PSGOOD");
                powercontrol->state(1);
                powercontrol->pgood(1);
                if (first_event)
                {
                    first_event = false;
                }
                else
                {
                    powercontrol->powerGood();
                }
            }
        }
        else if (fd == powercontrol->bios_post_fd)
        {
            if (buf == '0')
            {
                phosphor::logging::log<phosphor::logging::level::INFO>(
                    "BIOS POST COMPLETED");
                powercontrol->postComplete(true);
            }
            else
            {
                phosphor::logging::log<phosphor::logging::level::INFO>(
                    "!BIOS POST COMPLETED");
                powercontrol->postComplete(false);
            }
        }

        return 0;
    }

    int32_t forcePowerOff() override;
    int32_t setPowerState(int32_t newState) override;
    int32_t getPowerState() override;

  private:
    int reset_out_fd;
    int power_up_fd;
    int pgood_fd;
    int bios_post_fd;
    phosphor::watchdog::Timer timer;
    sdbusplus::bus::bus& bus;
    sd_event_io_handler_t callbackHandler;
    int32_t triggerReset();
};

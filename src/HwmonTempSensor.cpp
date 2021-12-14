/*
// Copyright (c) 2017 Intel Corporation
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

#include <unistd.h>

#include <HwmonTempSensor.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <iostream>
#include <istream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

// Temperatures are read in milli degrees Celsius, we need degrees Celsius.
// Pressures are read in kilopascal, we need Pascals.  On D-Bus for Open BMC
// we use the International System of Units without prefixes.
// Links to the kernel documentation:
// https://www.kernel.org/doc/Documentation/hwmon/sysfs-interface
// https://www.kernel.org/doc/Documentation/ABI/testing/sysfs-bus-iio
// For IIO RAW sensors we get a raw_value, an offset, and scale to compute
// the value = (raw_value + offset) * scale

std::vector<std::pair<size_t, size_t>> HwmonTempSensor::failedDevices{};

HwmonTempSensor::HwmonTempSensor(
    const std::string& path, const std::string& objectType,
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_service& io, const std::string& sensorName,
    std::vector<thresholds::Threshold>&& thresholdsIn,
    const struct SensorParams& thisSensorParameters, const float pollRate,
    const std::string& sensorConfiguration, const PowerState powerState,
    size_t bus, size_t address) :
    Sensor(boost::replace_all_copy(sensorName, " ", "_"),
           std::move(thresholdsIn), sensorConfiguration, objectType, false,
           false, thisSensorParameters.maxValue, thisSensorParameters.minValue,
           conn, powerState),
    std::enable_shared_from_this<HwmonTempSensor>(), objServer(objectServer),
    inputDev(io, open(path.c_str(), O_RDONLY)), waitTimer(io), path(path),
    offsetValue(thisSensorParameters.offsetValue),
    scaleValue(thisSensorParameters.scaleValue),
    sensorPollMs(static_cast<unsigned int>(pollRate * 1000)), bus(bus),
    address(address)
{
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/" + thisSensorParameters.typeName + "/" +
            name,
        "xyz.openbmc_project.Sensor.Value");

    if (thresholds::hasWarningInterface(thresholds))
    {
        thresholdInterfaceWarning = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/" + thisSensorParameters.typeName +
                "/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Warning");
    }
    if (thresholds::hasCriticalInterface(thresholds))
    {
        thresholdInterfaceCritical = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/" + thisSensorParameters.typeName +
                "/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Critical");
    }
    association = objectServer.add_interface("/xyz/openbmc_project/sensors/" +
                                                 thisSensorParameters.typeName +
                                                 "/" + name,
                                             association::interface);
    setInitialProperties(conn, thisSensorParameters.units);
}

HwmonTempSensor::~HwmonTempSensor()
{
    // close the input dev to cancel async operations
    inputDev.close();
    waitTimer.cancel();
    objServer.remove_interface(thresholdInterfaceWarning);
    objServer.remove_interface(thresholdInterfaceCritical);
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
}

void HwmonTempSensor::setupRead(void)
{
    if (!readingStateGood())
    {
        markAvailable(false);
        updateValue(std::numeric_limits<double>::quiet_NaN());
        restartRead();
        return;
    }

    std::weak_ptr<HwmonTempSensor> weakRef = weak_from_this();
    boost::asio::async_read_until(inputDev, readBuf, '\n',
                                  [weakRef](const boost::system::error_code& ec,
                                            std::size_t /*bytes_transfered*/) {
                                      std::shared_ptr<HwmonTempSensor> self =
                                          weakRef.lock();
                                      if (self)
                                      {
                                          self->handleResponse(ec);
                                      }
                                  });
}

void HwmonTempSensor::restartRead()
{
    std::weak_ptr<HwmonTempSensor> weakRef = weak_from_this();
    waitTimer.expires_from_now(boost::posix_time::milliseconds(sensorPollMs));
    waitTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }
        std::shared_ptr<HwmonTempSensor> self = weakRef.lock();
        if (!self)
        {
            return;
        }
        self->setupRead();
    });
}

void HwmonTempSensor::handleResponse(const boost::system::error_code& err)
{
    errorCode = err;
    if ((err == boost::system::errc::bad_file_descriptor) ||
        (err == boost::asio::error::misc_errors::not_found))
    {
        std::cerr << "Hwmon temp sensor " << name << " removed " << path
                  << "\n";
        return; // we're being destroyed
    }
    std::istream responseStream(&readBuf);
    if (!err)
    {
        std::string response;
        std::getline(responseStream, response);
        try
        {
            rawValue = std::stod(response);
            double nvalue = (rawValue + offsetValue) * scaleValue;
            updateValue(nvalue);
        }
        catch (const std::invalid_argument&)
        {
            if (incrementError())
            {
                errorCode = boost::system::errc::make_error_code(
                    boost::system::errc::invalid_argument);
                createEventLog();
            }
        }
    }
    else
    {
        if (incrementError())
        {
            createEventLog();
        }
    }

    responseStream.clear();
    inputDev.close();
    int fd = open(path.c_str(), O_RDONLY);
    if (fd < 0)
    {
        std::cerr << "Hwmon temp sensor " << name << " not valid " << path
                  << "\n";
        return; // we're no longer valid
    }
    inputDev.assign(fd);
    restartRead();
}

void HwmonTempSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}

void HwmonTempSensor::clearFailedDevices()
{
    failedDevices.clear();
}

// Creates a ReadFailure event log if the power is on and if the
// device hasn't previously had an error logged against it.
void HwmonTempSensor::createEventLog()
{
    if (!isPowerOn())
    {
        return;
    }

    if (std::find(failedDevices.begin(), failedDevices.end(),
                  std::make_pair(bus, address)) != failedDevices.end())
    {
        // Already have a failure on this device
        return;
    }

    failedDevices.emplace_back(bus, address);

    std::cerr << "Creating event log for sensor " << name << " read failure on "
              << path << " with error code " << errorCode.value() << "\n";

    std::map<std::string, std::string> additionalData;

    additionalData["SENSOR_NAME"] = name;
    additionalData["CALLOUT_IIC_BUS"] = std::to_string(bus);
    additionalData["CALLOUT_IIC_ADDR"] = std::to_string(address);
    additionalData["CALLOUT_ERRNO"] = std::to_string(errorCode.value());
    additionalData["IIC_ERROR_CATEGORY"] = errorCode.category().name();
    additionalData["IIC_ERROR_MESSAGE"] =
        errorCode.category().message(errorCode.value());
    additionalData["FILE"] = path;
    additionalData["_PID"] = std::to_string(getpid());

    dbusConnection->async_method_call(
        [this](const boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << "Failed to create an event log for "
                             "a failed read on sensor "
                          << name << "\n";
                return;
            }
        },
        "xyz.openbmc_project.Logging", "/xyz/openbmc_project/logging",
        "xyz.openbmc_project.Logging.Create", "Create",
        "xyz.openbmc_project.Sensor.Device.Error.ReadFailure",
        "xyz.openbmc_project.Logging.Entry.Level.Error", additionalData);
}

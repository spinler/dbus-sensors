/*
// Copyright (c) 2019 Intel Corporation
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

#include <NVMeSensor.hpp>

#include <filesystem>
#include <iostream>

static constexpr double maxReading = 127;
static constexpr double minReading = 0;

NVMeSensor::NVMeSensor(sdbusplus::asio::object_server& objectServer,
                       boost::asio::io_service&,
                       std::shared_ptr<sdbusplus::asio::connection>& conn,
                       const std::string& sensorName,
                       std::vector<thresholds::Threshold>&& thresholdsIn,
                       const std::string& sensorConfiguration,
                       const int busNumber) :
    Sensor(escapeName(sensorName), std::move(thresholdsIn), sensorConfiguration,
           NVMeSensor::CONFIG_TYPE, false, false, maxReading, minReading, conn,
           PowerState::on),
    bus(busNumber), objServer(objectServer), scanDelay(0)
{
    if (bus < 0)
    {
        throw std::invalid_argument("Invalid bus: Bus ID must not be negative");
    }

    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/temperature/" + name,
        "xyz.openbmc_project.Sensor.Value");

    if (thresholds::hasWarningInterface(thresholds))
    {
        thresholdInterfaceWarning = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/temperature/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Warning");
    }
    if (thresholds::hasCriticalInterface(thresholds))
    {
        thresholdInterfaceCritical = objectServer.add_interface(
            "/xyz/openbmc_project/sensors/temperature/" + name,
            "xyz.openbmc_project.Sensor.Threshold.Critical");
    }
    association = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/temperature/" + name,
        association::interface);

    setInitialProperties(conn, sensor_paths::unitDegreesC);
}

NVMeSensor::~NVMeSensor()
{
    // close the input dev to cancel async operations
    objServer.remove_interface(thresholdInterfaceWarning);
    objServer.remove_interface(thresholdInterfaceCritical);
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
}

void NVMeSensor::createAssociation()
{
    std::filesystem::path p(configurationPath);
    dbusConnection->async_method_call(
        [association = association,
         configurationPath = this->configurationPath](
            const boost::system::error_code ec,
            const std::variant<std::vector<Association>>& envelope) {
            if (ec == boost::asio::error::operation_aborted)
            {
                return;
            }

            if (ec)
            {
                std::cerr << "Error getting inventory association for "
                          << configurationPath << ": " << ec.message() << "\n";
                return;
            }

            std::vector<Association> sensorAssociations = {
                {
                    "chassis",
                    "all_sensors",
                    "/xyz/openbmc_project/inventory/system/chassis",
                },
            };

            auto configAssociations =
                std::get<std::vector<Association>>(envelope);
            for (const auto& entry : configAssociations)
            {
                if (std::get<0>(entry) == "probed_by")
                {
                    const auto& invPath = std::get<2>(entry);
                    sensorAssociations.emplace_back("inventory", "sensors",
                                                    invPath);
                }
            }

            association->register_property("Associations", sensorAssociations);
            association->initialize();
        },
        entityManagerName, p.parent_path().string(),
        "org.freedesktop.DBus.Properties", "Get", association::interface,
        "Associations");
}

bool NVMeSensor::sample()
{
    if (inError())
    {
        if (scanDelay == 0)
        {
            scanDelay = scanDelayTicks;
        }

        scanDelay--;
    }

    return scanDelay == 0;
}

void NVMeSensor::checkThresholds(void)
{
    thresholds::checkThresholds(this);
}

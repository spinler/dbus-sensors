#include "SlotPowerManager.hpp"

#include <boost/algorithm/string.hpp>
#include <phosphor-logging/lg2.hpp>

#include <filesystem>
#include <fstream>

PHOSPHOR_LOG2_USING;
namespace fs = std::filesystem;

std::unique_ptr<SlotPowerManager> slotPowerManager;

static const char* locationCodeInterface =
    "xyz.openbmc_project.Inventory.Decorator.LocationCode";
static const char* pcieSlotInterface =
    "xyz.openbmc_project.Inventory.Item.PCIeSlot";

constexpr auto baseI2CPath = "/sys/bus/i2c/devices/i2c-";

void SlotPowerManager::update(const ManagedObjectType& sensorConfigurations)
{
    std::vector<std::string> newSlots;
    std::vector<DeviceInfo> deviceConfigs;
    static std::map<std::string, std::string> slots;

    // Pull the applicable device configs out of the EM D-Bus objects
    getDeviceConfigs(sensorConfigurations, deviceConfigs);
    if (deviceConfigs.empty())
    {
        return;
    }

    debug("Found {SIZE} device configs", "SIZE", deviceConfigs.size());

    // Get map of locations codes to their slots
    if (slots.empty())
    {
        getSlots(slots);
    }

    // Build the slotDevices map
    for (auto& deviceInfo : deviceConfigs)
    {
        auto slotNameIt = slots.find(deviceInfo.locationCode);
        if (slotNameIt == slots.end())
        {
            error("No slot found for sensor location code {LOC}", "LOC",
                  deviceInfo.locationCode);
            continue;
        }

        const auto& slotName = slotNameIt->second;
        auto powerState = getPowerState(slotName);

        auto slotDataIt = slotDevices.find(slotName);
        if (slotDataIt == slotDevices.end())
        {
            debug("Initial power state of {SLOT} is {STATE}", "SLOT", slotName,
                  "STATE", powerState);
            deviceInfo.state = powerState;
            slotDevices.emplace(slotName, std::vector{deviceInfo});
            newSlots.push_back(slotName);
        }
        else
        {
            // Add another device to this slot.  As update() can get
            // called multiple times with the same set of slots, only
            // allow this if we added the slot for the first time in
            // this function call.
            if (std::find(newSlots.begin(), newSlots.end(), slotName) !=
                newSlots.end())
            {
                deviceInfo.state = powerState;
                slotDataIt->second.push_back(deviceInfo);
            }
        }
    }

    // Bind device drivers for devices that are present and powered on
    bindDrivers();
}

// Find the devices on slot power
void SlotPowerManager::getDeviceConfigs(
    const ManagedObjectType& sensorConfigurations,
    std::vector<DeviceInfo>& deviceConfigs)
{
    for (const auto& [objectPath, sensorData] : sensorConfigurations)
    {
        for (const auto& [type, config] : sensorData)
        {
            auto ownerIt = config.find("PowerStateOwner");
            if (ownerIt == config.end())
            {
                continue;
            }

            const auto& owner = std::get<std::string>(ownerIt->second);
            if (owner != "PCIeSlot")
            {
                error("Invalid PowerStateOwner type: {OWNER} in entity-manager "
                      "config",
                      "OWNER", owner);
                continue;
            }

            auto locCodeIt = config.find("LocationCode");
            if (locCodeIt == config.end())
            {
                error("Missing LocationCode entry on {PATH}", "PATH",
                      objectPath.str);
                continue;
            }

            if (!config.contains("Bus") || !config.contains("Address"))
            {
                error("Missing Bus or Address for {PATH}", "PATH",
                      objectPath.str);
                continue;
            }

            if (!std::get_if<uint64_t>(&config.at("Bus")) ||
                !std::get_if<uint64_t>(&config.at("Address")))
            {
                error("Invalid Bus or Address for {PATH}", "PATH",
                      objectPath.str);
                continue;
            }

            DeviceInfo deviceInfo;
            deviceInfo.name = std::get<std::string>(config.at("Name"));
            deviceInfo.type = std::get<std::string>(config.at("Type"));
            deviceInfo.bus = std::get<uint64_t>(config.at("Bus"));
            deviceInfo.address = std::get<uint64_t>(config.at("Address"));
            deviceInfo.locationCode = std::get<std::string>(locCodeIt->second);

            debug("DeviceInfo: Name: {NAME}, Type: {TYPE}", "NAME",
                  deviceInfo.name, "TYPE", deviceInfo.type);

            deviceConfigs.push_back(std::move(deviceInfo));
        }
    }
}

// Build the locCode->slotPath map
void SlotPowerManager::getSlots(std::map<std::string, std::string>& slots)
{
    auto& bus = static_cast<sdbusplus::bus::bus&>(*systemBus);

    auto method = bus.new_method_call(mapper::busName, mapper::path,
                                      mapper::interface, mapper::subtree);
    method.append(std::string{"/"}, 0,
                  std::vector<std::string>{pcieSlotInterface});
    auto reply = bus.call(method);

    GetSubTreeType subtree;
    reply.read(subtree);

    for (const auto& [path, objDict] : subtree)
    {
        const auto& busName = objDict[0].first;
        auto method =
            bus.new_method_call(busName.c_str(), path.c_str(),
                                properties::interface, properties::get);
        method.append(locationCodeInterface, "LocationCode");

        try
        {
            auto reply = bus.call(method);

            std::variant<std::string> locationCode;
            reply.read(locationCode);
            const auto& locCode = std::get<std::string>(locationCode);
            slots.emplace(locCode, path);
        }
        catch (const sdbusplus::exception::exception& ex)
        {
            error("Failed getting location code from {PATH}: {EX}", "PATH",
                  path, "EX", ex);
        }
    }
}

std::string SlotPowerManager::getPowerState(const std::string& path)
{
    auto& bus = static_cast<sdbusplus::bus::bus&>(*systemBus);
    static GetSubTreeType subtree;

    // Get the list of power state objects just once.
    if (subtree.empty())
    {
        auto method = bus.new_method_call(mapper::busName, mapper::path,
                                          mapper::interface, mapper::subtree);
        method.append(std::string{"/"}, 0,
                      std::vector<std::string>{powerStateInterface});
        auto reply = bus.call(method);
        reply.read(subtree);
    }

    auto entryIt = std::find_if(subtree.begin(), subtree.end(),
                                [path](const auto& subtreeEntry) {
                                    return path == subtreeEntry.first;
                                });
    if (entryIt == subtree.end())
    {
        throw std::runtime_error("No power state interface on " + path);
    }

    const auto& services = entryIt->second;

    auto method = bus.new_method_call(services[0].first.c_str(), path.c_str(),
                                      properties::interface, properties::get);
    method.append(powerStateInterface, "PowerState");

    auto reply = bus.call(method);

    std::variant<std::string> powerState;
    reply.read(powerState);

    return std::get<std::string>(powerState);
}

void SlotPowerManager::powerStateChanged(sdbusplus::message::message& msg)
{
    std::string objectPath = msg.get_path();
    std::string interface;
    std::map<std::string, std::variant<std::string>> properties;

    msg.read(interface, properties);

    auto propIt = properties.find("PowerState");
    if (propIt == properties.end())
    {
        return;
    }

    if (!slotDevices.contains(objectPath))
    {
        // Don't care about this slot
        return;
    }

    debug("Power state of {PATH} changed to {STATE}", "PATH", objectPath,
          "STATE", std::get<std::string>(propIt->second));

    const auto& powerState = std::get<std::string>(propIt->second);

    // Update the state inside the config data.
    auto& deviceConfigs = slotDevices.at(objectPath);
    std::for_each(
        deviceConfigs.begin(), deviceConfigs.end(),
        [powerState, this](auto& config) { config.state = powerState; });

    // Call slotOnFunc with the name of the sensor that changed
    if (boost::ends_with(powerState, "On") && slotOnFunc)
    {
        auto sensors =
            std::make_shared<boost::container::flat_set<std::string>>();

        std::for_each(deviceConfigs.begin(), deviceConfigs.end(),
                      [&sensors](const auto& config) {
                          // createSensors() doesn't want spaces
                          auto name =
                              boost::replace_all_copy(config.name, " ", "_");
                          sensors->insert(name);
                      });
        // Call createSensors() (which calls update())
        slotOnFunc(sensors);
    }
}

void SlotPowerManager::bindDrivers()
{
    for (const auto& [_, devices] : slotDevices)
    {
        for (const auto& device : devices)
        {
            if (boost::ends_with(device.state, "On"))
            {
                newDevice(device);
            }
        }
    }
}

void SlotPowerManager::newDevice(const DeviceInfo& device)
{
    if (deviceExists(device))
    {
        return;
    }

    std::string driverFile =
        baseI2CPath + std::to_string(device.bus) + "/new_device";

    std::ostringstream typeAddr; // e.g. "tmp435 0x4c"
    auto type = boost::algorithm::to_lower_copy(device.type);
    typeAddr << type << " 0x" << std::hex << device.address;

    std::ofstream file{driverFile};
    if (!file)
    {
        error("Could not open {FILE}", "FILE", driverFile);
        return;
    }

    file << typeAddr.str();
    if (file.bad() || file.fail())
    {
        auto e = errno;
        error("Error writing {DATA} to {FILE}, errno {ERRNO}", "DATA",
              typeAddr.str(), "FILE", driverFile, "ERRNO", e);
    }
}

bool SlotPowerManager::deviceExists(const DeviceInfo& device)
{
    std::ostringstream deviceFile;

    // e.g. /sys/bus/i2c/devices/i2c-29/29-004c
    deviceFile << baseI2CPath << device.bus << "/" << device.bus << "-"
               << std::setfill('0') << std::setw(4) << std::hex
               << device.address;

    return fs::exists(deviceFile.str());
}

bool SlotPowerManager::isDeviceOff(uint64_t bus, uint64_t address) const
{
    for (const auto& [_, devices] : slotDevices)
    {
        auto deviceIt = std::find_if(
            devices.begin(), devices.end(), [bus, address](const auto& device) {
                return (bus == device.bus) && (address == device.address);
            });

        if (deviceIt != devices.end())
        {
            return boost::ends_with(deviceIt->state, "Off") || !isPowerOn();
        }
    }
    return false;
}

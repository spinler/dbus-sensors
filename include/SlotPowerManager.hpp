#pragma once
#include <Utils.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>

using SlotOnFuncType = std::function<void(
    const std::shared_ptr<boost::container::flat_set<std::string>>&)>;

static const char* powerStateInterface =
    "xyz.openbmc_project.State.Decorator.PowerState";

/**
 * This class assists with handling sensor devices that are on the
 * same power domain as the slot the card plugs into.
 *
 * It requires
 *    "PowerStateOwner": "PCIeSlot",
 *    "LocationCode": "<location code of slot>"
 * in the entity-manager config to activate it for a device.
 *
 * public methods:
 *   isDeviceOff()
 *     - If the device is powered off or not based on the slot power state
 *
 *   update()
 *     - Called from createSensors() to refresh the slot power status
 *       so devices can be read if they are powered on.
 *
 * It also has propertiesChanged watches on the slot power states
 * so it can call createSensors() again when slots are turned on so
 * the sensor objects can be created.
 */
class SlotPowerManager
{
  public:
    SlotPowerManager(sdbusplus::asio::connection& systemBus,
                     SlotOnFuncType slotOnFunc) :
        systemBus(systemBus),
        match(static_cast<sdbusplus::bus::bus&>(systemBus),
              "type='signal',member='PropertiesChanged',path_namespace='" +
                  std::string(inventoryPath) + "',arg0namespace='" +
                  powerStateInterface + "'",
              [this](
                  sdbusplus::message_t& msg) { this->powerStateChanged(msg); }),
        slotOnFunc(std::move(slotOnFunc))
    {}

    bool isDeviceOff(uint64_t bus, uint64_t address) const;

    bool isDeviceOff(const SensorBaseConfigMap& cfg) const;

    void update(const ManagedObjectType& sensorConfigurations);

  private:
    struct DeviceInfo
    {
        std::string name;
        std::string type;
        std::string state;
        std::string locationCode;
        uint64_t bus{};
        uint64_t address{};
    };

    void powerStateChanged(sdbusplus::message::message& msg);
    static void getDeviceConfigs(const ManagedObjectType& sensorConfigurations,
                                 std::vector<DeviceInfo>& deviceConfigs);
    void getSlots(std::map<std::string, std::string>& slots);
    std::string getPowerState(const std::string& path);

    sdbusplus::asio::connection& systemBus;

    sdbusplus::bus::match::match match;

    std::map<std::string, std::vector<DeviceInfo>> slotDevices;

    SlotOnFuncType slotOnFunc;
};

extern std::unique_ptr<SlotPowerManager> slotPowerManager;

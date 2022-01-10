#pragma once
#include <Utils.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/asio/io_service.hpp>
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
 *     - Called from createSensors() to bind device drivers to their
 *       device when they are powered on so they can be read.
 *
 * It also has propertiesChanged watches on the slot power states
 * so it can call createSensors() again when slots are turned on so
 * the sensor objects can be created.
 */
class SlotPowerManager
{
  public:
    SlotPowerManager(boost::asio::io_service& io,
                     std::shared_ptr<sdbusplus::asio::connection> systemBus,
                     SlotOnFuncType slotOnFunc) :
        io(io),
        systemBus(systemBus),
        match(static_cast<sdbusplus::bus::bus&>(*systemBus),
              "type='signal',member='PropertiesChanged',path_namespace='" +
                  std::string(inventoryPath) + "',arg0namespace='" +
                  powerStateInterface + "'",
              std::bind(&SlotPowerManager::powerStateChanged, this,
                        std::placeholders::_1)),
        slotOnFunc(slotOnFunc)
    {}

    bool isDeviceOff(uint64_t bus, uint64_t address) const;

    void update(const ManagedObjectType& sensorConfigurations);

  private:
    struct DeviceInfo
    {
        std::string name;
        std::string type;
        std::string state;
        std::string locationCode;
        uint64_t bus;
        uint64_t address;
    };

    void powerStateChanged(sdbusplus::message::message& msg);
    void getDeviceConfigs(const ManagedObjectType& sensorConfigurations,
                          std::vector<DeviceInfo>& deviceConfigs);
    void getSlots(std::map<std::string, std::string>& slots);
    void newDevice(const DeviceInfo& device);
    void bindDrivers();
    bool deviceExists(const DeviceInfo& device);
    std::string getPowerState(const std::string& path);

    boost::asio::io_service& io;

    std::shared_ptr<sdbusplus::asio::connection> systemBus;

    sdbusplus::bus::match::match match;

    std::map<std::string, std::vector<DeviceInfo>> slotDevices;

    SlotOnFuncType slotOnFunc;
};

extern std::unique_ptr<SlotPowerManager> slotPowerManager;

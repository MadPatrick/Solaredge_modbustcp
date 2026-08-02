#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# SolarEdge ModbusTCP
#
# Source:  https://github.com/addiejanssen/domoticz-solaredge-modbustcp-plugin
# Author:  Addie Janssen (https://addiejanssen.com)
# License: MIT
#

"""
<plugin key="SolarEdge_ModbusTCP" name="SolarEdge ModbusTCP" author="Addie Janssen" version="2.1.4" externallink="https://github.com/MadPatrick/Solaredge_modbustcp">
    <description>
        <h2>SolarEdge Modbus TCP</h2>
        <p><strong>Version:</strong> 2.1.4</p>
        <p>Reads SolarEdge inverter, meter and battery data directly over Modbus TCP.</p>
        <h3>Features</h3>
        <ul>
            <li>Inverter status, electrical measurements, power, energy and temperature.</li>
            <li>Optional automatic discovery of connected meters and batteries.</li>
            <li>Creates missing Domoticz devices when enabled.</li>
            <li>Optional averaging and maximum-value calculations for selected measurements.</li>
            <li>Configurable polling interval and diagnostic log level.</li>
        </ul>
        <h3>Configuration</h3>
        <p>Enable Modbus TCP on the inverter, then enter its IP address, port and Modbus device address.</p>
    </description>
    <params>
        <param field="Address" label="Inverter IP Address" width="150px" required="true" />
        <param field="Port" label="Inverter Port Number" width="150px" required="true" default="502" />
        <param field="Mode3" label="Modbus device address" width="150px" required="true" default="1" />

        <param field="Mode6" label="Hardware components" width="150px" required="true" default="0" >
            <options>
                <option label="Inverter"                  value="0" default="true" />
                <option label="Inverter+Meters"           value="1"                />
                <option label="Inverter+Batteries"        value="2"                />
                <option label="Inverter+Meters+Batteries" value="3"                />
            </options>
        </param>

        <param field="Mode1" label="Add missing devices" width="150px" required="true" default="Yes" >
            <options>
                <option label="Yes" value="Yes" default="true" />
                <option label="No"  value="No"                 />
            </options>
        </param>

        <param field="Mode2" label="Interval" width="150px" required="true" default="5" >
            <options>
                <option label="1  second"  value="1"                />
                <option label="2  seconds" value="2"                />
                <option label="3  seconds" value="3"                />
                <option label="4  seconds" value="4"                />
                <option label="5  seconds" value="5" default="true" />
                <option label="10 seconds" value="10"               />
                <option label="20 seconds" value="20"               />
                <option label="30 seconds" value="30"               />
                <option label="60 seconds" value="60"               />
            </options>
        </param>

        <param field="Mode4" label="Automatic average/maximum" width="150px">
            <options>
                <option label="Enabled"  value="Yes" default="true" />
                <option label="Disabled" value="No"                 />
            </options>
        </param>

        <param field="Mode5" label="Log level" width="150px">
            <options>
                <option label="Normal"    value="0" default="true" />
                <option label="Verbose"   value="1"                />
                <option label="Verbose+"  value="2"                />
                <option label="Verbose++" value="3"                />
            </options>
        </param>
    </params>
</plugin>
"""

import Domoticz
import inspect
import json
import sys
import types

from datetime import datetime, timedelta
from enum import IntEnum, unique
from pymodbus.exceptions import ConnectionException


def _apply_pymodbus_legacy_compat():
    try:
        import pymodbus.constants as pymodbus_constants
    except ImportError:
        return

    if not hasattr(pymodbus_constants, "Endian"):
        class Endian:
            BIG = "big"
            LITTLE = "little"
            Big = "big"
            Little = "little"

        pymodbus_constants.Endian = Endian

    try:
        import pymodbus.payload
    except ImportError:
        from pymodbus.client import ModbusBaseClient

        class BinaryPayloadDecoder:
            def __init__(self, registers, byteorder="big", wordorder="big"):
                self._registers = list(registers)
                self._wordorder = wordorder

            @classmethod
            def fromRegisters(cls, registers, byteorder="big", wordorder="big"):
                return cls(registers, byteorder=byteorder, wordorder=wordorder)

            def _decode(self, data_type, count=1):
                registers = self._registers[:count]
                self._registers = self._registers[count:]
                return ModbusBaseClient.convert_from_registers(
                    registers,
                    data_type,
                    word_order=self._wordorder,
                )

            def decode_16bit_int(self):
                return self._decode(ModbusBaseClient.DATATYPE.INT16)

            def decode_16bit_uint(self):
                return self._decode(ModbusBaseClient.DATATYPE.UINT16)

            def decode_32bit_int(self):
                return self._decode(ModbusBaseClient.DATATYPE.INT32, 2)

            def decode_32bit_uint(self):
                return self._decode(ModbusBaseClient.DATATYPE.UINT32, 2)

            def decode_64bit_uint(self):
                return self._decode(ModbusBaseClient.DATATYPE.UINT64, 4)

            def decode_32bit_float(self):
                return self._decode(ModbusBaseClient.DATATYPE.FLOAT32, 2)

            def decode_string(self, size):
                register_count = (size + 1) // 2
                value = self._decode(ModbusBaseClient.DATATYPE.STRING, register_count)
                return value.encode("utf-8")

            def skip_bytes(self, count):
                register_count = (count + 1) // 2
                self._registers = self._registers[register_count:]

        class BinaryPayloadBuilder:
            def __init__(self, byteorder="big", wordorder="big"):
                self._registers = []
                self._wordorder = wordorder

            def _add(self, value, data_type):
                self._registers.extend(
                    ModbusBaseClient.convert_to_registers(
                        value,
                        data_type,
                        word_order=self._wordorder,
                    )
                )

            def add_16bit_int(self, value):
                self._add(value, ModbusBaseClient.DATATYPE.INT16)

            def add_16bit_uint(self, value):
                self._add(value, ModbusBaseClient.DATATYPE.UINT16)

            def add_32bit_int(self, value):
                self._add(value, ModbusBaseClient.DATATYPE.INT32)

            def add_32bit_uint(self, value):
                self._add(value, ModbusBaseClient.DATATYPE.UINT32)

            def add_64bit_uint(self, value):
                self._add(value, ModbusBaseClient.DATATYPE.UINT64)

            def add_32bit_float(self, value):
                self._add(value, ModbusBaseClient.DATATYPE.FLOAT32)

            def add_string(self, value):
                self._add(value, ModbusBaseClient.DATATYPE.STRING)

            def to_registers(self):
                return self._registers

        payload_module = types.ModuleType("pymodbus.payload")
        payload_module.BinaryPayloadDecoder = BinaryPayloadDecoder
        payload_module.BinaryPayloadBuilder = BinaryPayloadBuilder
        sys.modules["pymodbus.payload"] = payload_module

    try:
        import pymodbus.register_read_message
    except ImportError:
        from pymodbus.pdu.register_message import ReadHoldingRegistersResponse

        register_read_message_module = types.ModuleType("pymodbus.register_read_message")
        register_read_message_module.ReadHoldingRegistersResponse = ReadHoldingRegistersResponse
        sys.modules["pymodbus.register_read_message"] = register_read_message_module

    try:
        from pymodbus.client import ModbusSerialClient, ModbusTcpClient
    except ImportError:
        return

    if "pymodbus.client.sync" not in sys.modules:
        try:
            import pymodbus.client.sync  # noqa: F401
        except ImportError:
            sync_module = types.ModuleType("pymodbus.client.sync")
            sync_module.ModbusTcpClient = ModbusTcpClient
            sync_module.ModbusSerialClient = ModbusSerialClient
            try:
                from pymodbus.client import ModbusUdpClient
                sync_module.ModbusUdpClient = ModbusUdpClient
            except ImportError:
                pass
            sys.modules["pymodbus.client.sync"] = sync_module

    def _wrap_read_holding_registers(original, uses_device_id):
        def read_holding_registers(self, address, count=1, **kwargs):
            if uses_device_id:
                if "slave" in kwargs and "device_id" not in kwargs:
                    kwargs["device_id"] = kwargs.pop("slave")
                if "unit" in kwargs and "device_id" not in kwargs:
                    kwargs["device_id"] = kwargs.pop("unit")
                kwargs.pop("unit", None)
                kwargs.pop("slave", None)
                return original(self, address, count=count, **kwargs)
            return original(self, address, count, **kwargs)

        return read_holding_registers

    def _wrap_write_registers(original, uses_device_id):
        def write_registers(self, address, values, **kwargs):
            if uses_device_id:
                if "slave" in kwargs and "device_id" not in kwargs:
                    kwargs["device_id"] = kwargs.pop("slave")
                if "unit" in kwargs and "device_id" not in kwargs:
                    kwargs["device_id"] = kwargs.pop("unit")
                kwargs.pop("unit", None)
                kwargs.pop("slave", None)
            return original(self, address, values, **kwargs)

        return write_registers

    for client_cls in (ModbusTcpClient, ModbusSerialClient):
        if not getattr(client_cls, "_solaredge_legacy_api", False):
            uses_device_id = "device_id" in inspect.signature(client_cls.read_holding_registers).parameters
            client_cls.read_holding_registers = _wrap_read_holding_registers(
                client_cls.read_holding_registers,
                uses_device_id,
            )
            client_cls.write_registers = _wrap_write_registers(
                client_cls.write_registers,
                uses_device_id,
            )
            client_cls._solaredge_legacy_api = True

    if not getattr(ModbusSerialClient, "_solaredge_legacy_init", False):
        original_serial_init = ModbusSerialClient.__init__
        accepts_method = "method" in inspect.signature(original_serial_init).parameters

        if not accepts_method:
            def serial_init(self, *args, **kwargs):
                kwargs.pop("method", None)
                original_serial_init(self, *args, **kwargs)

            ModbusSerialClient.__init__ = serial_init

        ModbusSerialClient._solaredge_legacy_init = True


_apply_pymodbus_legacy_compat()

import solaredge_modbus

import inverters
import meters
import batteries

from helpers import DomoLog, LogLevels, SetLogLevel

#
# The plugin is using a few tables to setup Domoticz and to process the feedback from the inverter.
# The Column class is used to easily identify the columns in those tables.
#

@unique
class Column(IntEnum):

    ID              = 0
    NAME            = 1
    TYPE            = 2
    SUBTYPE         = 3
    SWITCHTYPE      = 4
    OPTIONS         = 5
    MODBUSNAME      = 6
    MODBUSSCALE     = 7
    FORMAT          = 8
    PREPEND_ROW     = 9
    PREPEND_MATH    = 10
    APPEND_MATH     = 11
    LOOKUP          = 12
    MATH            = 13


#
# The BasePlugin is the actual Domoticz plugin.
# This is where the fun starts :-)
#

class BasePlugin:

    def __init__(self):

        # The device dictionary will hold an entry for the inverter and each meter and battery (if applicable)
        # For each device, it will mention a name, the actual lookup table and a device index offset

        self.device_dictionary = {}

        # This is the solaredge_modbus Inverter object that will be used to communicate with the inverter.

        self.inverter = None
        self.inverter_address = None
        self.inverter_port = None
        self.inverter_unit = None

        # Default heartbeat is 10 seconds; therefore 30 samples in 5 minutes.

        self.max_samples = 30

        # Whether we should scan for meters and/or batteries

        self.scan_for_meters = False
        self.scan_for_batteries = False

        # Whether the plugin should add missing devices.
        # If set to True, a deleted device will be added on the next restart of Domoticz.

        self.add_devices = False

        # The Domoticz image ID for the SolarEdge icon.

        self.imageID = 0

        # The inverter, meter and battery tables provide an option to calculate
        # averages or maximum values. This is used to have nice graphs in Domoticz.
        # Some users don't want that; they want to have the actual values and store
        # them (via Domoticz) in external databases or use them in scripts.
        # If set to True, then the math is enabled otherwise we just passthrough

        self.do_math = True

        # When there is an issue contacting the inverter, the plugin will retry after a certain retry delay.
        # The actual time after which the plugin will try again is stored in the retry after variable.
        # According to the documenation, the inverter may need up to 2 minutes to "reset".

        self.retrydelay = timedelta(minutes = 2)
        self.retryafter = datetime.now() - timedelta(seconds = 1)

    def _load_device_icon(self):
        _IMAGE = "solaredge"
        existing_image = next(
            (image for name, image in Images.items()
             if str(name).casefold() == _IMAGE.casefold()),
            None,
        )
        if existing_image is not None:
            self.imageID = existing_image.ID
            Domoticz.Log(f"Icons found in database (ImageID={self.imageID}).")
            return

        try:
            Domoticz.Image(f"{_IMAGE}.zip").Create()
        except Exception as e:
            Domoticz.Error(f"Unable to load icon pack '{_IMAGE}.zip': {e}")
            return
        created_image = next(
            (image for name, image in Images.items()
             if str(name).casefold() == _IMAGE.casefold()),
            None,
        )
        if created_image is not None:
            self.imageID = created_image.ID
            Domoticz.Log("Icons created and loaded.")
        else:
            Domoticz.Error(f"Unable to load icon pack '{_IMAGE}.zip'")

    def _apply_device_icon(self):
        if not self.imageID:
            return
        for device in Devices.values():
            if device.Image != self.imageID:
                device.Update(nValue=device.nValue, sValue=device.sValue, Image=self.imageID)

    def _read_int_parameter(self, field, default, minimum=None, maximum=None):
        raw = Parameters.get(field, "")
        if raw is None or str(raw).strip() == "":
            return default
        try:
            value = int(raw)
            if minimum is not None and value < minimum:
                raise ValueError
            if maximum is not None and value > maximum:
                raise ValueError
            return value
        except (TypeError, ValueError):
            Domoticz.Error(
                "Invalid {} value '{}'. Using default {}.".format(
                    field, raw, default
                )
            )
            return default

    def _find_device_unit(self, unit_number):
        for device_name, device_details in self.device_dictionary.items():
            table = device_details.get("table")
            if not table:
                continue

            offset = device_details.get("offset", 0)
            for unit in table:
                if unit[Column.ID] + offset == unit_number:
                    return device_name, device_details, unit

        return None, None, None

    def _normalize_percentage_level(self, value):
        return max(0, min(100, int(round(float(value)))))

    def _get_power_limit_target(self, command, level, current_level):
        if command == "Off":
            return 0

        if command == "On":
            if current_level > 0:
                return current_level
            if level and int(level) > 0:
                return self._normalize_percentage_level(level)
            return 100

        if command == "Set Level":
            return self._normalize_percentage_level(level)

        raise ValueError("Unsupported command: {}".format(command))

    def _update_device_state(self, device, unit, value):
        if unit[Column.MODBUSNAME] == "active_power_limit":
            level = self._normalize_percentage_level(value)
            nValue = 1 if level > 0 else 0
            sValue = str(level)
        else:
            nValue = 0
            sValue = str(value)

        if nValue != device.nValue or sValue != device.sValue:
            device.Update(nValue=nValue, sValue=sValue, TimedOut=0)

    def _write_inverter_value(self, key, value):
        response = self.inverter.write(key, value)
        if hasattr(response, "isError") and response.isError():
            raise RuntimeError("Modbus write failed for {}".format(key))
        return response

    #
    # onStart is called by Domoticz to start the processing of the plugin.
    #

    def onStart(self):
        DomoLog(LogLevels.EXTRA, "Entered onStart()")

        self._load_device_icon()
        self._apply_device_icon()

        # Get the choices of the user and turn them into something we can use

        # Mode 6 defines which hardware components we should scan for
        components = self._read_int_parameter("Mode6", 0, 0, 3)
        if components == 1:
            self.scan_for_meters = True
        elif components == 2:
            self.scan_for_batteries = True
        elif components == 3:
            self.scan_for_meters = True
            self.scan_for_batteries = True
        else:
            self.scan_for_meters = False
            self.scan_for_batteries = False

        # Mode 1 defines if we should add missing devices or not
        if Parameters["Mode1"] == "Yes":
            self.add_devices = True
        else:
            self.add_devices = False

        # Mode 4 defines if we should do math or not
        if Parameters["Mode4"] == "Yes":
            self.do_math = True
        else:
            self.do_math = False

        # Domoticz will generate graphs showing an interval of 5 minutes.
        # Calculate the number of samples to store over a period of 5 minutes.
        poll_interval = self._read_int_parameter("Mode2", 5, 1, 300)
        self.max_samples = max(1, 300 // poll_interval)

        # Now set the interval at which the information is collected accordingly.
        Domoticz.Heartbeat(poll_interval)

        # Set the logging level
        log_level = self._read_int_parameter("Mode5", 0, 0, 3)
        SetLogLevel(LogLevels(log_level))

        self.inverter_address = Parameters["Address"]
        self.inverter_port = self._read_int_parameter("Port", 502, 1, 65535)
        self.inverter_unit = self._read_int_parameter("Mode3", 1, 1, 247)

        # Lets get in touch with the inverter.
        self.connectToInverter()

        DomoLog(LogLevels.EXTRA, "Leaving onStart()")


    #
    # OnHeartbeat is called by Domoticz at a specific interval as set in onStart()
    #

    def readDeviceValuesWithRetry(self, device_name, device_details, retried=False):
        try:
            if device_details["type"] == "inverter":
                source = self.inverter
            elif device_details["type"] == "meter":
                source = self.inverter.meters()[device_name]
            elif device_details["type"] == "battery":
                source = self.inverter.batteries()[device_name]
            else:
                return None

            return self.readDeviceValues(source, device_name)

        except ConnectionException as e:
            if retried:
                DomoLog(LogLevels.NORMAL, "Connection Exception when trying to communicate with: {}:{} Device Address: {} ({})".format(self.inverter_address, self.inverter_port, self.inverter_unit, e))
                return None

            DomoLog(LogLevels.EXTRA, "ConnectionException for {}; reconnecting once before retry: {}".format(device_name, e))

            # The inverter sometimes closes the TCP connection unexpectedly (e.g. idle timeout
            # on the device side). Reconnect once and retry the read immediately, so a single
            # dropped connection doesn't cost a full heartbeat cycle of missing data.

            self.disconnectInverter()
            self.retryafter = datetime.now()
            self.connectToInverter()

            if self.inverter and self.inverter.connected():
                DomoLog(LogLevels.EXTRA, "Retrying read for {} after reconnect.".format(device_name))
                return self.readDeviceValuesWithRetry(device_name, device_details, retried=True)

            return None

    def onHeartbeat(self):
        DomoLog(LogLevels.EXTRA, "Entered onHeartbeat()")

        if self.inverter and self.inverter.connected():

            for device_name, device_details in self.device_dictionary.items():

                if device_details["table"]:

                    values = self.readDeviceValuesWithRetry(device_name, device_details)

                    if values:
                        DomoLog(LogLevels.EXTRA, "Inverter returned information for {}".format(device_name))
                        to_log = dict(values)
                        if "c_serialnumber" in to_log:
                            to_log.pop("c_serialnumber")
                        DomoLog(LogLevels.VERBOSE, "device: {} values: {}".format(device_name, json.dumps(to_log, indent=4, sort_keys=False)))

                        self.processValues(device_details, values)
                    else:
                        self.disconnectInverter()
                        DomoLog(LogLevels.NORMAL, "Inverter returned no information for {}".format(device_name))

        else:
            self.connectToInverter()

        DomoLog(LogLevels.EXTRA, "Leaving onHeartbeat()")

    #
    # Go through the table and update matching devices
    # with the new values.
    #

    def processValues(self, device_details, inverter_data):

        DomoLog(LogLevels.EXTRA, "Entered processValues()")

        if device_details["table"]:
            table = device_details["table"]
            offset = device_details["offset"]

            # Just for cosmetics in the log

            updated = 0
            device_count = 0
            missing_keys = []

            # Now process each unit in the table.

            for unit in table:

                # Skip a unit when the matching device got deleted.

                if (unit[Column.ID] + offset) in Devices:
                    DomoLog(LogLevels.EXTRA, str(unit[Column.ID]) + "-> device available")

                    # Get the value for this unit from the Inverter data

                    try:
                        value = self.getUnitValue(unit, inverter_data)

                        # Time to store the value in Domoticz.
                        # Some devices require multiple values, in which case the plugin will combine those values.
                        # Currently, there is only a need to prepend one value with another.

                        if unit[Column.PREPEND_ROW]:
                            DomoLog(LogLevels.MAX, "-> has prepend lookup row")
                            prepend = self.getUnitValue(table[unit[Column.PREPEND_ROW]], inverter_data)
                            DomoLog(LogLevels.MAX, "prepend = {}".format(prepend))

                            if unit[Column.PREPEND_MATH]:
                                DomoLog(LogLevels.MAX, "-> has prepend math")
                                m = unit[Column.PREPEND_MATH]
                                prepend = m.get(prepend)
                                DomoLog(LogLevels.MAX, "prepend = {}".format(prepend))

                            sValue = unit[Column.FORMAT].format(prepend, value)

                        elif unit[Column.APPEND_MATH]:
                            DomoLog(LogLevels.MAX, "-> has append math")
                            m = unit[Column.APPEND_MATH]
                            append = m.get(0)
                            DomoLog(LogLevels.MAX, "append = {}".format(append))

                            sValue = unit[Column.FORMAT].format(value, append)

                        else:
                            DomoLog(LogLevels.MAX, "-> no prepend")
                            sValue = unit[Column.FORMAT].format(value)

                    except KeyError as e:
                        missing_keys.append(str(e))
                        continue

                    DomoLog(LogLevels.EXTRA, "sValue = {}".format(sValue))

                    # Only store the value in Domoticz when it has changed.
                    # TODO:
                    #   We should not store certain values when the inverter is sleeping.
                    #   That results in a strange graph; it would be better just to skip it then.

                    device = Devices[unit[Column.ID] + offset]
                    if unit[Column.MODBUSNAME] == "active_power_limit":
                        self._update_device_state(device, unit, value)
                        if unit[Column.MODBUSNAME] == "active_power_limit" and unit[Column.MATH]:
                            unit[Column.MATH].samples = []
                    elif sValue != device.sValue:
                        device.Update(nValue=0, sValue=str(sValue), TimedOut=0)
                        updated += 1

                    device_count += 1

                else:
                    DomoLog(LogLevels.MAX, str(unit[Column.ID]) + "-> skipping device not available")

            if missing_keys:
                DomoLog(
                    LogLevels.NORMAL,
                    "Inverter returned incomplete data; {} field(s) missing: {}. "
                    "This can happen when the inverter is sleeping (e.g. at night) or when there is a communication issue.".format(
                        len(missing_keys), ", ".join(missing_keys)
                    )
                )

            DomoLog(LogLevels.EXTRA, "Updated {} values out of {}".format(updated, device_count))

        DomoLog(LogLevels.EXTRA, "Leaving processValues()")

    #
    # Get the value of a particular unit from the inverter_data
    # and process it based on the information in the associated table.
    #

    def getUnitValue(self, row, inverter_data):

        DomoLog(LogLevels.EXTRA, "Entered getUnitValue()")

        # For certain units the table has a lookup table to replace the value with something else.
        if row[Column.LOOKUP]:
            DomoLog(LogLevels.MAX, "-> looking up...")

            lookup_table = row[Column.LOOKUP]
            to_lookup = int(inverter_data[row[Column.MODBUSNAME]])

            if to_lookup >= 0 and to_lookup < len(lookup_table):
                value = lookup_table[to_lookup]
            else:
                value = "Key not found in lookup table: {}".format(to_lookup)

        # When a math object is setup for the unit, update the samples in it and get the calculated value.
        elif row[Column.MATH] and self.do_math:
            DomoLog(LogLevels.MAX, "-> calculating...")
            m = row[Column.MATH]
            if row[Column.MODBUSSCALE]:
                m.update(inverter_data[row[Column.MODBUSNAME]], inverter_data[row[Column.MODBUSSCALE]])
            else:
                m.update(inverter_data[row[Column.MODBUSNAME]])

            value = m.get()

        # When there is no math object then just store the latest value.
        # Some date from the inverter need to be scaled before they can be stored.
        elif row[Column.MODBUSSCALE]:
            DomoLog(LogLevels.MAX, "-> scaling...")
            # we need to do some calculation here
            value = inverter_data[row[Column.MODBUSNAME]] * (10 ** inverter_data[row[Column.MODBUSSCALE]])

        # Some data require no action but storing in Domoticz.
        else:
            DomoLog(LogLevels.MAX, "-> copying...")
            value = inverter_data[row[Column.MODBUSNAME]]

        DomoLog(LogLevels.MAX, "value = {}".format(value))

        DomoLog(LogLevels.EXTRA, "Leaving getUnitValue()")

        return value


    #
    # Connect to the inverter and initialize the lookup tables.
    #

    def connectToInverter(self):

        DomoLog(LogLevels.EXTRA, "Entered connectToInverter()")

        # Setup the inverter object if it doesn't exist yet

        if (self.inverter == None):

            # Let's go
            DomoLog(LogLevels.MAX,
                "onStart Address: {} Port: {} Device Address: {}".format(
                    self.inverter_address,
                    self.inverter_port,
                    self.inverter_unit
                )
            )

            self.inverter = solaredge_modbus.Inverter(
                host = self.inverter_address,
                port = self.inverter_port,
                timeout = 15,
                unit = self.inverter_unit
            )

        # Do not stress the inverter when it did not respond in the previous attempt to contact it.

        if (self.inverter.connected() == False) and (self.retryafter <= datetime.now()):

            try:
                self.inverter.connect()

            except ConnectionException:

                # There are multiple reasons why this may fail.
                # - Perhaps the ip address or port are incorrect.
                # - The inverter may not be connected to the network,
                # - The inverter may be turned off.
                # - The inverter has a bad hairday....
                # Try again in the future.

                self.disconnectInverter()
                self.retryafter = datetime.now() + self.retrydelay

                DomoLog(LogLevels.NORMAL, "Connection Exception when trying to connect to: {}:{} Device Address: {}".format(self.inverter_address, self.inverter_port, self.inverter_unit))
                DomoLog(LogLevels.NORMAL, "Retrying to connect to inverter after: {}".format(self.retryafter))

            else:
                DomoLog(LogLevels.NORMAL, "Connection established with: {}:{} Device Address: {}".format(self.inverter_address, self.inverter_port, self.inverter_unit))

                # Let's get some values from the inverter and
                # figure out the type of the inverter and
                # meters and batteries if there are any

                try:
                    inverter_values = self.readDeviceValues(self.inverter, "Inverter")

                except ConnectionException as e:
                    self.disconnectInverter()
                    self.retryafter = datetime.now() + self.retrydelay

                    DomoLog(LogLevels.NORMAL, "Connection Exception when trying to communicate with: {}:{} Device Address: {} ({})".format(self.inverter_address, self.inverter_port, self.inverter_unit, e))
                    DomoLog(LogLevels.NORMAL, "Retrying to communicate with inverter after: {}".format(self.retryafter))

                else:
                    if inverter_values:
                        DomoLog(LogLevels.NORMAL, "Inverter returned information")

                        to_log = dict(inverter_values)
                        if "c_serialnumber" in to_log:
                            to_log.pop("c_serialnumber")
                        DomoLog(LogLevels.VERBOSE, "device: {} values: {}".format("Inverter", json.dumps(to_log, indent=4, sort_keys=False)))

                        known_sunspec_DIDS = set(item.value for item in solaredge_modbus.sunspecDID)

                        device_offset = 0
                        details = {
                            "type": "inverter",
                            "offset": device_offset,
                            "table": None
                        }

                        inverter_type = None
                        c_sunspec_did = inverter_values.get("c_sunspec_did")
                        if c_sunspec_did is None:
                            DomoLog(LogLevels.NORMAL, "Inverter returned incomplete data (missing 'c_sunspec_did'); will retry on next heartbeat.")
                            self.disconnectInverter()
                            self.retryafter = datetime.now() + self.retrydelay
                            return
                        if c_sunspec_did in known_sunspec_DIDS:
                            inverter_type = solaredge_modbus.sunspecDID(c_sunspec_did)
                            DomoLog(LogLevels.NORMAL, "Inverter type: {}".format(solaredge_modbus.C_SUNSPEC_DID_MAP[str(inverter_type.value)]))
                        else:
                            DomoLog(LogLevels.NORMAL, "Unknown inverter type: {}".format(c_sunspec_did))

                        if inverter_type == solaredge_modbus.sunspecDID.SINGLE_PHASE_INVERTER:
                            details.update({"table": inverters.SINGLE_PHASE_INVERTER})
                        elif inverter_type == solaredge_modbus.sunspecDID.THREE_PHASE_INVERTER:
                            details.update({"table": inverters.THREE_PHASE_INVERTER})
                        else:
                            details.update({"table": inverters.OTHER_INVERTER})

                        self.device_dictionary["Inverter"] = details
                        self.addUpdateDevices("Inverter")

                        # Scan for meters if required
                        if self.scan_for_meters:
                            DomoLog(LogLevels.NORMAL, "Scanning for meters")

                            device_offset = max(inverters.InverterUnit)
                            try:
                                all_meters = self.inverter.meters()
                            except ConnectionException as e:
                                DomoLog(LogLevels.NORMAL, "Connection Exception while scanning for meters: {}".format(e))
                                self.disconnectInverter()
                                self.retryafter = datetime.now() + self.retrydelay
                                DomoLog(LogLevels.NORMAL, "Retrying to communicate with inverter after: {}".format(self.retryafter))
                                return
                            if all_meters:
                                DomoLog(LogLevels.NORMAL, "Found at least one meter")

                                for meter, params in all_meters.items():
                                    meter_values = self.readDeviceValues(params, meter)

                                    if meter_values:
                                        DomoLog(LogLevels.NORMAL, "Inverter returned meter information")

                                        to_log = dict(meter_values)
                                        if "c_serialnumber" in to_log:
                                            to_log.pop("c_serialnumber")
                                        DomoLog(LogLevels.VERBOSE, "device: {} values: {}".format(meter, json.dumps(to_log, indent=4, sort_keys=False)))

                                        details = {
                                            "type": "meter",
                                            "offset": device_offset,
                                            "table": None
                                        }
                                        device_offset = device_offset + max(meters.MeterUnit)

                                        meter_type = None
                                        c_sunspec_did = meter_values.get("c_sunspec_did")
                                        if c_sunspec_did is None:
                                            DomoLog(LogLevels.NORMAL, "Meter '{}' returned incomplete data (missing 'c_sunspec_did'); skipping for now.".format(meter))
                                            continue
                                        if c_sunspec_did in known_sunspec_DIDS:
                                            meter_type = solaredge_modbus.sunspecDID(c_sunspec_did)
                                            DomoLog(LogLevels.NORMAL, "Meter type: {}".format(solaredge_modbus.C_SUNSPEC_DID_MAP[str(meter_type.value)]))
                                        else:
                                            DomoLog(LogLevels.NORMAL, "Unknown meter type: {}".format(c_sunspec_did))

                                        if meter_type == solaredge_modbus.sunspecDID.SINGLE_PHASE_METER:
                                            details.update({"table": meters.SINGLE_PHASE_METER})
                                        elif meter_type == solaredge_modbus.sunspecDID.WYE_THREE_PHASE_METER:
                                            details.update({"table": meters.WYE_THREE_PHASE_METER})
                                        else:
                                            details.update({"table": meters.OTHER_METER})

                                        self.device_dictionary[meter] = details
                                        self.addUpdateDevices(meter)
                                    else:
                                        DomoLog(LogLevels.NORMAL, "Found {}. BUT... inverter didn't return information".format(meter))
                            else:
                                DomoLog(LogLevels.NORMAL, "No meters found")
                        else:
                            DomoLog(LogLevels.NORMAL, "Skip scanning for meters")
                        # End scan for meters

                        # Scan for batteries if required
                        if self.scan_for_batteries:
                            DomoLog(LogLevels.NORMAL, "Scanning for batteries")

                            device_offset = max(inverters.InverterUnit) + (3 * max(meters.MeterUnit))
                            try:
                                all_batteries = self.inverter.batteries()
                            except ConnectionException as e:
                                DomoLog(LogLevels.NORMAL, "Connection Exception while scanning for batteries: {}".format(e))
                                self.disconnectInverter()
                                self.retryafter = datetime.now() + self.retrydelay
                                DomoLog(LogLevels.NORMAL, "Retrying to communicate with inverter after: {}".format(self.retryafter))
                                return
                            if all_batteries:
                                DomoLog(LogLevels.NORMAL, "Found at least one battery")

                                for battery, params in all_batteries.items():
                                    battery_values = self.readDeviceValues(params, battery)

                                    if battery_values:
                                        DomoLog(LogLevels.NORMAL, "Inverter returned battery information")

                                        to_log = dict(battery_values)
                                        if "c_serialnumber" in to_log:
                                            to_log.pop("c_serialnumber")
                                        DomoLog(LogLevels.VERBOSE, "device: {} values: {}".format(battery, json.dumps(to_log, indent=4, sort_keys=False)))

                                        details = {
                                            "type": "battery",
                                            "offset": device_offset,
                                            "table": None
                                        }
                                        device_offset = device_offset + max(batteries.BatteryUnit)

                                        battery_type = None
                                        c_sunspec_did = battery_values.get("c_sunspec_did")
                                        if c_sunspec_did is None:
                                            DomoLog(LogLevels.NORMAL, "Battery '{}' returned incomplete data (missing 'c_sunspec_did'); skipping for now.".format(battery))
                                            continue
                                        if c_sunspec_did in known_sunspec_DIDS:
                                            battery_type = solaredge_modbus.sunspecDID(c_sunspec_did)
                                            DomoLog(LogLevels.NORMAL, "Battery type: {}".format(solaredge_modbus.C_SUNSPEC_DID_MAP[str(battery_type.value)]))
                                        else:
                                            DomoLog(LogLevels.NORMAL, "Unknown battery type: {}".format(c_sunspec_did))

                                        details.update({"table": batteries.OTHER_BATTERY})

                                        self.device_dictionary[battery] = details
                                        self.addUpdateDevices(battery)
                                    else:
                                        DomoLog(LogLevels.NORMAL, "Found {}. BUT... inverter didn't return information".format(battery))
                            else:
                                DomoLog(LogLevels.NORMAL, "No batteries found")
                        else:
                            DomoLog(LogLevels.NORMAL, "Skip scanning for batteries")
                        # End scan for batteries

                    else:
                        self.disconnectInverter()
                        self.retryafter = datetime.now() + self.retrydelay

                        DomoLog(LogLevels.NORMAL, "Connection established with: {}:{} Device Address: {}. BUT... inverter returned no information".format(self.inverter_address, self.inverter_port, self.inverter_unit))
                        DomoLog(LogLevels.NORMAL, "Retrying to communicate with inverter after: {}".format(self.retryafter))
        else:
            DomoLog(LogLevels.NORMAL, "Retrying to communicate with inverter after: {}".format(self.retryafter))

        DomoLog(LogLevels.EXTRA, "Leaving connectToInverter()")

    def readDeviceValues(self, device, device_name):
        try:
            values = device.read_all()
        except ConnectionException as first_error:
            DomoLog(LogLevels.EXTRA, "ConnectionException while reading {}; reconnecting once before retry: {}".format(device_name, first_error))
            self.disconnectInverter()
            try:
                values = device.read_all()
            except ConnectionException:
                raise
            else:
                DomoLog(LogLevels.EXTRA, "Recovered Modbus TCP connection after reconnect.")
                return values
        else:
            if values:
                return values

            DomoLog(LogLevels.EXTRA, "read_all returned no values for {}; reconnecting once before retry.".format(device_name))
            self.disconnectInverter()
            return device.read_all()

    #
    # onStop is called by Domoticz when the plugin is stopped.
    #

    def onStop(self):
        DomoLog(LogLevels.EXTRA, "Entered onStop()")
        self.disconnectInverter()

    def onCommand(self, Unit, Command, Level, Hue):
        DomoLog(LogLevels.EXTRA, "Entered onCommand({}, {}, {}, {})".format(Unit, Command, Level, Hue))

        device_name, device_details, unit = self._find_device_unit(Unit)
        if not unit or device_details["type"] != "inverter" or unit[Column.MODBUSNAME] != "active_power_limit":
            DomoLog(LogLevels.NORMAL, "Ignoring unsupported command for unit {}".format(Unit))
            return

        current_level = 0
        if Unit in Devices:
            try:
                current_level = int(float(Devices[Unit].sValue))
            except (TypeError, ValueError):
                current_level = 0

        try:
            target_level = self._get_power_limit_target(Command, Level, current_level)
        except ValueError as e:
            DomoLog(LogLevels.NORMAL, str(e))
            return

        if not self.inverter or not self.inverter.connected():
            self.connectToInverter()

        if not self.inverter or not self.inverter.connected():
            DomoLog(LogLevels.NORMAL, "Cannot change Active Power Limit because the inverter is not connected.")
            return

        try:
            self._write_inverter_value(unit[Column.MODBUSNAME], target_level)
            self._write_inverter_value("commit_power_control_settings", 1)
        except (ConnectionException, RuntimeError, ValueError) as e:
            self.disconnectInverter()
            DomoLog(LogLevels.NORMAL, "Failed to change Active Power Limit to {}: {}".format(target_level, e))
            return

        if Unit in Devices:
            self._update_device_state(Devices[Unit], unit, target_level)

        if device_name in self.device_dictionary:
            values = self.readDeviceValuesWithRetry(device_name, device_details)
            if values:
                self.processValues(device_details, values)

        DomoLog(LogLevels.NORMAL, "Active Power Limit set to {}%".format(target_level))
        DomoLog(LogLevels.EXTRA, "Leaving onCommand()")

    def disconnectInverter(self):
        try:
            if self.inverter:
                self.inverter.disconnect()
        except Exception as e:
            DomoLog(LogLevels.EXTRA, "disconnectInverter: {}".format(e))

    #
    # Go through the table and update matching devices
    # with the new values.
    #

    def addUpdateDevices(self, device_name):

        DomoLog(LogLevels.EXTRA, "Entered addUpdateDevices()")

        if self.device_dictionary[device_name] and self.device_dictionary[device_name]["table"]:

            table = self.device_dictionary[device_name]["table"]
            offset = self.device_dictionary[device_name]["offset"]
            prepend_name = device_name + " - "

            # Set the number of samples on all the math objects.

            for unit in table:
                if unit[Column.MATH]  and self.do_math:
                    unit[Column.MATH].set_max_samples(self.max_samples)

            # We updated some device types over time.
            # Let's make sure that we have the correct type setup.

            updated_ids = set()

            for unit in table:
                if (unit[Column.ID] + offset) in Devices:
                    device = Devices[unit[Column.ID] + offset]
                    if (device.Type != unit[Column.TYPE] or
                        device.SubType != unit[Column.SUBTYPE] or
                        device.SwitchType != unit[Column.SWITCHTYPE] or
                        device.Options != unit[Column.OPTIONS]):

                        DomoLog(LogLevels.NORMAL, "Updating device \"{}\"".format(device.Name))

                        nValue = device.nValue
                        sValue = device.sValue

                        device.Update(
                                Type=unit[Column.TYPE],
                                Subtype=unit[Column.SUBTYPE],
                                Switchtype=unit[Column.SWITCHTYPE],
                                Options=unit[Column.OPTIONS],
                                nValue=nValue,
                                sValue=sValue
                        )
                        updated_ids.add(unit[Column.ID] + offset)

            # Add missing devices if needed.

            if self.add_devices:
                for unit in table:
                    if (unit[Column.ID] + offset) not in Devices and (unit[Column.ID] + offset) not in updated_ids:

                        DomoLog(LogLevels.NORMAL, "Adding device \"{}\"".format(prepend_name + unit[Column.NAME]))

                        Domoticz.Device(
                            Unit=unit[Column.ID] + offset,
                            Name=prepend_name + unit[Column.NAME],
                            Type=unit[Column.TYPE],
                            Subtype=unit[Column.SUBTYPE],
                            Switchtype=unit[Column.SWITCHTYPE],
                            Options=unit[Column.OPTIONS],
                            Used=1,
                            Image=self.imageID,
                        ).Create()

        DomoLog(LogLevels.EXTRA, "Leaving addUpdateDevices()")

#
# Instantiate the plugin and register the supported callbacks.
#

global _plugin
_plugin = BasePlugin()

def onStart():
    global _plugin
    _plugin.onStart()

def onStop():
    global _plugin
    _plugin.onStop()

def onHeartbeat():
    global _plugin
    _plugin.onHeartbeat()

def onCommand(Unit, Command, Level, Hue):
    global _plugin
    _plugin.onCommand(Unit, Command, Level, Hue)

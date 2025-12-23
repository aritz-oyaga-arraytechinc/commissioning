import datetime
import inspect
import struct
import math
from pymodbus.client import ModbusTcpClient
from abc import abstractmethod, ABC
from typing import override
from math import trunc
from functools import cached_property

MAX_TSC_PACKAGES = 24
MAX_IWC_PACKAGES = 10
MAX_TSC_REGISTERS = 39
MAX_IWC_REGISTERS = 30
reg_extended_control = 40007

class IConfiguration(ABC):
    @cached_property
    @abstractmethod
    def modbus_package(self) -> list[int]:
        """Define the telemetry configuration data to write in the device

        :return: A list of integers representing the data that must be written on the device telemetry configuration.
        """
        raise NotImplementedError

class IwcConfiguration(IConfiguration):
    def __init__(self, conf: list[tuple[int, int]]) -> None:
        self.conf_definition = conf
        if len(self.conf_definition) > MAX_IWC_PACKAGES:
            raise ValueError(f"Too many IWC packages: {len(self.conf_definition)}")

    @cached_property
    @override
    def modbus_package(self) -> list[int]:
        iwc_configuration_registers = [0] * int(MAX_IWC_PACKAGES * 1.5)
        registers_amount = 0
        for index, package in enumerate(self.conf_definition):
            iwc_configuration_registers[index] += package[0]
            iwc_configuration_registers[trunc(index / 4) + MAX_IWC_PACKAGES] += package[1] * pow(16, index % 4)
            registers_amount += package[1]
            if registers_amount > MAX_IWC_REGISTERS:
                    raise ValueError(f"Too many IWC registers: {registers_amount}")
        return iwc_configuration_registers

class TscConfiguration(IConfiguration):
    def __init__(self, conf: list[tuple[int, int]]) -> None:
        self.conf_definition = conf
        if len(self.conf_definition) > MAX_TSC_PACKAGES:
            raise ValueError(f"Too many TSC packages: {len(self.conf_definition)}")

    @cached_property
    @override
    def modbus_package(self) -> list[int]:
        tsc_configuration_registers = [0] * int(MAX_TSC_PACKAGES * 1.5)
        registers_amount = 0
        for index, package in enumerate(self.conf_definition):
            tsc_configuration_registers[index] = package[0]
            if index % 2 == 0:
                tsc_configuration_registers[int(index / 2) + MAX_TSC_PACKAGES] += package[1]
            else:
                tsc_configuration_registers[trunc(index / 2) + MAX_TSC_PACKAGES] += package[1] * 256
            registers_amount += package[1]
            if registers_amount > MAX_TSC_REGISTERS:
                raise ValueError(f"Too many TSC registers: {registers_amount}")
        return tsc_configuration_registers

class TscCoordinates(IConfiguration):
    def __init__(self, longitude: float, latitude: float) -> None:
        self.longitude = longitude
        self.latitude = latitude
    @cached_property
    @override
    def modbus_package(self) -> list[int]:
        coordinates = []
        long1, long2 = struct.unpack("<HH", struct.pack("<f", self.longitude))
        lat1, lat2 = struct.unpack("<HH", struct.pack("<f", self.latitude))
        coordinates.extend([long1, long2, lat1, lat2])
        return coordinates

class TscSunTracking(IConfiguration):
    def __init__(self, pitch: float, panel_width: float) -> None:
        self.pitch = pitch
        self.panel_width = panel_width
    @cached_property
    @override
    def modbus_package(self) -> list[int]:
        parameters = []
        pitch1, pitch2 = struct.unpack("<HH", struct.pack("<f", self.pitch))
        panel_width1, panel_width2 = struct.unpack("<HH", struct.pack("<f", self.panel_width))
        parameters.extend([pitch1, pitch2, panel_width1, panel_width2])
        return parameters

class TscPanID(IConfiguration):
    def __init__(self, pan_id: str) -> None:
        self.pan_id = pan_id
    @cached_property
    @override
    def modbus_package(self) -> list[int]:
        parameters = []
        pan_regs = struct.unpack(">4H", bytes.fromhex(self.pan_id))
        parameters.extend(pan_regs[::-1])
        return parameters

class TscSwMovementLimit(IConfiguration):
    def __init__(self, west_imit: float, east_limit: float) -> None:
        self.west_limit = west_imit
        self.east_limit = east_limit
    @cached_property
    @override
    def modbus_package(self) -> list[int]:
        """55 degrees to pulses"""
        parameters = [struct.unpack("<H", struct.pack("<h", int(self.west_limit * 34.70909090909091)))[0],
                      struct.unpack("<H", struct.pack("<h", int(self.east_limit * 34.70909090909091)))[0]]
        return parameters

class TscSmartLimits(IConfiguration):
    def __init__(self, smart_limits: dict[str, int]) -> None:
        self.smart_limits = smart_limits
    @cached_property
    @override
    def modbus_package(self) -> list[int]:
        """Degrees to radians little endian registers from West limit 1 to East limit 7"""
        parameters = []
        for key, angle in self.smart_limits.items():
            reg1, reg2 = struct.unpack("<HH", struct.pack("<f", angle * math.pi / 180.0))
            parameters.extend([reg1, reg2])
        return parameters

class TscSafePositions(IConfiguration):
    def __init__(self, safe_positions: dict[str, int]) -> None:
        self.safe_positions = safe_positions
    @cached_property
    @override
    def modbus_package(self) -> list[int]:
        """Degrees to radians little endian registers from Safe position 1 to 7"""
        parameters = []
        for key, angle in self.safe_positions.items():
            reg1, reg2 = struct.unpack("<HH", struct.pack("<f", angle * math.pi / 180.0))
            parameters.extend([reg1, reg2])
        return parameters

class TscBacktracking(IConfiguration):
    def __init__(self, backtracking3d1: int, backtracking3d2: int) -> None:
        self.backtracking3d1 = backtracking3d1
        self.backtracking3d2 = backtracking3d2
    @override
    def modbus_package(self) -> list[int]:
        parameters = []
        parameters.append(struct.unpack("<H", struct.pack("<h ", self.backtracking3d1))[0])
        parameters.append(struct.unpack("<H", struct.pack("<h ", self.backtracking3d2))[0])
        return parameters

class IwcWindSpeedThresholds(IConfiguration):
    def __init__(self, wind_speed_threshold_deactivation: float, wind_speed_threshold_activation: float) -> None:
        """Data entered in kilometers per hour"""
        self.wind_speed_threshold_deactivation = wind_speed_threshold_deactivation
        self.wind_speed_threshold_activation = wind_speed_threshold_activation
    @cached_property
    @override
    def modbus_package(self) -> list[int]:
        parameters = []
        deact1, deact2 = struct.unpack("<HH", struct.pack("<f", self.wind_speed_threshold_deactivation / 3.6))
        act1, act2 = struct.unpack("<HH", struct.pack("<f", self.wind_speed_threshold_activation / 3.6))
        parameters.extend([deact1, deact2, act1, act2])
        return parameters

class IwcWindAlarmTime(IConfiguration):
    def __init__(self, wind_alarm_time_deactivation: int, wind_alarm_time_activation: int) -> None:
        """Data entered in seconds"""
        self.wind_alarm_time_deactivation = wind_alarm_time_deactivation
        self.wind_alarm_time_activation = wind_alarm_time_activation
    @cached_property
    @override
    def modbus_package(self) -> list[int]:
        parameters = []
        parameters.append(struct.unpack("<H", struct.pack("<h", self.wind_alarm_time_deactivation))[0])
        parameters.append(struct.unpack("<H", struct.pack("<h", self.wind_alarm_time_activation))[0])
        return parameters

class IwcAvgTime(IConfiguration):
    def __init__(self, average_time: int) -> None:
        """Data entered in minutes"""
        self.average_time = average_time
    @cached_property
    @override
    def modbus_package(self) -> list[int]:
        parameters = []
        parameters.append(struct.unpack("<H", struct.pack("<h", self.average_time))[0])
        return parameters

class IwcRelaxTime(IConfiguration):
    def __init__(self, relax_time: int) -> None:
        """Data entered in minutes"""
        self.relax_time = relax_time
    @cached_property
    @override
    def modbus_package(self) -> list[int]:
        parameters = []
        parameters.append(struct.unpack("<H", struct.pack("<h", self.relax_time))[0])
        return parameters

class IwcSnowThreshold(IConfiguration):
    def __init__(self, snow_threshold: float) -> None:
        """Data entered in meters"""
        self.snow_threshold = snow_threshold
    @cached_property
    @override
    def modbus_package(self) -> list[int]:
        parameters = []
        parameters.extend(struct.unpack("<HH", struct.pack("<f", self.snow_threshold)))
        return parameters

class IwcSnowActivationTime(IConfiguration):
    def __init__(self, activation_time: int) -> None:
        """Data entered in minutes"""
        self.activation_time = activation_time
    @cached_property
    @override
    def modbus_package(self) -> int:
        return struct.unpack("<H", struct.pack("<h", self.activation_time))[0]

class IwcSnowDeactivationTime(IConfiguration):
    def __init__(self,deactivation_time: int) -> None:
        """Data entered in minutes"""
        self.deactivation_time = deactivation_time
    @cached_property
    @override
    def modbus_package(self) -> int:
        return struct.unpack("<H", struct.pack("<h", self.deactivation_time))[0]

class IwcSnowSampleTime(IConfiguration):
    def __init__(self, sample_period: int) -> None:
        """Data entered in minutes"""
        self.sample_period = sample_period
    @cached_property
    @override
    def modbus_package(self) -> int:
        return struct.unpack("<H", struct.pack("<h", self.sample_period))[0]

class IwcSnowMaxVariation(IConfiguration):
    def __init__(self, max_variation: int) -> None:
        """Data entered in millimeters"""
        self.max_variation = max_variation
    @cached_property
    @override
    def modbus_package(self) -> int:
        return struct.unpack("<H", struct.pack("<h", self.max_variation))[0]

class IwcPanID(IConfiguration):
    def __init__(self, pan_id: str) -> None:
        self.pan_id = pan_id
    @cached_property
    @override
    def modbus_package(self) -> list[int]:
        parameters = []
        pan_regs = struct.unpack(">4H", bytes.fromhex(self.pan_id))
        parameters.extend(pan_regs[::-1])
        return parameters

def iwc_set_sensors(client: ModbusTcpClient, dev_id: int, snow: bool, rika: bool, pira: bool, flood: bool, temperature: bool) -> bool:
    """Define the active sensors

            :param client: ModbusTcpClient
            :param dev_id: modbus device id
            :param snow: has snow sensor or not
            :param rika: has rika sensor or not
            :param pira: has pira sensor or not
            :param flood: has flood sensor or not
            :param temperature: has temperature sensor or not
            return True if the registers were configured, False otherwise.
        """
    try:
        reading = client.read_holding_registers(address=41008, count=1, device_id=dev_id)
        value = reading.registers[0] & 0xFC1F
        value = value + (snow << 5)
        value = value + (rika << 6)
        value = value + (pira << 7)
        value = value + (flood << 8)
        value = value + (temperature << 9)
        writing = client.write_register(address=41008, value=value, device_id=dev_id)
        if not writing.isError():
            print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} to {value} successfully.")
            return True
        else:
            print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} cannot be configured.")
        return False
    except Exception as e:
        print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} cannot be prepared -> {e}")
        return False

def iwc_set_modbus_id(client: ModbusTcpClient, dev_id: int, new_modbus_id: int) -> bool:
    """Define the new modbus identifier

            :param client: ModbusTcpClient
            :param dev_id: modbus device id
            :param new_modbus_id: device new modbus identifier
            return True if the registers were configured, False otherwise.
        """
    try:
        reading = client.read_holding_registers(address=41002, count=1, device_id=dev_id)
        value = reading.registers[0] & 0xFF00
        value = value + new_modbus_id
        writing = client.write_register(address=41002, value=value, device_id=dev_id)
        if not writing.isError():
            print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} to {new_modbus_id} successfully.")
            return True
        else:
            print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} cannot be configured.")
        return False
    except Exception as e:
        print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} cannot be prepared -> {e}")
        return False

def iwc_set_pan_id(client: ModbusTcpClient, dev_id: int, parameters: IwcPanID) -> bool:
    """Define the xbee PAN identifier

        :param client: ModbusTcpClient
        :param dev_id: modbus device id
        :param parameters: array with device xbee PAN identifier in hex
        return True if the registers were configured, False otherwise.
        """
    try:
        writing = client.write_registers(address=41062, values=parameters.modbus_package, device_id=dev_id)
        if not writing.isError():
            print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} successfully.")
            return True
        else:
            print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} cannot be configured.")
        return False
    except Exception as e:
        print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} cannot be prepared -> {e}")
        return False

def iwc_snow_alarm_activation_time(client: ModbusTcpClient, dev_id: int, parameters: IwcSnowActivationTime) -> bool:
    """Define the snow configuration

        :param client: ModbusTcpClient
        :param dev_id: modbus device id
        :param parameters: Snow alarm activation
        return True if the registers were configured, False otherwise.
    """
    try:
        writing = client.write_register(address=41217, value=parameters.modbus_package, device_id=dev_id)
        if not writing.isError():
            print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} successfully.")
            return True
        else:
            print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} cannot be configured.")
        return False
    except Exception as e:
        print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} cannot be prepared -> {e}")
        return False

def iwc_snow_deactivation_time(client: ModbusTcpClient, dev_id: int, parameters: IwcSnowDeactivationTime) -> bool:
    """Define the snow configuration

        :param client: ModbusTcpClient
        :param dev_id: modbus device id
        :param parameters: Snow alarm deactivation time
        return True if the registers were configured, False otherwise.
    """
    try:
        writing = client.write_register(address=41218, value=parameters.modbus_package, device_id=dev_id)
        if not writing.isError():
            print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} successfully.")
            return True
        else:
            print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} cannot be configured.")
        return False
    except Exception as e:
        print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} cannot be prepared -> {e}")
        return False

def iwc_snow_sample_period(client: ModbusTcpClient, dev_id: int, parameters: IwcSnowSampleTime) -> bool:
    """Define the snow configuration

        :param client: ModbusTcpClient
        :param dev_id: modbus device id
        :param parameters: sample_period
        return True if the registers were configured, False otherwise.
    """
    try:
        writing = client.write_register(address=41219, value=parameters.modbus_package, device_id=dev_id)
        if not writing.isError():
            print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} successfully.")
            return True
        else:
            print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} cannot be configured.")
        return False
    except Exception as e:
        print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} cannot be prepared -> {e}")
        return False

def iwc_snow_maximum_variation(client: ModbusTcpClient, dev_id: int, parameters: IwcSnowMaxVariation) -> bool:
    """Define the snow configuration

        :param client: ModbusTcpClient
        :param dev_id: modbus device id
        :param parameters: max_variation
        return True if the registers were configured, False otherwise.
    """
    try:
        writing = client.write_register(address=41220, value=parameters.modbus_package, device_id=dev_id)
        if not writing.isError():
            print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} successfully.")
            return True
        else:
            print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} cannot be configured.")
        return False
    except Exception as e:
        print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} cannot be prepared -> {e}")
        return False

def iwc_snow_threshold(client: ModbusTcpClient, dev_id: int, parameters: IwcSnowThreshold) -> bool:
    """Define the meters detected to trigger a snow alarm.

        :param client: ModbusTcpClient
        :param dev_id: modbus device id
        :param parameters: Snow height measured in meters to trigger a snow alarm.
        return True if the registers were configured, False otherwise.
    """
    try:
        writing = client.write_registers(address=41054, values=parameters.modbus_package, device_id=dev_id)
        if not writing.isError():
            print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} successfully.")
            return True
        else:
            print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} cannot be configured.")
        return False
    except Exception as e:
        print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} cannot be prepared -> {e}")
        return False

def iwc_relax_time(client: ModbusTcpClient, dev_id: int, parameters: IwcRelaxTime) -> bool:
    """Define the time in minutes to hold the alarm after an activation

        :param client: ModbusTcpClient
        :param dev_id: modbus device id
        :param parameters: Time to deactivate and activate the wind speed alarm
        return True if the registers were configured, False otherwise.
    """
    try:
        writing = client.write_registers(address=41214, values=parameters.modbus_package, device_id=dev_id)
        if not writing.isError():
            print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} successfully.")
            return True
        else:
            print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} cannot be configured.")
        return False
    except Exception as e:
        print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} cannot be prepared -> {e}")
        return False

def iwc_wind_alarms_2_and_3(client: ModbusTcpClient,
                            dev_id: int,
                            speed2: IwcWindSpeedThresholds,
                            speed3: IwcWindSpeedThresholds,
                            time2: IwcWindAlarmTime,
                            time3: IwcWindAlarmTime,
                            avg3_time: IwcAvgTime) -> bool:
    """Define the iwc wind alarm activation and deactivation.

        :param client: ModbusTcpClient
        :param dev_id: modbus device id
        :param speed2: Wind speed thresholds to deactivate and activate the wind speed alarm 2
        :param speed3: Wind speed thresholds to deactivate and activate the wind speed alarm 3
        :param time2: Time to deactivate and activate the wind speed alarm 2
        :param time3: Time to deactivate and activate the wind speed alarm 3
        :param avg3_time: Average time to deactivate and activate the wind speed alarm 3, maximum 10
        return True if the registers were configured, False otherwise.
    """
    try:
        values = []
        values.extend(speed2.modbus_package)
        values.extend(speed3.modbus_package)
        values.extend(time2.modbus_package)
        values.extend(time3.modbus_package)
        values.extend(avg3_time.modbus_package)
        writing = client.write_registers(address=41200, values=values, device_id=dev_id)
        if not writing.isError():
            print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} successfully.")
            return True
        else:
            print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} cannot be configured.")
        return False
    except Exception as e:
        print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} cannot be prepared -> {e}")
        return False

def iwc_wind_alarm_time(client: ModbusTcpClient, dev_id: int, parameters: IwcWindAlarmTime) -> bool:
    """Define the iwc wind alarm time activation and deactivation.

        :param client: ModbusTcpClient
        :param dev_id: modbus device id
        :param parameters: Time to deactivate and activate the wind speed alarm
        return True if the registers were configured, False otherwise.
    """
    try:
        writing = client.write_registers(address=41017, values=parameters.modbus_package, device_id=dev_id)
        if not writing.isError():
            print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} successfully.")
            return True
        else:
            print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} cannot be configured.")
        return False
    except Exception as e:
        print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} cannot be prepared -> {e}")
        return False

def iwc_wind_speed_thresholds(client: ModbusTcpClient, dev_id: int, parameters: IwcWindSpeedThresholds) -> bool:
    """Define the iwc wind speed thresholds

        :param client: ModbusTcpClient
        :param dev_id: modbus device id
        :param parameters: Wind speed thresholds to deactivate and activate the wind speed alarm
        return True if the registers were configured, False otherwise.
    """
    try:
        writing = client.write_registers(address=41011, values=parameters.modbus_package, device_id=dev_id)
        if not writing.isError():
            print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} successfully.")
            return True
        else:
            print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} cannot be configured.")
        return False
    except Exception as e:
        print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} cannot be prepared -> {e}")
        return False

def tsc_set_commissioning_state(client: ModbusTcpClient, dev_id: int, commissioning_state: int) -> bool:
    """Define the tracker commissioning state

        :param client: ModbusTcpClient
        :param dev_id: modbus device id
        :param commissioning_state: 3=Factory, 2=Mechanical, 1=Commissioning, 0=Commissioned
        return True if the registers were configured, False otherwise.
    """
    try:
        or_mask = commissioning_state << 5
        writing = client.mask_write_register(address=40000, and_mask=0xFF1F, or_mask=or_mask, device_id=dev_id)
        if not writing.isError():
            print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} successfully.")
            return True
        else:
            print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} cannot be configured.")
        return False
    except Exception as e:
        print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} cannot be prepared -> {e}")
        return False

def tsc_backtracking3d(client: ModbusTcpClient, dev_id: int, parameters: TscBacktracking) -> bool:
    """Define the backtracking 3D values

        :param client: ModbusTcpClient
        :param dev_id: modbus device id
        :param parameters: array with backtracking 3D values, S16
        return True if the registers were configured, False otherwise.
    """
    try:
        writing = client.write_registers(address=40040, values=parameters.modbus_package(), device_id=dev_id)
        if not writing.isError():
            print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} successfully.")
            return True
        else:
            print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} cannot be configured.")
        return False
    except Exception as e:
        print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} cannot be prepared -> {e}")
        return False

def tsc_safe_positions(client:ModbusTcpClient, dev_id: int, parameters: TscSafePositions) -> bool:
    """Define the safe position angles

        :param client: ModbusTcpClient
        :param dev_id: modbus device id
        :param parameters: array with device safe position angles, float little endian
        return True if the registers were configured, False otherwise.
    """
    try:
        writing = client.write_registers(address=41044, values=parameters.modbus_package, device_id=dev_id)
        if not writing.isError():
            print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} successfully.")
            return True
        else:
            print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} cannot be configured.")
        return False
    except Exception as e:
        print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} cannot be prepared -> {e}")
        return False

def tsc_smart_limits(client:ModbusTcpClient, dev_id: int, parameters: TscSmartLimits) -> bool:
    """Define the smart movement limits

        :param client: ModbusTcpClient
        :param dev_id: modbus device id
        :param parameters: array with device smart limits angles, float little endian
        return True if the registers were configured, False otherwise.
    """
    try:
        writing = client.write_registers(address=41111, values=parameters.modbus_package, device_id=dev_id)
        if not writing.isError():
            print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} successfully.")
            return True
        else:
            print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} cannot be configured.")
        return False
    except Exception as e:
        print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} cannot be prepared -> {e}")
        return False

def tsc_software_movement_limits(client:ModbusTcpClient, dev_id: int, parameters: TscSwMovementLimit) -> bool:
    """Define the software movement limits

        :param client: ModbusTcpClient
        :param dev_id: modbus device id
        :param parameters: array with device software limits written in pulses, S16
        return True if the registers were configured, False otherwise.
    """
    try:
        writing = client.write_registers(address=41037, values=parameters.modbus_package, device_id=dev_id)
        if not writing.isError():
            print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} successfully.")
            return True
        else:
            print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} cannot be configured.")
        return False
    except Exception as e:
        print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} cannot be prepared -> {e}")
        return False

def tsc_set_modbus_id(client: ModbusTcpClient, dev_id: int, new_modbus_id: int) -> bool:
    """Define the new modbus identifier

            :param client: ModbusTcpClient
            :param dev_id: modbus device id
            :param new_modbus_id: device new modbus identifier
            return True if the registers were configured, False otherwise.
        """
    try:
        or_mask = new_modbus_id
        writing = client.mask_write_register(address=41004, and_mask=0xFF00, or_mask=or_mask, device_id=dev_id)
        if not writing.isError():
            print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} to {new_modbus_id} successfully.")
            return True
        else:
            print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} cannot be configured.")
        return False
    except Exception as e:
        print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} cannot be prepared -> {e}")
        return False

def tsc_reset_communication(client: ModbusTcpClient, dev_id: int) -> bool:
    """Reset communications to set the values

        :param client: ModbusTcpClient
        :param dev_id: modbus device id
        return True if the registers were configured, False otherwise.
    """
    try:
        writing = client.mask_write_register(address=41004, and_mask=0xFEFF, or_mask=0x0100, device_id=dev_id)
        if not writing.isError():
            print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} successfully.")
            return True
        else:
            print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} cannot be configured.")
        return False
    except Exception as e:
        print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} cannot be prepared -> {e}")
        return False

def tsc_set_pan_id(client: ModbusTcpClient, dev_id: int, parameters: TscPanID) -> bool:
    """Define the xbee PAN identifier

        :param client: ModbusTcpClient
        :param dev_id: modbus device id
        :param parameters: array with device xbee PAN identifier in hex
        return True if the registers were configured, False otherwise.
        """
    try:
        writing = client.write_registers(address=41070, values=parameters.modbus_package, device_id=dev_id)
        if not writing.isError():
            print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} successfully.")
            return True
        else:
            print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} cannot be configured.")
        return False
    except Exception as e:
        print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} cannot be prepared -> {e}")
        return False

def tsc_set_solar_tracking(client: ModbusTcpClient, dev_id: int, parameters: TscSunTracking) -> bool:
    """Define the solar tracking configuration.

        :param client: ModbusTcpClient
        :param dev_id: modbus device id
        :param parameters: array with device solar tracking configuration, pitch and panel width
        return True if the registers were configured, False otherwise.
        """
    try:
        writing = client.write_registers(address=41033, values=parameters.modbus_package, device_id=dev_id)
        if not writing.isError():
            print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} successfully.")
            return True
        else:
            print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} cannot be configured.")
        return False
    except Exception as e:
        print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} cannot be prepared -> {e}")
        return False

def tsc_set_coordinates(client: ModbusTcpClient, dev_id: int, coordinates: TscCoordinates) -> bool:
    """Define the location of the device writing the longitude and latitude

    :param client: ModbusTcpClient
    :param dev_id: modbus device id
    :param coordinates: array with device coordinates modbus registers array, float little endian
    return True if the registers were configured, False otherwise.
    """
    try:
        writing = client. write_registers(address=41010, values=coordinates.modbus_package, device_id=dev_id)
        if not writing.isError():
            print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} successfully.")
            return True
        else:
            print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} cannot be configured.")
        return False
    except Exception as e:
        print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} cannot be prepared -> {e}")
        return False



def tsc_configure_telemetry(client: ModbusTcpClient, dev_id: int, conf: TscConfiguration) -> bool:
    """Configure the telemetry of the TSC.

        :param client: ModbusTcpClient
        :param dev_id: modbus device id
        :param conf: array with telemetry configuration modbus registers array
        return True if the registers were configured, False otherwise.
    """
    try:
        writing = client.write_registers(address=45000, values=conf.modbus_package, device_id=dev_id)
        if not writing.isError():
            print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} configured successfully.")
            return True
        else:
            print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} cannot be configured.")
        return False
    except Exception as e:
        print(f"TSC {dev_id} {inspect.currentframe().f_code.co_name} cannot be prepared -> {e}")
        return False

def tsc_save_non_volatile_memory(client: ModbusTcpClient, dev_id: int) -> bool:
    """Save the tsc data stored in the flash memory in the non-volatile memory.

        :param client: ModbusTcpClient
        :param dev_id: modbus device id
        return True if the registers were configured, False otherwise.
    """
    try:
        save_nvm = client.write_register(address=40007, value=1 << 15, device_id=dev_id)
        if not save_nvm.isError():
            print(f"TSC {dev_id} configured successfully.")
            return True
        else:
            print(f"TSC {dev_id} cannot save the configuration.")
        return False
    except Exception as e:
        print(f"TSC {dev_id} cannot be configured -> {e}")
        return False

def iwc_save_non_volatile_memory(client: ModbusTcpClient, dev_id: int) -> bool:
    """Save the iwc data stored in the flash memory in the non-volatile memory.

        :param client: ModbusTcpClient
        :param dev_id: modbus device id
        return True if the registers were configured, False otherwise.
    """
    try:
        save_nvm = client.write_register(address=40007, value=1 << 15, device_id=dev_id)
        if not save_nvm.isError():
            print(f"IWC {dev_id} configured successfully.")
            return True
        else:
            print(f"IWC {dev_id} cannot save the configuration.")
        return False
    except Exception as e:
        print(f"IWC {dev_id} cannot be configured -> {e}")
        return False

def iwc_configure_telemetry(client: ModbusTcpClient, dev_id: int, conf: IwcConfiguration) -> bool:
    """Configure the telemetry of the IWC.

    :param client: ModbusTcpClient
    :param dev_id: modbus device id
    :param conf: array with telemetry configuration modbus registers array
    return True if the registers were configured, False otherwise.
    """
    try:
        writing = client.write_registers(address=41224, values=conf.modbus_package, device_id=dev_id)
        if not writing.isError():
            save_nvm = client.write_register(address=40007, value=1 << 15, device_id=dev_id)
            if not save_nvm.isError():
                print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} configured successfully.")
                return True
            else:
                print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} cannot save the configuration.")
        else:
            print(f"IWC {dev_id} {inspect.currentframe().f_code.co_name} cannot be configured.")
        return False
    except Exception as e:
        print(f"IWC {dev_id} cannot be configured -> {e}")
        return False


def tsc_sync_clock(client: ModbusTcpClient, dev_id: int) -> bool:
    """Sync the clock of the gateway.

    param new_time: The new time to set.
    return: True if the clock was synced, False otherwise.
    """
    try:
        and_mask = 0xFFFC
        date_utc = datetime.datetime.now(datetime.timezone.utc)
        date_array = [date_utc.second, date_utc.minute, date_utc.hour, date_utc.day, date_utc.month, date_utc.year]

        writing = client.mask_write_register(address=reg_extended_control, and_mask=and_mask, or_mask=0x0001, device_id=dev_id)
        if not writing.isError():
            writing = client.write_registers(address=40001, values=date_array, device_id=dev_id)
            if not writing.isError():
                writing = client.mask_write_register(address=reg_extended_control, and_mask=and_mask, or_mask=0x0003, device_id=dev_id)
                if not writing.isError():
                    print(f"TSC {dev_id} date updated successfully.")
                    return True
                else:
                    print(f"TSC {dev_id} cannot update the date.")
            else:
                print(f"TSC {dev_id} cannot write the date.")
        else:
            print(f"TSC {dev_id} cannot enable the date change.")
        client.mask_write_register(address=reg_extended_control, and_mask=and_mask, or_mask=0x0000, device_id=dev_id)
        return False
    except Exception as e:
        print(f"TSC {dev_id} date cannot be updated -> {e}")
        return False

def iwc_sync_clock(client: ModbusTcpClient, dev_id: int) -> bool:
    """Sync the clock of the gateway.

    param new_time: The new time to set.
    return: True if the clock was synced, False otherwise.
    """
    try:
        date_utc = datetime.datetime.now(datetime.timezone.utc)
        date_array = [date_utc.second, date_utc.minute, date_utc.hour, date_utc.day, date_utc.month, date_utc.year]

        writing = client.write_register(address=reg_extended_control, value=1, device_id=dev_id)
        if not writing.isError():
            writing = client.write_registers(address=40001, values=date_array, device_id=dev_id)
            if not writing.isError():
                writing = client.write_register(address=reg_extended_control, value=3, device_id=dev_id)
                if not writing.isError():
                    print(f"IWC {dev_id} date updated successfully.")
                    return True
                else:
                    print(f"IWC {dev_id} cannot update the date.")
            else:
                print(f"IWC {dev_id} cannot write the date.")
        else:
            print(f"IWC {dev_id} cannot enable the date change.")
        client.write_register(address=reg_extended_control, value=0, device_id=dev_id)
        return False
    except Exception as e:
        print(f"IWC {dev_id} date cannot be updated -> {e}")
        return False



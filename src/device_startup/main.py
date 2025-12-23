from pymodbus.client import ModbusTcpClient
from src.device_startup.functions import (TscConfiguration, IwcConfiguration, TscCoordinates, TscSunTracking, TscSwMovementLimit, TscSmartLimits, TscSafePositions, TscBacktracking,\
    IwcWindSpeedThresholds, 
    tsc_sync_clock, \
    iwc_sync_clock, tsc_configure_telemetry, iwc_configure_telemetry, tsc_set_coordinates, tsc_set_solar_tracking, \
    TscPanID, tsc_set_pan_id, tsc_reset_communication, tsc_software_movement_limits, tsc_smart_limits, tsc_safe_positions, tsc_backtracking3d, tsc_save_non_volatile_memory, tsc_set_modbus_id, tsc_set_commissioning_state)

tsc_polling_packages = [
    (30000, 4),
    (42000, 1),
    (30006, 1),
    (30084, 1),
    (40000, 1),
    (30011, 2),
    (30088, 10),
    (30115, 10),
    (30041, 2),
    (30046, 3),
    (41004, 1),
    (30064, 1)]

iwc_polling_packages = [
    (30000, 14),
    (40001, 6),
    (41008, 1),
    (41213, 1),
    (41221, 1),
    (50026, 2)]

smart_limits = {
    "west1": 35,
    "west2": 30,
    "west3": 25,
    "west4": 20,
    "west5": 15,
    "west6": 10,
    "west7": 5,
    "east1": -35,
    "east2": -30,
    "east3": -25,
    "east4": -20,
    "east5": -15,
    "east6": -10,
    "east7": -5,
}

safe_positions = {
    "SP1": 25,
    "SP2": 15,
    "SP3": 35,
    "SP4": 30,
    "SP5": 40,
    "SP6": 50,
    "SP7": 0
}

client = ModbusTcpClient(host='10.12.241.101', port=502)

client.connect()

if client.connect():
    try:
        tsc_date_updated = tsc_sync_clock(client, 121)
        #iwc_date_updated = iwc_sync_clock(client, 230)
    except Exception as e:
        print(f"Dates cannot be updated -> {e}")
    try:
        tsc_conf_telemetry = TscConfiguration(tsc_polling_packages)
        tsc_conf_coordinates = TscCoordinates(longitude=-0.028761, latitude=0.746828)
        tsc_conf_sun_tracking = TscSunTracking(pitch=5, panel_width=2.01)
        tsc_conf_pan_id = TscPanID('0000000000000103')
        tsc_sw_limits = TscSwMovementLimit(55, -55)
        tsc_smart_limits_regs = TscSmartLimits(smart_limits)
        tsc_safe_positions_regs = TscSafePositions(safe_positions)
        tsc_backtracking_3d = TscBacktracking(2,3)

        iwc_conf_telemetry = IwcConfiguration(iwc_polling_packages)
    except Exception as e:
        print(f'Conf wrong {e}')
    else:
        """
        tsc_telemetry_updated = tsc_configure_telemetry(client, 121, tsc_conf_telemetry)
        tsc_set_coordinates(client, 121, tsc_conf_coordinates)
        tsc_set_solar_tracking(client, 121, tsc_conf_sun_tracking)
        tsc_software_movement_limits(client, 121, tsc_sw_limits)
        tsc_smart_limits(client, 121, tsc_smart_limits_regs)
        tsc_safe_positions(client, 121, tsc_safe_positions_regs)
        tsc_backtracking3d(client, 121, tsc_backtracking_3d)
        tsc_set_commissioning_state(client, 121, 0)
        tsc_set_modbus_id(client, 225, 121)
        tsc_set_pan_id(client, 121, tsc_conf_pan_id)
        tsc_save_non_volatile_memory(client, 121)
        tsc_reset_communication(client, 225)
        """

        #iwc_telemetry_updated = iwc_configure_telemetry(client, 230, iwc_conf_telemetry)

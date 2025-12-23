from pymodbus.client import ModbusTcpClient
from src.device_startup.functions import (TscConfiguration, IwcConfiguration, TscCoordinates, TscSunTracking, TscPanID,
                                          TscSwMovementLimit, TscSmartLimits, TscSafePositions, TscBacktracking,
                                          IwcWindSpeedThresholds, IwcWindSpeedThresholds, IwcWindAlarmTime,
                                          IwcAvgTime, IwcRelaxTime, IwcSnowThreshold,
                                          tsc_sync_clock, tsc_configure_telemetry, iwc_configure_telemetry,
                                          tsc_set_coordinates, tsc_set_solar_tracking, tsc_set_pan_id,
                                          tsc_reset_communication, tsc_software_movement_limits, tsc_smart_limits,
                                          tsc_safe_positions, tsc_backtracking3d, tsc_save_non_volatile_memory,
                                          tsc_set_modbus_id, tsc_set_commissioning_state,
                                          iwc_sync_clock, iwc_wind_speed_thresholds, iwc_wind_alarm_time,
                                          iwc_wind_alarms_2_and_3, iwc_relax_time, iwc_snow_threshold,
                                          IwcSnowActivationTime,
                                          IwcSnowDeactivationTime, IwcSnowSampleTime, IwcSnowMaxVariation,
                                          iwc_snow_alarm_activation_time, iwc_snow_deactivation_time,
                                          iwc_snow_sample_period, iwc_snow_maximum_variation, IwcPanID, iwc_set_pan_id,
                                          iwc_set_modbus_id, iwc_save_non_volatile_memory, iwc_set_sensors)
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
        iwc_date_updated = iwc_sync_clock(client, 230)
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
        iwc_wind_speed_threshold1 = IwcWindSpeedThresholds(60, 60)
        iwc_wind_alarm_time1 = IwcWindAlarmTime(1, 1)
        iwc_wind_speed_threshold2 = IwcWindSpeedThresholds(60, 60)
        iwc_wind_alarm_time2 = IwcWindAlarmTime(1, 1)
        iwc_wind_speed_threshold3 = IwcWindSpeedThresholds(37, 42)
        iwc_wind_alarm_time3 = IwcWindAlarmTime(1, 1)
        iwc_avg_time = IwcAvgTime(1)
        iwc_relax_t = IwcRelaxTime(20)
        iwc_snow_thr = IwcSnowThreshold(0.08)
        iwc_snow_conf1 = IwcSnowActivationTime(10)
        iwc_snow_conf2 = IwcSnowDeactivationTime(10)
        iwc_snow_conf3 = IwcSnowSampleTime(2)
        iwc_snow_conf4 = IwcSnowMaxVariation(80)
        iwc_conf_pan_id = IwcPanID('0000000000000103')

    except Exception as e:
        print(f'Conf wrong {e}')
    else:
        """"""
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


        iwc_telemetry_updated = iwc_configure_telemetry(client, 230, iwc_conf_telemetry)
        iwc_wind_speed_thresholds(client, 230, iwc_wind_speed_threshold1)
        iwc_wind_alarm_time(client, 230, iwc_wind_alarm_time1)
        iwc_wind_alarms_2_and_3(client, 230, iwc_wind_speed_threshold2, iwc_wind_speed_threshold3, iwc_wind_alarm_time2, iwc_wind_alarm_time3, iwc_avg_time)
        iwc_relax_time(client, 230, iwc_relax_t)
        iwc_snow_threshold(client, 230, iwc_snow_thr)
        iwc_snow_alarm_activation_time(client, 230, iwc_snow_conf1)
        iwc_snow_deactivation_time(client, 230, iwc_snow_conf2)
        iwc_snow_sample_period(client, 230, iwc_snow_conf3)
        iwc_snow_maximum_variation(client, 230, iwc_snow_conf4)
        iwc_set_sensors(client, 230, True,False,False,False,False)
        iwc_save_non_volatile_memory(client, 230)
        iwc_set_pan_id(client, 230, iwc_conf_pan_id)
        iwc_set_modbus_id(client, 230, 231)

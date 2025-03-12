import serial
import pyvesc
from pyvesc import VESCMessage
from pyvesc.VESC.messages import VedderCmd

'''
This script implements the most crucial part of the PyVESC interface.

The message classes GetMcConf and SetMcConf and defined here.
These classes implement motor controller configuration message setters and getters.
They enable FOC-parameter tuning, e.g tuning observer gain and -offset, pid KI and pid KP 
among alot of other paramter tuning.
'''

# Use correct serial port for your PC, VESC-tool can be used for this puropse
VESC_port = "/dev/ttyACM0"


DOUBLE_16 = 'h'
DOUBLE_32_AUTO = 'f'
BOOL = '?'
INT_32 = 'l'
INT_16 = 'h'
INT_8 = 'b'
UINT_32 = 'L'
UINT_16 = 'H'
UINT_8 = 'B'
ENUM = 'B'
BITFIELD = 'L'   # Not sure of this one, bitfield with 8 fields, testing long long

def read_full_packet(ser, timeout=2):
    """
    Accumulates bytes from the serial port until a complete VESC packet is available,
    then returns the decoded message.
    """
    buffer = b""
    ser.timeout = timeout
    while True:
        # Read available data
        new_data = ser.read(ser.in_waiting or 1)
        if new_data:
            buffer += new_data
            # Check if we at least have a valid start byte
            if buffer[0] not in [0x02, 0x03]:
                # Discard until we get a valid start byte
                buffer = buffer[1:]
                continue

            try:
                # Attempt to decode
                msg, consumed = pyvesc.decode(buffer)
                if msg is not None:
                    # Remove consumed bytes from buffer (in case more data is waiting)
                    buffer = buffer[consumed:]
                    return msg
            except Exception as e:
                # If decode raises an error, print it and break (or handle it as needed)
                print("Decode error:", e)
                break
        else:
            # If no new data is available, you might decide to timeout or continue waiting.
            break
    return None

# Message ids
COMM_GET_MCCONF = VedderCmd.COMM_GET_MCCONF # 14
COMM_SET_MCCONF = VedderCmd.COMM_SET_MCCONF # 13

class GetMcConf(metaclass=pyvesc.VESCMessage):
    
    # **TODO** : Adjust scaling factor and check data types
    
    id = COMM_GET_MCCONF
    can_id = None

    '''
     Fields syntax: (name, data type, scaling factor)
     The fields are taken from a fork implementing MCCONF commands (Supports fw 3.33)
     but also compared to fields from vedders bldc/datatypes.h
     in mc_configuration struct, and some fields added for fw 6.2 support.
     Lastly the data types and scaling comes from the VESC Parameter Editor
    '''

    # Fields based on confgenerator.c and
    # updated using VESC Parameter Editor
    '''
    # Gives some correct values e.g observer offset
    fields = [
        ('pwm_mode', 'B', 1),
        ('comm_mode', 'B', 1),
        ('motor_type', 'B', 1),
        ('sensor_mode', 'B', 1),
        ('l_current_max', DOUBLE_32_AUTO, 1000),
        ('l_current_min', DOUBLE_32_AUTO, 1000),
        ('l_in_current_max', DOUBLE_32_AUTO, 1000),
        ('l_in_current_min', DOUBLE_32_AUTO, 1000),
        ('l_abs_current_max', DOUBLE_32_AUTO, 1000),
        ('l_min_erpm', DOUBLE_32_AUTO, 1000),
        ('l_max_erpm', DOUBLE_32_AUTO, 1000),
        ('l_erpm_start', DOUBLE_16, 1e4),
        ('l_max_erpm_fbrake', DOUBLE_32_AUTO, 1000),
        ('l_max_erpm_fbrake_cc', DOUBLE_32_AUTO, 1000),
        ('l_min_vin', DOUBLE_16, 10),
        ('l_max_vin', DOUBLE_16, 10),
        ('l_battery_cut_start', DOUBLE_16, 10),
        ('l_battery_cut_end', DOUBLE_16, 10),
        ('l_slow_abs_current', 'B', 1), # Boolean
        ('l_temp_fet_start', 'B', 1), 
        ('l_temp_fet_end', 'B', 1),
        ('l_temp_motor_start', 'B', 1),
        ('l_temp_motor_end', 'B', 1),
        ('l_temp_accel_dec', DOUBLE_16, 1e4),
        ('l_min_duty', DOUBLE_16, 1e4),
        ('l_max_duty', DOUBLE_16, 1e4),
        ('l_watt_max', DOUBLE_32_AUTO, 1),
        ('l_watt_min', DOUBLE_32_AUTO, 1),
        ('l_current_max_scale', DOUBLE_16, 1e4), # Added for 6.2 support
        ('l_current_min_scale', DOUBLE_16, 1e4), # Added for 6.2 support
        ('l_duty_start', DOUBLE_16, 1e4), # Added for 6.2 support
        ('sl_min_erpm', DOUBLE_32_AUTO, 1000),
        ('sl_min_erpm_cycle_int_limit', DOUBLE_32_AUTO, 1000),
        ('sl_max_fullbreak_current_dir_change', DOUBLE_32_AUTO, 1000),
        ('sl_cycle_int_limit', DOUBLE_16, 10),
        ('sl_phase_advance_at_br', DOUBLE_16, 1e4),
        ('sl_cycle_int_rpm_br', DOUBLE_32_AUTO, 1000),
        ('sl_bemf_coupling_k', DOUBLE_32_AUTO, 1000),
        ('hall_table__0', 'b', 1), 
        ('hall_table__1', 'b', 1),
        ('hall_table__2', 'b', 1),
        ('hall_table__3', 'b', 1),
        ('hall_table__4', 'b', 1),
        ('hall_table__5', 'b', 1),
        ('hall_table__6', 'b', 1),
        ('hall_table__7', 'b', 1),
        ('hall_sl_erpm', DOUBLE_32_AUTO, 1000),
        ('foc_current_kp', DOUBLE_32_AUTO, 1e5),
        ('foc_current_ki', DOUBLE_32_AUTO, 1e5),
        ('foc_f_zv', DOUBLE_32_AUTO, 1000), # Changed name from 'foc_f_sw' to support 6.2
        ('foc_dt_us', DOUBLE_32_AUTO, 1e6),
        ('foc_encoder_inverted', 'B', 1), # Boolean
        ('foc_encoder_offset', DOUBLE_32_AUTO, 1000),
        ('foc_encoder_ratio', DOUBLE_32_AUTO, 1000),
        ('foc_sensor_mode', 'B', 1),
        ('foc_pll_kp', DOUBLE_32_AUTO, 1000),
        ('foc_pll_ki', DOUBLE_32_AUTO, 1000),
        ('foc_motor_l', DOUBLE_32_AUTO, 1e8),
        ('foc_ld_lq_diff', DOUBLE_32_AUTO, 1e8), # Added for 6.2 support
        ('foc_motor_r', DOUBLE_32_AUTO, 1e5),
        ('foc_motor_flux_linkage', DOUBLE_32_AUTO, 1e5),
        ('foc_observer_gain', DOUBLE_16, 1e-2),
        ('foc_observer_gain_slow', DOUBLE_32_AUTO, 1e-2),
        ('foc_observer_offset', DOUBLE_16, 1000), # Added for 6.2 support
        ('foc_duty_dowmramp_kp', DOUBLE_32_AUTO, 1000),
        ('foc_duty_dowmramp_ki', DOUBLE_32_AUTO, 1000),
        ('foc_start_curr_dec', DOUBLE_16, 1e4), # Added for 6.2 support
        ('foc_start_curr_dec_rpm', DOUBLE_32_AUTO, 1000), # Added for 6.2 support
        ('foc_openloop_rpm', DOUBLE_32_AUTO, 1000),
        ('foc_openloop_rpm_low', DOUBLE_16, 1000), # Added for 6.2 support
        ('foc_d_gain_scale_start', DOUBLE_16, 1000), # Added for 6.2 support
        ('foc_d_gain_scale_max_mod', DOUBLE_16, 1000), # Added for 6.2 support
        ('foc_sl_openloop_hyst', DOUBLE_16, 100),
        ('foc_sl_openloop_time_lock', DOUBLE_16, 100), # Added for 6.2 support
        ('foc_sl_openloop_time_ramp', DOUBLE_16, 100), # Added for 6.2 support
        ('foc_sl_openloop_time', DOUBLE_16, 100),
        ('foc_sl_openloop_boost_q', DOUBLE_16, 100), # Added for 6.2 support
        ('foc_sl_openloop__max_q', DOUBLE_16, 100), # Added for 6.2 support
        ('foc_hall_table__0', 'B', 1),
        ('foc_hall_table__1', 'B', 1),
        ('foc_hall_table__2', 'B', 1),
        ('foc_hall_table__3', 'B', 1),
        ('foc_hall_table__4', 'B', 1),
        ('foc_hall_table__5', 'B', 1),
        ('foc_hall_table__6', 'B', 1),
        ('foc_hall_table__7', 'B', 1),
        ('foc_hall_interp_erpm', DOUBLE_32_AUTO, 1), # Added for 6.2 support
        ('foc_sl_erpm', DOUBLE_32_AUTO, 1000),
        ('foc_sample_v0_v7',BOOL,1),
        ('foc_sample_high_current', BOOL, 1),
        ('foc_sat_comp_mode', 'B', 1), # Added for 6.2 support
        ('foc_sat_comp', DOUBLE_16, 1000),
        ('foc_temp_comp', 'B', 1),
        ('foc_temp_comp_base_temp', DOUBLE_16, 100),
        ('foc_current_filter_const', DOUBLE_16, 1e4), # Added for 6.2 support
        ('foc_cc_decoupling', 'B', 1),  # Added for 6.2 support (Unsure of data type)
        ('foc_observer_type', 'B', 1),  # Added for 6.2 support (Data type ?)
        ('foc_hfi_voltage_start', DOUBLE_16, 10),
        ('foc_hfi_voltage_run', DOUBLE_16, 10),
        ('foc_hfi_voltage_max', DOUBLE_16, 10),
        ('foc_hfi_gain', DOUBLE_16, 1000),
        ('foc_hfi_hyst', DOUBLE_16, 100),
        ('foc_sl_erpm_hfi', DOUBLE_32_AUTO, 1000),
        ('foc_hfi_start_samples', 'H', 1),
        ('foc_hfi_obs_ovr_sec', DOUBLE_32_AUTO, 1000),
        ('foc_hfi_samples', 'B', 1),
        ('foc_offsets_cal_on_boot', 'B', 1),
        ('foc_offsets_current__0', DOUBLE_32_AUTO, 1),
        ('foc_offsets_current__1', DOUBLE_32_AUTO, 1),
        ('foc_offsets_current__2', DOUBLE_32_AUTO, 1),
        ('foc_offsets_voltage__0', DOUBLE_16, 1e4),
        ('foc_offsets_voltage__1', DOUBLE_16, 1e4),
        ('foc_offsets_voltage__2', DOUBLE_16, 1e4),
        ('foc_offsets_voltage_undriven_0', DOUBLE_16, 1e4),
        ('foc_offsets_voltage_undriven_1', DOUBLE_16, 1e4),
        ('foc_offsets_voltage_undriven_2', DOUBLE_16, 1e4),
        ('foc_phase_filter_enable', 'B', 1),
        ('foc_phase_filter_disable_fault', 'B', 1),
        ('foc_phase_filter_max_erpm', DOUBLE_32_AUTO, 1e4),
        ('foc_mtpa_mode', 'B', 1),
        ('foc_fw_current_max', DOUBLE_32_AUTO, 1000),
        ('foc_fw_duty_start', DOUBLE_16, 1e4),
        ('foc_fw_ramp_time', DOUBLE_16, 1000),
        ('foc_fw_q_current_factor', DOUBLE_16, 1e4),
        ('foc_speed_soure', 'B', 1),
        ('gpd_buffer_notify_left', INT_16, 1),
        ('gpd_buffer_interpol', INT_16, 1),
        ('gpd_current_filter_const', DOUBLE_16, 1e4),
        ('gpd_current_kp', DOUBLE_32_AUTO, 1e5),
        ('gpd_current_ki', DOUBLE_32_AUTO, 1e5),
        ('sp_pid_loop_rate', 'B', 1),
        ('s_pid_kp', DOUBLE_32_AUTO, 1e6),
        ('s_pid_ki', DOUBLE_32_AUTO, 1e6),
        ('s_pid_kd', DOUBLE_32_AUTO, 1e6),
        ('s_pid_kd_filter', DOUBLE_16, 1e4), # Added for 6.2 support
        ('s_pid_min_erpm', DOUBLE_32_AUTO, 1000),
        ('s_pid_allow_braking', 'B', 1),
        ('s_pid_ramp_erpms_s', DOUBLE_32_AUTO, 1000), # Added for 6.2 support
        ('p_pid_kp', DOUBLE_32_AUTO, 1e6),
        ('p_pid_ki', DOUBLE_32_AUTO, 1e6),
        ('p_pid_kd', DOUBLE_32_AUTO, 1e6),
        ('p_pid_kd_proc', DOUBLE_32_AUTO, 1e6), # Added for 6.2 support
        ('p_pid_kd_filter', DOUBLE_16, 1e4), # Added for 6.2 support
        ('p_pid_ang_div', DOUBLE_32_AUTO, 1e5),
        ('p_pid_gain_dec_angle', DOUBLE_16, 10), # Added for 6.2 support
        ('p_pid_offset', DOUBLE_32_AUTO, 1e6), # Added for 6.2 support
        ('cc_startup_boost_duty', DOUBLE_16, 1e4),
        ('cc_min_current', DOUBLE_32_AUTO, 1000),
        ('cc_gain', DOUBLE_32_AUTO, 1e6),
        ('cc_ramp_step_max', DOUBLE_16, 1e4),
        ('m_fault_stop_time_ms', 'i', 1),
        ('m_duty_ramp_step', DOUBLE_16, 1e4),
        ('m_current_backoff_gain', DOUBLE_32_AUTO, 1e6),
        ('m_encoder_counts', 'I', 1),
        ('m_encoder_sin_amp', DOUBLE_16, 1000), # Added for 6.2 support
        ('m_encoder_cos_amp', DOUBLE_16, 1000), # Added for 6.2 support
        ('m_encoder_sin_offset', DOUBLE_16, 1000), # Added for 6.2 support
        ('m_encoder_cos_offset', DOUBLE_16, 1000), # Added for 6.2 support
        ('m_encoder_sincos_filter_constant', DOUBLE_16, 1000), # Added for 6.2 support
        ('m_encoder_sincos_phase_correction', DOUBLE_16, 1000), # Added for 6.2 support
        ('m_sensor_port_mode', 'B', 1),
        ('m_invert_direction', 'B', 1),
        ('m_drv8301_oc_mode', 'B', 1),
        ('m_drv8301_oc_adj', 'B', 1),
        ('m_bldc_f_sw_min', DOUBLE_32_AUTO, 1),
        ('m_bldc_f_sw_max', DOUBLE_32_AUTO, 1),
        ('m_dc_f_sw', DOUBLE_32_AUTO, 1),
        ('m_ntc_motor_beta', DOUBLE_32_AUTO, 1),
        ('m_out_aux_mode', 'B', 1), # Followling lines were added to support 6.2
        ('m_motor_temp_sens_type', 'B', 1),
        ('m_ptc_motor_coeff', DOUBLE_32_AUTO, 1),
        ('m_ntcx_ptcx_res', DOUBLE_16, 1e-1),
        ('m_ntcx_ptcx_temp_base', DOUBLE_16, 10),
        ('m_hall_extra_samples', 'B', 1),
        ('m_batt_filter_const', 'B', 1),
        ('si_motor_poles', 'B', 1),
        ('si_gear_ratio', DOUBLE_32_AUTO, 1),
        ('si_wheel_diameter', DOUBLE_32_AUTO, 1),
        ('si_battery_type', 'B', 1),
        ('si_battery_cells', 'B', 1),
        ('si_battery_ah', DOUBLE_32_AUTO, 1),
        ('si_motor_nl_current', DOUBLE_32_AUTO, 1),
        ('bms.type', 'B', 1),
        ('bms.limit_mode', 'B', 1),
        ('bms.t_limit_start', 'B', 1),
        ('bms.t_limit_end', 'B', 1),
        ('bms.soc_limit_start', DOUBLE_16, 1000),
        ('bms.soc_limit_end', DOUBLE_16, 1000),
        ('bms.fwd_can_mode', 'B', 1),
    ]
    '''
    fields = [
        ('pwm_mode', ENUM, 1),
        ('comm_mode', ENUM, 1),
        ('motor_type', ENUM, 1),
        ('sensor_mode', ENUM, 1),
        ('l_current_max', DOUBLE_32_AUTO, 1),
        ('l_current_min', DOUBLE_32_AUTO, 1),
        ('l_in_current_max', DOUBLE_32_AUTO, 1),
        ('l_in_current_min', DOUBLE_32_AUTO, 1),
        ('l_in_current_map_start', DOUBLE_16, 1), # Added for 6.2 support
        ('l_in_current_map_filter', DOUBLE_16, 1), # Added for 6.2 support
        ('l_abs_current_max', DOUBLE_32_AUTO, 1),
        ('l_min_erpm', DOUBLE_32_AUTO, 1),
        ('l_max_erpm', DOUBLE_32_AUTO, 1),
        ('l_erpm_start', DOUBLE_16, 1),
        ('l_max_erpm_fbrake', DOUBLE_32_AUTO, 1),
        ('l_max_erpm_fbrake_cc', DOUBLE_32_AUTO, 1),
        ('l_min_vin', DOUBLE_16, 1),
        ('l_max_vin', DOUBLE_16, 1),
        ('l_battery_cut_start', DOUBLE_16, 10),
        ('l_battery_cut_end', DOUBLE_16, 10),
        ('l_battery_regen_cut_start', DOUBLE_16, 10), # Added for 6.2 support
        ('l_battery_regen_cut_end', DOUBLE_16, 10), # Added for 6.2 support
        ('l_slow_abs_current', BOOL, 1), 
        ('l_temp_fet_start', UINT_8, 1), 
        ('l_temp_fet_end', UINT_8, 1),
        ('l_temp_motor_start', UINT_8, 1),
        ('l_temp_motor_end', UINT_8, 1),
        ('l_temp_accel_dec', DOUBLE_16, 1e4),
        ('l_min_duty', DOUBLE_16, 1e4),
        ('l_max_duty', DOUBLE_16, 1e4),
        ('l_watt_max', DOUBLE_32_AUTO, 1),
        ('l_watt_min', DOUBLE_32_AUTO, 1),
        ('l_current_max_scale', DOUBLE_16, 1e4), # Added for 6.2 support
        ('l_current_min_scale', DOUBLE_16, 1e4), # Added for 6.2 support
        ('l_duty_start', DOUBLE_16, 1e4), # Added for 6.2 support
        ('sl_min_erpm', DOUBLE_32_AUTO, 1000),
        ('sl_min_erpm_cycle_int_limit', DOUBLE_32_AUTO, 1000),
        ('sl_max_fullbreak_current_dir_change', DOUBLE_32_AUTO, 1000),
        ('sl_cycle_int_limit', DOUBLE_16, 10),
        ('sl_phase_advance_at_br', DOUBLE_16, 1e4),
        ('sl_cycle_int_rpm_br', DOUBLE_32_AUTO, 1000),
        ('sl_bemf_coupling_k', DOUBLE_32_AUTO, 1000),
        ('hall_table__0', INT_8, 1), 
        ('hall_table__1', INT_8, 1),
        ('hall_table__2', INT_8, 1),
        ('hall_table__3', INT_8, 1),
        ('hall_table__4', INT_8, 1),
        ('hall_table__5', INT_8, 1),
        ('hall_table__6', INT_8, 1),
        ('hall_table__7', INT_8, 1),
        ('hall_sl_erpm', DOUBLE_32_AUTO, 1000),
        ('foc_current_kp', DOUBLE_32_AUTO, 1e5),
        ('foc_current_ki', DOUBLE_32_AUTO, 1e5),
        ('foc_f_zv', DOUBLE_32_AUTO, 1000), # Changed name from 'foc_f_sw' to support 6.2
        ('foc_dt_us', DOUBLE_32_AUTO, 1e6),
        ('foc_encoder_inverted', BOOL, 1), # Boolean
        ('foc_encoder_offset', DOUBLE_32_AUTO, 1000),
        ('foc_encoder_ratio', DOUBLE_32_AUTO, 1000),
        ('foc_sensor_mode', ENUM, 1),
        ('foc_pll_kp', DOUBLE_32_AUTO, 1000),
        ('foc_pll_ki', DOUBLE_32_AUTO, 1000),
        ('foc_motor_l', DOUBLE_32_AUTO, 1e8),
        ('foc_ld_lq_diff', DOUBLE_32_AUTO, 1e8), # Added for 6.2 support
        ('foc_motor_r', DOUBLE_32_AUTO, 1e5),
        ('foc_motor_flux_linkage', DOUBLE_32_AUTO, 1e5),
        ('foc_observer_gain', DOUBLE_32_AUTO, 1e-2),
        ('foc_observer_gain_slow', DOUBLE_32_AUTO, 1e-2),
        ('foc_observer_offset', DOUBLE_16, 1000), # Added for 6.2 support
        ('foc_duty_dowmramp_kp', DOUBLE_32_AUTO, 1000),
        ('foc_duty_dowmramp_ki', DOUBLE_32_AUTO, 1000),
        ('foc_start_curr_dec', DOUBLE_16, 1e4), # Added for 6.2 support
        ('foc_start_curr_dec_rpm', DOUBLE_32_AUTO, 1000), # Added for 6.2 support
        ('foc_openloop_rpm', DOUBLE_32_AUTO, 1000),
        ('foc_openloop_rpm_low', DOUBLE_16, 1000), # Added for 6.2 support
        ('foc_d_gain_scale_start', DOUBLE_16, 1000), # Added for 6.2 support
        ('foc_d_gain_scale_max_mod', DOUBLE_16, 1000), # Added for 6.2 support
        ('foc_sl_openloop_hyst', DOUBLE_16, 100),
        ('foc_sl_openloop_time_lock', DOUBLE_16, 100), # Added for 6.2 support
        ('foc_sl_openloop_time_ramp', DOUBLE_16, 100), # Added for 6.2 support
        ('foc_sl_openloop_time', DOUBLE_16, 100),
        ('foc_sl_openloop_boost_q', DOUBLE_16, 100), # Added for 6.2 support
        ('foc_sl_openloop__max_q', DOUBLE_16, 100), # Added for 6.2 support
        ('foc_hall_table__0', UINT_8, 1),
        ('foc_hall_table__1', UINT_8, 1),
        ('foc_hall_table__2', UINT_8, 1),
        ('foc_hall_table__3', UINT_8, 1),
        ('foc_hall_table__4', UINT_8, 1),
        ('foc_hall_table__5', UINT_8, 1),
        ('foc_hall_table__6', UINT_8, 1),
        ('foc_hall_table__7', UINT_8, 1),
        ('foc_hall_interp_erpm', DOUBLE_32_AUTO, 1), # Added for 6.2 support
        ('foc_sl_erpm_start', DOUBLE_32_AUTO, 1000), # Added for 6.2 support
        ('foc_sl_erpm', DOUBLE_32_AUTO, 1000),
        ('foc_control_sample_mode', ENUM, 1), # Added for 6.2 support
        ('foc_current_sample_mode', ENUM, 1), # Added for 6.2 support
        ('foc_sat_comp_mode', ENUM, 1), # Added for 6.2 support
        ('foc_sat_comp', DOUBLE_16, 1000),
        ('foc_temp_comp', BOOL, 1),
        ('foc_temp_comp_base_temp', DOUBLE_16, 100),
        ('foc_current_filter_const', DOUBLE_16, 1e4), # Added for 6.2 support
        ('foc_cc_decoupling', ENUM, 1),  # Added for 6.2 support (Unsure of data type)
        ('foc_observer_type', ENUM, 1),  # Added for 6.2 support (Data type ?)
        ('foc_hfi_voltage_start', DOUBLE_16, 10),
        ('foc_hfi_voltage_run', DOUBLE_16, 10),
        ('foc_hfi_voltage_max', DOUBLE_16, 10),
        ('foc_hfi_gain', DOUBLE_16, 1000),
        ('foc_hfi_max_err', DOUBLE_16, 1000),
        ('foc_hfi_hyst', DOUBLE_16, 100),
        ('foc_sl_erpm_hfi', DOUBLE_32_AUTO, 1000),
        ('foc_hfi_start_samples', UINT_16, 1),
        ('foc_hfi_obs_ovr_sec', DOUBLE_32_AUTO, 1000),
        ('foc_hfi_samples', ENUM, 1),
        ('foc_offsets_cal_on_boot', BOOL, 1),
        ('foc_offsets_current__0', DOUBLE_32_AUTO, 1),
        ('foc_offsets_current__1', DOUBLE_32_AUTO, 1),
        ('foc_offsets_current__2', DOUBLE_32_AUTO, 1),
        ('foc_offsets_voltage__0', DOUBLE_16, 1e4),
        ('foc_offsets_voltage__1', DOUBLE_16, 1e4),
        ('foc_offsets_voltage__2', DOUBLE_16, 1e4),
        ('foc_offsets_voltage_undriven_0', DOUBLE_16, 1e4),
        ('foc_offsets_voltage_undriven_1', DOUBLE_16, 1e4),
        ('foc_offsets_voltage_undriven_2', DOUBLE_16, 1e4),
        ('foc_phase_filter_enable', BOOL, 1),
        ('foc_phase_filter_disable_fault', BOOL, 1),
        ('foc_phase_filter_max_erpm', DOUBLE_32_AUTO, 1e4),
        ('foc_mtpa_mode', ENUM, 1),
        ('foc_fw_current_max', DOUBLE_32_AUTO, 1000),
        ('foc_fw_duty_start', DOUBLE_16, 1e4),
        ('foc_fw_q_current_factor', DOUBLE_16, 1e4),
        ('foc_fw_ramp_time', DOUBLE_16, 1000),
        ('foc_speed_soure', ENUM, 1),
        ('foc_short_ls_on_zero_duty', BOOL, 1),
        ('sp_pid_loop_rate', ENUM, 1),
        ('s_pid_kp', DOUBLE_32_AUTO, 1e6),
        ('s_pid_ki', DOUBLE_32_AUTO, 1e6),
        ('s_pid_kd', DOUBLE_32_AUTO, 1e6),
        ('s_pid_kd_filter', DOUBLE_16, 1e4), # Added for 6.2 support
        ('s_pid_min_erpm', DOUBLE_32_AUTO, 1000),
        ('s_pid_allow_braking', BOOL, 1),
        ('s_pid_ramp_erpms_s', DOUBLE_32_AUTO, 1000), # Added for 6.2 support
        ('s_pid_speed_source', ENUM, 1), # Added for 6.2 support
        ('p_pid_kp', DOUBLE_32_AUTO, 1e6),
        ('p_pid_ki', DOUBLE_32_AUTO, 1e6),
        ('p_pid_kd', DOUBLE_32_AUTO, 1e6),
        ('p_pid_kd_proc', DOUBLE_32_AUTO, 1e6), # Added for 6.2 support
        ('p_pid_kd_filter', DOUBLE_16, 1e4), # Added for 6.2 support
        ('p_pid_ang_div', DOUBLE_32_AUTO, 1e5),
        ('p_pid_gain_dec_angle', DOUBLE_16, 10), # Added for 6.2 support
        ('p_pid_offset', DOUBLE_32_AUTO, 1e6), # Added for 6.2 support
        ('cc_startup_boost_duty', DOUBLE_16, 1e4),
        ('cc_min_current', DOUBLE_32_AUTO, 1000),
        ('cc_gain', DOUBLE_32_AUTO, 1e6),
        ('cc_ramp_step_max', DOUBLE_16, 1e4),
        ('m_fault_stop_time_ms', INT_32, 1),
        ('m_duty_ramp_step', DOUBLE_16, 1e4),
        ('m_current_backoff_gain', DOUBLE_32_AUTO, 1e6),
        ('m_encoder_counts', UINT_32, 1),
        ('m_encoder_sin_amp', DOUBLE_16, 1000), # Added for 6.2 support
        ('m_encoder_cos_amp', DOUBLE_16, 1000), # Added for 6.2 support
        ('m_encoder_sin_offset', DOUBLE_16, 1000), # Added for 6.2 support
        ('m_encoder_cos_offset', DOUBLE_16, 1000), # Added for 6.2 support
        ('m_encoder_sincos_filter_constant', DOUBLE_16, 1000), # Added for 6.2 support
        ('m_encoder_sincos_phase_correction', DOUBLE_16, 1000), # Added for 6.2 support
        ('m_sensor_port_mode', ENUM, 1),
        ('m_invert_direction', BOOL, 1),
        ('m_drv8301_oc_mode', ENUM, 1),
        ('m_drv8301_oc_adj', UINT_8, 1),
        ('m_bldc_f_sw_min', DOUBLE_32_AUTO, 1),
        ('m_bldc_f_sw_max', DOUBLE_32_AUTO, 1),
        ('m_dc_f_sw', DOUBLE_32_AUTO, 1),
        ('m_ntc_motor_beta', DOUBLE_32_AUTO, 1),
        ('m_out_aux_mode', ENUM, 1), # Followling lines were added to support 6.2
        ('m_motor_temp_sens_type', ENUM, 1),
        ('m_ptc_motor_coeff', DOUBLE_32_AUTO, 1),
        ('m_ntcx_ptcx_res', DOUBLE_16, 1),
        ('m_ntcx_ptcx_temp_base', DOUBLE_16, 1),
        ('m_hall_extra_samples', UINT_8, 1),
        ('m_batt_filter_const', UINT_8, 1),
        ('si_motor_poles', UINT_8, 1),
        ('si_gear_ratio', DOUBLE_32_AUTO, 1),
        ('si_wheel_diameter', DOUBLE_32_AUTO, 1),
        ('si_battery_type', ENUM, 1),
        ('si_battery_cells', 'B', 1),
        ('si_battery_ah', DOUBLE_32_AUTO, 1),
        ('si_motor_nl_current', DOUBLE_32_AUTO, 1),
        ('bms.type', ENUM, 1),
        ('bms.limit_mode', BITFIELD, 1),
        ('bms.t_limit_start', UINT_8, 1),
        ('bms.t_limit_end', UINT_8, 1),
        ('bms.soc_limit_start', DOUBLE_16, 1000),
        ('bms.soc_limit_end', DOUBLE_16, 1000),
        ('bms.vmin_limit_start', DOUBLE_16, 1000),
        ('bms.vmin_limit_end', DOUBLE_16, 1000),
        ('bms.vmax_limit_start', DOUBLE_16, 1000),
        ('bms.vmax_limit_end', DOUBLE_16, 1),
        ('bms.fwd_can_mode', ENUM, 1),
    ]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        for field in self.fields:
            setattr(self, field[0], kwargs.get(field[0], 0))  # Default to 0 if not passed

    def to_dict(self):
        """Returns a dictionary with all configuration field values."""
        config = {}
        for field in self.fields:
            field_name = field[0]
            config[field_name] = getattr(self, field_name, None)
        return config
    
    def param_list(self):
        """
        Returns a list of values for each field in the order of self.fields.
        """
        return [getattr(self, field[0], None) for field in self.fields]
    
    def __str__(self):
        output = "GetMcConf:\n"
        for field in self.fields:
            field_name = field[0]
            value = getattr(self, field_name, None)
            output += f"  {field_name}: {value}\n"
        return output

class SetMcConf(metaclass=pyvesc.VESCMessage):
    id = COMM_SET_MCCONF
    can_id = None
    
    '''
     Fields syntax: (name, data type, scaling factor)
     The fields are taken from a fork implementing MCCONF commands (Supports fw 3.33)
     but also compared to fields from vedders bldc/datatypes.h
     in mc_configuration struct, and some fields added for fw 6.2 support.
     Lastly the data types and scaling comes from the VESC Parameter Editor
    '''
    # The fields must match the getter exactly!
    fields = [
        ('pwm_mode', ENUM),
        ('comm_mode', ENUM),
        ('motor_type', ENUM),
        ('sensor_mode', ENUM),
        ('l_current_max', DOUBLE_32_AUTO),
        ('l_current_min', DOUBLE_32_AUTO),
        ('l_in_current_max', DOUBLE_32_AUTO),
        ('l_in_current_min', DOUBLE_32_AUTO),
        ('l_in_current_map_start', DOUBLE_16),  # Added for 6.2 support
        ('l_in_current_map_filter', DOUBLE_16),  # Added for 6.2 support
        ('l_abs_current_max', DOUBLE_32_AUTO),
        ('l_min_erpm', DOUBLE_32_AUTO),
        ('l_max_erpm', DOUBLE_32_AUTO),
        ('l_erpm_start', DOUBLE_16),
        ('l_max_erpm_fbrake', DOUBLE_32_AUTO),
        ('l_max_erpm_fbrake_cc', DOUBLE_32_AUTO),
        ('l_min_vin', DOUBLE_16),
        ('l_max_vin', DOUBLE_16),
        ('l_battery_cut_start', DOUBLE_16),
        ('l_battery_cut_end', DOUBLE_16),
        ('l_battery_regen_cut_start', DOUBLE_16),  # Added for 6.2 support
        ('l_battery_regen_cut_end', DOUBLE_16),      # Added for 6.2 support
        ('l_slow_abs_current', BOOL),
        ('l_temp_fet_start', UINT_8),
        ('l_temp_fet_end', UINT_8),
        ('l_temp_motor_start', UINT_8),
        ('l_temp_motor_end', UINT_8),
        ('l_temp_accel_dec', DOUBLE_16),
        ('l_min_duty', DOUBLE_16),
        ('l_max_duty', DOUBLE_16),
        ('l_watt_max', DOUBLE_32_AUTO),
        ('l_watt_min', DOUBLE_32_AUTO),
        ('l_current_max_scale', DOUBLE_16),  # Added for 6.2 support
        ('l_current_min_scale', DOUBLE_16),  # Added for 6.2 support
        ('l_duty_start', DOUBLE_16),         # Added for 6.2 support
        ('sl_min_erpm', DOUBLE_32_AUTO),
        ('sl_min_erpm_cycle_int_limit', DOUBLE_32_AUTO),
        ('sl_max_fullbreak_current_dir_change', DOUBLE_32_AUTO),
        ('sl_cycle_int_limit', DOUBLE_16),
        ('sl_phase_advance_at_br', DOUBLE_16),
        ('sl_cycle_int_rpm_br', DOUBLE_32_AUTO),
        ('sl_bemf_coupling_k', DOUBLE_32_AUTO),
        ('hall_table__0', INT_8),
        ('hall_table__1', INT_8),
        ('hall_table__2', INT_8),
        ('hall_table__3', INT_8),
        ('hall_table__4', INT_8),
        ('hall_table__5', INT_8),
        ('hall_table__6', INT_8),
        ('hall_table__7', INT_8),
        ('hall_sl_erpm', DOUBLE_32_AUTO),
        ('foc_current_kp', DOUBLE_32_AUTO),
        ('foc_current_ki', DOUBLE_32_AUTO),
        ('foc_f_zv', DOUBLE_32_AUTO),  # Changed name from 'foc_f_sw' to support 6.2
        ('foc_dt_us', DOUBLE_32_AUTO),
        ('foc_encoder_inverted', BOOL),
        ('foc_encoder_offset', DOUBLE_32_AUTO),
        ('foc_encoder_ratio', DOUBLE_32_AUTO),
        ('foc_sensor_mode', ENUM),
        ('foc_pll_kp', DOUBLE_32_AUTO),
        ('foc_pll_ki', DOUBLE_32_AUTO),
        ('foc_motor_l', DOUBLE_32_AUTO),
        ('foc_ld_lq_diff', DOUBLE_32_AUTO),  # Added for 6.2 support
        ('foc_motor_r', DOUBLE_32_AUTO),
        ('foc_motor_flux_linkage', DOUBLE_32_AUTO),
        ('foc_observer_gain', DOUBLE_32_AUTO),
        ('foc_observer_gain_slow', DOUBLE_32_AUTO),
        ('foc_observer_offset', DOUBLE_16),  # Added for 6.2 support
        ('foc_duty_dowmramp_kp', DOUBLE_32_AUTO),
        ('foc_duty_dowmramp_ki', DOUBLE_32_AUTO),
        ('foc_start_curr_dec', DOUBLE_16),   # Added for 6.2 support
        ('foc_start_curr_dec_rpm', DOUBLE_32_AUTO),  # Added for 6.2 support
        ('foc_openloop_rpm', DOUBLE_32_AUTO),
        ('foc_openloop_rpm_low', DOUBLE_16),  # Added for 6.2 support
        ('foc_d_gain_scale_start', DOUBLE_16),  # Added for 6.2 support
        ('foc_d_gain_scale_max_mod', DOUBLE_16),  # Added for 6.2 support
        ('foc_sl_openloop_hyst', DOUBLE_16),
        ('foc_sl_openloop_time_lock', DOUBLE_16),  # Added for 6.2 support
        ('foc_sl_openloop_time_ramp', DOUBLE_16),  # Added for 6.2 support
        ('foc_sl_openloop_time', DOUBLE_16),
        ('foc_sl_openloop_boost_q', DOUBLE_16),  # Added for 6.2 support
        ('foc_sl_openloop__max_q', DOUBLE_16),  # Added for 6.2 support
        ('foc_hall_table__0', UINT_8),
        ('foc_hall_table__1', UINT_8),
        ('foc_hall_table__2', UINT_8),
        ('foc_hall_table__3', UINT_8),
        ('foc_hall_table__4', UINT_8),
        ('foc_hall_table__5', UINT_8),
        ('foc_hall_table__6', UINT_8),
        ('foc_hall_table__7', UINT_8),
        ('foc_hall_interp_erpm', DOUBLE_32_AUTO),  # Added for 6.2 support
        ('foc_sl_erpm_start', DOUBLE_32_AUTO),  # Added for 6.2 support
        ('foc_sl_erpm', DOUBLE_32_AUTO),
        ('foc_control_sample_mode', ENUM),  # Added for 6.2 support
        ('foc_current_sample_mode', ENUM),  # Added for 6.2 support
        ('foc_sat_comp_mode', ENUM),  # Added for 6.2 support
        ('foc_sat_comp', DOUBLE_16),
        ('foc_temp_comp', BOOL),
        ('foc_temp_comp_base_temp', DOUBLE_16),
        ('foc_current_filter_const', DOUBLE_16),  # Added for 6.2 support
        ('foc_cc_decoupling', ENUM),  # Added for 6.2 support (Unsure of data type)
        ('foc_observer_type', ENUM),  # Added for 6.2 support (Data type ?)
        ('foc_hfi_voltage_start', DOUBLE_16),
        ('foc_hfi_voltage_run', DOUBLE_16),
        ('foc_hfi_voltage_max', DOUBLE_16),
        ('foc_hfi_gain', DOUBLE_16),
        ('foc_hfi_max_err', DOUBLE_16),
        ('foc_hfi_hyst', DOUBLE_16),
        ('foc_sl_erpm_hfi', DOUBLE_32_AUTO),
        ('foc_hfi_start_samples', UINT_16),
        ('foc_hfi_obs_ovr_sec', DOUBLE_32_AUTO),
        ('foc_hfi_samples', ENUM),
        ('foc_offsets_cal_on_boot', BOOL),
        ('foc_offsets_current__0', DOUBLE_32_AUTO),
        ('foc_offsets_current__1', DOUBLE_32_AUTO),
        ('foc_offsets_current__2', DOUBLE_32_AUTO),
        ('foc_offsets_voltage__0', DOUBLE_16),
        ('foc_offsets_voltage__1', DOUBLE_16),
        ('foc_offsets_voltage__2', DOUBLE_16),
        ('foc_offsets_voltage_undriven_0', DOUBLE_16),
        ('foc_offsets_voltage_undriven_1', DOUBLE_16),
        ('foc_offsets_voltage_undriven_2', DOUBLE_16),
        ('foc_phase_filter_enable', BOOL),
        ('foc_phase_filter_disable_fault', BOOL),
        ('foc_phase_filter_max_erpm', DOUBLE_32_AUTO),
        ('foc_mtpa_mode', ENUM),
        ('foc_fw_current_max', DOUBLE_32_AUTO),
        ('foc_fw_duty_start', DOUBLE_16),
        ('foc_fw_q_current_factor', DOUBLE_16),
        ('foc_fw_ramp_time', DOUBLE_16),
        ('foc_speed_soure', ENUM),
        ('foc_short_ls_on_zero_duty', BOOL),
        ('sp_pid_loop_rate', ENUM),
        ('s_pid_kp', DOUBLE_32_AUTO),
        ('s_pid_ki', DOUBLE_32_AUTO),
        ('s_pid_kd', DOUBLE_32_AUTO),
        ('s_pid_kd_filter', DOUBLE_16),
        ('s_pid_min_erpm', DOUBLE_32_AUTO),
        ('s_pid_allow_braking', BOOL),
        ('s_pid_ramp_erpms_s', DOUBLE_32_AUTO),
        ('s_pid_speed_source', ENUM),
        ('p_pid_kp', DOUBLE_32_AUTO),
        ('p_pid_ki', DOUBLE_32_AUTO),
        ('p_pid_kd', DOUBLE_32_AUTO),
        ('p_pid_kd_proc', DOUBLE_32_AUTO),
        ('p_pid_kd_filter', DOUBLE_16),
        ('p_pid_ang_div', DOUBLE_32_AUTO),
        ('p_pid_gain_dec_angle', DOUBLE_16),
        ('p_pid_offset', DOUBLE_32_AUTO),
        ('cc_startup_boost_duty', DOUBLE_16),
        ('cc_min_current', DOUBLE_32_AUTO),
        ('cc_gain', DOUBLE_32_AUTO),
        ('cc_ramp_step_max', DOUBLE_16),
        ('m_fault_stop_time_ms', INT_32),
        ('m_duty_ramp_step', DOUBLE_16),
        ('m_current_backoff_gain', DOUBLE_32_AUTO),
        ('m_encoder_counts', UINT_32),
        ('m_encoder_sin_amp', DOUBLE_16),
        ('m_encoder_cos_amp', DOUBLE_16),
        ('m_encoder_sin_offset', DOUBLE_16),
        ('m_encoder_cos_offset', DOUBLE_16),
        ('m_encoder_sincos_filter_constant', DOUBLE_16),
        ('m_encoder_sincos_phase_correction', DOUBLE_16),
        ('m_sensor_port_mode', ENUM),
        ('m_invert_direction', BOOL),
        ('m_drv8301_oc_mode', ENUM),
        ('m_drv8301_oc_adj', UINT_8),
        ('m_bldc_f_sw_min', DOUBLE_32_AUTO),
        ('m_bldc_f_sw_max', DOUBLE_32_AUTO),
        ('m_dc_f_sw', DOUBLE_32_AUTO),
        ('m_ntc_motor_beta', DOUBLE_32_AUTO),
        ('m_out_aux_mode', ENUM),
        ('m_motor_temp_sens_type', ENUM),
        ('m_ptc_motor_coeff', DOUBLE_32_AUTO),
        ('m_ntcx_ptcx_res', DOUBLE_16),
        ('m_ntcx_ptcx_temp_base', DOUBLE_16),
        ('m_hall_extra_samples', UINT_8),
        ('m_batt_filter_const', UINT_8),
        ('si_motor_poles', UINT_8),
        ('si_gear_ratio', DOUBLE_32_AUTO),
        ('si_wheel_diameter', DOUBLE_32_AUTO),
        ('si_battery_type', ENUM),
        ('si_battery_cells', 'B'),
        ('si_battery_ah', DOUBLE_32_AUTO),
        ('si_motor_nl_current', DOUBLE_32_AUTO),
        ('bms.type', ENUM),
        ('bms.limit_mode', BITFIELD),
        ('bms.t_limit_start', UINT_8),
        ('bms.t_limit_end', UINT_8),
        ('bms.soc_limit_start', DOUBLE_16),
        ('bms.soc_limit_end', DOUBLE_16),
        ('bms.vmin_limit_start', DOUBLE_16),
        ('bms.vmin_limit_end', DOUBLE_16),
        ('bms.vmax_limit_start', DOUBLE_16),
        ('bms.vmax_limit_end', DOUBLE_16),
        ('bms.fwd_can_mode', ENUM),
    ]

    def __init__(self, *args, **kwargs):
        raise NotImplementedError
        super().__init__(*args, **kwargs)
        for field in self.fields:
            setattr(self, field[0], kwargs.get(field[0], 0))

    @classmethod
    def from_params(cls, params):
        raise NotImplementedError
        # Create an instance without relying on a custom keyword.
        obj = cls()
        if len(params) != len(cls.fields):
            raise ValueError("Length of params list does not match expected fields length.")
        for index, field in enumerate(cls.fields):
            field_name, field_type = field[0], field[1]
            val = params[index]
            # For 'f' (DOUBLE_32_AUTO) leave as float.
            setattr(obj, field_name, val)
        return obj
    
    def __str__(self):
        output = "SetMcConf:\n"
        for field in self.fields:
            field_name = field[0]
            output += f"  {field_name}: {getattr(self, field_name)}\n"
        return output


def get_mc_conf():
    with serial.Serial(VESC_port, baudrate=115200, timeout=20) as ser:
        print(f"Connected to VESC on {VESC_port}")

        conf_msg = GetMcConf()
        try:
            msg = pyvesc.encode(conf_msg)
            ser.write(msg)
            print(f"Message encoded and written")
            decoded_response = read_full_packet(ser)
            return(decoded_response)
        except Exception as e:
            print(f"Encoding error: {e}")


def setter_test(observer_gain):
    with serial.Serial(VESC_port, baudrate=115200, timeout=20) as ser:
        config = get_mc_conf()

        if config is None:
            print("No configuration received. Aborting setter test.")
            return
        
        print(config)
        params = config.param_list()
        # Always use the alternative constructor:
        my_msg = SetMcConf.from_params(params)
        print("\nNew SetMcConf (after override):")
        print(my_msg)
        '''
        try:
            msg = pyvesc.encode(my_msg)
            ser.write(msg)
            print("Setter message written to VESC")
        except Exception as e:
            print(f"Encoding error from setter: {e}")
        '''
        upt_conf = get_mc_conf()
        print(f"Conf after setter: {upt_conf}")
        


if __name__ == "__main__":
    setter_test(observer_gain=30)

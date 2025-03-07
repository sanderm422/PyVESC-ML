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

# Message ids
COMM_GET_MCCONF = VedderCmd.COMM_GET_MCCONF # 14
COMM_SET_MCCONF = VedderCmd.COMM_SET_MCCONF

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
    fields = [
        ('pwm_mode', 'B', 1),
        ('comm_mode', 'B', 1),
        ('motor_type', 'B', 1),
        ('sensor_mode', 'B', 1),
        ('l_current_max', 'f', 1000),
        ('l_current_min', 'f', 1000),
        ('l_in_current_max', 'f', 1000),
        ('l_in_current_min', 'f', 1000),
        ('l_in_current_map_start', 'h', 1e4), # Added for 6.2 support
        ('l_in_current_map_filter', 'h', 1e4), # Added for 6.2 support
        ('l_abs_current_max', 'f', 1000),
        ('l_min_erpm', 'f', 1000),
        ('l_max_erpm', 'f', 1000),
        ('l_erpm_start', 'h', 1e4),
        ('l_max_erpm_fbrake', 'f', 1000),
        ('l_max_erpm_fbrake_cc', 'f', 1000),
        ('l_min_vin', 'h', 10),
        ('l_max_vin', 'h', 10),
        ('l_battery_cut_start', 'h', 10),
        ('l_battery_cut_end', 'h', 10),
        ('l_battery_regen_cut_start', 'h', 10), # Added for 6.2 support
        ('l_battery_regen_cut_end', 'h', 10), # Added for 6.2 support
        ('l_slow_abs_current', 'B', 1), # Boolean
        ('l_temp_fet_start', 'B', 1), 
        ('l_temp_fet_end', 'B', 1),
        ('l_temp_motor_start', 'B', 1),
        ('l_temp_motor_end', 'B', 1),
        ('l_temp_accel_dec', 'h', 1e4),
        ('l_min_duty', 'h', 1e4),
        ('l_max_duty', 'h', 1e4),
        ('l_watt_max', 'f', 1),
        ('l_watt_min', 'f', 1),
        ('l_current_max_scale', 'h', 1e4), # Added for 6.2 support
        ('l_current_min_scale', 'h', 1e4), # Added for 6.2 support
        ('l_duty_start', 'h', 1e4), # Added for 6.2 support
        ('sl_min_erpm', 'f', 1000),
        ('sl_min_erpm_cycle_int_limit', 'f', 1000),
        ('sl_max_fullbreak_current_dir_change', 'f', 1000),
        ('sl_cycle_int_limit', 'h', 10),
        ('sl_phase_advance_at_br', 'h', 1e4),
        ('sl_cycle_int_rpm_br', 'f', 1000),
        ('sl_bemf_coupling_k', 'f', 1000),
        ('hall_table__0', 'b', 1), 
        ('hall_table__1', 'b', 1),
        ('hall_table__2', 'b', 1),
        ('hall_table__3', 'b', 1),
        ('hall_table__4', 'b', 1),
        ('hall_table__5', 'b', 1),
        ('hall_table__6', 'b', 1),
        ('hall_table__7', 'b', 1),
        ('hall_sl_erpm', 'f', 1000),
        ('foc_current_kp', 'f', 1e5),
        ('foc_current_ki', 'f', 1e5),
        ('foc_f_zv', 'f', 1000), # Changed name from 'foc_f_sw' to support 6.2
        ('foc_dt_us', 'f', 1e6),
        ('foc_encoder_inverted', 'B', 1), # Boolean
        ('foc_encoder_offset', 'f', 1000),
        ('foc_encoder_ratio', 'f', 1000),
        ('foc_sensor_mode', 'B', 1),
        ('foc_pll_kp', 'f', 1000),
        ('foc_pll_ki', 'f', 1000),
        ('foc_motor_l', 'f', 1e8),
        ('foc_ld_lq_diff', 'f', 1e8), # Added for 6.2 support
        ('foc_motor_r', 'f', 1e5),
        ('foc_motor_flux_linkage', 'f', 1e5),
        ('foc_observer_gain', 'f', 1e-2),
        ('foc_observer_gain_slow', 'f', 1e-2),
        ('foc_observer_offset', 'h', 1000), # Added for 6.2 support
        ('foc_duty_dowmramp_kp', 'f', 1000),
        ('foc_duty_dowmramp_ki', 'f', 1000),
        ('foc_start_curr_dec', 'h', 1e4), # Added for 6.2 support
        ('foc_start_curr_dec_rpm', 'f', 1000), # Added for 6.2 support
        ('foc_openloop_rpm', 'f', 1000),
        ('foc_openloop_rpm_low', 'h', 1000), # Added for 6.2 support
        ('foc_d_gain_scale_start', 'h', 1000), # Added for 6.2 support
        ('foc_d_gain_scale_max_mod', 'h', 1000), # Added for 6.2 support
        ('foc_sl_openloop_hyst', 'h', 100),
        ('foc_sl_openloop_time_lock', 'h', 100), # Added for 6.2 support
        ('foc_sl_openloop_time_ramp', 'h', 100), # Added for 6.2 support
        ('foc_sl_openloop_time', 'h', 100),
        ('foc_sl_openloop_boost_q', 'h', 100), # Added for 6.2 support
        ('foc_sl_openloop__max_q', 'h', 100), # Added for 6.2 support
        ('foc_hall_table__0', 'B', 1),
        ('foc_hall_table__1', 'B', 1),
        ('foc_hall_table__2', 'B', 1),
        ('foc_hall_table__3', 'B', 1),
        ('foc_hall_table__4', 'B', 1),
        ('foc_hall_table__5', 'B', 1),
        ('foc_hall_table__6', 'B', 1),
        ('foc_hall_table__7', 'B', 1),
        ('foc_hall_interp_erpm', 'f', 1), # Added for 6.2 support
        ('foc_sl_erpm_start', 'f', 1000), # Added for 6.2 support
        ('foc_sl_erpm', 'f', 1000),
        ('foc_control_sample_mode', 'B', 1), # Added for 6.2 support
        ('foc_current_sample_mode', 'B', 1), # Added for 6.2 support
        ('foc_sat_comp_mode', 'B', 1), # Added for 6.2 support
        ('foc_sat_comp', 'h', 1000),
        ('foc_temp_comp', 'B', 1),
        ('foc_temp_comp_base_temp', 'h', 100),
        ('foc_current_filter_const', 'h', 1e4), # Added for 6.2 support
        ('foc_cc_decoupling', 'B', 1),  # Added for 6.2 support (Unsure of data type)
        ('foc_observer_type', 'B', 1),  # Added for 6.2 support (Data type ?)
        ('foc_hfi_voltage_start', 'h', 10),
        ('foc_hfi_voltage_run', 'h', 10),
        ('foc_hfi_voltage_max', 'h', 10),
        ('foc_hfi_gain', 'h', 1000),
        ('foc_hfi_max_err', 'h', 1000),
        ('foc_hfi_hyst', 'h', 100),
        ('foc_sl_erpm_hfi', 'f', 1000),
        ('foc_hfi_start_samples', 'H', 1),
        ('foc_hfi_obs_ovr_sec', 'f', 1000),
        ('foc_hfi_samples', 'B', 1),
        ('foc_offsets_cal_on_boot', 'B', 1),
        ('foc_offsets_current__0', 'f', 1),
        ('foc_offsets_current__1', 'f', 1),
        ('foc_offsets_current__2', 'f', 1),
        ('foc_offsets_voltage__0', 'h', 1e4),
        ('foc_offsets_voltage__1', 'h', 1e4),
        ('foc_offsets_voltage__2', 'h', 1e4),
        ('foc_offsets_voltage_undriven_0', 'h', 1e4),
        ('foc_offsets_voltage_undriven_1', 'h', 1e4),
        ('foc_offsets_voltage_undriven_2', 'h', 1e4),
        ('foc_phase_filter_enable', 'B', 1),
        ('foc_phase_filter_disable_fault', 'B', 1),
        ('foc_phase_filter_max_erpm', 'f', 1e4),
        ('foc_mtpa_mode', 'B', 1),
        ('foc_fw_current_max', 'f', 1000),
        ('foc_fw_duty_start', 'h', 1e4),
        ('foc_fw_q_current_factor', 'h', 1e4),
        ('foc_fw_ramp_time', 'h', 1000),
        ('foc_speed_soure', 'B', 1),
        ('foc_short_ls_on_zero_duty', 'B', 1),
        ('sp_pid_loop_rate', 'B', 1),
        ('s_pid_kp', 'f', 1e6),
        ('s_pid_ki', 'f', 1e6),
        ('s_pid_kd', 'f', 1e6),
        ('s_pid_kd_filter', 'h', 1e4), # Added for 6.2 support
        ('s_pid_min_erpm', 'f', 1000),
        ('s_pid_allow_braking', 'B', 1),
        ('s_pid_ramp_erpms_s', 'f', 1000), # Added for 6.2 support
        ('s_pid_speed_source', 'B', 1), # Added for 6.2 support
        ('p_pid_kp', 'f', 1e6),
        ('p_pid_ki', 'f', 1e6),
        ('p_pid_kd', 'f', 1e6),
        ('p_pid_kd_proc', 'f', 1e6), # Added for 6.2 support
        ('p_pid_kd_filter', 'h', 1e4), # Added for 6.2 support
        ('p_pid_ang_div', 'f', 1e5),
        ('p_pid_gain_dec_angle', 'h', 10), # Added for 6.2 support
        ('p_pid_offset', 'f', 1e6), # Added for 6.2 support
        ('cc_startup_boost_duty', 'h', 1e4),
        ('cc_min_current', 'f', 1000),
        ('cc_gain', 'f', 1e6),
        ('cc_ramp_step_max', 'h', 1e4),
        ('m_fault_stop_time_ms', 'i', 1),
        ('m_duty_ramp_step', 'h', 1e4),
        ('m_current_backoff_gain', 'f', 1e6),
        ('m_encoder_counts', 'I', 1),
        ('m_encoder_sin_amp', 'h', 1000), # Added for 6.2 support
        ('m_encoder_cos_amp', 'h', 1000), # Added for 6.2 support
        ('m_encoder_sin_offset', 'h', 1000), # Added for 6.2 support
        ('m_encoder_cos_offset', 'h', 1000), # Added for 6.2 support
        ('m_encoder_sincos_filter_constant', 'h', 1000), # Added for 6.2 support
        ('m_encoder_sincos_phase_correction', 'h', 1000), # Added for 6.2 support
        ('m_sensor_port_mode', 'B', 1),
        ('m_invert_direction', 'B', 1),
        ('m_drv8301_oc_mode', 'B', 1),
        ('m_drv8301_oc_adj', 'B', 1),
        ('m_bldc_f_sw_min', 'f', 1),
        ('m_bldc_f_sw_max', 'f', 1),
        ('m_dc_f_sw', 'f', 1),
        ('m_ntc_motor_beta', 'f', 1),
        ('m_out_aux_mode', 'B', 1), # Followling lines were added to support 6.2
        ('m_motor_temp_sens_type', 'B', 1),
        ('m_ptc_motor_coeff', 'f', 1),
        ('m_ntcx_ptcx_res', 'h', 1e-1),
        ('m_ntcx_ptcx_temp_base', 'h', 10),
        ('m_hall_extra_samples', 'B', 1),
        ('m_batt_filter_const', 'B', 1),
        ('si_motor_poles', 'B', 1),
        ('si_gear_ratio', 'f', 1),
        ('si_wheel_diameter', 'f', 1),
        ('si_battery_type', 'B', 1),
        ('si_battery_cells', 'B', 1),
        ('si_battery_ah', 'f', 1),
        ('si_motor_nl_current', 'f', 1),
        ('bms.type', 'B', 1),
        ('bms.limit_mode', 'B', 1),
        ('bms.t_limit_start', 'B', 1),
        ('bms.t_limit_end', 'B', 1),
        ('bms.soc_limit_start', 'h', 1000),
        ('bms.soc_limit_end', 'h', 1000),
        ('bms.vmin_limit_start', 'h', 1000),
        ('bms.vmin_limit_end', 'h', 1000),
        ('bms.vmax_limit_start', 'h', 1000),
        ('bms.vmax_limit_end', 'h', 1),
        ('bms.fwd_can_mode', 'B', 1),
    ]
    

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        for field in self.fields:
            setattr(self, field[0], kwargs.get(field[0], 0))  # Default to 0 if not passed

def extract_ascii_from_response(response):
    """ Extracts printable ASCII characters from a raw response. """
    return ''.join(chr(b) for b in response if 32 <= b <= 126)  # Only printable ASCII

def get_mc_conf():
    with serial.Serial(VESC_port, baudrate=115200, timeout=20) as ser:
        print(f"Connected to VESC on {VESC_port}")

        conf_msg = GetMcConf()
        try:
            msg = pyvesc.encode(conf_msg)
            ser.write(msg)
            print(f"Message encoded and written")

            # Read dynamically
            data = ser.read(ser.in_waiting or 200)
            print(f"Received {len(data)} bytes: {data.hex()}")

            # Extract ASCII for debugging
            ascii_from_hex = extract_ascii_from_response(data)
            print(f"ascii extracted from hex: {ascii_from_hex}")

            print(f"Manually extracted data: \n{data[0]},\n{data[1]},\n{data[2]},\n{data[3]},\n{data[4]},\n{data[5]},\n{data[6]},\n{data[7]},\n{data[8]},\n{data[9]}")
            print(f"{data[10]},\n{data[11]},\n{data[12]},\n{data[13]},\n{data[14]},\n{data[15]},\n{data[16]},\n{data[17]},\n{data[18]},\n{data[19]}")
            print(f"{data[20]},\n{data[21]},\n{data[22]},\n{data[23]},\n{data[24]},\n{data[25]},\n{data[26]},\n{data[27]},\n{data[28]},\n{data[29]}")


            # Check start byte
            if data and data[0] not in [0x02, 0x03]:
                print(f"Warning: Unexpected start byte: {data[0]:02X}")

            # Try decoding
            msg, consumed = pyvesc.decode(data)
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                msg, consummed = pyvesc.decode(data)
            
            if msg is None:
                print("Decoding failed, msg is None.")

            else:
                print("Message decoded successfully!")
                print(msg)
                for field_name, _, _ in msg.fields:
                    print(f"{field_name}: {getattr(msg, field_name)}")

        except Exception as e:
            print(f"Encoding error: {e}")
        
        

if __name__ == "__main__":
    get_mc_conf()
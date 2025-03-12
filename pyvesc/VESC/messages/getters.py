from pyvesc.protocol.base import VESCMessage
from pyvesc.VESC.messages import VedderCmd


pre_v3_33_fields = [('temp_mos1', 'h', 10),
                    ('temp_mos2', 'h', 10),
                    ('temp_mos3', 'h', 10),
                    ('temp_mos4', 'h', 10),
                    ('temp_mos5', 'h', 10),
                    ('temp_mos6', 'h', 10),
                    ('temp_pcb',  'h', 10),
                    ('current_motor', 'i', 100),
                    ('current_in',  'i', 100),
                    ('duty_now',    'h', 1000),
                    ('rpm',         'i', 1),
                    ('v_in',        'h', 10),
                    ('amp_hours',   'i', 10000),
                    ('amp_hours_charged', 'i', 10000),
                    ('watt_hours',  'i', 10000),
                    ('watt_hours_charged', 'i', 10000),
                    ('tachometer', 'i', 1),
                    ('tachometer_abs', 'i', 1),
                    ('mc_fault_code', 'c', 0)]


class GetVersion(metaclass=VESCMessage):
    """ Gets version fields
    """
    id = VedderCmd.COMM_FW_VERSION

    fields = [
            ('comm_fw_version', 'b', 0),
            ('fw_version_major', 'b', 0),
            ('fw_version_minor', 'b', 0)
    ]

    def __str__(self):
        return f"{self.comm_fw_version}.{self.fw_version_major}.{self.fw_version_minor}"


class GetValues(metaclass=VESCMessage):
    """ Gets internal sensor data
    """
    id = VedderCmd.COMM_GET_VALUES

    fields = [
        ('temp_fet', 'h', 10),
        ('temp_motor', 'h', 10),
        ('avg_motor_current', 'i', 100),
        ('avg_input_current', 'i', 100),
        ('avg_id', 'i', 100),
        ('avg_iq', 'i', 100),
        ('duty_cycle_now', 'h', 1000),
        ('rpm', 'i', 1),
        ('v_in', 'h', 10),
        ('amp_hours', 'i', 10000),
        ('amp_hours_charged', 'i', 10000),
        ('watt_hours', 'i', 10000),
        ('watt_hours_charged', 'i', 10000),
        ('tachometer', 'i', 1),
        ('tachometer_abs', 'i', 1),
        ('mc_fault_code', 'c', 0),
        ('pid_pos_now', 'i', 1000000),
        ('app_controller_id', 'c', 0),
        ('time_ms', 'i', 1),
    ]


class GetRotorPosition(metaclass=VESCMessage):
    """ Gets rotor position data
    
    Must be set to DISP_POS_MODE_ENCODER or DISP_POS_MODE_PID_POS (Mode 3 or 
    Mode 4). This is set by SetRotorPositionMode (id=21).
    """
    id = VedderCmd.COMM_ROTOR_POSITION

    fields = [
            ('rotor_pos', 'i', 100000)
    ]

'''
The message classes GetMcConf and SetMcConf and defined here.
These classes implement motor controller configuration message setters and getters.
They enable FOC-parameter tuning, e.g tuning observer gain and -offset, pid KI and pid KP 
among alot of other paramter tuning.
'''

# Message ids
COMM_GET_MCCONF = VedderCmd.COMM_GET_MCCONF # 14
# Data types
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

class GetMcConf(metaclass=VESCMessage):
    
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

from pyvesc.protocol.base import VESCMessage
from pyvesc.protocol.interface import encode
from pyvesc.VESC.messages import VedderCmd


class SetDutyCycle(metaclass=VESCMessage):
    """ Set the duty cycle.

    :ivar duty_cycle: Value of duty cycle to be set (range [-1e5, 1e5]).
    """
    id = VedderCmd.COMM_SET_DUTY
    fields = [
        ('duty_cycle', 'i', 100000)
    ]


class SetRPM(metaclass=VESCMessage):
    """ Set the RPM.

    :ivar rpm: Value to set the RPM to.
    """
    id = VedderCmd.COMM_SET_RPM
    fields = [
        ('rpm', 'i')
    ]


class SetCurrent(metaclass=VESCMessage):
    """ Set the current (in milliamps) to the motor.

    :ivar current: Value to set the current to (in milliamps).
    """
    id = VedderCmd.COMM_SET_CURRENT
    fields = [
        ('current', 'i', 1000)
    ]


class SetCurrentBrake(metaclass=VESCMessage):
    """ Set the current brake (in milliamps).

    :ivar current_brake: Value to set the current brake to (in milliamps).
    """
    id = VedderCmd.COMM_SET_CURRENT_BRAKE
    fields = [
        ('current_brake', 'i', 1000)
    ]


class SetPosition(metaclass=VESCMessage):
    """Set the rotor angle based off of an encoder or sensor

    :ivar pos: Value to set the current position or angle to.
    """
    id = VedderCmd.COMM_SET_POS
    fields = [
        ('pos', 'i', 1000000)
    ]


class SetRotorPositionMode(metaclass=VESCMessage):
     """Sets the rotor position feedback mode.

     It is reccomended to use the defined modes as below:
         * DISP_POS_OFF
         * DISP_POS_MODE_ENCODER
         * DISP_POS_MODE_PID_POS
         * DISP_POS_MODE_PID_POS_ERROR

     :ivar pos_mode: Value of the mode
     """

     DISP_POS_OFF = 0
     DISP_POS_MODE_ENCODER = 3
     DISP_POS_MODE_PID_POS = 4
     DISP_POS_MODE_PID_POS_ERROR = 5

     id = VedderCmd.COMM_SET_DETECT
     fields = [
         ('pos_mode', 'b')
     ]


class SetServoPosition(metaclass=VESCMessage):
    """Sets the position of s servo connected to the VESC.

    :ivar servo_pos: Value of position (range [0, 1])
    """

    id = VedderCmd.COMM_SET_SERVO_POS
    fields = [
        ('servo_pos', 'h', 1000)
    ]


class Alive(metaclass=VESCMessage):
    """Heartbeat signal to keep VESC alive"""
    id = VedderCmd.COMM_ALIVE
    fields = []

'''
The message class FocDetectApply implements the commands for initialising
foc-parameters for the VESC
'''

# Defining message ids.
COMM_DETECT_APPLY_ALL = VedderCmd.COMM_DETECT_APPLY_ALL_FOC # = 58
COMM_SET_MCCONF = VedderCmd.COMM_SET_MCCONF 

class FocDetectApply(metaclass=VESCMessage):
    id = COMM_DETECT_APPLY_ALL # foc_detect_apply_all has id = 58
    can_id = None

    # Fields from comm/commands.c line 2271, in vedders bldc repo.
    fields = [
        ('detect_can', 'B'), # CAN is false to ensure UART connection
        ('max_power_loss', 'f'),
        ('min_current', 'f'),
        ('max_current', 'f'),
        ('openloop_rpm', 'f'),
        ('sl_erpm', 'f'),
    ]

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

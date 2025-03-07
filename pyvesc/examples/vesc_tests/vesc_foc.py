import serial
import pyvesc
from pyvesc import VESCMessage
from pyvesc.VESC.messages import VedderCmd

'''
This script aims to test the functionality of VESC commands
that are useful for FOC-tuning. 

These messages have not been predefined, 
demands custom declaration of VESC messages.

**TODO**
Starting with initialising with foc_detect_apply_all,
followed by tuning with motor_control_conf
'''

# Use correct serial port for your PC, VESC-tool can be used for this puropse
VESC_port = "/dev/ttyACM0"

# Defining message ids.
COMM_DETECT_APPLY_ALL = VedderCmd.COMM_DETECT_APPLY_ALL_FOC # = 58

class FocDetectApply(metaclass=pyvesc.VESCMessage):
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

def foc_detect_apply_all(
        detect_can, max_power_loss, min_current, max_current, openloop_rpm, sl_erpm):
    try:
        with serial.Serial(VESC_port, baudrate=115200,timeout=2) as ser:
            print(f"Connected to VESC on {VESC_port}")
            # Create message
            msg = FocDetectApply(detect_can, max_power_loss, min_current, max_current, openloop_rpm, sl_erpm)
            print("Message created")
            # Encode message
            encoded_msg = pyvesc.encode_request(msg)
            print("Successfully encoded")
            # Send message
            ser.write(encoded_msg)
            print("Message sent")
            # Read response
            response = ser.read(100)
            print(f"Raw Response (Hex): {response.hex()}")
            # Decode response
            decoded_response = pyvesc.decode(response)
            print("VESC Response:", decoded_response)

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    
    detect_can = 0
    max_power_loss = 0
    min_current = 0
    max_current = 0
    openloop_rpm = 0
    sl_erpm = 0

    # Testat med:
    # 0, 10.0, 5.0, 10.0, 2000.0, 3000.0
    # 0, 0.0, 0.0, 0.0, 0.0, 0.0
    
    foc_detect_apply_all(detect_can, max_power_loss, min_current, max_current, openloop_rpm, sl_erpm)
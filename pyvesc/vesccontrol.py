import serial
import pyvesc
from pyvesc import VESCMessage
from pyvesc.VESC.messages import VedderCmd
from pyvesc.VESC.messages import GetMcConf

# Use the correct serial port for your PC. E.g using VESC Tool
VESC_Port = "/dev/ttyACM0"

class VESCControl:
    def __init__(self, port=VESC_Port, baudrate=115200, timeout=20):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
    
    def _connect(self):
        return serial.Serial(self.port, baudrate=self.baudrate, timeout=self.timeout)
    
    def _read_packet(self, ser, timeout=2):
        """
        Accumulates bytes from the serial port until a complete VESC packet is available,
        then returns the decoded message.
        """
        buffer = b""
        ser.timeout = timeout
        while True:
            new_data = ser.read(ser.in_waiting or 1)
            if new_data:
                buffer += new_data
                if buffer[0] not in [0x02, 0x03]:
                    buffer = buffer[1:]
                    continue
                try:
                    msg, consumed = pyvesc.decode(buffer)
                    if msg is not None:
                        buffer = buffer[consumed:]
                        return msg
                except Exception as e:
                    print("Decode error:", e)
                    break
            else:
                break
        return None

    def get_mcconf(self, params=None):
        '''
        Returns motor controller configuration parameter settings.
        Params specify which parameters to return, if not specified
        all available parameters are returned
        '''
        with self._connect() as ser:
            print(f"Connected to VESC on {self.port}")
            my_msg = GetMcConf()
            try:
                msg = pyvesc.encode(my_msg)
                ser.write(msg)
                decoded = self._read_packet(ser)
                if decoded:
                    full_conf = {field[0]: getattr(decoded, field[0], None) for field in GetMcConf.fields}
                    if params:
                        # Return only the requested parameters.
                        requested_conf = {}
                        for param in params:
                            if param in full_conf:
                                requested_conf[param] = full_conf[param]
                            else:
                                print(f"Parameter '{param}' not found in configuration.")
                        return requested_conf
                    else:
                        return decoded
                else:
                    print("No response to be decoded")
            except Exception as e:
                print(f"Error in McConf getter: {e}")
        return None
    
    def set_mcconf(self):
        # **TODO**
        return
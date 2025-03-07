import serial
import pyvesc
from pyvesc import VESCMessage
from pyvesc.VESC.messages import GetVersion, GetValues

'''
This script communicates with the VESC using the predefined messages
from the PyVESC library.

It serves as a quick and easy way to verify that 
serial communication with teh VESC is fully operational.
'''



# Use correct serial port for your PC, VESC-tool can be used for this puropse
VESC_port = "/dev/ttyACM0"

def extract_ascii_from_response(response):
    """ Extracts printable ASCII characters from a raw response. """
    return ''.join(chr(b) for b in response if 32 <= b <= 126)  # Only printable ASCII

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

def get_firmware():
    try:
        with serial.Serial(VESC_port, baudrate=115200,timeout=2) as ser:
            print(f"Connected to VESC on {VESC_port}")
            # Create message
            fw_request = GetVersion()
            # Encode message
            encoded_msg = pyvesc.encode_request(fw_request)
            print("Successfully encoded")
            # Send message
            ser.write(encoded_msg)
            print("Message sent")

            # Decode response
            decoded_response = read_full_packet(ser)
            if decoded_response is not None:
                print(f"Decoded message: {decoded_response}")
            else:
                print("Failed to decode a complete packet.")

    except Exception as e:
        print(f"Error: {e}")

def get_values():
    try:
        with serial.Serial(VESC_port, baudrate=115200, timeout=2) as ser:
            print(f"Connected to VESC on {VESC_port}")
            b = b'/01'
            msg = GetValues(0,0,0,0,0,0,0,0,0,0,
                            0,0,0,0,0,0,0,0,0)
            encoded = pyvesc.encode(msg=msg)
            print("Massage Encoded")
            ser.write(encoded)
            decoded_response = read_full_packet(ser)
            print("Packet decoded")
            if decoded_response is not None:
                print(f"Decoded message: {decoded_response}")
            else:
                print("Failed to decode a complete packet.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    get_firmware()
    get_values()

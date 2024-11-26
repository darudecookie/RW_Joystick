import serial
import time
import inputs
import struct

import pylab as plt

from scipy.spatial.transform import Rotation

msg_key = (
    0,

    10,
    11,

    20,

    254,
    255,
)
msg_val = (
    "stop",

    "set_quaternion",
    "get_quaternion",

    "get_rpm",

    "reserved",
    "reserved",
)


class SerialGrapher:
    def __init__(self):

        self.start_char = b'<'
        self.stop_char = b'>'

        port: str = "/dev/ttyACM0"
        baud_rate: int = 115200
        MCU_init_sequence = b"<MCU_init>\n"
        self.sensitivity = 0.5
        
        
        while True:
            try:
                self.Serial_port = serial.Serial(
                    port=port,
                    baudrate=baud_rate,
                )
                print("Opened mcu serial port")
                break
            except serial.SerialException:
                print(
                    f"failed to open serial port to MCU, port: {
                        port}, retrying in 0.5 seconds"
                )
                time.sleep(0.5)

        while True:
            if self.Serial_port.in_waiting > 0:
                read_msg = self.Serial_port.readline()
                print(read_msg)
                if MCU_init_sequence == read_msg:
                    print("Communication Established")
                    break
                else:
                    print(
                        "Failed to establish MCU communication, retrying in 0.5 seconds"
                    )
                    time.sleep(0.5)



    def n_floats_to_bytes(self, input_floats: float) -> str:
        float_data = b''
        for i in range(len(input_floats)):
            float_data += struct.pack('f', input_floats)
        return float_data
    def n_bytes_to_float(self, input_bytes: str) -> float:
        n_floats = int(len(input_bytes)/4)
        floats = []
        for i in range(start=0,stop=n_floats,step=4):
            floats.append(struct.unpack('f', input_bytes[i::i+3]))
        return floats
    
    def key_from_val(self, value: str) -> int:
        return self.msg_key[self.msg_val.index(value)]



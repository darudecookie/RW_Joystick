import serial
import time
import inputs


msg_key = (
    0,
    10,
    11,
    254,
    255,
)
msg_val = (
    "stop",
    "set_quaternion",
    "get_quaternion",
    "reserved",
    "reserved",
)


def decode_1_float(input_bytes: str) -> float:
    output_float = 0
    for i in range(0, 3):
        float_section = float(
            int.from_bytes(input_bytes[i : i + 1], byteorder="little")
        )
        if float_section >= 101:
            float_section -= 101
            float_section = -float_section

        float_section /= 10 ** (i * 2)
        output_float += float_section

    return output_float


def decode_n_floats(input_bytes: str, n_floats: int = 7) -> list[float]:
    if len(input_bytes) / 3 != n_floats:
        return False
    float_array = []
    for i in range(0, n_floats * 3, 3):
        float_array.append(decode_1_float(input_bytes[i : i + 2]))

    return float_array


def key_from_val(value: str) -> int:
    return msg_key[msg_val.index(value)]


def encode_1_float(input_float: float) -> str:
    if -100 <= input_float <= 100:
        float_sections = [
            int(input_float),
            int(input_float * 100) - int(input_float) * 100,
            int(input_float * 100 * 100) - int(input_float * 100) * 100,
        ]

        for i in range(3):
            if float_sections[i] < 0:
                float_sections[i] = abs(float_sections[i]) + 101
        return (
            float_sections[0].to_bytes(1, byteorder="little", signed=False)
            + float_sections[1].to_bytes(1, byteorder="little", signed=False)
            + float_sections[2].to_bytes(1, byteorder="little", signed=False)
        )


def encode_n_floats(
    input_float_array: float,
) -> str:
    output_bytes = rb""

    for input_float in input_float_array:
        output_bytes += encode_1_float(input_float)

    return output_bytes


class SerialReader:
    def __init__(self):
        port: str = "/dev/ttyACM0"
        baud_rate: int = 115200
        MCU_init_sequence = b"<MCU_init>\n"

        self.start_char = 254  #  b'þ'
        self.start_char = self.start_char.to_bytes(
            length=1, byteorder="little", signed=False
        )

        self.stop_char = 255  #   b'ÿ'
        self.stop_char = self.stop_char.to_bytes(
            length=1, byteorder="little", signed=False
        )

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
                    f"failed to open serial port to MCU, port: {port}, retrying in 0.5 seconds"
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

        self.to_send_data =[]

    def read_from_serial(self):
        if self.Serial_port.in_waiting > 0:

            read_msg = self.Serial_port.readline()
            msg_identifier = int.from_bytes(read_msg[0])
            msg_data = read_msg[1::]

            match msg_identifier:
                case key_from_val("get_quaternion"):
                    read_quaternion = decode_n_floats(msg_data, n_floats=4)
                    self.report_data("current quaternion", read_quaternion)
    
    def send_data_to_serial(self):
        while len(self.to_send_data)>0:
            self.Serial_port.write(self.start_char)
            self.Serial_port.write(self.to_send_data.pop(0))
            self.Serial_port.write(self.stop_char)

    def controller_to_quaternion(self, controller_input:float[3])->float[4]:
        
    def read_controller(self)->float[3]:

    def report_data(self, identifier: str, data) -> None:
        print(f"{identifier}: {data[0]}, {data[1]}, {data[2]},{data[3]}")


def main():
    print("start")
    


main()

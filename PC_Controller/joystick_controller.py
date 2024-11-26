import serial
import time
import inputs

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


def key_from_val(value: str) -> int:
    return msg_key[msg_val.index(value)]


def decode_1_float(input_bytes: str) -> float:
    output_float = 0
    for i in range(0, 3):
        float_section = float(
            int.from_bytes(input_bytes[i: i + 1], byteorder="little")
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
        float_array.append(decode_1_float(input_bytes[i: i + 2]))

    return float_array


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
        self.sensitivity = 0.5

        self.joy_values = [0, 0, 0]

        self.rpm_data = []
        self.current_quaternion_data = []
        self.target_quaternion_data = []

        self.paused = False
        self.old_paused = False

        self.start_char = 254  # b'þ'
        self.start_char = self.start_char.to_bytes(
            length=1, byteorder="little", signed=False
        )

        self.stop_char = 255  # b'ÿ'
        self.stop_char = self.stop_char.to_bytes(
            length=1, byteorder="little", signed=False
        )

        plt.ion()
        self.Plot = plt.plot(self.joy_values[0], (180, -180))[0]
        plt.plot(self.joy_values[1], (180, -180))[0]
        plt.plot(self.joy_values[2], (180, -180))[0]

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
        msg = self.start_char

        if self.paused != self.old_paused:
            msg += int(0).to_bytes()
            if self.paused:
                msg += int(1).to_bytes()
            elif not self.paused:
                msg += int(0).to_bytes()

            self.old_paused = self.paused
        else:
            msg += encode_n_floats(self.quaternion_from_joy(self.joy_values))

        msg += self.stop_char
        msg += self.stop_char

        self.Serial_port.write(msg)

        # takes info from peripheral and sets appropriate flags/vars ****BINDINGS ARE HERE******
    def parse_event_from_peripheral(self):
        gamepad_events = inputs.get_gamepad()

        if gamepad_events:
            for gamepad_event in gamepad_events:
                match gamepad_event.ev_type:
                    case "Key":  # button press
                        # thumb trigger/ wpn release is hit (jointhold)
                        if gamepad_event.code == "BTN_WEST" and gamepad_event.state == 1:
                            self.paused = not self.paused
                        # trigger button is hit (toggle position/rotation)
                        elif gamepad_event.code == "BTN_Z" and gamepad_event.state == 1:
                            self.joy_values = [0, 0, 0]

                    case "Absolute":  # joystick
                        # normalizing joy val(0-255) to -1 to 1
                        joy_val = (gamepad_event.state - 128) / 128
                        match gamepad_event.code:
                            # NOTE: I switched the x and y values on the joystick and on the robot
                            case "ABS_Y":
                                self.joy_values[0] += joy_val*self.sensitivity
                            case "ABS_X":
                                self.joy_values[1] += joy_val*self.sensitivity
                            case "ABS_Z":
                                self.joy_values[2] += joy_val*self.sensitivity

    def report_data(self, identifier: str, data) -> None:
        print(f"{identifier}: {data[0]}, {data[1]}, {data[2]},{data[3]}")
        plt.draw()

    def quaternion_from_joy(self, joy_val):
        return Rotation.from_euler('xyz', joy_val, degrees=True).as_quat()


def main():
    print("start")

    SerialReader_Object = SerialReader()

    last_send = 0

    send_interval_millis = 250
    send_interval_millis /= 1000

    while True:
        SerialReader_Object.parse_event_from_peripheral()
        SerialReader_Object.read_from_serial()

        if (time.time()-last_send) > send_interval_millis:
            SerialReader_Object.send_data_to_serial()
            last_send = time.time()

        counter += 1


main()

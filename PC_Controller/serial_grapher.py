import serial
import time
import struct
import atexit

from scipy.spatial.transform import Rotation

import openpyxl

# import matplotlib.pyplot as plt


msg_key = (
    0,

    10,
    11,

    20,
    21,

    30,

    254,
    255,
)
msg_val = (
    "stop",

    "set_quaternion",
    "get_quaternion",

    "set_rpm",
    "get_rpm",

    "get_pwm",

    "reserved",
    "reserved",
)


class DataEntry:
    target_quaternion: list[float] = [0, 0, 0, 0]
    current_quaternion: list[float] = [0, 0, 0, 0]
    target_rpm: list[float] = [0, 0, 0, 0]
    current_rpm: list[float] = [0, 0, 0, 0]
    current_pwm: list[int] = [0, 0, 0, 0]


class SerialGrapher:
    def __init__(self):

        self.start_char = b'<'
        self.stop_char = b'>'
        MCU_init_sequence = b"<MCU_init>\n"

        port: str = "/dev/ttyACM0"
        baud_rate: int = 115200

        self.Workbook_setup = False
        self.spreadsheet_name = ("testlogs/rws_test_"+str(time.ctime())+".xlsx").replace(":",".")
        # don't want to fully rewrite/overwrite the save file every cycle bc laggy
        self.cycles_between_saves = 10
        self.spreadsheet_entry = DataEntry()
        self.current_spreadsheet_row = 2
        self.init_time = time.time()

        while False:
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

        while False:
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

        self.Workbook = openpyxl.Workbook()
        self.excel_sheet=self.Workbook.active

        self.excel_sheet["A1"] = "Time"
        self.excel_sheet["B1"] = "Target Quaternion"
        self.excel_sheet["C1"] = "Current Quaternion"
        self.excel_sheet["D1"] = "Target RPM"
        self.excel_sheet["E1"] = "Current RPM"
        self.excel_sheet["F1"] = "Current PWM"

        self.Workbook_setup = True

        self.counter = 0

    def mainloop(self):
        self.read_from_serial()

        if self.counter % 10 == 0:
            self.save_data_to_spreadsheet()

        if self.counter % 50 == 0:
            self.save_spreadsheet_to_file()

        self.counter += 1

    def read_from_serial(self):
        '''
        if self.Serial_port.in_waiting > 0:

            read_msg = self.Serial_port.readline()
            msg_identifier = int.from_bytes(read_msg[0])
            msg_data = read_msg[1::]

            match msg_identifier:
                case self.key_from_val("get_quaternion"):
                    self.spreadsheet_entry.current_quaternion = str(
                        self.n_bytes_to_float(msg_data))

                case self.key_from_val("get_rpm"):
                    self.spreadsheet_entry.current_rpm = str(
                        self.n_bytes_to_float(msg_data))

                case self.key_from_val("get_pwm"):
                    self.spreadsheet_entry.current_quaternion = str(
                        self.n_bytes_to_float(msg_data))
        '''
        self.spreadsheet_entry.current_quaternion = [0,0,0,1]


    def save_data_to_spreadsheet(self):
        self.excel_sheet["A"+str(self.current_spreadsheet_row)
                         ] = str(time.time()-self.init_time)
        self.excel_sheet["B"+str(self.current_spreadsheet_row)
                         ] = str(self.spreadsheet_entry.target_quaternion)
        self.excel_sheet["C"+str(self.current_spreadsheet_row)
                         ] = str(self.spreadsheet_entry.current_quaternion)
        self.excel_sheet["D"+str(self.current_spreadsheet_row)
                         ] = str(self.spreadsheet_entry.target_rpm)
        self.excel_sheet["E"+str(self.current_spreadsheet_row)
                         ] = str(self.spreadsheet_entry.current_rpm)
        self.excel_sheet["F"+str(self.current_spreadsheet_row)
                         ] = str(self.spreadsheet_entry.current_pwm)

        self.current_spreadsheet_row += 1
        self.spreadsheet_entry = DataEntry()

    def save_spreadsheet_to_file(self):
        if self.Workbook_setup:
            self.Workbook.save(self.spreadsheet_name)

    def n_floats_to_bytes(self, input_floats: float) -> str:
        float_data = b''
        for i in range(len(input_floats)):
            float_data += struct.pack('f', input_floats)
        return float_data

    def n_bytes_to_float(self, input_bytes: str) -> float:
        n_floats = int(len(input_bytes)/4)
        floats = []
        for i in range(start=0, stop=n_floats, step=4):
            floats.append(struct.unpack('f', input_bytes[i::i+3]))
        return floats

    def key_from_val(self, value: str) -> int:
        return self.msg_key[self.msg_val.index(value)]


def main():
    SerialGrapher_Object = SerialGrapher()

    atexit.register(SerialGrapher_Object.save_spreadsheet_to_file)

    while True:
        SerialGrapher_Object.mainloop()


if __name__ == "__main__":
    main()

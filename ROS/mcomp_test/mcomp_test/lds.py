import array
import struct
import sys

import serial

offset = 0.12  # 12 cm offset from the front of the robot


class LFCDLaser:

    def __init__(self, port, baud_rate):
        self.port = port
        self.baud_rate = baud_rate
        self.shutting_down = False
        self.serial = serial.Serial(port, baud_rate)

    def __del__(self):
        self.serial.write(b"e")  # stop motor

    def poll(self):
        """Poll the LiDAR sensor once, returning an array of 360
        points.

        Returns:
            List of 360 points with their position in the array from
            359->0 going clock-wise, and the detected distance. [num,
            dist]. If the distance is 0 then it is outside the range
            of the laser.
        """
        self.serial.write(b"b")  # Start poll
        temp_char = 0
        start_count = 0
        got_scan = False
        raw_bytes = array.array('B', [0] * 2520)
        good_sets = 0
        motor_speed = 0
        rpms = 0
        index = 0
        ret = []
        while not self.shutting_down and not got_scan:
            # Wait until first data sync of frame: 0xFA, 0xA0
            temp_char = ord(self.serial.read())

            if start_count == 0:
                if temp_char == 0xFA:
                    start_count = 1
            elif start_count == 1:
                if temp_char == 0xA0:
                    start_count = 0

                    # Now that entire start sequence has been found, read in the rest of the message
                    got_scan = True
                    raw_bytes[0] = 0xFA
                    raw_bytes[1] = 0xA0

                    for i in range(2, 2520):
                        raw_bytes[i] = ord(self.serial.read())

                    # read data in sets of 6
                    for i in range(0, 2520, 42):
                        if (raw_bytes[i] == 0xFA
                                and raw_bytes[i + 1] == (0xA0 + i // 42)
                                # and CRC check
                            ):
                            good_sets += 1
                            motor_speed += struct.unpack(
                                "<H", raw_bytes[i + 2:i + 4])[0]
                            rpms = struct.unpack(
                                "<H", raw_bytes[i + 2:i + 4])[0] / 10

                            for j in range(i + 4, i + 40, 6):
                                index = 6 * (i // 42) + (j - 4 - i) // 6

                                # Four bytes per reading
                                byte0 = raw_bytes[j]
                                byte1 = raw_bytes[j + 1]
                                byte2 = raw_bytes[j + 2]
                                byte3 = raw_bytes[j + 3]

                                # Remaining bits are the range in mm
                                intensity = (byte1 << 8) + byte0

                                # Last two bytes represent the uncertainty or intensity, might also be pixel area of target...
                                # intensity = (byte3 << 8) + byte2
                                range_ = (byte3 << 8) + byte2
                                ret.append([
                                    359 - index, (range_ / 1000) - offset
                                ])  # converted to meters
                                # print(f"r[{359-index}]={range_/1000},", end='')

                    # time_increment = motor_speed/good_sets/1e8
        self.serial.write(b"e")  # stop the motor after polling
        return ret

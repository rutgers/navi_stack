#!/usr/bin/env python
import serial, time

def write_pace(fp, msg, delay):
    for byte in msg:
        fp.write(byte)
        port.flushOutput()
        time.sleep(delay)

if __name__ == '__main__':
    port = serial.Serial(
        port = '/dev/gps_novatel',
        baudrate = 115200,
        timeout = None
    )

    port.write('UNLOGALL COM1\r\n')
    port.write('UNLOGALL COM2\r\n')
    port.write('UNLOGALL COM3\r\n')
    port.write('ASSIGNLBAND OMNISTAR 1557845 1200\n')
    port.write('UTMZONE SET 10\r\n')
    port.write('LOG BESTUTMA ONTIME 1\r\n')
    port.write('SAVECONFIG\r\n')

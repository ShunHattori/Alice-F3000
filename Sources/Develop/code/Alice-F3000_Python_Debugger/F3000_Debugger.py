# coding: utf-8

import sys
import time
import serial
import serial.tools.list_ports
from serial import SerialException
from enum import IntEnum


def search_com_port():
    coms = serial.tools.list_ports.comports()
    comlist = []
    for scanned_port in coms:
        print(scanned_port.description)
        comlist.append(scanned_port.device)
    print('Connected COM ports: ' + str(comlist))
    return comlist


def initialize_com_driver(port):
    global driver
    try:
        driver.close()
    except SerialException:
        pass

    try:
        driver.baudrate = 500000
        driver.port = port
        driver.open()
        driver.timeout = 0
    except SerialException:
        print('@ERROR : Alice_F3000 IS NOT AVAILABLE')
    else:
        print('#connection(Alice_F3000) has been initialized:' + driver.name)


def throw_data(data):
    linefeed = '\n'
    # encode node
    try:
        linefeed_enc = linefeed.encode('utf-8')
        data_enc = str(data).encode('utf-8')
    except UnicodeEncodeError:
        print('@CATCH EXCEPT WHILE DECOING DATA!')

    # transmit node
    try:
        driver.reset_input_buffer()
        driver.reset_output_buffer()
        driver.write(data_enc)
        driver.write(linefeed_enc)
        # print(data_enc)
        # print(linefeed_enc)
    except SerialException:
        print('@CATCH EXCEPT WHILE TRANSMITTING DATA TO DRIVER! PLEASE MOUNT IT AGAIN.')
        while True:
            try:
                time.sleep(1.0)
                initialize_com_driver('COM15')
            except SerialException:
                print('@INITIALIZE ERROR HAS OCCURED.')
            else:
                break
    pass


def print_processing_speed():
    global prevloopTime
    if waitMs(0.1):
        print("loop-period:" + str((time.time() - prevloopTime) * 1000) + " msec")
        prevloopTime = time.time()
    return 0


def waitMs(wait):
    global prevtime
    if((time.time() - prevtime) > wait):
        prevtime = time.time()
        return True
    return False


# initialize com port
port_list = search_com_port()
driver = serial.Serial()
initialize_com_driver('COM15')

print("#PORT INITIALIZED.")

prevloopTime = time.time()
prevtime = 0
pwm_count = 0
flag = True

while True:
    # print_processing_speed()
    if(flag == True):
        pwm_count += 2
    else:
        pwm_count -= 2
    if(pwm_count > 245):
        flag = False
    if(pwm_count < -245):
        flag = True
    throw_data("set motor")
    throw_data(str(pwm_count))
    print(str(pwm_count))
    time.sleep(0.002)

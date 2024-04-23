#! /usr/bin/env python3
# coding=utf-8

"""
 description:
 author:		kevin.wang
 create date:	2024-04-23
 version:		1.0.0
"""


import os
import time
import serial
import serial.tools.list_ports
import threading
import termios
import fcntl
import sys

global_thread_running_flag = True
global_process_data = True
global_serial = None
global_msg = []


class KeyManager():
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.oldterm = termios.tcgetattr(self.fd)
        self.newattr = termios.tcgetattr(self.fd)
        self.newattr[3] = self.newattr[3] & ~termios.ICANON & ~termios.ECHO
        termios.tcsetattr(self.fd, termios.TCSANOW, self.newattr)
        self.oldflags = fcntl.fcntl(self.fd, fcntl.F_GETFL)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, self.oldflags | os.O_NONBLOCK)

    def reInitKey(self):
        self.fd = sys.stdin.fileno()
        self.newattr = termios.tcgetattr(self.fd)
        self.newattr[3] = self.newattr[3] & ~termios.ICANON & ~termios.ECHO
        termios.tcsetattr(self.fd, termios.TCSANOW, self.newattr)
        aflag = fcntl.fcntl(self.fd, fcntl.F_GETFL)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, aflag | os.O_NONBLOCK)

    def __del__(self):
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.oldterm)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, self.oldflags)

    def disableEcho(self):
        self.newattr[3] = self.newattr[3] & ~termios.ICANON & ~termios.ECHO
        termios.tcsetattr(self.fd, termios.TCSANOW, self.newattr)

    def enableEcho(self):
        self.newattr[3] = self.newattr[3] & ~termios.ICANON | termios.ECHO
        termios.tcsetattr(self.fd, termios.TCSANOW, self.newattr)

    def getKey(self):
        try:
            # 一定要4个字符，否则Fn等功能健无法
            c = sys.stdin.read(4)
            if c:
                return c
        except IOError:
            pass
        return ""


# process receive packets
def read_received_packet():
    while global_thread_running_flag:
        if global_process_data:
            try:
                msglength = global_serial.in_waiting
                if msglength < 8:
                    continue
                else:
                    global_msg = []
                    for i in range(msglength):
                        tmp = ord(global_serial.read())
                        global_msg.append(tmp)
                        
                    print(global_msg) 
            except OSError:
                pass


def send_to_serial(cmd):
    global global_serial
    global global_process_data
    try:
        global_serial.write(cmd)
    except OSError as e:
        print('Start Serial USB Initialize ..')
        print('Wait for a moment ..')
        global_process_data = False
        global_serial.close()

        retrytimes = 0
        while retrytimes <= 5:
            retrytimes += 1 
            controller_ports = [p.device for p in serial.tools.list_ports.comports() if 'USB Serial' == p.description]
            if controller_ports:
                break
            else:
                print(f"retry times: {retrytimes}")

        global_serial = serial.Serial(controller_ports[0],2000000)
        time.sleep(0.2)
        global_process_data = True
        print('Re-Initialize Serial USB OK')


if __name__ == '__main__':
    controller_ports = [p.device for p in serial.tools.list_ports.comports() if 'USB Serial' == p.description]

    #print(controller_ports)

    if not controller_ports:
        raise IOError("no controller found")

    global_serial = serial.Serial(controller_ports[0],2000000)
    time.sleep(0.2)

    serial_received_handle = threading.Thread(target = read_received_packet, daemon = True)
    serial_received_handle.start()

    keymrg = KeyManager()
    while True:
        keyval = keymrg.getKey()
        if keyval == 'q':
            break;
        elif keyval == 'k':
            cmd = bytearray(8)
            for i in range(8):
                cmd[i] = i;
            send_to_serial(cmd)
        elif keyval == 'j':
            cmd = bytearray(8)
            for i in range(8):
                cmd[i] = 8 - i;
            send_to_serial(cmd)
    
    global_thread_running_flag = False
    serial_received_handle.join()
    global_serial.close()

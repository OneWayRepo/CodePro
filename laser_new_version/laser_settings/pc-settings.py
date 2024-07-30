#! /usr/bin/env python3
# coding=utf-8

"""
 description:
 author:		kevin.wang
 create date:	2024-07-17
 version:		1.0.0
"""

import serial
from serial.tools import list_ports

import struct
import time
import threading

import logging

from zlib import crc32

from jsonmrg import JsonMrg
import json

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


class InstrumentStatus:
    def __init__(self):
        self.instrumentIndex = 0
        self.MAXinstrument = 0
        self.percent = 0
        self.reply = ''
        self.retry = 0
        # INIT:  
        # PROCESS:
        # OK
        # FAIL
        # FINISH
        # CONTINUE 
        self.status = 'INIT'


class TeensyController:
    def __init__(self, instrumentstatus, port, baud_rate=115200):
        ports = [p.device for p in list_ports.comports() if port == p.device]
        if not ports:
            raise ValueError(f"No device found with serial number: {serial_number}")

        self.packet_serial = serial.Serial(ports[0], baudrate=baud_rate, timeout=1)

        self.instrumentstatus = instrumentstatus

        self.lock = threading.RLock()
        self.query_interval = 1.0  # Query interval in seconds
        self.running = False
        self.query_thread = None
        self.thread_read_received_packet = None

        # type A: means reply=1 is OK
        # type V: means reply is value
        # type S: means reply=8 is value save OK
        self.instruments = [
                    ['TC1:TCPIDCAL=4@1', 'A'],
                    ['TC1:TCSW=1@1', 'A'],
                    ['TC1:TCTUNESTATUS?@1', 'V'],
                    ['TC1:TCTD!@1', 'S'],
                    ['TC1:TCTI!@1', 'S'],
                    ['TC1:TCP!@1', 'S'],
                    ['TC1:TCCTRLINTERVAL!@1', 'S'],

                    ['TC1:TCPIDALGO=4@2', 'A'],
                    ['TC1:TCSW=1@2', 'A'],
                    ['TC1:TCTUNESTATUS?@2', 'V'],
                    ['TC1:TCPIDP!@2', 'S'],
                    ['TC1:TCPIDTI!@2', 'S'],
                    ['TC1:TCPIDTD!@2', 'S'],
                    ['TC1:TCCTRLINTERVAL!@2', 'S'],

                    ['TC1:TCPIDCAL=4@3', 'A'],
                    ['TC1:TCSW=1@3', 'A'],
                    ['TC1:TCTUNESTATUS?@3', 'V'],
                    ['TC1:TCTD!@3', 'S'],
                    ['TC1:TCTI!@3', 'S'],
                    ['TC1:TCP!@3', 'S'],
                    ['TC1:TCCTRLINTERVAL!@3', 'S'],

                    ['TC1:TCPIDCAL=4@4', 'A'],
                    ['TC1:TCSW=1@4', 'A'],
                    ['TC1:TCTUNESTATUS?@4', 'V'],
                    ['TC1:TCTD!@4', 'S'],
                    ['TC1:TCTI!@4', 'S'],
                    ['TC1:TCP!@4', 'S'],
                    ['TC1:TCCTRLINTERVAL!@4', 'S']
                ]

        self.instrumentstatus.MAXinstrument = len(self.instruments)

    def transparent_command(self, command):
        with self.lock:
            title_packet = b'C'
            bytes_command = command.encode('utf-8')
            packet = title_packet + bytes_command
            crc = crc32(packet)
            self.packet_serial.write(packet + struct.pack('<I', crc))
            self.packet_serial.write(b'\x0A\x0D')

    def query_status(self):
        with self.lock:
            packet = b'Q'
            crc = crc32(packet)
            self.packet_serial.write(packet + struct.pack('<I', crc))
            self.packet_serial.write(b'\x0A\x0D')

    def set_temperature_setpoints(self, setpoints):
        with self.lock:
            if len(setpoints) != 6:
                raise ValueError("Must provide 6 temperature setpoints")
            
            packet = b'S' + struct.pack('6f', *setpoints)
            crc = crc32(packet)
            try:
                self.packet_serial.write(packet + struct.pack('<I', crc))
                self.packet_serial.write(b'\x0A\x0D')
            except Exception as e:
                print(e)

    def analyze_TCM_reply(self, reply):
        # save reply
        self.instrumentstatus.reply = reply

        if self.instruments[self.instrumentstatus.instrumentIndex][1] == 'A':
            if reply[4:11] == 'REPLY=1':
                self.instrumentstatus.status = 'OK'
            else:
                self.instrumentstatus.status = 'FAIL'
        elif self.instruments[self.instrumentstatus.instrumentIndex][1] == 'V': 
            pos_1 = reply.find('=', 0)
            pos_2 = reply.find('@', 0)
            if pos_1 != -1 and pos_2 != -1:
                if reply[pos_1 + 1:pos_2] == '100':
                    self.instrumentstatus.percent = reply[pos_1 + 1:pos_2]
                    self.instrumentstatus.status = 'OK'
                else:
                    self.instrumentstatus.percent = reply[pos_1 + 1:pos_2]
                    self.instrumentstatus.status = 'CONTINUE'
            else:
                self.instrumentstatus.status = 'FAIL'
        elif self.instruments[self.instrumentstatus.instrumentIndex][1] == 'S': 
            if reply[4:11] == 'REPLY=8':
                self.instrumentstatus.status = 'OK'
            else:
                self.instrumentstatus.status = 'FAIL'
        else:
            self.instrumentstatus.status = 'FAIL'

    def on_packet_received(self, packet):
        if len(packet) < 4:
            return

        received_crc = struct.unpack('<I', packet[-4:])[0]
        calculated_crc = crc32(packet[:-4])

        if received_crc != calculated_crc:
            print("CRC mismatch")
            return

        if packet[0] == ord('S'):  # Status packet
            laser_status = packet[1:6]
            temp_data = packet[6:-4]
            
            #print("Laser TTL Status:", [bool(x) for x in laser_status])
            
            for i in range(6):
                state = temp_data[i*7]
                temp = struct.unpack('>h', temp_data[i*7 + 1:i*7 + 3])[0] / 100.0
                tec_voltage = struct.unpack('>h', temp_data[i*7 + 3:i*7 + 5])[0] / 100.0
                tec_current = struct.unpack('>h', temp_data[i*7 + 5:i*7 + 7])[0] / 100.0
                
                state_str = ["IDLE", "WARM_UP", "ACTIVE", "ERROR"][state]
                #print(f"Channel {i}: State: {state_str}, Temp: {temp:.2f}°C, TEC Voltage: {tec_voltage:.2f}, TEC Current: {tec_current:.2f}")
        
        elif packet[0] == ord('A'):  # Acknowledgment packet
            print("Parameters set successfully")
        
        elif packet[0] == ord('N'):  # NAK packet
            print("Command not acknowledged")

        elif packet[0] == ord('T'):  # Transparent Command 
            reply_cmd = packet[1:-4].decode()
            self.analyze_TCM_reply(reply_cmd)

        elif packet[0] == ord('C'):  # Acknowledgment packet
            reply_cmd = packet[1:-4].decode()
            print(reply_cmd)
            print("Command Send successfully")

    def query_loop(self):
        while self.running:
            self.query_status()
            time.sleep(self.query_interval)

    def received_loop(self):
        msg = []
        while self.running:
            #msg.append(ord(self.packet_serial.read()))
            if self.packet_serial.in_waiting == 0:
                continue

            char = self.packet_serial.read(1)
            if char == b'\r' and msg[-1] == 0x0A:
                self.on_packet_received(bytearray(msg[:-1]))
                msg = []
                continue
            msg += char

    def start(self):
        self.running = True
        self.query_thread = threading.Thread(target=self.query_loop)
        self.query_thread.start()

        self.thread_read_received_packet = threading.Thread(target=self.received_loop)
        self.thread_read_received_packet.start()

    def stop(self):
        self.running = False
        self.packet_serial.close()
        if self.query_thread:
            self.query_thread.join()
        if self.thread_read_received_packet:
            self.thread_read_received_packet.join()

    def run(self):
        try:
            self.start()
            while self.instrumentstatus.status != 'FINISH':
                self.transparent_command(
                        self.instruments[self.instrumentstatus.instrumentIndex][0])
                self.instrumentstatus.status = 'PROCESS'
                print('NO. ' + str(self.instrumentstatus.instrumentIndex + 1) 
                      + ' Retry: ' + str(self.instrumentstatus.retry + 1) + ' Instrument: ' + self.instruments[self.instrumentstatus.instrumentIndex][0])

                timeout = 10
                while self.instrumentstatus.status == 'PROCESS' and timeout != 0: 
                    time.sleep(0.5)
                    timeout = timeout - 1

                if self.instrumentstatus.status == 'OK':
                    self.instrumentstatus.instrumentIndex = self.instrumentstatus.instrumentIndex + 1
                    self.instrumentstatus.retry = 0
                    if self.instrumentstatus.instrumentIndex == self.instrumentstatus.MAXinstrument:
                        self.instrumentstatus.status = 'FINISH'
                        print('Send Commands Finish')
                elif self.instrumentstatus.status == 'FAIL':
                    if self.instrumentstatus.retry != 3: 
                        self.instrumentstatus.retry += 1
                    else:
                        self.instrumentstatus.status = 'FINISH'
                        print('Send Commands Fail: ' + self.instruments[self.instrumentstatus.instrumentIndex][0])
                        print('Reply: ' + self.instrumentstatus.reply)
                elif self.instrumentstatus.status == 'PROCESS':
                    self.instrumentstatus.status = 'FINISH'
                    print('Send Commands Timeout ')
                elif self.instrumentstatus.status == 'CONTINUE':
                    print('PID tuning: %' + self.instrumentstatus.percent)
                else:
                    print('Send Commands Unknown Error')

                time.sleep(1)

        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            self.stop()

def crc32_to_bytes(crc32_value):
    # Pack the CRC32 integer into bytes using little-endian format
    return struct.pack('<I', crc32_value)


if __name__ == "__main__":
    instrumentstatus = InstrumentStatus()
    controller = TeensyController(instrumentstatus, "/dev/ttyACM0")  # Adjust port as needed

    controller.run()

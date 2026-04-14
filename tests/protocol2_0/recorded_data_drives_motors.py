import asyncio
import time
import xml.etree.ElementTree as ET
from matplotlib import animation, style
import qtm_rt
import numpy as np
import matplotlib.pyplot as plt


import tkinter as tk
from dynamixel_sdk import *
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import ThreadingOSCUDPServer

# ---------------- DYNAMIXEL SETTINGS ----------------

# Some troubleshooting: 
# There is no status packet1: 
    # Verify you're flashed to Protocol 2.0 By using the Recovery option (Update won't upgrade from Protocol 1 to 2)
    # Using the proper motor ID (Flashing with prior step will set the ID to 1)
    # Your COM/Serial port is correct; this file is formatted for the mac system of doing thing
    # Your QTM is ok
# I'm getting out-of-range errors
    # Go into the Wizard, and below the second to the right that shows various operating modes (velocity, etc.)
    # Select "Extended Position." If that doesn't show up, verify you're on firmware version 44/2C(?
MY_DXL = 'X_SERIES'
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

DXL_MINIMUM_POSITION_VALUE = 0
DXL_MAXIMUM_POSITION_VALUE = 4095

packet_counter = 0 
packet_modulo = 7

BAUDRATE = 57600
PROTOCOL_VERSION = 2.0

left_motor_id = 1
right_motor_id = 2
clap_percent = 0
DEVICENAME = '/dev/tty.usbserial-FT763GF8'

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

#todo clean this shit!


# some constants
max_distance = 542.92 # units. take or leave it. chud.
min_distance = 80.26 

FLAG_print_everything_because_we_like_wasting_time = False



portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if not portHandler.openPort():
    print("Failed to open port")
    raise SystemExit

if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to set baudrate")
    portHandler.closePort()
    raise SystemExit

for motor_id in [left_motor_id, right_motor_id]:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
    )

    if dxl_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(dxl_comm_result))
        portHandler.closePort()
        raise SystemExit
    
    if dxl_error != 0:
        print(packetHandler.getRxPacketError(dxl_error))
        portHandler.closePort()
        raise SystemExit

print(f"Dynamixel {motor_id} connected")


def set_motor_positon(signal, sign, ID):
    # Set the first motor, then the right one
    # Goals      will be respectively +- 4095
    
    signal = signal * sign
    
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
        portHandler, ID, ADDR_GOAL_POSITION, signal
    )
    
  

def update_motors(signal):
    for motor in [left_motor_id , right_motor_id]:
        set_motor_positon(signal, 1 if motor == 0 else -1 ,motor)


def create_body_index(xml_string):
    xml = ET.fromstring(xml_string)

    body_to_index = {}
    for index, body in enumerate(xml.findall("*/Body/Name")):
        body_to_index[index] = body.text.strip() # type: ignore

    return body_to_index


def body_enabled_count(xml_string):
    xml = ET.fromstring(xml_string)
    return sum(enabled.text == "true" for enabled in xml.findall("*/Body/Enabled"))


async def main():
    global clap_percent
    start_time = time.time()
    x = [0]
    y = [0]
    live_data = []
    live_time = []
    
    connection = await qtm_rt.connect("35.3.96.104", version="1.24")
    distance_history = []
    if connection is None:
        print("Failed to connect")
        return

    xml_string = await connection.get_parameters(parameters=["6d"])
    index_to_body = create_body_index(xml_string)

    print("{} of {} 6DoF bodies enabled".format(
        body_enabled_count(xml_string), len(index_to_body))
    )

    def on_packet(packet):
        # jfc this cannot be good for me
        global packet_counter 
        global packet_modulo
        packet_counter = (packet_counter + 1) % packet_modulo
        if(packet_counter == 0): 
            info, bodies = packet.get_6d()
            # print("Framenumber: {} - Body count: {}".format(
            #     packet.framenumber, info.body_count)
            # )

            positions = []
            for i, body_data in enumerate(bodies):
                body_name = index_to_body.get(i, f"Body_{i}")
                position, rotation = body_data
                new_pos = []
                for i in position:
                    new_pos.append(float(i))
                positions.append([body_name, new_pos])
                
            distance = abs(positions[0][1][1] - positions[1][1][1])
            distance = float('{:.2f}'.format(distance))
            
            # print(distance, f"max distance: {max(distance_history)}," f"min distance: {min(distance_history)}")
            clap_percent = distance / (max_distance - min_distance)
            
            
            
            # print((clap_percent * 2048.0))
            print("FUCK!       ", packet_counter)
            # Why is the FUCKING SIGNAL WRONG GOD DAMN IT -sz
            update_motors(int(clap_percent * 2048.0))

    while True:
        await connection.stream_frames(components=["6d"], on_packet=on_packet)
        
        


if __name__ == "__main__":
    asyncio.run(main())
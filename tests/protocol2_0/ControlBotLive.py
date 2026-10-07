import asyncio
import collections
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

# region constants and boilerplate

# QUICKSTART: ==================================================
# Read up on your OS' method for USB access.
# For OSX, run "ls /dev/tty*" - 
#   For me, the proper port is always shown as some mutation of "[...]usbserial."
# On Windows machines, you'll find devices as COM[X]; 
#   Either pop open device manager (Win + X , M)
#   Or, allegedly, run the "chgport" command.
# Anyways, throw that name into DEVICENAME.

# Another thing you'll have to do is to set the IP address. Make sure you're on the same
# network as the capture computer (I prefer to tether both to my hotspot, as MWireless 
# can be a bit of a headache. 

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
# We can't keep up with the rate of packets we receive, so we only take in 
# every nth packet
packet_modulo = 9

init_motor_1 = 0
init_motor_2 = 0

# todo implement startup

BAUDRATE = 2000000
PROTOCOL_VERSION = 2.0

left_motor_id = 1
right_motor_id = 2
flip_left = True
flip_right = False
clap_percent = 0

DEVICENAME = '/dev/tty.usbserial-FT6S4EG7'
'/dev/tty.usbserial-FT763GF8'

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

#todo Re-calibrate

# Fill in non-populated spans with None.
# NOTE: This data was all filled in with 1-axis data (so this is the distance
    # in the frontal horizontal axis, instead of full 3D cartesian.) 
    # IE we wanna recalibrate these
# to re-calibrate them.
partipants = [
    {'Name' : "Zeke",
     "Minspan" : 77,
     "Elbowspan" : 879,
     "Wingspan" : None},
    
    {'Name' : "Evan",
    "Minspan" : 68.05,
    "Elbowspan" : None,
    "Wingspan" : 1482},
        
]

chosen_name = 'Zeke'
# WOW THIS IS SO DUMB LMAOOOOOOO
spans_names = [key for key in partipants[0].keys() if key is not "Name"]
    
    
# Get the 2nd shortest span; if they have an elbowspan and wingspan on file, this 
# Should grab the elbowspan, eg.
# This is over-engineered. So what!


smallspan = [part for part in partipants if part["Name"].upper() == chosen_name.upper()][0]["Minspan"]

# Get the elbowspan if possible, else get wingspan
bigspan = (
    [part for part in partipants if part["Name"].upper() == chosen_name.upper()][0]["Elbowspan"] or 
    [part for part in partipants if part["Name"].upper() == chosen_name.upper()][0]["Wingspan"])

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


# endregion constants and boilerplate

def set_motor_positon(signal, sign, ID):
    # Set the first motor, then the right one
    # Goals      will be respectively +- 4095

    #todo fix    
    index = ID - 1
    # Weird offset logic for the motors, iirc? You'll probably have to tune this for each
    # new setup
    things = [3103, 949]
    
    signal = signal * sign
    
    input =  min(signal + 2048, things[index]) if (not index) else max(signal + 2048, things[index])  
    
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
        portHandler, ID, ADDR_GOAL_POSITION, input #(?)
    )

def update_motors(signal):
    for motor in [left_motor_id , right_motor_id]:
        set_motor_positon(signal, 1 if motor == 1 else -1 ,motor)

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
    # init_motor_1, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, left_motor_id, ADDR_PRESENT_POSITION)
    # init_motor_2, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, right_motor_id, ADDR_PRESENT_POSITION)
    print('set init position!')
    global clap_percent
    start_time = time.time()
    x = [0]
    y = [0]
    live_data = []
    live_time = []
    
    #For Davis studio141.213.186.223
    # connection = await qtm_rt.connect("141.213.186.223", version="1.24")
    # connection = await qtm_rt.connect("10.130.130.44", version="1.24")
    # connection = await qtm_rt.connect("10.231.3.44", version="1.24")
    connection = await qtm_rt.connect("10.243.187.44", version="1.24")
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
        
        global packet_counter 
        global packet_modulo
        packet_counter = (packet_counter + 1) % packet_modulo
        if(packet_counter == 0): 
            info, bodies = packet.get_6d()
            # print("Framenumber: {} - Body count: {}".format(
            #     packet.framenumber, info.body_count)
            # )

            positions = collections.deque(5*[['', [-1, -1, -1]]], 5)
            for i, body_data in enumerate(bodies):
                body_name = index_to_body.get(i, f"Body_{i}")
                position, rotation = body_data
                new_pos = []
                for i in position:
                    new_pos.append(float(i))
                positions.appendleft([body_name, new_pos])
                
            print(len(new_pos),len(positions),len(rotation),len(new_pos))
            distance_x = abs(positions[0][1][0] - positions[1][1][0])
            distance_y = abs(positions[0][1][1] - positions[1][1][1])
            distance_z = abs(positions[0][1][2] - positions[1][1][2])

            # distance = float('{:.2f}'.format(distance))
            distance_norm = np.sqrt(pow(distance_x,2) + pow(distance_y,2) + pow(distance_z,2))      
            distance_norm = float('{:.2f}'.format(distance_norm))
            distance = distance_norm
            #! DONT TOUCH! update the constants at the top
            clap_percent = distance / (bigspan - smallspan)
            
            # right increqase, left decrease to back off from clap
            
            # Why is the fricking SIGNAL WRONG GOD darn IT -sz
            try: 
                
                # update_signal = int(clap_percent * 2048.0)
                update_signal = int(clap_percent * 2048.0 / 2)
                
                update_motors(update_signal)
            except:
                pass

    while True:
        await connection.stream_frames(components=["6d"], on_packet=on_packet)

if __name__ == "__main__":
    asyncio.run(main())
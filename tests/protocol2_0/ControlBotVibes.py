import asyncio
import collections
import time
import xml.etree.ElementTree as ET
import qtm_rt
import numpy as np

import tkinter as tk
from dynamixel_sdk import *
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import ThreadingOSCUDPServer


# ---------------- DYNAMIXEL SETTINGS ----------------

MY_DXL = 'X_SERIES'
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

DXL_MINIMUM_POSITION_VALUE = 0
DXL_MAXIMUM_POSITION_VALUE = 4095

packet_counter = 0

# We can't keep up with the rate of packets we receive,
# so we only take in every nth packet
packet_modulo = 9

init_motor_1 = 0
init_motor_2 = 0

BAUDRATE = 2000000
PROTOCOL_VERSION = 2.0

left_motor_id = 1
right_motor_id = 2

flip_left = True
flip_right = False
clap_percent = 0

DEVICENAME = '/dev/tty.usbserial-FT6S4EG7'

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0


# ---------------- PARTICIPANT SETTINGS ----------------

partipants = [
    {
        'Name': "Zeke",
        "Minspan": 77,
        "Elbowspan": 879,
        "Wingspan": None
    },

    {
        'Name': "Evan",
        "Minspan": 68.05,
        "Elbowspan": None,
        "Wingspan": 1482
    },
]

chosen_name = 'Zeke'

spans_names = [
    key for key in partipants[0].keys()
    if key != "Name"
]


chosen_participant = next(
    part for part in partipants
    if part["Name"].upper() == chosen_name.upper()
)

smallspan = chosen_participant["Minspan"]

# Get elbowspan if possible, otherwise wingspan
bigspan = (
    chosen_participant["Elbowspan"]
    if chosen_participant["Elbowspan"] is not None
    else chosen_participant["Wingspan"]
)


FLAG_print_everything_because_we_like_wasting_time = False


# ---------------- DYNAMIXEL CONNECTION ----------------

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
        portHandler,
        motor_id,
        ADDR_TORQUE_ENABLE,
        TORQUE_ENABLE
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


# ---------------- MOTOR CONTROL ----------------

def set_motor_positon(signal, sign, ID):

    # Set the first motor, then the right one
    # Goals will be respectively +- 4095

    index = ID - 1

    # Weird offset logic for the motors
    things = [3103, 949]

    signal = signal * sign

    if index == 0:
        goal_position = min(
            signal + 2048,
            things[index]
        )
    else:
        goal_position = max(
            signal + 2048,
            things[index]
        )

    # Dynamixel expects an integer
    goal_position = int(goal_position)

    # Keep position within valid range
    goal_position = max(
        DXL_MINIMUM_POSITION_VALUE,
        min(goal_position, DXL_MAXIMUM_POSITION_VALUE)
    )

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
        portHandler,
        ID,
        ADDR_GOAL_POSITION,
        goal_position
    )

    if dxl_comm_result != COMM_SUCCESS:
        print(
            f"Motor {ID} communication error: "
            f"{packetHandler.getTxRxResult(dxl_comm_result)}"
        )

    if dxl_error != 0:
        print(
            f"Motor {ID} error: "
            f"{packetHandler.getRxPacketError(dxl_error)}"
        )


def update_motors(signal):

    set_motor_positon(
        signal,
        1,
        left_motor_id
    )

    set_motor_positon(
        signal,
        -1,
        right_motor_id
    )


# ---------------- QTM FUNCTIONS ----------------

def create_body_index(xml_string):

    xml = ET.fromstring(xml_string)

    body_to_index = {}

    for index, body in enumerate(xml.findall("*/Body/Name")):
        body_to_index[index] = body.text.strip()

    return body_to_index


def body_enabled_count(xml_string):

    xml = ET.fromstring(xml_string)

    return sum(
        enabled.text == "true"
        for enabled in xml.findall("*/Body/Enabled")
    )


# ---------------- MAIN ----------------

async def main():

    global clap_percent
    global packet_counter

    print("set init position!")

    start_time = time.time()

    x = [0]
    y = [0]

    live_data = []
    live_time = []

    # QTM connection
    connection = await qtm_rt.connect("141.213.186.223", version="1.24")
    # connection = await qtm_rt.connect("10.130.130.44", version="1.24")
    # connection = await qtm_rt.connect("10.231.3.44", version="1.24")
    # connection = await qtm_rt.connect("10.243.187.44",version="1.24"

    if connection is None:
        print("Failed to connect")
        return

    print("Connected to QTM")

    xml_string = await connection.get_parameters(
        parameters=["6d"]
    )

    index_to_body = create_body_index(xml_string)

    print(
        "{} of {} 6DoF bodies enabled".format(
            body_enabled_count(xml_string),
            len(index_to_body)
        )
    )


    # ---------------- QTM PACKET CALLBACK ----------------

    def on_packet(packet):

        global packet_counter
        global clap_percent

        packet_counter = (
            packet_counter + 1
        ) % packet_modulo

        if packet_counter != 0:
            return

        info, bodies = packet.get_6d()

        if len(bodies) < 2:
            print("Need at least 2 QTM bodies")
            return


        # Get body positions
        positions = []

        for body_index, body_data in enumerate(bodies):

            body_name = index_to_body.get(
                body_index,
                f"Body_{body_index}"
            )

            position, rotation = body_data

            new_pos = []

            for value in position:
                new_pos.append(float(value))

            positions.append([
                body_name,
                new_pos
            ])


        # ---------------- DISTANCE ----------------

        pos1 = positions[0][1]
        pos2 = positions[1][1]

        distance_x = abs(
            pos1[0] - pos2[0]
        )

        distance_y = abs(
            pos1[1] - pos2[1]
        )

        distance_z = abs(
            pos1[2] - pos2[2]
        )

        distance_norm = np.sqrt(
            distance_x ** 2 +
            distance_y ** 2 +
            distance_z ** 2
        )

        distance = float(
            '{:.2f}'.format(distance_norm)
        )


        # ---------------- CLAP PERCENT ----------------

        span_range = bigspan - smallspan

        if span_range == 0:
            print("ERROR: bigspan and smallspan are equal")
            return

        clap_percent = distance / span_range


        # ---------------- MOTOR CONTROL ----------------

        # right increase, left decrease to back off from clap

        try:

            update_signal = int(
                clap_percent * 2048.0 / 2
            )

            update_motors(update_signal)

        except Exception as error:

            print(
                f"Motor update error: {error}"
            )


        # ---------------- DEBUG ----------------

        if FLAG_print_everything_because_we_like_wasting_time:

            print(
                f"Body 1: {positions[0][0]} "
                f"{positions[0][1]}"
            )

            print(
                f"Body 2: {positions[1][0]} "
                f"{positions[1][1]}"
            )

            print(
                f"Distance: {distance:.2f} "
                f"| Clap %: {clap_percent:.3f} "
                f"| Signal: {update_signal}"
            )


    # ---------------- START QTM STREAM ----------------

    while True:

        await connection.stream_frames(
            components=["6d"],
            on_packet=on_packet
        )


if __name__ == "__main__":
    asyncio.run(main())
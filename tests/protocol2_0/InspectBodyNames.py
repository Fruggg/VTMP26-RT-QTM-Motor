
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
            print(body_name)

        # ---------------- DISTANCE ----------------

    while True:
        
        await connection.stream_frames(
            components=["6d"],
            on_packet=on_packet
        )


if __name__ == "__main__":
    asyncio.run(main())
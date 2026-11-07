
import asyncio
import collections
import csv
import time
import xml.etree.ElementTree as ET

import qtm_rt

import numpy as np

import tkinter as tk
from dynamixel_sdk import *
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import ThreadingOSCUDPServer
# sweep_unity.py
# Smoothly opens and closes both arms at 50 updates/second, like the real pipeline.
# Press Ctrl+C in the terminal to stop.
import math
import time
from pythonosc.udp_client import SimpleUDPClient


# ============================================================
# CONFIGURATION
# ============================================================


# Take data from participant (right) to control motors on left
BOT_SELECTION = "P"  # Options: "P" or "E"
BASE_ID = 10
BOT_SELECTION = BOT_SELECTION.upper()

if BOT_SELECTION not in ("P", "E"):
    raise ValueError("BOT_SELECTION must be either 'P' or 'E'")

# Automatically select QTM body names
LEFT_BODY_NAME = f"{BOT_SELECTION}L"
RIGHT_BODY_NAME = f"{BOT_SELECTION}R"

print(f"Selected bot: {BOT_SELECTION}")
print(f"Tracking bodies: {LEFT_BODY_NAME}, {RIGHT_BODY_NAME}")


# ---------------- DYNAMIXEL SETTINGS ----------------

MY_DXL = 'X_SERIES'
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

DXL_MINIMUM_POSITION_VALUE = 0
DXL_MAXIMUM_POSITION_VALUE = 4095

packet_counter = 0
packet_modulo = 9

UNITY_IP = "127.0.0.1"
UNITY_PORT = 9000

RATE_HZ = 50          # updates per second (same as the real pipeline)
PERIOD_S = 4.0        # seconds for one full open-and-close
MAX_DEG = 92.7        # PLACEHOLDER full-open angle. Replace after homing the real robot.


# Stuff for the participant

P_maxspan = None
P_minspan = None
# ---------
E_maxspan = None
E_minspan = None



# DEVICENAME = '/dev/tty.usbserial-FT6S4EG7'
DEVICENAME = '/dev/tty.usbserial-FT763GF8'

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0


# ---------------- PARTICIPANT SETTINGS ----------------



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

    client = SimpleUDPClient(UNITY_IP, UNITY_PORT)
    await asyncio.sleep(.500)
    
    with open('Wingspans.csv', mode='r', newline='', encoding='utf-8') as file:
    # Read in the values for the wingspans 
        global P_maxspan
        global P_minspan
        global E_maxspan
        global E_minspan
        reader = csv.reader(file)
        header_row = next(reader)
        first_row = next(reader)
        print(first_row)
        # P_maxspan = first_row[0]['Part_maxspan']
        # P_minspan = first_row[0]['Part_minspan']
        # E_maxspan = first_row[0]['Evan_maxspan']
        # E_minspan = first_row[0]['Evan_minspan']

        P_maxspan = float(first_row[0])
        P_minspan = float(first_row[1])
        E_maxspan = float(first_row[2])
        E_minspan = float(first_row[3])

    print("Set init position!")

    start_time = time.time()

    x = [0]
    y = [0]

    live_data = []
    live_time = []

    # QTM connection
    connection = await qtm_rt.connect(
        "141.213.186.223",
        version="1.24"
    )

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

    # --------------------------------------------------------
    # VERIFY SELECTED BODIES
    # --------------------------------------------------------

    body_to_index = {
        name: index
        for index, name in index_to_body.items()
    }

    required_bodies = [
        LEFT_BODY_NAME,
        RIGHT_BODY_NAME
    ]

    missing_bodies = [
        name for name in required_bodies
        if name not in body_to_index
    ]

    if missing_bodies:
        print(
            f"ERROR: Missing QTM bodies: {missing_bodies}"
        )
        print(
            f"Available bodies: {list(body_to_index.keys())}"
        )
        return

    left_index = body_to_index[LEFT_BODY_NAME]
    right_index = body_to_index[RIGHT_BODY_NAME]

    print(f"Left body: {LEFT_BODY_NAME} (index {left_index})")
    print(f"Right body: {RIGHT_BODY_NAME} (index {right_index})")

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

        if len(bodies) <= max(left_index, right_index):
            print("Selected QTM body indices are unavailable")
            return

        # ----------------------------------------------------
        # EXTRACT SELECTED BODY POSITIONS
        # ----------------------------------------------------

        positions = {}

        for body_index in [left_index, right_index]:

            body_name = index_to_body[body_index]

            position, rotation = bodies[body_index]

            positions[body_name] = [
                float(value)
                for value in position
            ]

        pos1 = positions[LEFT_BODY_NAME]
        pos2 = positions[RIGHT_BODY_NAME]

        # ---------------- DISTANCE ----------------

        distance_x = abs(pos1[0] - pos2[0])
        distance_y = abs(pos1[1] - pos2[1])
        distance_z = abs(pos1[2] - pos2[2])

        distance_norm = np.sqrt(
            distance_x ** 2 +
            distance_y ** 2 +
            distance_z ** 2
        )

        distance = float('{:.2f}'.format(distance_norm))

        # ---------------- CLAP PERCENT ----------------

        bigspan = P_maxspan if BOT_SELECTION == "P" else E_maxspan
        minspan = P_minspan if BOT_SELECTION == "P" else E_minspan


        span_range = bigspan - minspan

        if span_range == 0:
            print("ERROR: bigspan and smallspan are equal")
            return

        # Elliott Code
        clap_percent = distance / span_range
        clap_percent = min(clap_percent, 1.0)
        

        left = clap_percent * MAX_DEG
        right = -clap_percent * MAX_DEG
        print("Sending OSC:", [60, left, right])

        try:
            client.send_message("/vtmp/display/A", [60, float(left), float(right)])
        except Exception as e:
            print("OSC SEND ERROR:", e)


    # ---------------- START QTM STREAM ----------------

    while True:

        await connection.stream_frames(
            components=["6d"],
            on_packet=on_packet
        )


if __name__ == "__main__":
    asyncio.run(main())

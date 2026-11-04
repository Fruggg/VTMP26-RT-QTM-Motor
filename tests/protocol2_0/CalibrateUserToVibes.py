import asyncio
import collections
import datetime
import time
import xml.etree.ElementTree as ET
import qtm_rt
import numpy as np


# '''
# The issue seems to be that filtered_data is being eradicated, while the queue is remaining
# fine. This doesn't seem to have any tangible event (same playback has different times; 
# queue is not being filled to capacity always, though it does happen at point.)
# My guess for debugging is to go in and mess around with the logic, to use
# Another structure instead of filtered_data
# Maybe it can't process the request in time? Switch to a second queue that is only appended
# if it fits into the requirements set by filtered_data?
# '''


MY_DXL = 'X_SERIES'
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

DXL_MINIMUM_POSITION_VALUE = 0
DXL_MAXIMUM_POSITION_VALUE = 4095

packet_counter = 0
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

chosen_user = 'Evan'
filtered_data = []
spans_names = [
    key for key in partipants[0].keys()
    if key != "Name"
]

chosen_participant = next(
    part for part in partipants
    if part["Name"].upper() == chosen_user.upper()
)

smallspan = chosen_participant["Minspan"]

bigspan = (
    chosen_participant["Elbowspan"]
    if chosen_participant["Elbowspan"] is not None
    else chosen_participant["Wingspan"]
)


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


async def main():
    time = datetime.datetime.now(datetime.UTC).timestamp()
    global packet_counter
    global distance_history_queue

    # KEEP THIS OUTSIDE on_packet
    distance_memory_size = 2000
    distance_history_queue = collections.deque(
        maxlen=distance_memory_size
    )

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

    def on_packet(packet):

        global packet_counter
        global distance_history_queue
        packet_counter = (packet_counter + 1) % packet_modulo

        if packet_counter != 0:
            return
        
        info, bodies = packet.get_6d()

        if len(bodies) < 2:
            print("Less than 2 bodies detected")
            return

        positions = []

        for body_index, body_data in enumerate(bodies):

            body_name = index_to_body.get(
                body_index,
                f"Body_{body_index}"
            )

            position, rotation = body_data

            new_pos = [
                float(value)
                for value in position
            ]

            positions.append([
                body_name,
                new_pos
            ])

        # First two QTM bodies
        pos1 = positions[0][1]
        pos2 = positions[1][1]
        
        distance_x = abs(pos1[0] - pos2[0])
        distance_y = abs(pos1[1] - pos2[1])
        distance_z = abs(pos1[2] - pos2[2])

        distance_norm = np.sqrt(
            distance_x ** 2 +
            distance_y ** 2 +
            distance_z ** 2
        )

        distance = float(distance_norm)

        # Add this measurement to persistent history
        distance_history_queue.append(distance)

        
        # Don't calculate statistics until we have enough data
        if len(distance_history_queue) < 20:
            print(
                f"Collecting data: "
                f"{len(distance_history_queue)}/20"
            )
            return
        

        history = np.array(distance_history_queue)

        # Q1 and Q3
        q1 = np.percentile(history, 25)
        q3 = np.percentile(history, 75)

        iqr = q3 - q1

        # Outlier bounds
        lower_bound = q1 - 1.5 * iqr
        upper_bound = q3 + 1.5 * iqr
        global filtered_data
        
        filtered_data = [
            value
            for value in distance_history_queue
            if lower_bound <= value <= upper_bound
        ]

        if len(filtered_data) == 0:
            return
        
        filt_min = min(filtered_data)
        filt_max = max(filtered_data)

        filt_range = filt_max - filt_min

        if filt_range == 0:
            return
        
        # Top 5%
        upper_filtered = [
            value
            for value in filtered_data
            if value >= 0.98 * filt_max
        ]

        # Bottom 5%
        lower_filtered = [
            value
            for value in filtered_data
            if value <= filt_min + 0.02 * filt_range
        ]

        if len(upper_filtered) == 0:
            return
        
        if len(lower_filtered) == 0:
            return
        
        upper_avg = sum(upper_filtered) / len(upper_filtered)
        lower_avg = sum(lower_filtered) / len(lower_filtered)

        span_range = abs(upper_avg - lower_avg)

        if span_range == 0:
            return
        current_phi = distance / span_range

        print(
            f"Time: {(datetime.datetime.now(datetime.UTC).timestamp() - time):.2f} "
            f"Opt. Widespan (AU): {upper_avg:.2f} "
            f"| Opt. Smallspan (AU): {lower_avg:.2f} "
            f"| φ: {current_phi:.3f}"
        )

    while True:
        await connection.stream_frames(
            components=["6d"],
            on_packet=on_packet
        )
        global filtered_data, que
        


if __name__ == "__main__":
    asyncio.run(main())
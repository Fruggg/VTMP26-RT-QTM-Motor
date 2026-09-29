import asyncio
import time
import xml.etree.ElementTree as ET
from matplotlib import animation, style
import qtm_rt

import numpy as np
import matplotlib.pyplot as plt
# some constants
max_distance = 542.92 # units. take or leave it. chud.
min_distance = 80.26 

FLAG_print_everything_because_we_like_wasting_time = False

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
    start_time = time.time()
    x = [0]
    y = [0]
    live_data = []
    live_time = []
    
    # connection = await qtm_rt.connect("35.3.96.104", version="1.24")
    connection = await qtm_rt.connect("10.177.37.44", version="1.24") # for studio
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
            # print(positions)
        distance = abs(positions[0][1][1] - positions[1][1][1])
        distance = float('{:.2f}'.format(distance))
        distance_history.append(distance)
        
        print(distance, f"max distance: {max(distance_history)}," f"min distance: {min(distance_history)}")
        plt.show()

    while True:
        await connection.stream_frames(components=["6d"], on_packet=on_packet)
        
        


if __name__ == "__main__":
    asyncio.run(main())
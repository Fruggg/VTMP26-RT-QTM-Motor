# Listen here, little chud, here's what we're gonna do...

# Take the data from the mocap markers. In our current dataset, the motion is along the y axes. Maximum distance between the hands is roughly _
# Use this and perform Analyze to 


# Taken from basic example

"""
    Minimal usage example
    Connects to QTM and streams 3D data forever
    (start QTM first, load file, Play->Play with Real-Time output)
"""

import asyncio
import qtm_rt


def on_packet(packet):
    """ Callback function that is called everytime a data packet arrives from QTM """
    print("Framenumber: {}".format(packet.framenumber))
    header, markers = packet.get_3d_markers()
    print("Component info: {}".format(header))
    for marker in markers:
        print("\t", marker)


async def main():
    """ Main function """
    # connection = await qtm_rt.connect("35.3.82.132")
    connection = await qtm_rt.connect("172.16.47.213", version='1.24') # laptop 
    if connection is None:
        return

    while True:
        await connection.stream_frames(components=["3d"], on_packet=on_packet)

    
if __name__ == "__main__":
    asyncio.run(main())
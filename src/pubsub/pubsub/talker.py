#!/usr/bin/env python
# -*- coding: utf-8 -*-

#********************************************#
# Talker Template

# Creted for:
# ROS2 Workshop 2020
# Roverentwicklung für Explorationsaufgaben
# Institute for Space Systems
# University of Stuttgart

# Created by Patrick Winterhalder
# IRS, University of Stuttgart
#********************************************#

import rclpy
from rclpy.node import Node

# Import Publisher Library (Python Library)
from .pubsub_library import MinimalPublisher

# Import Message Types (Message Files)
from pubsub_msg.msg import CustomMsg1
from pubsub_msg.msg import CustomMsg2


def main(args=None):
    rclpy.init(args=args)

    # Start nodes here, should create object <node_name> for every node
    talker_1 = MinimalPublisher(NODE_NAME="pubsub_talker_1", TOPIC_NAME="/chatter1", MSG_TYPE=CustomMsg1, MSG_PERIOD=0.25)
    talker_2 = MinimalPublisher(NODE_NAME="pubsub_talker_2", TOPIC_NAME="/chatter2", MSG_TYPE=CustomMsg2, MSG_PERIOD=0.5)

    while rclpy.ok():
        try:
            # Insert main looping script here...

            # Eg. create custom msgs, fill and send them
            # CustomMsg1 to talker_1
            msg_1 = CustomMsg1()
            msg_1.temperature = [24.5, 25.5, 26.5]
            msg_1.pressure = [1011.5, 1012.55, 1010.11]
            msg_1._humidity = [0.002, 0.0019, 0.0021]
            talker_1.timer_publish(msg_1)

            # CustomMsg2 to talker_2
            msg_2 = CustomMsg2()
            msg_2.pitch_ctrl = 0.0
            msg_2.yaw_ctrl = 0.22
            talker_2.timer_publish(msg_2)


            # Insert "rclpy.spin_once(<node_name>, timeout_sec=0.1)" at the end of 
            # "try" for every node running in this script
            rclpy.spin_once(talker_1, timeout_sec=0.1)
            rclpy.spin_once(talker_2, timeout_sec=0.1)
            
        except (KeyboardInterrupt, SystemExit):
            print("\n\nShutting down...")
            # Insert "<node_name>.destroy_node()" here once for every node running in this script
            talker_1.destroy_node()
            talker_2.destroy_node()
            rclpy.shutdown()
        


if __name__ == '__main__':
    main()
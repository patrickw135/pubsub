#!/usr/bin/env python
# -*- coding: utf-8 -*-

#********************************************#
# Listener Template

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

# Import Subscriber Library
from .pubsub_library import CustomMsg1_sub
from .pubsub_library import CustomMsg2_sub

# Import Message Types
from pubsub_msg.msg import CustomMsg1
from pubsub_msg.msg import CustomMsg2


def main(args=None):
    rclpy.init(args=args)

    # Start nodes here, should create object <node_name> for every node
    listener_1 = CustomMsg1_sub(NODE_NAME="pubsub_listener_1", TOPIC_NAME="/chatter1",MSG_TYPE=CustomMsg1, NUM_MSGS=0)
    listener_2 = CustomMsg2_sub(NODE_NAME="pubsub_listener_2", TOPIC_NAME="/chatter2",MSG_TYPE=CustomMsg2, NUM_MSGS=0)


    while rclpy.ok():
        try:
            # Insert main looping script here...

            # Receive Topics by running nodes
            rclpy.spin_once(listener_1, timeout_sec=0.01)
            rclpy.spin_once(listener_2, timeout_sec=0.01)

            # Do sth if message was received
            if listener_1.topic_received is True:
                listener_1.topic_received = False # zurücksetzen
                msg_1 = listener_1.return_msg()
                #*********************************
                # Do sth based on received message
                #*********************************
            if listener_2.topic_received is True:
                listener_2.topic_received = False # zurücksetzen
                msg_2 = listener_2.return_msg()
                #*********************************
                # Do sth based on received message
                #*********************************
            

            # Check if "msg_1" or "msg_2" is available as a local variable
            if 'msg_1' in locals():
                # Print msg_1
                listener_1.print_msg()
            if 'msg_2' in locals():
                # Print msg_2
                listener_2.print_msg()

            # Here, you can now also publish/pass on the results of this script by
            # using the "MinimalPublisher" class. For this please  refer to "talker.py".

        except (KeyboardInterrupt, SystemExit):
            print("\n\nShutting down...")

            # Insert "<node_name>.destroy_node()" here for all running nodes in this script
            # eg:
            listener_1.destroy_node()
            listener_2.destroy_node()
            rclpy.shutdown()
        


if __name__ == '__main__':
    main()
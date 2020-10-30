#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

# Import Subscriber Library
from .pubsub_library import MinimalSubscriber

# Import Message Types
from pubsub_msg.msg import CustomMsg1
from pubsub_msg.msg import CustomMsg2


def main(args=None):
    rclpy.init(args=args)

    # Start nodes here, should create object <node_name> for every node
    listener_1 = MinimalSubscriber(NODE_NAME="pubsub_listener_1", TOPIC_NAME="/chatter1",MSG_TYPE=CustomMsg1, NUM_MSGS=0)
    listener_2 = MinimalSubscriber(NODE_NAME="pubsub_listener_2", TOPIC_NAME="/chatter2",MSG_TYPE=CustomMsg2, NUM_MSGS=0)


    while rclpy.ok():
        try:

            # Insert main looping script here
            # Then insert "rclpy.spin_once(<node_name>, timeout_sec=0.1)" for every node running in this script
            
            pass
        except (KeyboardInterrupt, SystemExit):
            print("\n\nShutting down...")

            # Insert "<node_name>.destroy_node()" here for all running nodes in this script
            # eg:
            listener_1.destroy_node()
            listener_2.destroy_node()
            rclpy.shutdown()
        


if __name__ == '__main__':
    main()
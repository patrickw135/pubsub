import rclpy
from rclpy.node import Node

# Import Publisher Library
from .pubsub_library import MinimalPublisher

# Import Message Types
from libs_msgs_package.msg import Userconsole
from pubsub_msg.msg import CustomMsg1
from pubsub_msg.msg import CustomMsg2


def main(args=None):
    rclpy.init(args=args)

    # Start nodes here, should create object <node_name> for every node
    talker_1 = MinimalPublisher(NODE_NAME="pubsub_talker_1", TOPIC_NAME="/chatter1", MSG_TYPE=CustomMsg1)
    talker_2 = MinimalPublisher(NODE_NAME="pubsub_talker_2", TOPIC_NAME="/chatter2", MSG_TYPE=CustomMsg2)

    while rclpy.ok():
        try:

            # Insert main looping script here
            # Then insert "rclpy.spin_once(<node_name>, timeout_sec=0.1)" for every node running in this script
            rclpy.spin_once(talker_1, timeout_sec=0.1)
            rclpy.spin_once(talker_2, timeout_sec=0.1)
            
        except (KeyboardInterrupt, SystemExit):
            print("\n\nShutting down...")
            # Insert "<node_name>.destroy_node()" here for all running nodes in this script
            talker_1.destroy_node()
            talker_2.destroy_node()
            rclpy.shutdown()
        


if __name__ == '__main__':
    main()
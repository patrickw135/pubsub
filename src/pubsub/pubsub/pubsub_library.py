# ROS 2 Minimal Publisher/Subscriber Class Template


import rclpy
from rclpy.node import Node

# Use:
# from pubsub_library import MinimalPublisher
# from pubsub_library import MinimalSubscriber
# minimal_publisher = MinimalPublisher(NODE_NAME='minimal_pub', TOPIC_NAME='user_controller', MSG_TYPE=Usercontroller)
# minimal_subscriber = MinimalSubscriber(NODE_NAME='minimal_sub', TOPIC_NAME='epos_feedback', MSG_TYPE=Eposreturn)




#******************************************************************************#
# Definition of Parent Classes

class MinimalPublisher(Node):

    def __init__(self, NODE_NAME, TOPIC_NAME, MSG_TYPE, MSG_PERIOD):
        self.PUBLISHER_NAME= NODE_NAME
        self.TOPIC_NAME = TOPIC_NAME
        self.CUSTOM_MSG = MSG_TYPE
        self.timer_period = MSG_PERIOD  # [seconds]
        # Init above laying class Node
        super().__init__(self.PUBLISHER_NAME)
        print("\t- " + str(TOPIC_NAME) + "\n")
        self.publisher_ = self.create_publisher(
            self.CUSTOM_MSG, 
            self.TOPIC_NAME, 
            10)
        self.new_msg = False
        # Define Node Frequency, equivalent to ~rospy.rate()
        self.timer = self.create_timer(self.timer_period, self.publisher_timer)
        return

    def publisher_timer(self):
        if self.new_msg is True:
            try:
                #self.get_logger().info('Pub:')
                self.publisher_.publish(self.msg)
                self.new_msg = False
            except TypeError:
                print("[ERROR] Msg-Data-Types do not match")
        return

    # Publish using Timer
    def timer_publish(self, msg):
        self.msg=msg
        self.new_msg = True
        return

    # Publish directly without Timer
    def direct_publish(self, msg):
        try:
            #self.get_logger().info('Pub:')
            self.publisher_.publish(msg)
        except TypeError:
            print("[ERROR] Msg-Data-Types do not match")
        return









class MinimalSubscriber(Node):

    def __init__(self, NODE_NAME, TOPIC_NAME, MSG_TYPE, NUM_MSGS):
        self.NODE_NAME= NODE_NAME
        self.TOPIC_NAME = TOPIC_NAME
        self.CUSTOM_MSG = MSG_TYPE
        self.NUM_MSGS = NUM_MSGS
        self.topic_received = False
        # Init above laying class Node
        super().__init__(self.NODE_NAME)
        self.subscription = self.create_subscription(
            self.CUSTOM_MSG,            # Message Type
            self.TOPIC_NAME,            # Topic Name
            self.listener_callback,     # Callback Function
            self.NUM_MSGS)              # List of saved messages
        self.subscription  # prevent unused variable warning
        return

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        return










#***************************************************************************#
# Child Classes inherite through super().__init__(...) from parent class
# Child's purpose is to simplify subscribing to custom messages 
# For this to work you must customize "listener_callback", "return_msg" (and "print_msg")



# Exemplary Subscriber Class: Inheriting from MinimalSubscriber Class
# This class is custom made for receiving the specific msg type but uses MinimalSubscriber as foundation
# Create a new class of this type for every custom msg you want to receive, customizing the listener_callback, 
# return_msg (and print_msg) functions to correspond to your message

class example_subscriber(MinimalSubscriber):

    def __init__(self, NODE_NAME, TOPIC_NAME, MSG_TYPE, NUM_MSGS):
        # Init MinimalSubscriber:
        super().__init__(NODE_NAME, TOPIC_NAME, MSG_TYPE, NUM_MSGS)
        print("\t- " + str(TOPIC_NAME))
        # Additional variables only for Epos subscriber
        self.pos_actual = None
        self.vel_actual = None
        self.baffle_switch = None
        self.current = None
        self.temp = None
    

    # !!! CUSTOMIZE THIS FUNCTION TO WORK WITH YOUR MESSAGE FILE !!!
    def listener_callback(self, msg): # Overwrites callback from inherited class
        self.topic_received = True
        self.pos_actual = msg.pos_actual
        self.vel_actual = msg.vel_actual
        self.baffle_switch = msg.baffle_switch
        self.current = msg.epos_current
        self.temp = msg.epos_temp
        #print_msg(self) # activate to print the received data --> customize print_msg(self) !
        return

    # !!! CUSTOMIZE THIS FUNCTION TO WORK WITH YOUR MESSAGE FILE !!!
    def return_msg(self): # Extract only msg variables from "self" object
        msg = self.CUSTOM_MSG()
        msg.pos_actual = self.pos_actual
        msg.vel_actual = self.vel_actual
        msg.baffle_switch = self.baffle_switch
        msg.epos_current = self.current
        msg.epos_temp = self.temp
        return msg

    # !!! CUSTOMIZE THIS FUNCTION TO WORK WITH YOUR MESSAGE FILE !!! (optional)
    def print_msg(self):
        print("[SUBSCRIBER] %s received topic:\t/%s" %(self.NODE_NAME, self.TOPIC_NAME))
        string = "Received:\t"

        # Check length of all transmitted value lists to be equal (equal number of parameters)
        length = len(self.pos_actual)
        if any(len(lst) != length for lst in [self.vel_actual, self.current, self.temp]):
            print("[TRANSMISSION ERROR] Length of lists inside topic %s do not match" %(self.TOPIC_NAME))
        else:
            complete_list = []
            complete_list.append(self.pos_actual)
            complete_list.append(self.vel_actual)
            complete_list.append(self.current)
            complete_list.append(self.temp)
            #print(complete_list)
            for i in range(len(complete_list)):
                for y in range(len(complete_list[0])):
                    string += "%.2f,\t" %(complete_list[i][y])
            string += str(self.baffle_switch)
            print(string)
        return


# Import Dependencies
import rospy 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import math

class DistanceReader:
    def __init__(self):
        # Initialize the node
        rospy.init_node('turtlesim_distance_node', anonymous=True)

        # Initialize subscriber: input the topic name, message type and callback signature  
        rospy.Subscriber("/turtle1/pose", Pose, self.callback)

        # Initialize publisher: input the topic name, message type and msg queue size
        self.distance_publisher = rospy.Publisher('/turtle_dist', Float64, queue_size=10)

        # Initialize variables to store the previous position
        self.prev_x = None
        self.prev_y = None

        # Initialize total distance traveled
        self.total_distance = 0.0

        # Printing to the terminal, ROS style
        rospy.loginfo("Initialized node!")

        # This blocking function call keeps python from exiting until node is stopped
        rospy.spin()

    # Whenever a message is received from the specified subscriber, this function will be called
    def callback(self, msg):

        if self.prev_x is not None and self.prev_y is not None:
            # Calculate the distance between current and previous positions
            diff_x = msg.x - self.prev_x
            diff_y = msg.y - self.prev_y
            distance = math.sqrt(diff_x ** 2 + diff_y ** 2)

            # Update total distance traveled
            self.total_distance += distance

            # Publish the total distance
            self.distance_publisher.publish(self.total_distance)

            rospy.loginfo("Total distance: %s", self.total_distance)

        # Update previous position
        self.prev_x = msg.x
        self.prev_y = msg.y

if __name__ == '__main__': 
    try: 
        distance_reader_class_instance = DistanceReader()
    except rospy.ROSInterruptException: 
        pass

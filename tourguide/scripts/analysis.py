import rospy
import tf
from geometry_msgs.msg import Point

class Analysis():
    def __init__(self):
        rospy.init_node('analysis', anonymous=False)

        rospy.on_shutdown(self.shutdown)

        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(5.0))

    def run(self):
        while not rospy.is_shutdown():
            self.position = self.get_position()
            self.x = self.position.x
            self.y = self.position.y
            rospy.loginfo("robot at x:%.3f,y:%.3f" %(self.x, self.y))


    def get_position(self):
        # Get the current transform between the map and base_link frames
        try:
            # self.tf_listener.lookupTransform("/map", "/base_link", rospy.Time(0), transform)
            (transform, rotation)  = self.tf_listener.lookupTransform("/map", "/base_link", rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return Point(*transform)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        rospy.sleep(1)
 
if __name__ == '__main__':
    test = Analysis()
    rospy.spin()
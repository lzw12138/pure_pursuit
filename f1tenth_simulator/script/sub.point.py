import rospy
import numpy as np

from geometry_msgs.msg import PointStamped

class Turtlebot_core():
    def __init__(self):
        rospy.init_node("test", anonymous=True)
        self.wps_buf = []
        rospy.Subscriber("/clicked_point", PointStamped, self.wpsCallback, queue_size=1)
        rospy.spin()

    def wpsCallback(self, data):
        print('x')
        p1 = np.zeros(2)
        p1[0] = data.point.x
        p1[1] = data.point.y
        self.wps_buf.append(p1)
        np.savetxt("wps.txt", np.array(self.wps_buf))
        print("save way point success!!!")
        print("p:"+str(p1))



if __name__ == "__main__":
    turtlebot_core = Turtlebot_core()

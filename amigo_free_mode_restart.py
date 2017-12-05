#!/usr/bin/env python

import roslaunch
import rospy
import rospkg
import os
from std_msgs.msg import String

class AutoStarter(object):
    """
    Launches amigo_free_mode and waits for it to either shutdown or to restart.
    """
    def __init__(self):
        """
        Launches amigo_free_mode and starts a makes a subscriber that listens when it should
        shutdown or restart. When the node stops it will also shutdown the launch file.
        """

        self.start()

        rospy.Subscriber('action', String, self._callback)

        rospy.spin()

        self.launch.shutdown()

    def start(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        ## Get in the right directory
        rospack = rospkg.RosPack()

        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [
            os.path.join(rospack.get_path('amigo_bringup'), "launch", "state_machines", "free_mode.launch")])

        self.launch.start()

    def stop(self):

        self.launch.shutdown()

    def _callback(self, msg):
        """
        :param msg: Is a string with either shutdown or restart.
        """

        if msg.data == "shutdown":
            self.stop()
            print "Performed {}".format(msg.data)
        elif msg.data == "restart":
            self.stop()
            self.start()
            print "Performed {}".format(msg.data)

if __name__ == "__main__":
    rospy.init_node('auto_starter', anonymous=False)

    autostarter = AutoStarter()


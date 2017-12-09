# ROS
import os
import roslaunch
import rospkg
import rospy
from std_msgs.msg import String

# TU/e Robotics
from node_alive.srv import AutoStarterCommand, AutoStarterCommandRequest, AutoStarterCommandResponse


class AutoStarter(object):
    """
    Launches amigo_free_mode and waits for a command to either shutdown or to restart.
    """
    def __init__(self):
        """
        Creates a service over which the command will be received, next amigo free mode is started and keeps running
        until it gets a command to either shutdown or to restart.
        """
        self._command_service = rospy.Service('auto_starter_command', AutoStarterCommand, self._handle_auto_starter_command)
        self.start()
        rospy.spin()
        self.launch.shutdown()

    def _handle_auto_starter_command(self, req):
        """
        All request from the service are passed into this function and depending on the value it will either shutdown
        or restart the launch file.
        :param req: the command request that the service receives from the client.
        :return: returns the integer 1  when function finishes successful.
        """
        if req.command == AutoStarterCommandRequest.STOP:
            self.stop()
            rospy.loginfo("Performed shutdown")
        elif req.command == AutoStarterCommandRequest.RESTART:
            self.stop()
            self.start()
            rospy.loginfo("Performed restart")
        return AutoStarterCommandResponse(AutoStarterCommandResponse.SUCCEEDED)

    def start(self):
        """
        Finds the right directory of the launch file of free mode and launches the file.
        """
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # Get in the right directory
        rospack = rospkg.RosPack()

        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [
            os.path.join(rospack.get_path('amigo_bringup'), "launch", "state_machines", "free_mode.launch")])
        self.launch.start()

    def stop(self):
        """
        Shuts the launch file down.
        """
        self.launch.shutdown()
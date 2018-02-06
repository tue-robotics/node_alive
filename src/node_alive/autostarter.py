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
    Launches a launch file and waits for a command to either shutdown or to restart.
    It is also possible to start other launch files that are stored on the parameter server, like the demo's.
    """
    launch_files_list = []

    def __init__(self):
        """
        Loads the launch files that can be started from the parameter server.
        Creates a service over which the command will be received, a publisher that publishes the current launch file.
        Set some variables.
        """
        self.launch_files_list = rospy.get_param('~launch_files_list')
        self._command_service = rospy.Service('auto_starter_command', AutoStarterCommand, self._srv_callback)
        self._status_pub = rospy.Publisher('current_launch_file', String, queue_size=1, latch=True)

        self.launch = None
        self._pending_req = None
        self._pending_resp = None

    def update(self):
        """
        If there is a request of the service that was pending, it will be executed in the main thread when sleep is over
        After executing the request, the pending status will be reset.
        :return:
        """
        if self._pending_req is not None:
            self._pending_resp = self._handle_auto_starter_command(self._pending_req)
            self._pending_req = None

    def _srv_callback(self, req):
        """
        Recieves the request from the service and sets it to pending. As long as there is no response, the function
        will sleep till it gets one. After that the response will reset.
        :param req: the command request that the service receives from the client contains two data types.
        :return:
        """
        self._pending_resp = None
        self._pending_req = req
        rate = rospy.Rate(10.0)
        while self._pending_resp is None and not rospy.is_shutdown():
            rate.sleep()
        return self._pending_resp

    def _handle_auto_starter_command(self, req):
        """
        All request from the service are passed into this function and depending on the value it will either shutdown,
        restart the launch file or start another launch file.
        :param req: the command request that the service receives from the client contains two data types.
        :return: returns the integer 1  when function finishes successful or -1 or -2 when no or incorrect file is given.
        """

        if req.filename in self.launch_files_list:
            if req.command == AutoStarterCommandRequest.START:
                if self.launch is None:
                    self.start('amigo_bringup', os.path.join("launch", "state_machines", req.filename))
                    rospy.loginfo("Performed new start")
                    return AutoStarterCommandResponse(AutoStarterCommandResponse.SUCCEEDED)
                else:
                    self.stop()
                    self.start('amigo_bringup', os.path.join("launch", "state_machines", req.filename))
                    rospy.loginfo("Performed restart")
                    return AutoStarterCommandResponse(AutoStarterCommandResponse.SUCCEEDED)
            elif req.command == AutoStarterCommandRequest.STOP:
                self.stop()
                rospy.loginfo("Performed shutdown")
                return AutoStarterCommandResponse(AutoStarterCommandResponse.SUCCEEDED)
            else:
                return AutoStarterCommandResponse(AutoStarterCommandResponse.LAUNCH_ERROR)
        else:
            return AutoStarterCommandResponse(AutoStarterCommandResponse.FILE_NOT_PRESENT)

    def start(self, package, path):
        """
        Finds the right directory of the launch file and launches the file.
        :param package: package of the current launch file
        :param path: path to the current launch file
        """
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # Get in the right directory
        rospack = rospkg.RosPack()

        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [
            os.path.join(rospack.get_path(package), path)])
        self.launch.start()

        self.update_current_launch_file(package, path)

    def stop(self):
        """
        Shuts the launch file down, this is only possible when there is a launch file running.
        """
        if self.launch is not None:
            self.launch.shutdown()
        else:
            rospy.logwarn("No LaunchParent, No launch file to shut down")

    def update_current_launch_file(self, package, path):
        """
        Publishes the current launch file
        :param package: package of the current launch file
        :param path: path to the current launch file
        """
        full_path = os.path.join(package, path)
        rospy.loginfo("The following launch file is running: %s", full_path)
        self._status_pub.publish(full_path)
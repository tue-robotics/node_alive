# System
import os

# ROS
import roslaunch
import rospkg
import rospy
import rosnode
from std_msgs.msg import String

# TU/e Robotics
from node_alive.srv import AutoStarterCommand, AutoStarterCommandRequest, AutoStarterCommandResponse


class AutoStarter(object):
    """
    Waits for an input from the service to start up a launch file, after this it can restart the same file, start
    another new launch file or stop the current launch file. The list of possible launch files and the package name are
    available on the parameter server.
    """

    def __init__(self):
        """
        Gets the list of launch files and the package name from the parameter server
        Creates a service over which the command will be received and a publisher that publishes the current launch file.
        """
        self._bringup_package_path = rospy.get_param('~package_name')
        self._launch_files_list = rospy.get_param('~launch_files_list')
        self._command_service = rospy.Service('auto_starter_command', AutoStarterCommand, self._srv_callback)
        self._status_pub = rospy.Publisher('current_launch_file', String, queue_size=1, latch=True)

        self._launch = None             # Stores the launch file location
        self._pending_req = None        # Stores the input from the service
        self._pending_resp = None       # Stores the response from the function _handle_auto_starter_command

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
        :param req: (AutoStarterCommandResponse) the command request that the service receives from the client contains
        two data types, which launch file and which command.
        :return response:
        """
        self._time_out = 10.0
        self._pending_resp = None
        self._pending_req = req
        rate = rospy.Rate(10.0)
        t_end = rospy.Time.now() + rospy.Duration(self._time_out)

        while self._pending_resp is None and not rospy.is_shutdown():
            rate.sleep()
            # if there is no response after getting a request. It will set the request to None again and waits for new
            # new request from service.
            if rospy.Time.now() > t_end:
                self._pending_req = None
                self._pending_resp = AutoStarterCommandResponse(AutoStarterCommandResponse.LAUNCH_ERROR)
                rospy.logwarn("No response after getting request. No launch files launched. "
                              "Request is set to None and waits for new request from service")
                break

        response = self._pending_resp
        self._pending_resp = None
        return response

    def _handle_auto_starter_command(self, req):
        """
        All request from the service are passed into this function and depending on the values it will either shutdown,
        restart the launch file or start another launch file.
        :param req: (AutoStarterCommandResponse) the command request that the service receives from the client contains
        two data types, which launch file and which command.
        :return AutoStarterCommandResponse:
        """
        if req.filename in self._launch_files_list:
            if req.command == AutoStarterCommandRequest.START:
                if self._launch is None:
                    self.start(self._bringup_package_path , os.path.join("launch", "state_machines", req.filename))
                    rospy.loginfo("Performed new start")
                    return AutoStarterCommandResponse(AutoStarterCommandResponse.SUCCEEDED)
                else:
                    self.stop()
                    self.start( self._bringup_package_path, os.path.join("launch", "state_machines", req.filename))
                    rospy.loginfo("Performed restart")
                    return AutoStarterCommandResponse(AutoStarterCommandResponse.SUCCEEDED)
            elif req.command == AutoStarterCommandRequest.STOP:
                rospy.loginfo("Performing shutdown")
                self.stop()
                rospy.loginfo("Performed shutdown")
                return AutoStarterCommandResponse(AutoStarterCommandResponse.SUCCEEDED)
            else:
                return AutoStarterCommandResponse(AutoStarterCommandResponse.LAUNCH_ERROR)
        else:
            return AutoStarterCommandResponse(AutoStarterCommandResponse.FILE_NOT_PRESENT)

    def start(self, package, path):
        """
        Before a launche file is started, it kills the node state_machine to make sure that all running launch files
        are closed. Then it finds the right directory of the launch file and launches the file.
        :param package: package of the current launch file
        :param path: path to the current launch file
        """
        rosnode.kill_nodes(['/state_machine'])
        rospy.sleep(0.1)

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # Get in the right directory
        rospack = rospkg.RosPack()

        self._launch = roslaunch.parent.ROSLaunchParent(uuid, [
            os.path.join(rospack.get_path(package), path)])
        self._launch.start()
        self._update_current_launch_file(package, path)

    def stop(self):
        """
        Shuts the launch file down, this is only possible when there is a launch file running.
        """
        if self._launch is not None:
            self._launch.shutdown()
        else:
            rospy.logwarn("No LaunchParent, No launch file to shut down")

    def _update_current_launch_file(self, package, path):
        """
        Publishes the current launch file
        :param package: package of the current launch file
        :param path: path to the current launch file
        """
        full_path = os.path.join(package, path)
        rospy.loginfo("The following launch file is running: %s", full_path)
        self._status_pub.publish(full_path)

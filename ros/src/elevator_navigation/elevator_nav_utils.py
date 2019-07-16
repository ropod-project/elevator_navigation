import rospy
from tf import transformations as tf

from geometry_msgs.msg import PoseWithCovarianceStamped
from maneuver_navigation.msg import Goal as ManeuverNavigationGoal
from ropod_ros_msgs.msg import NavElevatorFeedback, Status

class ElevatorNavData(object):
    def __init__(self):
        self.current_action = None
        self.waiting_pose = None
        self.inside_elevator_pose = None
        self.elevator_door_position = None
        self.elevator_id = -1
        self.base_pose = PoseWithCovarianceStamped()

def get_feedback_msg_skeleton(action_id, action_type):
    '''Returns a NavElevatorFeedback message with the following fields prefilled:
    * feedback.action_id
    * feedback.type
    * feedback.status.domain, and
    * feedback.status.module_code

    Keyword arguments:
    action_id: str -- ID of the action
    action_type: str -- elevator action type

    '''
    feedback_msg = NavElevatorFeedback()
    feedback_msg.feedback.action_id = action_id
    feedback_msg.feedback.action_type = action_type
    feedback_msg.feedback.status.domain = Status.COMPONENT
    feedback_msg.feedback.status.module_code = Status.ELEVATOR_ACTION
    return feedback_msg

def has_timed_out(start_time, timeout):
    '''Returns True if the difference between the current time and the given
    start time is greater than or equal to the given timeout; returns False otherwise.

    Keyword arguments:
    start_time: float -- start time of an operation
    timeout: float -- operation timeout

    '''
    return (rospy.get_time() - start_time) >= timeout

def is_waypoint_achieved(pose1, pose2, pos_tolerance_m, orientation_tolerance_rad):
    '''Returns True if the position distance between the poses is less than
    pos_tolerance_m and the orientation distance is less than orientation_tolerance_rad.

    Keyword arguments:
    pose1: geometry_msgs.msg.PoseStamped
    pose2: geometry_msgs.msg.PoseStamped
    pos_tolerance_m: float -- position tolerance in meters
    orientation_tolerance_rad: float -- orientation tolerance in radians

    '''
    pos_distance = ((pose1.position.x - pose2.position.x)**2 \
                  + (pose1.position.y - pose2.position.y)**2)

    pose1_euler_orientation = tf.euler_from_quaternion([pose1.orientation.w, pose1.orientation.x,
                                                        pose1.orientation.y, pose1.orientation.z])

    pose2_euler_orientation = tf.euler_from_quaternion([pose2.orientation.w, pose2.orientation.x,
                                                        pose2.orientation.y, pose2.orientation.z])

    angular_distance = abs(pose1_euler_orientation[2] - pose2_euler_orientation[2])
    return pos_distance < pos_tolerance_m and angular_distance < orientation_tolerance_rad

def send_maneuver_nav_goal(goal_pub, frame_id, start_pose, goal_pose):
    '''Sends a maneuver navigation goal from "start_pose" to "goal_pose"
    in the given frame using the given goal publisher.

    Keyword arguments:
    goal_pub: rospy.Publisher -- maneuver navigation goal publisher
    frame_id: str -- frame ID of the start and goal poses
    start_pose: geometry_msgs.msg.Pose -- start pose for the navigation
    goal_pose: geometry_msgs.msg.Pose -- goal pose for the navigation

    '''
    # we remove any leading or trailing / from the frame_id
    frame_id = frame_id.strip('/')

    nav_goal = ManeuverNavigationGoal()
    nav_goal.start.header.frame_id = frame_id
    nav_goal.start.header.stamp = rospy.Time.now()
    nav_goal.start.pose = start_pose

    nav_goal.goal.header.frame_id = frame_id
    nav_goal.goal.header.stamp = rospy.Time.now()
    nav_goal.goal.pose = goal_pose

    goal_pub.publish(nav_goal)

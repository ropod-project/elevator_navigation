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

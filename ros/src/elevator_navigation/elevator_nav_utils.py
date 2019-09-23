import rospy
from tf import transformations as tf

from geometry_msgs.msg import PoseWithCovarianceStamped
from maneuver_navigation.msg import Goal as ManeuverNavigationGoal
from ropod_ros_msgs.msg import NavElevatorFeedback, Status, Position
from cart_collection.cart_collection_utils import is_point_in_polygon

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

def is_polygon_in_polygon(polygon1, polygon2):
    '''
    Returns true if all points of polygon1 are contained within polygon2

    polygon1: geometry_msgs.msg.Polygon -- polygon which is supposed to be
                inside polygon2
    polygon2: list of ropod_ros_msgs.msg.Position -- polygon which should
                contain polygon1
    '''

    for p in polygon1.points:
        ropod_point = Position()
        ropod_point.x = p.x
        ropod_point.y = p.y
        if not is_point_in_polygon(ropod_point, polygon2.vertices):
            return False
    return True

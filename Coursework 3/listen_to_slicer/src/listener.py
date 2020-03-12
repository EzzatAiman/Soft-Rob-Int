#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from ros_igtl_bridge.msg import igtltransform
import geometry_msgs.msg
#from Ambra import Kindness
#from Mitton import Laughter
#from Lin import Wisdom
#from Ezzat import Wittiness
#from Gina import Ioannis


def getData(data):
    
    #rospy.loginfo(data.transform.translation.x)
    print "Current coordinates are: "
    print "x:", data.transform.translation.x
    print "y:", data.transform.translation.y
    print "z:", data.transform.translation.z
    

    """
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.

    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4

    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose

    return all_close(pose_goal, current_pose, 0.01)
    """
	

def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/IGTL_TRANSFORM_IN', igtltransform, getData)

    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

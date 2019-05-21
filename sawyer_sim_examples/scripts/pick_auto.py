#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import roslib
roslib.load_manifest('sawyer_sim_examples')
import tf
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):

  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):

  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    sys.argv.append('joint_states:=/robot/joint_states')
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pick_auto',
                    anonymous=True)


    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "right_arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    group.set_planner_id("RRTConnectkConfigDefault")

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)


    planning_frame = group.get_planning_frame()
    #print "============ Reference frame: %s" % planning_frame

    eef_link = group.get_end_effector_link()
    #print "============ End effector: %s" % eef_link

    group_names = robot.get_group_names()
    #print "============ Robot Groups:", robot.get_group_names()

    #print "============ Printing robot state"
    #print robot.get_current_state()
    #print ""


    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_to_joint_state(self):
    group = self.group

    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/6
    joint_goal[2] = 0
    joint_goal[3] = (2.7*pi)/4
    joint_goal[4] = 0
    joint_goal[5] = -pi/2
    joint_goal[6] = pi/2

    group.go(joint_goal, wait=True)

    group.stop()

    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose_goal(self):
    group = self.group

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = 1.0
    pose_goal.position.x = 0.883
    pose_goal.position.y = -0.082
    pose_goal.position.z = 0.854
    group.set_pose_target(pose_goal)

    plan = group.go(wait=True)

    group.stop()
    group.clear_pose_targets()

    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)




  def plan_cartesian_path(self, scale=1, x=0.75, y=0.0, z=0.2, zi=1.0):
    group = self.group
    waypoints = []
    
    '''
    wpose = group.get_current_pose().pose
    print wpose
    print '/n'
    '''

    wpose.orientation.z = zi
    wpose.position.x = x 
    wpose.position.y = y 
    wpose.position.z = z
    waypoints.append(copy.deepcopy(wpose))


    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    return plan, fraction

  def display_trajectory(self, plan):
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)

    display_trajectory_publisher.publish(display_trajectory);

  def execute_plan(self, plan):
    group = self.group
    group.execute(plan, wait=True)






def main():
  
  listener = tf.TransformListener()
  
  try:
    tutorial = MoveGroupPythonIntefaceTutorial()
    
    #print "============ Press `Enter` to execute a movement using a joint state goal ..."
    
    #tutorial.go_to_joint_state()
    
    try:            
        (position, rotation) = listener.lookupTransform('/world', '/object_2', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
        
    X = position[0]
    Y = position[1]
    Z = position[0] + 0.1
    Zi = rotation[2]
    
    print "Plan and display a Cartesian path ..."
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path(x=X, y=Y, z=Z, zi=Zi)
    
    print "Display a saved trajectory ..."
    raw_input()
    tutorial.display_trajectory(cartesian_plan)

    print "Execute a saved path ..."
    raw_input()
    tutorial.execute_plan(cartesian_plan)
    

   
    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()


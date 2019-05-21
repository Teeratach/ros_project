#!/usr/bin/env python
import argparse
import struct
import sys
import copy

import rospy ,tf
import rospkg
from math import pi

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

import intera_interface

#kinect_pose = Pose( position=Point(x=-1.25, y=0.0, z=1)
#orientation=Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0)))

def load_gazebo_models(table_pose=Pose(position=Point(x=0.75, y=0.0, z=0.0)),
                       table_reference_frame="world",
                       
                       block_pose=Pose(position=Point(x=0.8225, y=0.1265, z=0.7725)),
                       block_reference_frame="world",
                       
                       kinect_pose = Pose( position=Point(x=0.75, y=0.0, z=2.0), orientation=Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0)) ), 
                       kinect_reference_frame="world",
                       
                       tube_pose=Pose(position=Point(x=0.7225, y=0.1665, z=0.7725)),
                       tube_reference_frame="world",
                       
                       block_pose2=Pose(position=Point(x=0.6225, y=-0.2665, z=0.7725))
                       
                       ):
    
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('sawyer_sim_examples')+"/models/"
    
    
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    
    # Load kinect SDF
    kinect_xml = ''
    with open (model_path + "kinect/model.sdf", "r") as kinect_file:
        kinect_xml=kinect_file.read().replace('\n', '')
    
    # Load Block URDF
    block_xml = ''
    with open (model_path + "wood_cube_5cm/model.sdf", "r") as block_file:
        block_xml=block_file.read().replace('\n', '')
    
    # Load tube URDF
    tube_xml = ''
    with open (model_path + "tube_2_25cm/model.sdf", "r") as tube_file:
        tube_xml=tube_file.read().replace('\n', '')
        
        
        
        
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    
    # Spawn kinect SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("kinect", kinect_xml, "/",
                             kinect_pose, kinect_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    
    # Spawn wood_cube URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_urdf = spawn_urdf("wood_cube", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))
        
    # Spawn tube URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_urdf = spawn_urdf("tube", tube_xml, "/",
                               tube_pose, tube_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    # Spawn wood_cube2 URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_urdf = spawn_urdf("wood_cube2", block_xml, "/",
                               block_pose2, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))
        
        
        

def delete_gazebo_models():
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("wood_cube")
        resp_delete = delete_model("wood_cube2")
        resp_delete = delete_model("kinect")
        resp_delete = delete_model("tube")
    except rospy.ServiceException, e:
        print("Delete Model service call failed: {0}".format(e))

def main():

    rospy.init_node("Pose_and_table")

    load_gazebo_models()
    rospy.on_shutdown(delete_gazebo_models)

    rate = rospy.Rate(0.1)
    while not rospy.is_shutdown():
            print ''
            rate.sleep()
    return 0

if __name__ == '__main__':
    sys.exit(main())

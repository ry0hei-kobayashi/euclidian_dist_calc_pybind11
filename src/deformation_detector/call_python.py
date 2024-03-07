#!/usr/bin/env python
import rospy
import smach_ros
import smach

import numpy as np
from simpleicp import PointCloud, SimpleICP

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math


import rospkg
rospack = rospkg.RosPack()
current_path = rospack.get_path('deformation_detector')


class EuclidianCalcPython(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure'])

    def execute(self,userdata):
        try:
            #bf_array = np.copy(userdata.pcl_point_one)
            #af_array = np.copy(userdata.pcl_point_two)


            #//for debug, array from txt file//
            path = current_path + "/src/deformation_detector/data/"
            print(path)
            bf_array = np.loadtxt(path + "milk_carton_0.txt")
            af_array = np.loadtxt(path + "milk_carton_15.txt")

            #########################Filtered PointCloud by Threshold#################################
            #camera x:depth

            bf_depth_mean = np.mean(bf_array[:, 0])
            bf_filtered_indices = bf_array[:, 0] <= bf_depth_mean + 0.05
            bf_array_filtered = bf_array[bf_filtered_indices]
            
            af_depth_mean = np.mean(af_array[:, 0])
            af_filtered_indices = af_array[:, 0] <= af_depth_mean + 0.05 
            af_array_filtered = af_array[af_filtered_indices]
            

            bf_pos = bf_array_filtered[:, :3]
            af_pos = af_array_filtered[:, :3]
            bf_color = bf_array_filtered[:, 3:]
            af_color = af_array_filtered[:, 3:]

            ########################PointCloud Registration#################################
            pc_fix = PointCloud(bf_pos, columns=["x", "y", "z"])
            pc_mov = PointCloud(af_pos, columns=["x", "y", "z"])

            print('############################################')
            print(pc_fix)
            print('############################################')
            print(pc_mov)
            print('############################################')


            ### Create simpleICP object, add point clouds
            icp = SimpleICP()
            icp.add_point_clouds(pc_fix, pc_mov)
            H, X_mov_transformed, rigid_body_transformation_params, distance_residuals = icp.run(max_overlap_distance=0.1)

            rospy.logerr('############################################################################')
            rospy.logwarn(H)

            rospy.logerr('############################################################################')
            #rospy.logwarn(distance_residuals)
            #rospy.logerr('############################################################################')
            
            ###transformed
            rotation = H[:3, :3]
            #print(rotation)
            #bf_trans = np.dot(bf_pos, rotation.T)
            af_trans = X_mov_transformed
            #rospy.logwarn(af_trans)
            print(af_trans)


            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')

            ax.scatter(bf_pos[:, 0], bf_pos[:, 1], bf_pos[:, 2], c=bf_color/255.0, marker='o',s=0.1, label='before_pcl')
            ax.scatter(af_trans[:, 0], af_trans[:, 1], af_trans[:, 2], c=af_color/255.0, marker='o',s=0.1, label='after_pcl')

            ###for 3d figure, calculate centroids
            X = bf_pos[:,0] #change here camera_rgb_optical_frame
            Y = bf_pos[:,1]
            Z = bf_pos[:,2]

            max_range = np.array([X.max()-X.min(), Y.max()-Y.min(), Z.max()-Z.min()]).max() * 0.5

            mid_x = (X.max()+X.min()) * 0.5
            mid_y = (Y.max()+Y.min()) * 0.5
            mid_z = (Z.max()+Z.min()) * 0.5
            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_zlim(mid_z - max_range, mid_z + max_range)

            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_aspect('equal')


            plt.show()








            #euclidian distance calc python version
            from datetime import datetime
            start_time = datetime.now()

            distance = 0 
            for p in af_trans:
                d_temp = 10000
                for q in bf_pos:
                    d = np.linalg.norm( p - q )
                    if d < d_temp:
                        d_temp = d
            
                distance = distance + d_temp
                print(distance)

            end_time = datetime.now()
            elapsed_time = end_time - start_time

            rospy.logwarn('####time######')
            rospy.logwarn(elapsed_time)
            rospy.logwarn('####deform######')
            rospy.logwarn(distance)
            rospy.logwarn('##########')


            return 'success'

        except:
            import traceback
            rospy.logerr(traceback.format_exc())
            return 'failure'

if __name__ == '__main__':
    rospy.init_node('euclid_python')
    sm = smach.StateMachine(outcomes=['success','failure'])

    with sm:
        smach.StateMachine.add('Debug', EuclidianCalcPython(),
                transitions={'success': 'success',
                             'failure': 'failure'})
    sm.execute()
  


#!/usr/bin/env python3
import rospy
import numpy as np
import sys
import cv2 
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import actionlib
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image,CameraInfo
from image_geometry import PinholeCameraModel
from kortex_driver.srv import *
from kortex_driver.msg import *


class Robot:
    def __init__(self):
        self.ns = "my_gen3"
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/my_gen3/camera/camera/color/image_raw", Image, self.image_callback)
        #self.camera_info_sub = rospy.Subscriber("/my_gen3/camera/camera/color/camera_info", CameraInfo, self.camera_callback)
        self.vel_pub = rospy.Publisher('/my_gen3/in/joint_velocity',Base_JointSpeeds,queue_size=10,latch =True)
        self.center = []
        # self.info_msg = CameraInfo()
        # self.cam_model = PinholeCameraModel()
        # self.point_msg = geometry_msgs.msg.PoseStamped()
        self.listener = tf.TransformListener()
        self.planning_group = "arm"
        self.commander = moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander(robot_description = "/my_gen3/robot_description", ns = self.ns)
        self.scene = moveit_commander.PlanningSceneInterface(ns = self.ns)
        self.group = moveit_commander.MoveGroupCommander(self.planning_group,robot_description = "/my_gen3/robot_description", ns = self.ns)
        self.display_trajectory_publisher = rospy.Publisher('/my_gen3/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self.exectute_trajectory_client = actionlib.SimpleActionClient(
            '/my_gen3/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self.exectute_trajectory_client.wait_for_server()

        self.planning_frame = self.group.get_planning_frame()
        self.eef_link = self.group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        self.curr_state = self.robot.get_current_state()


    def publish_JV(self,joint_vel):

        joint1_vel = JointSpeed()
        joint1_vel.joint_identifier = 0
        joint1_vel.value = joint_vel[0]
        joint1_vel.duration = 0
        joint2_vel = JointSpeed()
        joint2_vel.joint_identifier = 1
        joint2_vel.value = joint_vel[1]
        joint2_vel.duration = 0
        joint3_vel = JointSpeed()
        joint3_vel.joint_identifier = 2
        joint3_vel.value = joint_vel[2]
        joint3_vel.duration = 0
        joint4_vel = JointSpeed()
        joint4_vel.joint_identifier = 3
        joint4_vel.value = joint_vel[3]
        joint4_vel.duration = 0
        joint5_vel = JointSpeed()
        joint5_vel.joint_identifier = 4
        joint5_vel.value = joint_vel[4]
        joint5_vel.duration = 0
        joint6_vel = JointSpeed()
        joint6_vel.joint_identifier = 5
        joint6_vel.value = joint_vel[5]
        joint6_vel.duration = 0
        joint7_vel = JointSpeed()
        joint7_vel.joint_identifier = 6
        joint7_vel.value = joint_vel[6]
        joint7_vel.duration = 0
        jointMsg = Base_JointSpeeds()
        jointMsg.joint_speeds = [joint1_vel,joint2_vel,joint3_vel,joint4_vel,joint5_vel,joint6_vel,joint7_vel]
        jointMsg.duration = 0
        self.vel_pub.publish(jointMsg)
        

    def calculateCameraVel(self, obj_center):
        f = 554.3827128226441
        #ImageJacob = np.array([[-f, 0, obj_center[0], (obj_center[0]*obj_center[1])/f, -(f**2 + ((obj_center[0]**2)/f)), obj_center[1]],[0, -f, obj_center[1], f + ((obj_center[1])/f), -(obj_center[0]*obj_center[1])/f, -1*obj_center[0]]])
        #ImageJacob = np.array([[-f, 0, obj_center[0], 0, 0, 0],[0, -f, obj_center[1],0, 0, 0]])
        ImageJacob = np.array([[0, 0, 0, (obj_center[0]*obj_center[1])/f, -(f**2 + ((obj_center[0]**2)/f)), obj_center[1]],[0, 0, 0, f + ((obj_center[1])/f), -(obj_center[0]*obj_center[1])/f, -1*obj_center[0]]])
        center_x = 320
        center_y = 240
        error = 100 * np.array([[center_x - obj_center[0]],[center_y - obj_center[1]]])
        inv_ImageJacob = np.linalg.pinv(ImageJacob)
        camera_vel =  np.dot(inv_ImageJacob, error)
        transformed_camvel = [0,-1*camera_vel[0],-1*camera_vel[1],0 ,0 ,0]
        print("ImageJacoba" + str(ImageJacob))
        print("INV ImageJacoba" + str(inv_ImageJacob))
        print("came vel: "+str(camera_vel))
        print("error: " + str(error))
        return transformed_camvel

        


    # def camera_callback(self,data):
    #     self.info_msg = data

    def image_callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print (e)
        self.center = self.getCenter(cv_image)
        image_drawn = cv2.circle(cv_image, (self.center[0],self.center[1]), radius=2, color=(0,0,255), thickness=-1)
        image_drawn = cv2.circle(cv_image, (int(cv_image.shape[1]/2),int(cv_image.shape[0]/2)), radius=2, color=(255,0,0), thickness=-1)
        cam_vel = self.calculateCameraVel(self.center)
        jacob = self.group.get_jacobian_matrix(self.group.get_current_joint_values())
        #print("Jacob:" + str(jacob))
        joint_vel = np.dot(np.linalg.pinv(jacob),cam_vel)
        #joint_vel = 10 * joint_vel
        print("joint vel:" + str(joint_vel))
        self.publish_JV(list(joint_vel))
        cv2.imshow("Center",image_drawn)
        # self.cam_model.fromCameraInfo(self.info_msg)
        # cam_model_point = self.cam_model.projectPixelTo3dRay(self.cam_model.rectifyPoint((self.center[0],self.center[1])))
        #rospy.loginfo("PIXEL CO-ORDINATES")
        #rospy.loginfo(str(self.center[0]-int(cv_image.shape[1]/2))+","+str(self.center[1]-int(cv_image.shape[0]/2)))
        #rospy.loginfo(str(int(cv_image.shape[1]/2))+","+str(int(cv_image.shape[0]/2)))
    # #    rospy.loginfo("3D RAy")
    # #    rospy.loginfo(cam_model_point)
    #     self.point_msg.pose.position.x = cam_model_point[0]
    #     self.point_msg.pose.position.y = cam_model_point[1]
    #     self.point_msg.pose.position.z = 1
    #     self.point_msg.pose.orientation.x = 0
    #     self.point_msg.pose.orientation.y = 0
    #     self.point_msg.pose.orientation.z = 0
    #     self.point_msg.pose.orientation.w = 1
    #     self.point_msg.header.stamp = rospy.Time.now()
    #     self.point_msg.header.frame_id = self.cam_model.tfFrame()
    #     self.listener.waitForTransform(self.cam_model.tfFrame(), "world", rospy.Time.now(), rospy.Duration(1.0))
    #     tf_point = self.listener.transformPose("world",self.point_msg)
    # #    rospy.loginfo("TF")
    # #    rospy.loginfo(tf_point)
        cv2.waitKey(1)

    def getCenter(self,image):
        upper_blue = np.array([128,255,255])
        lower_blue = np.array([90,50,70])
        hsv_img = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv_img, lower_blue, upper_blue)
        blue_res = cv2.bitwise_and(image,image, mask=blue_mask)
        blue_M = cv2.moments(blue_mask)
        blue_cX = int(blue_M["m10"] / blue_M["m00"])
        blue_cY = int(blue_M["m01"] / blue_M["m00"])
        return [blue_cX,blue_cY]

def main():
    
    #ic.color_thresholding(ic.image)
    rospy.init_node("node_obj_tracking",anonymous = True)
    ic = Robot()
    #ic.publish_JV()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()  

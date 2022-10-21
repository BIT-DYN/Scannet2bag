# -*- coding:UTF-8 -*-
import cv2
import time, sys, os
from ros import rosbag
import roslib
import rospy
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pcl2
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Transform
import numpy as np
from pyquaternion import Quaternion
import tf

# import ImageFile
from PIL import ImageFile
from PIL import Image as ImagePIL


def ReadImages(filename):
    all = []
    files = os.listdir(filename)
    all_num = 0
    for f in sorted(files):
        if os.path.splitext(f)[1] in ['.png']:
            all_num = all_num+1
    for i in range(all_num):
        all.append(filename+str(i)+'.png')
    return all


def ReadPose(filename):
    file = open(filename,'r')
    all = file.readlines()
    posedata = []
    for f in all:
        line = f.rstrip('\n').split(' ')
        line = np.array(line).astype(float).reshape(4,4)
        posedata.append(line)
    return posedata


def depth2xyz(depth_map,fx,fy,cx,cy,flatten=False,depth_scale=1000):
    h,w=np.mgrid[0:depth_map.shape[0],0:depth_map.shape[1]]
    z=depth_map/depth_scale
    x=(w-cx)*z/fx
    y=(h-cy)*z/fy
    xyz=np.dstack((x,y,z)) if flatten==False else np.dstack((x,y,z)).reshape(-1,3)
    #xyz=cv2.rgbd.depthTo3d(depth_map,depth_cam_matrix)
    return xyz


def CreateBag():#img,imu, bagname, timestamps
    '''read time stamps'''
    # 所转化场景的根目录
    root_dir = "/media/dyn/DYN1/Research/dataset/iSDF/seqs/scene0004_00/"
    # 返回所有图片的位置
    imgs = ReadImages(root_dir+"frames/depth/")
    print("The length of images:",len(imgs))
    imagetimestamps=[]
    # 返回一个列表，每个元素为一个4*4数组
    posedata = ReadPose(root_dir+"traj.txt") #the url of  IMU data
    print("The length of poses:",len(posedata))
    # 帧率30
    imagetimestamps = np.linspace(0+1.0/30.0,(len(imgs))/30.0, len(imgs))
    # print(imagetimestamps)
    # 所创建rosbag的目录和名称
    bag = rosbag.Bag("/media/dyn/DYN1/Research/dataset/iSDF/rosbag/scene_0004.bag", 'w')

    try:
        for i in range(len(posedata)):

            '''posestamped消息'''
            # posestamped = PoseStamped()
            # pose = Pose()
            # pose_numpy = posedata[i]
            # # 坐标直接写入
            # pose.position.x = pose_numpy[0,3]
            # pose.position.y = pose_numpy[1,3]
            # pose.position.z = pose_numpy[2,3]
            # # 四元数由旋转矩阵得到
            # rotate_numpy = pose_numpy[0:3,0:3]
            # # print(np.linalg.inv(rotate_numpy))
            # # print(rotate_numpy)
            # # q = Quaternion(matrix=rotate_numpy)
            # q = tf.transformations.quaternion_from_matrix(pose_numpy)
            # pose.orientation.x = q[0]
            # pose.orientation.y = q[1]
            # pose.orientation.z = q[2]
            # pose.orientation.w = q[3]
            # poseStamp = rospy.rostime.Time.from_sec(float(imagetimestamps[i]))
            # # print(poseStamp)
            # posestamped.header.stamp = poseStamp
            # posestamped.pose = pose
            # bag.write("/camera_pose",posestamped,poseStamp)

            '''tf消息'''
            tf_oxts_msg = TFMessage()
            tf_oxts_transform = TransformStamped() 
            oxts_tf = Transform()           
            tf_oxts_transform.header.stamp = rospy.rostime.Time.from_sec(float(imagetimestamps[i]))
            tf_oxts_transform.header.frame_id = '/world'
            tf_oxts_transform.child_frame_id = '/camera'
            pose_numpy = posedata[i]
            # 坐标直接写入
            oxts_tf.translation.x = pose_numpy[0,3]
            oxts_tf.translation.y = pose_numpy[1,3]
            oxts_tf.translation.z = pose_numpy[2,3]
            # 四元数由旋转矩阵得到
            rotate_numpy = pose_numpy[0:3,0:3]
            # print(np.linalg.inv(rotate_numpy))
            # print(rotate_numpy)
            # q = Quaternion(matrix=rotate_numpy)
            q = tf.transformations.quaternion_from_matrix(pose_numpy)
            oxts_tf.rotation.x = q[0]
            oxts_tf.rotation.y = q[1]
            oxts_tf.rotation.z = q[2]
            oxts_tf.rotation.w = q[3]
            tf_oxts_transform.transform = oxts_tf
            tf_oxts_msg.transforms.append(tf_oxts_transform)

            bag.write("/camera_pose",tf_oxts_transform,tf_oxts_transform.header.stamp)

        for i in range(len(imgs)):
            print("Adding %s" % imgs[i])

            raw_depth = cv2.imread(imgs[i], -1)
            [H, W] = raw_depth.shape

            raw_depth_np = raw_depth.astype(np.float32)
            pc = depth2xyz(raw_depth_np, 570.021362, 570.021362, 319.500000, 239.500000, flatten=True)
            # pc = pointcloud_from_depth_torch(depth, 570.021362, 570.021362, 319.500000, 239.500000)

            header = Header()
            header.frame_id = "/camera"
            header.stamp = rospy.Time.from_sec(float(imagetimestamps[i]))
            fields = [PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1),]

            pcl_msg = pcl2.create_cloud(header, fields, pc)

            bag.write('/camera_point', pcl_msg, pcl_msg.header.stamp)


    finally:
        bag.close()

if __name__ == "__main__":
    # 分别输入图像的文件夹、轨迹的文件夹、要保存的包位置
    # print(sys.argv)
    CreateBag()
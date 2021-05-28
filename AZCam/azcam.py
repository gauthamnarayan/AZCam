import rospy
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from franka_interface_msgs.srv import AZCamTransformMsg, AZCamTransformMsgRequest
import numpy as np
import cv2
import copy
import time
import open3d as o3d
import socket
import threading

class AZCam:
    """
    Class to provide images from the azure kinect camera. 
    
    Depth not supported as I am not aware of a way to rectify
    the rational_polynomial distorion model.
    Without rectification, real world experiment will fail as points cannot be
    projected back to 3D correctly without being rectified first in 2D
    """

    def __init__(self, rosnode_name='azcam_client', namespace='azcam_front', init_new_node=True, crop_rectified=True):
        if(init_new_node):
            rospy.init_node(rosnode_name)

        if(namespace == None):
            raise AssertionError("Namespace has to be defined. cameraMatrix associated with namespace and camera")

        self.namespace = namespace
        self.crop_rectified = crop_rectified
        cameraInfo = rospy.wait_for_message('/' + self.namespace + '/rgb/camera_info', CameraInfo, timeout=5)
        self.cameraMatrix = np.array(cameraInfo.P).astype(np.float64).reshape([3,4])
        self.imageHeight = cameraInfo.height
        self.imageWidth = cameraInfo.width
        print("done with init")

    def getCurrentIRImage(self):
        """
        Obtain latest IR Image from ROS.
        """
        msg = rospy.wait_for_message('/' + self.namespace + '/ir/image_raw', Image, timeout=5)
        image = self.imgMsg2cv2(msg) 

        # TODO: Fix resize image
        # x = image.shape[1]
        # y = image.shape[0]
        # cx = x/2 
        # # Square and resize the image
        # image = image[:, int(cx-y/2) : int(cx+y/2), :]
        # image = cv2.resize(image, (480,480))
        return image

    def getCurrentRGBImage480x480(self):
        """
        Obtain latest 480x480 sized RGB Image from ROS.
        """
        msg = rospy.wait_for_message('/' + self.namespace + '/rgb/image_rect_color', Image, timeout=5)
        image = self.imgMsg2cv2(msg) 

        x = image.shape[1]
        y = image.shape[0]
        cx = x/2 
        # Square and resize the image
        image = image[:, int(cx-y/2) : int(cx+y/2), :]
        image = cv2.resize(image, (480,480))
        return image

    def _getRGBVideo480x480(self):
        trjImages = []
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(('127.0.0.1',5416))
        s.settimeout(0.01)

        stop = False
        while not stop:
            try:
                msg = s.recv(4096)
                if(msg == b'stop'):
                    stop = True
            except:
                rgbImage = self.getCurrentRGBImage480x480()
                depthImage = self.getCurrentDepthImage480x480()
                trjImages.append([rgbImage, depthImage])
                self.trjImages = trjImages

    def _createSocketConn(self):
        s = socket.socket()
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        host = "127.0.0.1"
        port = 5416
        s.bind((host, port))
        s.listen(1)
        conn, addr = s.accept()  
        self.conn = conn
    
    def startRGBVideo480x480(self, videoImgIdx=0):
        """
        Start collecting a video asynchronously.
        """
        self.videoImgIdx = videoImgIdx
        th1 = threading.Thread(target=self._createSocketConn)
        th2 = threading.Thread(target=self._getRGBVideo480x480)
        th1.start()
        time.sleep(0.1)
        th2.start()
        th1.join()

    def stopRGBVideo480x480(self):
        """
        Stop collecting a video asynchronously.
        """
        self.conn.sendall(b'stop')

    def getLatestRGBVideo480x480(self):
        """
        Obtain collected video after starting and stopping RGB video collection.
        """
        rgbVideo = [x[0] for x in self.trjImages]
        rgbVideo = np.array(rgbVideo)
        return rgbVideo

    def saveLatestRGBVideo480x480(self, videoDir, idx=0):
        """
        Save collected video after starting and stopping RGB video collection.
        """
        rgbVideo = self.getLatestRGBVideo480x480()
        for i in range(rgbVideo.shape[0]):
            if(idx != 0):
                cv2.imwrite(videoDir + "/rgb_" + str(idx + i) + ".jpg", rgbVideo[i])
            else:
                cv2.imwrite(videoDir + "/rgb_" + str(i) + ".jpg", rgbVideo[i])
        return idx + i

    def getLatestRGBDVideo480x480(self):
        """
        Obtain collected RGBD after starting and stopping video collection.
        """
        rgbVideo = [x[0] for x in self.trjImages]
        rgbVideo = np.array(rgbVideo)
        depthVideo = [x[1] for x in self.trjImages]
        depthVideo = np.array(depthVideo)
        return rgbVideo, depthVideo

    def saveLatestRGBDVideo480x480(self, videoDir, idx=None):
        """
        Save latest collected RGBD video after starting and stopping RGB video collection.
        """
        rgbVideo, depthVideo = self.getLatestRGBDVideo480x480()
        for i in range(rgbVideo.shape[0]):
            if(idx):
                cv2.imwrite(videoDir + "/rgb_" + str(idx + i) + ".jpg", rgbVideo[i])
                cv2.imwrite(videoDir + "/depth_" + str(idx + i) + ".jpg", depthVideo[i])
            else:
                cv2.imwrite(videoDir + "/rgb_" + str(i) + ".jpg", rgbVideo[i])
                cv2.imwrite(videoDir + "/depth_" + str(i) + ".jpg", depthVideo[i])
        return idx + i

    def getCurrentRGBImage(self):
        """
        Provides RGB image at current time at original resolution.
        """
        msg = rospy.wait_for_message('/' + self.namespace + '/rgb/image_rect_color', Image, timeout=0.01)
        image = self.imgMsg2cv2(msg) 
        return image

    def getCurrentDepthImage480x480(self):
        msg = rospy.wait_for_message('/' + self.namespace + '/depth_uint8_mm/image_rect', Image, timeout=5)
        image = self.imgMsg2cv2(msg) 
        image = image[:,280:1000]
        image = cv2.resize(image, (480,480))
        return image

    def getCurrentDepthImage480x480_test(self):
        # This is for testing purposes only. Need to understand how to obtain depth images in mm instead of cm.
        msg = rospy.wait_for_message('/' + self.namespace + '/depth_to_rgb/image_raw', Image, timeout=5)
        image = self.imgMsg2cv2(msg) 
        image = image[:,280:1000]
        image = cv2.resize(image, (480,480))
        return image

    def getCurrentDepthImage(self):
        """
        Provides Depth image at current time.
        """
        msg = rospy.wait_for_message('/' + self.namespace + '/depth_uint8_mm/image_rect', Image, timeout=5)
        image = self.imgMsg2cv2(msg) 
        return image

    def unscalePixelCoords(self, pixelCoords, rgbImage):
        """
        This function can provide pixel coordinates in the original(before resizing) image coordinate system.
        """
        assert(rgbImage.shape[0] == rgbImage.shape[1]), "Only square images supported now"
        newCoords = np.array(copy.deepcopy(pixelCoords))
        ratio = float(self.imageHeight)/rgbImage.shape[0]
        newCoords = newCoords * ratio
        newCoords[1] += 280 # To re-align the x axis crop.
        newCoords = newCoords.astype(np.int32) #Important to have everything in int, used as indices.
        return newCoords

    def getCamera3DPoint(self, cam2DPt):
        """
        Obtain 3D points in the camera's reference frame.
        """
        pcd = self.getCurrentPointCloud2()
        points = np.array(pcd.points)
        points = points.reshape(self.imageHeight, self.imageWidth, 3)
        return points[cam2DPt[0], cam2DPt[1]]

    def getWorld3DPoint(self, cam3DPt, srcFrame=None, tgtFrame=None):
        """
        Obtain 3D points in world coordinate system.
        """
        print("Looking for transform")
        rospy.wait_for_service('franka_transform_lookup')
        print("Done waiting for service")
        transformSrv = rospy.ServiceProxy('franka_transform_lookup', AZCamTransformMsg)
        req = AZCamTransformMsgRequest()
        req.cameraX = cam3DPt[0]
        req.cameraY = cam3DPt[1]
        req.cameraZ = cam3DPt[2]

        if(srcFrame == None and tgtFrame == None):
            req.sourceFrame = self.namespace + '/' + 'rgb_camera_link'
            req.targetFrame = 'panda_link0'
        else:
            req.sourceFrame = srcFrame
            req.targetFrame = tgtFrame 

        msg = transformSrv(req)
        world3DPt = np.array([msg.worldX, msg.worldY, msg.worldZ])
        print("Received transform")
        return world3DPt

    def getRGBfromPointCloud2(self):
        """
        Obtain RGB Images from the pointcloud.
        This is sometimes usefull to segment directly on the pointcloud.
        """
        pcd = self.getCurrentPointCloud2()
        rgb = np.array(pcd.colors) * 255 
        rgb = rgb.reshape(self.imageHeight, self.imageWidth, 3).astype(np.uint8)
        return rgb

    def getDepthfromPointCloud2(self):
        """
        Obtain depth images from the pointcloud.
        Warning: Do not use this method, as the depth image has holes and can lead to robot going to undetermined positions.
        """
        pcd = self.getCurrentPointCloud2()
        points = np.array(pcd.points) 
        depthImage = points.reshape(self.imageHeight, self.imageWidth, 3)
        return depthImage

    def getCurrentPointCloud2(self):
        """
        Provides pointcloud from latest available ROS message as an Open3D vector.
        Use numpy.asarray(pcd.points) to convert to numpy
        """
        msg = rospy.wait_for_message('/' + self.namespace + '/points2', PointCloud2, timeout=5)

        from pypcd import pypcd
        cloud = pypcd.PointCloud.from_msg(msg)
        cloudData = cloud.pc_data.view(np.float32).reshape(cloud.pc_data.shape + (-1,))
        xyz = cloudData[:, :3]
        rgb = pypcd.decode_rgb_from_pcl(cloud.pc_data['rgb'])

        rgb = rgb.astype(np.float64) / rgb.max() # Open3d color range[0,1]
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd.colors = o3d.utility.Vector3dVector(rgb)
        return pcd

    def getThresholdedPointCloud(self, pcd, thresholds):
        """
        Color based thresholding
        :param thresholds: numpy array [[r_upper, r_lower], [g_upper, g_lower], [b_upper, b_lower]]
        """
        xyz = np.array(pcd.points)
        rgb = np.array(pcd.colors)

        condition = ((rgb[:,0]>=thresholds[0,0]) & (rgb[:,0]<=thresholds[0,1]) &
                     (rgb[:,1]>=thresholds[1,0]) & (rgb[:,1]<=thresholds[1,1]) &
                     (rgb[:,2]>=thresholds[2,0]) & (rgb[:,2]<=thresholds[2,1]))

        pcd.points = o3d.utility.Vector3dVector(xyz[condition])
        pcd.colors = o3d.utility.Vector3dVector(rgb[condition])
        return pcd


    def displayPointCloud(self, pcd):
        """
        Displays point cloud.
        Known issues - sometimes the point cloud looks empty, just display again
        and it will work on repeated attempts. Need to debug why this happens.
        """
        print("Displaying point cloud of size: ", np.array(pcd.points).shape)
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(pcd)
        ctr = vis.get_view_control()
        parameters = o3d.io.read_pinhole_camera_parameters("/home/gautham/programs/RPAD/trials/franka_experiments/ScreenCamera_azcam.json")
        ctr.convert_from_pinhole_camera_parameters(parameters)
        vis.run()
        vis.destroy_window()

    def savePointCloud(self, filename,  pcd):
        print("Saving point cloud of size: %s as %s" % (str(np.array(pcd.points).shape), filename))
        o3d.io.write_point_cloud(filename, pcd)

    def imgMsg2cv2(self, msg):
        """
        Convert a ROS Image msg to cv2 array
        """
        if(msg.encoding == 'bgra8'):
            image = np.frombuffer(msg.data, dtype=np.uint8)
            image = image.reshape([msg.height, msg.width, 4])
            image = image[:,:,:3] # We need only RGB channels, not alpha channel
        elif(msg.encoding == 'mono8'):
            image = np.frombuffer(msg.data, dtype=np.uint8)
            image = image.reshape([msg.height, msg.width])
        elif(msg.encoding == '32FC1'):
            image = np.frombuffer(msg.data, dtype=np.float32)
            image = image.reshape([msg.height, msg.width])
        elif(msg.encoding == 'mono16'):
            image = np.frombuffer(msg.data, dtype=np.float16)
            image = image.reshape([msg.height, msg.width])
        else:
            raise AssertionError("Image encoding != bgra8, 32FC1, currently support only bgra8, 32FC1")
        return image

    def displayImage(self, image):
        cv2.imshow("image", image)
        cv2.waitKey()

def main():
    azcam = AZCam()
    while(True):
        image = azcam.getCurrentImage(image_size=(1000,1000))
        cv2.imshow("image", image)
        cv2.waitKey(1)

if __name__ == '__main__':
    main()




#!/usr/bin/env python3
# Persona 5 — Detección de Duckiebot precedente
# CORRECCIÓN: usa ratio del patrón en vez de solo detección binaria
#             publica al tópico unificado safety/block (no conduccion/block)
import rospy
import cv2
import os
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import BoolStamped
from cv_bridge import CvBridge

class VehicleNode(DTROS):
    def __init__(self, node_name):
        super().__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        veh = os.environ['VEHICLE_NAME']
        self.freq = rospy.get_param('~process_frequency', 2.0)
        self.threshold = rospy.get_param('~detection_threshold', 0.05)
        self.bridge = CvBridge()
        self.last_stamp = rospy.Time.now()
        self.blocking = False

        # Blob detector para círculos del patrón trasero
        params = cv2.SimpleBlobDetector_Params()
        params.minArea = 10
        params.minDistBetweenBlobs = 2
        self.detector = cv2.SimpleBlobDetector_create(params)

        self.image_sub = rospy.Subscriber(f'/{veh}/camera_node/image/compressed', CompressedImage, self._cb_img, queue_size=1)
        self.pub_block = rospy.Publisher(f'/{veh}/safety/block', BoolStamped, queue_size=1)

    def _cb_img(self, msg):
        now = rospy.Time.now()
        if (now - self.last_stamp).to_sec() < 1.0/self.freq:
            return
        self.last_stamp = now

        img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        h, w = img.shape[:2]

        detected, centers = cv2.findCirclesGrid(
            img, patternSize=(7,3),
            flags=cv2.CALIB_CB_SYMMETRIC_GRID,
            blobDetector=self.detector)

        should_block = False
        if detected and centers is not None:
            xs = [p[0][0] for p in centers]
            ratio = (max(xs) - min(xs)) / w
            should_block = ratio >= self.threshold

        if should_block != self.blocking:
            self.blocking = should_block
            out = BoolStamped()
            out.header.stamp = rospy.Time.now()
            out.data = should_block
            self.pub_block.publish(out)
            rospy.loginfo(f'Vehicle: {"bloqueando" if should_block else "liberando"}')

if __name__ == '__main__':
    node = VehicleNode('vehicle_node')
    rospy.spin()


#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import json
import threading
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, Range
from cv_bridge import CvBridge

from duckietown.dtros import DTROS, NodeType, TopicType
from dt_robot_utils import get_robot_name

from flask import Flask, jsonify, Response, render_template
from flask_cors import CORS

app = Flask(__name__, template_folder='templates')
CORS(app)

# Memoria compartida expandida para incluir el ToF
global_data = {
    "raw_frame": None,
    "hsv_frame": None,
    "qr_json": {},
    "logs": [],
    "tof_dist_cm": "--",
    "tof_alert": False
}

def add_log(msg, type="info"):
    global_data["logs"].insert(0, {"type": type, "msg": msg})
    if len(global_data["logs"]) > 15:
        global_data["logs"].pop()

class QRCodeDetectorNode(DTROS):
    def __init__(self, node_name):
        super(QRCodeDetectorNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        robot = get_robot_name()
        
        self.detector = cv2.QRCodeDetector()
        self.bridge = CvBridge()
        
        # Filtro HSV 
        self.lower_color = np.array([0, 120, 70])
        self.upper_color = np.array([10, 255, 255])
        
        # 1. Conexión a la cámara real del Duckiebot
        self.sub_img = rospy.Subscriber(
            f'/{robot}/camera_node/image/compressed', CompressedImage, self.image_cb, queue_size=1, buff_size="10MB"
        )
        
        # 2. Conexión al sensor ToF para alimentar el Dashboard
        self.sub_tof = rospy.Subscriber(
            f'/{robot}/front_center_tof_driver_node/range', Range, self.tof_cb, queue_size=1
        )
        
        self.pub_qr = rospy.Publisher(
            "~qr_data", String, queue_size=10, dt_topic_type=TopicType.PERCEPTION
        )
        
        msg = f"Nodo QR conectado al robot [{robot}] con telemetría ToF."
        self.loginfo(msg)
        add_log(msg, "success")

    def tof_cb(self, msg):
        # Procesar distancia para el frontend
        if msg.range < msg.max_range:
            dist_cm = msg.range * 100
            global_data["tof_dist_cm"] = f"{dist_cm:.1f}"
            global_data["tof_alert"] = dist_cm <= 7.0 # Alerta visual en el dashboard
        else:
            global_data["tof_dist_cm"] = "Libre"
            global_data["tof_alert"] = False

    def image_cb(self, msg):
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_color, self.upper_color)
            hsv_result = cv2.bitwise_and(img, img, mask=mask)
            
            _, jpeg_raw = cv2.imencode('.jpg', img)
            _, jpeg_hsv = cv2.imencode('.jpg', hsv_result)
            global_data["raw_frame"] = jpeg_raw.tobytes()
            global_data["hsv_frame"] = jpeg_hsv.tobytes()
            
            data, bbox, _ = self.detector.detectAndDecode(img)
            
            if data:
                try:
                    qr_json = json.loads(data)
                    global_data["qr_json"] = qr_json
                    self.pub_qr.publish(String(data))
                    add_log("QR detectado y decodificado exitosamente", "success")
                except json.JSONDecodeError:
                    add_log("QR detectado, pero no es un JSON válido", "warn")
                
        except Exception as e:
            add_log(f"Error procesando cámara: {e}", "error")

# ================= FLASK API =================

def generate_frames(frame_type):
    while True:
        frame = global_data.get(frame_type)
        if frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/video_raw')
def video_raw():
    return Response(generate_frames("raw_frame"), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/video_hsv')
def video_hsv():
    return Response(generate_frames("hsv_frame"), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/data')
def get_data():
    return jsonify({
        "qr_json": global_data["qr_json"],
        "logs": global_data["logs"],
        "tof_dist_cm": global_data["tof_dist_cm"],
        "tof_alert": global_data["tof_alert"]
    })

if __name__ == "__main__":
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, threaded=True), daemon=True).start()
    node = QRCodeDetectorNode("qr_detector_node")
    rospy.spin()

    #comentario
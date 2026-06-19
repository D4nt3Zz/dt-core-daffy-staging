#!/usr/bin/env python3
# Persona 5 — Parada por sensor ToF frontal
# CORRECCIÓN: solo reporta cambios de estado, no publica en loop
import rospy
import os
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import Range
from duckietown_msgs.msg import BoolStamped
from dt_robot_utils import get_robot_name

class ToFNode(DTROS):
    def __init__(self, node_name):
        super().__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        robot = get_robot_name()
        self.umbral = rospy.get_param('~distancia_umbral', 0.07)  # 7 cm
        self.estado_actual = False

        self.pub_block = rospy.Publisher(f'/{robot}/safety/block', BoolStamped, queue_size=1)
        rospy.Subscriber(f'/{robot}/front_center_tof_driver_node/range', Range, self._cb_range, queue_size=1)

    def _cb_range(self, msg):
        # msg.range = inf si no hay objeto dentro del rango máximo
        demasiado_cerca = (msg.range < msg.max_range and msg.range <= self.umbral)

        # Solo publicar si hay cambio de estado (evita spam)
        if demasiado_cerca != self.estado_actual:
            self.estado_actual = demasiado_cerca
            out = BoolStamped()
            out.header.stamp = rospy.Time.now()
            out.data = demasiado_cerca
            self.pub_block.publish(out)
            rospy.loginfo(f'ToF: objeto {"detectado" if demasiado_cerca else "liberado"} a {msg.range:.3f}m')

if __name__ == '__main__':
    node = ToFNode('tof_node')
    rospy.spin()


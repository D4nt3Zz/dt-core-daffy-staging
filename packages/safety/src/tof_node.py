#!/usr/bin/env python3
# Persona 5 — Parada por sensor ToF frontal con lectura en CM
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
        
        # Umbral de seguridad configurado a 0.07 metros (7 cm)
        self.umbral = rospy.get_param('~distancia_umbral', 0.07)  
        self.estado_actual = False

        self.pub_block = rospy.Publisher(f'/{robot}/safety/block', BoolStamped, queue_size=1)
        rospy.Subscriber(f'/{robot}/front_center_tof_driver_node/range', Range, self._cb_range, queue_size=1)

    def _cb_range(self, msg):
        # Si el sensor no detecta nada, arroja infinito. Lo ignoramos para no ensuciar la pantalla.
        if msg.range >= msg.max_range:
            return 

        # 1. CONVERSIÓN A CENTÍMETROS
        distancia_cm = msg.range * 100

        # Evalúa si la distancia es menor o igual al umbral (7 cm)
        demasiado_cerca = (msg.range <= self.umbral)

        # 2. IMPRESIÓN EN PANTALLA (Controlada por tiempo)
        # throttle(0.5, ...) significa que solo imprimirá este mensaje como máximo 2 veces por segundo
        if demasiado_cerca:
            rospy.logwarn_throttle(0.5, f"¡ALERTA! Objeto bloqueando la pista a: {distancia_cm:.1f} cm")
        else:
            # Si quieres ver la distancia todo el tiempo, incluso cuando está lejos:
            rospy.loginfo_throttle(1.0, f"Camino libre. Objeto más cercano detectado a: {distancia_cm:.1f} cm")

        # 3. LÓGICA DE BLOQUEO PARA EL MISSION CONTROLLER (Solo publica si cambia de estado)
        if demasiado_cerca != self.estado_actual:
            self.estado_actual = demasiado_cerca
            out = BoolStamped()
            out.header.stamp = rospy.Time.now()
            out.data = demasiado_cerca
            self.pub_block.publish(out)
            
            estado_str = "FRENANDO MOTOR" if demasiado_cerca else "LIBERANDO MOTOR"
            rospy.loginfo(f"[{estado_str}] Cambio de estado enviado al bus de seguridad.")

if __name__ == '__main__':
    node = ToFNode('tof_node')
    rospy.spin()

    #comentario
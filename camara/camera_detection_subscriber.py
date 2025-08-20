#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32

def detect_yellow(frame):
    # Convertir el frame de BGR a HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Definir rango para el color amarillo en HSV
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])

    # Crear una máscara para el color amarillo
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Obtener dimensiones del frame
    height, width, _ = frame.shape

    # Definir proporciones para el área central más grande
    vertical_split_left = width // 4
    vertical_split_right = 3 * width // 4
    horizontal_split_top = height // 4
    horizontal_split_bottom = 3 * height // 4

    # Extraer solo la parte central del frame para la detección
    central_area = mask[horizontal_split_top:horizontal_split_bottom, vertical_split_left:vertical_split_right]

    # Dividir la parte central en tres secciones verticales
    central_width = vertical_split_right - vertical_split_left
    section_width = central_width // 3

    left_section = central_area[:, :section_width]
    center_section = central_area[:, section_width:2*section_width]
    right_section = central_area[:, 2*section_width:]

    # Sumar los valores de los pixeles en cada sección
    left_sum = np.sum(left_section)
    center_sum = np.sum(center_section)
    right_sum = np.sum(right_section)

    # Determinar la posición del color amarillo
    if left_sum == 0 and center_sum == 0 and right_sum == 0:
        position = 2  # No se detectó amarillo
    elif left_sum > center_sum and left_sum > right_sum:
        position = -1  # Amarillo detectado en la izquierda
    elif center_sum > left_sum and center_sum > right_sum:
        position = 0  # Amarillo detectado en el centro
    else:
        position = 1  # Amarillo detectado en la derecha

    return position, mask  # También devuelve la máscara para la visualización

def color_detection(image_msg):
    try:
        # Convertir el mensaje de imagen de ROS a un frame de OpenCV
        frame = CvBridge().imgmsg_to_cv2(image_msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)
        return

    # Realizar la detección de color
    position, mask = detect_yellow(frame)

    # Mostrar la imagen y la máscara
    cv2.imshow("Original Image", frame)
    cv2.imshow("Mask", mask)

    # Publicar el resultado en el tópico "arduino_control"
    arduino_pub.publish(position)

    # Esperar por la entrada del teclado y limpiar
    cv2.waitKey(1)

def color_detection_node():
    rospy.init_node('color_detection_node', anonymous=True)

    # Suscribirse al tópico donde se publica la imagen de la cámara
    rospy.Subscriber("camera/image_raw", Image, color_detection)

    # Publicar en el tópico "arduino_control"
    global arduino_pub
    arduino_pub = rospy.Publisher("arduino_control", Int32, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    color_detection_node()

    # Limpiar ventanas de OpenCV al finalizar
    cv2.destroyAllWindows()

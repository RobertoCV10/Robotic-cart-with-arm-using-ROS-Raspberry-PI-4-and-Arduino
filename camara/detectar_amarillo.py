import cv2
import numpy as np

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
        position = "No se detectó amarillo"
    elif left_sum > center_sum and left_sum > right_sum:
        position = 1
    elif center_sum > left_sum and center_sum > right_sum:
        position = 0
    else:
        position = -1

    return position

def main():
    # Capturar video desde la cámara
    cap = cv2.VideoCapture(0)

    while True:
        # Leer el frame
        ret, frame = cap.read()
        if not ret:
            break

        # Detectar color amarillo y obtener la posición
        position = detect_yellow(frame)

        # Mostrar la posición
        if position == 1:
            print("Amarillo detectado en la izquierda")
        elif position == 0:
            print("Amarillo detectado en el centro")
        elif position == -1:
            print("Amarillo detectado en la derecha")
        else:
            print(position)

        # Mostrar el frame con las divisiones (opcional)
        height, width, _ = frame.shape
        vertical_split_left = width // 4
        vertical_split_right = 3 * width // 4
        horizontal_split_top = height // 4
        horizontal_split_bottom = 3 * height // 4

        # Dibujar líneas de división
        cv2.line(frame, (vertical_split_left, horizontal_split_top), (vertical_split_left, horizontal_split_bottom), (0, 255, 0), 2)
        cv2.line(frame, (vertical_split_right, horizontal_split_top), (vertical_split_right, horizontal_split_bottom), (0, 255, 0), 2)
        cv2.line(frame, (vertical_split_left, horizontal_split_top), (vertical_split_right, horizontal_split_top), (0, 255, 0), 2)
        cv2.line(frame, (vertical_split_left, horizontal_split_bottom), (vertical_split_right, horizontal_split_bottom), (0, 255, 0), 2)

        # Mostrar el frame
        cv2.imshow('Frame', frame)

        # Salir con la tecla 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Liberar el objeto de captura y cerrar las ventanas
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

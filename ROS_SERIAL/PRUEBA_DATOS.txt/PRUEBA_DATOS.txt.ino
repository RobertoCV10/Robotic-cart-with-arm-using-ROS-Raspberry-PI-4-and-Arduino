#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

char command;

ros::Subscriber<std_msgs::Char> sub("arduino_commands", &commandCallback);

void setup() {
  Serial.begin(57600); // Inicia la comunicación serial a 9600 baudios
  nh.initNode(); // Inicializa el nodo ROS
  Serial.println("Esperando informacion desde ROS");
  
}

void loop() {
  if (Serial.available() > 0) { // Si hay datos disponibles para leer
    command = Serial.read(); // Lee el caracter recibido
    
    // Activa las funciones según el comando recibido
    if (command == 'f') {
      avanzar();
    } else if (command == 'b') {
      retroceder();
    } else if (command == 'l') {
      izquierda();
    } else if (command == 'r') {
      derecha();
    }
  }
  
  nh.spinOnce(); // Mantén la comunicación con ROS
  delay(100); // Pequeña pausa para estabilidad
}

void avanzar() {
  Serial.println("Avanzar");
  // Aquí iría el código para mover el carrito hacia adelante
}

void retroceder() {
  Serial.println("Retroceder");
  // Aquí iría el código para mover el carrito hacia atrás
}

void izquierda() {
  Serial.println("Izquierda");
  // Aquí iría el código para girar el carrito a la izquierda
}

void derecha() {
  Serial.println("Derecha");
  // Aquí iría el código para girar el carrito a la derecha
}

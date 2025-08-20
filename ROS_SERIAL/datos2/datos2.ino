#include <ros.h>
#include <std_msgs/Char.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

// pienes motor y encoder
const byte Encoder_C1 = 3; // Cable amarillo pin 3 digital
const byte Encoder_C2 = 4; // Cable verde al pin 4 digital
const byte Motor_Enable = 5; // Pin 5 digital para habilitar el puente H
const byte Motor_Input1 = 6; // Pin 6 digital para control de dirección del puente H
const byte Motor_Input2 = 7; // Pin 7 digital para control de dirección del puente H


byte Encoder_C1Last;
int paso;
boolean direccion;

void commandCallback(const std_msgs::Char& msg) {
    char command = msg.data;
    
    // Procesar el comando recibido
    switch(command) {
        case 'w':
            avanzar();
            break;
        case 's':
            retroceder();
            break;
        case 'a':
            izquierda();
            break;
        case 'd':
            derecha();
            break;
        default:
            alto();
            break;
    }
}

ros::Subscriber<std_msgs::Char> sub("arduino_commands", &commandCallback);

void setup() {
    nh.initNode();
    nh.subscribe(sub);
    Serial.begin(57600);

    pinMode(Motor_Enable, OUTPUT);
    pinMode(Motor_Input1, OUTPUT);
    pinMode(Motor_Input2, OUTPUT);
  
    // Inicialmente, deshabilitamos el puente H
    digitalWrite(Motor_Enable, LOW);
}

void loop() {
    nh.spinOnce();
    // interrupcion para el encoder
    pinMode(Encoder_C2, INPUT);
    attachInterrupt(digitalPinToInterrupt(3), calculapulso, CHANGE);
    
    mostrarInformacion(); // Mostrar información actual del motor  
}


void avanzar() {
  Serial.println("Avanzar");
  digitalWrite(Motor_Enable, HIGH);
  digitalWrite(Motor_Input1, HIGH);
  digitalWrite(Motor_Input2, LOW);
  // Aquí iría el código para mover el carrito hacia adelante
}

void retroceder() {
  Serial.println("Retroceder");
  digitalWrite(Motor_Enable, HIGH);
  digitalWrite(Motor_Input1, LOW);
  digitalWrite(Motor_Input2, HIGH);
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

void alto() {
  Serial.println("STOP");
  delay(2000);
}


//FUNCIONES PARA DATOS DEL MOTOR

void calculapulso()
{
  int Lstate = digitalRead(Encoder_C1);
  if ((Encoder_C1Last == LOW) && Lstate == HIGH)
  {
    int val = digitalRead(Encoder_C2);
    if (val == LOW && direccion)
    {
      direccion = false; // Atrás
    }
    else if (val == HIGH && !direccion)
    {
      direccion = true;  // Adelante
    }
  }
  Encoder_C1Last = Lstate;

  if (!direccion)  paso++;
  else  paso--;
}

void mostrarInformacion() {
  Serial.print("Paso: ");
  Serial.print(paso);
  Serial.print(", ");
  Serial.print("Dirección: ");
  Serial.println(direccion ? "Adelante" : "Atras");
}

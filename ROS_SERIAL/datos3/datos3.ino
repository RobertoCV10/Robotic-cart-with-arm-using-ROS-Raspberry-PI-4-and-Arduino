#include <ros.h>
#include <std_msgs/Char.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

// MOTOR A
const byte EncoderA_C1 = 3; // Cable amarillo pin 3 digital
const byte EncoderA_C2 = 4; // Cable verde al pin 4 digital
const byte MotorA_Enable = 5; // Pin 5 digital para habilitar el puente H
const byte MotorA_Input1 = 6; // Pin 6 digital para control de dirección del puente H
const byte MotorA_Input2 = 7; // Pin 7 digital para control de dirección del puente H

// MOTOR B
const byte EncoderB_C1 = 18; // Cable amarillo pin 3 digital
const byte EncoderB_C2 = 17; // Cable verde al pin 4 digital
const byte MotorB_Enable = 21; // Pin 5 digital para habilitar el puente H
const byte MotorB_Input1 = 20; // Pin 6 digital para control de dirección del puente H
const byte MotorB_Input2 = 19; // Pin 7 digital para control de dirección del puente H

byte EncoderA_C1Last;
byte EncoderB_C1Last;

int PasoA;
int PasoB;
boolean DireccionA;
boolean DireccionB;

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

    //motor A
    pinMode(MotorA_Enable, OUTPUT);
    pinMode(MotorA_Input1, OUTPUT);
    pinMode(MotorA_Input2, OUTPUT);
    //motor B
    pinMode(MotorA_Enable, OUTPUT);
    pinMode(MotorA_Input1, OUTPUT);
    pinMode(MotorA_Input2, OUTPUT);
  
    // Inicialmente, deshabilitamos el puente H
    digitalWrite(MotorA_Enable, LOW);
    digitalWrite(MotorB_Enable, LOW);
}

void loop() {
    nh.spinOnce();
    // interrupcion para el encoder A
    pinMode(EncoderA_C2, INPUT);
    attachInterrupt(digitalPinToInterrupt(3), CalculoPulsoA, CHANGE);
    
    // interrupcion para el encoder B
    pinMode(EncoderB_C2, INPUT);
    attachInterrupt(digitalPinToInterrupt(18), CalculoPulsoB, CHANGE);

    mostrarInformacion(); // Mostrar información actual del motor  
}


void avanzar() {
  digitalWrite(MotorA_Enable, HIGH);
  digitalWrite(MotorB_Enable, HIGH);

  digitalWrite(MotorA_Input1, HIGH);
  digitalWrite(MotorA_Input2, LOW);
  digitalWrite(MotorB_Input1, HIGH);
  digitalWrite(MotorB_Input2, LOW);
  // Aquí iría el código para mover el carrito hacia adelante
}

void retroceder() {
  digitalWrite(MotorA_Enable, HIGH);
  digitalWrite(MotorB_Enable, HIGH);
  
  digitalWrite(MotorA_Input1, LOW);
  digitalWrite(MotorA_Input2, HIGH);
  digitalWrite(MotorB_Input1, LOW);
  digitalWrite(MotorB_Input2, HIGH);
  // Aquí iría el código para mover el carrito hacia atrás
}

void izquierda() {
  digitalWrite(MotorA_Enable, HIGH);
  digitalWrite(MotorB_Enable, HIGH);
  
  digitalWrite(MotorA_Input1, HIGH);
  digitalWrite(MotorA_Input2, LOW);
  digitalWrite(MotorB_Input1, LOW);
  digitalWrite(MotorB_Input2, HIGH);
  // Aquí iría el código para girar el carrito a la izquierda
}

void derecha() {
  digitalWrite(MotorA_Enable, HIGH);
  digitalWrite(MotorB_Enable, HIGH);
  
  digitalWrite(MotorA_Input1, LOW);
  digitalWrite(MotorA_Input2, HIGH);
  digitalWrite(MotorB_Input1, HIGH);
  digitalWrite(MotorB_Input2, LOW);
  // Aquí iría el código para girar el carrito a la derecha
}

void alto() {
  digitalWrite(MotorA_Enable, LOW);
  digitalWrite(MotorB_Enable, LOW);
  
  digitalWrite(MotorA_Input1, LOW);
  digitalWrite(MotorA_Input2, LOW);
  digitalWrite(MotorB_Input1, LOW);
  digitalWrite(MotorB_Input2, LOW);
  delay(2000);
}


//FUNCIONES PARA DATOS DEL MOTOR

void CalculoPulsoA()
{
  int LstateA = digitalRead(EncoderA_C1);
  if ((EncoderA_C1Last == LOW) && LstateA == HIGH)
  {
    int val = digitalRead(EncoderA_C2);
    if (val == LOW && DireccionA)
    {
      DireccionA = false; // Atrás
    }
    else if (val == HIGH && !DireccionA)
    {
      DireccionA = true;  // Adelante
    }
  }
  EncoderA_C1Last = LstateA;

  if (!DireccionA)  PasoA++;
  else  PasoA--;
}

void CalculoPulsoB()
{
  int LstateB = digitalRead(EncoderB_C1);
  if ((EncoderB_C1Last == LOW) && LstateB == HIGH)
  {
    int val = digitalRead(EncoderB_C2);
    if (val == LOW && DireccionB)
    {
      DireccionB = false; // Atrás
    }
    else if (val == HIGH && !DireccionB)
    {
      DireccionB = true;  // Adelante
    }
  }
  EncoderB_C1Last = LstateB;

  if (!DireccionB)  PasoB++;
  else  PasoB--;
}

void mostrarInformacion() {
  //MOTOR A
  Serial.print("PasoA: ");
  Serial.print(PasoA);
  Serial.print(", ");
  Serial.print("DirecciónA: ");
  Serial.println(DireccionA ? "Adelante" : "Atras");

  //MOTOR B
  Serial.print("PasoB: ");
  Serial.print(PasoB);
  Serial.print(", ");
  Serial.print("DirecciónB: ");
  Serial.println(DireccionB ? "Adelante" : "Atras");
}

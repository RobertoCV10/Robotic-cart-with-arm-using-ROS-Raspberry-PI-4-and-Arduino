/*          Pines arduino

 Puente H : 
           In 1 : 30
           In 2 : 31
           In 3 : 32
           In 4 : 33
           Ena 1 : 44
           Ena 2 : 45

Encoders :

          C1A : 3
          C2B : 2
          C1B : 19
          C2B : 18

Servos :

          Servo 1 : 5
          Servo 2 : 6
          Servo 3 : 7
          
Sensor TUF :

          Sda :20
          Scl :21
          
Sensores IR :

         IR 1 : 22
         IR 2 : 23

Sensor de Ampers :

        PIN : A15


Letras Libres:

q
e
t
u
i
o
p
f
g
h
v


Letras ya asignadas sin uso:


*/


#include <ros.h>
#include <std_msgs/Char.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

ros::NodeHandle nh;

// Nombres de los servos
Servo servo1;
Servo servo2;
Servo servo3;

// Grado inicial del Servo
int anguloServo1 = 50;
int anguloServo2 = 50;
int anguloServo3 = 50;

const int sensorIzquierdaPin =22; // Pin donde está conectado el sensor infrarrojo izquierdo
const int sensorDerechaPin = 23;   // Pin donde está conectado el sensor infrarrojo derecho

// Definición de pines para el Motor A
const int C1 = 3; // Entrada de la señal A del encoder del motor 1.
const int C2 = 2; // Entrada de la señal B del encoder del motor 1.
const byte MotorA_Enable = 44; // Pin 5 digital para habilitar el puente H
const byte MotorA_Input1 = 30; // Pin 6 digital para control de dirección del puente H
const byte MotorA_Input2 = 31; // Pin 7 digital para control de dirección del puente H

// Definición de pines para el Motor B
const int C1B = 19; // Entrada de la señal A del encoder del motor 2.
const int C2B = 18; // Entrada de la señal B del encoder del motor 2.
const byte MotorB_Enable = 45; // Pin 21 digital para habilitar el puente H
const byte MotorB_Input1 = 32; // Pin 20 digital para control de dirección del puente H
const byte MotorB_Input2 = 33; // Pin 19 digital para control de dirección del puente H

//velocidades para los motores
int Pwm_left_forward = 255;
int Pwm_right_forward = 254;

int Pwm_left_flip = 254;
int Pwm_right_flip = 254;

//entradas desde la raps hacia el arduino
const int pinEntrada1 = 24;  // Pin de entrada para el primer LED
const int pinEntrada2 = 25;  // Pin de entrada para el segundo LED

//Variables para los encoders y odometria
volatile int n = 0;
volatile byte ant = 0;
volatile byte act = 0;

volatile int nB = 0;
volatile byte antB = 0;
volatile byte actB = 0;

unsigned long lastTime = 0;  // Tiempo anterior
unsigned long sampleTime = 100;  // Tiempo de muestreo en milisegundos

unsigned long lastTimeB = 0;  // Tiempo anterior
unsigned long sampleTimeB = 100;  // Tiempo de muestreo en milisegundos

const float pulsePerRevolution = 44.0; // Pulsos por revolución (11 pulsos por cuadrante * 4)
const float wheelRadius = 0.04; // Radio de la rueda en metros (ejemplo: 3 cm)

const float pulsePerRevolutionB = 44.0; // Pulsos por revolución (11 pulsos por cuadrante * 4)
const float wheelRadiusB = 0.04; // Radio de la rueda en metros (ejemplo: 3 cm)

const float wheelBase = 0.19; // Distancia entre las ruedas en metros (ejemplo: 15 cm)

float x = 0.0; // Posición X del robot
float y = 0.0; // Posición Y del robot
float theta = 0.0; // Orientación del robot en radianes

//sensor TUF
const int MAX_DISTANCE_CM = 30; // Rango máximo en centímetros
const int NUM_SAMPLES = 10;     // Número de muestras para promediar

int sensorIzquierdaValue = 0; // Variable para almacenar el valor de lectura del sensor izquierdo
int sensorDerechaValue = 0;   // Variable para almacenar el valor de lectura del sensor derecho


//Seccion amperimetro
const float Sensibilidad = 0.185; // Sensibilidad en Voltios/Amperio para sensor de 5A
const unsigned long intervalo = 1000; // Intervalo de 1 segundo (1000 ms)

unsigned long tiempoAnterior = 0; // Almacena el último tiempo de medición

// Declaración de funciones
void commandCallback(const std_msgs::String& msg);
void avanzar();
void retroceder();
void izquierda();
void derecha();
void alto();
void CalculoPulsoA();
void CalculoPulsoB();
void mostrarInformacion();

bool executingCommand = false;  // Variable para controlar si se está ejecutando un comando
// Variable de los modos
bool modoManualActivo = true;
bool modoAutonomoActivo = false;

// Suscriptor para recibir comandos desde ROS
ros::Subscriber<std_msgs::String> sub("arduino_commands", &commandCallback);

// Publicador para enviar retroalimentación a ROS
std_msgs::String feedback_msg;
ros::Publisher feedback_pub("arduino_feedback", &feedback_msg);

void setup() {
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(feedback_pub);
    Serial.begin(57600);

    Wire.begin();        // Inicializar comunicación I2C
    sensor.init();       // Inicializar el sensor VL53L0X
    sensor.setTimeout(500); // Establecer tiempo de espera para lecturas del sensor
    // Iniciar el modo de medición continua
    sensor.startContinuous();
    
    // Definir pines del Servo
    servo1.attach(5);  // Define el pin para el servo 1
    servo2.attach(6);  // Define el pin para el servo 2
    servo3.attach(7);  // Define el pin para el servo 3

    // Inicialmente, los servos se colocan en posición media
    servo1.write(anguloServo1);
    servo2.write(anguloServo2);
    servo3.write(anguloServo3);

    // Inicialización de pines para el Motor A
    pinMode(MotorA_Enable, OUTPUT);
    pinMode(MotorA_Input1, OUTPUT);
    pinMode(MotorA_Input2, OUTPUT);

    // Inicialización de pines para el Motor B
    pinMode(MotorB_Enable, OUTPUT);
    pinMode(MotorB_Input1, OUTPUT);
    pinMode(MotorB_Input2, OUTPUT);

    pinMode(pinEntrada1, INPUT);
    pinMode(pinEntrada2, INPUT);

    pinMode(sensorIzquierdaPin, INPUT); // Configura el pin del sensor izquierdo como entrada
    pinMode(sensorDerechaPin, INPUT);   // Configura el pin del sensor derecho como entrada

    // Inicialmente, deshabilitamos el puente H
    digitalWrite(MotorA_Enable, LOW);
    digitalWrite(MotorB_Enable, LOW);

    // Configuración de pines e interrupciones para los encoders
    pinMode(C1, INPUT);
    pinMode(C2, INPUT);
    
    attachInterrupt(digitalPinToInterrupt(C1), encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(C2), encoder, CHANGE);
    
    pinMode(C1B, INPUT);
    pinMode(C2B, INPUT);
    
    attachInterrupt(digitalPinToInterrupt(C1B), encoderB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(C2B), encoderB, CHANGE);
}

void loop() {
    nh.spinOnce();
}



void commandCallback(const std_msgs::String& msg) {
    String command = msg.data;

    int estadoEntrada1 = digitalRead(pinEntrada1);
    int estadoEntrada2 = digitalRead(pinEntrada2);

    if (command == "z") {
        modoManualActivo = true;
        modoAutonomoActivo = false;
        Modo_Manual();  // Publicar retroalimentación hacia ROS
    } else if (command == "c") {
        modoAutonomoActivo = true;
        modoManualActivo = false;
        Modo_Autonomo();  // Publicar retroalimentación hacia ROS
    } else if (modoManualActivo) {
        if (command == "w") {
            avanzar();
        } else if (command == "s") {
            retroceder();
        } else if (command == "d") {
            izquierda();
        } else if (command == "a") {
            derecha();
        } else if (command == "p") {
            alto();
        } else if (command == "m") {
            incrementarAnguloServo1();
        } else if (command == "n") {
            decrementarAnguloServo1();
        } else if (command == "j") {
            incrementarAnguloServo2();
        } else if (command == "k") {
            decrementarAnguloServo2();
        } else if (command == "r") {
            incrementarAnguloServo3();
        } else if (command == "l") {
            decrementarAnguloServo3();
        } else if (command == "y") {
            measureDistance();
        } else if (command == "x") {
            medirCorriente();
        } else if (command == "b") {
            verificarSensores();
        }
    } else if (modoAutonomoActivo) {
        if (estadoEntrada1 == HIGH) {
            izquierda();
        } else if (estadoEntrada1 == LOW && estadoEntrada2 == LOW) {
            alto();
        } else if (estadoEntrada2 == HIGH) {
            derecha();
        }
    }
}



void avanzar() {
    analogWrite(MotorA_Enable, Pwm_left_forward);
    analogWrite(MotorB_Enable, Pwm_right_forward);

    digitalWrite(MotorA_Input1, HIGH);
    digitalWrite(MotorA_Input2, LOW);
    digitalWrite(MotorB_Input1, HIGH);
    digitalWrite(MotorB_Input2, LOW);

    // Publicar retroalimentación hacia ROS
    feedback_msg.data = "Estado Avanzando";
    feedback_pub.publish(&feedback_msg);

    delay(25);
    alto();
}

void retroceder() {
    analogWrite(MotorA_Enable, Pwm_left_forward);
    analogWrite(MotorB_Enable, Pwm_right_forward);

    digitalWrite(MotorA_Input1, LOW);
    digitalWrite(MotorA_Input2, HIGH);
    digitalWrite(MotorB_Input1, LOW);
    digitalWrite(MotorB_Input2, HIGH);

    // Publicar retroalimentación hacia ROS
    feedback_msg.data = "Estado Retrocediendo";
    feedback_pub.publish(&feedback_msg);
    
    delay(25);
    alto();
}

void izquierda() {
    analogWrite(MotorA_Enable, Pwm_left_flip);
    analogWrite(MotorB_Enable, Pwm_right_flip);

    digitalWrite(MotorA_Input1, HIGH);
    digitalWrite(MotorA_Input2, LOW);
    digitalWrite(MotorB_Input1, LOW);
    digitalWrite(MotorB_Input2, HIGH);

    // Publicar retroalimentación hacia ROS
    feedback_msg.data = "Estado izquierda";
    feedback_pub.publish(&feedback_msg);

    delay(25);
    alto();
}

void derecha() {
    analogWrite(MotorA_Enable, Pwm_left_flip);
    analogWrite(MotorB_Enable, Pwm_right_flip);

    digitalWrite(MotorA_Input1, LOW);
    digitalWrite(MotorA_Input2, HIGH);
    digitalWrite(MotorB_Input1, HIGH);
    digitalWrite(MotorB_Input2, LOW);

    // Publicar retroalimentación hacia ROS
    feedback_msg.data = "Estado derecha";
    feedback_pub.publish(&feedback_msg);

    delay(25);
    alto();
}

void alto() {
    digitalWrite(MotorA_Enable, LOW);
    digitalWrite(MotorB_Enable, LOW);

    digitalWrite(MotorA_Input1, LOW);
    digitalWrite(MotorA_Input2, LOW);
    digitalWrite(MotorB_Input1, LOW);
    digitalWrite(MotorB_Input2, LOW);

    // Publicar retroalimentación hacia ROS
    //feedback_msg.data = "Estado detenido";
    //feedback_pub.publish(&feedback_msg);
}

// Incrementar y decrementar ángulos de los servos
void incrementarAnguloServo1() {
    if (anguloServo1 < 180) {
        anguloServo1++;
    } else {
        anguloServo1 = 180; // Establecer el ángulo máximo permitido
    }
    servo1.write(anguloServo1);

    String mensaje = "Servo 1 - angulo " + String(anguloServo1);
    feedback_msg.data = mensaje.c_str();
    feedback_pub.publish(&feedback_msg);
}

void decrementarAnguloServo1() {
    if (anguloServo1 > 0) {
        anguloServo1--;
    } else {
        anguloServo1 = 0; // Establecer el ángulo máximo permitido
    }
    servo1.write(anguloServo1);

    String mensaje = "Servo 1 - angulo " + String(anguloServo1);
    feedback_msg.data = mensaje.c_str();
    feedback_pub.publish(&feedback_msg);
}

void incrementarAnguloServo2() {
    if (anguloServo2 < 180) {
        anguloServo2++;
    } else {
        anguloServo2 = 180; // Establecer el ángulo máximo permitido
    }
    servo2.write(anguloServo2);

    String mensaje = "Servo 2 - angulo " + String(anguloServo2);
    feedback_msg.data = mensaje.c_str();
    feedback_pub.publish(&feedback_msg);
}

void decrementarAnguloServo2() {
    if (anguloServo2 > 0) {
        anguloServo2--;
    } else {
        anguloServo2 = 0; // Establecer el ángulo máximo permitido
    }
    servo2.write(anguloServo2);

    String mensaje = "Servo 2 - angulo " + String(anguloServo2);
    feedback_msg.data = mensaje.c_str();
    feedback_pub.publish(&feedback_msg);
}

void incrementarAnguloServo3() {
    if (anguloServo3 < 180) {
        anguloServo3++;
    } else {
        anguloServo3 = 180; // Establecer el ángulo máximo permitido
    }
    servo3.write(anguloServo3);

    String mensaje = "Servo 3 - angulo " + String(anguloServo3);
    feedback_msg.data = mensaje.c_str();
    feedback_pub.publish(&feedback_msg);
}

void decrementarAnguloServo3() {
    if (anguloServo3 > 0) {
        anguloServo3--;
    } else {
        anguloServo3 = 0; // Establecer el ángulo máximo permitido
    }
    servo3.write(anguloServo3);

    String mensaje = "Servo 3 - angulo " + String(anguloServo3);
    feedback_msg.data = mensaje.c_str();
    feedback_pub.publish(&feedback_msg);
}

void Modo_Manual() {
    // Publicar retroalimentación hacia ROS
    feedback_msg.data = "Modo Manual";
    feedback_pub.publish(&feedback_msg);
}

void Modo_Autonomo() {
    // Publicar retroalimentación hacia ROS
    feedback_msg.data = "Modo Autonomo";
    feedback_pub.publish(&feedback_msg);
}

// Función para medir la distancia y promediar varias lecturas
void measureDistance() {
  long totalDistance = 0; // Variable para acumular la distancia total
  int validSamples = 0;   // Contador de muestras válidas
  int distance;
  
  // Tomar múltiples muestras y acumular las distancias
  for (int i = 0; i < NUM_SAMPLES; i++) {
    distance = sensor.readRangeContinuousMillimeters(); // Leer distancia en milímetros
    
    if (sensor.timeoutOccurred()) {
      String mensaje = "Sensor - TIMEOUT";
      feedback_msg.data = mensaje.c_str();
      feedback_pub.publish(&feedback_msg);
      continue; // Saltar esta iteración si ocurre un timeout
    }
    
    if (distance >= 0 && distance <= MAX_DISTANCE_CM * 10) { // Convertir cm a mm
      totalDistance += distance;
      validSamples++;
    } else {
    String mensaje = "Sensor - Lecturas invalidas";
    feedback_msg.data = mensaje.c_str();
    feedback_pub.publish(&feedback_msg);
    }
    
    delay(50); // Esperar 50 ms entre lecturas
  }

  // Calcular el promedio de las distancias si hay muestras válidas
  if (validSamples > 0) {
    float averageDistance = (totalDistance / validSamples) / 10.0; // Convertir mm a cm
    Serial.println(String("Sensor - Distancia Promedio: ") + String(averageDistance) + String(" cm"));

    String mensaje = "Sensor - Distancia Promedio: " + String(averageDistance) + " cm";
    feedback_msg.data = mensaje.c_str();
    feedback_pub.publish(&feedback_msg);
    
  } else {
    String mensaje = "Sensor - No se obtuvieron muestras validas";
    feedback_msg.data = mensaje.c_str();
    feedback_pub.publish(&feedback_msg);  }
}

void verificarSensores() {
  sensorIzquierdaValue = digitalRead(sensorIzquierdaPin); // Lee el valor del sensor izquierdo
  sensorDerechaValue = digitalRead(sensorDerechaPin);     // Lee el valor del sensor derecho

  if (sensorIzquierdaValue == 0 && sensorDerechaValue == 0) {
    String mensaje = "IR - Obstacles in both sides";
    feedback_msg.data = mensaje.c_str();
    feedback_pub.publish(&feedback_msg);
  } 
  else if (sensorIzquierdaValue == 0 && sensorDerechaValue == 1) {
    String mensaje = "IR - Left obstacle";
    feedback_msg.data = mensaje.c_str();
    feedback_pub.publish(&feedback_msg);
  } 
  else if (sensorIzquierdaValue == 1 && sensorDerechaValue == 0) {
    String mensaje = "IR - Right obstacle";
    feedback_msg.data = mensaje.c_str();
    feedback_pub.publish(&feedback_msg);
  } 
  else if (sensorIzquierdaValue == 1 && sensorDerechaValue == 1) {
    String mensaje = "IR - No obstacle";
    feedback_msg.data = mensaje.c_str();
    feedback_pub.publish(&feedback_msg);
  }
}

//Seccion Amperimetro

void medirCorriente() {
  unsigned long tiempoActual = millis(); // Obtiene el tiempo actual

  // Verifica si ha pasado el intervalo de tiempo deseado
  if (tiempoActual - tiempoAnterior >= intervalo) {
    tiempoAnterior = tiempoActual; // Actualiza el tiempoAnterior

    float corrientePromedio = getCorrientePromedio(200); // Obtiene la corriente promedio de 200 muestras
    String corrienteString = String(corrientePromedio, 3); // Convierte el valor de corriente a string con 3 decimales
    /*Serial.print("Corriente: ");
    Serial.println(corrienteString); */

    String mensaje = "Ampers consumed by base: " + corrienteString;
    feedback_msg.data = mensaje.c_str();
    feedback_pub.publish(&feedback_msg);
    
  }

  // Otras tareas que no bloquean el programa pueden ir aquí
}

float getCorrientePromedio(int nMuestras) {
  float voltajeSensor;
  float corrienteTotal = 0;

  for (int i = 0; i < nMuestras; i++) {
    voltajeSensor = analogRead(A15) * (5.0 / 1023.0); // Lectura del sensor
    corrienteTotal += (voltajeSensor - 2.5) / Sensibilidad; // Ecuación para obtener la corriente
  }

  float corrientePromedio = corrienteTotal / nMuestras; // Calcula el promedio
  return corrientePromedio;
}

//Seccion y odometria

void encoder(void) {
  ant = act;
  
  if (digitalRead(C1)) bitSet(act, 1); else bitClear(act, 1);            
  if (digitalRead(C2)) bitSet(act, 0); else bitClear(act, 0);
  
  if (ant == 2 && act == 0) n++;
  if (ant == 0 && act == 1) n++;
  if (ant == 3 && act == 2) n++;
  if (ant == 1 && act == 3) n++;
  
  if (ant == 1 && act == 0) n--;
  if (ant == 3 && act == 1) n--;
  if (ant == 0 && act == 2) n--;
  if (ant == 2 && act == 3) n--;
}

void encoderB(void) {
  antB = actB;
  
  if (digitalRead(C1B)) bitSet(actB, 1); else bitClear(actB, 1);            
  if (digitalRead(C2B)) bitSet(actB, 0); else bitClear(actB, 0);
  
  if (antB == 2 && actB == 0) nB++;
  if (antB == 0 && actB == 1) nB++;
  if (antB == 3 && actB == 2) nB++;
  if (antB == 1 && actB == 3) nB++;
  
  if (antB == 1 && actB == 0) nB--;
  if (antB == 3 && actB == 1) nB--;
  if (antB == 0 && actB == 2) nB--;
  if (antB == 2 && actB == 3) nB--;
}

void updateOdometry() {
  static unsigned long lastUpdateTime = 0;
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastUpdateTime) / 1000.0; // Convertir a segundos
  
  if (deltaTime > 0) {
    // Calcular velocidades angulares y lineales
    float angularVelocity = calculateAngularVelocity();
    float linearVelocity = calculateLinearVelocity(angularVelocity);

    float angularVelocityB = calculateAngularVelocityB();
    float linearVelocityB = calculateLinearVelocityB(angularVelocityB);

    // Calcular desplazamiento lineal promedio
    float deltaS = (linearVelocity + linearVelocityB) / 2.0 * deltaTime;
    
    // Calcular cambio en la orientación
    float deltaTheta = (linearVelocityB - linearVelocity) / wheelBase * deltaTime;

    // Actualizar posición y orientación
    x += deltaS * cos(theta);
    y += deltaS * sin(theta);
    theta += deltaTheta;

    /*Serial.print("Numero de cuentas MOTOR 1: "); Serial.print(n);
    Serial.print(" | Velocidad Angular (rad/s): "); Serial.print(angularVelocity);
    Serial.print(" | Velocidad Lineal (m/s): "); Serial.print(linearVelocity);
    Serial.print(" | Numero de cuentas MOTOR 2: "); Serial.print(nB);
    Serial.print(" | Velocidad Angular2 (rad/s): "); Serial.print(angularVelocityB);
    Serial.print(" | Velocidad Lineal2 (m/s): "); Serial.print(linearVelocityB);
    Serial.print(" | X: "); Serial.print(x);
    Serial.print(" | Y: "); Serial.print(y);
    Serial.print(" | Theta: "); Serial.println(theta);*/

    // Imprimir velocidades y odometría
    String mensaje = "Motor 1 - Lineal speed:" + String(linearVelocity);
    String mensaje2 = "Motor 2 - Lineal speed:" + String(linearVelocityB);
    String mensaje3 = " " + String((linearVelocity+linearVelocityB)/2);
    String mensaje4 = "Motors - X:" + String(x);
    String mensaje5 = "Motors - Y:" + String(y);
    String mensaje6 = "Motors - Theta:" + String(theta);
    feedback_msg.data = mensaje.c_str();
    feedback_pub.publish(&feedback_msg);
    feedback_msg.data = mensaje2.c_str();
    feedback_pub.publish(&feedback_msg);
    feedback_msg.data = mensaje3.c_str();
    feedback_pub.publish(&feedback_msg);
    feedback_msg.data = mensaje4.c_str();
    feedback_pub.publish(&feedback_msg);
    feedback_msg.data = mensaje5.c_str();
    feedback_pub.publish(&feedback_msg);
    feedback_msg.data = mensaje6.c_str();
    feedback_pub.publish(&feedback_msg);


    // Actualizar el tiempo de la última actualización
    lastUpdateTime = currentTime;
  }
}

float calculateAngularVelocity() {
  static unsigned long lastCalculationTime = 0;
  static int lastCount = 0;
  
  unsigned long currentTime = millis();
  int currentCount = n;
  
  float deltaTime = (currentTime - lastCalculationTime) / 1000.0; // Convertir a segundos
  int deltaCount = currentCount - lastCount;
  
  if (deltaTime == 0) return 0; // Evitar división por cero
  
  float angularVelocity = (deltaCount * 2 * PI) / (pulsePerRevolution * deltaTime); // Velocidad angular en rad/s
  
  lastCalculationTime = currentTime;
  lastCount = currentCount;
  
  return angularVelocity;
}

float calculateAngularVelocityB() {
  static unsigned long lastCalculationTimeB = 0;
  static int lastCountB = 0;
  
  unsigned long currentTimeB = millis();
  int currentCountB = nB;
  
  float deltaTimeB = (currentTimeB - lastCalculationTimeB) / 1000.0; // Convertir a segundos
  int deltaCountB = currentCountB - lastCountB;
  
  if (deltaTimeB == 0) return 0; // Evitar división por cero
  
  float angularVelocityB = (deltaCountB * 2 * PI) / (pulsePerRevolutionB * deltaTimeB); // Velocidad angular en rad/s
  
  lastCalculationTimeB = currentTimeB;
  lastCountB = currentCountB;
  
  return angularVelocityB;
}

float calculateLinearVelocity(float angularVelocity) {
  return wheelRadius * angularVelocity; // Velocidad lineal en m/s
}

float calculateLinearVelocityB(float angularVelocityB) {
  return wheelRadiusB * angularVelocityB; // Velocidad lineal en m/s
}

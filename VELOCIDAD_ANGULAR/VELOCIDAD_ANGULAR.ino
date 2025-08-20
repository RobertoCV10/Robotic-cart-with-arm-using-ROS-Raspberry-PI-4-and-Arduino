// Motor de CD con motorreductor, caja de engranes metálicos y encoder cuadratura. Torque de 5.2 kg.cm a 110 rpm.
// Autor: Ing. Josep Ramon Vidal. Joober Tecnologies. https://joober.page.link/joober

const byte EncoderA_C1 = 3; // Cable amarillo pin 3 digital
const byte EncoderA_C2 = 4; // Cable verde al pin 4 digital

const byte EncoderB_C1 = 18; // Cable amarillo pin 18 digital
const byte EncoderB_C2 = 17; // Cable verde al pin 17 digital

byte EncoderA_C1Last;
int pasoA;
boolean direccionA;
int pasoAnteriorA;

byte EncoderB_C1Last;
int pasoB;
boolean direccionB;
int pasoAnteriorB;

float diametro = 5.0;  // Supongamos un diámetro de 5 cm (ajustar según tus especificaciones)
float anchoEje = 10.0;  // Ancho del eje del robot en cm
float distanciaCentro = 10.0;  // Distancia entre el eje de las ruedas y el centro del robot en cm
float velocidadLinealA = 0.0;   // Velocidad lineal de la rueda A en cm/s
float velocidadLinealB = 0.0;   // Velocidad lineal de la rueda B en cm/s
float velocidadLinealRobot = 0.0;  // Velocidad lineal del robot en cm/s
float velocidadAngularRobot = 0.0;  // Velocidad angular del robot en rad/s

unsigned long tiempoAnteriorA = 0;  // Tiempo anterior de la lectura del encoder A
unsigned long tiempoAnteriorB = 0;  // Tiempo anterior de la lectura del encoder B

void setup() {
  Serial.begin(57600);
  
  pinMode(EncoderA_C2, INPUT);
  pinMode(EncoderB_C2, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(EncoderA_C1), calculapulsoA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncoderB_C1), calculapulsoB, CHANGE);
}

void loop() {
  Serial.print("PasoA: ");
  Serial.print(pasoA);
  Serial.print(", ");
  Serial.print("ÁnguloA: ");
  Serial.println(pasoA * 0.481283422459893);  // Convertir pasos a grados

  Serial.print("PasoB: ");
  Serial.print(pasoB);
  Serial.print(", ");
  Serial.print("ÁnguloB: ");
  Serial.println(pasoB * 0.481283422459893);  // Convertir pasos a grados

  // Imprimir velocidad lineal y angular del robot diferencial
  Serial.print("Velocidad lineal del robot: ");
  Serial.print(velocidadLinealRobot);
  Serial.println(" cm/s");

  Serial.print("Velocidad angular del robot: ");
  Serial.print(velocidadAngularRobot);
  Serial.println(" rad/s");

  delay(100);  // Retardo para reducir la frecuencia de salida por serial
}

void calculapulsoA() {
  int LstateA = digitalRead(EncoderA_C1);
  if ((EncoderA_C1Last == LOW) && LstateA == HIGH) {
    int valA = digitalRead(EncoderA_C2);
    if (valA == LOW && direccionA) {
      direccionA = false; // Atrás
    } else if (valA == HIGH && !direccionA) {
      direccionA = true;  // Adelante
    }
  }
  EncoderA_C1Last = LstateA;

  if (!direccionA) {
    pasoA++;
  } else {
    pasoA--;
  }

  unsigned long tiempoActual = millis();
  unsigned long intervaloA = tiempoActual - tiempoAnteriorA;
  tiempoAnteriorA = tiempoActual;

  velocidadLinealA = (float)(pasoA - pasoAnteriorA) / intervaloA * 1000.0 / 0.481283422459893;
  pasoAnteriorA = pasoA;

  // Calcular velocidad lineal del robot (rueda A)
  velocidadLinealRobot = (velocidadLinealA + velocidadLinealB) / 2.0;  // Promedio de las velocidades lineales
  velocidadAngularRobot = (velocidadLinealB - velocidadLinealA) / distanciaCentro;  // Calcular velocidad angular del robot

  String VelocidadLinealDato = String(velocidadLinealRobot);
  String VelocidadAngularDato = String(velocidadAngularRobot);
}

void calculapulsoB() {
  int LstateB = digitalRead(EncoderB_C1);
  if ((EncoderB_C1Last == LOW) && LstateB == HIGH) {
    int valB = digitalRead(EncoderB_C2);
    if (valB == LOW && direccionB) {
      direccionB = false; // Atrás
    } else if (valB == HIGH && !direccionB) {
      direccionB = true;  // Adelante
    }
  }
  EncoderB_C1Last = LstateB;

  if (!direccionB) {
    pasoB++;
  } else {
    pasoB--;
  }

  unsigned long tiempoActual = millis();
  unsigned long intervaloB = tiempoActual - tiempoAnteriorB;
  tiempoAnteriorB = tiempoActual;

  velocidadLinealB = (float)(pasoB - pasoAnteriorB) / intervaloB * 1000.0 / 0.481283422459893;
  pasoAnteriorB = pasoB;

  // Calcular velocidad lineal del robot (rueda B)
  velocidadLinealRobot = (velocidadLinealA + velocidadLinealB) / 2.0;  // Promedio de las velocidades lineales
  velocidadAngularRobot = (velocidadLinealB - velocidadLinealA) / distanciaCentro;  // Calcular velocidad angular del robot
  String VelocidadLinealDato = String(velocidadLinealRobot);
  String VelocidadAngularDato = String(velocidadAngularRobot);
  
}

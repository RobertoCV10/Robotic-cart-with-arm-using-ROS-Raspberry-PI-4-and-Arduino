// Motor de CD con motorreductor, caja de engranes metálicos y encoder cuadratura. Torque de 5.2 kg.cm a 110 rpm.
// Autor: Ing. Josep Ramon Vidal. Joober Tecnologies. https://joober.page.link/joober

const byte Encoder_C1 = 3; // Cable amarillo pin 3 digital
const byte Encoder_C2 = 4; // Cable verde al pin 4 digital
const byte Motor_Enable = 5; // Pin 5 digital para habilitar el puente H
const byte Motor_Input1 = 6; // Pin 6 digital para control de dirección del puente H
const byte Motor_Input2 = 7; // Pin 7 digital para control de dirección del puente H

byte Encoder_C1Last;
int paso;
boolean direccion;

void setup()
{
  Serial.begin(57600);
  pinMode(Motor_Enable, OUTPUT);
  pinMode(Motor_Input1, OUTPUT);
  pinMode(Motor_Input2, OUTPUT);

  // Inicialmente, deshabilitamos el puente H
  digitalWrite(Motor_Enable, LOW);
}

void loop()
{
  pinMode(Encoder_C2, INPUT);
  attachInterrupt(digitalPinToInterrupt(3), calculapulso, CHANGE);

  // Ejemplo de movimiento: Girar el motor hacia adelante durante 2 segundos
  moverMotor(1); // 1 para adelante, -1 para atrás
  delay(2000);   // Tiempo de movimiento

  mostrarInformacion(); // Mostrar información actual del motor
}

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

void moverMotor(int direccion)
{
  // Habilitar el puente H
  digitalWrite(Motor_Enable, HIGH);

  // Determinar dirección del motor
  if (direccion == 1)
  {
    digitalWrite(Motor_Input1, HIGH);
    digitalWrite(Motor_Input2, LOW);
  }
  else if (direccion == -1)
  {
    digitalWrite(Motor_Input1, LOW);
    digitalWrite(Motor_Input2, HIGH);
  }

  // Aquí puedes ajustar el tiempo de activación del motor según tu necesidad
  // Por ejemplo, podrías usar un temporizador o simplemente un delay para demostración.
  delay(1000); // Tiempo de activación del motor

  // Deshabilitar el puente H
  digitalWrite(Motor_Enable, LOW);
}

void mostrarInformacion() {
  Serial.print("Paso: ");
  Serial.print(paso);
  Serial.print(", ");
  Serial.print("Dirección: ");
  Serial.println(direccion ? "Adelante" : "Atras");
}

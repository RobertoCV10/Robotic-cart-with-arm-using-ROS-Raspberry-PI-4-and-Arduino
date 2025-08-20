//Motor de CD con motorreductor, caja de engranes metálicos y encoder cuadratura. Torque de 5.2 kg.cm a 110 rpm.
//Autor: Ing. Josep Ramon Vidal. Joober Tecnologies. https://joober.page.link/joober

const byte EncoderA_C1 = 3; // Cable amarillo pin 3 digital
const byte EncoderA_C2 = 4; // Cable verde al pin 4 digital

const byte EncoderB_C1 = 18; // Cable amarillo pin 18 digital
const byte EncoderB_C2 = 17; // Cable verde al pin 17 digital

byte EncoderA_C1Last;
int pasoA;
boolean direccionA;

byte EncoderB_C1Last;
int pasoB;
boolean direccionB;

void setup()
{
  Serial.begin(57600);
}

void loop()
{
  pinMode(EncoderA_C2, INPUT);
  pinMode(EncoderB_C2, INPUT);
  attachInterrupt(digitalPinToInterrupt(3), calculapulsoA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), calculapulsoA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(18), calculapulsoB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(18), calculapulsoB, CHANGE);
  

  Serial.print("PasoA: ");
  Serial.print(pasoA);
  Serial.print(", ");
  Serial.print("ÁnguloA: ");
  Serial.println(pasoA * 0.481283422459893); 


  Serial.print("PasoB: ");
  Serial.print(pasoB);
  Serial.print(", ");
  Serial.print("ÁnguloB: ");
  Serial.println(pasoB * 0.481283422459893); 
  
  /*Para por cada vuelta del rotor antes de la caja de engranes, se producen 22 pulsos (o pasos) del encoder. 
  El motor tiene una relación de reducción en su caja de engranes de 1:34. Por tanto, se tienen 748 ticks o pulsos 
  del encoder por cada revolución del rotor después de la caja de engranes. Por lo que 748/360 = 0.48128...*/

}

void calculapulsoA()
{
  int LstateA = digitalRead(EncoderA_C1);
  if ((EncoderA_C1Last == LOW) && LstateA == HIGH)
  {
    int valA = digitalRead(EncoderA_C2);
    if (valA == LOW && direccionA)
    {
      direccionA = false; //Atrás
    }
    else if (valA == HIGH && !direccionA)
    {
      direccionA = true;  //Adelante
    }
  }
  EncoderA_C1Last = LstateA;

  if (!direccionA)  pasoA++;
  else  pasoA--;
}


void calculapulsoB()
{
  int LstateB = digitalRead(EncoderB_C1);
  if ((EncoderB_C1Last == LOW) && LstateB == HIGH)
  {
    int valB = digitalRead(EncoderB_C2);
    if (valB == LOW && direccionB)
    {
      direccionB = false; //Atrás
    }
    else if (valB == HIGH && !direccionB)
    {
      direccionB = true;  //Adelante
    }
  }
  EncoderB_C1Last = LstateB;

  if (!direccionB)  pasoB++;
  else  pasoB--;
}

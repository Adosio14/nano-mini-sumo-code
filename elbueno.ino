/*Valores modificables para el Robot*/
int VelocidadBusqueda = 220; /*-----> Valor entre 70 y 255<----*/
int VelocidadAtaque = 255;   /*-----> Valor entre 70 y 255<----*/
int VelocidadGiro = 255;     
int VelocidadGiroBusqueda = VelocidadGiro / 2; /*-----> Valor entre 70 y 255<----*/
int Distanciaoponente = 20;   /*-----> Valor entre 10 y 150<----*/
/*----------------------------------*/

/*No modificar el siguiete programa*/
#include <MotorDriver.h>

MotorDriver mD;
MotorDriver mI;

#define PinLienaI A0
#define PinLienaD A1

// REINTEGRADO: Pines del sensor frontal
#define Echo 10      //Pin servo de arriba (Servo_1)
#define Trigger 9    //Pin servo de Abajo  (Servo_2)
/*-------------------------------------*/

#define MotorDerecho 3
#define MotorIzquierdo 4

// Pines de los sensores laterales
#define RightEcho A2
#define RightTrigger A3
#define LeftTrigger A4
#define LeftEcho A5

int tiempoFrenado = 550;
int tiempoLimite = 400;
int millisViejo = 0;
int Vel = 80;  //130

bool startSequenceComplete = false;

// Variables para scan no-bloqueante
enum ScanState { GIRANDO, AVANZANDO };
ScanState scanState = GIRANDO;
unsigned long scanTimer = 0;
const unsigned long GIRO_TIME = 450;
const unsigned long AVANCE_TIME = 450;

void setup() {
  Serial.begin(9600);        //iniciar puerto serie
  pinMode(PinLienaI, INPUT);   //definir pin como entrada
  pinMode(PinLienaD, INPUT);   //definir pin como entrada

  // REINTEGRADO: Configuración del sensor frontal
  pinMode(Trigger, OUTPUT);
  pinMode(Echo, INPUT);
  digitalWrite(Trigger, LOW);
  /*---------------------------------------------*/

  // Configuración de los sensores laterales
  pinMode(RightTrigger, OUTPUT);
  pinMode(RightEcho, INPUT);
  digitalWrite(RightTrigger, LOW);

  pinMode(LeftTrigger, OUTPUT);
  pinMode(LeftEcho, INPUT);
  digitalWrite(LeftTrigger, LOW);
}

// El loop() no cambia, ya que la lógica está en checkOpponentAndAct()
void loop() {
  if (millis() >= 3750) {
    if (!startSequenceComplete) {
      scan(); 
      if (SensorLineaD() == 1 && SensorLineaI() == 1) {
        handleLineDetection(true, true);
        return; 
      } else if (SensorLineaD() == 1) {
        handleLineDetection(true, false);
        return; 
      } else if (SensorLineaI() == 1) {
        handleLineDetection(false, true);
        return; 
      }
    } else {
      // Comportamiento normal
      if (SensorLineaD() == 1 && SensorLineaI() == 1) {
        handleLineDetection(true, true);
      } else if (SensorLineaD() == 1) {
        handleLineDetection(true, false);
      } else if (SensorLineaI() == 1) {
        handleLineDetection(false, true);
      } 
      else if (!checkOpponentAndAct()) {
        Busqueda();
      }
    }
  }
}

// handleLineDetection() no cambia, ya que usa checkOpponentAndAct()
void handleLineDetection(bool derecho, bool izquierdo) {
  if (derecho && izquierdo) {
    millisViejo = millis();
    while ((millis() - millisViejo) < tiempoFrenado) { 
      Atras(); 
      if (checkOpponentAndAct()) { return; }
    }
    millisViejo = millis();
    while ((millis() - millisViejo) < tiempoLimite / 2) {
      GiroDer();
      if (checkOpponentAndAct()) { return; }
    }
  } else if (derecho) {
    millisViejo = millis();
    while ((millis() - millisViejo) < tiempoFrenado) {
      Atras();
      if (checkOpponentAndAct()) { return; }
    }
    millisViejo = millis();
    while ((millis() - millisViejo) < tiempoLimite) {
      GiroIzq(); 
      if (checkOpponentAndAct()) { return; }
    }
  } else if (izquierdo) {
    millisViejo = millis();
    while ((millis() - millisViejo) < tiempoFrenado) { 
      Atras(); 
      if (checkOpponentAndAct()) { return; }
    }
    millisViejo = millis();
    while ((millis() - millisViejo) < tiempoLimite) {
      GiroDer(); 
      if (checkOpponentAndAct()) { return; }
    }
  }
}

// Las funciones de movimiento no cambian
void Busqueda() {
  mD.motor(MotorDerecho, FORWARD, VelocidadBusqueda);
  mI.motor(MotorIzquierdo, FORWARD, VelocidadBusqueda);
}
void Ataque() {
  mD.motor(MotorDerecho, FORWARD, VelocidadAtaque);
  mI.motor(MotorIzquierdo, FORWARD, VelocidadAtaque);
}
void GiroIzq() {
  mD.motor(MotorDerecho, FORWARD, VelocidadGiro);
  mI.motor(MotorIzquierdo, BACKWARD, VelocidadGiro);
}
void GiroDer() {
  mD.motor(MotorDerecho, BACKWARD, VelocidadGiro);
  mI.motor(MotorIzquierdo, FORWARD, VelocidadGiro);
}
void Atras() {
  mD.motor(MotorDerecho, BACKWARD, 125);
  mI.motor(MotorIzquierdo, BACKWARD, 125);
}

// scan() no cambia, ya que usa checkOpponentAndAct()
void scan() {
  unsigned long currentTime = millis();
  
  if (scanTimer == 0) {
    scanTimer = currentTime;
    scanState = AVANZANDO;
  }
  
  switch (scanState) {
    case GIRANDO:
      mD.motor(MotorDerecho, FORWARD, VelocidadGiroBusqueda);
      mI.motor(MotorIzquierdo, BACKWARD, VelocidadGiroBusqueda);
      if (currentTime - scanTimer >= GIRO_TIME) {
        scanState = AVANZANDO;
        scanTimer = currentTime;
      }
      break;
    case AVANZANDO:
      mD.motor(MotorDerecho, FORWARD, VelocidadBusqueda);
      mI.motor(MotorIzquierdo, FORWARD, VelocidadBusqueda);
      if (currentTime - scanTimer >= AVANCE_TIME) {
        scanState = GIRANDO;
        scanTimer = currentTime;
      }
      break;
  }
  
  if (checkOpponentAndAct()) { 
    startSequenceComplete = true;
    Serial.println("Oponente encontrado");
  }
}

// Las funciones de línea no cambian
int SensorLineaI() {
  if (digitalRead(PinLienaI) == LOW) {
    delay(1);
    if (digitalRead(PinLienaI) == LOW) { return 1; }
  }
  return 0;
}
int SensorLineaD() {
  if (digitalRead(PinLienaD) == LOW) {
    delay(1);
    if (digitalRead(PinLienaD) == LOW) { return 1; }
  }
  return 0;
}


// La función getDistance() no cambia
float getDistance(int triggerPin, int echoPin) {
  unsigned long t;
  float d;

  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  t = pulseIn(echoPin, HIGH, 30000);
  d = t * 0.017165; // Cálculo optimizado

  if (d <= 0) { 
    return 999.0; 
  } else {
    return d;
  }
}

// NUEVA: Función para chequear el sensor frontal
int isOponenteFrontal() {
  float d = getDistance(Trigger, Echo); // Usa los pines 9 y 10
  if (d <= Distanciaoponente) {
    return 1;
  } else {
    return 0;
  }
}

// Las funciones de sensor lateral no cambian
int isOponenteLeft() {
  float d = getDistance(LeftTrigger, LeftEcho);
  if (d <= Distanciaoponente) {
    return 1;
  } else {
    return 0;
  }
}
int isOponenteRight() {
  float d = getDistance(RightTrigger, RightEcho);
  if (d <= Distanciaoponente) {
    return 1;
  } else {
    return 0;
  }
}

// MODIFICADO: Función principal de detección (Ahora con 3 sensores)
// Devuelve 'true' si encontró y actuó
// Devuelve 'false' si no encontró oponente
bool checkOpponentAndAct() {
  // Leemos el sensor frontal primero
  bool oponenteFrontal = (isOponenteFrontal() == 1);
  
  // PRIORIDAD 1: Si el sensor frontal lo ve, ataca.
  if (oponenteFrontal) {
    Ataque();
    return true;
  }

  // Si el frontal está libre, leemos los sensores laterales
  bool oponenteIzquierdo = (isOponenteLeft() == 1);
  bool oponenteDerecho = (isOponenteRight() == 1);

  // PRIORIDAD 2: Si ambos lados lo ven, es un objetivo "ancho". Ataca.
  if (oponenteIzquierdo && oponenteDerecho) {
    Ataque();
    return true;
  } 
  
  // PRIORIDAD 3: Solo izquierda
  else if (oponenteIzquierdo) {
    GiroIzq(); // Girar hacia él
    return true;
  } 
  
  // PRIORIDAD 4: Solo derecha
  else if (oponenteDerecho) {
    GiroDer(); // Girar hacia él
    return true;
  }
  
  // No se encontró oponente
  return false;
}
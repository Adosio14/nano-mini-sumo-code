/*Valores modificables para el Robot*/
int VelocidadBusqueda = 220; /*-----> Valor entre 70 y 255<----*/
int VelocidadAtaque = 255;   /*-----> Valor entre 70 y 255<----*/
int VelocidadGiro = 255;    
int VelocidadGiroBusqueda = VelocidadGiro / 2; /*-----> Valor entre 70 y 255<----*/
int Distanciaoponente = 20;  /*-----> Valor entre 10 y 150<----*/
/*----------------------------------*/

/*No modificar el siguiete programa*/
#include <MotorDriver.h>

MotorDriver mD;
MotorDriver mI;

#define PinLienaI A0
#define PinLienaD A1
#define Echo 10    //Pin servo de arriba (Servo_1)
#define Trigger 9  //Pin servo de Abajo  (Servo_2)
#define MotorDerecho 3
#define MotorIzquierdo 4

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
  Serial.begin(9600);          //iniciar puerto serie
  pinMode(PinLienaI, INPUT);   //definir pin como entrada
  pinMode(PinLienaD, INPUT);   //definir pin como entrada
  pinMode(Trigger, OUTPUT);    //Configuramoms el pin de "trigger" como salida
  pinMode(Echo, INPUT);        //Configuramoms el pin de "echo" como entrada
  digitalWrite(Trigger, LOW);  //Ponemos en voltaje bajo(0V) el pin de "trigger"
}

void loop() {
  if (millis() >= 3750) {
    if (!startSequenceComplete) {
      scan();
      
      // Verificar sensores de línea durante el scan
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
      // Comportamiento normal después de completar el scan
      if (SensorLineaD() == 1 && SensorLineaI() == 1) {
        handleLineDetection(true, true);
      } else if (SensorLineaD() == 1) {
        handleLineDetection(true, false);
      } else if (SensorLineaI() == 1) {
        handleLineDetection(false, true);
      } else if (SensorOponente() == 1) {
        Ataque();
      } else {
        Busqueda();
      }
    }
  }
}

void handleLineDetection(bool derecho, bool izquierdo) {
  if (derecho && izquierdo) {
    //Serial.println("Giro Ambos");
    millisViejo = millis();
    while ((millis() - millisViejo) < tiempoFrenado) { 
      Atras(); 
      // Verificar oponente durante el retroceso
      if (SensorOponente() == 1) { 
        Ataque(); 
        return; 
      }
    }
    millisViejo = millis();
    while ((millis() - millisViejo) < tiempoLimite / 2) {
      GiroDer();
      if (SensorOponente() == 1) { 
        Ataque(); 
        return; 
      }
    }
  } else if (derecho) {
    //Serial.println("Giro Derecho");
    millisViejo = millis();
    while ((millis() - millisViejo) < tiempoFrenado) {
      Atras();
      if (SensorOponente() == 1) { 
        Ataque(); 
        return; 
      }
    }
    millisViejo = millis();
    while ((millis() - millisViejo) < tiempoLimite) {
      GiroIzq(); // Cambié a GiroIzq para alejarse de la línea derecha
      if (SensorOponente() == 1) { 
        Ataque(); 
        return; 
      }
    }
  } else if (izquierdo) {
    //Serial.println("Giro Izquierdo");
    millisViejo = millis();
    while ((millis() - millisViejo) < tiempoFrenado) { 
      Atras(); 
      if (SensorOponente() == 1) { 
        Ataque(); 
        return; 
      }
    }
    millisViejo = millis();
    while ((millis() - millisViejo) < tiempoLimite) {
      GiroDer(); // Cambié a GiroDer para alejarse de la línea izquierda
      if (SensorOponente() == 1) { 
        Ataque(); 
        return; 
      }
    }
  }
}

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

void scan() {
  unsigned long currentTime = millis();
  
  // Si es la primera vez que llamamos scan, inicializar el timer
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
  
  // Verificar si encontramos oponente durante el scan
  if (SensorOponente() == 1) {
    startSequenceComplete = true;
    Serial.println("Oponente encontrado");
    Ataque(); // Comenzar inmediatamente el ataque
  }
}

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

int SensorOponente() {
  unsigned long t;
  float d;

  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigger, LOW);

  t = pulseIn(Echo, HIGH, 30000);
  d = t * 0.000001 * 34300.0 / 2.0;

  if (d <= Distanciaoponente && d > 0) { // Agregué verificación d > 0 para evitar lecturas erróneas
    return 1;
  } else {
    return 0;
  }
}
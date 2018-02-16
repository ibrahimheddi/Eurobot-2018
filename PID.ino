#include <FlexiTimer2.h>
#include <digitalWriteFast.h> 
 
// Codeur incrémental
#define codeurInterruptionA 0
#define codeurInterruptionB 1
#define codeurPinA 2
#define codeurPinB 3
volatile long ticksCodeur = 0;
 
// Moteur CC
#define directionMoteur  4
#define pwmMoteur  5
 
// Cadence d'envoi des données en ms
#define TSDATA 100
unsigned long tempsDernierEnvoi = 0;
unsigned long tempsCourant = 0;
 
// Cadence d'échantillonnage en ms
#define CADENCE_MS 10
volatile double dt = CADENCE_MS/1000.;
volatile double temps = -CADENCE_MS/1000.;
 
volatile double omega;
volatile double commande = 0.;
volatile double vref = 3.14;
 
// PID
volatile double Kp = 0.29;
volatile double Ki = 8.93;
volatile double P_x = 0.;
volatile double I_x = 0.;
volatile double ecart = 0.;
 
// Initialisations
void setup(void) {
 
  // Codeur incrémental
  pinMode(codeurPinA, INPUT);      // entrée digitale pin A codeur
  pinMode(codeurPinB, INPUT);      // entrée digitale pin B codeur
  digitalWrite(codeurPinA, HIGH);  // activation de la résistance de pullup
  digitalWrite(codeurPinB, HIGH);  // activation de la résistance de pullup
  attachInterrupt(codeurInterruptionA, GestionInterruptionCodeurPinA, CHANGE);
  attachInterrupt(codeurInterruptionB, GestionInterruptionCodeurPinB, CHANGE);
 
  // Moteur CC
  pinMode(directionMoteur, OUTPUT);
  pinMode(pwmMoteur, OUTPUT);
 
  // Liaison série
  Serial.begin(9600);
  Serial.flush();
 
  // Compteur d'impulsions de l'encodeur
  ticksCodeur = 0;
 
  // La routine isrt est exécutée à cadence fixe
  FlexiTimer2::set(CADENCE_MS, 1/1000., isrt); // résolution timer = 1 ms
  FlexiTimer2::start();
 
}
 
// Boucle principale
void loop() {
 
  // Ecriture des données sur la liaison série
  ecritureData();
 
}
 
void isrt(){
 
  int codeurDeltaPos;
  double tensionBatterie;
 
  // Nombre de ticks codeur depuis la dernière fois
  codeurDeltaPos = ticksCodeur;
  ticksCodeur = 0;
 
  // Calcul de la vitesse de rotation
  omega = ((2.*3.141592*((double)codeurDeltaPos))/1920.)/dt;  // en rad/s
 
  /******* Régulation PID ********/
  // Ecart entre la consigne et la mesure
  ecart = vref - omega;
 
  // Terme proportionnel
  P_x = Kp * ecart;
 
  // Calcul de la commande
  commande = P_x + I_x;
 
  // Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
  I_x = I_x + Ki * dt * ecart;
  /******* Fin régulation PID ********/
 
  // Envoi de la commande au moteur
  tensionBatterie = 7.2;
  CommandeMoteur(commande, tensionBatterie);
 
  temps += dt;
}
 
void ecritureData(void) {
 
  // Ecriture des données en sortie tous les TSDATA millisecondes
  tempsCourant = millis();
  if (tempsCourant-tempsDernierEnvoi > TSDATA) {
    Serial.print(temps);
 
    Serial.print(",");
    Serial.print(omega);
 
    Serial.print("\r");
    Serial.print("\n");
 
    tempsDernierEnvoi = tempsCourant;
  }
}
 
void CommandeMoteur(double tension, double tensionBatterie)
{
    int tension_int;
 
    // Normalisation de la tension d'alimentation par
        // rapport à la tension batterie
    tension_int = (int)(255*(tension/tensionBatterie));
 
    // Saturation par sécurité
    if (tension_int>255) {
        tension_int = 255;
    }
    if (tension_int<-255) {
        tension_int = -255;
    }
 
        // Commande PWM
    if (tension_int>=0) {
        digitalWrite(directionMoteur, LOW);
        analogWrite(pwmMoteur, tension_int);
    }
    if (tension_int<0) {
        digitalWrite(directionMoteur, HIGH);
        analogWrite(pwmMoteur, -tension_int);
    }
}
 
// Routine de service d'interruption attachée à la voie A du codeur incrémental
void GestionInterruptionCodeurPinA()
{
  if (digitalReadFast2(codeurPinA) == digitalReadFast2(codeurPinB)) {
    ticksCodeur--;
  }
  else {
    ticksCodeur++;
  }
}
 
// Routine de service d'interruption attachée à la voie B du codeur incrémental
void GestionInterruptionCodeurPinB()
{
  if (digitalReadFast2(codeurPinA) == digitalReadFast2(codeurPinB)) {
    ticksCodeur++;
  }
  else {
    ticksCodeur--;
  }
}


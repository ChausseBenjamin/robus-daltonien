/*
Projet: DefiDuParcous2
Equipe: P6
Auteurs: Simon Gagné
Description: Programme pour du robot #2 pour le défi du parcours
Date: Derniere date de modification
*/

/* ****************************************************************************
Inclure les librairies de functions que vous voulez utiliser
**************************************************************************** */
#include <LibRobus.h> // Essentielle pour utiliser RobUS
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <SoftwareSerial.h>
#include <SPI.h>

using namespace std;

/* ****************************************************************************
Variables globales et defines
**************************************************************************** */
// -> defines...
// L'ensemble des fonctions y ont acces
#define BLEU 0
#define VERT 1
#define JAUNE 2
#define ROUGE 3
#define PIN_SUIVEUR A0
#define MICRO_AMBIANT A1
#define MICRO_5K A2

// Pick analog outputs, for the UNO these three work well
// use ~560  ohm resistor between Red & Blue, ~1K for green (its brighter)
#define redpin 30
#define yellowpin 22
#define greenpin 26
#define bluepin 24
// for a common anode LED, connect the common pin to +5V
// for common cathode, connect the common to groundé

// set to false if using a common cathode LED
#define commonAnode false

// our RGB -> eye-recognized gamma color
byte gammatable[256];
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

/* ****************************************************************************
Vos propres fonctions sont creees ici
**************************************************************************** */

// Avance le robot sur une distance
void AvancerDistanceAccel(float distance)
{
	// code
	int PULSE_PAR_CM = 3200 / (3 * PI); // 3pouces de diametre = 7,62 cm
	int MAX_PULSE_PAR_INTERVALLE = 550;
	int MIN_PULSE_PAR_INTERVALLE = 215;

	int pulseAFaire = distance * PULSE_PAR_CM;
	int pulseFait = 0;
	int nbVerif = 0;
	int nbPulseVouluParIntervalle = 200;
	int intervalleTemps = 80;

	int pulseReelTotalGauche = 0;
	int pulseReelTotalDroit = 0;

	int nbPulseVouluTotal = 0;

	// To be updated
	float kp = 0.0005;
	float ki = 0.0001;

	float previousSpeedGauche = 0.2;
	float previousSpeedDroit = 0.2;
	float accelerationFactor = 0.2;
	MOTOR_SetSpeed(LEFT, previousSpeedGauche);
	MOTOR_SetSpeed(RIGHT, previousSpeedDroit);

	while (pulseFait < pulseAFaire)
	{
		delay(intervalleTemps);
		nbVerif++;
		int nbPulseReelGauche = ENCODER_Read(LEFT);
		int nbPulseReelDroit = ENCODER_Read(RIGHT);
		int erreurGauche = nbPulseVouluParIntervalle - nbPulseReelGauche;
		int erreurDroit = nbPulseVouluParIntervalle - nbPulseReelDroit;

		nbPulseVouluTotal += nbPulseVouluParIntervalle;
		if (pulseAFaire - pulseFait < 4000)
		{
			nbPulseVouluParIntervalle -= 100;
			if (nbPulseVouluParIntervalle < MIN_PULSE_PAR_INTERVALLE)
			{
				nbPulseVouluParIntervalle = MIN_PULSE_PAR_INTERVALLE;
			}
		}
		else if (nbPulseVouluParIntervalle < MAX_PULSE_PAR_INTERVALLE)
		{
			nbPulseVouluParIntervalle += 30;
		}
		pulseReelTotalGauche += nbPulseReelGauche;
		pulseReelTotalDroit += nbPulseReelDroit;
		ENCODER_Reset(LEFT);
		ENCODER_Reset(RIGHT);
		int erreurTotalGauche = nbPulseVouluTotal - pulseReelTotalGauche;
		int erreurTotalDroit = nbPulseVouluTotal - pulseReelTotalGauche;

		float correctionGauche = erreurGauche * kp + erreurTotalGauche * ki;
		float correctionDroit = erreurDroit * kp + erreurTotalDroit * ki;

		float newSpeedGauche = previousSpeedGauche + correctionGauche;
		float newSpeedDroit = previousSpeedDroit + correctionDroit;
		MOTOR_SetSpeed(LEFT, newSpeedGauche);
		MOTOR_SetSpeed(RIGHT, newSpeedDroit);

		previousSpeedGauche = newSpeedGauche;
		previousSpeedDroit = newSpeedDroit;
		pulseFait += nbPulseReelDroit;
	}

	MOTOR_SetSpeed(LEFT, 0);
	MOTOR_SetSpeed(RIGHT, 0);
	ENCODER_Reset(LEFT);
	ENCODER_Reset(RIGHT);
}

void AvancerDistanceConstant(float distance)
{
	// code
	int PULSE_PAR_CM = 3200 / (7.62 * PI); // 3pouces de diametres = 7,62 cm

	float pulseAFaire = distance * PULSE_PAR_CM;
	int pulseFait = 0;
	int nbVerif = 0;
	int nbPulseVouluParIntervalle = 300;
	int intervalleTemps = 75;

	int pulseReelTotalGauche = 0;
	int pulseReelTotalDroit = 0;

	// To be updated
	float kp = 0.0005;
	float ki = 0.0001;

	float previousSpeedGauche = 0.25;
	float previousSpeedDroit = 0.20;
	MOTOR_SetSpeed(LEFT, previousSpeedGauche);
	MOTOR_SetSpeed(RIGHT, previousSpeedDroit);

	while (pulseFait < pulseAFaire)
	{
		delay(intervalleTemps);
		nbVerif++;
		int nbPulseReelGauche = ENCODER_Read(LEFT);
		int nbPulseReelDroit = ENCODER_Read(RIGHT);
		int erreurGauche = nbPulseVouluParIntervalle - nbPulseReelGauche;
		int erreurDroit = nbPulseVouluParIntervalle - nbPulseReelDroit;

		Serial.print("Nb pusle reel : ");
		Serial.print(nbPulseReelDroit);
		Serial.print(" ----- Motor Speed Left: ");
		Serial.print(previousSpeedGauche);
		Serial.print(" ----- Erreur Gauche: ");
		Serial.print(erreurGauche);
		Serial.print("\n");

		int pulseVouluTotal = nbPulseVouluParIntervalle * nbVerif;
		pulseReelTotalGauche += nbPulseReelGauche;
		pulseReelTotalDroit += nbPulseReelDroit;
		ENCODER_Reset(LEFT);
		ENCODER_Reset(RIGHT);
		int erreurTotalGauche = pulseVouluTotal - pulseReelTotalGauche;
		int erreurTotalDroit = pulseVouluTotal - pulseReelTotalGauche;

		float correctionGauche = erreurGauche * kp + erreurTotalGauche * ki;
		float correctionDroit = erreurDroit * kp + erreurTotalDroit * ki;

		float newSpeedGauche = previousSpeedGauche + correctionGauche;
		float newSpeedDroit = previousSpeedDroit + correctionDroit;
		MOTOR_SetSpeed(LEFT, newSpeedGauche);
		MOTOR_SetSpeed(RIGHT, newSpeedDroit);

		previousSpeedGauche = newSpeedGauche;
		previousSpeedDroit = newSpeedDroit;
		pulseFait += nbPulseReelDroit;
	}

	MOTOR_SetSpeed(LEFT, 0);
	MOTOR_SetSpeed(RIGHT, 0);
	ENCODER_Reset(LEFT);
	ENCODER_Reset(RIGHT);
}

void AvancerMasterSlave(float distance)
{
	// Right = master
	ENCODER_Reset(LEFT);
	ENCODER_Reset(RIGHT);
	float kp = 0.0005;
	float ki = 0.00005;
	float erreurTotale = 0;

	int PULSE_PAR_UNIT = 3200 / (3 * PI); // 3pouces de diametres = 7,62 cm
	float nbPulseAFaire = distance * PULSE_PAR_UNIT;
	float nbPulseFait = 0;
	float nbPulseVoulu = 0;
	float previousSpeedLeft = 0.15;
	float previousSpeedRight = 0.15;
	MOTOR_SetSpeed(LEFT, 0.15);
	MOTOR_SetSpeed(RIGHT, 0.15);
	bool wasStopped = false;
	while (nbPulseFait < nbPulseAFaire)
	{
		float erreur = 0;
		delay(40);
		int valueIR0 = ROBUS_ReadIR(0);
		int valueIR1 = ROBUS_ReadIR(1);
		if (valueIR0 > 300 || valueIR1 > 300)
		{
			while (valueIR0 > 300 || valueIR1 > 300)
			{
				delay(25);
				valueIR0 = ROBUS_ReadIR(0);
				valueIR1 = ROBUS_ReadIR(1);
				MOTOR_SetSpeed(LEFT, 0);
				MOTOR_SetSpeed(RIGHT, 0);
				wasStopped = true;
			}
		}
		if (wasStopped)
		{
			MOTOR_SetSpeed(LEFT, 0.15);
			MOTOR_SetSpeed(RIGHT, 0.15);
		}
		int pulseLeft = ENCODER_Read(LEFT);
		int pulseRight = ENCODER_Read(RIGHT);
		nbPulseFait += pulseLeft;
		nbPulseVoulu += pulseRight;

		erreur = pulseRight - pulseLeft;
		erreurTotale = nbPulseVoulu - nbPulseFait;

		float correction = erreur * kp + erreurTotale * ki;
		float newSpeedLeft = previousSpeedLeft + correction;
		float newSpeedRight = previousSpeedRight;

		if (previousSpeedRight < 0.3 && nbPulseAFaire - nbPulseFait > 5000)
		{
			float ratio = previousSpeedRight / newSpeedLeft;
			newSpeedRight = previousSpeedRight + 0.005;
			newSpeedLeft = newSpeedRight / ratio;
		}
		if (previousSpeedRight > 0.15 && nbPulseAFaire - nbPulseFait < 5000)
		{
			float ratio = previousSpeedRight / newSpeedLeft;
			newSpeedRight = previousSpeedRight - 0.005;
			newSpeedLeft = newSpeedRight / ratio;
		}

		ENCODER_Reset(LEFT);
		ENCODER_Reset(RIGHT);
		MOTOR_SetSpeed(LEFT, newSpeedLeft);
		MOTOR_SetSpeed(RIGHT, newSpeedRight);
		previousSpeedLeft = newSpeedLeft;
		previousSpeedRight = newSpeedRight;
	}
	MOTOR_SetSpeed(0, 0);
	MOTOR_SetSpeed(1, 0);
}

void FaireArc(int couleur)
{
	float rayonGauche = couleur * 12 + 6 + 3.625; // 3.625 = moitié largeur robot
	float rayonDroit = couleur * 12 + 6 - 3.625;

	float arcGauche = 2 * PI * rayonGauche * 0.25; // 0.25 = 90/360
	float arcDroit = 2 * PI * rayonDroit * 0.25;

	int PULSE_PAR_UNIT = 3200 / (3 * PI); // 3pouces de diametres = 7,62 cm
	float nbPulseAFaireGauche = arcGauche * PULSE_PAR_UNIT;
	float nbPulseAFaireDroit = arcDroit * PULSE_PAR_UNIT;

	if (couleur == BLEU)
	{
		AvancerMasterSlave(2);
		Tourner(90, RIGHT);
		AvancerMasterSlave(2);
	}
	else
	{
		float ratio = nbPulseAFaireGauche / nbPulseAFaireDroit;
		ENCODER_Reset(LEFT);
		ENCODER_Reset(RIGHT);
		float kp = 0.0005;
		float ki = 0.00002;
		float erreurTotale = 0;

		float nbPulseFaitGauche = 0;
		float nbPulseVoulu = 0;
		float previousSpeed = 0.25 * ratio;
		MOTOR_SetSpeed(LEFT, 0.25 * ratio);
		MOTOR_SetSpeed(RIGHT, 0.25);
		while (nbPulseFaitGauche <= nbPulseAFaireGauche)
		{
			float erreur = 0;
			delay(40);
			int pulseLeft = ENCODER_Read(LEFT);
			int pulseRight = ENCODER_Read(RIGHT);
			nbPulseFaitGauche += pulseLeft;
			nbPulseVoulu += pulseRight * ratio;

			erreur = pulseRight * ratio - pulseLeft;
			erreurTotale = nbPulseVoulu - nbPulseFaitGauche;

			float correction = erreur * kp + erreurTotale * ki;
			float newSpeed = previousSpeed + correction;
			ENCODER_Reset(LEFT);
			ENCODER_Reset(RIGHT);
			MOTOR_SetSpeed(LEFT, newSpeed);
			previousSpeed = newSpeed;
		}
		MOTOR_SetSpeed(0, 0);
		MOTOR_SetSpeed(1, 0);
	}
}

void Tourner(float degree, int cote)
{
	ENCODER_Reset(LEFT);
	ENCODER_Reset(RIGHT);
	int moteur = LEFT;
	if (cote == LEFT)
	{
		moteur = RIGHT;
	}
	double nbPulses = 0;
	// Serial.println(nbPulses);
	// bPulses = (910 * (asin((degree*(M_PI / 180.0))/2))/(M_PI / 180.0));
	// Serial.println(nbPulses);
	nbPulses = 3840; // 1 roue
	nbPulses = 3700; // 2 roues

	// float arc  = angle*rayon;
	// nbPulses = (3840 / 90) * degree;
	float nbInches = 7.25 * 2 * PI * (degree / 360);
	// float nbInches = 7.125 * 2 * PI * (degree/360);
	float PULSE_PAR_INCHES = 3200 / (3 * PI);
	nbPulses = nbInches * PULSE_PAR_INCHES;
	int i = 0;
	Serial.println(nbInches);
	Serial.println(nbPulses);

	while (i <= nbPulses)
	{
		MOTOR_SetSpeed(moteur, 0.25);
		// MOTOR_SetSpeed(RIGHT, -0.25);
		//  MOTOR_SetSpeed(1, -0.3);
		i = ENCODER_Read(moteur);
	}

	MOTOR_SetSpeed(LEFT, 0);
	MOTOR_SetSpeed(RIGHT, 0);
	// MOTOR_SetSpeed(1, 0);
}

void Tourner2Roues(float degree, int cote)
{
	int roueMaster = LEFT;
	int roueSlave = RIGHT;
	if (cote == RIGHT)
	{
		roueMaster = RIGHT;
		roueSlave = LEFT;
	}
	ENCODER_Reset(LEFT);
	ENCODER_Reset(RIGHT);
	float kp = 0.0005;
	float ki = 0.00005;
	float erreurTotale = 0;

	int PULSE_PAR_UNIT = 3200 / (3 * PI); // 3pouces de diametres = 7,62 cm
	float nbInches = 3.625 * 2 * PI * (degree / 360);
	float nbPulseAFaire = nbInches * -PULSE_PAR_UNIT;
	float nbPulseFait = 0;
	float nbPulseVoulu = 0;
	float previousSpeed = -0.22;
	MOTOR_SetSpeed(roueSlave, -0.22);
	MOTOR_SetSpeed(roueMaster, 0.20);
	while (nbPulseFait > nbPulseAFaire)
	{
		float erreur = 0;
		delay(40);
		int pulseSlave = ENCODER_Read(roueSlave);
		int pulseMaster = ENCODER_Read(roueMaster);
		nbPulseFait += pulseSlave;
		nbPulseVoulu += pulseMaster;

		erreur = pulseMaster - (-1 * pulseSlave);
		erreurTotale = nbPulseVoulu - (-1 * nbPulseFait);

		float correction = erreur * kp + erreurTotale * ki;
		float newSpeed = previousSpeed - correction;

		ENCODER_Reset(LEFT);
		ENCODER_Reset(RIGHT);
		MOTOR_SetSpeed(roueSlave, newSpeed);
		previousSpeed = newSpeed;
	}
	MOTOR_SetSpeed(0, 0);
	MOTOR_SetSpeed(1, 0);
}

/*typedef struct
{
	int index;
	float pulseValue;
} SmallestIR;

SmallestIR TrouverPlusPetitIR(int leftValue, int rightValue, int frontValue)
{
	SmallestIR retour = new SmallestIR();
	if (distanceLeft < distanceRight)
	{
		if (distanceLeft < distanceFront)
		{

		}
	}
}*/

void ReorienterContinue()
{
	// 597
	// 163
	// 102
	// 82

	// LEFT = 0
	// FRONT = 1
	// RIGHT = 2
	int expectedIRValue = 0;
	int couleur = 1; // FindColor
	switch (couleur)
	{
	case BLEU:
		expectedIRValue = 597;
		break;
	case VERT:
		expectedIRValue = 163;
		break;
	case JAUNE:
		expectedIRValue = 102;
		break;
	case ROUGE:
		expectedIRValue = 82;
		break;
	}
	bool orientationTrouvee = false;

	ENCODER_Reset(LEFT);
	ENCODER_Reset(RIGHT);
	float kp = 0.0005;
	float ki = 0.00005;
	float previousSpeed = -0.22;
	float nbPulseFait = 0;
	float nbPulseVoulu = 0;
	float erreurTotale = 0;
	MOTOR_SetSpeed(LEFT, previousSpeed);
	MOTOR_SetSpeed(RIGHT, 0.20);
	while (!orientationTrouvee)
	{
		float erreur = 0;
		delay(40);
		int pulseLeft = ENCODER_Read(LEFT);
		int pulseRight = ENCODER_Read(RIGHT);
		nbPulseFait += pulseLeft;
		nbPulseVoulu += pulseRight;

		erreur = pulseRight - (-1 * pulseLeft);
		erreurTotale = nbPulseVoulu - (-1 * nbPulseFait);

		float correction = erreur * kp + erreurTotale * ki;
		float newSpeed = previousSpeed - correction;

		ENCODER_Reset(LEFT);
		ENCODER_Reset(RIGHT);

		int distanceRight = ROBUS_ReadIR(2);
		/*Serial.println(distanceRight);
		Serial.println(expectedIRValue);
		Serial.println(orientationTrouvee);
		Serial.println("\n");*/
		if (abs(distanceRight - expectedIRValue) <= 2)
		{
			orientationTrouvee = true;
			MOTOR_SetSpeed(LEFT, 0);
			MOTOR_SetSpeed(RIGHT, 0);
		}
		else
		{
			MOTOR_SetSpeed(LEFT, newSpeed);
			previousSpeed = newSpeed;
		}
	}
}

int ReorienterPar60Deg(int couleurDepart)
{
	int maxValue = 0;
	int currentNbRotations;
	int maxNbRotations;

	for (int i = 0; i < 6; i++)
	{
		int IRRight = ROBUS_ReadIR(2);
		delay(50);

		if (IRRight > maxValue)
		{
			maxValue = IRRight;
			maxNbRotations = currentNbRotations;
		}
		Tourner2Roues(55.5, LEFT);
		delay(500);
		currentNbRotations++;
	}

	for (int i = 0; i < 6 - maxNbRotations; i++)
	{
		Tourner2Roues(55.5, LEFT);
		delay(500);
	}

	int minDiffCouleur = 1000;
	int diffBleu = abs(maxValue - 500);
	if (minDiffCouleur > diffBleu)
	{
		minDiffCouleur = diffBleu;
		couleurDepart = BLEU;
	}
	int diffVert = abs(maxValue - 163);
	if (minDiffCouleur > diffVert)
	{
		minDiffCouleur = diffVert;
		couleurDepart = VERT;
	}
	int diffJaune = abs(maxValue - 103);
	if (minDiffCouleur > diffJaune)
	{
		minDiffCouleur = diffJaune;
		couleurDepart = JAUNE;
	}
	return couleurDepart;
}

void Suivre_Ligne()
{
	// Couleur = color;
	// cette fonction va débuter lorsque les trois capteurs détectent du blanc. Autrement dit, lorsque :
	// analogRead(PIN_SUIVEUR) > 4.65
	// AvancerMasterSlave(6);
	// Tourner(45, RIGHT);
	float lecture = analogRead(PIN_SUIVEUR);
	/*while (lecture > 952.0) //avancer tout droit jusqu'à ce que tu vois une ligne noire
	{
		AvancerMasterSlave(0.01);
		lecture = analogRead(PIN_SUIVEUR);
	}*/
	Tourner(25, RIGHT);
	while (1)
	{
		if (lecture >= 0.0 && lecture < 82.0) // Tous ne détectent pas de blanc
		{
			Serial.println("\nAucun");
			Serial.println(lecture);
			AvancerMasterSlave(0.02); // on se redresse et on avance d'un pouce
		}
		else if (lecture >= 82.0 && lecture < 225.2) // Seulement bleu détecte du blanc
		{
			Serial.println("Bleu");
			Serial.println(lecture);
			Tourner(25, LEFT);
			AvancerMasterSlave(0.02); // on se redresse et on avance d'un pouce
		}
		else if (lecture >= 225.2 && lecture < 368.6) // Seulement jaune détecte du blanc
		{
			Serial.println("Jaune");
			Serial.println(lecture);
			AvancerMasterSlave(0.02); // un peu weird comme lecture pcq jaune est au centre
		}
		else if (lecture >= 368.6 && lecture < 512) // Jaune et bleu détectent du blanc
		{
			Serial.println("Jaune et bleu");
			Serial.println(lecture);
			Tourner(25, LEFT);
			AvancerMasterSlave(0.02);
		}
		else if (lecture >= 512 && lecture < 655.3) // Seulement rouge détecte du blanc
		{
			Serial.println("Rouge");
			Serial.println(lecture);
			Tourner(30, RIGHT);
			AvancerMasterSlave(0.02);
		}
		else if (lecture >= 655.3 && lecture < 809.0) // Rouge et bleu détectent du blanc
		{
			Serial.println("Rouge et bleu");
			Serial.println(lecture);
			AvancerMasterSlave(0.02);
			; // on est pile au centre
		}
		else if (lecture >= 809.0 && lecture < 952) // Rouge et jaune détectent du blanc
		{
			Serial.println("Rouge et jaune");
			Serial.println(lecture);
			Tourner(30, RIGHT);
			AvancerMasterSlave(0.02);
		}
		else // Tous les capteurs détectent du blanc
		{
			Serial.println("Tous");
			Serial.println(lecture);
			// Tourner(8, RIGHT);
			AvancerMasterSlave(0.02);
			;
		}
		delay(10);
		lecture = analogRead(PIN_SUIVEUR);
	}
	// AvancerMasterSlave(6);
	Tourner(45, RIGHT); // pour se redresser
}

void BalayerSurface(float longueurAFaire, float largeurAFaire)
{
	// Doit être dirigé vers la droite (on doit voir son côté droit)
	float LARGEUR_ROBOT = 7.25; // 18.5cm - 7.25in

	float largeurBalayee = 0;
	int nbLargeurFait = 0;

	AvancerMasterSlave(largeurAFaire / 2);
	Tourner2Roues(87 * 2, RIGHT);
	while (largeurBalayee < largeurAFaire)
	{
		if (nbLargeurFait > 0)
		{
			int directionVirage = nbLargeurFait % 2;
			if (directionVirage == 1)
			{
				Tourner(185, directionVirage);
			}
			else
			{
				Tourner(180, directionVirage);
			}
		}
		delay(200);
		AvancerMasterSlave(longueurAFaire);
		largeurBalayee += LARGEUR_ROBOT;
		nbLargeurFait++;
	}
}

void testIR()
{
	int somme = 0;
	int n = 0;
	while (true)
	{
		int distanceRight = ROBUS_ReadIR(0);
		somme += distanceRight;
		n++;
		float moyenne = somme / n;
		Serial.print("Valeur : ");
		Serial.print(distanceRight);
		Serial.print(" --- Moyenne : ");
		Serial.print(moyenne);
		Serial.println("\n");
		delay(25);
	}
}

/* ****************************************************************************
Fonctions d'initialisation (setup)
**************************************************************************** */
// -> Se fait appeler au debut du programme
// -> Se fait appeler seulement un fois
// -> Generalement on y initilise les varibbles globales

void setup()
{
	BoardInit();
	// pinMode(3,OUTPUT); // redpin est une broche de sortie

	/*if (tcs.begin())
	{
		Serial.println("Found sensor");
		tcs.setInterrupt(false); // turn on LED
	}
	else
	{
		Serial.println("No TCS34725 found ... check your connections");
		while (1)
			; // halt!
	}*/

	// use these three pins to drive an LED
	/*pinMode(redpin, OUTPUT);
	pinMode(greenpin, OUTPUT);
	pinMode(bluepin, OUTPUT);
	pinMode(yellowpin, OUTPUT);*/

	// thanks PhilB for this gamma table!
	// it helps convert RGB colors to what humans see
	/*for (int i = 0; i < 256; i++)
	{
		float x = i;
		x /= 255;
		x = pow(x, 2.5);
		x *= 255;

		if (commonAnode)
		{
			gammatable[i] = 255 - x;
		}
		else
		{
			gammatable[i] = x;
		}
		// Serial.println(gammatable[i]);
	}*/
}

/* ****************************************************************************
Fonctions de boucle infini (loop())
**************************************************************************** */
// -> Se fait appeler perpetuellement suite au "setup"

int DeterminerCouleur()
{
	uint16_t clear, red, green, blue;

	delay(150); // takes 50ms to read

	tcs.getRawData(&red, &green, &blue, &clear);

	/*Serial.print("C:\t"); Serial.print(clear);
	Serial.print("\tR:\t"); Serial.print(red);
	Serial.print("\tG:\t"); Serial.print(green);
	Serial.print("\tB:\t"); Serial.print(blue);*/

	// Figure out some basic hex code for visualization
	uint32_t sum = clear;
	float r, g, b;
	r = red;   // r /= sum;
	g = green; // g /= sum;
	b = blue;  // b /= sum;
	// r *= 256; g *= 256; b *= 256;
	/*Serial.print("\t");
	Serial.print((int)r, HEX);
	Serial.print((int)g, HEX);
	Serial.print((int)b, HEX);
	Serial.println();*/
	int retour = color(red, blue, green);
	void turnoff();
	// Serial.print((int)r ); Serial.print(" "); Serial.print((int)g);Serial.print(" ");  Serial.println((int)b );

	/* analogWrite(redpin, gammatable[(int)r]);
	 analogWrite(greenpin, gammatable[(int)g]);
	 analogWrite(bluepin, gammatable[(int)b]);*/

	return retour;
}

int color(int red, int blue, int green)
{
	int retour = 0;
	// if(red && blue=0 && green=0 )
	if (red < blue && red > green)
	{ // rouge
		turnoff();
		digitalWrite(redpin, HIGH);
		// Serial.println("rouge");

		// pinMode(redpin,OUTPUT);
		retour = ROUGE;
	}
	else if (blue > 1.9 * red && green < 1.2 * blue)
	{
		turnoff();
		digitalWrite(greenpin, HIGH);
		// Serial.println("vert");
		retour = VERT;
	}

	else if (green > 1.5 * red && green < blue)
	{
		turnoff();
		digitalWrite(bluepin, HIGH);
		// Serial.println("bleu");
		retour = BLEU;
	}
	else if (red > 1.3 * blue && red > green)
	{
		turnoff();
		digitalWrite(yellowpin, HIGH);
		// Serial.println("jaune");
		retour = JAUNE;
	}
	return retour;
}
/*else if(blue>1.5*red && blue>1.1*green){
		Serial.println("noir");
	}
	else if(blue>red && green>red){
		Serial.println("blanc");
	}*/
/* else{
	Serial.println("tapis");
 }*/

void turnoff()
{
	digitalWrite(redpin, LOW);
	digitalWrite(greenpin, LOW);
	digitalWrite(bluepin, LOW);
	digitalWrite(yellowpin, LOW);
}

void Micro()
{
	float ambiant = 0;
	float Micro5k = 0;
	float diff = 0;

	ambiant = analogRead(MICRO_AMBIANT);
	Micro5k = analogRead(MICRO_5K);
	diff = Micro5k - ambiant;

	while (diff < 60)
	{
		Serial.println(diff);
		ambiant = analogRead(MICRO_AMBIANT);
		Micro5k = analogRead(MICRO_5K);
		diff = Micro5k - ambiant;
	}
}

void loop()
{
	// SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
	delay(100); // Delais pour décharger le CPU
	// SERVO_SetAngle(1, 0);

	int couleurDepart = 1000;
	/*Micro();
	couleurDepart = ReorienterPar60Deg(couleurDepart);
	SERVO_SetAngle(1, 90);
	FaireArc(couleurDepart);
	AvancerMasterSlave(24);
	FaireArc(couleurDepart);
	AvancerMasterSlave(8 * 12);
	Suivre_Ligne();*/
	if (ROBUS_IsBumper(REAR))
	{
		AvancerMasterSlave(82);

		while (true)
		{
			// do nothing --- needed to stop "loop"
		}
	}

	if (ROBUS_IsBumper(LEFT))
	{
		/*Tourner2Roues(87, RIGHT);
		delay(1000);
		Tourner2Roues(87, LEFT);
		delay(1000);
		Tourner2Roues(87*2, LEFT);
		delay(1000);*/
		Tourner(90, LEFT);
		delay(1000);
		Tourner(90, RIGHT);
	}
	if (ROBUS_IsBumper(RIGHT))
	{
		BalayerSurface(60, 60);

		while (true)
		{
			/* do nothing --- needed to stop "loop" */
		}
	}
}
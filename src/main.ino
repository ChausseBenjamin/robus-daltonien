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
	float ki = 0.00002;
	float erreurTotale = 0;

	int PULSE_PAR_UNIT = 3200 / (3 * PI); // 3pouces de diametres = 7,62 cm
	float nbPulseAFaire = distance * PULSE_PAR_UNIT;
	float nbPulseFait = 0;
	float nbPulseVoulu = 0;
	float previousSpeed = 0.27;
	MOTOR_SetSpeed(LEFT, 0.27);
	MOTOR_SetSpeed(RIGHT, 0.25);
	while (nbPulseFait < nbPulseAFaire)
	{
		float erreur = 0;
		delay(40);
		int pulseLeft = ENCODER_Read(LEFT);
		int pulseRight = ENCODER_Read(RIGHT);
		nbPulseFait += pulseLeft;
		nbPulseVoulu += pulseRight;

		erreur = pulseRight - pulseLeft;
		erreurTotale = nbPulseVoulu - nbPulseFait;

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

void FaireArc(int couleur)
{
	float rayonGauche = couleur * 12 + 6 + 3.625; // 3.625 = moitié largeur robot
	float rayonDroit = couleur * 12 + 6 - 3.625;

	float arcGauche = 2 * PI * rayonGauche * 0.25; // 0.25 = 90/360
	float arcDroit = 2 * PI * rayonDroit * 0.25;

	int PULSE_PAR_UNIT = 3200 / (3 * PI); // 3pouces de diametres = 7,62 cm
	float nbPulseAFaireGauche = arcGauche * PULSE_PAR_UNIT;
	float nbPulseAFaireDroit = arcDroit * PULSE_PAR_UNIT;

	float ratio = nbPulseAFaireGauche / nbPulseAFaireDroit;
	ENCODER_Reset(LEFT);
	ENCODER_Reset(RIGHT);
	float kp = 0.0005;
	float ki = 0.00002;
	float erreurTotale = 0;

	float nbPulseFaitGauche = 0;
	float nbPulseVoulu = 0;
	float previousSpeed = 0.27 * ratio;
	MOTOR_SetSpeed(LEFT, 0.27 * ratio);
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

void Tourner(float degree, int cote)
{
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
	float nbInches = 3.625 * 2 * PI * (degree / 360);
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

void Tourner2Roues(float degree)
{
	// Right = master
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
	MOTOR_SetSpeed(LEFT, -0.22);
	MOTOR_SetSpeed(RIGHT, 0.20);
	while (nbPulseFait > nbPulseAFaire)
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
		MOTOR_SetSpeed(LEFT, newSpeed);
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

void Reorienter()
{
	// 500
	// 163
	// 100
	// 82

	// LEFT = 0
	// FRONT = 1
	// RIGHT = 2
	int expectedIRValue = 0;
	int couleur = 1; // FindColor
	switch (couleur)
	{
	case BLEU:
		expectedIRValue = 500;
		break;
	case VERT:
		expectedIRValue = 160;
		break;
	case JAUNE:
		expectedIRValue = 100;
		break;
	case ROUGE:
		expectedIRValue = 80;
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
		if (abs(distanceRight - expectedIRValue) <= 5)
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

void BalayerSurface()
{
	float LARGEUR_ROBOT = 18.5;
	// Test area = 129x126cm

	// Avancer until it scans a line
	// Snake until it scans another line

	float largeurBalayee = 0;
	float largeurABalayer = 134; // To scan somehow
	int nbLargeurFait = 0;

	while (largeurBalayee < largeurABalayer)
	{
		AvancerDistanceConstant(132);
		largeurBalayee += LARGEUR_ROBOT;
		nbLargeurFait++;
		Tourner(180, nbLargeurFait % 2);
	}
}

void testIR()
{
	int somme = 0;
	int n = 0;
	while (true)
	{
		int distanceRight = ROBUS_ReadIR(2);
		somme += distanceRight;
		n++;
		float moyenne = somme / n;
		Serial.println(moyenne);
		delay(50);
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
}

/* ****************************************************************************
Fonctions de boucle infini (loop())
**************************************************************************** */
// -> Se fait appeler perpetuellement suite au "setup"

void loop()
{
	// SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
	delay(100); // Delais pour décharger le CPU
	if (ROBUS_IsBumper(REAR))
	{
		// Différentes parties du parcours
		Reorienter();

		while (true)
		{
			/* do nothing --- needed to stop "loop" */
		}
	}
	if (ROBUS_IsBumper(LEFT))
	{
		// Différentes parties du parcours
		testIR();

		while (true)
		{
			/* do nothing --- needed to stop "loop" */
		}
	}
	if (ROBUS_IsBumper(FRONT))
	{
		// Différentes parties du parcours

		// AvancerMasterSlave(84);
		// FaireArc(VERT);

		while (true)
		{
			/* do nothing --- needed to stop "loop" */
		}
	}
	if (ROBUS_IsBumper(RIGHT))
	{
		// Différentes parties du parcours

		// AvancerMasterSlave(84);
		// FaireArc(BLEU);

		while (true)
		{
			/* do nothing --- needed to stop "loop" */
		}
	}
}
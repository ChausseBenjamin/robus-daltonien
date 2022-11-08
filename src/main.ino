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
	//Right = master
    ENCODER_Reset(LEFT);
    ENCODER_Reset(RIGHT);
	float kp = 0.0005;
	float ki = 0.00005;
	float erreurTotale = 0;

    int PULSE_PAR_UNIT = 3200 / (3 * PI); // 3pouces de diametres = 7,62 cm
	float nbPulseAFaire = distance * PULSE_PAR_UNIT;
	float nbPulseFait = 0;
	float nbPulseVoulu = 0;
	float previousSpeed = 0.25;
	MOTOR_SetSpeed(LEFT, 0.25);
	MOTOR_SetSpeed(RIGHT, 0.25);
    while (nbPulseFait < nbPulseAFaire)
    {
    	float erreur = 0;
    	delay(200);
		int pulseLeft = ENCODER_Read(LEFT);
		int pulseRight = ENCODER_Read(RIGHT);
		nbPulseFait += pulseLeft;
		nbPulseVoulu += pulseRight;

		erreur = pulseRight - pulseLeft;
		erreurTotale = nbPulseVoulu - nbPulseFait;

        float correction = erreur * kp + erreurTotale * ki;
		float newSpeed = previousSpeed + correction;
		Serial.print("Nb pusle gauche : ");
		Serial.print(pulseLeft);
		Serial.print(" ------ Nb pusle droit : ");
		Serial.println(pulseRight);
		Serial.print("Nb total gauche : ");
		Serial.print(nbPulseFait);
		Serial.print(" ------ Nb total droit : ");
		Serial.println(nbPulseVoulu);
		Serial.print("erreur : ");
		Serial.print(erreur);
		Serial.print(" ----- Erreur totale : ");
		Serial.print(erreurTotale);
		Serial.print("\n");
		Serial.print("\n");
		ENCODER_Reset(LEFT);
    	ENCODER_Reset(RIGHT);
		MOTOR_SetSpeed(LEFT, newSpeed);
		previousSpeed = newSpeed;    
    }
    MOTOR_SetSpeed(0, 0);
    MOTOR_SetSpeed(1, 0);
}

void faireArc(int couleur)
{
	float rayonGauche = couleur * 12 + 6 + 3.625; //3.625 = moitié largeur robot
	float rayonDroit = couleur * 12 + 6 - 3.625;

	float arcGauche = 2 * PI * rayonGauche * 0.25; //0.25 = 90/360
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
	float previousSpeed = 0.25 * ratio;
	MOTOR_SetSpeed(LEFT, 0.25 * ratio);
	MOTOR_SetSpeed(RIGHT, 0.25);
    while (nbPulseFaitGauche <= nbPulseAFaireGauche)
    {
    	float erreur = 0;
    	delay(100);
		int pulseLeft = ENCODER_Read(LEFT);
		int pulseRight = ENCODER_Read(RIGHT);
		nbPulseFaitGauche += pulseLeft;
		nbPulseVoulu += pulseRight*ratio;

		erreur = pulseRight*ratio - pulseLeft;
		erreurTotale += nbPulseVoulu - nbPulseFaitGauche;

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

void Tourner(int degree, int cote)
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
	//nbPulses = (3840 / 90) * degree;
	float nbCms = 18.5 * (PI / 180) * degree;
	int PULSE_PAR_CM = 3200 / (7.62 * PI);
	nbPulses = nbCms * PULSE_PAR_CM;
	int i = 0;

	while (i <= nbPulses)
	{
		Serial.println(nbPulses);
		MOTOR_SetSpeed(moteur, 0.3);
		// MOTOR_SetSpeed(1, -0.3);
		i += ENCODER_ReadReset(moteur);
	}

	MOTOR_SetSpeed(moteur, 0);
	// MOTOR_SetSpeed(1, 0);
}

void Faire180()
{
	double nbPulses = 0;
	int i = 0;
	// nbPulses = 3620; //Robot A
	nbPulses = 3950; // Robot B
	while (i <= nbPulses)
	{
		Serial.println(nbPulses);
		MOTOR_SetSpeed(0, 0.2);
		MOTOR_SetSpeed(1, -0.2);
		i += ENCODER_ReadReset(0);
	}

	MOTOR_SetSpeed(RIGHT, 0);
	MOTOR_SetSpeed(LEFT, 0);
	ENCODER_Reset(LEFT);
	ENCODER_Reset(RIGHT);
}

void Reorienter()
{

	float nbCms = 18.5 * (PI / 180) * 360;
	int PULSE_PAR_CM = 3200 / (7.62 * PI);
	float nbPulsesNeeded = nbCms * PULSE_PAR_CM;
	int i = 0;
	float pulseMinIR = 0;
	int minIR = 2000; //High Value

	while (i <= nbPulsesNeeded/2)
	{
		MOTOR_SetSpeed(LEFT, 0.2);
		MOTOR_SetSpeed(RIGHT, -0.2);
		delay(20);
		
		i = ENCODER_Read(LEFT);
		
		uint16_t valeurIR = ROBUS_ReadIR(0);
		if (minIR > valeurIR)
		{
			minIR = valeurIR;
			pulseMinIR = i;
		}
	}
	MOTOR_SetSpeed(LEFT, 0);
	MOTOR_SetSpeed(RIGHT, 0);

	delay(1000);

	while (i >= pulseMinIR)
	{
		MOTOR_SetSpeed(LEFT, -0.2);
		MOTOR_SetSpeed(RIGHT, 0.2);
		delay(20);
		
		i = ENCODER_Read(LEFT);
	}
	MOTOR_SetSpeed(LEFT, 0);
	MOTOR_SetSpeed(RIGHT, 0);
	ENCODER_Reset(LEFT);
}

void BalayerSurface()
{
	float LARGEUR_ROBOT = 18.5;
	//Test area = 129x126cm

	//Avancer until it scans a line
	//Snake until it scans another line

	float largeurBalayee = 0;
	float largeurABalayer = 134; //To scan somehow
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
	while(true)
	{
		uint16_t valeurIR = ROBUS_ReadIR(0);
		Serial.println(valeurIR);
		delay(100);
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
	/*if (ROBUS_IsBumper(REAR))
	{
		// Différentes parties du parcours

		//AvancerMasterSlave(84);
		testIR();


		while (true)
		{
			/* do nothing --- needed to stop "loop" 
		}
	}*/
	
	if (ROBUS_IsBumper(LEFT))
	{
		// Différentes parties du parcours

		//AvancerMasterSlave(84);
		faireArc(JAUNE);


		while (true)
		{
			/* do nothing --- needed to stop "loop" */
		}
	}
	if (ROBUS_IsBumper(FRONT))
	{
		// Différentes parties du parcours

		//AvancerMasterSlave(84);
		faireArc(VERT);


		while (true)
		{
			/* do nothing --- needed to stop "loop" */
		}
	}
	if (ROBUS_IsBumper(RIGHT))
	{
		// Différentes parties du parcours

		//AvancerMasterSlave(84);
		faireArc(BLEU);


		while (true)
		{
			/* do nothing --- needed to stop "loop" */
		}
	}
}
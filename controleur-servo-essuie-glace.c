/* 
 * Gère la position d'un servomoteur comme si il s'agissait d'un
 * essuie glace.
 * 
 * INT1: Allume l'essuie-glaces, puis active/désactive le balayage rapide.
 * INT2: Lance 1 balayage, ou arrête l'essuie-glaces.
 * L'essuie-glace s'arrête toujours au point de repos.
 * 
 * Le signal de contrôle des servomoteurs est difficile à générer, 
 * car il consiste en une pulsation étroite et d'une durée très précise, 
 * mais avec une fréquence très basse.
 * 
 * @author jean-michel-gonet
 */

#include <xc.h>

/**
 * Bits de configuration:
 */
#pragma config FOSC = INTIO67  // Osc. interne, A6 et A7 comme IO.
#pragma config IESO = OFF      // Pas d'osc. au démarrage.
#pragma config FCMEN = OFF     // Pas de monitorage de l'oscillateur.

// Nécessaires pour ICSP / ICD:
#pragma config MCLRE = EXTMCLR // RE3 est actif comme master reset.
#pragma config WDTEN = OFF     // Watchdog inactif.
#pragma config LVP = OFF       // Single Supply Enable bits off.

/**
 * Les 2 bits moins signifiants du rapport cyclique.
 * Sont copiés dans CCP5CON.DC5B
 */
char PWM_ccpr2LSB;

/**
 * Les 8 bits plus signifiants du rapport cyclique.
 * Sont copiés dans CCPR5L.
 */
char PWM_ccpr8MSB;

/**
 * Gère la séquence de 10 étapes de 2ms pour générer le signal
 * complet de 20ms.
 */
void PWM_gereSequence() {
    static char n = 0;

    // Établit le rapport cyclique selon la séquence:
    switch(n) {
        case 0:
            CCPR5L = PWM_ccpr8MSB;
            CCP5CONbits.DC5B = PWM_ccpr2LSB;
            break;
        default:
            CCPR5L = 0;
            CCP5CONbits.DC5B = 0;
    }

    // Suivante étape dans la séquence:
    n++;
    if (n > 9) {
        n = 0;
    }
}

/**
 * Calcule la configuration du PWM pour la position indiquée.
 * @param position Position du servomoteur (Valeur entre -125 et +125).
 */
void PWM_configure(signed char position) {
    unsigned int pwm;

    // Calcule le rapport cyclique selon la position:
    pwm = 375;        // Point milieu entre 250 et 500.
    pwm += position;

    // Sépare le rapport cyclique en 2 + 6 bits:
    PWM_ccpr2LSB = pwm & 0x03;
    PWM_ccpr8MSB = pwm >> 2;
}

/** Limites pour la position du servomoteur. */
#define SERVO_MAX 125
#define SERVO_MIN -125

/**
 * Position du servomoteur.
 * Entre 0 et SERVO_MAX.
 */
signed int SERVO_position;

/**
 * Établit la position du servomoteur.
 * @param position La position.
 */
void SERVO_place(signed char position) {
    SERVO_position = position;
    PWM_configure(position);
}

/**
 * Déplace le servomoteur.
 * @param deplacement Nombre d'unités que le servomoteur doit
 * se déplacer.
 * @return La position du servomoteur.
 */
signed char SERVO_deplace(signed char deplacement) {

    // Déplace le servomoteur:
   SERVO_position += deplacement;

   // Vérifie le dépassement:
   if (SERVO_position > SERVO_MAX) {
       SERVO_position = SERVO_MAX;
   }
   if (SERVO_position < SERVO_MIN) {
       SERVO_position = SERVO_MIN;
   }

   // Configure le PWM:
   PWM_configure((signed char) SERVO_position);

   // Indique la position du servomoteur.
   return SERVO_position;
}

/**
 * États pour la machine à états.
 */
enum Etats {
    REPOS,
    BALAYAGE_1X,
    BALAYAGE
};

/**
 * État actuel de la machine à états.
 */
enum Etats etat = REPOS;

/**
 * Événements pour la machine à états.
 */
enum Evenements {
    STOP,
    START,
    TICTAC
};

#define BALAYAGE_LENT 5
#define BALAYAGE_RAPIDE 2
/**
 * Machine à états.
 * @param e événement à traiter.
 */
void SERVO_machine(enum Evenements e) {
    static unsigned char attente;
    static unsigned char retardement;
    static signed char sens = 1;
    signed char position;

    switch(etat) {
        // L'essuie-glace est à l'arrêt:
        case REPOS:
            switch(e) {
                case STOP:
                    retardement = BALAYAGE_LENT;
                    sens = 1;
                    etat = BALAYAGE_1X;
                    break;

                case START:
                    retardement = BALAYAGE_LENT;
                    sens = 1;
                    etat = BALAYAGE;
                    break;
            }
            break;

        // Effectue 1 balayage, puis s'arrête:
        case BALAYAGE_1X:
            switch(e) {
                case TICTAC:
                    attente++;
                    if (attente >= retardement) {
                        attente = 0;

                        position = SERVO_deplace(sens);

                        if (position >= SERVO_MAX) {
                            sens = -sens;
                        }
                        if (position <= SERVO_MIN) {
                            etat = REPOS;
                        }
                    }
                    break;
            }
            break;

        // Balaye en continu:
        case BALAYAGE:
            switch(e) {
                case STOP:
                    etat = BALAYAGE_1X;
                    break;

                case START:
                    if (retardement == BALAYAGE_LENT) {
                        retardement = BALAYAGE_RAPIDE;
                    } else {
                        retardement = BALAYAGE_LENT;
                    }
                    break;

                case TICTAC:
                    attente++;
                    if (attente >= retardement) {
                        attente = 0;

                        position = SERVO_deplace(sens);

                        if (position >= SERVO_MAX) {
                            sens = -sens;
                        }
                        if (position <= SERVO_MIN) {
                            sens = -sens;
                        }
                    }
                    break;
            }
            break;
    }
}

/**
 * Gère les interruptions.
 */
void interrupt interruptionsHP() {
    if (INTCON3bits.INT2IF) {
        INTCON3bits.INT2IF=0;
        SERVO_machine(STOP);
    }
    if (INTCON3bits.INT1IF) {
        INTCON3bits.INT1IF = 0;
        SERVO_machine(START);
    }
    if (PIR1bits.TMR2IF) {
        PIR1bits.TMR2IF = 0;
        PWM_gereSequence();
        SERVO_machine(TICTAC);
    }
}

/*
 * Initialise le hardware.
 * Configure le temporisateur 2 pour produire des interruptions et un PWM de
 * 2ms de période. Configure également les interruptions INT1 et INT2.
 * Toutes les interruptions sont configurées en haute priorité (Sur le PIC18F, 
 * il n'est pas possible d'activer que les interruptions de basse priorité).
 */
void initialiseHardware() {
    ANSELA = 0x00;      // Désactive les convertisseurs A/D.
    ANSELB = 0x00;      // Désactive les convertisseurs A/D.
    ANSELC = 0x00;      // Désactive les convertisseurs A/D.
    TRISA = 0x00;       // Tous les bits du port A comme sorties.

    // Active le PWM sur CCP5:
    CCPTMRS1bits.C5TSEL = 0;    // CCP5 branché sur tmr2
    T2CONbits.T2CKPS = 1;       // Diviseur de fréq. pour tmr2 sur 32
    T2CONbits.TMR2ON = 1;       // Active le tmr2
    PR2 = 125;                  // Période du tmr2.
    CCP5CONbits.CCP5M = 0xF;    // Active le CCP5.
    TRISAbits.RA4 = 0;          // Active la sortie du CCP5.

    // Prépare les interruptions de haute priorité tmr2:
    PIE1bits.TMR2IE = 1;        // Active les interruptions.
    IPR1bits.TMR2IP = 1;        // En haute priorité.
    PIR1bits.TMR2IF = 0;        // Baisse le drapeau.

    // Prépare les interruptions de basse priorité INT1 et INT2:
    TRISBbits.RB2 = 1;     // INT2 comme entrée digitale.
    TRISBbits.RB1 = 1;     // INT1 comme entrée digitale.

    INTCON2bits.RBPU=0;    // Active les résistances de tirage...
    WPUBbits.WPUB2 = 1;    // ... sur INT2 ...
    WPUBbits.WPUB1 = 1;    // ... et INT1.

    INTCON2bits.INTEDG2 = 0; // Interruptions de INT2 sur flanc descendant.
    INTCON2bits.INTEDG1 = 0; // Interruptions de INT1 sur flanc descendant.

    INTCON3bits.INT2IE = 1;  // Active les interruptions pour INT2...
    INTCON3bits.INT2IP = 1;  // ... en haute priorité.
    INTCON3bits.INT1IE = 1;  // Active les interruptions pour INT1...
    INTCON3bits.INT1IP = 1;  // ... en haute priorité.

    // Active les interruptions de haute priorité:
    RCONbits.IPEN = 1;
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 0;    
}

/**
 * Point d'entrée du programme.
 */
void main() {

    // Initialise les interruptions et les périphériques:
    initialiseHardware();

    // Initialise le servomoteur au point d'arrêt:
    SERVO_place(SERVO_MIN);

    // Les interruptions font tout le travail:
    while(1);
}

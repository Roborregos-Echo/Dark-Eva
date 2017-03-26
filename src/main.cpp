///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////                                         ///////////////////
///////////////////              T E O R Í A                ///////////////////
///////////////////                                         ///////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//------------------------------ VERSIÓN 1.3.8 --------------------------------
//--------------------------- 25 / MARZO / 2017 -----------------------------
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////



//********************************************
//********************************************
//---------------- LIBRERÍAS ----------------
//********************************************
//********************************************
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>
#include <EEPROM.h>
#include <NewPing.h>



//********************************************
//********************************************
//------------------ HEADERS -----------------
//********************************************
//********************************************
void velocidad(int ai, int ad, int ci, int cd);
void resolverLaberinto();
void checarLimit();
void checarColor();
void servoMotor();
void vueltaDer();
void vueltaIzq();



//********************************************
//********************************************
//---------- DECLARACIÓN VARIABLES -----------
//********************************************
//********************************************


//******************************************
//--------------- LABERINTO ----------------
#define switch_preferencia 33
const byte DERECHA      =   0;
const byte IZQUIERDA    =   1;

byte preferencia;


//******************************************
//------------- PANTALLA LCD ---------------
LiquidCrystal_I2C lcd(0x27, 16, 2);


//******************************************
//------------ CLASE CUADRO ----------------

// iEstado
const int NO_EXISTE     =   0;
const int SIN_RECORRER  =   1;
const int RECORRIDO     =   2;
const int CHECKPOINT    =   3;
const int NEGRO         =   4;
const int RAMPA         =   5;
const int INICIO        =   6;

// lastMove
const byte TO_NORTH = 0;
const byte TO_EAST  = 1;
const byte TO_SOUTH = 2;
const byte TO_WEST  = 3;

// iOrientacion
const byte A_norte = 0;
const byte B_norte = 1;
const byte C_norte = 2;
const byte D_norte = 3;

byte iOrientacion = A_norte;

// Coordenadas maximas
const byte X_MAX = 15;
const byte Y_MAX = 15;
const byte Z_MAX = 2;

// Coordeanas actuales
int    x_actual = 0;
int    y_actual = 0;
int    z_actual = 0;

// Ultimas coordeanas vistas
byte    x_last  = 255;
byte    y_last  = 255;
byte    x_last2 = 255;
byte    y_last2 = 255;

// Varias
byte x_InicioB = 255;
byte y_InicioB = 255;
byte z_InicioB = 255;
byte x_InicioC = 255;
byte y_InicioC = 255;
byte z_InicioC = 255;
byte x_inicio, y_inicio, z_inicio;
bool shortMove, Last, Last2;
bool A_wall, B_wall, C_wall, D_wall;

// Se utiliza para guardar las coordenadas de los cuadros "SIN_RECORRER"
byte ArrayCSR;
byte x_recorrer[50];
byte y_recorrer[50];


//******************************************
//--------------- MOTORES ------------------

const int VEL_MOTOR                 =   245;

const int VEL_MOTOR_RAMPA           =   240;
const int VEL_MOTOR_RAMPA_ENCODER   =   255;

const int VEL_MOTOR_VUELTA          =   150;

const int VEL_MOTOR_ALINEAR          =   125;
const int VEL_MOTOR_ALINEAR_ENCODER  =   125;

const int ENC1   = 18;
const int ENC2   = 19;
volatile int steps = 0;

const int MOV_FRENTE                =   0;
const int MOV_RAMPA_SUBIR           =   1;
const int MOV_RAMPA_BAJAR           =   2;
const int MOV_RAMPA_NO_SUBIR        =   3;
const int MOV_RAMPA_NO_BAJAR        =   4;
const int MOV_REVERSA               =   5;
const int MOV_FRENTE_ALINEAR        =   6;
const int MOV_REVERSA_ALINEAR       =   7;


//******************************************
//------------- IMU BNO055 ----------------
bool bajarRampa     = false;
bool subirRampa     = false;

const float PRECISION_IMU = 4.5;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

double setIzq, setDer, inIzq, inDer, outIzq, outDer;
PID izqPID(&inIzq, &outIzq, &setIzq, 15, 0, 0, DIRECT);
PID derPID(&inDer, &outDer, &setDer, 15, 0, 0, REVERSE);

int cuadrosVisitados = 0;
int vueltasDadas = 0;
bool rampaCambio = false;



//******************************************
//----------- SHARP GP2Y0A21YK -------------
const int SHARP_A   = 12;
const int SHARP_B1  = 11;
const int SHARP_B2  = 0;
const int SHARP_C   = 2;
const int SHARP_D1  = 9;
const int SHARP_D2  = 1;


//******************************************
//------------- ULTRASONICOS ---------------
#define TRIG_A 23
#define ECHO_A 24

#define TRIG_B 25
#define ECHO_B 26

#define TRIG_C 27
#define ECHO_C 28

#define TRIG_D 29
#define ECHO_D 30

// Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define MAX_DISTANCE 400

int MARGEN_FALTANTE = 3;
bool bumper     = false;
bool boolUltra  = false;
bool boolAvanzo = false;
int lecturasUltra[20];
int lecturasComparador[20];
int contador_ultra = 0;
int lecturasDiferentes = 0;
int primeraLectura_A, primeraLectura_C;
int segundaLectura_A, segundaLectura_C;
long millisPasado;
char faltanteChar;
int faltante_CM;

NewPing ULTRA_A(TRIG_A, ECHO_A, MAX_DISTANCE);
NewPing ULTRA_B(TRIG_B, ECHO_B, MAX_DISTANCE);
NewPing ULTRA_C(TRIG_C, ECHO_C, MAX_DISTANCE);
NewPing ULTRA_D(TRIG_D, ECHO_D, MAX_DISTANCE);


//******************************************
//------------- PATHFINDING ----------------
const int GRID_MAX = X_MAX * Y_MAX;

int openList[GRID_MAX];
int closedList[GRID_MAX];
int backList[GRID_MAX];
int neighbors[4];


//******************************************
//------------ RAMPA ALGORITHM -------------
const byte ABAJO    = 1;
const byte ARRIBA   = 2;

const byte SUBIR            = 0;
const byte BAJAR            = 1;
const byte SUBIR_BAJAR      = 2;
const byte BAJAR_SUBIR      = 3;
const byte REGRESA_ARRIBA   = 4;
const byte REGRESA_ABAJO    = 5;


bool piso1      = false;
bool piso2      = false;
bool rampaid    = false;

byte firstFloor     = 0;
byte rampaDiff      = 4;
byte lastMove;            // 0,1,0
byte permisoRampa;
byte x_rampa, y_rampa;


//******************************************
//-------------- CHECKPOINT ----------------
const int TOTAL_GRID_MAX = X_MAX * Y_MAX * Z_MAX;
byte checkList1[GRID_MAX];
byte checkList2[GRID_MAX];


//******************************************
//-------------- INTERRUPTS ----------------

// Interupcion del nano;
#define interruptNano 2
const int heatDefiner = 3;
const int visualDefiner = 14;


//******************************************
//------------- SERVO MOTOR ----------------
#define servoPin 8 //PWM
Servo servo;


//******************************************
//---------------- LIMIT -----------------
const int LIMIT_IZQUIERDO = 31;
const int LIMIT_DERECHO = 32;


//******************************************
//---------------- TCS3200 -----------------
#define S0 9
#define S1 10
#define S2 12
#define S3 11
#define sensorOut 13
#define BOTON_COLOR 22

const byte ESTADO_OFF           = 0;
const byte ESTADO_NEGRO         = 1;
const byte ESTADO_CHECKPOINT    = 2;
const byte ESTADO_LISTO         = 3;

const byte COLOR_NEGRO          = 0;
const byte COLOR_CHECKPOINT     = 1;

int iN_CHECKPOINT, iR_CHECKPOINT, iV_CHECKPOINT, iA_CHECKPOINT, iN_NEGRO, iR_NEGRO, iV_NEGRO, iA_NEGRO;
int iNone, iRojo, iVerde, iAzul;
byte EstadoColor = 0;
byte BotonColor;


//********************************************
//********************************************
//---------------- FUNCIONES ----------------
//********************************************
//********************************************
class Cuadro {
    byte estado;
    bool mlx, norte, este, sur, oeste;

    public: Cuadro() {
        estado  = NO_EXISTE;
        norte   = false;
        este    = false;
        sur     = false;
        oeste   = false;
        mlx     = false;
    }

    void setPared(char cSentido, bool bBool) {
        switch(cSentido) {
            case 'N':
                norte = bBool;
                break;
            case 'E':
                este = bBool;
                break;
            case 'S':
                sur = bBool;
                break;
            case 'O':
                oeste = bBool;
                break;
        }
    }

    void setEstado(int iEstado) {
        estado = iEstado;
    }

    void setmlx(bool b) {
        mlx = b;
    }

    bool getmlx() {
        return mlx;
    }

    bool getPared(char cSentido) {
        switch(cSentido) {
            case 'N':
                return norte;
                break;
            case 'E':
                return este;
                break;
            case 'S':
                return sur;
                break;
            case 'O':
                return oeste;
                break;
        }
    }

    int getEstado() {
        return estado;
    }
};

Cuadro cuadros[X_MAX][Y_MAX][Z_MAX];

//********************************************
//------------------- IMU -------------------
float getAngulo() {
    sensors_event_t event;
    bno.getEvent(&event);
    return event.orientation.x;
}


//********************************************
//----------------- MOTORES ------------------
#define Pin1_DERECHA_ADELANTE 50
#define Pin2_DERECHA_ADELANTE 52
#define ENABLE_DERECHA_ADELANTE 7   // PWM

#define Pin1_DERECHA_ATRAS 42
#define Pin2_DERECHA_ATRAS 44
#define ENABLE_DERECHA_ATRAS 5   // PWM

#define Pin1_IZQUIERDA_ADELANTE 46
#define Pin2_IZQUIERDA_ADELANTE 48
#define ENABLE_IZQUIERDA_ADELANTE 6   // PWM

#define Pin1_IZQUIERDA_ATRAS 40
#define Pin2_IZQUIERDA_ATRAS 38
#define ENABLE_IZQUIERDA_ATRAS 4   // PWM


void avanzar() {
    digitalWrite(Pin1_DERECHA_ADELANTE, LOW);
    digitalWrite(Pin1_DERECHA_ATRAS, LOW);
    digitalWrite(Pin1_IZQUIERDA_ADELANTE, LOW);
    digitalWrite(Pin1_IZQUIERDA_ATRAS, LOW);

    digitalWrite(Pin2_DERECHA_ADELANTE, HIGH);
    digitalWrite(Pin2_DERECHA_ATRAS, HIGH);
    digitalWrite(Pin2_IZQUIERDA_ADELANTE, HIGH);
    digitalWrite(Pin2_IZQUIERDA_ATRAS, HIGH);
}

void reversa() {
    digitalWrite(Pin1_DERECHA_ADELANTE, HIGH);
    digitalWrite(Pin1_DERECHA_ATRAS, HIGH);
    digitalWrite(Pin1_IZQUIERDA_ADELANTE, HIGH);
    digitalWrite(Pin1_IZQUIERDA_ATRAS, HIGH);

    digitalWrite(Pin2_DERECHA_ADELANTE, LOW);
    digitalWrite(Pin2_DERECHA_ATRAS, LOW);
    digitalWrite(Pin2_IZQUIERDA_ADELANTE, LOW);
    digitalWrite(Pin2_IZQUIERDA_ATRAS, LOW);
}

void detener() {
    velocidad(255, 255, 255, 255);

    digitalWrite(Pin1_DERECHA_ADELANTE, HIGH);
    digitalWrite(Pin1_DERECHA_ATRAS, HIGH);
    digitalWrite(Pin1_IZQUIERDA_ADELANTE, HIGH);
    digitalWrite(Pin1_IZQUIERDA_ATRAS, HIGH);

    digitalWrite(Pin2_DERECHA_ADELANTE, HIGH);
    digitalWrite(Pin2_DERECHA_ATRAS, HIGH);
    digitalWrite(Pin2_IZQUIERDA_ADELANTE, HIGH);
    digitalWrite(Pin2_IZQUIERDA_ATRAS, HIGH);

    delay(100);

    digitalWrite(Pin1_DERECHA_ADELANTE, LOW);
    digitalWrite(Pin1_DERECHA_ATRAS, LOW);
    digitalWrite(Pin1_IZQUIERDA_ADELANTE, LOW);
    digitalWrite(Pin1_IZQUIERDA_ATRAS, LOW);

    digitalWrite(Pin2_DERECHA_ADELANTE, LOW);
    digitalWrite(Pin2_DERECHA_ATRAS, LOW);
    digitalWrite(Pin2_IZQUIERDA_ADELANTE, LOW);
    digitalWrite(Pin2_IZQUIERDA_ATRAS, LOW);
}

void vueltaDerecha() {
    digitalWrite(Pin1_DERECHA_ADELANTE, HIGH);
    digitalWrite(Pin1_DERECHA_ATRAS, HIGH);
    digitalWrite(Pin1_IZQUIERDA_ADELANTE, LOW);
    digitalWrite(Pin1_IZQUIERDA_ATRAS, LOW);

    digitalWrite(Pin2_DERECHA_ADELANTE, LOW);
    digitalWrite(Pin2_DERECHA_ATRAS, LOW);
    digitalWrite(Pin2_IZQUIERDA_ADELANTE, HIGH);
    digitalWrite(Pin2_IZQUIERDA_ATRAS, HIGH);
}

void vueltaIzquierda() {
    digitalWrite(Pin1_DERECHA_ADELANTE, LOW);
    digitalWrite(Pin1_DERECHA_ATRAS, LOW);
    digitalWrite(Pin1_IZQUIERDA_ADELANTE, HIGH);
    digitalWrite(Pin1_IZQUIERDA_ATRAS, HIGH);

    digitalWrite(Pin2_DERECHA_ADELANTE, HIGH);
    digitalWrite(Pin2_DERECHA_ATRAS, HIGH);
    digitalWrite(Pin2_IZQUIERDA_ADELANTE, LOW);
    digitalWrite(Pin2_IZQUIERDA_ATRAS, LOW);
}

void horizontalDerecha() {
    digitalWrite(Pin1_DERECHA_ADELANTE, HIGH);
    digitalWrite(Pin1_DERECHA_ATRAS, LOW);
    digitalWrite(Pin1_IZQUIERDA_ADELANTE, LOW);
    digitalWrite(Pin1_IZQUIERDA_ATRAS, HIGH);

    digitalWrite(Pin2_DERECHA_ADELANTE, LOW);
    digitalWrite(Pin2_DERECHA_ATRAS, HIGH);
    digitalWrite(Pin2_IZQUIERDA_ADELANTE, HIGH);
    digitalWrite(Pin2_IZQUIERDA_ATRAS, LOW);
}

void horizontalIzquierda() {
    digitalWrite(Pin1_DERECHA_ADELANTE, LOW);
    digitalWrite(Pin1_DERECHA_ATRAS, HIGH);
    digitalWrite(Pin1_IZQUIERDA_ADELANTE, HIGH);
    digitalWrite(Pin1_IZQUIERDA_ATRAS, LOW);

    digitalWrite(Pin2_DERECHA_ADELANTE, HIGH);
    digitalWrite(Pin2_DERECHA_ATRAS, LOW);
    digitalWrite(Pin2_IZQUIERDA_ADELANTE, LOW);
    digitalWrite(Pin2_IZQUIERDA_ATRAS, HIGH);
}


//******************************************
//----------------- SHARP ------------------
float getSharpCorta(int iSharp) {
    //int sharp = 2316.6 *(0.985 / analogRead(A7));  viejo
    //3742.4 * (1 / pow(sharpRead[3], 1.081)); osvaldo
    //3582.4 * (pow(sharpRead[3], -1.047)); neto
    int sharpRead[20];
    float resultado = 0, promedio = 0;
    for(int i = 0; i < 20; i++)
        sharpRead[i] = analogRead(iSharp);

    for (int j = 0; j < 20; j++) {
        for (int i = 0; i < 19; i++) {
            int temp;
            if(sharpRead[i] > sharpRead[i + 1]) {
                temp = sharpRead[i + 1];
                sharpRead[i + 1] = sharpRead[i];
                sharpRead[i] = temp;
            }
        }
    }

    for (int i = 5; i < 15; i++)
        promedio += sharpRead[i];

    promedio /= 10;

    if (promedio >= 175 && promedio <= 550)
        resultado = 2429 * (pow(promedio, -1.004));
    else
        resultado = 30;

    return resultado;
}

int getUltrasonicoMediana(char cSentido) {
    switch(cSentido) {
        case 'A':
            return NewPing::convert_cm(ULTRA_A.ping_median(3));

        case 'B':
            return NewPing::convert_cm(ULTRA_B.ping_median(3));

        case 'C':
            return NewPing::convert_cm(ULTRA_C.ping_median(3));

        case 'D':
            return NewPing::convert_cm(ULTRA_D.ping_median(3));
    }
}

int getUltrasonicoUno(char cSentido) {
    switch(cSentido) {
        case 'A':
            return ULTRA_A.ping_cm();

        case 'B':
            return ULTRA_B.ping_cm();

        case 'C':
            return ULTRA_C.ping_cm();

        case 'D':
            return ULTRA_D.ping_cm();
    }
}

void agregarLecturas(char cSentido) {
    lecturasUltra[contador_ultra++] = getUltrasonicoUno(cSentido);
}

void checarAvance() {
    for(int i = 0; i < 20; i++)
        lecturasComparador[i] = lecturasUltra[i];

    for(int i = 0; i < 20; i++) {
        for(int j = 0; j < 20; j++) {
            if(lecturasComparador[j] == lecturasUltra[i]) {
                lecturasComparador[j] = 0;
                boolUltra = true;
            }
        }
        if(boolUltra) {
            lecturasDiferentes++;
            boolUltra = false;
        }
    }

    if(lecturasDiferentes > 10)
        boolAvanzo = true;
    else
        boolAvanzo = false;


    for(int i=0; i < 20; i++)
        lecturasUltra[i] = 0;

    contador_ultra = 0;
    boolUltra = false;
}


void primeraLectura() {
    primeraLectura_A = getUltrasonicoMediana('A');
    primeraLectura_C = getUltrasonicoMediana('C');
}

/*void comprobarAvance() {
    if(primeraLectura_A != 0) {
        faltanteChar = 'A';
        segundaLectura_A = getUltrasonicoMediana('A');
        if(segundaLectura_A == 0 || (abs(primeraLectura_A - segundaLectura_A) >= (30 - MARGEN_FALTANTE) &&
        abs(primeraLectura_A - segundaLectura_A) <= (30 + MARGEN_FALTANTE))) {
            faltante_CM = 0;
        } else {
            faltante_CM = 30 - abs(primeraLectura_A - segundaLectura_A);
        }
    } else if(primeraLectura_C != 0) {
        faltanteChar = 'C';
        segundaLectura_C = getUltrasonicoMediana('C');
        if(segundaLectura_C == 0 || (abs(primeraLectura_C - segundaLectura_C) >= (30 - MARGEN_FALTANTE) &&
        abs(primeraLectura_C - segundaLectura_C) <= (30 + MARGEN_FALTANTE))) {
            faltante_CM = 0;
        } else {
            faltante_CM = 30 - abs(primeraLectura_C - segundaLectura_C);
        }
    } else {
        faltante_CM = 0;
    }
}*/

void checarFaltante() {
    faltante_CM = map(lecturasDiferentes, 1, 20, 30, 0);
}
//******************************************
//---------------- ENCODER -----------------
void addStep() {
  steps++;
}


//******************************************
//******************************************
//---------------interrupts-----------------
//******************************************
//******************************************

bool inFire = false;
bool first_victim = true;

void victim_Detected() {
     inFire = true;
}



void checarInterr() {
    if(inFire == true && !cuadros[x_actual][y_actual][z_actual].getmlx()) {
        int lecturaB = getUltrasonicoUno('B');
        int lecturaD = getUltrasonicoUno('D');

        if(digitalRead(heatDefiner) == 0 && digitalRead(visualDefiner) == 0) {
            if(lecturaB != 0 && lecturaB < 15 && getSharpCorta(SHARP_B1) < 15 && getSharpCorta(SHARP_B2) < 15)
            {
                detener();
                lcd.clear();
                for (int i = 0; i < 8; i++) {
                    lcd.noBacklight();
                    delay(100);
                    lcd.backlight();
                    delay(100);
                }
                int pos = steps;
                steps = 0;

                lcd.print("VICTIMA DERECHA");
                vueltaIzq();
                servoMotor();
                if(first_victim) {
                    delay(500);
                    servoMotor();
                    first_victim = false;
                }
                vueltaDer();

                cuadros[x_actual][y_actual][z_actual].setmlx(true);
                inFire = false;
                steps = pos;
            }

        } else if (digitalRead(heatDefiner) == 1 && digitalRead(visualDefiner) == 0){
            if(lecturaD != 0 && lecturaD < 15 && getSharpCorta(SHARP_D1) < 15 && getSharpCorta(SHARP_D2) < 15)
            {
                detener();
                lcd.clear();
                for (int i = 0; i < 8; i++) {
                    lcd.noBacklight();
                    delay(100);
                    lcd.backlight();
                    delay(100);
                }
                int pos = steps;
                steps = 0;

                lcd.print("VICTIMA IZQUIERDA");
                vueltaDer();
                servoMotor();
                if(first_victim) {
                    delay(500);
                    servoMotor();
                    first_victim = false;
                  }
                  vueltaIzq();

                  cuadros[x_actual][y_actual][z_actual].setmlx(true);
                  inFire = false;
                  steps = pos;
            }
        } else if (digitalRead(heatDefiner) == 0 && digitalRead(visualDefiner) == 1){
            if(lecturaD != 0 && lecturaD < 15 && getSharpCorta(SHARP_D1) < 15 && getSharpCorta(SHARP_D2) < 15)
            {
                detener();
                lcd.clear();
                for (int i = 0; i < 8; i++) {
                    lcd.noBacklight();
                    delay(100);
                    lcd.backlight();
                    delay(100);
                }
                int pos = steps;
                steps = 0;

                lcd.print("VICTIMA VISUAL");
                vueltaDer();
                servoMotor();
                if(first_victim) {
                    delay(500);
                    servoMotor();
                    first_victim = false;
                  }
                  vueltaIzq();

                  cuadros[x_actual][y_actual][z_actual].setmlx(true);
                  inFire = false;
                  steps = pos;
            }

        }

    }
}


void checarLimit() {
    bool izq = false;
    bool der = false;

    if(digitalRead(LIMIT_IZQUIERDO) == 1)
        izq = true;

    if(digitalRead(LIMIT_DERECHO) == 1)
        der = true;

    if(izq || der) {
        detener();
        lcd.clear();
        lcd.print("  LIMIT  ");
        int pos = steps;
        steps = 0;
        for (int i = 0; i < 4; i++) {
            lcd.noBacklight();
            delay(50);
            lcd.backlight();
            delay(50);
        }

        if(izq && der) {
            lcd.setCursor(1, 0);
            lcd.print("  IZQ     DER ");
            while (steps <= 700) {
                reversa();
            }
            detener();
            //TODO: PONER PARED
            steps = 9999;
        } else if (izq){
            lcd.setCursor(1, 0);
            lcd.print("  IZQ");
            while (steps <= 500) {
                reversa();
            }
            detener();
            while (steps <= 1000) {
                horizontalDerecha();
            }
            detener();
            steps = pos - 500;
        } else if (der){
            lcd.setCursor(1, 0);
            lcd.print("           DER");
            while (steps <= 500) {
                reversa();
            }
            detener();
            while (steps <= 1000) {
                horizontalIzquierda();
            }
            detener();
            steps = pos - 500;
        }
    }
}



//******************************************
//--------------- MOVIMIENTO ---------------
void velocidad(int ai, int ad, int ci, int cd) {
    if(ad > 255)
        analogWrite(ENABLE_DERECHA_ADELANTE, 255);
    else
        analogWrite(ENABLE_DERECHA_ADELANTE, ad);

    if(cd > 255)
        analogWrite(ENABLE_DERECHA_ATRAS, 255);
    else
        analogWrite(ENABLE_DERECHA_ATRAS, cd);

    if(ai > 255)
        analogWrite(ENABLE_IZQUIERDA_ADELANTE, 255);
    else
        analogWrite(ENABLE_IZQUIERDA_ADELANTE, ai);

    if(ci > 255)
        analogWrite(ENABLE_IZQUIERDA_ATRAS, 255);
    else
        analogWrite(ENABLE_IZQUIERDA_ATRAS, ci);
}


void alinear() {
    int lecturaUltra, lecturaSharp;
    bool alfa       =   false;
    bool bravo      =   false;
    bool charlie    =   false;
    bool delta      =   false;

    velocidad(VEL_MOTOR_ALINEAR, VEL_MOTOR_ALINEAR_ENCODER, VEL_MOTOR_ALINEAR, VEL_MOTOR_ALINEAR);

    lecturaUltra = getUltrasonicoUno('A');
    lecturaSharp = getSharpCorta(SHARP_A);
    if(0 < lecturaUltra && lecturaUltra < 20 && 0 < lecturaSharp && lecturaSharp < 20)
        alfa = true;

    lecturaUltra = getUltrasonicoUno('B');
    lecturaSharp = getSharpCorta(SHARP_B1);
    if(0 < lecturaUltra && lecturaUltra < 20 && 0 < lecturaSharp && lecturaSharp < 20)
        bravo = true;

    lecturaUltra = getUltrasonicoUno('C');
    lecturaSharp = getSharpCorta(SHARP_C);
    if(0 < lecturaUltra && lecturaUltra < 20 && 0 < lecturaSharp && lecturaSharp < 20)
        charlie = true;

    lecturaUltra = getUltrasonicoUno('D');
    lecturaSharp = getSharpCorta(SHARP_D1);
    if(0 < lecturaUltra && lecturaUltra < 20 && 0 < lecturaSharp && lecturaSharp < 20)
        delta = true;

    if(bravo && delta) {
        while (abs(getSharpCorta(SHARP_B1) - getSharpCorta(SHARP_D1)) > 1.5) {
            unsigned long inicio = millis();
            while(getSharpCorta(SHARP_B1) - getSharpCorta(SHARP_D1) > 1.5) {
                lcd.clear();
                lcd.print(111111);
                horizontalDerecha();
                if (millis() >= inicio + 800) {
                    detener();
                    return;
                }
            }
            detener();

            inicio = millis();
            while(getSharpCorta(SHARP_D1) - getSharpCorta(SHARP_B1) > 1.5) {
                lcd.clear();
                lcd.print(222222);
                horizontalIzquierda();
                if (millis() >= inicio + 800) {
                    detener();
                    return;
                }
            }
            detener();
        }
    } else if(bravo) {
        while (!(getSharpCorta(SHARP_B1) > 6.5 && getSharpCorta(SHARP_B1) < 8.5)) {
            unsigned long inicio = millis();
            while (getSharpCorta(SHARP_B1) < 6.5) {
                lcd.clear();
                lcd.print(333333);
                horizontalIzquierda();
                if (millis() >= inicio + 800) {
                    detener();
                    return;
                }
            }
            detener();
            inicio = millis();
            while (getSharpCorta(SHARP_B1) > 8.5) {
                lcd.clear();
                lcd.print(444444);
                horizontalDerecha();
                if (millis() >= inicio + 800) {
                    detener();
                    return;
                }
            }
            detener();
        }

        /*if (abs(getSharpCorta(SHARP_B1) - getSharpCorta(SHARP_B2)) > 1.5 ) {
            while (getSharpCorta(SHARP_B2) < getSharpCorta(SHARP_B1)) {
                vueltaDerecha();
            }
            detener();
            while (getSharpCorta(SHARP_B1) < getSharpCorta(SHARP_B2)) {
                vueltaIzquierda();
            }
            detener();
        }*/
    } else if(delta) {
        while (!(getSharpCorta(SHARP_D1) > 6.5 && getSharpCorta(SHARP_D1) < 8.5)) {
            unsigned long inicio = millis();
            while (getSharpCorta(SHARP_D1) < 6.5) {
                lcd.clear();
                lcd.print(555555);
                horizontalDerecha();
                if (millis() >= inicio + 800) {
                    detener();
                    return;
                }
            }
            detener();
            inicio = millis();
            while (getSharpCorta(SHARP_D1) >  8.5) {
                lcd.clear();
                lcd.print(666666);
                horizontalIzquierda();
                if (millis() >= inicio + 800) {
                    detener();
                    return;
                }
            }
            detener();
        }
        /*if (abs(getSharpCorta(SHARP_D1) - getSharpCorta(SHARP_D2)) > 1.5 ) {
            while (getSharpCorta(SHARP_D2) < getSharpCorta(SHARP_D1)) {
                vueltaIzquierda();
            }
            detener();
            while (getSharpCorta(SHARP_D1) < getSharpCorta(SHARP_D2)) {
                vueltaDerecha();
            }
            detener();
        }*/
    }

    if (alfa) {
        unsigned long inicio;
        while (!(getSharpCorta(SHARP_A) > 7 && getSharpCorta(SHARP_A) < 9)) {
            inicio = millis();
            while (getSharpCorta(SHARP_A) < 7) {
                lcd.clear();
                lcd.print(777777);
                reversa();
                if (millis() >= inicio + 800) {
                    detener();
                    return;
                }
            }
            detener();
            inicio = millis();
            while (getSharpCorta(SHARP_A) > 9) {
                lcd.clear();
                lcd.print(888888);
                avanzar();
                if (millis() >= inicio + 800) {
                    detener();
                    return;
                }
            }
            detener();
        }
    } else if (charlie) {
        unsigned long inicio;
        while (!(getSharpCorta(SHARP_C) > 7 && getSharpCorta(SHARP_C) < 9)) {
            inicio = millis();
            while (getSharpCorta(SHARP_C) < 7) {
                lcd.clear();
                lcd.print(999999);
                avanzar();
                if (millis() >= inicio + 800) {
                    detener();
                    return;
                }
            }
            detener();

            inicio = millis();
            while (getSharpCorta(SHARP_C) > 9) {
                lcd.clear();
                lcd.print(123456);
                reversa();
                if (millis() >= inicio + 800) {
                    detener();
                    return;
                }
            }
            detener();
        }
    }
}



void vueltaIzq() {
    vueltasDadas++;
    float posInicial, posFinal, limInf, limSup;
    posInicial = getAngulo();
    lcd.setCursor(8, 1);
    lcd.print("Vuel Izq");
    switch(iOrientacion) {
        case A_norte:
            posFinal = 270;
            setIzq = 270;
            setDer = 270;
            break;

        case B_norte:
            posFinal = 180;
            setIzq = 180;
            setDer = 180;
            break;

        case C_norte:
            posFinal = 90;
            setIzq = 90;
            setDer = 90;
            break;

        case D_norte:
            posFinal = 0;
            setIzq = 0;
            setDer = 0;
            break;
    }

    if (posFinal - PRECISION_IMU <= 0)
        limInf = posFinal + 360 - PRECISION_IMU;
    else
        limInf = posFinal - PRECISION_IMU;

    if (posFinal + PRECISION_IMU > 360)
        limSup =  posFinal - 360 + PRECISION_IMU;
    else
        limSup = posFinal + PRECISION_IMU;

    unsigned long inicio = millis();
    velocidad(VEL_MOTOR_VUELTA, VEL_MOTOR_VUELTA, VEL_MOTOR_VUELTA, VEL_MOTOR_VUELTA);
    vueltaIzquierda();

    if(limSup > limInf) {
        while(!(posInicial >= limInf && posInicial <= limSup)) {
            posInicial = getAngulo();
            if (millis() >= inicio + 6000) {
                velocidad(VEL_MOTOR, VEL_MOTOR, VEL_MOTOR, VEL_MOTOR);
            } else if (millis() >= inicio + 12000) {
                velocidad(VEL_MOTOR + 35, VEL_MOTOR + 35, VEL_MOTOR + 35, VEL_MOTOR +35);
            } else if (millis() >= inicio + 18000) {
                detener();
                vueltaIzquierda();
                velocidad(VEL_MOTOR_RAMPA, VEL_MOTOR_RAMPA, VEL_MOTOR_RAMPA, VEL_MOTOR_RAMPA);
                delay(500);
                velocidad(VEL_MOTOR + 35, VEL_MOTOR + 35, VEL_MOTOR + 35, VEL_MOTOR +35);
            }
        }
        detener();
    } else {
        while(!(posInicial >= limInf || posInicial <= limSup)) {
            posInicial = getAngulo();
            if (millis() >= inicio + 6000) {
                velocidad(VEL_MOTOR, VEL_MOTOR, VEL_MOTOR, VEL_MOTOR);
            } else if (millis() >= inicio + 12000) {
                velocidad(VEL_MOTOR + 35, VEL_MOTOR + 35, VEL_MOTOR + 35, VEL_MOTOR +35);
            } else if (millis() >= inicio + 18000) {
                detener();
                vueltaDerecha();
                velocidad(VEL_MOTOR_RAMPA, VEL_MOTOR_RAMPA, VEL_MOTOR_RAMPA, VEL_MOTOR_RAMPA);
                delay(500);
                velocidad(VEL_MOTOR + 35, VEL_MOTOR + 35, VEL_MOTOR + 35, VEL_MOTOR +35);
            }
        }
        detener();
    }
    velocidad(VEL_MOTOR, VEL_MOTOR, VEL_MOTOR, VEL_MOTOR);
    detener();

    switch(iOrientacion) {
        case A_norte:
            iOrientacion = B_norte;
            break;

        case B_norte:
            iOrientacion = C_norte;
            break;

        case C_norte:
            iOrientacion = D_norte;
            break;

        case D_norte:
            iOrientacion = A_norte;
            break;
    }
}

void vueltaDer() {
    vueltasDadas++;
    float posInicial, posFinal, limInf, limSup;
    lcd.setCursor(8, 1);
    lcd.print("Vuel Der");
    posInicial = getAngulo();
    switch(iOrientacion) {
        case A_norte:
            posFinal = 90;
            setIzq = 90;
            setDer = 90;
            break;

        case B_norte:
            posFinal = 0;
            setIzq = 0;
            setDer = 0;
            break;

        case C_norte:
            posFinal = 270;
            setIzq = 270;
            setDer = 270;
            break;

        case D_norte:
            posFinal = 180;
            setIzq = 180;
            setDer = 180;
            break;
    }

    if (posFinal - PRECISION_IMU <= 0)
        limInf = posFinal + 360 - PRECISION_IMU;
    else
        limInf = posFinal - PRECISION_IMU;

    if (posFinal + PRECISION_IMU > 360)
        limSup =  posFinal - 360 + PRECISION_IMU;
    else
        limSup = posFinal + PRECISION_IMU;

    unsigned long inicio = millis();
    velocidad(VEL_MOTOR_VUELTA, VEL_MOTOR_VUELTA, VEL_MOTOR_VUELTA, VEL_MOTOR_VUELTA);
    vueltaDerecha();

    if(limSup > limInf) {
        while(!(posInicial >= limInf && posInicial <= limSup)) {
            posInicial = getAngulo();
            if (millis() >= inicio + 6000) {
                velocidad(VEL_MOTOR, VEL_MOTOR, VEL_MOTOR, VEL_MOTOR);
            } else if (millis() >= inicio + 12000) {
                velocidad(VEL_MOTOR + 35, VEL_MOTOR + 35, VEL_MOTOR + 35, VEL_MOTOR +35);
            } else if (millis() >= inicio + 18000) {
                detener();
                vueltaIzquierda();
                velocidad(VEL_MOTOR_RAMPA, VEL_MOTOR_RAMPA, VEL_MOTOR_RAMPA, VEL_MOTOR_RAMPA);
                delay(500);
                velocidad(VEL_MOTOR + 35, VEL_MOTOR + 35, VEL_MOTOR + 35, VEL_MOTOR +35);
            }
        }
        detener();
    } else {
        while(!(posInicial >= limInf || posInicial <= limSup)) {
            posInicial = getAngulo();
            if (millis() >= inicio + 6000) {
                velocidad(VEL_MOTOR, VEL_MOTOR, VEL_MOTOR, VEL_MOTOR);
            } else if (millis() >= inicio + 12000) {
                velocidad(VEL_MOTOR + 35, VEL_MOTOR + 35, VEL_MOTOR + 35, VEL_MOTOR +35);
            } else if (millis() >= inicio + 18000) {
                detener();
                vueltaIzquierda();
                velocidad(VEL_MOTOR_RAMPA, VEL_MOTOR_RAMPA, VEL_MOTOR_RAMPA, VEL_MOTOR_RAMPA);
                delay(500);
                velocidad(VEL_MOTOR + 35, VEL_MOTOR + 35, VEL_MOTOR + 35, VEL_MOTOR +35);
            }
        }
        detener();
    }
    velocidad(VEL_MOTOR, VEL_MOTOR, VEL_MOTOR, VEL_MOTOR);
    detener();

    switch(iOrientacion) {
        case A_norte:
            iOrientacion = D_norte;
            break;

        case B_norte:
            iOrientacion = A_norte;
            break;

        case C_norte:
            iOrientacion = B_norte;
            break;

        case D_norte:
            iOrientacion = C_norte;
            break;
    }
}



void setNewPos() {
    switch(lastMove) {
        case TO_NORTH:
        y_actual += rampaDiff;
        break;

        case TO_EAST:
        x_actual += rampaDiff;
        break;

        case TO_SOUTH:
        y_actual -= rampaDiff;
        break;

        case TO_WEST:
        x_actual -= rampaDiff;
        break;
    }
}

void setRampa() {
    switch(lastMove) {
        case TO_NORTH:
        cuadros[x_actual][y_actual-1][z_actual].setEstado(RAMPA);
        break;

        case TO_EAST:
        cuadros[x_actual-1][y_actual][z_actual].setEstado(RAMPA);
        break;

        case TO_SOUTH:
        cuadros[x_actual][y_actual-1][z_actual].setEstado(RAMPA);
        break;

        case TO_WEST:
        cuadros[x_actual+1][y_actual][z_actual].setEstado(RAMPA);
        break;
    }
}


void checarRampa() {
    if(subirRampa || bajarRampa) {
        lcd.clear();

        cuadros[x_actual][y_actual][z_actual].setEstado(RECORRIDO);

        if(x_last != 255) {
            if(cuadros[x_last][y_last][z_actual].getEstado() != RECORRIDO)
                cuadros[x_last][y_last][z_actual].setEstado(SIN_RECORRER);

            Last = false;
        }

        if(x_last2 != 255) {
            if(cuadros[x_last2][y_last2][z_actual].getEstado() != RECORRIDO)
                cuadros[x_last2][y_last2][z_actual].setEstado(SIN_RECORRER);

            Last2 = false;
        }

        // Determina si empezo abajo o arriba
        if(firstFloor == 0) {
            if(subirRampa)
                firstFloor = ABAJO;

            if(bajarRampa)
                firstFloor = ARRIBA;
        }

        if(firstFloor == ABAJO) {
            if(subirRampa) {
              lcd.print("SUBIENDO");
              z_actual++;
              setNewPos();
              setRampa();
              permisoRampa = SUBIR;
            } else if(bajarRampa) {
              lcd.print("BAJANDO");
              z_actual--;
              setNewPos();
              permisoRampa = BAJAR;

            }
        } else if(firstFloor == ARRIBA) {
            if(subirRampa) {
              lcd.print("SUBIENDO");
              z_actual--;
              setNewPos();
              permisoRampa = SUBIR;
            } else if(bajarRampa) {
              lcd.print("BAJANDO");
              z_actual++;
              setNewPos();
              setRampa();
              permisoRampa = BAJAR;
            }
        }
    }
    subirRampa = false;
    bajarRampa = false;
}


void alinearIMU() {
    if (rampaCambio || cuadrosVisitados > 20 || vueltasDadas > 15) {
        int lecturaUltra, lecturaSharp;
        bool alfa       =   false;
        bool bravo      =   false;
        bool charlie    =   false;
        bool delta      =   false;
        velocidad(VEL_MOTOR_ALINEAR, VEL_MOTOR_ALINEAR_ENCODER, VEL_MOTOR_ALINEAR, VEL_MOTOR_ALINEAR);


        lecturaUltra = getUltrasonicoUno('A');
        lecturaSharp = getSharpCorta(SHARP_A);
        if(0 < lecturaUltra && lecturaUltra < 20 && 0 < lecturaSharp && lecturaSharp < 20)
            alfa = true;

        lecturaUltra = getUltrasonicoUno('B');
        lecturaSharp = getSharpCorta(SHARP_B1);
        if(0 < lecturaUltra && lecturaUltra < 20 && 0 < lecturaSharp && lecturaSharp < 20)
            bravo = true;

        lecturaUltra = getUltrasonicoUno('C');
        lecturaSharp = getSharpCorta(SHARP_C);
        if(0 < lecturaUltra && lecturaUltra < 20 && 0 < lecturaSharp && lecturaSharp < 20)
            charlie = true;

        lecturaUltra = getUltrasonicoUno('D');
        lecturaSharp = getSharpCorta(SHARP_D1);
        if(0 < lecturaUltra && lecturaUltra < 20 && 0 < lecturaSharp && lecturaSharp < 20)
            delta = true;

        if(rampaCambio) {
            bool temp;
            byte ultimaOrientacion = iOrientacion;
            switch (iOrientacion) {
                case B_norte:
                    vueltaDer();
                    temp = alfa;
                    alfa = bravo;
                    bravo = charlie;
                    charlie = delta;
                    delta = temp;

                    break;

                case C_norte:
                    vueltaDer();
                    vueltaDer();
                    temp = alfa;
                    alfa = charlie;
                    charlie = temp;
                    temp = bravo;
                    bravo = charlie;
                    charlie = temp;
                    break;

                case D_norte:
                    vueltaIzq();
                    temp = alfa;
                    alfa = delta;
                    delta = charlie;
                    charlie = bravo;
                    bravo = temp;
                    break;
            }
            steps = 0;
            velocidad(VEL_MOTOR_ALINEAR, VEL_MOTOR_ALINEAR, VEL_MOTOR_ALINEAR, VEL_MOTOR_ALINEAR);
            unsigned long inicio = millis();

            if (bravo) {
                lecturaSharp = getSharpCorta(SHARP_B1);
                while(steps <= lecturaSharp * 210) {
                    horizontalDerecha();
                    if (millis() >= inicio + 1500)
                        steps = 9999;
                }
            } else if (delta) {
                lecturaSharp = getSharpCorta(SHARP_D1);
                while(steps <= lecturaSharp * 210) {
                    horizontalIzquierda();
                    if (millis() >= inicio + 1500)
                        steps = 9999;
                }
            } else if (charlie) {
                lecturaSharp = getSharpCorta(SHARP_C);
                while(steps <= lecturaSharp * 210) {
                    reversa();
                    if (millis() >= inicio + 1500)
                        steps = 9999;
                }
            } else if (alfa) {
                lecturaSharp = getSharpCorta(SHARP_A);
                while(steps <= lecturaSharp * 210) {
                    avanzar();
                    if (millis() >= inicio + 1500)
                        steps = 9999;
                }
            }


            detener();
            lcd.clear();
            lcd.print("CALIBRANDO IMU");
            delay(1400);
            bno.begin();
            lcd.clear();
            lcd.print("CALIBRADO");
            delay(50);

            switch (ultimaOrientacion) {
                case B_norte:
                    vueltaIzq();
                    break;

                case C_norte:
                    vueltaDer();
                    vueltaDer();
                    break;

                case D_norte:
                    vueltaDer();
                    break;
            }
            vueltasDadas = 0;
            cuadrosVisitados = 0;
            rampaCambio = false;

        } else if ((cuadrosVisitados > 20 || vueltasDadas > 15) && iOrientacion == A_norte) {
            steps = 0;
            velocidad(VEL_MOTOR_ALINEAR, VEL_MOTOR_ALINEAR, VEL_MOTOR_ALINEAR, VEL_MOTOR_ALINEAR);
            unsigned long inicio = millis();

            if (alfa) {
                lecturaSharp = getSharpCorta(SHARP_A);
                while(steps <= lecturaSharp * 210) {
                    avanzar();
                    if (millis() >= inicio + 1500) {
                        detener();
                        return;
                    }
                }

            } else if (charlie) {
                lecturaSharp = getSharpCorta(SHARP_C);
                while(steps <= lecturaSharp * 210) {
                    reversa();
                    if (millis() >= inicio + 1500) {
                        detener();
                        return;
                    }
                }

            } else if (bravo) {
                lecturaSharp = getSharpCorta(SHARP_B1);
                while(steps <= lecturaSharp * 210) {
                    horizontalDerecha();
                    if (millis() >= inicio + 1500) {
                        detener();
                        return;
                    }
                }

            } else if (delta) {
                lecturaSharp = getSharpCorta(SHARP_D1);
                while(steps <= lecturaSharp * 210) {
                    horizontalIzquierda();
                    if (millis() >= inicio + 1500) {
                        detener();
                        return;
                    }
                }
            } else
                return;
            detener();


            lcd.clear();
            lcd.print("CALIBRANDO IMU");
            delay(1400);
            bno.begin();
            lcd.clear();
            lcd.print("CALIBRADO");
            delay(50);

            vueltasDadas = 0;
            cuadrosVisitados = 0;
        }
    }
}


void movimientoDerecho(int fuente) {
    float angle;
    imu::Vector<3> vecm = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    switch (fuente) {
        case MOV_FRENTE:
            avanzar();
            angle = getAngulo();
            if(angle > 320) {
                inIzq = - (360 - angle);
                inDer = - (360 - angle);
            } else {
                inIzq = angle;
                inDer = angle;
            }

            izqPID.Compute();
            derPID.Compute();
            velocidad(VEL_MOTOR + outIzq - outDer, VEL_MOTOR + outDer - outIzq, VEL_MOTOR + outIzq - outDer, VEL_MOTOR + outDer - outIzq);
            vecm = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
            if(vecm.y() < -4)
                bumper = true;

            if(setIzq - inIzq > 9) {
                detener();
                int pos = steps;
                steps = 0;
                while (steps <= 800)
                    movimientoDerecho(MOV_REVERSA);
                detener();
                steps = 0;
                while (steps <= 1200)
                    horizontalDerecha();
                detener();
                velocidad(VEL_MOTOR_ALINEAR, VEL_MOTOR_ALINEAR, VEL_MOTOR_ALINEAR, VEL_MOTOR_ALINEAR);
                do {
                    angle = getAngulo();
                    if(angle > 320)
                        inIzq = - (360 - angle);
                    else
                        inIzq = angle;
                    vueltaDerecha();
                } while(setIzq - inIzq  > 3);
                detener();
                steps = pos * 0.80;
            } else if(inIzq - setIzq > 9) {
                detener();
                delay(1000);
                int pos = steps;
                steps = 0;
                while (steps <= 800)
                    movimientoDerecho(MOV_REVERSA);
                detener();
                steps = 0;
                while (steps <= 1200)
                    horizontalIzquierda();
                detener();
                velocidad(VEL_MOTOR_ALINEAR, VEL_MOTOR_ALINEAR, VEL_MOTOR_ALINEAR, VEL_MOTOR_ALINEAR);
                do {
                    angle = getAngulo();
                    if(angle > 320)
                        inIzq = - (360 - angle);
                    else
                        inIzq = angle;

                    vueltaIzquierda();
                } while(inIzq - setIzq  > 3);
                detener();
                steps = pos * 0.80;
            }


            /*if(contador_ultra < 20 && millis() - millisPasado > 48 && millis() - millisPasado < 52) {
                millisPasado = millis();
                agregarLecturas('A');
            }*/
            break;

        case MOV_RAMPA_SUBIR:
            avanzar();
            angle = getAngulo();
            if(angle > 320) {
                inIzq = - (360 - angle);
                inDer = - (360 - angle);
            } else {
                inIzq = angle;
                inDer = angle;
            }

            izqPID.Compute();
            derPID.Compute();
            velocidad(VEL_MOTOR_RAMPA + outIzq, VEL_MOTOR_RAMPA_ENCODER + outDer, VEL_MOTOR_RAMPA + outIzq, VEL_MOTOR_RAMPA_ENCODER + outDer);
            break;

        case MOV_RAMPA_BAJAR:
            avanzar();
            angle = getAngulo();
            if(angle > 320) {
                inIzq = - (360 - angle);
                inDer = - (360 - angle);
            } else {
                inIzq = angle;
                inDer = angle;
            }

            izqPID.Compute();
            derPID.Compute();
            velocidad(100 + outIzq, 100 + outDer, 100 + outIzq, 100 + outDer);
            break;

        case MOV_RAMPA_NO_SUBIR:
            reversa();
            velocidad(100 + outIzq, 100 + outDer, 100 + outIzq, 100 + outDer);
            break;

        case MOV_RAMPA_NO_BAJAR:
            reversa();
            angle = getAngulo();
            if(angle > 320) {
                inIzq = - (360 - angle);
                inDer = - (360 - angle);
            } else {
                inIzq = angle;
                inDer = angle;
            }
            izqPID.Compute();
            derPID.Compute();

            velocidad(VEL_MOTOR_RAMPA + outIzq, VEL_MOTOR_RAMPA_ENCODER + outDer, VEL_MOTOR_RAMPA + outIzq, VEL_MOTOR_RAMPA + outDer);
            break;
        case MOV_REVERSA:
            reversa();
            velocidad(VEL_MOTOR, VEL_MOTOR, VEL_MOTOR, VEL_MOTOR);
            break;
        case MOV_FRENTE_ALINEAR:
            avanzar();
            angle = getAngulo();
            if(angle > 320) {
                inIzq = - (360 - angle);
                inDer = - (360 - angle);
            } else {
                inIzq = angle;
                inDer = angle;
            }

            izqPID.Compute();
            derPID.Compute();
            velocidad(VEL_MOTOR_ALINEAR + outIzq - outDer, VEL_MOTOR_ALINEAR + outDer - outIzq, VEL_MOTOR_ALINEAR + outIzq - outDer, VEL_MOTOR_ALINEAR + outDer - outIzq);
            break;
        case MOV_REVERSA_ALINEAR:
            reversa();
            velocidad(VEL_MOTOR_ALINEAR, VEL_MOTOR_ALINEAR, VEL_MOTOR_ALINEAR, VEL_MOTOR_ALINEAR);
            break;
    }
}

void moverCuadro() {
    //delay(100);
    //primeraLectura();
    cuadrosVisitados++;
    steps = 0;
    while (steps <= 3700) {
        movimientoDerecho(MOV_FRENTE);
        checarInterr();
        checarLimit();
    }

    imu::Vector<3> vec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    if(vec.y() < -9) {
        lcd.home();
        lcd.print("SUBIR RAMPA");
        subirRampa = true;
        rampaCambio = true;
        checarRampa();

        switch (permisoRampa) {
            case REGRESA_ARRIBA:
            case BAJAR_SUBIR:
            case BAJAR:
                lcd.clear();
                lcd.print(permisoRampa);
                lcd.print("ERROR");
                delay(50000);
                break;

            case SUBIR:
                while (vec.y() < -4) {
                    movimientoDerecho(MOV_RAMPA_SUBIR);
                    vec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
                }
                steps = 0;

                while (steps <= 1000) {
                    movimientoDerecho(MOV_FRENTE);
                }
                steps = 0;
                break;

            /*case SUBIR_BAJAR:
                while (vec.y() < -10.0) {
                    movimientoDerecho(MOV_RAMPA_SUBIR);
                    vec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
                }

                velocidad(VEL_MOTOR, VEL_MOTOR, VEL_MOTOR, VEL_MOTOR);
                steps = 0;
                while (steps <= 2000) {
                    movimientoDerecho(MOV_FRENTE);
                }

                detener();
                delay(200);
                alinearIMU();
                vueltaDer();
                delay(200);
                vueltaDer();
                delay(200);

                velocidad(VEL_MOTOR, VEL_MOTOR, VEL_MOTOR, VEL_MOTOR);
                steps = 0;
                while (steps <= 2000) {
                    movimientoDerecho(MOV_FRENTE);
                }

                vec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
                while (vec.y() > 10.0) {
                    movimientoDerecho(MOV_RAMPA_BAJAR);
                    vec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
                }
                detener();
                alinearIMU();
                break;

            case REGRESA_ABAJO:
                detener();
                while (vec.y() < -10.0) {
                    movimientoDerecho(MOV_RAMPA_NO_SUBIR);
                    vec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
                }
                detener();
                delay(200);
                vueltaDer();
                delay(200);
                vueltaDer();
                break;*/
        }
    } else if(vec.y() > 9) {
        lcd.home();
        lcd.print("BAJAR RAMPA");
        bajarRampa = true;
        rampaCambio = true;
        checarRampa();

        switch (permisoRampa) {
            case REGRESA_ABAJO:
            case SUBIR_BAJAR:
            case SUBIR:
                lcd.clear();
                lcd.home();
                lcd.print("ERROR");
                break;

            case BAJAR:
                while (vec.y() > 4) {
                    movimientoDerecho(MOV_RAMPA_BAJAR);
                    vec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
                }
                steps = 0;

                while (steps <= 1000) {
                    movimientoDerecho(MOV_FRENTE);
                }
                steps = 0;
                break;

            /*case BAJAR_SUBIR:
                while (vec.y() > 10.0) {
                    movimientoDerecho(MOV_RAMPA_BAJAR);
                    vec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
                }

                velocidad(VEL_MOTOR, VEL_MOTOR, VEL_MOTOR, VEL_MOTOR);
                steps = 0;
                while (steps <= 3000) {
                    movimientoDerecho(MOV_FRENTE);
                }

                detener();
                alinearIMU();
                delay(200);
                vueltaDer();
                delay(200);
                vueltaDer();
                delay(200);

                velocidad(VEL_MOTOR, VEL_MOTOR, VEL_MOTOR, VEL_MOTOR);
                steps = 0;
                while (steps <= 3000) {
                    movimientoDerecho(MOV_FRENTE);
                }

                vec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
                while (vec.y() < -10.0) {
                    movimientoDerecho(MOV_RAMPA_SUBIR);
                    vec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
                }
                detener();
                alinearIMU();
                break;

            case REGRESA_ARRIBA:
                detener();
                while (vec.y() > 10.0) {
                    movimientoDerecho(MOV_RAMPA_NO_BAJAR);
                    vec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
                }

                detener();
                delay(200);
                vueltaDer();
                delay(200);
                vueltaDer();
                break;*/
        }
    } else {
        steps = 0;
        velocidad(VEL_MOTOR, VEL_MOTOR, VEL_MOTOR, VEL_MOTOR);
        while (steps <= 950) {
            movimientoDerecho(MOV_FRENTE);
            checarInterr();
            checarLimit();
        }
    }

    velocidad(VEL_MOTOR, VEL_MOTOR, VEL_MOTOR, VEL_MOTOR);
    steps = 0;
    while (steps <= 750) {
        movimientoDerecho(MOV_FRENTE);
        checarInterr();
        checarLimit();
    }
    detener();
    checarColor();
    alinearIMU();
    alinear();
}


void reversaCuadro() {
    steps = 0;
    while (steps <= 4800)
        movimientoDerecho(MOV_REVERSA);
    detener();
}


void absoluteMove(char cLado) {
    switch (iOrientacion) {
        case A_norte:
        switch(cLado) {
            case 'N':
                lastMove = TO_NORTH;
                y_actual++;
                moverCuadro();
                break;

            case 'E':
                lastMove = TO_EAST;
                x_actual++;
                vueltaDer();
                alinear();
                moverCuadro();
                break;

            case 'S':
                lastMove = TO_SOUTH;
                y_actual--;
                vueltaDer();
                checarInterr();
                checarLimit();
                vueltaDer();
                checarInterr();
                checarLimit();
                //vueltaAtras();
                alinear();
                moverCuadro();
                break;

            case 'O':
                x_actual--;
                lastMove = TO_WEST;
                vueltaIzq();
                alinear();
                moverCuadro();
                break;
        }
        break;

        case B_norte:
        switch(cLado) {
            case 'N':
                lastMove = TO_NORTH;
                y_actual++;
                vueltaDer();
                alinear();
                moverCuadro();
                break;

            case 'E':
                lastMove = TO_EAST;
                x_actual++;
                vueltaDer();
                checarInterr();
                checarLimit();
                vueltaDer();
                checarInterr();
                checarLimit();
                //vueltaAtras();
                alinear();
                moverCuadro();
                break;

            case 'S':
                lastMove = TO_SOUTH;
                y_actual--;
                vueltaIzq();
                alinear();
                moverCuadro();
                break;

            case 'O':
                lastMove = TO_WEST;
                x_actual--;
                moverCuadro();
                break;
        }
        break;

        case C_norte:
        switch(cLado) {
            case 'N':
                lastMove = TO_NORTH;
                y_actual++;
                vueltaDer();
                checarInterr();
                checarLimit();
                vueltaDer();
                checarInterr();
                checarLimit();
                //vueltaAtras();
                alinear();
                moverCuadro();
                break;

            case 'E':
                lastMove = TO_EAST;
                x_actual++;
                vueltaIzq();
                alinear();
                moverCuadro();
                break;

            case 'S':
                lastMove = TO_SOUTH;
                y_actual--;
                moverCuadro();
                break;

            case 'O':
                lastMove = TO_WEST;
                x_actual--;
                vueltaDer();
                alinear();
                moverCuadro();
                break;
        }
        break;

        case D_norte:
        switch(cLado) {
            case 'N':
                lastMove = TO_NORTH;
                y_actual++;
                vueltaIzq();
                alinear();
                moverCuadro();
                break;

            case 'E':
                lastMove = TO_EAST;
                x_actual++;
                moverCuadro();
                break;

            case 'S':
                lastMove = TO_SOUTH;
                y_actual--;
                vueltaDer();
                alinear();
                moverCuadro();
                break;

            case 'O':
                lastMove = TO_WEST;
                x_actual--;
                vueltaDer();
                checarInterr();
                checarLimit();
                vueltaDer();
                checarInterr();
                checarLimit();
                //vueltaAtras();
                alinear();
                moverCuadro();
                break;
        }
        break;
    }
}


void agregarLast(char cSentido){
    if(Last2)
        if(cuadros[x_last2][y_last2][z_actual].getEstado() == NO_EXISTE)
            cuadros[x_last2][y_last2][z_actual].setEstado(SIN_RECORRER);

    if(Last) {
        x_last2 = x_last;
        y_last2 = y_last;
        Last2 = true;
    }

    switch(cSentido) {
        case 'N':
            x_last = x_actual;
            y_last = y_actual+1;
            break;

        case 'E':
            x_last = x_actual+1;
            y_last = y_actual;
            break;

        case 'S':
            x_last = x_actual;
            y_last = y_actual-1;
            break;

        case 'O':
            x_last = x_actual-1;
            y_last = y_actual;
            break;
    }
    Last = true;
}

//Formula para identificar un cuadro (x,y) en una de las listas
int coordToGrid(byte x, byte y) {
    return x + (y * X_MAX);
}

//Formula para identificar un 'num' en un cuadro (x,y)
byte gridToCoord(byte grid, char eje) {
    if(eje == 'x')
        return grid % X_MAX;
    else if(eje == 'y')
        return grid / X_MAX;
}

byte totalGridToCoord(int grid, char eje) {
    byte z = grid/ (X_MAX*Y_MAX);
    byte y = ( grid - ((X_MAX*Y_MAX)*z) ) / X_MAX;
    byte x = ( grid - ((X_MAX*Y_MAX)*z) ) % X_MAX;

    switch (eje) {
      case 'x':
          return x;

      case 'y':
        return y;

      case 'z':
        return z;
    }
}

byte pathway(byte x_inicial, byte y_inicial, byte x_final, byte y_final) {
    return fabs(x_final - x_inicial) + fabs(y_final - y_inicial);
}

//******************************************
//------------- PATHFINDING ----------------
void Pathfinding(byte x_destino, byte y_destino, byte &ref) {
    //lcd.println("Estoy en PathFinding");
    //lcd.println("Actual = " + String(x_actual)+ "," + String(y_actual));
    //lcd.println("Destino = " + String(x_destino)+ "," + String(y_destino));
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.print("    " + String(x_destino) + "," + String(y_destino) + "," + String(z_actual));
    bool pathFinished = false;
    int Grid;
    byte x_path = x_actual;
    byte y_path = y_actual;

    byte neighbor_index;
    int neighborsortValue;
    int LastPathValue;
    int openSortValue;
    int open_index;
    int newGrid;
    bool firstLoop = false;

    byte x_lastPath, y_lastPath;
    int back_index = GRID_MAX-1;
    byte x_back;
    byte y_back;
    bool backFinished = false;
    int TempGrid;
    int lastBackGrid;

    int gridActual = coordToGrid(x_actual, y_actual);

    for (int i = 0; i<GRID_MAX; i++)
        openList[i] = 999;

    for (int i = 0; i<GRID_MAX; i++)
        closedList[i] = 999;

    for (int i = 0; i<GRID_MAX; i++)
        backList[i] = 999;


    while (!pathFinished) {
        //lcd.println("Entre al while");
        neighborsortValue = 999;
        openSortValue = 999;
        LastPathValue = pathway(x_path, y_path, x_destino, y_destino);
        //lcd.println("LastPathValue =" + String(LastPathValue));

        if(x_destino == x_actual && y_destino == y_actual) {
            pathFinished = true;
        }

        for (int i = 0; i<4; i++)
            neighbors[i] = 999;

        if(!firstLoop) {
            closedList[coordToGrid(x_path, y_path)] = LastPathValue;
            firstLoop = true;
        }


        if (!cuadros[x_path][y_path][z_actual].getPared('S')) {
            if(y_path > 0) {
                if (cuadros[x_path][y_path-1][z_actual].getEstado() == INICIO || cuadros[x_path][y_path-1][z_actual].getEstado() == CHECKPOINT ||
                cuadros[x_path][y_path-1][z_actual].getEstado() == RECORRIDO || (x_path == x_destino && y_path-1 == y_destino)) {
                    Grid = coordToGrid(x_path, y_path-1);
                    if(closedList[Grid] == 999) {
                        openList[Grid] = pathway(x_path, y_path-1, x_destino, y_destino);
                        neighbors[0] = pathway(x_path, y_path-1, x_destino, y_destino);
                    }
                }
            }
        }

        if (!cuadros[x_path][y_path][z_actual].getPared('E')) {
            if(x_path < X_MAX -1) {
                if (cuadros[x_path+1][y_path][z_actual].getEstado() == INICIO || cuadros[x_path+1][y_path][z_actual].getEstado() == CHECKPOINT ||
                cuadros[x_path+1][y_path][z_actual].getEstado() == RECORRIDO || (x_path+1 == x_destino && y_path == y_destino)) {
                    Grid = coordToGrid(x_path+1, y_path);
                    if(closedList[Grid] == 999) {
                        openList[Grid] = pathway(x_path+1, y_path, x_destino, y_destino);
                        neighbors[1] = pathway(x_path+1, y_path, x_destino, y_destino);
                    }
                }
            }
        }

        if (!cuadros[x_path][y_path][z_actual].getPared('N')) {
            if(y_path < Y_MAX -1) {
                if (cuadros[x_path][y_path+1][z_actual].getEstado() == INICIO || cuadros[x_path][y_path+1][z_actual].getEstado() == CHECKPOINT ||
                cuadros[x_path][y_path+1][z_actual].getEstado() == RECORRIDO || (x_path == x_destino && y_path+1 == y_destino)) {
                    Grid = coordToGrid(x_path, y_path+1);
                    if(closedList[Grid] == 999) {
                        openList[Grid] = pathway(x_path, y_path+1, x_destino, y_destino);
                        neighbors[2] = pathway(x_path, y_path+1, x_destino, y_destino);
                    }
                }
            }
        }

        if (!cuadros[x_path][y_path][z_actual].getPared('O')) {
            if(x_path > 0) {
                if (cuadros[x_path-1][y_path][z_actual].getEstado() == INICIO || cuadros[x_path-1][y_path][z_actual].getEstado() == CHECKPOINT ||
                cuadros[x_path-1][y_path][z_actual].getEstado() == RECORRIDO || (x_path-1 == x_destino && y_path == y_destino)) {
                    Grid = coordToGrid(x_path-1, y_path);
                    if(closedList[Grid] == 999) {
                        openList[Grid] = pathway(x_path-1, y_path, x_destino, y_destino);
                        neighbors[3] = pathway(x_path-1, y_path, x_destino, y_destino);
                    }
                }
            }
        }

        for (int i = 0; i<4; i++) {
            if(neighbors[i] < neighborsortValue) {
                neighbor_index = i;
                neighborsortValue = neighbors[i];
            }
        }

        if (neighborsortValue < LastPathValue) {
            LastPathValue = neighborsortValue;
            switch(neighbor_index) {
                case 0:
                    y_path--;
                    break;

                case 1:
                    x_path++;
                    break;

                case 2:
                    y_path++;
                    break;

                case 3:
                    x_path--;
                    break;
            }
        } else {
            for(int i = 0; i<GRID_MAX; i++) {
                if(openList[i] < openSortValue) {
                    open_index = i;
                    openSortValue = openList[i];
                }
            }

            if(openSortValue < neighborsortValue) {
                x_path = gridToCoord(open_index, 'x');
                y_path = gridToCoord(open_index, 'y');
            } else {
                switch(neighbor_index) {
                    case 0:
                        y_path--;
                        break;

                    case 1:
                        x_path++;
                        break;

                    case 2:
                        y_path++;
                        break;

                    case 3:
                        x_path--;
                        break;
                }
            }
        }
        //lcd.println("Paths: " + String(x_path) + "," + String(y_path) + " = " + String(pathway(x_path, y_path, x_destino, y_destino)));
        //delay(1000);
        //lcd.println("LastPath = " + String(x_lastPath) + "," + String(y_lastPath));
        //delay(1000);
        if(x_path == x_destino && y_path == y_destino) {
                    /*for(int i = 0; i<GRID_MAX; i++)
                    {
                    //lcd.println("Open[" + String(i) + "] = " + String (openList[i]));
                }
                for(int i = 0; i<GRID_MAX; i++)
                {
                //lcd.println("Closed[" + String(i) + "] = " + String (closedList[i]));
            }*/

            //Encontrar camino de regreso
            x_back = x_path;
            y_back = y_path;
            //lcd.println(x_back);
            //lcd.println(y_back);

            while(!backFinished) {
                for (int i = 0; i<4; i++)
                neighbors[i] = 999;
                neighborsortValue = 999;

                if(!cuadros[x_back][y_back][z_actual].getPared('S')) {
                    if(y_back > 0) {
                        TempGrid = coordToGrid(x_back, y_back-1);
                        if(closedList[TempGrid] != 999 && backList[lastBackGrid+1] != TempGrid)
                        neighbors[0] = pathway(x_back, y_back-1, x_actual, y_actual);
                    }
                }

                if(!cuadros[x_back][y_back][z_actual].getPared('E')) {
                    if(x_back < X_MAX -1) {
                        TempGrid = coordToGrid(x_back+1, y_back);
                        if(closedList[TempGrid] != 999 && backList[lastBackGrid+1] != TempGrid)
                        neighbors[1] = pathway(x_back+1, y_back, x_actual, y_actual);
                    }
                }

                if(!cuadros[x_back][y_back][z_actual].getPared('N')) {
                    if(y_back < Y_MAX -1) {
                        TempGrid = coordToGrid(x_back, y_back+1);
                        if(closedList[TempGrid] != 999 && backList[lastBackGrid+1] != TempGrid)
                        neighbors[2] = pathway(x_back, y_back+1, x_actual, y_actual);
                    }
                }

                if(!cuadros[x_back][y_back][z_actual].getPared('O')) {
                    if(x_back > 0) {
                        TempGrid = coordToGrid(x_back-1, y_back);
                        if(closedList[TempGrid] != 999 && backList[lastBackGrid+1] != TempGrid)
                        neighbors[3] = pathway(x_back-1, y_back, x_actual, y_actual);
                    }
                }

                for (int i = 0; i<4; i++) {
                    if(neighbors[i] < neighborsortValue) {
                        neighbor_index = i;
                        neighborsortValue = neighbors[i];
                    }
                }

                switch(neighbor_index) {
                    case 0:
                        y_back--;
                        break;

                    case 1:
                        x_back++;
                        break;

                    case 2:
                        y_back++;
                        break;

                    case 3:
                        x_back--;
                        break;
                }

                if(firstLoop) {
                    backList[back_index] = coordToGrid(x_destino, y_destino);
                    back_index--;
                    x_back = x_lastPath;
                    y_back = y_lastPath;
                    firstLoop = false;
                }

                newGrid = coordToGrid(x_back, y_back);
                backList[back_index] = newGrid;
                lastBackGrid = back_index;

                ////lcd.println("BackList = " + String(backList[back_index]));

                /*//lcd.print(x_back);
                //lcd.print(" ");
                //lcd.println(y_back);
                delay(3000);*/
                back_index--;
                //------------
                if(x_back == x_actual && y_back == y_actual) {
                    backList[back_index+1] = 999;
                    //lcd.println("Entre al final");
                    //delay(500);
                    //Dar ||denes de movimiento para llegar
                    backFinished = true;

                    if(ref == 255) {
                        //x_actual = x_destino;
                        //y_actual = y_destino;
                        for(int i = 0; i<GRID_MAX; i++) {
                            if(backList[i] != 999) {
                                if(gridActual-backList[i] == X_MAX) {
                                    //lcd.println("Abajo");
                                    absoluteMove('S');
                                    gridActual -= X_MAX;
                                }

                                if(gridActual-backList[i] == -1) {
                                    //lcd.println("Derecha");
                                    absoluteMove('E');
                                    gridActual += 1;
                                }

                                if(gridActual-backList[i] == -X_MAX) {
                                    //lcd.println("Arriba");
                                    absoluteMove('N');
                                    gridActual += X_MAX;
                                }

                                if(gridActual-backList[i] == 1) {
                                    //lcd.println("Izquierda");
                                    absoluteMove('O');
                                    gridActual -= 1;
                                }
                            }
                        }
                    } else {
                        for(int i = 0; i<GRID_MAX; i++) {
                            if(backList[i] != 999)
                            ref++;
                        }
                    }
                }
            }
            pathFinished = true;
        }
    x_lastPath = x_path;
    y_lastPath = y_path;
    newGrid = coordToGrid(x_path, y_path);
    closedList[newGrid] = openList[newGrid];
    openList[newGrid] = 999;
    }
}


//----------- FUNCIONES CUADRO -------------


void verificarCSR() {
    ArrayCSR = 0;
    for(int x=0; x<X_MAX; x++) {
        for(int y=0; y<Y_MAX; y++) {
            if(cuadros[x][y][z_actual].getEstado() == SIN_RECORRER) {
                x_recorrer[ArrayCSR] = x;
                y_recorrer[ArrayCSR] = y;
                ArrayCSR++;
            }
        }
    }
}


void verificarRampa() {
    for (int y=0; y<Y_MAX; y++) {
        for(int x=0; x<X_MAX; x++) {
            if(cuadros[x][y][z_actual].getEstado() == RAMPA) {
                x_rampa = x;
                y_rampa = y;
                rampaid = true;
            }
        }
    }
}

// Detecta paredes y las actualiza en el array cuadros[] en la posicion actual
void checarParedes() {
    shortMove = false;
    A_wall = false; B_wall = false; C_wall = false; D_wall = false;
    int lectura;

    switch(iOrientacion) {
        case A_norte:
        if(y_actual > 0) {
            lectura = getUltrasonicoUno('C');
            if((lectura == 0 || lectura > 15 ) && (cuadros[x_actual][y_actual-1][z_actual].getEstado()==NO_EXISTE  ||
            cuadros[x_actual][y_actual-1][z_actual].getEstado()==SIN_RECORRER))
            {
                agregarLast('S');
                shortMove = true;
            }
            else
            {
                C_wall = true;
            }
        }
        if(lectura != 0 && lectura < 15)
            cuadros[x_actual][y_actual][z_actual].setPared('S', true);

        lectura = getUltrasonicoUno('B');
        if((lectura == 0 || lectura > 15) && (cuadros[x_actual+1][y_actual][z_actual].getEstado()==NO_EXISTE  ||
        cuadros[x_actual+1][y_actual][z_actual].getEstado()==SIN_RECORRER)) {
            agregarLast('E');
            shortMove = true;
        } else {
            B_wall = true;
        }
        if(lectura != 0 && lectura < 15)
            cuadros[x_actual][y_actual][z_actual].setPared('E', true);

        lectura = getUltrasonicoUno('A');
        if((lectura == 0 || lectura > 15) && (cuadros[x_actual][y_actual+1][z_actual].getEstado()==NO_EXISTE  ||
        cuadros[x_actual][y_actual+1][z_actual].getEstado()==SIN_RECORRER)) {
            agregarLast('N');
            shortMove = true;
        } else {
            A_wall = true;
        }
        if(lectura != 0 && lectura < 15)
            cuadros[x_actual][y_actual][z_actual].setPared('N', true);

        if(x_actual > 0)
        {
            lectura = getUltrasonicoUno('D');
            if((lectura == 0 || lectura > 15) && (cuadros[x_actual-1][y_actual][z_actual].getEstado()==NO_EXISTE  ||
            cuadros[x_actual-1][y_actual][z_actual].getEstado()==SIN_RECORRER))
            {
                agregarLast('O');
                shortMove = true;
            }
            else
            {
                D_wall = true;
            }
        }

        if(lectura != 0 && lectura < 15)
            cuadros[x_actual][y_actual][z_actual].setPared('O', true);

        break;
        //--------------------------------------------------------------------
        case B_norte:
        lectura = getUltrasonicoUno('C');
        if((lectura == 0 || lectura > 15) && (cuadros[x_actual+1][y_actual][z_actual].getEstado()==NO_EXISTE  ||
        cuadros[x_actual+1][y_actual][z_actual].getEstado()==SIN_RECORRER)) {
            agregarLast('E');
            shortMove = true;
        } else {
            C_wall = true;
        }
        if((lectura != 0 && lectura < 15))
        cuadros[x_actual][y_actual][z_actual].setPared('E', true);

        lectura = getUltrasonicoUno('B');
        if((lectura == 0 || lectura > 15) && (cuadros[x_actual][y_actual+1][z_actual].getEstado()==NO_EXISTE  ||
        cuadros[x_actual][y_actual+1][z_actual].getEstado()==SIN_RECORRER)) {
            agregarLast('N');
            shortMove = true;
        } else {
            B_wall = true;
        }
        if(lectura != 0 && lectura < 15)
        cuadros[x_actual][y_actual][z_actual].setPared('N', true);


        if(x_actual > 0)
        {
            lectura = getUltrasonicoUno('A');
            if((lectura == 0 || lectura > 15) && (cuadros[x_actual-1][y_actual][z_actual].getEstado()==NO_EXISTE  ||
            cuadros[x_actual-1][y_actual][z_actual].getEstado()==SIN_RECORRER))
            {
                agregarLast('O');
                shortMove = true;
            }
            else
            {
                A_wall = true;
            }
        }
        if(lectura != 0 && lectura < 15)
            cuadros[x_actual][y_actual][z_actual].setPared('O', true);

        if(y_actual > 0)
        {
            lectura = getUltrasonicoUno('D');
            if((lectura == 0 || lectura > 15) && (cuadros[x_actual][y_actual-1][z_actual].getEstado()==NO_EXISTE  ||
            cuadros[x_actual][y_actual-1][z_actual].getEstado()==SIN_RECORRER))
            {
            agregarLast('S');
            shortMove = true;
            }
            else
            {
                D_wall = true;
            }
        }

        if(lectura != 0 && lectura < 15)
            cuadros[x_actual][y_actual][z_actual].setPared('S', true);

        break;
        //--------------------------------------------------------------------
        case C_norte:
        lectura = getUltrasonicoUno('C');
        if((lectura == 0 || lectura > 15)&& (cuadros[x_actual][y_actual+1][z_actual].getEstado()==NO_EXISTE  ||
        cuadros[x_actual][y_actual+1][z_actual].getEstado()==SIN_RECORRER)) {
            agregarLast('N');
            shortMove = true;
        } else {
            C_wall = true;
        }
        if((lectura != 0 && lectura < 15))
            cuadros[x_actual][y_actual][z_actual].setPared('N', true);

        if(x_actual > 0)
        {
            lectura = getUltrasonicoUno('B');
            if((lectura == 0 || lectura > 15) && (cuadros[x_actual-1][y_actual][z_actual].getEstado()==NO_EXISTE  ||
            cuadros[x_actual-1][y_actual][z_actual].getEstado()==SIN_RECORRER))
            {
                agregarLast('O');
                shortMove = true;
            }
            else
            {
                B_wall = true;
            }
        }
        if(lectura != 0 && lectura < 15)
        cuadros[x_actual][y_actual][z_actual].setPared('O', true);

        if(y_actual > 0)
        {
            lectura = getUltrasonicoUno('A');
            if((lectura == 0 || lectura > 15) && (cuadros[x_actual][y_actual-1][z_actual].getEstado()==NO_EXISTE  ||
            cuadros[x_actual][y_actual-1][z_actual].getEstado()==SIN_RECORRER))
            {
                agregarLast('S');
                shortMove = true;
            }
            else
            {
                A_wall = true;
            }
        }
        if(lectura != 0 && lectura < 15)
            cuadros[x_actual][y_actual][z_actual].setPared('S', true);

        lectura = getUltrasonicoUno('D');
        if((lectura == 0 || lectura > 15) && (cuadros[x_actual+1][y_actual][z_actual].getEstado()==NO_EXISTE  ||
        cuadros[x_actual+1][y_actual][z_actual].getEstado()==SIN_RECORRER)) {
            agregarLast('E');
            shortMove = true;
        } else {
            D_wall = true;
        }
        if(lectura != 0 && lectura < 15)
            cuadros[x_actual][y_actual][z_actual].setPared('E', true);

        break;
        //--------------------------------------------------------------------
        case D_norte:
        if(x_actual > 0)
        {
            lectura = getUltrasonicoUno('C');
            if((lectura == 0 || lectura > 15) > 15 && (cuadros[x_actual-1][y_actual][z_actual].getEstado()==NO_EXISTE  ||
            cuadros[x_actual-1][y_actual][z_actual].getEstado()==SIN_RECORRER))
            {
                agregarLast('O');
                shortMove = true;
            }
            else
            {
                C_wall = true;
            }
        }

        if(lectura != 0 && lectura < 15)
            cuadros[x_actual][y_actual][z_actual].setPared('O', true);


        if(y_actual > 0)
        {
            lectura = getUltrasonicoUno('B');
            if((lectura == 0 || lectura > 15) && (cuadros[x_actual][y_actual-1][z_actual].getEstado()==NO_EXISTE  ||
            cuadros[x_actual][y_actual-1][z_actual].getEstado()==SIN_RECORRER))
            {
                agregarLast('S');
                shortMove = true;
            }
            else
            {
                B_wall = true;
            }
        }


        if(lectura != 0 && lectura < 15)
            cuadros[x_actual][y_actual][z_actual].setPared('S', true);

        lectura = getUltrasonicoUno('A');
        if((lectura == 0 || lectura > 15) && (cuadros[x_actual+1][y_actual][z_actual].getEstado()==NO_EXISTE  ||
        cuadros[x_actual+1][y_actual][z_actual].getEstado()==SIN_RECORRER)) {
            agregarLast('E');
            shortMove = true;
        } else {
            A_wall = true;
        }

        if(lectura != 0 && lectura < 15)
            cuadros[x_actual][y_actual][z_actual].setPared('E', true);

        lectura = getUltrasonicoUno('D');
        if((lectura == 0 || lectura > 15) && (cuadros[x_actual][y_actual+1][z_actual].getEstado()==NO_EXISTE  ||
        cuadros[x_actual][y_actual+1][z_actual].getEstado()==SIN_RECORRER)) {
            agregarLast('N');
            shortMove = true;
        } else {
            D_wall = true;
        }
        if(lectura != 0 && lectura < 15)
            cuadros[x_actual][y_actual][z_actual].setPared('N', true);
        break;
    }
}


void checarLasts() {
    if(x_last2 != 255 && y_last2 != 255) {
        x_last = x_last2;
        y_last = y_last2;
        x_last2 = 255;
        y_last2 = 255;
        Last2 = false;
    } else {
        x_last = 255;
        y_last = 255;
        Last = false;
    }

    if(x_actual == x_last && y_actual == y_last) {
        x_last = 255;
        y_last = 255;
        Last = false;
    }
}


//Verificar si en el for i es i>1 o i>0 (probar)
void recorrerX() {
    //lcd.println("Recorrer X");
    for(int k=0; k<Z_MAX; k++){
        for(int j=0; j<Y_MAX; j++) {
            for(int i=X_MAX-1; i>0; i--) {
                cuadros[i][j][k].setEstado(cuadros[i-1][j][k].getEstado());
                cuadros[i][j][k].setPared('N', cuadros[i-1][j][k].getPared('N'));
                cuadros[i][j][k].setPared('E', cuadros[i-1][j][k].getPared('E'));
                cuadros[i][j][k].setPared('S', cuadros[i-1][j][k].getPared('S'));
                cuadros[i][j][k].setPared('O', cuadros[i-1][j][k].getPared('O'));
                cuadros[i][j][k].setmlx(cuadros[i-1][j][k].getmlx());
            }
            cuadros[0][j][k].setEstado(NO_EXISTE);
            cuadros[0][j][k].setPared('N', false);
            cuadros[0][j][k].setPared('E', false);
            cuadros[0][j][k].setPared('S', false);
            cuadros[0][j][k].setPared('O', false);
            cuadros[0][j][k].setmlx(false);
        }
    }

    x_actual++;
    x_inicio++;
    if(x_InicioB != 255)
        x_InicioB++;
    if(x_InicioC != 255)
        x_InicioC++;

    if(x_last != 255 && y_last != 255)
        x_last++;

    if(x_last2 != 255 && y_last2 != 255)
        x_last2++;
    //boolRecorrerX = true;
}

void recorrerY() {
    //lcd.println("Recorrer Y");
    for(int k=0; k<Z_MAX; k++){
        for(int j=0; j<X_MAX; j++) {
            for(int i=Y_MAX-1; i>0; i--){
                cuadros[j][i][k].setEstado(cuadros[j][i-1][k].getEstado());
                cuadros[j][i][k].setPared('N', cuadros[j][i-1][k].getPared('N'));
                cuadros[j][i][k].setPared('E', cuadros[j][i-1][k].getPared('E'));
                cuadros[j][i][k].setPared('S', cuadros[j][i-1][k].getPared('S'));
                cuadros[j][i][k].setPared('O', cuadros[j][i-1][k].getPared('O'));
                cuadros[j][i][k].setmlx(cuadros[i-1][j][k].getmlx());
            }
            cuadros[j][0][k].setEstado(NO_EXISTE);
            cuadros[j][0][k].setPared('N', false);
            cuadros[j][0][k].setPared('E', false);
            cuadros[j][0][k].setPared('S', false);
            cuadros[j][0][k].setPared('O', false);
            cuadros[j][0][k].setmlx(false);
        }
    }

    y_actual++;
    y_inicio++;
    if(y_InicioB != 255)
        y_InicioB++;
    if(y_InicioC != 255)
        y_InicioC++;

    if(x_last != 255 && y_last != 255)
        y_last++;

    if(x_last2 != 255 && y_last2 != 255)
        y_last2++;
    //boolRecorrerY = true;
}

void gotoInicio(byte x_final, byte y_final) {
    byte nearInicio = pathway(x_actual, y_actual, x_final, y_final);
    bool boolInicio = false;

    if(nearInicio == 1) {
        //lcd.println("Entre a nearInicio");
        //lcd.println("Inicio " + String(x_final) + "," + String(y_final));
        //lcd.println("Actual " + String(x_actual) + "," + String(y_actual));
        int xInicio = x_final - x_actual;
        int yInicio = y_final - y_actual;

        if(xInicio == 1) {
            //lcd.println("Inicio E");
            if(!cuadros[x_actual][y_actual][z_actual].getPared('E')) {
                absoluteMove('E');
                boolInicio = true;
            }
        }
        else if(xInicio == -1) {
            //lcd.println("Inicio O");
            if(!cuadros[x_actual][y_actual][z_actual].getPared('O')) {
                absoluteMove('O');
                boolInicio = true;
            }
        }
        else if(yInicio == 1) {
            //lcd.println("Inicio N");
            if(!cuadros[x_actual][y_actual][z_actual].getPared('N')) {
                absoluteMove('N');
                boolInicio = true;
            }
        }
        else if(yInicio == -1) {
            //lcd.println("Inicio S");
            if(!cuadros[x_actual][y_actual][z_actual].getPared('S')) {
                absoluteMove('S');
                boolInicio = true;
            }
        }
    }

    if(!boolInicio) {
        byte var = 255;
        Pathfinding(x_final, y_final,  var);
    }
}

void resolverLaberinto() {
    if(shortMove) {
        //lcd.println("ENTRE AL SHORTMOVE");

        if(preferencia == IZQUIERDA) {
            if(!D_wall) {
                //lcd.println("Short Izquierda");
                switch(iOrientacion) {
                    case A_norte:
                    x_actual--;
                    lastMove = TO_WEST;
                    break;

                    case B_norte:
                    y_actual--;
                    lastMove = TO_SOUTH;
                    break;

                    case C_norte:
                    x_actual++;
                    lastMove = TO_EAST;
                    break;

                    case D_norte:
                    y_actual++;
                    lastMove = TO_NORTH;
                    break;
                }
                vueltaIzq();
                alinear();
                moverCuadro();
                checarLasts();
            } else if(!A_wall) {
                //lcd.println("Short Frente");
                switch(iOrientacion) {
                    case A_norte:
                    y_actual++;
                    lastMove = TO_NORTH;
                    break;

                    case B_norte:
                    x_actual--;
                    lastMove = TO_WEST;
                    break;

                    case C_norte:
                    y_actual--;
                    lastMove = TO_SOUTH;
                    break;

                    case D_norte:
                    x_actual++;
                    lastMove = TO_EAST;
                    break;
                }
                moverCuadro();
                checarLasts();
            } else if(!B_wall) {
                //lcd.println("Short Derecha");
                switch(iOrientacion) {
                    case A_norte:
                    x_actual++;
                    lastMove = TO_EAST;
                    break;

                    case B_norte:
                    y_actual++;
                    lastMove = TO_NORTH;
                    break;

                    case C_norte:
                    x_actual--;
                    lastMove = TO_WEST;
                    break;

                    case D_norte:
                    y_actual--;
                    lastMove = TO_SOUTH;
                    break;
                }
                vueltaDer();
                moverCuadro();
                checarLasts();
            } else {
                //lcd.println("Short Atras");
                switch(iOrientacion) {
                    case A_norte:
                    y_actual--;
                    lastMove = TO_SOUTH;
                    break;

                    case B_norte:
                    x_actual++;
                    lastMove = TO_EAST;
                    break;

                    case C_norte:
                    y_actual++;
                    lastMove = TO_NORTH;
                    break;

                    case D_norte:
                    x_actual--;
                    lastMove = TO_WEST;
                    break;
                }
                vueltaDer();
                checarInterr();
                checarLimit();
                vueltaDer();
                checarInterr();
                checarLimit();
                moverCuadro();
                checarLasts();
            }
        }else

        if(preferencia == DERECHA)
        {
            if(!B_wall) {
                //lcd.println("Short Derecha");
                switch(iOrientacion) {
                    case A_norte:
                    x_actual++;
                    lastMove = TO_EAST;
                    break;

                    case B_norte:
                    y_actual++;
                    lastMove = TO_NORTH;
                    break;

                    case C_norte:
                    x_actual--;
                    lastMove = TO_WEST;
                    break;

                    case D_norte:
                    y_actual--;
                    lastMove = TO_SOUTH;
                    break;
                }
                vueltaDer();
                moverCuadro();
                checarLasts();
            }else if(!A_wall) {
                //lcd.println("Short Frente");
                switch(iOrientacion) {
                    case A_norte:
                    y_actual++;
                    lastMove = TO_NORTH;
                    break;

                    case B_norte:
                    x_actual--;
                    lastMove = TO_WEST;
                    break;

                    case C_norte:
                    y_actual--;
                    lastMove = TO_SOUTH;
                    break;

                    case D_norte:
                    x_actual++;
                    lastMove = TO_EAST;
                    break;
                }
                moverCuadro();
                checarLasts();
            }
            else if(!D_wall) {
                //lcd.println("Short Izquierda");
                switch(iOrientacion) {
                    case A_norte:
                    x_actual--;
                    lastMove = TO_WEST;
                    break;

                    case B_norte:
                    y_actual--;
                    lastMove = TO_SOUTH;
                    break;

                    case C_norte:
                    x_actual++;
                    lastMove = TO_EAST;
                    break;

                    case D_norte:
                    y_actual++;
                    lastMove = TO_NORTH;
                    break;
                }
                vueltaIzq();
                alinear();
                moverCuadro();
                checarLasts();
            }else{
                    switch(iOrientacion) {
                        case A_norte:
                        y_actual--;
                        lastMove = TO_SOUTH;
                        break;

                        case B_norte:
                        x_actual++;
                        lastMove = TO_EAST;
                        break;

                        case C_norte:
                        y_actual++;
                        lastMove = TO_NORTH;
                        break;

                        case D_norte:
                        x_actual--;
                        lastMove = TO_WEST;
                        break;
                    }
                    vueltaDer();
                    checarInterr();
                    checarLimit();
                    vueltaDer();
                    checarInterr();
                    checarLimit();
                    moverCuadro();
                    checarLasts();
                }

            }

    } else {
        if(Last) {
            //lcd.println("LAST");
            lcd.clear();
            lcd.print("GOTO LAST");
            byte var = 255;
            Pathfinding(x_last, y_last, var);

            x_last = 255;
            y_last = 255;
            Last = false;
        } else {
            //lcd.println("GOTO-SR");
            lcd.clear();
            lcd.print("GOTO SR");
            int LowestCSR = 999;
            int iCSR;

            verificarCSR();
            if(ArrayCSR > 0) {
                for (int i = 0; i<ArrayCSR; i++) {
                    byte var = 0;
                    Pathfinding(x_recorrer[i], y_recorrer[i], var);
                    if(var < LowestCSR) {
                        LowestCSR = var;
                        iCSR = i;
                    }
                }
                byte var = 255;
                Pathfinding(x_recorrer[iCSR], y_recorrer[iCSR], var);
            } else {
                // GOTO RAMPA
                verificarRampa();
                if (rampaid)
                {
                    lcd.clear();
                    lcd.print("GOTO RAMPA");
                    byte var = 255;
                    Pathfinding(x_rampa, y_rampa, var);
                    rampaid = false;
                }
                else{
                //lcd.println("GOTO-INICIO");
                lcd.clear();
                lcd.print("GOTO INICIO");
                for(int y=0; y < Y_MAX; y++)
                {
                    for (int x=0; x < X_MAX; x++)
                    {
                        if(cuadros[x][y][z_actual].getEstado() == INICIO)
                        {
                            x_inicio = x;
                            y_inicio = y;
                        }
                    }
                }

                gotoInicio(x_inicio, y_inicio);

                lcd.clear();
                for (int i = 0; i < 8; i++) {
                    lcd.noBacklight();
                    delay(75);
                    lcd.backlight();
                    delay(75);
                }
                lcd.print(" T E O R I A ES");
                lcd.setCursor(0, 1);
                lcd.print("P R A C T I C A");
                delay(60000);
            }
            //gotoSR
            }
        }
    }
}


//******************************************
//******************************************
//--------------SERVO MOTOR-----------------
//******************************************
//******************************************

void servoMotor() {
    if(servo.read() == 0)
        servo.write(180);
    else
        servo.write(0);
    delay(500);
}

void checarmlx() {
    if(!cuadros[x_actual][y_actual][z_actual].getmlx()) {
        servoMotor();
        cuadros[x_actual][y_actual][z_actual].setmlx(true);
    }
}

void EEPROMWriteInt(int p_address, int p_value) {
     byte lowByte = ((p_value >> 0) & 0xFF);
     byte highByte = ((p_value >> 8) & 0xFF);

     EEPROM.write(p_address, lowByte);
     EEPROM.write(p_address + 1, highByte);
}

int EEPROMReadInt(int p_address) {
     byte lowByte = EEPROM.read(p_address);
     byte highByte = EEPROM.read(p_address + 1);

     return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
 }

//-------------- ESCRIBIR EN LA EEPROM ----------------
void escribirValores() {
    EEPROMWriteInt(2, iN_NEGRO);
}

//-------------- LEER DE LA EEPROM ----------------
void leerValores() {
    iN_NEGRO = EEPROMReadInt(2);
}



//******************************************
//---------------jdwjj00------------------

/* Cambia la frecuencia del sensor:
*  0 = filtro apagado
*  2 = filtro del 2%
*  20 = filtro del 20%
*  100 = filtro del 100%
*/
void setFrecuencia(byte frecuencia) {
    switch(frecuencia) {
        case 0:
            digitalWrite(S0, LOW);
            digitalWrite(S1, LOW);
            break;

        case 2:
            digitalWrite(S0, LOW);
            digitalWrite(S1, HIGH);
            break;

        case 20:
            digitalWrite(S0, HIGH);
            digitalWrite(S1, LOW);
            break;

        case 100:
            digitalWrite(S0, HIGH);
            digitalWrite(S1, HIGH);
            break;
    }
}


/* Cambia el filtro del sensor:
*  'N' = Sin filtro
*  'R' = Filtro rojo
*  'G' = Filtro verde
*  'B' = Filtro azul
*/
void setFiltro(char filtro) {
    switch(filtro) {
        case 'N':
        digitalWrite(S2, HIGH);
        digitalWrite(S3, LOW);
        break;

        case 'R':
        digitalWrite(S2, LOW);
        digitalWrite(S3, LOW);
        break;

        case 'G':
        digitalWrite(S2, HIGH);
        digitalWrite(S3, HIGH);
        break;

        case 'B':
        digitalWrite(S2, LOW);
        digitalWrite(S3, HIGH);
        break;
    }
}

// Valor que retorna el sensor
int getColor() {
    return (pulseIn(sensorOut, LOW));
}

// Funcion para calibrar los colores que se usaran
void calibrarColor() {
    while(EstadoColor == ESTADO_OFF) {
        //lcd.println("Calibrar...");
        lcd.setCursor(0, 0);
        lcd.print("   Calibrar... ");
        BotonColor = digitalRead(BOTON_COLOR);
        if(BotonColor == 0) {
            //*Limpia la pantalla
            EstadoColor = ESTADO_NEGRO;
            delay(500);
        }
    }

    lcd.clear();
    lcd.print("    Negro...");
    while(EstadoColor == ESTADO_NEGRO) {
        //lcd.println("Calibrar Negro");
        BotonColor = digitalRead(BOTON_COLOR);
        if(BotonColor == 0) {
            setFiltro('N');
            iN_NEGRO = getColor();

            //*Limpia la pantalla
            delay(500);
            escribirValores();
            //EstadoColor = ESTADO_CHECKPOINT;
            EstadoColor = ESTADO_LISTO;
        }
    }

    lcd.clear();
    lcd.print("   LISTO... ");
    delay(4000);
    EstadoColor++;
    /*while(EstadoColor == ESTADO_LISTO) {
        //lcd.println("Listo...");
        BotonColor = digitalRead(BOTON_COLOR);
        if(BotonColor == 1) {
            //Limpia la pantalla
            EstadoColor++;
            //Pasa al siguiente estado
        }
    }*/
}

// Regresa true si el sensor detecta el color que declares en el parametro
// respetando el margen elegido
bool checarCuadroColor(byte cuadro, byte margen) {
    setFiltro('N');
    iNone = getColor();

    if(iNone <= iN_NEGRO+margen && iNone >= iN_NEGRO-margen)
        return true;
    else
        return false;
}


void checarColor() {
    if(checarCuadroColor(COLOR_NEGRO, 50)) {
        lcd.setCursor(0, 0);
        lcd.print("NEGRO DETECTADO!");
        delay(1000);
        cuadros[x_actual+1][y_actual][z_actual].setPared('O', true);
        cuadros[x_actual-1][y_actual][z_actual].setPared('E', true);
        cuadros[x_actual][y_actual+1][z_actual].setPared('S', true);
        cuadros[x_actual][y_actual-1][z_actual].setPared('N', true);
        cuadros[x_actual][y_actual][z_actual].setEstado(NEGRO);
        reversaCuadro();
        switch (iOrientacion) {
            case A_norte:
            y_actual--;
            break;

            case B_norte:
            x_actual++;
            break;

            case C_norte:
            y_actual++;
            break;

            case D_norte:
            x_actual--;
            break;
        }
        A_wall = true;
    }


    /*if(checarCuadroColor(CHECKPOINT, 20))
    {
      int TotalGrid;
      for(int z=0; z<Z_MAX; z++)
      {
        for(int y = 0; y < Y_MAX; y++)
        {
          for(int x = 0; x < X_MAX; x++)
          {
            TotalGrid = totalCoordToGrid(x, y, z);

            if(cuadros[x][y][z].getmlx())
            checkList1[TotalGrid] += 16;

            if(cuadros[x][y][z].getPared('S'))
            checkList1[TotalGrid] += 8;

            if(cuadros[x][y][z].getPared('E'))
            checkList1[TotalGrid] += 4;

            if(cuadros[x][y][z].getPared('N'))
            checkList1[TotalGrid] += 2;

            if(cuadros[x][y][z].getPared('O'))
            checkList1[TotalGrid] += 1;

            switch(cuadros[x][y][z].getEstado()){
                case NO_EXISTE:
                checkList2[TotalGrid] = 0;
                break;

                case SIN_RECORRER:
                checkList2[TotalGrid] = 1;
                break;

                case RECORRIDO:
                checkList2[TotalGrid] = 2;
                break;

                case CHECKPOINT:
                checkList2[TotalGrid] = 3;
                break;

                case NEGRO:
                checkList2[TotalGrid] = 4;
                break;

                case RAMPA:
                checkList2[TotalGrid] = 5;
                break;

                case INICIO:
                checkList2[TotalGrid] = 6;
                break;
            }

          }
        }
      }
  }*/

}

// Si el array esta a punto de salir de los parametros, mueve la matriz una linea completa
void checarArray() {

    if (x_actual < 1) {
        for(int i = x_actual; i<1; i++)
        {
            recorrerX();
        }
    } else
    if(y_actual < 1) {
        for(int i = y_actual; i<1; i++)
        {
            recorrerY();
        }
    }
}


void imprimirValores1() {
    lcd.clear();
    lcd.print("     SHARPS");
    delay(1000);
    while (digitalRead(BOTON_COLOR) != 0) {
        lcd.clear();
        lcd.setCursor(5, 0);
        lcd.print(getSharpCorta(SHARP_C));
        lcd.setCursor(5, 1);
        lcd.print(getSharpCorta(SHARP_A));
        delay(500);
    }
    delay(500);
}

void imprimirValores2() {
    delay(1000);
    while (digitalRead(BOTON_COLOR) != 0) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(getSharpCorta(SHARP_B2));
        lcd.setCursor(7,0);
        lcd.print(getSharpCorta(SHARP_D2));
        lcd.setCursor(0, 1);
        lcd.print(getSharpCorta(SHARP_B1));
        lcd.setCursor(7,1);
        lcd.print(getSharpCorta(SHARP_D1));
        delay(500);
    }
    delay(500);
}

void imprimirValores3() {
    lcd.clear();
    lcd.print("  ULTRASONICOS");
    delay(1000);
        while(digitalRead(BOTON_COLOR) != 0){
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(getUltrasonicoMediana('A'));
        lcd.setCursor(7,0);
        lcd.print(getUltrasonicoMediana('B'));
        lcd.setCursor(0, 1);
        lcd.print(getUltrasonicoMediana('C'));
        lcd.setCursor(7,1);
        lcd.print(getUltrasonicoMediana('D'));
        delay(500);
        }
    delay(500);
}

void hacerPruebas() {
    lcd.clear();
    lcd.print(" Quieres hacer");
    lcd.setCursor(0, 1);
    lcd.print("    pruebas?");
    delay(800);
    if(digitalRead(BOTON_COLOR) == 0) {
        imprimirValores1();
        imprimirValores2();
        imprimirValores3();
    } else {
        leerValores();
        lcd.clear();
        lcd.print("VALORES ANADIDOS");
    }
}


void setup() {
    Serial.begin(9600);
    //PORTC = (1 << PORTC4) | (1 << PORTC5);    // Habilita ‘pullups’.
    //pinMode(interruptNano, INPUT_PULLUP);  //Pone el pin de interrupcion a la escucha
    //attachInterrupt(digitalPinToInterrupt(interruptNano), victim_Detected, LOW); //Declara la funcion a ejecutar en interruptB
    attachInterrupt(digitalPinToInterrupt(ENC1), addStep, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2), addStep, CHANGE);
    setFrecuencia(100);           //Establece la frecuencia del TCS3200
    pinMode(sensorOut, INPUT);   //Inicializa el pin que recibira la informacion del TCS3200
    pinMode(S0, OUTPUT);         //Establece  pin de Salida
    pinMode(S1, OUTPUT);         //Establece  pin de Salida
    pinMode(S2, OUTPUT);         //Establece  pin de Salida
    pinMode(S3, OUTPUT);         //Establece  pin de Salida
    pinMode(Pin1_DERECHA_ADELANTE, OUTPUT);
    pinMode(Pin1_DERECHA_ATRAS, OUTPUT);
    pinMode(Pin1_IZQUIERDA_ADELANTE, OUTPUT);
    pinMode(Pin1_IZQUIERDA_ATRAS, OUTPUT);
    pinMode(Pin2_DERECHA_ADELANTE, OUTPUT);
    pinMode(Pin2_DERECHA_ATRAS, OUTPUT);
    pinMode(Pin2_IZQUIERDA_ADELANTE, OUTPUT);
    pinMode(Pin2_IZQUIERDA_ATRAS, OUTPUT);
    pinMode(ENABLE_DERECHA_ADELANTE, OUTPUT);
    pinMode(ENABLE_IZQUIERDA_ADELANTE, OUTPUT);
    pinMode(ENABLE_DERECHA_ATRAS, OUTPUT);
    pinMode(ENABLE_IZQUIERDA_ATRAS, OUTPUT);
    pinMode(heatDefiner, INPUT);
    pinMode(visualDefiner, INPUT);
    pinMode(BOTON_COLOR, INPUT);
    lcd.begin();
    lcd.backlight();

    if (digitalRead(BOTON_COLOR) == 0) {
         lcd.clear();
         lcd.print("SUELTE EL BOTON");
         delay(800);
         calibrarColor();
     } else {
         leerValores();
         lcd.clear();
         lcd.print("VALORES ANADIDOS");
     }

    lcd.clear();
    lcd.print("   ROBORREGOS");
    lcd.setCursor(0, 1);
    lcd.print("   T E O R I A");

    servo.attach(servoPin);
    if(servo.read() < 90)
        servo.write(0);

    if(servo.read() >= 90)
        servo.write(180);

    izqPID.SetMode(AUTOMATIC);
    derPID.SetMode(AUTOMATIC);
    setIzq = 0;
    setDer = 0;

    float angle = getAngulo();
    if(angle > 320) {
        inIzq = - (360 - angle);
        inDer = - (360 - angle);
    } else {
        inIzq = angle;
        inDer = angle;
    }

    velocidad(VEL_MOTOR, VEL_MOTOR, VEL_MOTOR, VEL_MOTOR);

    // Inicializa toda la matriz de checkpoint en 0
    for(int i = 0; i < GRID_MAX; i++) {
        checkList1[i] = 0;
        checkList2[i] = 0;
    }
    x_actual = 1;
    y_actual = 1;
    z_actual = 0;
    cuadros[x_actual][y_actual][z_actual].setEstado(INICIO);

    delay(1000);
    hacerPruebas();

    getUltrasonicoUno('D');

    lcd.clear();
    lcd.print("CALIBRANDO IMU");
    delay(1500);
    bno.begin();
    bno.setExtCrystalUse(true);
    lcd.clear();
    lcd.print("CALIBRADO");
    delay(50);

    if(digitalRead(switch_preferencia) == 0)
        preferencia = DERECHA;
    else
        preferencia = IZQUIERDA;
    //attachInterrupt(digitalPinToInterrupt(interruptNano), victim_Detected, LOW);

}

void loop() {
    lcd.clear();
    if(cuadros[x_actual][y_actual][z_actual].getEstado() != INICIO)
       cuadros[x_actual][y_actual][z_actual].setEstado(RECORRIDO);

    checarArray();
    lcd.setCursor(0,1);
    lcd.print(String(x_actual) + "," + String(y_actual) + "," + String(z_actual));
    delay(50);

    checarParedes();
    if(cuadros[x_actual][y_actual][z_actual].getPared('S')) {
       lcd.setCursor(0, 0);
       lcd.print("S");
    }
    if(cuadros[x_actual][y_actual][z_actual].getPared('E')) {
       lcd.setCursor(2, 0);
       lcd.print("E");
    }
    if(cuadros[x_actual][y_actual][z_actual].getPared('N')) {
       lcd.setCursor(4, 0);
       lcd.print("N");
    }
    if(cuadros[x_actual][y_actual][z_actual].getPared('O')) {
       lcd.setCursor(6, 0);
       lcd.print("O");
   }
    resolverLaberinto();
}

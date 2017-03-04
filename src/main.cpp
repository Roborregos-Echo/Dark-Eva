///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////                                         ///////////////////
///////////////////                 E V A                   ///////////////////
///////////////////                                         ///////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//------------------------------ VERSIÓN 1.1.0 --------------------------------
//--------------------------- 03 / MARZO / 2017 -----------------------------
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


//********************************************
//********************************************
//------------------ HEADERS -----------------
//********************************************
//********************************************
void resolverLaberinto();
void servoMotor();


//********************************************
//********************************************
//---------------- LIBRERÍAS ----------------
//********************************************
//********************************************
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <PID_v1.h>



//********************************************
//********************************************
//---------- DECLARACIÓN VARIABLES -----------
//********************************************
//********************************************

//******************************************
//--------------PANTALLA LCD----------------
LiquidCrystal_I2C lcd(0x27, 16, 2);


//******************************************
//-------------CLASE CUADRO-----------------

// iEstado
const int NO_EXISTE     =   0;
const int SIN_RECORRER  =   1;
const int RECORRIDO     =   2;
const int CHECKPOINT    =   3;
const int NEGRO         =   4;
const int RAMPA         =   5;
const int INICIO        =   6;

// LastMove
const byte TO_NORTH = 0;
const byte TO_EAST  = 1;
const byte TO_SOUTH = 2;
const byte TO_WEST  = 3;

// iOrientacion
const byte A_NORTE = 0;
const byte B_NORTE = 1;
const byte C_NORTE = 2;
const byte D_NORTE = 3;

byte iOrientacion = A_NORTE;

// Coordenadas maximas
const byte X_MAX = 15;
const byte Y_MAX = 15;
const byte Z_MAX = 3;

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
//----------- SHIELD ADAFRUIT --------------
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *MotorAI = AFMS.getMotor(1);
Adafruit_DCMotor *MotorAD = AFMS.getMotor(4);
Adafruit_DCMotor *MotorCI = AFMS.getMotor(2);
Adafruit_DCMotor *MotorCD = AFMS.getMotor(3);


//******************************************
//--------------- MOTORES ------------------

const int VEL_MOTOR         =   150;
const int VEL_MOTOR_VUELTA  =   120;
const int VEL_MOTOR_ENCODER  =   115;

const int ENC1   = 18;
const int ENC2   = 19;
unsigned long steps = 0;


//******************************************
//------------- IMU BNO055 ----------------
bool bajarRampa = false;
bool subirRampa = false;
bool permisoRampa = true;
const float PRECISION_IMU = 2.5;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

double setIzq, setDer, inIzq, inDer, outIzq, outDer, inRam, outRam, setRam;
PID izqPID(&inIzq, &outIzq, &setIzq, 15, 0, 0, DIRECT);
PID derPID(&inDer, &outDer, &setDer, 20, 0, 0, REVERSE);
PID ramPID(&inRam, &outRam, &setRam, 60, 0, 0, REVERSE);


//******************************************
//----------- SHARP GP2Y0A21YK -------------
const int SHARP_A   = 12;
const int SHARP_B1  = 14;
const int SHARP_B2  = 9;
const int SHARP_C   = 2;
const int SHARP_D1  = 13;
const int SHARP_D2  = 7;
const int SHARP_LA  = 0;
const int SHARP_LC  = 11;


//******************************************
//------------- PATHFINDING ----------------
const int GRID_MAX = X_MAX * Y_MAX;

int openList[GRID_MAX];
int closedList[GRID_MAX];
int backList[GRID_MAX];
int Neighbors[4];


//******************************************
//-------------RAMPA ALGORITHM--------------
const byte ABAJO = 1; const byte ARRIBA = 2;

bool Piso1 = false;
bool Piso2 = false;
bool Piso3 = false;
bool Fusion = false;        // Cambiar a true cuando se junten los pisos
bool GridOriginal[GRID_MAX];
bool SharpRampa = true;

byte firstFloor = 0;
byte RampaDiff = 4;
byte PisoReal  = 0;
byte MoveL1, MoveL2;
byte LastMove;            // 0,1,0
byte RampaLastMove;


//******************************************
//-------------- CHECKPOINT ----------------
const int TOTAL_GRID_MAX = X_MAX * Y_MAX * Z_MAX;
byte checkList1[GRID_MAX];
byte checkList2[GRID_MAX];


//******************************************
//---------------INTERRUPTS-----------------

// Interupcion del nano;
#define InterruptNano 2
#define InterruptBoton 3
#define InterruptDefiner 9


//******************************************
//--------------SERVO MOTOR-----------------
#define servoPin 9 //PWM
Servo servo;


//******************************************
//-----------------TCS3200------------------
#define S0 7
#define S1 8
#define S2 5
#define S3 4
#define sensorOut 6
#define BOTON_COLOR 12

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


//********************************************
//------------------- IMU -------------------
float getAngulo() {
    sensors_event_t event;
    bno.getEvent(&event);
    return event.orientation.x;
}


//********************************************
//----------------- MOTORES ------------------
void avanzar() {
    MotorAI -> run(FORWARD);
    MotorAD -> run(FORWARD);
    MotorCI -> run(FORWARD);
    MotorCD -> run(FORWARD);
}

void reversa() {
    MotorAI -> run(BACKWARD);
    MotorAD -> run(BACKWARD);
    MotorCI -> run(BACKWARD);
    MotorCD -> run(BACKWARD);
}

void detener() {
    MotorAI -> run(BRAKE);
    MotorAD -> run(BRAKE);
    MotorCI -> run(BRAKE);
    MotorCD -> run(BRAKE);
}

void vueltaDerecha() {
    MotorAI -> run(FORWARD);
    MotorAD -> run(BACKWARD);
    MotorCI -> run(FORWARD);
    MotorCD -> run(BACKWARD);
}

void vueltaIzquierda() {
    MotorAI -> run(BACKWARD);
    MotorAD -> run(FORWARD);
    MotorCI -> run(BACKWARD);
    MotorCD -> run(FORWARD);
}

void horizontalDerecha() {
    MotorAI -> run(FORWARD);
    MotorAD -> run(BACKWARD);
    MotorCI -> run(BACKWARD);
    MotorCD -> run(FORWARD);
}

void horizontalIzquierda() {
    MotorAI -> run(BACKWARD);
    MotorAD -> run(FORWARD);
    MotorCI -> run(FORWARD);
    MotorCD -> run(BACKWARD);
}


//******************************************
//----------------- SHARP ------------------
float getSharpCorta(int iSharp) {
    //int sharp = 2316.6 *(0.985 / analogRead(A7));  viejo
    //3742.4 * (1 / pow(sharpRead[3], 1.081)); osvaldo
    //3582.4 * (pow(sharpRead[3], -1.047)); neto
    int sharpRead[20];
    float resultado;
    for(int i = 0; i < 20; i++) {
        sharpRead[i] = analogRead(iSharp);
    }

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

    if (sharpRead[10] >= 75 && sharpRead[10] <= 580)
        resultado = 2429 * (pow(sharpRead[10], -1.004));
    else
        resultado = 30;

    return resultado;
}

float getSharpLarga(int iSharp) {
    // resultado = 20472 * pow(sharpRead[4], -1.045
    int sharpRead[30];
    float resultado;
    for(int i = 0; i < 30; i++) {
        sharpRead[i] = analogRead(iSharp);
    }

    for (int j = 0; j < 29; j++) {
        for (int i = 0; i < 28; i++) {
            int temp;
            if(sharpRead[i] > sharpRead[i + 1]) {
                temp = sharpRead[i + 1];
                sharpRead[i + 1] = sharpRead[i];
                sharpRead[i] = temp;
            }
        }
    }

    if (sharpRead[15] >= 100 and sharpRead[15] <= 500)
        resultado = 17921 * pow(sharpRead[15], -1.072);
    else
        resultado = -1;

    return resultado;
}


//******************************************
//---------------- ENCODER -----------------
void addStep(){
  steps++;
}


//******************************************
//--------------- MOVIMIENTO ---------------
void velocidad(int ai, int ad, int ci, int cd) {
    if(ai >= 255)
        MotorAI -> setSpeed(255);
    else
        MotorAI -> setSpeed(ai);

    if(ad >= 255)
        MotorAD -> setSpeed(255);
    else
        MotorAD -> setSpeed(ad);

    if(ci >= 255)
        MotorCI -> setSpeed(255);
    else
        MotorCI -> setSpeed(ci);

    if(cd >= 255)
        MotorCD -> setSpeed(255);
    else
        MotorCD -> setSpeed(cd);
}

void alinear() {
    velocidad(VEL_MOTOR_VUELTA, VEL_MOTOR_ENCODER, VEL_MOTOR_VUELTA, VEL_MOTOR_VUELTA);
    if(getSharpCorta(SHARP_B1) < 20 && getSharpCorta(SHARP_D1) < 20) {
        while (abs(getSharpCorta(SHARP_B1) - getSharpCorta(SHARP_D1)) > 1) {
            while(getSharpCorta(SHARP_B1) - getSharpCorta(SHARP_D1) > 1)
                horizontalDerecha();
            detener();
            while(getSharpCorta(SHARP_D1) - getSharpCorta(SHARP_B1) > 1)
                horizontalIzquierda();
            detener();
        }
    } else if(getSharpCorta(SHARP_B1) < 20) {
        while (!(getSharpCorta(SHARP_B1) > 9.0 && getSharpCorta(SHARP_B1) < 10.0)) {
            while (getSharpCorta(SHARP_B1) < 9)
                horizontalIzquierda();
            detener();
            while (getSharpCorta(SHARP_B1) > 10)
                horizontalDerecha();
            detener();
        }
        if (abs(getSharpCorta(SHARP_B1) - getSharpCorta(SHARP_B2)) > 1.5 ) {
            while (getSharpCorta(SHARP_B2) < getSharpCorta(SHARP_B1))
                vueltaDerecha();
            detener();
            while (getSharpCorta(SHARP_B1) < getSharpCorta(SHARP_B2))
                vueltaIzquierda();
            detener();
        }
    } else if(getSharpCorta(SHARP_D1) < 20) {
        while (!(getSharpCorta(SHARP_D1) > 9.0 && getSharpCorta(SHARP_D1) < 10.0)) {
            while (getSharpCorta(SHARP_D1) < 9)
                horizontalDerecha();
            detener();
            while (getSharpCorta(SHARP_D1) > 10)
                horizontalIzquierda();
            detener();
        }
        if (abs(getSharpCorta(SHARP_D1) - getSharpCorta(SHARP_D2)) > 1.5 ) {
            while (getSharpCorta(SHARP_D2) < getSharpCorta(SHARP_D1))
                vueltaIzquierda();
            detener();
            while (getSharpCorta(SHARP_D1) < getSharpCorta(SHARP_D2))
                vueltaDerecha();
            detener();
        }
    }


    if (getSharpCorta(SHARP_A) < 20) {
        unsigned long inicio = millis();
        while (!(getSharpCorta(SHARP_A) > 8.5 && getSharpCorta(SHARP_A) < 9.5 && getSharpLarga(SHARP_LA) < 60)) {
            while (getSharpCorta(SHARP_A) < 8.5)
                reversa();
            detener();
            while (getSharpCorta(SHARP_A) > 9.5 && getSharpLarga(SHARP_LA) < 60)
                avanzar();
            detener();
            lcd.home();
            lcd.print("ALINEAR AVANZAR");
            if (millis() + 8000 >= inicio)
                break;
        }
        detener();
    }

    /*if (getSharpCorta(SHARP_C) < 14) {
        //lcd.clear();
        //lcd.home();
        //lcd.print("SHARP C");
        while (!(getSharpCorta(SHARP_C) > 8 && getSharpCorta(SHARP_C) < 10)) {
            lcd.home();
            lcd.print("1");
            while (getSharpCorta(SHARP_C) < 8.5)
                avanzar();
            detener();
            lcd.home();
            lcd.print("2");
            while (getSharpCorta(SHARP_C) > 9.5 && getSharpLarga(SHARP_LC) < 30) {
                reversa();
                lcd.clear();
                lcd.setCursor(0, 1);
                lcd.print(getSharpCorta(SHARP_C));
            }
            detener();
            lcd.home();
            lcd.print("3");
        }
    }*/
}


void vueltaIzq() {
    float posInicial, posFinal, limInf, limSup;
    posInicial = getAngulo();
    lcd.setCursor(8, 1);
    lcd.print("Vuel Izq");
    switch(iOrientacion) {
        case A_NORTE:
        posFinal = 270;
        setIzq = 270;
        setDer = 270;
        break;

        case B_NORTE:
        posFinal = 180;
        setIzq = 180;
        setDer = 180;
        break;

        case C_NORTE:
        posFinal = 90;
        setIzq = 90;
        setDer = 90;
        break;

        case D_NORTE:
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

    velocidad(VEL_MOTOR_VUELTA, VEL_MOTOR_VUELTA, VEL_MOTOR_VUELTA, VEL_MOTOR_VUELTA);
    vueltaIzquierda();

    if(limSup > limInf) {
        while(!(posInicial >= limInf && posInicial <= limSup)) {
            posInicial = getAngulo();
        }
        detener();
    } else {
        while(!(posInicial >= limInf || posInicial <= limSup)) {
            posInicial = getAngulo();
        }
        detener();
    }
    velocidad(VEL_MOTOR, VEL_MOTOR, VEL_MOTOR, VEL_MOTOR);
    detener();

    switch(iOrientacion) {
        case A_NORTE:
        iOrientacion = B_NORTE;
        break;

        case B_NORTE:
        iOrientacion = C_NORTE;
        break;

        case C_NORTE:
        iOrientacion = D_NORTE;
        break;

        case D_NORTE:
        iOrientacion = A_NORTE;
        break;
    }
    delay(400);
}

void vueltaDer() {
    float posInicial, posFinal, limInf, limSup;
    lcd.setCursor(8, 1);
    lcd.print("Vuel Der");
    posInicial = getAngulo();
    switch(iOrientacion) {
        case A_NORTE:
        posFinal = 90;
        setIzq = 90;
        setDer = 90;
        break;

        case B_NORTE:
        posFinal = 0;
        setIzq = 0;
        setDer = 0;
        break;

        case C_NORTE:
        posFinal = 270;
        setIzq = 270;
        setDer = 270;
        break;

        case D_NORTE:
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

    velocidad(VEL_MOTOR_VUELTA, VEL_MOTOR_VUELTA, VEL_MOTOR_VUELTA, VEL_MOTOR_VUELTA);
    vueltaDerecha();

    if(limSup > limInf) {
        while(!(posInicial >= limInf && posInicial <= limSup)) {
            posInicial = getAngulo();
        }
        detener();
    } else {
        while(!(posInicial >= limInf || posInicial <= limSup)) {
            posInicial = getAngulo();
        }
        detener();
    }
    velocidad(VEL_MOTOR, VEL_MOTOR, VEL_MOTOR, VEL_MOTOR);
    detener();

    switch(iOrientacion) {
        case A_NORTE:
        iOrientacion = D_NORTE;
        break;

        case B_NORTE:
        iOrientacion = A_NORTE;
        break;

        case C_NORTE:
        iOrientacion = B_NORTE;
        break;

        case D_NORTE:
        iOrientacion = C_NORTE;
        break;
    }
    delay(200);
}

void vueltaAtras() {
    float posInicial, posFinal, limInf, limSup;
    posInicial = getAngulo();
    lcd.setCursor(8, 1);
    lcd.print("Vuel Atr");
    switch(iOrientacion) {
        case A_NORTE:
        posFinal = 180;
        setIzq = 180;
        setDer = 180;
        break;

        case B_NORTE:
        posFinal = 90;
        setIzq = 90;
        setDer = 90;
        break;

        case C_NORTE:
        posFinal = 0;
        setIzq = 0;
        setDer = 0;
        break;

        case D_NORTE:
        posFinal = 270;
        setIzq = 270;
        setDer = 270;
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

    velocidad(VEL_MOTOR_VUELTA, VEL_MOTOR_VUELTA, VEL_MOTOR_VUELTA, VEL_MOTOR_VUELTA);
    vueltaDerecha();

    if(limSup > limInf) {
        while(!(posInicial >= limInf && posInicial <= limSup)) {
            posInicial = getAngulo();
        }
        detener();
    } else {
        while(!(posInicial >= limInf || posInicial <= limSup)) {
            posInicial = getAngulo();
        }

        detener();
    }
    velocidad(VEL_MOTOR, VEL_MOTOR, VEL_MOTOR, VEL_MOTOR);
    detener();

    switch(iOrientacion) {
        case A_NORTE:
        iOrientacion = C_NORTE;
        break;

        case B_NORTE:
        iOrientacion = D_NORTE;
        break;

        case C_NORTE:
        iOrientacion = A_NORTE;
        break;

        case D_NORTE:
        iOrientacion = B_NORTE;
        break;
    }
    delay(200);
}

class Cuadro {
    byte Estado;
    bool Norte, Este, Sur, Oeste;
    bool Mlx;

    public: Cuadro() {
        Estado  = NO_EXISTE;
        Norte   = false;
        Este    = false;
        Sur     = false;
        Oeste   = false;
        Mlx     = false;
    }

    void setPared(char cSentido, bool bBool) {
        switch(cSentido) {
            case 'N':
            Norte = bBool;
            break;
            case 'E':
            Este = bBool;
            break;
            case 'S':
            Sur = bBool;
            break;
            case 'O':
            Oeste = bBool;
            break;
        }
    }

    void setEstado(int iEstado) {
        Estado = iEstado;
    }

    void setMlx(bool b) {
        Mlx = b;
    }

    bool getMlx() {
        return Mlx;
    }

    bool getPared(char cSentido) {
        switch(cSentido) {
            case 'N':
            return Norte;
            break;
            case 'E':
            return Este;
            break;
            case 'S':
            return Sur;
            break;
            case 'O':
            return Oeste;
            break;
        }
    }

    int getEstado() {
        return Estado;
    }
};

Cuadro cuadros[X_MAX][Y_MAX][Z_MAX];

void setWall(byte x, byte y, byte z){
    switch(LastMove)
    {
        case TO_NORTH:
        cuadros[x][y][z].setPared('S', true);
        break;

        case TO_EAST:
        cuadros[x][y][z].setPared('O', true);
        break;

        case TO_SOUTH:
        cuadros[x][y][z].setPared('N', true);
        break;

        case TO_WEST:
        cuadros[x][y][z].setPared('E', true);
        break;
    }
}

void setInicioB(){
    switch(LastMove)
    {
        case TO_NORTH:
        x_InicioB = x_actual;
        y_InicioB = y_actual-1;
        break;

        case TO_EAST:
        x_InicioB = x_actual-1;
        y_InicioB = y_actual;
        break;

        case TO_SOUTH:
        x_InicioB = x_actual;
        y_InicioB = y_actual+1;
        break;

        case TO_WEST:
        x_InicioB = x_actual+1;
        y_InicioB = y_actual;
        break;

    }

    z_InicioB = z_actual;
}

void setInicioC(){
    switch(LastMove)
    {
        case TO_NORTH:
        x_InicioC = x_actual;
        y_InicioC = y_actual-1;
        break;

        case TO_EAST:
        x_InicioC = x_actual-1;
        y_InicioC = y_actual;
        break;

        case TO_SOUTH:
        x_InicioC = x_actual;
        y_InicioC = y_actual+1;
        break;

        case TO_WEST:
        x_InicioC = x_actual+1;
        y_InicioC = y_actual;
        break;

    }

    z_InicioC = z_actual;

}

void setRampa(byte x, byte y, byte z){
    cuadros[x][y][z].setEstado(RAMPA);
}

void checarRampa(){
    if(subirRampa or bajarRampa)
    {

        SharpRampa = false;

        if(x_last != 255 and y_last != 255)
        {
            cuadros[x_last][y_last][z_actual].setEstado(SIN_RECORRER);
            x_last = 255;
            y_last = 255;
            Last = false;
        }

        if(x_last2 != 255 and y_last2 != 255)
        {
            cuadros[x_last2][y_last2][z_actual].setEstado(SIN_RECORRER);
            x_last2 = 255;
            y_last2 = 255;
            Last2 = false;
        }
        lcd.clear();
        lcd.print("     RAMPA");
        if(firstFloor == 0)
        {
            if(subirRampa)
            firstFloor = ABAJO;

            if(bajarRampa)
            firstFloor = ARRIBA;
        }

        switch(LastMove)
        {
            case TO_NORTH:
            lcd.setCursor(0,1);
            lcd.print("TO_NORTH");
            setRampa(x_actual, y_actual +1, z_actual);
            y_actual += (RampaDiff);
            RampaLastMove = TO_NORTH;
            break;

            case TO_EAST:
            lcd.setCursor(0,1);
            lcd.print("TO_EAST");
            setRampa(x_actual +1, y_actual, z_actual);
            x_actual += (RampaDiff);
            RampaLastMove = TO_EAST;
            break;

            case TO_SOUTH:
            lcd.setCursor(0,1);
            lcd.print("TO_SOUTH");
            setRampa(x_actual, y_actual -1, z_actual);
            y_actual -= (RampaDiff);
            RampaLastMove = TO_SOUTH;
            break;

            case TO_WEST:
            lcd.setCursor(0,1);
            lcd.print("TO_WEST");
            setRampa(x_actual -1, y_actual, z_actual);
            x_actual -= (RampaDiff);
            RampaLastMove = TO_WEST;
            break;
        }

        if(firstFloor == ABAJO)
        {
            if(subirRampa)
            {
                if(Piso3 and !Fusion)
                {
                    z_actual--;
                    setWall(x_actual, y_actual, z_actual);
                    C_wall = true;
                    //resolverLaberinto();
                }
                else
                if(Fusion)
                {
                    z_actual++;
                    C_wall = true;
                    //resolverLaberinto();
                }
                else
                {
                    z_actual++;
                    setInicioB();
                    C_wall = true;
                    //resolverLaberinto();
                }
            }
            else
            if(bajarRampa)
            {
                if(!Piso2)
                {
                    z_actual++;
                    setInicioC();
                    //setWall arriba
                    switch(LastMove)
                    {
                        case TO_NORTH:
                        setWall(x_actual, y_actual - (RampaDiff+1), z_actual-1);
                        break;

                        case TO_EAST:
                        setWall(x_actual - (RampaDiff+1), y_actual, z_actual-1);
                        break;

                        case TO_SOUTH:
                        setWall(x_actual, y_actual + (RampaDiff+1), z_actual-1);
                        break;

                        case TO_WEST:
                        setWall(x_actual + (RampaDiff+1), y_actual, z_actual-1);
                        break;
                    }
                    C_wall = true;
                    //resolverLaberinto();
                }
                else
                {
                    z_actual--;
                    setWall(x_actual, y_actual, z_actual);
                    C_wall = true;
                    //resolverLaberinto();
                }
            }

        }
        else
        if(firstFloor == ARRIBA)
        {
            if(bajarRampa)
            {
                if(Piso2)
                {
                    z_actual += 2;
                    setInicioC();
                    C_wall = true;
                    //resolverLaberinto();
                }
                else
                {
                    z_actual++;
                    setInicioB();
                    C_wall = true;
                    //resolverLaberinto();
                }
            }
            else
            if(subirRampa)
            {
                if(z_actual == 1 and Piso2)
                {
                    z_actual--;
                    setWall(x_actual, y_actual, z_actual);
                    //resolverLaberinto();
                }
                else
                if(z_actual == 1 and !Piso2)
                {
                    permisoRampa = false;
                    setInicioC();
                    switch(LastMove)
                    {
                        case TO_NORTH:
                        setWall(x_actual, y_actual + (RampaDiff+1), z_actual-1);
                        break;

                        case TO_EAST:
                        setWall(x_actual + (RampaDiff+1), y_actual, z_actual-1);
                        break;

                        case TO_SOUTH:
                        setWall(x_actual, y_actual - (RampaDiff+1), z_actual);
                        break;

                        case TO_WEST:
                        setWall(x_actual - (RampaDiff+1), y_actual, z_actual);
                        break;
                    }
                    A_wall = true;
                    //resolverLaberinto();
                }
                else
                if(z_actual == 2)
                {
                    z_actual -= 2;
                    setWall(x_actual, y_actual, z_actual);
                    C_wall = true;
                    //resolverLaberinto();
                }
            }
        }

        switch(LastMove)
        {
            case TO_NORTH:
            setRampa(x_actual, y_actual -1, z_actual);
            break;

            case TO_EAST:
            setRampa(x_actual -1, y_actual, z_actual);
            break;

            case TO_SOUTH:
            setRampa(x_actual, y_actual +1, z_actual);
            break;

            case TO_WEST:
            setRampa(x_actual +1, y_actual, z_actual);
            break;
        }
    }
    subirRampa = false;
    bajarRampa = false;
}

//******************************************
//******************************************
//---------------INTERRUPTS-----------------
//******************************************
//******************************************

bool inFire = false;

void funcionB() {
     inFire = true;
}

void funcionD() {
}

void checarInterr() {
    unsigned long pos = 0;
    if(inFire == true) {
        if(digitalRead(InterruptDefiner))
        {
            pos = steps;
            detener();
            delay(2000);
            vueltaIzq();
            servoMotor();
            delay(1000);
            vueltaDer();
            delay(2000);
            inFire = false;
            steps = pos;
        }
        else
        {
            pos = steps;
            detener();
            delay(2000);
            vueltaDer();
            servoMotor();
            delay(1000);
            vueltaIzq();
            delay(2000);
            inFire = false;
            steps = pos;
        }

    }

}

void moverCuadro() {
    steps = 0;
    while (steps <= 2500) {
        avanzar();
        if(getAngulo() > 320)
            inIzq = - (360 - getAngulo());
        else
            inIzq = getAngulo();

        if(getAngulo() > 320)
            inDer = - (360 - getAngulo());
        else
            inDer = getAngulo();

        izqPID.Compute();
        derPID.Compute();
        velocidad(VEL_MOTOR + outIzq, VEL_MOTOR + outDer, VEL_MOTOR + outIzq, VEL_MOTOR + outDer);
        checarInterr();
    }

    imu::Vector<3> vec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    if(vec.y() < -4.0) {
        subirRampa = true;
        checarRampa();
        if (permisoRampa) {
            setRam = (getSharpCorta(SHARP_D1) + getSharpCorta(SHARP_D2)) / 2;
            while (vec.y() < -4.0) {
                if(getAngulo() > 320)
                    inIzq = - (360 - getAngulo());
                else
                    inIzq = getAngulo();
                if(getAngulo() > 320)
                    inDer = - (360 - getAngulo());
                else
                    inDer = getAngulo();
                inRam = getSharpCorta(SHARP_D1);
                ramPID.Compute();
                izqPID.Compute();
                derPID.Compute();
                if (outRam > 4)
                    velocidad(VEL_MOTOR, VEL_MOTOR + outRam, VEL_MOTOR, VEL_MOTOR + outRam);
                else
                    velocidad(VEL_MOTOR + outIzq, VEL_MOTOR + outDer + outRam, VEL_MOTOR + outIzq, VEL_MOTOR + outDer + outRam);
                vec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
            }
            velocidad(VEL_MOTOR, VEL_MOTOR, VEL_MOTOR, VEL_MOTOR);
        }
    } else if(vec.y() > 4.0) {
        bajarRampa = true;
        checarRampa();
        if (permisoRampa) {
            while (vec.y() > 4.0) {
                if(getAngulo() > 320)
                    inIzq = - (360 - getAngulo());
                else
                    inIzq = getAngulo();
                if(getAngulo() > 320)
                    inDer = - (360 - getAngulo());
                else
                    inDer = getAngulo();
                izqPID.Compute();
                derPID.Compute();
                velocidad(VEL_MOTOR + outIzq, VEL_MOTOR + outDer, VEL_MOTOR + outIzq, VEL_MOTOR + outDer);
                vec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
            }
            velocidad(VEL_MOTOR, VEL_MOTOR, VEL_MOTOR, VEL_MOTOR);
        }
    } else {
        steps = 0;
        while (steps <= 1500) {
            avanzar();
            if(getAngulo() > 320)
                inIzq = - (360 - getAngulo());
            else
                inIzq = getAngulo();
            if(getAngulo() > 320)
                inDer = - (360 - getAngulo());
            else
                inDer = getAngulo();
            izqPID.Compute();
            derPID.Compute();
            velocidad(VEL_MOTOR + outIzq, VEL_MOTOR + outDer, VEL_MOTOR + outIzq, VEL_MOTOR + outDer);
            lcd.setCursor(0, 1);
            lcd.print(steps);
            checarInterr();
        }
    }
    steps = 0;
    avanzar();
    while (steps <= 1500) {
        avanzar();
        if(getAngulo() > 320)
            inIzq = - (360 - getAngulo());
        else
            inIzq = getAngulo();
        if(getAngulo() > 320)
            inDer = - (360 - getAngulo());
        else
            inDer = getAngulo();
        izqPID.Compute();
        derPID.Compute();
        velocidad(VEL_MOTOR + outIzq, VEL_MOTOR + outDer, VEL_MOTOR + outIzq, VEL_MOTOR + outDer);
        checarInterr();
    }
    detener();
    alinear();
    delay(500);
    permisoRampa = true;
}


void reversaCuadro() {
    steps = 0;
    reversa();
    while (steps <= 4700) {
        lcd.setCursor(0, 1);
        lcd.print(steps);

        /*if(getAngulo() > 320)
            inDer = - (360 - getAngulo());
        else
            inDer = getAngulo();

        if(getAngulo() > 320)
            inIzq = - (360 - getAngulo());
        else
            inIzq = getAngulo();

        izqPID.Compute();
        derPID.Compute();
        velocidad(VEL_MOTOR + outIzq, VEL_MOTOR + outDer, VEL_MOTOR + outIzq, VEL_MOTOR + outDer);*/
    }
    detener();
}


void absoluteMove(char cLado) {
    switch (iOrientacion) {
        case A_NORTE:
        switch(cLado) {
            case 'N':
            LastMove = TO_NORTH;
            moverCuadro();
            break;

            case 'E':
            LastMove = TO_EAST;
            vueltaDer();
            alinear();
            moverCuadro();
            break;

            case 'S':
            LastMove = TO_SOUTH;
            vueltaAtras();
            alinear();
            moverCuadro();
            break;

            case 'O':
            LastMove = TO_WEST;
            vueltaIzq();
            alinear();
            moverCuadro();
            break;
        }
        break;

        case B_NORTE:
        switch(cLado) {
            case 'N':
            LastMove = TO_NORTH;
            vueltaDer();
            alinear();
            moverCuadro();
            break;

            case 'E':
            LastMove = TO_EAST;
            vueltaAtras();
            alinear();
            moverCuadro();
            break;

            case 'S':
            LastMove = TO_SOUTH;
            vueltaIzq();
            alinear();
            moverCuadro();
            break;

            case 'O':
            LastMove = TO_WEST;
            moverCuadro();
            break;
        }
        break;

        case C_NORTE:
        switch(cLado) {
            case 'N':
            LastMove = TO_NORTH;
            vueltaAtras();
            alinear();
            moverCuadro();
            break;

            case 'E':
            LastMove = TO_EAST;
            vueltaIzq();
            alinear();
            moverCuadro();
            break;

            case 'S':
            LastMove = TO_SOUTH;
            moverCuadro();
            break;

            case 'O':
            LastMove = TO_WEST;
            vueltaDer();
            alinear();
            moverCuadro();
            break;
        }
        break;

        case D_NORTE:
        switch(cLado) {
            case 'N':
            LastMove = TO_NORTH;
            vueltaIzq();
            alinear();
            moverCuadro();
            break;

            case 'E':
            LastMove = TO_EAST;
            moverCuadro();
            break;

            case 'S':
            LastMove = TO_SOUTH;
            vueltaDer();
            alinear();
            moverCuadro();
            break;

            case 'O':
            LastMove = TO_WEST;
            vueltaAtras();
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

    switch (eje)
    {
      case 'x':
      return x;
      break;

      case 'y':
      return y;
      break;

      case 'z':
      return z;
      break;
    }
}

int totalCoordToGrid(byte x, byte y, byte z){
  return x + (y * X_MAX) + (z * (X_MAX * Y_MAX));
}

byte pathway(byte x_inicial, byte y_inicial, byte x_final, byte y_final) {
    return fabs(x_final - x_inicial) + fabs(y_final - y_inicial);
}

//******************************************
//------------- PATHFINDING ----------------
void Pathfinding(byte x_destino, byte y_destino, byte &ref) {
    Serial.println("Estoy en PathFinding");
    Serial.println("Actual = " + String(x_actual)+ "," + String(y_actual));
    Serial.println("Destino = " + String(x_destino)+ "," + String(y_destino));
    lcd.setCursor(0, 1);
    lcd.print("    " + String(x_destino) + "," + String(y_destino));
    bool pathFinished = false;
    int Grid;
    byte x_path = x_actual;
    byte y_path = y_actual;

    byte neighbor_index;
    int NeighborSortValue;
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
        Serial.println("Entre al while");
        NeighborSortValue = 999;
        openSortValue = 999;
        LastPathValue = pathway(x_path, y_path, x_destino, y_destino);
        Serial.println("LastPathValue =" + String(LastPathValue));

        if(x_destino == x_actual and y_destino == y_actual) {
            pathFinished = true;
            lcd.setCursor(0, 1);
            lcd.print("Sali rapido");
            delay(500);
            Serial.println("Sali rapidamente");
        }

        for (int i = 0; i<4; i++)
            Neighbors[i] = 999;

        if(!firstLoop) {
            closedList[coordToGrid(x_path, y_path)] = LastPathValue;
            firstLoop = true;
        }


        if (!cuadros[x_path][y_path][z_actual].getPared('S')) {
            if(y_path > 0) {
                if (cuadros[x_path][y_path-1][z_actual].getEstado() == INICIO or cuadros[x_path][y_path-1][z_actual].getEstado() == CHECKPOINT or
                cuadros[x_path][y_path-1][z_actual].getEstado() == RECORRIDO or (x_path == x_destino and y_path-1 == y_destino)) {
                    Grid = coordToGrid(x_path, y_path-1);
                    if(closedList[Grid] == 999) {
                        openList[Grid] = pathway(x_path, y_path-1, x_destino, y_destino);
                        Neighbors[0] = pathway(x_path, y_path-1, x_destino, y_destino);
                    }
                }
            }
        }

        if (!cuadros[x_path][y_path][z_actual].getPared('E')) {
            if(x_path < X_MAX -1) {
                if (cuadros[x_path+1][y_path][z_actual].getEstado() == INICIO or cuadros[x_path+1][y_path][z_actual].getEstado() == CHECKPOINT or
                cuadros[x_path+1][y_path][z_actual].getEstado() == RECORRIDO or (x_path+1 == x_destino and y_path == y_destino)) {
                    Grid = coordToGrid(x_path+1, y_path);
                    if(closedList[Grid] == 999) {
                        openList[Grid] = pathway(x_path+1, y_path, x_destino, y_destino);
                        Neighbors[1] = pathway(x_path+1, y_path, x_destino, y_destino);
                    }
                }
            }
        }

        if (!cuadros[x_path][y_path][z_actual].getPared('N')) {
            if(y_path < Y_MAX -1) {
                if (cuadros[x_path][y_path+1][z_actual].getEstado() == INICIO or cuadros[x_path][y_path+1][z_actual].getEstado() == CHECKPOINT or
                cuadros[x_path][y_path+1][z_actual].getEstado() == RECORRIDO or (x_path == x_destino and y_path+1 == y_destino)) {
                    Grid = coordToGrid(x_path, y_path+1);
                    if(closedList[Grid] == 999) {
                        openList[Grid] = pathway(x_path, y_path+1, x_destino, y_destino);
                        Neighbors[2] = pathway(x_path, y_path+1, x_destino, y_destino);
                    }
                }
            }
        }

        if (!cuadros[x_path][y_path][z_actual].getPared('O')) {
            if(x_path > 0) {
                if (cuadros[x_path-1][y_path][z_actual].getEstado() == INICIO or cuadros[x_path-1][y_path][z_actual].getEstado() == CHECKPOINT or
                cuadros[x_path-1][y_path][z_actual].getEstado() == RECORRIDO or (x_path-1 == x_destino and y_path == y_destino)) {
                    Grid = coordToGrid(x_path-1, y_path);
                    if(closedList[Grid] == 999) {
                        openList[Grid] = pathway(x_path-1, y_path, x_destino, y_destino);
                        Neighbors[3] = pathway(x_path-1, y_path, x_destino, y_destino);
                    }
                }
            }
        }

        for (int i = 0; i<4; i++) {
            if(Neighbors[i] < NeighborSortValue) {
                neighbor_index = i;
                NeighborSortValue = Neighbors[i];
            }
        }

        if (NeighborSortValue < LastPathValue) {
            LastPathValue = NeighborSortValue;
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

            if(openSortValue < NeighborSortValue) {
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

        Serial.println("Paths: " + String(x_path) + "," + String(y_path) + " = " + String(pathway(x_path, y_path, x_destino, y_destino)));
        //delay(1000);
        Serial.println("LastPath = " + String(x_lastPath) + "," + String(y_lastPath));
        //delay(1000);
        if(x_path == x_destino and y_path == y_destino) {
                    /*for(int i = 0; i<GRID_MAX; i++)
                    {
                    Serial.println("Open[" + String(i) + "] = " + String (openList[i]));
                }
                for(int i = 0; i<GRID_MAX; i++)
                {
                Serial.println("Closed[" + String(i) + "] = " + String (closedList[i]));
            }*/

            //Encontrar camino de regreso
            x_back = x_path;
            y_back = y_path;
            /*Serial.println(x_back);
            Serial.println(y_back);*/

            while(!backFinished) {
                for (int i = 0; i<4; i++)
                Neighbors[i] = 999;

                NeighborSortValue = 999;


                if(!cuadros[x_back][y_back][z_actual].getPared('S')) {
                    if(y_back > 0) {
                        TempGrid = coordToGrid(x_back, y_back-1);
                        if(closedList[TempGrid] != 999 and backList[lastBackGrid+1] != TempGrid)
                        Neighbors[0] = pathway(x_back, y_back-1, x_actual, y_actual);
                    }
                }

                if(!cuadros[x_back][y_back][z_actual].getPared('E')) {
                    if(x_back < X_MAX -1) {
                        TempGrid = coordToGrid(x_back+1, y_back);
                        if(closedList[TempGrid] != 999 and backList[lastBackGrid+1] != TempGrid)
                        Neighbors[1] = pathway(x_back+1, y_back, x_actual, y_actual);
                    }
                }

                if(!cuadros[x_back][y_back][z_actual].getPared('N')) {
                    if(y_back < Y_MAX -1) {
                        TempGrid = coordToGrid(x_back, y_back+1);
                        if(closedList[TempGrid] != 999 and backList[lastBackGrid+1] != TempGrid)
                        Neighbors[2] = pathway(x_back, y_back+1, x_actual, y_actual);
                    }
                }

                if(!cuadros[x_back][y_back][z_actual].getPared('O')) {
                    if(x_back > 0) {
                        TempGrid = coordToGrid(x_back-1, y_back);
                        if(closedList[TempGrid] != 999 and backList[lastBackGrid+1] != TempGrid)
                        Neighbors[3] = pathway(x_back-1, y_back, x_actual, y_actual);
                    }
                }

                for (int i = 0; i<4; i++) {
                    if(Neighbors[i] < NeighborSortValue) {
                        neighbor_index = i;
                        NeighborSortValue = Neighbors[i];
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

                if(firstLoop){
                    backList[back_index] = coordToGrid(x_destino, y_destino);
                    back_index--;
                    x_back = x_lastPath;
                    y_back = y_lastPath;
                    firstLoop = false;
                }

                newGrid = coordToGrid(x_back, y_back);
                backList[back_index] = newGrid;
                lastBackGrid = back_index;

                //Serial.println("BackList = " + String(backList[back_index]));

                /*Serial.print(x_back);
                Serial.print(" ");
                Serial.println(y_back);
                delay(3000);*/
                back_index--;
                //-----------
                if(x_back == x_actual and y_back == y_actual) {
                    backList[back_index+1] = 999;
                    Serial.println("Entre al final");
                    //delay(500);
                    //Dar ordenes de movimiento para llegar
                    backFinished = true;

                    if(ref == 255) {
                        x_actual = x_destino;
                        y_actual = y_destino;

                        for(int i = 0; i<GRID_MAX; i++) {
                            if(backList[i] != 999) {
                                if(gridActual-backList[i] == X_MAX) {
                                    Serial.println("Abajo");
                                    absoluteMove('S');
                                    gridActual -= X_MAX;
                                }

                                if(gridActual-backList[i] == -1) {
                                    Serial.println("Derecha");
                                    absoluteMove('E');
                                    gridActual += 1;
                                }

                                if(gridActual-backList[i] == -X_MAX) {
                                    Serial.println("Arriba");
                                    absoluteMove('N');
                                    gridActual += X_MAX;
                                }

                                if(gridActual-backList[i] == 1) {
                                    Serial.println("Izquierda");
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


void guardarCSR(){
    ArrayCSR = 0;
    for(int x=0; x<X_MAX; x++)
    {
        for(int y=0; y<Y_MAX; y++)
        {
            if(cuadros[x][y][z_actual].getEstado() == SIN_RECORRER)
            {
                x_recorrer[ArrayCSR] = x;
                y_recorrer[ArrayCSR] = y;
                ArrayCSR++;
            }
        }
    }
}

// Detecta paredes y las actualiza en el array cuadros[] en la posicion actual
void checarParedes(){
    shortMove = false;
    A_wall = false; B_wall = false; C_wall = false; D_wall = false;

    switch(iOrientacion) {
        case A_NORTE:
        if(y_actual > 0)
            if(getSharpCorta(SHARP_C) > 15 and (cuadros[x_actual][y_actual-1][z_actual].getEstado()==NO_EXISTE  or
            cuadros[x_actual][y_actual-1][z_actual].getEstado()==SIN_RECORRER)) {
                agregarLast('S');
                shortMove = true;
            } else {
                C_wall = true;
            }

        if(getSharpCorta(SHARP_C) < 15 and SharpRampa)
            cuadros[x_actual][y_actual][z_actual].setPared('S', true);


        if(getSharpCorta(SHARP_B1) > 15 and (cuadros[x_actual+1][y_actual][z_actual].getEstado()==NO_EXISTE  or
        cuadros[x_actual+1][y_actual][z_actual].getEstado()==SIN_RECORRER)) {
            agregarLast('E');
            shortMove = true;
        } else {
            B_wall = true;
        }
        if(getSharpCorta(SHARP_B1) < 15)
            cuadros[x_actual][y_actual][z_actual].setPared('E', true);

        if(getSharpCorta(SHARP_A) > 15 and (cuadros[x_actual][y_actual+1][z_actual].getEstado()==NO_EXISTE  or
        cuadros[x_actual][y_actual+1][z_actual].getEstado()==SIN_RECORRER)) {
            agregarLast('N');
            shortMove = true;
        } else {
            A_wall = true;
        }
        if(getSharpCorta(SHARP_A) < 15)
            cuadros[x_actual][y_actual][z_actual].setPared('N', true);

        if(x_actual > 0)
            if(getSharpCorta(SHARP_D1) > 15 and (cuadros[x_actual-1][y_actual][z_actual].getEstado()==NO_EXISTE  or
            cuadros[x_actual-1][y_actual][z_actual].getEstado()==SIN_RECORRER)) {
                agregarLast('O');
                shortMove = true;
            } else {
                D_wall = true;
            }
        if(getSharpCorta(SHARP_D1) < 15)
            cuadros[x_actual][y_actual][z_actual].setPared('O', true);

        break;
        //--------------------------------------------------------------------
        case B_NORTE:
        if(getSharpCorta(SHARP_C)  > 15 and (cuadros[x_actual+1][y_actual][z_actual].getEstado()==NO_EXISTE  or
        cuadros[x_actual+1][y_actual][z_actual].getEstado()==SIN_RECORRER)) {
            agregarLast('E');
            shortMove = true;
        } else {
            C_wall = true;
        }
        if(getSharpCorta(SHARP_C)  < 15 and SharpRampa)
        cuadros[x_actual][y_actual][z_actual].setPared('E', true);

        if(getSharpCorta(SHARP_B1)  > 15 and (cuadros[x_actual][y_actual+1][z_actual].getEstado()==NO_EXISTE  or
        cuadros[x_actual][y_actual+1][z_actual].getEstado()==SIN_RECORRER)) {
            agregarLast('N');
            shortMove = true;
        } else {
            B_wall = true;
        }
        if(getSharpCorta(SHARP_B1) < 15)
        cuadros[x_actual][y_actual][z_actual].setPared('N', true);


        if(x_actual > 0)
            if(getSharpCorta(SHARP_A)  > 15 and (cuadros[x_actual-1][y_actual][z_actual].getEstado()==NO_EXISTE  or
            cuadros[x_actual-1][y_actual][z_actual].getEstado()==SIN_RECORRER)) {
                agregarLast('O');
                shortMove = true;
            } else {
                A_wall = true;
            }
        if(getSharpCorta(SHARP_A) < 15)
            cuadros[x_actual][y_actual][z_actual].setPared('O', true);

        if(y_actual > 0)
            if(getSharpCorta(SHARP_D1)  > 15 and (cuadros[x_actual][y_actual-1][z_actual].getEstado()==NO_EXISTE  or
            cuadros[x_actual][y_actual-1][z_actual].getEstado()==SIN_RECORRER)) {
            agregarLast('S');
            shortMove = true;
        } else {
            D_wall = true;
        }
        if(getSharpCorta(SHARP_D1)  < 15)
            cuadros[x_actual][y_actual][z_actual].setPared('S', true);

        break;
        //--------------------------------------------------------------------
        case C_NORTE:
        if(getSharpCorta(SHARP_C) > 15 and (cuadros[x_actual][y_actual+1][z_actual].getEstado()==NO_EXISTE  or
        cuadros[x_actual][y_actual+1][z_actual].getEstado()==SIN_RECORRER)) {
            agregarLast('N');
            shortMove = true;
        } else {
            C_wall = true;
        }
        if(getSharpCorta(SHARP_C) < 15 and SharpRampa)
            cuadros[x_actual][y_actual][z_actual].setPared('N', true);

        if(x_actual > 0)
            if(getSharpCorta(SHARP_B1) > 15 and (cuadros[x_actual-1][y_actual][z_actual].getEstado()==NO_EXISTE  or
            cuadros[x_actual-1][y_actual][z_actual].getEstado()==SIN_RECORRER)) {
            agregarLast('O');
            shortMove = true;
        } else {
            B_wall = true;
        }
        if(getSharpCorta(SHARP_B1) < 15)
        cuadros[x_actual][y_actual][z_actual].setPared('O', true);

        if(y_actual > 0)
            if(getSharpCorta(SHARP_A) > 15 and (cuadros[x_actual][y_actual-1][z_actual].getEstado()==NO_EXISTE  or
            cuadros[x_actual][y_actual-1][z_actual].getEstado()==SIN_RECORRER)) {
            agregarLast('S');
            shortMove = true;
        } else {
            A_wall = true;
        }
        if(getSharpCorta(SHARP_A) < 15)
            cuadros[x_actual][y_actual][z_actual].setPared('S', true);

        if(getSharpCorta(SHARP_D1) > 15 and (cuadros[x_actual+1][y_actual][z_actual].getEstado()==NO_EXISTE  or
        cuadros[x_actual+1][y_actual][z_actual].getEstado()==SIN_RECORRER)) {
            agregarLast('E');
            shortMove = true;
        } else {
            D_wall = true;
        }
        if(getSharpCorta(SHARP_D1) < 15)
            cuadros[x_actual][y_actual][z_actual].setPared('E', true);

        break;
        //--------------------------------------------------------------------
        case D_NORTE:
        if(x_actual > 0)
            if(getSharpCorta(SHARP_C) > 15 and (cuadros[x_actual-1][y_actual][z_actual].getEstado()==NO_EXISTE  or
            cuadros[x_actual-1][y_actual][z_actual].getEstado()==SIN_RECORRER)) {
                agregarLast('O');
                shortMove = true;
            } else {
                C_wall = true;
            }
        if(getSharpCorta(SHARP_C) < 15 and SharpRampa)
            cuadros[x_actual][y_actual][z_actual].setPared('O', true);


        if(y_actual > 0)
            if(getSharpCorta(SHARP_B1) > 15 and (cuadros[x_actual][y_actual-1][z_actual].getEstado()==NO_EXISTE  or
            cuadros[x_actual][y_actual-1][z_actual].getEstado()==SIN_RECORRER)) {
                agregarLast('S');
                shortMove = true;
            } else {
                B_wall = true;
        }

        if(getSharpCorta(SHARP_B1) < 15)
            cuadros[x_actual][y_actual][z_actual].setPared('S', true);

        if(getSharpCorta(SHARP_A) > 15 and (cuadros[x_actual+1][y_actual][z_actual].getEstado()==NO_EXISTE  or
        cuadros[x_actual+1][y_actual][z_actual].getEstado()==SIN_RECORRER)) {
            agregarLast('E');
            shortMove = true;
        } else {
            A_wall = true;
        }

        if(getSharpCorta(SHARP_A) < 15)
            cuadros[x_actual][y_actual][z_actual].setPared('E', true);


        if(getSharpCorta(SHARP_D1) > 15 and (cuadros[x_actual][y_actual+1][z_actual].getEstado()==NO_EXISTE  or
        cuadros[x_actual][y_actual+1][z_actual].getEstado()==SIN_RECORRER)) {
            agregarLast('N');
            shortMove = true;
        } else {
            D_wall = true;
        }
        if(getSharpCorta(SHARP_D1) < 15)
            cuadros[x_actual][y_actual][z_actual].setPared('N', true);
        break;
    }

    SharpRampa = true;
}


void checarLasts(){
    if(x_last2 != 255 and y_last2 != 255) {
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

    if(x_actual == x_last and y_actual == y_last) {
        x_last = 255;
        y_last = 255;
        Last = false;
    }
}


//Verificar si en el for i es i>1 o i>0 (probar)
void recorrerX(){
    Serial.println("Recorrer X");
    for(int k=0; k<Z_MAX; k++){
        for(int j=0; j<Y_MAX; j++) {
            for(int i=X_MAX-1; i>0; i--) {
                cuadros[i][j][k].setEstado(cuadros[i-1][j][k].getEstado());
                cuadros[i][j][k].setPared('N', cuadros[i-1][j][k].getPared('N'));
                cuadros[i][j][k].setPared('E', cuadros[i-1][j][k].getPared('E'));
                cuadros[i][j][k].setPared('S', cuadros[i-1][j][k].getPared('S'));
                cuadros[i][j][k].setPared('O', cuadros[i-1][j][k].getPared('O'));
                cuadros[i][j][k].setMlx(cuadros[i-1][j][k].getMlx());
            }
            cuadros[0][j][k].setEstado(NO_EXISTE);
            cuadros[0][j][k].setPared('N', false);
            cuadros[0][j][k].setPared('E', false);
            cuadros[0][j][k].setPared('S', false);
            cuadros[0][j][k].setPared('O', false);
            cuadros[0][j][k].setMlx(false);
        }
    }

    x_actual++;
    x_inicio++;
    x_InicioB++;
    x_InicioC++;

    if(x_last != 255 and y_last != 255)
        x_last++;

    if(x_last2 != 255 and y_last2 != 255)
        x_last2++;
    //boolRecorrerX = true;
}

void recorrerY(){
    Serial.println("Recorrer Y");
    for(int k=0; k<Z_MAX; k++){
        for(int j=0; j<X_MAX; j++) {
            for(int i=Y_MAX-1; i>0; i--){
                cuadros[j][i][k].setEstado(cuadros[j][i-1][k].getEstado());
                cuadros[j][i][k].setPared('N', cuadros[j][i-1][k].getPared('N'));
                cuadros[j][i][k].setPared('E', cuadros[j][i-1][k].getPared('E'));
                cuadros[j][i][k].setPared('S', cuadros[j][i-1][k].getPared('S'));
                cuadros[j][i][k].setPared('O', cuadros[j][i-1][k].getPared('O'));
                cuadros[j][i][k].setMlx(cuadros[i-1][j][k].getMlx());
            }
            cuadros[j][0][k].setEstado(NO_EXISTE);
            cuadros[j][0][k].setPared('N', false);
            cuadros[j][0][k].setPared('E', false);
            cuadros[j][0][k].setPared('S', false);
            cuadros[j][0][k].setPared('O', false);
            cuadros[j][0][k].setMlx(false);
        }
    }

    y_actual++;
    y_inicio++;
    y_InicioB++;
    y_InicioC++;

    if(x_last != 255 and y_last != 255)
        y_last++;

    if(x_last2 != 255 and y_last2 != 255)
        y_last2++;
    //boolRecorrerY = true;
}

void gotoInicio(byte x_final, byte y_final){

    byte nearInicio = pathway(x_actual, y_actual, x_final, y_final);
    bool boolInicio = false;

    if(nearInicio == 1) {
        Serial.println("Entre a nearInicio");
        Serial.println("Inicio " + String(x_final) + "," + String(y_final));
        Serial.println("Actual " + String(x_actual) + "," + String(y_actual));
        int xInicio = x_final - x_actual;
        int yInicio = y_final - y_actual;

        if(xInicio == 1) {
            Serial.println("Inicio E");
            if(!cuadros[x_actual][y_actual][z_actual].getPared('E')) {
                absoluteMove('E');
                boolInicio = true;
            }
        }
        else if(xInicio == -1) {
            Serial.println("Inicio O");
            if(!cuadros[x_actual][y_actual][z_actual].getPared('O')) {
                absoluteMove('O');
                boolInicio = true;
            }
        }
        else if(yInicio == 1) {
            Serial.println("Inicio N");
            if(!cuadros[x_actual][y_actual][z_actual].getPared('N')) {
                absoluteMove('N');
                boolInicio = true;
            }
        }
        else if(yInicio == -1) {
            Serial.println("Inicio S");
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

void RampaMoveX(){
    switch(RampaLastMove)
    {
        case TO_NORTH:
        y_actual++;
        break;

        case TO_EAST:
        x_actual++;
        break;

        case TO_SOUTH:
        y_actual--;
        break;

        case TO_WEST:
        x_actual--;
        break;
    }
}

void resolverLaberinto(){
    if(shortMove) {
        Serial.println("ENTRE AL SHORTMOVE");
        if(!D_wall) {
            Serial.println("Short Izquierda");
            switch(iOrientacion) {
                case A_NORTE:
                x_actual--;
                LastMove = TO_WEST;
                break;

                case B_NORTE:
                y_actual--;
                LastMove = TO_SOUTH;
                break;

                case C_NORTE:
                x_actual++;
                LastMove = TO_EAST;
                break;

                case D_NORTE:
                y_actual++;
                LastMove = TO_NORTH;
                break;
            }
            vueltaIzq();
            alinear();
            moverCuadro();
            checarLasts();
        } else if(!A_wall) {
            Serial.println("Short Frente");
            switch(iOrientacion) {
                case A_NORTE:
                y_actual++;
                LastMove = TO_NORTH;
                break;

                case B_NORTE:
                x_actual--;
                LastMove = TO_WEST;
                break;

                case C_NORTE:
                y_actual--;
                LastMove = TO_SOUTH;
                break;

                case D_NORTE:
                x_actual++;
                LastMove = TO_EAST;
                break;
            }
            moverCuadro();
            checarLasts();
        } else if(!B_wall) {
            Serial.println("Short Derecha");
            switch(iOrientacion) {
                case A_NORTE:
                x_actual++;
                LastMove = TO_EAST;
                break;

                case B_NORTE:
                y_actual++;
                LastMove = TO_NORTH;
                break;

                case C_NORTE:
                x_actual--;
                LastMove = TO_WEST;
                break;

                case D_NORTE:
                y_actual--;
                LastMove = TO_SOUTH;
                break;
            }
            vueltaDer();
            moverCuadro();
            checarLasts();
        } else {
            Serial.println("Short Atras");
            switch(iOrientacion) {
                case A_NORTE:
                y_actual--;
                LastMove = TO_SOUTH;
                break;

                case B_NORTE:
                x_actual++;
                LastMove = TO_EAST;
                break;

                case C_NORTE:
                y_actual++;
                LastMove = TO_NORTH;
                break;

                case D_NORTE:
                x_actual--;
                LastMove = TO_WEST;
                break;
            }
            vueltaAtras();
            moverCuadro();
            checarLasts();
        }
    } else {
        if(Last) {
            Serial.println("LAST");
            lcd.clear();
            lcd.print("GOTO LAST");
            /*Serial.print(x_last);
            Serial.print(" | ");
            Serial.println(y_last);*/
            /*for(int x=0; x<X_MAX; x++)
            {
            for(int y=0; y<Y_MAX; y++)
            {
            Serial.print(y);
            Serial.print(" , ");
            Serial.print(x);
            Serial.print(" = ");
            Serial.println(cuadros[x][y][z_actual].getEstado());
                }
            }
            Serial.println("Ya");*/
            byte var = 255;
            Pathfinding(x_last, y_last, var);

            x_last = 255;
            y_last = 255;
            Last = false;
            //gotoLast
        } else {
            Serial.println("GOTO-SR");
            lcd.clear();
            lcd.print("GOTO SR");
            int LowestCSR = 999;
            int iCSR;

            guardarCSR();
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
                Serial.println("GOTO-INICIO");
                lcd.clear();

                if(z_actual == 2)
                {
                    lcd.print("GOTO INICIO C");
                    delay(400);
                    Piso3 = true;
                    gotoInicio(x_InicioC, y_InicioC);
                    //RampaMoveX();
                }
                else
                if(z_actual == 1)
                {
                    lcd.print("GOTO INICIO B");
                    delay(400);
                    Piso2 = true;
                    gotoInicio(x_InicioB, y_InicioB);
                    //RampaMoveX();
                }
                else
                if(!Piso2 and x_InicioB != 255)
                {
                    lcd.print("GOTO INICIO C");
                    delay(400);
                    gotoInicio(x_InicioC, y_InicioC);
                    //RampaMoveX();
                }
                else
                {
                    lcd.print("GOTO INICIO");
                    delay(400);
                    gotoInicio(x_inicio, y_inicio);

                    Serial.println("MARI PUTISIMO, YA LLEGUE");
                    lcd.print("  LLEGUE");
                    delay(20000);
                }
            }
            //gotoSR
        }
    }
}


//******************************************
//******************************************
//--------------SERVO MOTOR-----------------
//******************************************
//******************************************


//Gira el servo 180 grados
void servoMotor() {
    if(servo.read() == 0)
        servo.write(180);
    else
        servo.write(0);
}

void checarMlx(){
  if(!cuadros[x_actual][y_actual][z_actual].getMlx())
  {
    servoMotor();
    cuadros[x_actual][y_actual][z_actual].setMlx(true);
  }
}


//******************************************
//---------------TCS3200------------------

/* Cambia la frecuencia del sensor:
*  0 = filtro apagado
*  2 = filtro del 2%
*  20 = filtro del 20%
*  100 = filtro del 100%
*/
void setFrecuencia(byte frecuencia){
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
void setFiltro(char filtro){
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
int getColor(){
    return (pulseIn(sensorOut, LOW));
}

// Funcion para calibrar los colores que se usaran
void calibrarColor(){
    while(EstadoColor == ESTADO_OFF) {
        Serial.println("Calibrar...");
        lcd.setCursor(0, 0);
        lcd.print("   Calibrar...");
        BotonColor = digitalRead(BOTON_COLOR);
        if(BotonColor == 1) {
            //*Limpia la pantalla
            EstadoColor = ESTADO_NEGRO;
            delay(1000);
        }
    }

    lcd.clear();
    lcd.print("    Negro...");
    while(EstadoColor == ESTADO_NEGRO) {
        Serial.println("Calibrar Negro");
        BotonColor = digitalRead(BOTON_COLOR);
        if(BotonColor == 1) {
            setFiltro('N');
            iN_NEGRO = getColor();

            setFiltro('R');
            iR_NEGRO = getColor();

            setFiltro('G');
            iV_NEGRO = getColor();

            setFiltro('B');
            iA_NEGRO = getColor();

            //*Limpia la pantalla
            delay(1000);
            EstadoColor = ESTADO_CHECKPOINT;
        }
    }

    lcd.clear();
    lcd.print("  Checkpoint...");
    while(EstadoColor == ESTADO_CHECKPOINT) {
        Serial.println("Calibrar Checkpoint");
        BotonColor = digitalRead(BOTON_COLOR);
        if(BotonColor == 1) {
            setFiltro('N');
            iN_CHECKPOINT = getColor();

            setFiltro('R');
            iR_CHECKPOINT = getColor();

            setFiltro('G');
            iV_CHECKPOINT = getColor();

            setFiltro('B');
            iA_CHECKPOINT = getColor();

            //*Limpia la pantalla
            EstadoColor = ESTADO_LISTO;
            delay(1000);
        }
    }

    lcd.clear();
    lcd.print("   LISTO... ");
    while(EstadoColor == ESTADO_LISTO) {
        Serial.println("Listo...");
        BotonColor = digitalRead(BOTON_COLOR);
        if(BotonColor == 1) {
            //*Limpia la pantalla
            EstadoColor++;
            //*Pasa al siguiente estado
        }
    }
}

// Regresa true si el sensor detecta el color que declares en el parametro
// respetando el margen elegido
bool checarCuadroColor(byte cuadro, byte margen) {
    setFiltro('N');
    iNone = getColor();

    setFiltro('R');
    iRojo = getColor();

    setFiltro('G');
    iVerde = getColor();

    setFiltro('B');
    iAzul = getColor();

    switch (cuadro) {
        case COLOR_NEGRO:
        if(iNone <= iN_NEGRO+margen and iNone >= iN_NEGRO-margen and iRojo <= iR_NEGRO+margen and iRojo >= iR_NEGRO-margen
        and iVerde <= iV_NEGRO+margen and iVerde >= iV_NEGRO-margen and iAzul <= iA_NEGRO+margen and iAzul >= iA_NEGRO-margen)
            return true;
        else
            return false;
        break;

        case COLOR_CHECKPOINT:
        if(iNone <= iN_CHECKPOINT+margen and iNone >= iN_CHECKPOINT-margen and iRojo <= iR_CHECKPOINT+margen and iRojo >= iR_CHECKPOINT-margen
        and iVerde <= iV_CHECKPOINT+margen and iVerde >= iV_CHECKPOINT-margen and iAzul <= iA_CHECKPOINT+margen and iAzul >= iA_CHECKPOINT-margen)
            return true;
        else
            return false;
        break;

        default:
        return false;
    }
}


void checarColor(){

    if(checarCuadroColor(COLOR_NEGRO, 80))
    {
        lcd.setCursor(0, 0);
        lcd.print("NEGRO DETECTADO!");
        cuadros[x_actual+1][y_actual][z_actual].setPared('O', true);
        cuadros[x_actual-1][y_actual][z_actual].setPared('E', true);
        cuadros[x_actual][y_actual+1][z_actual].setPared('S', true);
        cuadros[x_actual][y_actual-1][z_actual].setPared('N', true);
        cuadros[x_actual][y_actual][z_actual].setEstado(NEGRO);
        reversaCuadro();
        switch (iOrientacion) {
            case A_NORTE:
            y_actual--;
            break;

            case B_NORTE:
            x_actual++;
            break;

            case C_NORTE:
            y_actual++;
            break;

            case D_NORTE:
            x_actual--;
            break;
        }
        A_wall = true;
    }

/*
    if(checarCuadroColor(CHECKPOINT, 20))
    {
      int TotalGrid;
      for(int z=0; z<Z_MAX; z++)
      {
        for(int y = 0; y < Y_MAX; y++)
        {
          for(int x = 0; x < X_MAX; x++)
          {
            TotalGrid = totalCoordToGrid(x, y, z);

            if(cuadros[x][y][z].getMlx())
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
void checarArray(){

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


void LackOfProgress(){

  for(int i=0; i<TOTAL_GRID_MAX; i++){

    int Num, Div;
    int list[5];
    byte Count = 0;

    byte x = totalGridToCoord(i, 'x');
    byte y = totalGridToCoord(i, 'y');
    byte z = totalGridToCoord(i, 'z');

    Num = checkList1[i];

    while(Num != 0)
    {
      Div = Num / 2;
      list[Count] = Num % 2;
      Num = Div;
      Count++;
    }

    if(list[4])
    cuadros[x][y][z].setMlx(true);
    else
    cuadros[x][y][z].setMlx(false);

    if(list[3])
    cuadros[x][y][z].setPared('S', true);
    else
    cuadros[x][y][z].setPared('S', false);

    if(list[2])
    cuadros[x][y][z].setPared('E', true);
    else
    cuadros[x][y][z].setPared('E', false);

    if(list[1])
    cuadros[x][y][z].setPared('N', true);
    else
    cuadros[x][y][z].setPared('N', false);

    if(list[0])
    cuadros[x][y][z].setPared('O', true);
    else
    cuadros[x][y][z].setPared('O', false);

    switch(checkList2[i])
    {
      case NO_EXISTE:
      cuadros[x][y][z].setEstado(NO_EXISTE);
      break;

      case SIN_RECORRER:
      cuadros[x][y][z].setEstado(SIN_RECORRER);
      break;

      case RECORRIDO:
      cuadros[x][y][z].setEstado(RECORRIDO);
      break;

      case CHECKPOINT:
      cuadros[x][y][z].setEstado(CHECKPOINT);
      break;

      case NEGRO:
      cuadros[x][y][z].setEstado(NEGRO);
      break;

      case RAMPA:
      cuadros[x][y][z].setEstado(RAMPA);
      break;

      case INICIO:
      cuadros[x][y][z].setEstado(INICIO);
      break;

    }
  }
}


void setup() {
    delay(1000);
    Serial.begin(9600);
    PORTC = (1 << PORTC4) | (1 << PORTC5);    // Habilita ‘pullups’.
    pinMode(InterruptNano, INPUT_PULLUP);  //Pone el pin de interrupcion a la escucha
    pinMode(InterruptBoton, INPUT_PULLUP);  //Pone el pin de interrupcion a la escucha
    attachInterrupt(digitalPinToInterrupt(InterruptNano), funcionB, LOW); //Declara la funcion a ejecutar en interruptB
    attachInterrupt(digitalPinToInterrupt(InterruptBoton), funcionD, LOW); //Declara la funcion a ejectura en interruptD
    attachInterrupt(digitalPinToInterrupt(ENC1), addStep, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2), addStep, CHANGE);
    servo.attach(servoPin);      //Pin PWM a donde estará conectado el servo
    setFrecuencia(20);           //Establece la frecuencia del TCS3200
    pinMode(sensorOut, INPUT);   //Inicializa el pin que recibira la informacion del TCS3200
    pinMode(S0, OUTPUT);         //Establece  pin de Salida
    pinMode(S1, OUTPUT);         //Establece  pin de Salida
    pinMode(S2, OUTPUT);         //Establece  pin de Salida
    pinMode(S3, OUTPUT);         //Establece  pin de Salida
    if(!bno.begin())
           Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    bno.setExtCrystalUse(true);

    AFMS.begin();
    MotorAI -> run(RELEASE);
    MotorAD -> run(RELEASE);
    MotorCI -> run(RELEASE);
    MotorCD -> run(RELEASE);
    velocidad(VEL_MOTOR, VEL_MOTOR, VEL_MOTOR, VEL_MOTOR);

    lcd.begin();
    lcd.backlight();
    lcd.print("   ROBORREGOS");
    lcd.setCursor(0, 1);
    lcd.print("     E V A");

    delay(100);
    izqPID.SetMode(AUTOMATIC);
    derPID.SetMode(AUTOMATIC);
    ramPID.SetMode(AUTOMATIC);
    setIzq = 0;
    setDer = 0;
    if(getAngulo() > 300)
        inIzq = - (360 - getAngulo());
    else
        inIzq = getAngulo();
    if(getAngulo() > 300)
        inDer = - (360 - getAngulo());
    else
        inDer = getAngulo();

    // Inicializa toda la matriz de checkpoint en 0
    for(int i = 0; i<GRID_MAX; i++) {
        checkList1[i] = 0;
        checkList2[i] = 0;
    }


    x_inicio = 1; y_inicio = 1; z_inicio = 0;
    x_actual = 1; y_actual = 1; z_actual = 0;
    cuadros[x_actual][y_actual][z_actual].setEstado(INICIO);

    pinMode(13, OUTPUT);
    pinMode(6, INPUT);
    delay(500);
    lcd.clear();
    //calibrarColor();
    lcd.clear();
}

void loop() {
    lcd.clear();
    if(cuadros[x_actual][y_actual][z_actual].getEstado() != INICIO)
       cuadros[x_actual][y_actual][z_actual].setEstado(RECORRIDO);
   checarArray();
   lcd.setCursor(0,1);
   lcd.print(String(x_actual) + "," + String(y_actual) + "," + String(z_actual));
   delay(500);
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
   checarColor();
}

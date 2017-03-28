#include <PixyUART.h>
#include <Arduino.h>
#include <i2cmaster.h>


PixyUART pixy;
uint16_t blocks;

#define pinReset 13
#define pinListener 9

#define visualLed 11
#define heatLed 10
#define visualDefiner 12
#define Pin_Interrupt 2
#define heatDefiner 3

// Pin_Define = 0 ---> Derecho
// Pin_Define = 1 ---> Izquierdo

// Victimas visuales
bool tomarLecturas;
int x_min = 80;
int x_max = 240;

// Victimas de Calor
bool inFire = false;
int MLX_B = 0x5C<<1; // Sensor mlx derecho
//int MLX_D = 0x5A<<1; // Sensor mlx izquierdo
int iCounter = 0;
float temp_ambienteB, temp_ambienteD;  // Temperatura del ambiente


//-------------------------------------------
//----------- FUNCIONES DE CALOR ------------

// Funcion para calcular la temperatura
float temperature(int address) {
  int dev = address;
  int data_low = 0;
  int data_high = 0;
  int pec = 0;

  // Escribe
  i2c_start_wait(dev+I2C_WRITE);
  i2c_write(0x07);

  // Lee
  i2c_rep_start(dev+I2C_READ);
  data_low = i2c_readAck();       // Lee 1 byte y envía ack.
  data_high = i2c_readAck();      // Lee 1 byte y envía ack
  pec = i2c_readNak();
  i2c_stop();


  // Esto convierte los bytes altos y bajos juntos y procesa la temperatura.
  double tempFactor = 0.02;       // 0.02 grados por LSB (medida de
                                  // resolución del MLX90614).
  double tempData = 0x0000;
  int frac;                       // Datos después del punto decimal.

  // Esto oculta el error del byte alto y lo mueve a la izquierda
  // 8 bits y agrega el byte bajo.
  tempData = (double)(((data_high & 0x007F) << 8) + data_low);
  tempData = (tempData * tempFactor)-0.01;
  float celcius = tempData - 273.15;

  // Retorna la temperatura en Celcius.
  return celcius;
}


// Regresa el valor del mlx izquierdo(D) o derecho(B)
float getTemperature(char cLado){

  switch (cLado)
  {
    case 'B':
    return temperature(MLX_B);
    break;

    /*case 'D':
    return temperature(MLX_D);
    break;*/

    default:
    return -1;
  }
}


// Guarda la temperatura ambiente de cada mlx
void guardarAmbiente(){
  temp_ambienteB = getTemperature('B');

  //temp_ambienteD = getTemperature('D');
}

// Te dice si la temperatura de algun mlx es mayor que la del ambiente
void checarTemperatura(byte grados){
  while((getTemperature('B') - temp_ambienteB) > grados)
  {
      if(iCounter < 1)
      {
       //Serial.println("DERECHO");
       digitalWrite(heatDefiner, LOW);
       digitalWrite(visualDefiner, LOW);
       delay(200);
       digitalWrite(Pin_Interrupt, LOW);
       digitalWrite(heatLed, LOW);
       delay(200);
      iCounter++;
      }

      digitalWrite(visualDefiner, LOW);
      digitalWrite(heatDefiner, LOW);
      digitalWrite(heatLed, HIGH);
      digitalWrite(Pin_Interrupt, HIGH);
  }
  /*while((getTemperature('D') - temp_ambienteD) > grados)
  {
      if(iCounter < 1)
      {
       //Serial.println("IZQUIERDO");
       digitalWrite(visualDefiner, LOW);
       digitalWrite(heatDefiner, HIGH);
       delay(200);
       digitalWrite(Pin_Interrupt, LOW);
       digitalWrite(heatLed, LOW);
       delay(200);
        iCounter++;
      }

      digitalWrite(visualDefiner, LOW);
      digitalWrite(heatDefiner, HIGH);
      digitalWrite(heatLed, HIGH);
      digitalWrite(Pin_Interrupt, HIGH);
  }*/
}



//-------------------------------------------
//----------- FUNCIONES VISUALES ------------

void identificarLetra(){
  int x_block[30];
  int y_block[30];
  int width_block[30];
  int height_block[30];
  byte lecturas[10];
  int contador = 0;

// Hace 5 lecturas de bloques y los almacena en 5 partes diferentes
  while(contador < 30)
  {
    blocks = pixy.getBlocks();
    if(blocks != 0)
    {
      for(int i=0; i<blocks; i++)
      {
       x_block[contador + i] = pixy.blocks[i].x;
       y_block[contador + i] = pixy.blocks[i].y;
       width_block[contador + i] = pixy.blocks[i].width;
       height_block[contador + i] = pixy.blocks[i].height;
      }
      contador += 3;
      lecturas[(contador-1)/3] = blocks;
    }

  }
// Elige la lectura con mayor numero de bloques detectados
  byte goodReads = 0;
  byte goodBlock = 0;
  for(int i=0; i<5; i++){
    if(lecturas[i] > goodReads)
    {
      goodReads = lecturas[i];
      goodBlock = i;
    }
  }

  for(int i=0; i<goodReads; i++)
  {
    Serial.print("X: ");
    Serial.println(x_block[goodBlock*3 + i]);
    Serial.print("Y: ");
    Serial.println(y_block[goodBlock*3 + i]);
    Serial.print("Width: ");
    Serial.println(width_block[goodBlock*3 + i]);
    Serial.print("Height: ");
    Serial.println(height_block[goodBlock*3 + i]);
  }

  delay(5000);
}


// Filtra los valores solo para cuando los bloques detectados esten en el centro
void leerBlocks(){

  blocks = pixy.getBlocks();
  //Serial.println(blocks);
  int x_media[3];

  tomarLecturas = false;

  if(blocks)
  {
    for(int i = 0; i<blocks; i++)
    {
      x_media[i] = pixy.blocks[i].x + (pixy.blocks[i].width / 2);
    }

    tomarLecturas = true;

    for(int i = 0; i<blocks; i++)
    {
      if(x_media[i] > x_max || x_media[i] < x_min)
        tomarLecturas = false;
    }

  }


}

void checarVision(){
  do
  {
    leerBlocks();
    if(tomarLecturas && iCounter < 1)
      {
       //Serial.println("VISUAL KIT");
       digitalWrite(visualDefiner, HIGH);
       digitalWrite(heatDefiner, LOW);
       delay(200);
       digitalWrite(Pin_Interrupt, LOW);
       digitalWrite(visualLed, HIGH);
       delay(200);
      iCounter++;
      }

      digitalWrite(visualDefiner, HIGH);
      digitalWrite(heatDefiner, LOW);
      digitalWrite(visualLed, LOW);
      digitalWrite(Pin_Interrupt, HIGH);
  }
  while(tomarLecturas);
}

void setup() {
  //Serial.begin(9600);
  pinMode(pinReset, OUTPUT);
  digitalWrite(pinReset, HIGH);
  pinMode(pinListener, INPUT);

// Visual
  pinMode(visualLed, OUTPUT);
  pinMode(visualDefiner, OUTPUT);
  pixy.init();

// Calor
  i2c_init();
  PORTC = (1 << PORTC4) | (1 << PORTC5);
  pinMode(Pin_Interrupt, OUTPUT);
  pinMode(heatDefiner, OUTPUT);
  pinMode(heatLed, OUTPUT);
  digitalWrite(Pin_Interrupt, HIGH);
  delay(500);

  guardarAmbiente();
}

void loop() {

  digitalWrite(Pin_Interrupt, HIGH);
  digitalWrite(heatLed, HIGH);
  digitalWrite(visualLed, LOW);

  if(digitalRead(pinListener))
  {
    digitalWrite(pinReset, HIGH);
    delay(100);
    digitalWrite(pinReset, LOW);
  }

  checarTemperatura(2);
  checarVision();
  iCounter = 0;
}

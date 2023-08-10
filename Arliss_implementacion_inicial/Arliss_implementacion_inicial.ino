//################ Iniciar librerias ###################//

///// Librerias de GPS

  #include <TinyGPS++.h>
  #include <SoftwareSerial.h>
  SoftwareSerial gpsSerial(10,11); //RX = 10; TX = 11
  TinyGPSPlus gps; // Declaramos el GPS
///// Librerias de brujula

  #include <Wire.h>
// init para el cmps12
///********** TODO: Revisar esta parte porque van a utilizar el LSM303 no el cmps12
  #define CMPS12_ADDRESS 0x60  // Address of CMPS12 shifted right one bit for arduino wire library
  #define ANGLE_8  1           // Register to read 8bit angle from

  #include <LSM303.h>
  LSM303 compass;
//// Libreria de barometro

  #include "SparkFunMPL3115A2.h"
  MPL3115A2 myPressure;
//// Libreria de servo

  #include <Servo.h>
  Servo servo_paracaidas;
//################# Variables Globales ######################  ///
// Todas las variables que se van a ocupar atravez de la mision

//////#/#/#/#/  Parametros de mision () /#/#/#/#/#/#
float rango_altura_activacion = 100; //(En metros) A que altura sobre la altura inicial quieres que el robot detecte que esta subiendo
int fase_de_inicio = 0; // La fase en la que va a iniciar el robot
float tolerancia_de_meta = 10; // Radio de tolerancia en metros de que tan cerca consideramos que el robot ya llego a su meta (necesario cuando el GPS no tiene buena resolucion)
bool DEBUG = true; //Determinar si queremos saber todos las salidas
float gps_latitud_meta = 0; // latitud meta
float gps_longitud_meta = 0; // longitud meta
float velocidad_descenso_activacion = -.5; // metro por segundo 
float velocidad_aterrizage_tolerancia = .2 // metros por segundo a que rango +/- determinamos que estamos en el piso
///// Parametros del robot (navegacion, control, actuacion, sensor)
float kp = 1.5; // factor proporcional del control del motor (que tan agresivos queremos acercarnos a el heading correcto)
int pos_servo_cerrado = 20; // posicion en la que el servo esta cerrado
int pos_servo_abierto = 180; // posicion en la que el servo esta abierto para soltar el paracaidas
unsigned long periodo_barometro = 200; // en millis
unsigned long periodo_gps = 1000;  // en millis
unsigned long periodo_brujula = 100;  // en millis
unsigned long timer_barometro = 0; // en millis
unsigned long timer_gps = 0;  // en millis
unsigned long timer_brujula = 0;  // en millis
///// Pines del robot
int pin_motor_izquierda = 6; // Pin conectado al mosfet de la izquierda (controla el motor, actuar con salida pwm analog output)
int pin_motor_derecha = 5; // Pin conectado al mosfet de la derecha (controla el motor, actuar con salida pwm analog output)


///// Estado del robot (no modificar directamente estos variables)
int fase_actual = 0; // fase actual del robot
float velocidad_vertical_actual = 0; // metros por segundo
float altura_actual = 0; // metros
float altura_pasada = 0; // metros (se usa esto para guardar una altura anterior y determinar la velocidad)
float heading_actual = 0; // grados sobrel el horizonte
float gps_latitud_actual = 0; // latitud
float gps_longitud_actual = 0; // longitud
float altura_inicial = 0; // Una captura de la altura en metros del robot al inicio (Idealmente que sea cerca al piso)


///// variables helpers (variables que ayudan!)
bool timer_activado_fase0 = false;
unsigned long tiempo_fase0 = 0;

bool timer_activado_fase1 = false;
unsigned long tiempo_fase1 = 0;

bool timer_activado_fase2 = false;
unsigned long tiempo_fase2 = 0;

bool timer_activado_fase3 = false;
unsigned long tiempo_fase3 = 0;

////// variables de prueba (temporales para prueba)
// heading deseado (Esta variable se consigue haciendo el calculo de orientacion de nuestra meta al robot)
float goal_heading = 20;

void setup() {
 
///// Setup: Iniciar sensores y comunicacion
// iniciar comm i2c para sensores
  Wire.begin();
//iniciar comm serial
  Serial.begin(9600);  // Start serial port
//iniciar barometro
  myPressure.begin(); // Get sensor online
  //Configure the sensor
  myPressure.setModeAltimeter(); // Measure altitude above sea level in meters
  //myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags
//iniciar gps

  gpsSerial.begin(9600);
//iniciar brujula

  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){ -573,   -412,  -1220};
  compass.m_max = (LSM303::vector<int16_t>){ +363,   +422,   -337};

///// Setup: Iniciar outputs (motores y servos)
//Definir los outputs para el motor
  pinMode(pin_motor_derecha, OUTPUT);  // definir output para motor de la derecha
  pinMode(pin_motor_izquierda, OUTPUT);  // definir output para motor de la derecha

//Definir los outputs para el servo

  servo_paracaidas.attach();
  servo_paracaidas.write();
////// Setup: Actualizar los estados y lectura de sensor

  delay(5000); // dormir 5 segundos para permitir que los sensores se estabilicen

fase_actual = fase_de_inicio; // declarar la fase en la que vamos a iniciar (con el parametro de mision fase_de_inicio)

/// TODO: Utilizar barometro para determinar la altura
  altura_inicial = gps.altitude.meters(); /// TODO


  velocidad_vertical_actual = 0; 
/// TODO: Utilizar barometro para determinar la altura
  altura_actual = gps.altitude.meters(); /// TODO
  heading_actual = compass.heading();
  gps_latitud_actual = gps.location.lat();
  gps_longitud_actual = gps.location.lng();

if(DEBUG){
  /// TODO: Verficiar las lecturas
  Serial.println("-------------Lectura inicial------------");
  Serial.print("altura_inicial: ");
  Serial.print(altura_inicial);
  Serial.print("velocidad_vertical_actual: ");
  Serial.print(velocidad_vertical_actual);
  Serial.print("heading_actual: ");
  Serial.print(heading_actual);
  Serial.print("gps_latitud_actual: ");
  Serial.print(gps_latitud_actual);
  Serial.print("gps_longitud_actual: ");
  Serial.print(gps_longitud_actual);
}

////// Iniciar timers de los sensores
unsigned long timer_barometro = millis(); // en millis
unsigned long timer_gps = millis();  // en millis
unsigned long timer_brujula = millis();  // en millis

}

void loop() {
/// REGLAS
// NO PONER NINGUN DELAY()
// SOLO MODIFICAR VARIABLE AFUERA DE LA SECCION "condiciones"

///////// Leer todos los sensores

// timer de gps
if((millis()-timer_gps) > periodo_gps){
  gps_latitud_actual = gps.location.lat();
  gps_longitud_actual = gps.location.lng();
  timer_gps = millis();
}

// timer de barometro
if((millis()-timer_barometro) > periodo_barometro){
  /// TODO: Actualizar altura 
  altura_pasada = altura_actual;
  altura_actual = 0; /// <----- leer el sensor y que te de la altura actual 

  velocidad_vertical_actual = ((altura_actual - altura_pasada) / (millis()-timer_barometro)) * 1000 // Velocidad vertical en metros por segudo
  timer_barometro = millis();
}

// timer de brujula
if(fase_actual == 4){ // No es necesario leer la brujula hasta la etapa 4
  if((millis()-timer_brujula) > periodo_brujula){
    heading_actual = compass.heading();
    timer_brujula = millis();
  }
}

// Timer para transmitir datos LORA
/// TODO:
// Timer para grabar en SD
/// TODO:



//////////Condiciones///////////
switch (fase_actual) {
  case 0:
    //fase_actual 0
    if(altura_inicial + rango_altura_activacion < altura_actual){ //Si llegamos a esta condicion significa que estamos arriba de 100 metros y queremos estar en esta condicion por 10 segundos
     
      if(timer_activado_fase0){// si el timer ya esta activado vamos ver cuanto tiempo falta
        if(millis() - tiempo_fase0 > 10000 ){
          fase_actual = 1;
        }
      }
      else{
        tiempo_fase0 = millis();
        timer_activado_fase0 = true;
      }
    }
    else{
      timer_activado_fase0 = false
    }

    break;
  case 1:
    //fase_actual 1
    if(velocidad_vertical_actual < velocidad_descenso_activacion){ //Si llegamos a esta condicion significa que tenemos una velocidad negativa AKA estamos bajando 
     
      if(timer_activado_fase1){// si el timer ya esta activado vamos ver cuanto tiempo falta
        if(millis() - tiempo_fase1 > 10000 ){
          fase_actual = 2;
        }
      }
      else{
        tiempo_fase1 = millis();
        timer_activado_fase1 = true;
      }
    }
    else{
      timer_activado_fase1 = false
    }

    break;
  case 2:
    //fase_actual 2
    if(velocidad_vertical_actual < velocidad_aterrizage_tolerancia && velocidad_vertical_actual > -velocidad_aterrizage_tolerancia){ //Si llegamos a esta condicion significa que tenemos una velocidad negativa AKA estamos bajando 
     
      if(timer_activado_fase2){// si el timer ya esta activado vamos ver cuanto tiempo falta
        if(millis() - tiempo_fase2 > 10000 ){
          fase_actual = 3;
        }
      }
      else{
        tiempo_fase1 = millis();
        timer_activado_fase2 = true;
      }
    }
    else{
      timer_activado_fase2 = false
    }
    break;
  case 3:
    //fase_actual 3
    // esperar 3 segundos
    if(timer_activado_fase3){// si el timer ya esta activado vamos ver cuanto tiempo falta
        if(millis() - tiempo_fase3 > 3000 ){
          fase_actual = 4;
        }
      }
    else{
        tiempo_fase1 = millis();
        timer_activado_fase3 = true;
      }

    break;
  case 4:
    //fase_actual 4
    /// TODO: Crear una equacion que tome la distancia de gps actual del robot y la meta. Si la distancia es menor a "tolerancia_de_meta" (variable) entoces entrar a etapa 5
    break;
  case 5:
    //fase_actual 5
    break;



  default:
    // if nothing else matches, do the default
    // default is optional
    break;
}


////////// Acciones/////////////
switch (fase_actual) {
 
  case 0:
    //fase_actual 1
    break;
  case 1:
    //fase_actual 1
    break;
  case 2:
    //fase_actual 2
    break;
  case 3:
    //fase_actual 3
    /// TODO: mover las llantas a velocidad 100% para lograr que el paracaidas y el robot no se enreden con forme se despega el paragaidas 
    /// TODO: Activar el servo para despegar el paracaidas
    break;
  case 4:
    //fase_actual 4
   
    float error;
    // Se ocupa un delay de 100 millis pero en la implementacion real se va a cambiar por un non-blocking delay
    delay(100);
    heading_actual = heading_cmps12()/ 10;
    Serial.print("    angle full: ");     // Display 16 bit angle with decimal place
   
    Serial.print(heading, DEC);
    Serial.print("    angle desired: ");    
    Serial.print(goal_heading, DEC);

    //calcular la differencia entre angulo deseado y actual
    error = (goal_heading - heading);
   
    if (error > 180){
        error = error -2*180;
    }
    else if( error < -180){
        error = error +2*180;
    }
    else{
    error = error;
    }

    Serial.print("    error: ");    
    Serial.println(error, DEC);

    int output;
    //tomar el error y corregir
    if(error > 0){
     output = error * kp;
     if(output > 255){
       output = 255;
     }
     
    analogWrite(motor_pin_derecha, 0  );
    analogWrite(motor_pin_izquierda, 0 + output);
    }
    else if(error == 0){
    analogWrite(motor_pin_derecha, 0 );
    analogWrite(motor_pin_izquierda, 0);
    }
    else{
      output = abs(error) *kp;
           if(output > 255){
       output = 255;
     }
    analogWrite(motor_pin_derecha, 0 + output);
    analogWrite(motor_pin_izquierda, 0);
    }
    break;
  case 5:
    //fase_actual 5
    break;

  default:
    // if nothing else matches, do the default
    // default is optional
    break;
  }
}

/// TODO: Borrar esta parte pero antes de hacer eso, cambiar la fase 4 con elemetos de lms303 brujula
unsigned int heading_cmps12(){ 
  unsigned char high_byte, low_byte, angle8;
  char pitch, roll;
  unsigned int angle16;
  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(ANGLE_8);                     //Sends the register we wish to start reading from
  Wire.endTransmission();

  // Request 5 bytes from the CMPS12
  // this will give us the 8 bit bearing,
  // both bytes of the 16 bit bearing, pitch and roll
  Wire.requestFrom(CMPS12_ADDRESS, 5);      

  while(Wire.available() < 5);        // Wait for all bytes to come back

  angle8 = Wire.read();               // Read back the 5 bytes
  high_byte = Wire.read();
  low_byte = Wire.read();
  pitch = Wire.read();
  roll = Wire.read();

  angle16 = high_byte;                 // Calculate 16 bit angle
  angle16 <<= 8;
  angle16 += low_byte;
  return angle16;
}

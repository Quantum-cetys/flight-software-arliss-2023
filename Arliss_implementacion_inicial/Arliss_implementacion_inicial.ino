//Implementaicon inicial del codigo final hecho por Tomas con la idea que los demas puedan llenar los otros componentes
//un cambio al codigo!!!
// cambio a la rama de tomas!
//################ Iniciar librerias ###################//

///// Librerias de GPS
///********** TODO: poner libreria aqui **************//

///// Librerias de brujula
///********** TODO: poner libreria aqui **************//
#include <Wire.h>
// init para el cmps12
#define CMPS12_ADDRESS 0x60  // Address of CMPS12 shifted right one bit for arduino wire library
#define ANGLE_8  1           // Register to read 8bit angle from

//// Libreria de barometro
///********** TODO: poner libreria aqui **************//


//################# Variables Globales ######################  ///
// Todas las variables que se van a ocupar atravez de la mision

//////#/#/#/#/  Parametros de mision () /#/#/#/#/#/#
float rango_altura_activacion = 100; //(En metros) A que altura sobre la altura inicial quieres que el robot detecte que esta subiendo 
int fase_de_inicio = 0; // La fase en la que va a iniciar el robot
float tolerancia_de_meta = 10; // Radio de tolerancia en metros de que tan cerca consideramos que el robot ya llego a su meta (necesario cuando el GPS no tiene buena resolucion)
bool DEBUG = true; //Determinar si queremos saber todos las salidas
float gps_latitud_meta = 0; // latitud meta
float gps_longitud_meta = 0; // longitud meta

///// Parametros del robot (navegacion, control, actuacion)
float kp = 1.5; // factor proporcional del control del motor (que tan agresivos queremos acercarnos a el heading correcto)
int pos_servo_cerrado = 20; // posicion en la que el servo esta cerrado
int pos_servo_abierto = 180; // posicion en la que el servo esta abierto para soltar el paracaidas

///// Pines del robot
int pin_motor_izquierda = 6; // Pin conectado al mosfet de la izquierda (controla el motor, actuar con salida pwm analog output)
int pin_motor_derecha = 5; // Pin conectado al mosfet de la derecha (controla el motor, actuar con salida pwm analog output)


///// Estado del robot (no modificar directamente estos variables)
int fase_actual = 0; // fase actual del robot
float velocidad_vertical_actual = 0; // metros por segundo
float altura_actual = 0; // metros
float heading_actual = 0; // grados sobrel el horizonte 
float gps_latitud_actual = 0; // latitud
float gps_longitud_actual = 0; // longitud
float altura_inicial = 0; // Una captura de la altura en metros del robot al inicio (Idealmente que sea cerca al piso)








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
///********** TODO: Iniciar sensor aqui **************//

//iniciar gps
///********** TODO: Iniciar sensor aqui **************//

//iniciar brujula
///********** TODO: Iniciar sensor aqui **************//

///// Setup: Iniciar outputs (motores y servos)

//Definir los outputs para el motor
pinMode(pin_motor_derecha, OUTPUT);  // definir output para motor de la derecha
pinMode(pin_motor_izquierda, OUTPUT);  // definir output para motor de la derecha

//Definir los outputs para el servo
///********** TODO: Iniciar servo aqui **************//


////// Setup: Actualizar los estados y lectura de sensor

delay(5000) // dormir 5 segundos para permitir que los sensores se estabilicen 

fase_actual = fase_de_inicio; // declarar la fase en la que vamos a iniciar (con el parametro de mision fase_de_inicio) 

/// TODO: guardar altura inicial del robot para uso en las fases futuras
/// TODO: altura_inicial = 

////////////////// TODO: agregar todos los estados del robot en esta seccion
/// TODO: velocidad_vertical_actual =  
/// TODO: altura_actual = 
/// TODO: heading_actual =  
/// TODO: gps_latitud_actual = 
/// TODO: gps_longitud_actual = 

if(DEBUG){
  Serial.println("-------------Lectura inicial------------")
  Serial.print("altura_inicial: ")
  Serial.print(altura_inicial)
  Serial.print("altura_inicial: ")
  Serial.print(altura_inicial)
  Serial.print("altura_inicial: ")
  Serial.print(altura_inicial)
  Serial.print("altura_inicial: ")
  Serial.print(altura_inicial)
  Serial.print("altura_inicial: ")
  Serial.print(altura_inicial)
}

}

void loop() {
/// REGLAS
// NO PONER NINGUN DELAY()
// SOLO MODIFICAR VARIABLE AFUERA DE LA SECCION "condiciones"

//////////Condiciones///////////
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
    break;
  case 4:
    //fase_actual 4
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

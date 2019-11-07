/*
 * Instituto Tecnologico de Costa Rica
 * Computer Engineering
 * Taller de Programacion
 * País de Origen: Costa Rica
 * 
 * Código Servidor
 * Implementación del servidor NodeMCU
 * Proyecto 2, semestre 2
 * 2019
 * Version 2.0
 * Version de Arduino utilizada: 1.8.10
 * 
 * Profesor: Milton Villegas Lemus
 * Autores:  Santiago Gamboa Ramirez      - versión 1.0
 *           José Fernando Morales Vargas - versión 2.0 (soporte para el MPU9250 agregado)
 * 
 * Restricciónes: Bibliotecas ESP8266WiFi y mpu9250 (sparkfun, no borderflight) instaladas
 */
 
#include <ESP8266WiFi.h>
#define MAX_SRV_CLIENTS 1
#define PORT 7070

/*
 * ssid: Nombre de la Red a la que se va a conectar el Arduino
 * password: Contraseña de la red
 * 
 * Este servidor no funciona correctamente en las redes del TEC,
 * se recomienda crear un hotspot con el celular
 */
const char* ssid = "Tec5_EXT";
const char* password = "ApartamentosTec";
// servidor con el puerto y variable con la maxima cantidad de clientes
WiFiServer server(PORT);
WiFiClient serverClients[MAX_SRV_CLIENTS];
/*
 * Intervalo de tiempo que se espera para comprobar que haya un nuevo mensaje
 */
unsigned long previousMillis = 0, temp = 0;
unsigned long previousMillisDir=0;
const long interval = 100;
const long intervalDirectionals=1000;

/**
 * Variables para manejar las luces y polaridad de motores con el registro de corrimiento.
 * Utilizan una función propia de Arduino llamada shiftOut.
 * shiftOut(ab,clk,LSBFIRST,data), la función recibe 2 pines, el orden de los bits 
 * y un dato de 8 bits.
 * El registro de corrimiento tiene 8 salidas, desde QA a QH.
 * Ejemplos al enviar data: 
 * data = B00000000 -> todas encendidas
 * data = B11111111 -> todas apagadas
 * data = B00001111 -> depende de LSBFIRST o MSBFIRST la mitad encendida y la otra mitad apagada
 * cada bit representa una salida en el registro de corrimiento.
 * Como los inputs 1-4 del L298 se encuentra conectados de QA a QD (3-6 en el diagrama), deben modificar el byte que se envía dependiendo de la orientación en la que quieren los motores
 * Revisar el datasheet del L298 para ver de forma más precisa el comportamiento del driver según sus inputs. El siguiente es un ejemplo:
 * IN1 = 0, IN2 = 1 -> AVANZA
 * IN1 = 1, IN2 = 0 -> RETROCEDE
 * IN1 = 1, IN2 = 1 -> FRENADO CON FUERZA
 * IN1 = 0, IN2 = 0 -> FRENADO SIN FUERZA
 * la velocidad de movimiento depende del pwm
 */
#define ab 14 //GPIO14 = D5
#define clk 12 //GPIO12 = D6
byte data = B00001111;

//pin del fotosensor
#define LDRPin A0

/**
 Pines para manejo del pwm del motor
 EnA controla el pwm del motor 1
 EnB controla el pwm del motor 2
 a mayor pwm, mayor velocidad
*/
#define EnA 13 //GPIO13 = D7
#define EnB 15 //GPIO13 = D8

//PINES COMUNICACIÓN i2c con el MPU9250.
#define SCL 5 //GPIO5 = D1
#define SDA 4 //GPIO4 = D2

#include "quaternionFilters.h"
#include "MPU9250.h"
// Pin definitions

#define I2Cclock 400000
#define I2Cport Wire //Librería Wire activa las resistencias de pull-up internas necesarias para la comunicación con el protocolo I2C
#define MPU9250_ADDRESS 0x68   // Use either this line or the next to select which I2C address your device is using
MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

//Variables usadas en tiempo de ejecución
int tiempoDeGiro = 0;
double mayorAcc = 0;
//Indican si luces traseras están en modo direccional.
boolean dirD = false;
boolean DirD_ON=false;
boolean dirI = false;
boolean DirI_ON=false;

//Indica que el cronometro en el giro ha iniciado
bool turnR=false,turnL=false;
unsigned long inicio=0,fin=0,tiempogiro=0; 
unsigned long currentTurntime;

//Orientacion en el giro
float Orient;
float currentOrient;

float Pitch,Roll; //Valores de elevación-inclinación y alabeo, respectivamente

//Bandera para movimiento especial: cuadrado
boolean movEspecial=false,direct=false;
const long intervaloDirecto=3000;
const long intervaloGiro=886;
int contEspecial=0;

//Bandera para movimiento en infinito
boolean movInf=false;
boolean InfI=false, InfD=false, direct2=false;
const long intervaloGiroInf1=2300;
const long intervaloGiroInf2=3500;
const long intervaloDirInf1=1500;
const long intervaloDirInf2=2700;
int contadorInf=0; 

//Bandera para movimiento que busca el norte
boolean movNorte=false;
const long intervaloDirectoN=1500;
const long intervaloGiroN=380;
boolean back=false;
boolean gira=false;

//valor de la aceleracion maxima
float aceleracion=0.0;

//Banderas para el diagnostico
boolean diagnostico = false,buenestado=false;
boolean avanza=false, retrocede=false, girader=false, giraizq=false,luces=false;

/**Variables de bytes de control de movimiento y luces: byte comando = 0bxxxxxxxx
*Descripción de byte:
*comando[0]=IN1
*comando[1]=IN2
*comando[2]=IN3
*comando[3]=IN4
*comando[4]=LED delantero izq
*comando[5]=LED delantero der
*comando[6]=LED trasero izq
*comando[7]=LED trasero der
*/
byte stops=B00001111;
byte forward=B10101111;
byte backwards=B01011111;
byte right=B01101111;
byte left=B10011111;

bool f_light=false;
bool b_light=false;

void accelSetup(){
  Wire.begin();
  while(!Serial){};
  delay(1000);
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  if (c == 0x71) // WHO_AM_I debe ser 0x71
  {
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
    myIMU.initMPU9250();

    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    if (d != 0x48)
    {
      Serial.println(d, HEX);
      Serial.println(F("No se pudo conectar al magnetómetro!"));
      Serial.flush();
      abort();
    }
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    Serial.println("AK8963 initialized for active data mode....");
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();
  }else{
    Serial.print("No se pudo conectar al MPU9250: 0x");
    Serial.println(c, HEX);
    Serial.flush();
    //abort();
  }
}

void updateAccelInfo(){
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);
    myIMU.readGyroData(myIMU.gyroCount);
    myIMU.readMagData(myIMU.magCount);
    
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    // Calculate the gyro value into actual degrees per second
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  }
  
  // antes de leer cuartenion
  myIMU.updateTime();
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

  myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                    * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                    * *(getQ()+3));
  myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                    * *(getQ()+2)));
  myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                    * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                    * *(getQ()+3));
  myIMU.pitch *= RAD_TO_DEG;
  myIMU.yaw   *= RAD_TO_DEG;
  // Ajustado a la declinación de Cartago 2,6 °
  // - http://www.ngdc.noaa.gov/geomag-web/#declination
  // Pueden utilizar un celular para calibrar el valor. El 0 es el norte.
  myIMU.yaw  += 3;
  myIMU.roll *= RAD_TO_DEG;
  
}


/**
 * Función de configuración.
 * Se ejecuta la primera vez que el módulo se enciende.
 * Si no puede conectarse a la red especificada entra en un ciclo infinito 
 * hasta ser reestablecido y volver a llamar a la función de setup.
 * La velocidad de comunicación serial es de 115200 baudios, tenga presente
 * el valor para el monitor serial.
 */ 
void setup() {
  Serial.begin(115200);
  accelSetup();
  pinMode(EnA,OUTPUT);
  pinMode(EnB,OUTPUT);
  pinMode(clk,OUTPUT);
  pinMode(ab,OUTPUT);
  pinMode(LDRPin,INPUT);

  // ip estática para el servidor
  IPAddress ip(192,168,99,160);
  IPAddress gateway(192,168,99,1);;//(192,168,99,1);
  IPAddress subnet(255,255,255,0);

  WiFi.config(ip, gateway, subnet);

  // Modo para conectarse a la red
  WiFi.mode(WIFI_STA);
  // Intenta conectar a la red
  WiFi.begin(ssid, password);
  
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if (i == 21) {
    Serial.print("\nCould not connect to: ")
; Serial.println(ssid);
    while (1) delay(500);
  } else {
    Serial.println("\nIt´s connected");
  }
  server.begin();
  server.setNoDelay(true);
  shiftOut(ab, clk, LSBFIRST, data);
}

/*
 * Función principal que llama a las otras funciones y recibe los mensajes del cliente
 * Esta función comprueba que haya un nuevo mensaje y llama a la función de procesar
 * para interpretar el mensaje recibido.
 */
void loop() {
  // En esta función pueden comparar la lectura del acelerómetro para saber cual es su aceleración mayor
  updateAccelInfo();
  float acelx=myIMU.ax;
  float acely=myIMU.ay;
  double acel=sqrt(pow(acelx,2)+pow(acely,2));
  if (acel>aceleracion){
    aceleracion=acel;}
  if (diagnostico==true){setDiag();
    }
  if (dirI==true || dirD==true){setDireccionales();} //Se llama a la funcion que genera el parpadeo de las direccionales
  if (turnR==true || turnL==true){setTurn();}
  if (movEspecial==true){
    if (contEspecial==4){
      data=stops;
      shiftOut(ab,clk,LSBFIRST,data);
      movEspecial=false;
      direct=false;
      contEspecial=0;}
    if (direct==true){setSpecial1();}
    else if (direct==false && movEspecial==true){setSpecial2();}
    }
  if (movNorte==true){
    if (gira==true){setNorte3();}
    else if (direct==true){setNorte1();}
    else if (back==true){setNorte2();}
    }
  if (movInf==true){
    if (contadorInf==4){
      data=stops;
      shiftOut(ab,clk,LSBFIRST,data);
      movInf=false;
      direct=false;InfI=false;InfD=false;
      contadorInf=0;
      }
     else{setInf();}
    }
    
  unsigned long currentMillis = millis();
  uint8_t i;
  //check if there are any new clients
  if (server.hasClient()) {
    for (i = 0; i < MAX_SRV_CLIENTS; i++) {
      if (!serverClients[i] || !serverClients[i].connected()) {
        if (serverClients[i]) serverClients[i].stop();
        serverClients[i] = server.available();
        continue;
      }
    }
    //no free/disconnected spot so reject
    WiFiClient serverClient = server.available();
    serverClient.stop();
  }

  //chequea si el intervalo para buscar por mensajes ya paso
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    for (i = 0; i < MAX_SRV_CLIENTS; i++) {
      if (serverClients[i] && serverClients[i].connected()) {
        if(serverClients[i].available()){
          // Leemos el cliente hasta el caracter '\r'
          String mensaje = serverClients[i].readStringUntil('\r');
          // Eliminamos el mensaje leído.
          serverClients[i].flush();
          
          // Preparamos la respuesta para el cliente
          String respuesta; 
          procesar(mensaje, &respuesta);

          
          Serial.println(mensaje);
          // Escribimos la respuesta al cliente.
          serverClients[i].println(respuesta);
        serverClients[i].stop();
      }
    }
  }

}}


/*
 * Función para dividir los comandos en pares llave, valor
 * para ser interpretados y ejecutados por el Carro
 * Un mensaje puede tener una lista de comandos separados por ;
 * Se analiza cada comando por separado.
 * Esta función es semejante a string.split(char) de python
 * 
 */
void procesar(String input, String * output){
  //Buscamos el delimitador ;
  Serial.println("Checking input....... ");
  int comienzo = 0, delComa, del2puntos;
  bool result = false;
  delComa = input.indexOf(';',comienzo);
  
  while(delComa>0){
    String comando = input.substring(comienzo, delComa);
    Serial.print("Processing comando: ");
    Serial.println(comando);
    del2puntos = comando.indexOf(':');
    /*
    * Si el comando tiene ':', es decir tiene un valor
    * se llama a la función exe 
    */
    if(del2puntos>0){
        String llave = comando.substring(0,del2puntos);
        String valor = comando.substring(del2puntos+1);

        Serial.print("(llave, valor) = ");
        Serial.print(llave);
        Serial.println(valor);
        //Una vez separado en llave valor 
        *output = implementar(llave,valor); 
    }
    /*
    * Si el comando no recibe sobrecargas, chequea si es alguno de los comandos que no la necesitan
    * a output se le asigna lo que retornen las funciones llamadas, puesto que las mismas indican si hubo un error o no
    */
    
    else if(comando == "saved"){
      *output = getSaved();       
    }
    else if(comando == "roll"){
      *output = getRoll();        
    }
    else if(comando == "pitch"){
      *output = getPitch();        
    }
    else if(comando == "yaw"){
      *output = getYaw();        
    }
    else if(comando == "sense"){
      *output = getSense();        
    }
    else if(comando == "Infinite"){
      *output = infinite();         
    }
    else if(comando == "North"){
      *output = north();        
    }
    else if(comando == "Diag"){
      
      *output = diagnostic();        
    }
    else if(comando == "Especial"){
      *output = especial();         
    }else{
      *output = "no se ejecutaron comandos";
    }
    
    comienzo = delComa+1;
    delComa = input.indexOf(';',comienzo);
  }
}

String implementar(String llave, String valor){
  String result="ok;";
  Serial.print("Comparing llave: ");
  Serial.println(llave);
  if(llave == "pwm"){
    Serial.print("Move... : ");
    Serial.println(valor);
    int valorEntero= valor.toInt();
    if (valorEntero == 0){
      if (f_light==true){
        data=stops^B00001111;}
      else{data=stops^B00001100;}
      b_light=true;
      turnR=false;turnL=false;
      shiftOut(ab,clk,LSBFIRST,data);//Control del shift register para indicar la detención de los motores: byte stop=0b00001111
           
      result="Motor frenado";
    }
    else if (valorEntero>=500 && valorEntero<=1023){
      if (f_light==true && b_light==false){
        data=forward^B00000011;}
      else if (f_light==false && b_light==true){
        data=forward^B00001100;}
      else if (f_light==true && b_light==true){
        data=forward^B00001111;}
      else{data=forward;}
      analogWrite(EnA,valorEntero);
      analogWrite(EnB,valorEntero);
      shiftOut(ab,clk,LSBFIRST,data); //Control del shift register para indicar que ambos motores se muevan hacia el frente: byte forward=B10101111
      result="Motor a hacia adelante";
    }
    else if (valorEntero<=-500 && valorEntero>=-1023 ){
      if (f_light==true && b_light==false){
        data=backwards^B00000011;}
      else if (f_light==false && b_light==true){
        data=backwards^B00001100;}
      else if (f_light==true && b_light==true){
        data=backwards^B00001111;}
      else{data=backwards;}
      analogWrite(EnA,abs(valorEntero));
      analogWrite(EnB,abs(valorEntero));
      shiftOut(ab,clk,LSBFIRST,data); //Control del shift register para indicar que ambos motores se muevan hacia el atrás: byte backwards=B01010000
      result="Motor a hacia atras";
    }
    else{
      //Se le avisa al usuario que el valor ingresado fue incorrecto
      return "Valor inválido. El valor x de PWM debe ser: -1023<= x <= -500 ó 500<= x <= 1023. \r Si desea frenar el carro: x=0";
    }
  }
 
  else if(llave == "dir"){
    switch (valor.toInt()){
      case 1:
        if (f_light==true && b_light==false){
          data=right^B00000011;}
        else if (f_light==false && b_light==true){
          data=right^B00001100;}
        else if (f_light==true && b_light==true){
          data=right^B00001111;}
        else{data=right;}
        
        analogWrite(EnB,600);
        analogWrite(EnA,0);
        shiftOut(ab,clk,LSBFIRST,data);
        result="Girando derecha;";
        
        break;
      case -1:
        if (f_light==true && b_light==false){
          data=left^B00000011;}
        else if (f_light==false && b_light==true){
          data=left^B00001100;}
        else if (f_light==true && b_light==true){
          data=left^B00001111;}
        else{data=left;}
        analogWrite(EnA,600);
        analogWrite(EnB,0);
        shiftOut(ab,clk,LSBFIRST,data);
        result="Girando izquierda;";
        break;

      default:
        if (f_light==true && b_light==false){
          data=forward^B00000011;}
        else if (f_light==false && b_light==true){
          data=forward^B00001100;}
        else if (f_light==true && b_light==true){
          data=forward^B00001111;}
        else{data=forward;}
        analogWrite(EnB,800);
        analogWrite(EnA,800);
        shiftOut(ab,clk,LSBFIRST,data);
        result="Curso directo";
        break;
    }
  }
  else if(llave[0] == 'l'){
    Serial.println("Cambiando Luces");
    Serial.print("Valor luz: ");
    Serial.println(valor);
    
    //utilizar operadores lógicos de bit a bit (bitwise operators)
    switch (llave[1]){
      case 'f':
        Serial.println("Luces frontales");
        if (valor == "1"){
          if (f_light==false){
            f_light=true;
            data=data^B00000011;
            shiftOut(ab,clk,LSBFIRST,data);}
          else{
            shiftOut(ab,clk,LSBFIRST,data);}
          result="Luces frontales encendidas;";
        }
        else if (valor == "0"){
          if (f_light==true){
            f_light=false;
            data=data^B00000011;
            shiftOut(ab,clk,LSBFIRST,data);}
          else{
            shiftOut(ab,clk,LSBFIRST,data);}
          result = "Luces frontales apagadas;";
        }
        //# AGREGAR CÓDIGO PARA ENCENDER LUCES FRONTALES
        break;
      case 'b':
        Serial.println("Luces traseras");
        //# AGREGAR CÓDIGO PARA ENCENDER O APAGAR LUCES TRASERAS
        if (valor == "1"){
          if (b_light==false){
            b_light=true;
            if (dirI==false){DirI_ON=true;}
            if (dirD==false){DirD_ON=true;}
            data=data^B00001100;
            shiftOut(ab,clk,LSBFIRST,data);}
          else{
            shiftOut(ab,clk,LSBFIRST,data);}
          result="Luces traseras encendidas;";
        }
        else if (valor == "0"){
          if (b_light==true){
            b_light=false;
            if (dirI==false){DirI_ON=false;}
            if (dirD==false){DirD_ON=false;}
            data=data^B00001100;
            shiftOut(ab,clk,LSBFIRST,data);}
          else{
            shiftOut(ab,clk,LSBFIRST,data);}
          result = "Luces traseras apagadas;";
        }
        break;
      /**
       * # AGREGAR CASOS CON EL FORMATO l[caracter]:valor;
       * SI SE DESEAN manejar otras salidas del registro de corrimiento
       */
      case 'l':
        Serial.println("Direccionales para la izquierda");
        //# AGREGAR CÓDIGO PARA ENCENDER O APAGAR DIRECCIONAL IZQUIERDA
        if (valor == "1"){
          dirD=false;
          dirI=true;
          result = "Se han activado las direccionales izquierdas.";
        }
        else if (valor == "0"){
          dirI = false;
          if (b_light==true){
            if (DirI_ON==false){
              DirI_ON=true;
              data=data^B00000100;
              shiftOut(ab,clk,LSBFIRST,data);}
            }
          else{
            if (b_light==false){
              if (DirI_ON==true){
                DirI_ON=false;
                data=data^B00000100;
                shiftOut(ab,clk,LSBFIRST,data);
                }}}
                result = "Se han desactivado las direccionales izquierdas.";}
          
        break;
      case 'r':
        Serial.println("Direccionales para la derecha");
        // AGREGAR PARA CÓDIGO PARA ENCENDER O APAGAR DIRECCIONAL DERECHA
        if (valor == "1"){
          dirI=false;
          dirD = true;
          result = "Se han activado las direccionales derechas.";
        }
        else if (valor == "0"){
          dirD = false;
          if (b_light==true){
            if (DirD_ON==false){
              DirD_ON=true;
              data=data^B00001000;
              shiftOut(ab,clk,LSBFIRST,data);}
            else{shiftOut(ab,clk,LSBFIRST,data);}
            }
          else{
            if (b_light==false){
              if (DirD_ON==true){
                DirD_ON=false;
                data=data^B00001000;
                shiftOut(ab,clk,LSBFIRST,data);
                }
              else{shiftOut(ab,clk,LSBFIRST,data);}
                }}
                result = "Se han desactivado las direccionales derechas.";}
          
        
        break;
      default:
        Serial.println("Ninguna de las anteriores");
        result = "No hay cambios;";
        break;}
    
  }
  else if (llave == "TurnTime"){
    //AGREGAR CODIGO PARA CALCULAR TIEMPO DE GIRO
    //pueden utilizar millis para calcular tiempo de giro
    switch (valor.toInt()){
      case 1:
      { turnL=false;
        turnR=true;
        Orient=myIMU.yaw;
        inicio=millis();
        result="Circulo a la derecha. Orientacion: "+String(Orient);
        break;
      }
      case -1:
      {
        turnL=true;
        turnR=false;
        Orient=myIMU.yaw;
        inicio=millis();
        result="Circulo a la derecha. Orientacion: "+String(Orient);
        result = "Circulo a la izquierda;";
        break;
      }
      default:
      {
          result = "no se dio vuelta";
          break;
      }
    }
  }
  /**
   * El comando tiene el formato correcto pero no tiene sentido para el servidor
   */
  else{
    result = "Undefined key value: " + llave+";";
    Serial.println(result);
  }
  //posicionan el shiftOut según les parezca más conveniente
  //shiftOut(ab, clk, LSBFIRST, data);
  return result;
}


void setTurn(){
  unsigned long currentTurntime=millis();
  currentOrient=myIMU.yaw;
  if (turnR==true){
    data=right;
    analogWrite(EnB,800);
    analogWrite(EnA,0);
    shiftOut(ab,clk,LSBFIRST,data);
    if ((currentOrient>(Orient+3))&&(currentOrient<(Orient+10))){
      fin=currentTurntime;
      turnR=false;
      data=stops;
      shiftOut(ab,clk,LSBFIRST,data);
      tiempogiro=fin-inicio;
    }}
   if (turnL==true){
    data=left;
    analogWrite(EnB,0);
    analogWrite(EnA,800);
    shiftOut(ab,clk,LSBFIRST,data);
    if ((currentOrient<(Orient-3))&&(currentOrient>(Orient-10))){
      fin=currentTurntime;
      turnL=false;
      data=stops;
      shiftOut(ab,clk,LSBFIRST,data);
      tiempogiro=fin-inicio;
    }
  }
  
  }
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void setDireccionales(){
  
  //AGREGAR CODIGO QUE ENCIENDE Y APAGA LUCES TRASERAS DEPENDIENDO DEL VALOR DE LAS GLOBALES QUE LES CORRESPONDEN
  if (dirI==true){
    unsigned long currentTime=millis();
    if (currentTime-previousMillisDir>=intervalDirectionals){
        DirI_ON=!DirI_ON;
        previousMillisDir = currentTime;
        data=data^B00000100; //Se invierten únicamente el bit que maneja la luz trasera izquierda.
        shiftOut(ab,clk,LSBFIRST,data);
              }
        }
  else if (dirD==true){
    unsigned long currentTime=millis();
    if (currentTime-previousMillisDir>=intervalDirectionals){
        DirD_ON=!DirD_ON;
        previousMillisDir = currentTime;
        data=data^B00001000; //Se invierte únicamente el bit que maneja la luz trasera derecha.
        shiftOut(ab,clk,LSBFIRST,data);
    }}}
void setSpecial1(){
  unsigned long currentTime=millis();
  if (currentTime-inicio<intervaloDirecto){
    data=forward;
    analogWrite(EnA,800);
    analogWrite(EnB,800);
    shiftOut(ab,clk,LSBFIRST,data);
    }
  else{
    direct=false;
    inicio=currentTime;
    data=stops;
    shiftOut(ab,clk,LSBFIRST,data);
    delay(100);
    }
  }
void setSpecial2(){
  unsigned long currentTime=millis();
  if (currentTime-inicio<intervaloGiro){
    data=right;
    analogWrite(EnA,0);
    analogWrite(EnB,800);
    shiftOut(ab,clk,LSBFIRST,data);
    }
  else{
    direct=true;
    inicio=currentTime;
    data=stops;
    shiftOut(ab,clk,LSBFIRST,data);
    delay(100);
    contEspecial++;
    }
  }

void setNorte1(){
  Orient=myIMU.yaw;
  unsigned long currentTime=millis();
  if ((0<Orient)&& (Orient<35)){
    direct=false;
    back=false;
    gira=false;
    movNorte=false;
    data=stops;
    shiftOut(ab,clk,LSBFIRST,data);}
  else if (currentTime-inicio<intervaloDirectoN){
    data=forward;
    analogWrite(EnA,800);
    analogWrite(EnB,800);
    shiftOut(ab,clk,LSBFIRST,data);
    }
  else{
    gira=true;
    inicio=currentTime;}
  }
void setNorte2(){
  Orient=myIMU.yaw;
  unsigned long currentTime=millis();
  if ((0<Orient)&& (Orient<35)){
    direct=false;
    back=false;
    gira=false;
    movNorte=false;
    data=stops;
    shiftOut(ab,clk,LSBFIRST,data);}
  else if (currentTime-inicio<intervaloDirectoN){
    data=backwards;
    analogWrite(EnA,650);
    analogWrite(EnB,650);
    shiftOut(ab,clk,LSBFIRST,data);
    }
  else{
    gira=true;
    inicio=currentTime;}
  }
void setNorte3(){
  Orient=myIMU.yaw;
  unsigned long currentTime=millis();
  if ((0<Orient)&& (Orient<35)){
    direct=false;
    back=false;
    gira=false;
    movNorte=false;
    data=stops;
    shiftOut(ab,clk,LSBFIRST,data);}
  else if (currentTime-inicio<intervaloGiroN){
    data=right;
    analogWrite(EnA,0);
    analogWrite(EnB,800);
    shiftOut(ab,clk,LSBFIRST,data);
    }
  else{
    gira=false;
    direct=!direct;
    back=!back;
    inicio=currentTime;}
  }

void setInf(){
  unsigned long currentTime=millis();
  if (InfD==true){
    if (currentTime-inicio<intervaloGiroInf1){
      data=forward;
      analogWrite(EnA,500);
      analogWrite(EnB,1023);
      shiftOut(ab,clk,LSBFIRST,data);
      }
     else{
      InfD=false;
      InfI=false;
      direct2=false;
      direct=true;
      inicio=currentTime;
      contadorInf++;
      delay(50);
      }
    }
  else if (InfI==true){
    if (currentTime-inicio<intervaloGiroInf2){
      data=forward;
      analogWrite(EnA,1023);
      analogWrite(EnB,500);
      shiftOut(ab,clk,LSBFIRST,data);
      }
     else{
      InfD=false;
      InfI=false;
      direct=false;
      direct2=true;
      inicio=currentTime;
      contadorInf++;
      delay(50);
      }
    }
  else if (direct==true){
    if (currentTime-inicio<intervaloDirInf1){
      data=forward;
      analogWrite(EnA,800);
      analogWrite(EnB,800);
      shiftOut(ab,clk,LSBFIRST,data);
      }
     else{
      InfD=false;
      InfI=true;
      direct=false;
      inicio=currentTime;
      contadorInf++;
      delay(50);
      }
    }
   else if (direct2==true){
    if (currentTime-inicio<intervaloDirInf2){
      data=forward;
      analogWrite(EnA,800);
      analogWrite(EnB,800);
      shiftOut(ab,clk,LSBFIRST,data);
      }
     else{
      InfD=false;
      InfI=false;
      direct=false;
      direct2-false;
      inicio=currentTime;
      contadorInf++;
      delay(50);
      }
    }
  }

void setDiag(){
  unsigned long currentTime=millis();  
 
  Roll=myIMU.roll;
  Orient=myIMU.yaw;
  Pitch=myIMU.pitch;
  if (luces==true){
    data=B00000000;
    shiftOut(ab,clk,LSBFIRST,data);
    delay(1500);
    data=B00001111;
    shiftOut(ab,clk,LSBFIRST,data);
    delay(1500);
    luces=false;
    if (Orient==3.0 && Roll==0.0 && Pitch==0.0){
      diagnostico=false;
      buenestado=false;
      } 
    else{avanza=true;}
    }
  if (avanza==true){
    data=forward;
    analogWrite(EnA,800);
    analogWrite(EnB,800);
    shiftOut(ab,clk,LSBFIRST,data);
    delay(1500);
    float acelx=myIMU.ax;
    float acely=myIMU.ay;
    double acel=sqrt(pow(acelx,2)+pow(acely,2));
    if (acel>0.1){
      avanza=false;
      retrocede=true;
      inicio=currentTime;}
    else if (currentTime-inicio>intervaloDirecto){
      diagnostico=false;
      buenestado=false;
      avanza=false;
      data=stops;
      shiftOut(ab,clk,LSBFIRST,data);
      }
    }
   if (retrocede==true){
    data=backwards;
    analogWrite(EnA,800);
    analogWrite(EnB,800);
    shiftOut(ab,clk,LSBFIRST,data);
    delay(1500);
    float acelx=myIMU.ax;
    float acely=myIMU.ay;
    double acel=sqrt(pow(acelx,2)+pow(acely,2));
    if (acel>0.1){
      retrocede=false;
      girader=true;
      inicio=currentTime;}
    else if (currentTime-inicio>intervaloDirecto){
      diagnostico=false;
      buenestado=false;
      retrocede=false;
      data=stops;
      shiftOut(ab,clk,LSBFIRST,data);
      }
    }
  if (girader==true){
    data=forward;
    analogWrite(EnA,0);
    analogWrite(EnB,800);
    shiftOut(ab,clk,LSBFIRST,data);
    delay(1500);
    float acelx=myIMU.ax;
    float acely=myIMU.ay;
    double acel=sqrt(pow(acelx,2)+pow(acely,2));
    if (acel>0.1){
      girader=false;
      giraizq=true;
      inicio=currentTime;}
    else if (currentTime-inicio>intervaloDirecto){
      diagnostico=false;
      buenestado=false;
      girader=false;
      data=stops;
      shiftOut(ab,clk,LSBFIRST,data);
      }
    }
  if (giraizq==true){
    data=forward;
    analogWrite(EnA,800);
    analogWrite(EnB,0);
    shiftOut(ab,clk,LSBFIRST,data);
    delay(1500);
    float acelx=myIMU.ax;
    float acely=myIMU.ay;
    double acel=sqrt(pow(acelx,2)+pow(acely,2));
    if (acel>0.1){
      diagnostico=false;
      buenestado=true;
      giraizq=false;
      data=stops;
      shiftOut(ab,clk,LSBFIRST,data);}
    else if (currentTime-inicio>intervaloDirecto){
      diagnostico=false;
      buenestado=false;
      giraizq=false;
      data=stops;
      shiftOut(ab,clk,LSBFIRST,data);
      }
    }
  
  }

  
//AGREGAR CODIGO DE LOS DISTINTOS COMANDOS
/**
recordar que puede usar myIMU para conseguir los valores de los sensores.
myIMU.az = aceleración en x
myIMU.roll = valor de roll
etc. pueden ver donde se configuran los valores en updateAccelInfo()
*/

String getSaved(){
  String result;
  if (buenestado==true){result = "Carro en buen estado. Tiempo de giro registrado: "+String(tiempogiro*pow(10,-3))+" segundos"+", maxima aceleracion registrada: "+String(aceleracion)+"m/s**2";;}
  else{result="El diagnostico no fue exitoso o no se ha realizado aun.";}
  return result;
}
String getSense(){
  int V = analogRead(LDRPin);            
  int ilum=map(V,0,1024,0,100);  //Se obtiene el valor del divisor de tension en un rango de 0 a 100
  float temp=myIMU.temperature;
  String result = "Nivel de iluminacion: "+String(ilum);
  return result;
}
String getYaw(){
  Orient=myIMU.yaw;
  String result = "Orientacion: "+String(Orient);
  return result;
}
String getPitch(){
  Pitch=myIMU.pitch;
  String result = "Grado de inclinacion: "+String(Pitch);
  return result;
}
String getRoll(){
  Roll=myIMU.roll;
  String riesgo;
  if ((157<Roll)&&(Roll<180)){riesgo="No hay";}
  else if ((-157>Roll)&&(Roll>-179.9)){riesgo="No hay";}
  else{riesgo="Hay";}
  String result ="Nivel de alabeo: "+String(Roll)+" Rango aceptado sin riesgo volteo: 157<=x<=180 y -179.9<=x<=-157 "+riesgo+" riesgo de volteo.";
  return result;
 }
String infinite(){
  movInf=true;
  InfD=true;
  inicio=millis();
  String result = "Movimiento en forma de infinito.";
  return result;
}
String north(){
  movNorte=true;direct=true;
  inicio=millis();
  String result = "Buscando el Norte.";
  return result;
} 
String especial(){
  movEspecial=true;
  direct=true;
  inicio=millis();
  String result = "Movimiento especial: cuadrado";
  return result;
}
String diagnostic(){
  diagnostico=true;luces=true;
  inicio=millis();
  String result = "Realizando diagnostico... Invoque funcion de Saved para conocer el resultado.";
  return result; 
  }
//PUEDEN AGREGAR MÁS FUNCIONES PARA TENER UN CÓDIGO MÁS LIMPIO

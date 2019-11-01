#define data 2
#define clk 3
#define EnA 9
#define EnB 10
byte forward = B10100000; 
byte backward = B01010000;
byte left = B01100000;
byte right = B10010000;
byte stops=B00000000;
void setup() {
  Serial.begin(9600);
  pinMode(clk, OUTPUT); // make the clock pin an output
  pinMode(data , OUTPUT); // make the data pin an output
  pinMode(EnA,OUTPUT);
  pinMode(EnB, OUTPUT);
  shiftOut(data, clk, LSBFIRST, stops); // send this binary value to the shift register
}

void loop() {
  if (Serial.available()>0){
    int comando= Serial.read();
    if (comando=='f'){
      int valorEntero=map(1023,0,1023,0,255);
      analogWrite(EnA,valorEntero);
      analogWrite(EnB,valorEntero);
      shiftOut(data,clk,LSBFIRST,forward);
      Serial.println("Movimiento hacia el frente");
      }
    if (comando=='b'){
      shiftOut(data,clk,LSBFIRST,backward);
      Serial.println("Movimiento hacia atrás");
      }
     if (comando=='l'){
      shiftOut(data,clk,LSBFIRST,left);
      Serial.println("Movimiento hacia la izquierda");
      }
     if (comando=='r'){
      shiftOut(data,clk,LSBFIRST,right);
      Serial.println("Movimiento hacia la derecha");
      }
     if (comando=='s'){
      shiftOut(data,clk,LSBFIRST,stops);
      Serial.println("Freno");
      }}
  
}

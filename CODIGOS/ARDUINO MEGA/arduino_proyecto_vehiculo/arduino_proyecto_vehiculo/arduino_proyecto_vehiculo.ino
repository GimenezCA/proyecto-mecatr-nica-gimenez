// seguidor de linea y deteccion de objetos-proyecto 2025 bis
// se comentan las lineas "Serial.print()" que no son utilizadas
// definicion de los pines del arduino uno 

// pines para control de los motores de las ruedas

//motores derechos (mirando el auto defrente) 
#define enA 7//Enable1 L298 Pin enA 
#define in1 6 //Motor1  L298 Pin in1 
#define in2 5 //Motor1  L298 Pin in2 
//motores izquierdos (mirando el auto defrente)
#define in3 4 //Motor2  L298 Pin in3 
#define in4 3 //Motor2  L298 Pin in4 
#define enB 2 //Enable2 L298 Pin enB 

  
//pines para los sensores infrarrojos para el control del seguidor de linea
//mirando de frente

int L_S1= A0; //ir sensor 1 izquierda
int L_S2= A1; //ir sensor 2 izquierda
int R_S3= A2; //ir sensor 3 derecha
int R_S4 =A3;//ir sensor 4 derecha 

// valores iniciales de los sonsores
int sensor[4] = {0, 0, 0, 0};


//pines para el sensor de ultrasonido para la deteccion de objetos

#define echo A4    //Echo pin
#define trigger A5 //Trigger pin


// pin para el control del servomotor de giro de sensor US 

#define servo2 A6//mueve el sensor US 

// pines para control del servo que levanta la platarforma

#define servo1 A7//levanta la plataforma


// pin para el sensor de lluvia
int pinLluvia=52;
int lluvia;

//banderas
int flag; 
int flag2; 


//velocidad inicial de los motores
int vel_inic ;

// comparadores 
int Set;
int dist_izq, dist_frontal, dist_der; 

// valores iniciales del PID 
float Kp ;float Ki ; float Kd ;

// inicio los valores en 0
float error , P , I , D , PID_value ; //corregir los valores en este formato 0.0
float previous_error , previous_I ;

String comando;


// declaro los pines como entrada o salida
void setup(){ 
flag=0;
flag2=0;
vel_inic = 110;
Set=30;
Kp = 15;
Ki = 5;
Kd = 5;
error = 0;
P = 0;
I = 0;
D = 0;
PID_value = 0;
float previous_error = 0, previous_I = 0;

pinMode(13, OUTPUT);
Serial.begin(115200); // 
Serial1.begin(115200);
pinMode(L_S1, INPUT);  //sensor IR 0
pinMode(L_S2, INPUT);  //sensor IR 1
pinMode(R_S3, INPUT);  //sensor IR 2
pinMode(R_S4, INPUT);  //sensor IR 3  

// sensores ir como entrada y salida
pinMode(echo, INPUT );    // sensor ultrasonico pin "Echo"  como entrada - input
pinMode(trigger, OUTPUT); // sensor ultrasonico pin "Trigger" como salida - Output

// declaro los pines del puente H L298 y servo como salida
pinMode(enA, OUTPUT);  //motores derechos mirando de frente al vehiculo
pinMode(in1, OUTPUT); 
pinMode(in2, OUTPUT); 
pinMode(in3, OUTPUT);   
pinMode(in4, OUTPUT); 
pinMode(enB, OUTPUT); //motores izq mirando de frente al vehiculo
 

pinMode(servo1, OUTPUT); //levanta la plataforma
pinMode(servo2, OUTPUT); //gira el sensor de ultrasonido
pinMode(pinLluvia, INPUT);

// servomotor de plataforma

for (int angle = 200; angle >= 155; angle -= 2)  {
   servoPulse(servo1, angle);  }
delay(500);


}

void loop(){

//seguidor de linea 

lluvia=digitalRead(pinLluvia);
  //Serial.print(lluvia); //primer valor en monitor serial
  //Serial.print("\t");

if (flag ==0){ 

if (lluvia==1){

Ultrasonic_read();
//compara distancia frontal
dist_frontal = Ultrasonic_read();
//Serial.print(dist_frontal); 
//Serial.print("\t");
//("D F=");Serial.println(dist_frontal); // imprime la distancia frontal en el monitor serial

read_sensor_values();
//Si el sensor derecho y el sensor izquierdo están en color blanco, llamará a la función de avance

if(flag2==0){

if(dist_frontal > Set ){
  read_sensor_values();
  //Serial.print(error); 
  //Serial.print("\t");

if (error == 100 || error==3) { // 1110 O 1000  Gire a la derecha mirando de frente al vehiculo hasta que detecte un camino recto.
       do {
      read_sensor_values();
      sharpRightTurn(); 
          } 
      while (error != 0);
                  } 
  
else if (error == 101 || error== -3) { //0111 O 0001 Gire a la izq mirando de frente al vehiculo hasta que de que detecte solo el camino recto (irá hacia adelante en caso de que esté recto y a la derecha "|--")  
                                       //hasta que detecta un camino recto   
      do {
      read_sensor_values();
      sharpLeftTurn();
          } 
      while (error != 0);
      
                      }
           
 else if (error == 102) {  // 1111 esta al completo fuera de la linea negra  
                           //Gire a la izquierda hasta que detecte un camino recto.
      Stop();
      delay(500);
      do {
      read_sensor_values();
      backword();
          }
         while (error != 0);
                        }
                
 else if (error == 103) {  // 0000 el vehiculo al completo esta en la linea negra
                           //Gire a la izquierda hasta que detecte un camino recto o deténgase si llega a un callejón sin salida.
 
       if (flag2 == 0){ //bandera
       Stop();
       delay(1000);
      analogWrite(enA, 100); //velocidad motores derechos (mirando el auto de frente)
      analogWrite(enB, 110); //velocidad motores izq (mirando el auto de frente)
      forword();
      delay(500);
      Stop();
      read_sensor_values();
      if (error == 103) {     
        Stop();
        delay (1000);
        flag2 = 1; //final del recorrido
        for (int angle = 155; angle <= 200; angle += 2)  {
        servoPulse(servo1, angle);
         flag = 1; }
         
      }
       }
        }

  else {
    calculate_pid();
    motor_control();
       }
  }
   else{
    Check_side(); 
          }   
           }
            } 
else{
     Stop();
     delay(50);
     for (int angle = 155; angle <= 200; angle += 2)  {
          servoPulse(servo1, angle);
          flag = 1; }
          Stop(); 
     }
      }

else {
     //Led_prueba();
     mando_esp32cam() ;
     }     
}
            
//funcion de lectura de sensores IR mirados de frente
 void read_sensor_values()
{
  sensor[0] = !digitalRead(L_S1); //sensor 1 mirado de frente al vehiculo de izq a der
  sensor[1] = !digitalRead(L_S2); //sensor 2 mirado de frente al vehiculo de izq a der
  sensor[2] = !digitalRead(R_S3); //sensor 3 mirado de frente al vehiculo de izq a der
  sensor[3] = !digitalRead(R_S4); //sensor 4 mirado de frente al vehiculo de izq a der

// errores segun la posocion de los sensores sobre la linea
 
  if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0))
    error = 3; //llego a una esquina de 90° - el vehiculo debe girar a la derecha mirando de frente 
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0))//debe girar a la derecha con mas fuerza
    error = 2;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 1))//debe girar a la derecha levemente
    error = 1;
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1))//avanza
    error = 0;
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1 ))//debe girar a la izquierda levemente
    error = -1;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1))//debe girar a la izquierda con mas fuerza
    error = -2;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1))
    error = -3; // llego a una esquina de 90° - el vehiculo debe girar a la izquierda mirando de frente 
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0)) // gira a la izquierda 
    error = 100;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1)) // gira a la derecha
    error = 101;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1)) // esta fuera de la linea al completo
    error = 102;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0)) // estan todos los sensores sobre la linea
    error = 103;
}

// el calculo actua sobre los errores -3 a 3 
void calculate_pid()
{
  P = error;
  //Serial.print("P= ");
  //Serial.print(P); //4° valor en la linea del monitor serial
  //Serial.print("\t");
  I = I + previous_I;  
  D = error - previous_error;
  //Serial.print("D= ");
  //Serial.print(D); //5° valor en la linea del monitor serial
  //Serial.print("\t");
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  previous_I = I;
  previous_error = error;
}

void motor_control()
{
  // Cálculo de la velocidad efectiva del motor
  int left_motor_speed = vel_inic-PID_value; // motores manejados por enA - derecho mirados de frente al vehculo
  int right_motor_speed = vel_inic+PID_value; // motores manejados por enB - izq mirados de frente al vehiculo

  // Limita la velocidad del motor, no debe exceder el valor máximo de PWM.
  left_motor_speed = constrain(left_motor_speed, 0, 255);
  right_motor_speed = constrain(right_motor_speed, 0, 255);

    //Serial.print(PID_value); //6° valor en la linea del monitor serial
    //Serial.print("\t");
    //Serial.print(left_motor_speed); //7° valor en la linea del monitor serial
    //Serial.print("\t");
    //Serial.println(right_motor_speed); // 8° valor en la linea del monitor serial y salta de linea
  analogWrite(enA, left_motor_speed); //son los motores derechos mirando el auto de frente
  analogWrite(enB, right_motor_speed); //son los motores izquierdos mirando el auto de frente
  forword();
}

 // movimiento del servo
void servoPulse (int pin, int angle){
int pwm = (angle*11) + 500; //conversion del angulo en microsegundos
 digitalWrite(pin, HIGH);
 delayMicroseconds(pwm);
 digitalWrite(pin, LOW);
 delay(50);
  }

//lectura del sensor ultasonico

long Ultrasonic_read(){
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  long time = pulseIn (echo, HIGH); // obtengo el tiempo ida y vuelta del sonido,  
  return time / 59; // calcula la distancia segun el tiempo captado poe el sensor y el coeficieonte de velocidad de sonido 1/59 
}

// compara las distancias medidas por es sensor ultrasonico
void compareDistance(){
  if(dist_izq > dist_der){
 esquive_izquierdo(); //mirado de frente
   }
  else{
    esquive_derecho(); //miradode frente
  
  }
}

 
//verifica si hay obetos a la derecha o a la izquierda
void Check_side(){
    Stop();
    delay(100);
 for (int angle = 100; angle <= 150; angle += 10)  {
   servoPulse(servo2, angle);  }
    delay(800);
    dist_der = Ultrasonic_read();
    //Serial.print("D R=");Serial.println(dist_der);
    delay(400);
  for (int angle = 150; angle >= 50; angle -= 10)  {
   servoPulse(servo2, angle);  }
    delay(800);
    dist_izq = Ultrasonic_read();
    //Serial.print("D L=");Serial.println(dist_izq);
    delay(400);
 for (int angle = 50; angle <= 100; angle += 10)  {
   servoPulse(servo2, angle);  }
    delay(600);
    compareDistance();
}

void Led_prueba() {
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);}

void mando_esp32cam() {
  if (Serial1.available()) {
    comando = Serial1.readStringUntil('\n');  // Espera hasta recibir \n
    comando.trim(); // Elimina \r, espacios, etc.

    if (comando == "Forward") {
      Serial.println("Comando: Forward");
      analogWrite(enA, 140);
      analogWrite(enB, 140);
      forword();
      // Acción para avanzar
    }
    else if (comando == "Left") {
      Serial.println("Comando: Left");
      sharpLeftTurn();
    } 
    else if (comando == "Right") {
      Serial.println("Comando: Right");
      sharpRightTurn();
    } 
    else if (comando == "Back") {
      Serial.println("Comando: Back");
      backword();
    } 
    else if (comando == "Stop") {
      Serial.println("Comando: Stop");
      Stop();
    } 
    else {
      Serial.print("Comando desconocido: ");
      Serial.println(comando);
    }
  }
}

void forword(){ //avance  
//motores derecho avanza
digitalWrite(in1, HIGH); 
digitalWrite(in2, LOW);  
//motores izquierdo avanza
digitalWrite(in3, HIGH); 
digitalWrite(in4, LOW); 
}

void backword(){ // reversa
// motor derecho en reversa
analogWrite(enA, 100);//velocidad motores derechos (mirando el auto de frente)
digitalWrite(in1, LOW); 
digitalWrite(in2, HIGH);  
//motor izquierdo en reversa
analogWrite(enB, 100);//velocidad motores izq (mirando el auto de frente)
digitalWrite(in3, LOW); 
digitalWrite(in4, HIGH); 
}

void Stop(){ //detencion
analogWrite(enA, 0); 
analogWrite(enB, 0);
digitalWrite(in1, LOW);  
digitalWrite(in2, LOW);  
digitalWrite(in3, LOW); 
digitalWrite(in4, LOW); 
}

void sharpRightTurn(){//doblar a la derecha mirando el vehiculo de frente
analogWrite(enA, 150); //velocidad motores derechos (mirando el auto defrente) 
analogWrite(enB, 150); //velocidad motores izq (mirando el auto de frente)
digitalWrite(in2, HIGH); 
digitalWrite(in1, LOW);  
digitalWrite(in4, LOW); 
digitalWrite(in3, HIGH);
}

void sharpLeftTurn() { //girar a la izquierda mirando el vehiculo de frente
analogWrite(enA, 150); //velocidad motores derechos (mirando el auto defrente) 
analogWrite(enB, 150); //velocidad motores izq (mirando el auto de frente)
digitalWrite(in2, LOW); 
digitalWrite(in1, HIGH); 
digitalWrite(in4, HIGH); 
digitalWrite(in3, LOW); 
}

void esquive_izquierdo(){ //mirado de frente
  analogWrite(enA, 200); // velocidad motores derechos (mirando el auto defrente) 
  analogWrite(enB, 40); //velocidad motores izq (mirando el auto de frente)
  forword();
  delay(800);
  analogWrite(enA, 40); // velocidad motores derechos (mirando el auto defrente) 
  analogWrite(enB, 240);
  forword();
  delay(1500);
  analogWrite(enA, 220); // velocidad motores derechos (mirando el auto defrente) 
  analogWrite(enB, 30); //velocidad motores izq (mirando el auto de frente)
  forword();
  delay(800);
  }

void esquive_derecho(){ //mirado de frente
  analogWrite(enA, 40); // velocidad motores derechos (mirando el auto defrente) 
  analogWrite(enB, 240); //velocidad motores izq (mirando el auto de frente)
  forword();
  delay(800);
  analogWrite(enA, 250); // velocidad motores derechos (mirando el auto defrente) 
  analogWrite(enB, 30);
  forword();
  delay(1800);
  analogWrite(enA, 40); // velocidad motores derechos (mirando el auto defrente) 
  analogWrite(enB, 250); //velocidad motores izq (mirando el auto de frente)
  forword();
  delay(1000);
  }

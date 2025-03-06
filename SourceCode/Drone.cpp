#include <Wire.h>
#include <Servo.h>

#define MOTOR_PIN1 7 // setăm câteva pinuri pentru motoare
#define MOTOR_PIN2 6
#define MOTOR_PIN3 4
#define MOTOR_PIN4 5

unsigned long counter_1, counter_2, counter_3, counter_4, current_count;  //  niste variabile necesare pentru funcția IR

byte last_CH1_state, last_CH2_state, last_CH3_state, last_CH4_state, last_CH5_state; // alte funcții necesare pentru funcția IR

int input_YAW;      // variabile ce tin minte valorile pentru miscarea quadcopterului
int input_PITCH;    
int input_ROLL;     
int input_THROTTLE; 

int ch1Value;  // variabilele pentru ce citim de la receiver si telecomanda
int ch2Value;
int ch3Value;
int ch4Value;

Servo right_prop1;
Servo right_prop2;
Servo left_prop1;
Servo left_prop2;

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ; // variabile pentru sistemul de balans
 
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];

float elapsedTime, time, timePrev;
int i, vs1,vs2,vd1,vd2;
float rad_to_deg = 180/3.141592654; 

float PIDX, PIDY, errorX, previous_errorX, errorY, previous_errorY;

float pid_pX=0, pid_pY=0;  
float pid_iX=0, pid_iY=0;
float pid_dX=0, pid_dY=0;

/////////////////PID CONSTANTS/////////////////
double kpX=0, kpY=1; // 2
double kiX=0, kiY=0.001; // 0
double kdX=0, kdY=0; // 0.6
///////////////////////////////////////////////

float desired_angleX = 0, desired_angleY = 0; //unghiurile la care vrem sa stea drona cand o controlam

int verifySpeed(int val){  // o functie ce seteaza viteza unui motor in limitele impuse
  if(val>2000)
    return 2000;
  if(val<1030)
    return 1000;
  return val;
}

int verifyPID(int val){  // o functie ce seteaza diferenta maxima de viteza la drona pentru sistemul de balans
  if(val>400)
    return 400;
  if(val<-400)
    return -400;
  return val;
}

int verifyAngle(int angle){  // pentru a face drona sa stea dreapta
  if(angle>=-2 && angle<=2)
    return 0;
  return angle;
}

void setup() {
  Wire.begin();
  Wire.beginTransmission(0x68);  // incepem comunicare cu MPU6050, adica componenta responsabila pentru balansarea dronei
  // 0x68 este adresa din memorie pentru a incepe comunicarea cu aceasta piesa
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  Serial.begin(250000); //aceasta linie este pentru eventuale erori

  PCICR |= (1 << PCIE0);    //cum nu putem utiliza functia PulseIn de la arduino pentru a citit valorile de la telecomanda, suntem nevoiti sa ne programam
  // propria functie, iar pentru aceasta trebuie sa permitem utilizarea intreruperilor de catre pinurile placii arduino, PCICR este locatia in placa arduino
  // unde putem seta sa se permita intreruperi iar scriind 1<<PCIE0 permitem utilizarea intreruperilor pe registrul B al placii arduino, adica pin-urile
  // D8-D13                                             
  PCMSK0 |= (1 << PCINT0);  // setam ca pinul D8 sa permita intreruperi
  PCMSK0 |= (1 << PCINT1);  // setam ca pinul D9 sa permita intreruperi    
  PCMSK0 |= (1 << PCINT2);  // setam ca pinul D10 sa permita intreruperi                                       
  PCMSK0 |= (1 << PCINT3);  // setam ca pinul D11 sa permita intreruperi
  
  left_prop1.attach(MOTOR_PIN1);  // atasam motoarele de pinurile corespondente
  left_prop2.attach(MOTOR_PIN2);
  right_prop1.attach(MOTOR_PIN3);
  right_prop2.attach(MOTOR_PIN4);

  time = millis(); //necesar pentru a determina timpul trecut

  left_prop1.writeMicroseconds(1000);  // trezim ESC-urile
  left_prop2.writeMicroseconds(1000); 
  right_prop1.writeMicroseconds(1000);
  right_prop2.writeMicroseconds(1000);
  
  delay(7000); // o pauza pentru a conecta bateria si la motoare
  //delay(1000);
}
 
void loop() {
  ////Controls////
  ch1Value = map(input_ROLL, 1000, 1954, -25, 25);  // setam valoarea citita de la telecomanda ca sa fie unghi
  ch2Value = map(input_PITCH, 1000, 2000, -25, 25); // setam valoarea citita de la telecomanda ca sa fie unghi
  ch3Value = input_THROTTLE;  // citim valoarea pentru puterea la motoare
  ch4Value = map(input_YAW, 1000,2000, -100, 100); // valoarea pentru rotirea dronei

  //Serial.println(ch4Value);

  timePrev = time;
  time = millis();  

  elapsedTime = (time - timePrev) / 1000; // aflam cat timp a trecut de la rularea loop-ului trecut

  //Serial.println(elapsedTime);

  desired_angleY = verifyAngle(ch1Value); //setam ca unghiul la care sa stea drona sa fie cel citit
  desired_angleX = verifyAngle(ch2Value);
  /////////////////////////////I M U/////////////////////////////////////

  if( input_THROTTLE > 1030){
    Wire.beginTransmission(0x68);
    Wire.write(0x3B); //cerem adresa 0x3B - ce corespunde acceleratiei pe axa X
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,6,true);   // cerem 6 adrese, 2 pentru fiecare axa
    
    Acc_rawX=Wire.read()<<8|Wire.read(); 
    Acc_rawY=Wire.read()<<8|Wire.read();
    Acc_rawZ=Wire.read()<<8|Wire.read();

    /*---X---*/
    Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
    /*---Y---*/
    Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
    
    Wire.beginTransmission(0x68);
    Wire.write(0x43); //datele pentru giroscop
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,4,true); //doar 4 adrese, nu avem nevoie de axa Z
   
    Gyr_rawX=Wire.read()<<8|Wire.read(); //
    Gyr_rawY=Wire.read()<<8|Wire.read();
 
    Gyro_angle[0] = Gyr_rawX/131.0; 
    /*---Y---*/
    Gyro_angle[1] = Gyr_rawY/131.0;

    /*---X axis angle---*/
    Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
    /*---Y axis angle---*/
    Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];  // calculam unghiurile citite de senzor
    
   
    /*Now we have our angles in degree and values from -100º to 100º aprox*/
  
    /*///////////////////////////P I D///////////////////////////////////*/

    /*
    Serial.print(Total_angle[1]);
    Serial.print("X ");
    Serial.print(Total_angle[0]);
    Serial.print("Y\n");
    /*
    Serial.print(desired_angleX);
    Serial.print("X ");
    Serial.print(desired_angleY);
    Serial.print("Y\n");*/
  
    errorX = Total_angle[1] - desired_angleX;  // calculam diferenta de unghi la care este drona
    errorY = Total_angle[0] - desired_angleY;

    pid_pX = kpX*errorX; // calculam partea proportionala la stabilizarea dronei
    pid_pY = kpY*errorY;

    if(-3 <errorX <3)
    {
      pid_iX = pid_iX+(kiX*errorX);  // calculam partea integrala la stabilizarea dronei
    }
    else
      pid_iX = 0;
      
    if(-3 <errorY <3)
    {
      pid_iY = pid_iY+(kiY*errorY);  
    }
    else
      pid_iX = 0;

    pid_dX = kdX*((errorX - previous_errorX)/elapsedTime); // calculam partea derivata la stabilizarea dronei
    pid_dY = kdY*((errorY - previous_errorY)/elapsedTime);

    PIDX = pid_pX + pid_iX + pid_dX; // calculam valorile pentru balansare
    PIDY = pid_pY + pid_iY + pid_dY;

    PIDX = verifyPID(PIDX);
    PIDY = verifyPID(PIDY);

    vs1 = verifySpeed(ch3Value + PIDX - PIDY); // calculam viteza fiecarui motor
    vs2 = verifySpeed(ch3Value - PIDX - PIDY);
    vd1 = verifySpeed(ch3Value + PIDX + PIDY);
    vd2 = verifySpeed(ch3Value - PIDX + PIDY);
    
    left_prop1.writeMicroseconds(vs1); // setam viteza fiecarui motor
    left_prop2.writeMicroseconds(vs2);
    right_prop1.writeMicroseconds(vd1);
    right_prop2.writeMicroseconds(vd2);


    Serial.print(vs1);
    Serial.print(" ");
    Serial.print(vs2);
    Serial.print(" ");
    Serial.print(vd1);
    Serial.print(" ");
    Serial.print(vd2);
    Serial.print("\n");
    
    previous_errorX = errorX; //tinem minte eroarea precedenta a dronei pentru a ajuta la stabilizare
    previous_errorY = errorY;
  }
  else{
    left_prop1.writeMicroseconds(1000); // daca setam o viteza foarte mica sa oprim motoarele
    left_prop2.writeMicroseconds(1000);
    right_prop1.writeMicroseconds(1000);
    right_prop2.writeMicroseconds(1000);

    pid_pX=0; pid_pY=0;
    pid_iX=0; pid_iY=0;
    pid_dX=0; pid_dY=0;
  }
}

//functia de intreruperi 
//----------------------------------------------

ISR(PCINT0_vect){
//citim la ce punct din timp suntem la rularea programului
  
  current_count = micros();
  ///////////////////////////////////////Channel 1
  if(PINB & B00000001){                             //Functia IR pentru telecomanda 8
    if(last_CH1_state == 0){                        //verificam care dintre pinuri si-a schimbat starea
      last_CH1_state = 1;                            
      counter_1 = current_count;                     
    }
  }
  else if(last_CH1_state == 1){                          
    last_CH1_state = 0;                              
    input_ROLL = current_count - counter_1;   
  }

  ///////////////////////////////////////Channel 2
  if(PINB & B00000010 ){                             //pin D9 -- B00000010                                              
    if(last_CH2_state == 0){                                               
      last_CH2_state = 1;                                                   
      counter_2 = current_count;                                             
    }
  }
  else if(last_CH2_state == 1){                                           
    last_CH2_state = 0;                                                     
    input_PITCH = current_count - counter_2;                             
  }
  
  ///////////////////////////////////////Channel 3
  if(PINB & B00000100 ){                             //pin D10 - B00000100                                         
    if(last_CH3_state == 0){                                             
      last_CH3_state = 1;                                                  
      counter_3 = current_count;                                               
    }
  }
  else if(last_CH3_state == 1){                                             
    last_CH3_state = 0;                                                    
    input_THROTTLE = current_count - counter_3;                            

  }
  
  ///////////////////////////////////////Channel 4
  if(PINB & B00001000 ){                             //pin D11  -- B00010000                      
    if(last_CH4_state == 0){                                               
      last_CH4_state = 1;                                                   
      counter_4 = current_count;                                              
    }
  }
  else if(last_CH4_state == 1){                                             
    last_CH4_state = 0;                                                  
    input_YAW = current_count - counter_4;                            
  }
}

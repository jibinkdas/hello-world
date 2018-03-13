/////sensor input//////
#define int0 2//sensor
#define int1 3
////switch connections///////
#define aut 4//auto switches
#define emg 13//emergency switch
#define sw1 5//vuccum switch
#define sw2 6//main motor
#define sw3 7//forward switch
#define rev A2//reverse switch
#define air_in A0//air in
//////relay connections///////
#define vaccum 12//vaccum out
#define air_out A1//air out
//////motor connections///////
#define in1 9//rev motor outs
#define in2 8//rev motor outs
#define e1 10//rev motor out
#define m 11 //main motor out

int i=0;
volatile boolean rev_state=0,air_state=0,vcm_state=0,mm_state=0,frwd_state=0,aut_state=0,em=0;//status of switche


void setup() 
{
  pinMode(int0,INPUT);
  pinMode(int1,INPUT);
  pinMode(sw1,INPUT);
  pinMode(sw2,INPUT);
  pinMode(sw3,INPUT);//FORWAED SWITCH
  pinMode(aut,INPUT);
  pinMode(em,INPUT);
  pinMode(rev,INPUT);
  pinMode(air_in,INPUT);

  
  pinMode(vaccum,OUTPUT); 
  pinMode(air_out,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(m,OUTPUT);
  pinMode(e1,OUTPUT);
  Serial.begin(9600);
  Serial.println("start");

  ///////initial state////////
  M2off();
  digitalWrite(vaccum,HIGH);
  digitalWrite(air_out,HIGH);
  M1stop();
  
//    if(digitalRead(int0)==1)
//    { 
//    M1SpeedUp();  
//    while(digitalRead(int0)!=0)
//    {
//    M1backward(); 
//    Serial.println("back");
//       if(emergency())
//      {
//        M1stop(); 
//        break;
//      }
//    }
//    M1speedDown(); 
//     M1stop();  
//    }
  
}

void loop() 
{
  
Serial.println("loop");
vaccumCtrl();
m2Ctrl();
airCtrl();
moving_motor_control();
cycle();

if(emergency())
{
   Serial.println("EMERGENCY");
  vcm_state=0;
  mm_state=0;
  frwd_state=0;
  air_state=0;
  rev_state=0;
}
  
}

///////////////////////////////////end of main/////////////////////







 
  void M1forward()////reverse motor controll
{
//i=0;*
//analogWrite(e1,i);
digitalWrite(in1,HIGH);
digitalWrite(in2,LOW);
}

void M1backward()
{
//i=0;
//analogWrite(e1,i);
digitalWrite(in1,LOW);
digitalWrite(in2,HIGH);
}

void M1speedDown()
{
 i=255;
for(i;i>=0;i--)
  {analogWrite(e1,i);delay(4);
     if(emergency())
    {
      vcm_state=0;
      mm_state=0;
      frwd_state=0;
      aut_state=0;
      air_state=0;
      break;
    }
  }
}

void M1SpeedUp()
{
 i=0;
for(i;i<=255;i++)
  {analogWrite(e1,i);delay(4);
    if(emergency())
    {
      vcm_state=0;
      mm_state=0;
      frwd_state=0;
      aut_state=0;
      air_state=0;
      break;
    }
    }
}

void M1stop()
{
i=0;
analogWrite(e1,i);
}

void M2on()
{
analogWrite(m,255);
}
void M2off()
{
analogWrite(m,0);
}

void m2Ctrl()
{
   //Serial.println("M2CTRL");
   if(digitalRead(sw2)==0)
   { 
     Serial.println("M2CTRL");
    mm_state=!mm_state;
    delay(500);
    while(!digitalRead(sw2));
   }
    if(mm_state==1)
    M2on();
    else
    M2off();
}
void vaccumCtrl()
{
  //Serial.println("vcm");
 if(digitalRead(sw1)==0)
 { 
    Serial.println("VCCUM");
  vcm_state=!vcm_state;
  delay(500);
  while(!digitalRead(sw1)); 
 }
  if(vcm_state==1)
  { 
  digitalWrite(vaccum,LOW); 
  }
  else
  digitalWrite(vaccum,HIGH);
}

void airCtrl()
{
 //Serial.println("AIR");
 if(digitalRead(air_in)==0)
 { 
    Serial.println("AIR");
  air_state=!air_state;
  delay(500);
  while(!digitalRead(air_in)); 
 }
  if(air_state==1)
  { 
  digitalWrite(air_out,LOW); 
  }
  else
  digitalWrite(air_out,HIGH);
}




boolean emergency()
{
  
   if(digitalRead(emg)==0)
 {
  Serial.println("emg"); 
  em=!em;
  delay(500);
  while(!digitalRead(emg)); 
 }
  if(em==1)
  { 
  em=0;
  return 1;
  }
  else
  return 0;
}


void moving_motor_control()
{
  if(digitalRead(sw3)==0)
 { 
    Serial.println("forward");
  frwd_state=!frwd_state;
  delay(500);
  while(!digitalRead(sw3));
 }
  if(frwd_state==1)
  {
  M1forward();
  M1SpeedUp();
  while(frwd_state==1)
  {
    Serial.println("manual frd");
    vaccumCtrl();
    m2Ctrl();
    airCtrl();
    if(emergency())
    {
      air_state=0;
      vcm_state=0;
      mm_state=0;
      frwd_state=0;
      break;
    }
Serial.println("emg out"); 
    if(digitalRead(int1)==0)
    {
      Serial.println("sensor"); 
    M1speedDown();
    M1stop();
    frwd_state=0;
    break;
    }
Serial.println("sen out"); 
    if(digitalRead(sw3)==0)
    { 
    Serial.println("change");
    frwd_state=0;
    M1speedDown();
    M1stop();   
    while(!digitalRead(sw3));
    }

  } 
  }
  else
  M1stop();
///////////////////////////motor backward
if(digitalRead(rev)==0)
 { 
 Serial.println("backward");
  rev_state=!rev_state;
  while(!digitalRead(rev));
 }
  if(rev_state==1)
  {
  M1backward();
  M1SpeedUp();
  
  while(rev_state==1)
  {
     Serial.println("man backward");
    vaccumCtrl();
    m2Ctrl();
    airCtrl();
    if(emergency())
    {
      air_state=0;
      vcm_state=0;
      mm_state=0;
      rev_state=0;
    }

    if(digitalRead(int0)==0)
    { 
    M1speedDown();
    M1stop();
    rev_state=0;
    break;
    }
    
    if(digitalRead(rev)==0)
    { 
    rev_state=!rev_state;
    M1speedDown();
    M1stop();
    while(!digitalRead(rev));
    }

  } 
  }
  else
  M1stop();
}






void cycle()
{
if(digitalRead(aut)==0)
 { 
    Serial.println("cucle start");
    aut_state=!aut_state;
    digitalWrite(vaccum,LOW);
    M2on();
    frwd_state=1; 
    delay(100);
  while(!digitalRead(aut));
 }
 
if(aut_state==1)
{
  while(aut_state==1)
  { 
    M1forward();
    M1SpeedUp();  
    while(frwd_state == 1)
    { 
       Serial.println("for"); 

        if(emergency())
        {
          air_state=0;
          vcm_state=0;
          mm_state=0;
          rev_state=0;
          aut_state=0;
          M2off();
          frwd_state=0;
          break;
        }                         
        if(digitalRead(int1)==0)
        { 
          M1speedDown();
          M1stop();
          frwd_state=0;
          rev_state=1;
          digitalWrite(vaccum,HIGH);
          digitalWrite(air_out,LOW);
          M1backward();
          M1SpeedUp();
          break; 
        }               
        if(digitalRead(aut)==0)
        { 
          Serial.println("cucle stop");
          aut_state=!aut_state;
          delay(50);
          while(!digitalRead(aut));
          frwd_state=0;
          rev_state=0;
          break;
       }             
    }///fwrd loop clossing 
    while(rev_state==1)
    {  
      Serial.println("rev");
      
      if(emergency())
      {
        air_state=0;
        vcm_state=0;
        mm_state=0;
        rev_state=0;
        aut_state=0;
        M2off();
        break;
      }
      if(digitalRead(int0)==0)
      { 
        M1speedDown();
        M1stop();
        frwd_state=0;
        rev_state=0;
        aut_state=!aut_state;        
        digitalWrite(vaccum,HIGH);
        digitalWrite(air_out,LOW);
        //M1forward();
        //M1SpeedUp();
        break;
      }       
      if(digitalRead(aut)==0)
      { 
         Serial.println("cycle stop");
        aut_state=!aut_state;
        delay(50);
        while(!digitalRead(aut));
        rev_state=0;
        frwd_state=0;
        break;
      }       
    }  ///rev loop clossing         
    if(digitalRead(aut)==0)
    { 
      Serial.println("cycle stop");
      aut_state=!aut_state;
      delay(50);
      while(!digitalRead(aut));
      break;
    }

 }//////while cycle condition close
}//////if cycle condition close

  
}//////stop of cycle


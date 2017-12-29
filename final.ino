#include <SoftwareSerial.h>

SoftwareSerial lidarSerial(2, 3); // RX, TX
SoftwareSerial mySerial(10, 11); // RX, TX
  int Gpsdata;             // for incoming serial data
  unsigned int finish =0;  // indicate end of message
  unsigned int pos_cnt=0;  // position counter
  unsigned int lat_cnt=0;  // latitude data counter
  unsigned int log_cnt=0;  // longitude data counter
  unsigned int flg    =0;  // GPS flag
  unsigned int com_cnt=0;  // comma counter
  char lat[20];            // latitude array
  char lg[20];             // longitude array
#define ADC_ref 2.7
#define zero_x 1.6
#define zero_y 1.57
#define zero_z 1.35
#define sensitivity_x 0.2
#define sensitivity_y 0.2
#define sensitivity_z 0.2

unsigned int value_x;
unsigned int value_y;
unsigned int value_z;

float xv;
float yv;
float zv;

float angle_x;
float angle_y;
float angle_z; 

   void Receive_GPS_Data();
int dist,strength,check,i,uart[9];
const int HEADER=0X59;
boolean state=0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  analogReference(EXTERNAL);
  //lidarSerial.begin(115200);
}

void loop() 
{
  // Serial.println(digitalRead(5));

if(digitalRead(5)==0)
 { 
    Serial.print("lidar data");
    Serial.println('/');
  state=1;
  delay(500);
  while(!digitalRead(5));
 }
  
  while(state==1)
  {
        if(digitalRead(5)==0)
     { 
        Serial.println("lidar data");
      state=0;
      delay(500);
      while(!digitalRead(5));
     }
    //Serial.println("state");
    lidarSerial.begin(115200);
  if(lidarSerial.available()) 
  {
    //Serial.println("1");
    if(lidarSerial.read()==HEADER) 
    {
      //Serial.println("2");
      uart[0]=HEADER;
      if(lidarSerial.read()==HEADER) 
      {
        //Serial.println("3");
        uart[1]=HEADER;
        for(i=2;i<9;i++)
        {
        uart[i]=lidarSerial.read();//Serial.println(uart[i]);}
        check=uart[0]+uart[1]+uart[2]+uart[3]+uart[4]+uart[5]+uart[6]+uart[7];
 //      Serial.println(uart[8]);
//        Serial.println(check);
//       Serial.println(check&&0xff);
        //uart[8]=check&&0xff;
        //if(uart[8]==(check&&0xff))
        if(uart[8]<500)
        {
            dist=uart[2]+uart[3]*256;
            //strength=uart[4]+uart[5]*256;
            if(dist<1000&&dist>30)
            {
            Serial.print("dist=");
            Serial.print(dist);
//            Serial.print('\t');
//            Serial.print("strength=");
//            Serial.print(strength);
            Serial.print('/');
            Serial.print('\n'); 
            //Serial.println("get data");
            //state=0;
            uart[2]=0,uart[3]=0,dist=0;
            
            lidarSerial.end();
            break;
            }
             else
            {
            Serial.println("error reading"); 
            state=1;    
            }          
         }
        else
        {
        Serial.println("error data");     
        }
      }
    }

  }
  
  }
}

        mySerial.begin(9600);
        Receive_GPS_Data();  
      //           
      Serial.print("Latitude  : ");
      Serial.print(lat);
      Serial.println('/');
      Serial.print("Longitude : ");
      Serial.print(lg);
      Serial.println('/');
      finish = 0;pos_cnt = 0;
      value_x = analogRead(A5);
      value_y = analogRead(A0);
      value_z = analogRead(A4);
//      Serial.println (value_x);
//      Serial.println (value_y);
//      Serial.println (value_z);
      //
//      Serial.println (0.00263671875*value_x);
//      Serial.println (0.00263671875*value_y);
//      Serial.println (0.00263671875*value_z);
      delay(1000);
            xv=(value_x/1024.0*ADC_ref-zero_x)/sensitivity_x;

//      Serial.print ("x= ");
//      Serial.print (xv);
//      Serial.print(" g ");
      //
      yv=(value_y/1024.0*ADC_ref-zero_y)/sensitivity_y;
      ////
//      Serial.print ("y= ");
//      Serial.print (yv);
//      Serial.print(" g ");
      //
      zv=(value_z/1024.0*ADC_ref-zero_z)/sensitivity_z;
      //
//      Serial.print ("z= ");
//      Serial.print (zv);
//      Serial.print(" g ");
      //
      //Serial.print("\n");
      //
      //Serial.println("Rotation ");
      
      Serial.print("x= ");
      
      angle_x =atan2(-yv,-zv)*57.2957795+180;
      
      Serial.print(angle_x);
      Serial.print(" deg");
      Serial.print(" ");
      
      Serial.print("y= ");
      
      angle_y =atan2(-xv,-zv)*57.2957795+180;
      
      Serial.print(angle_y);
      Serial.print(" deg");
      Serial.print(" ");
      
      Serial.print("z= ");
      
      angle_z =atan2(-yv,-xv)*57.2957795+180;
      
      Serial.print(angle_z);
      Serial.print(" deg");
      Serial.print('/');
      Serial.print("\n");
      
      //delay(1000);
      //state=1;
      mySerial.end();


}


 void Receive_GPS_Data()
  {
    while(finish==0){
      while(mySerial.available()>0){         // Check GPS data
        Gpsdata = mySerial.read();
        //Serial.print(char(Gpsdata)); 
        flg = 1;
       if( Gpsdata=='$' && pos_cnt == 0)   // finding GPRMC header
         pos_cnt=1;
       if( Gpsdata=='G' && pos_cnt == 1)
         pos_cnt=2;
       if( Gpsdata=='P' && pos_cnt == 2)
         pos_cnt=3;
       if( Gpsdata=='R' && pos_cnt == 3)
         pos_cnt=4;
       if( Gpsdata=='M' && pos_cnt == 4)
         pos_cnt=5;
       if( Gpsdata=='C' && pos_cnt==5 )
         pos_cnt=6;
       if(pos_cnt==6 &&  Gpsdata ==','){   // count commas in message
         com_cnt++;
         flg=0;
       }
 
       if(com_cnt==3 && flg==1){
        lat[lat_cnt++] =  Gpsdata;         // latitude
        flg=0;
       }
 
       if(com_cnt==5 && flg==1){
         lg[log_cnt++] =  Gpsdata;         // Longitude
         flg=0;
       }
 
       if( Gpsdata == '*' && com_cnt >= 5){
         com_cnt = 0;                      // end of GPRMC message
         lat_cnt = 0;
         log_cnt = 0;
         flg     = 0;
         finish  = 1;
 
      }
    }
}
}

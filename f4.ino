#include <SoftwareSerial.h>

SoftwareSerial lidarSerial(2, 3); // RX, TX
SoftwareSerial mySerial(10, 11); // RX, TX
int Gpsdata;             // for incoming serial data
unsigned int finish = 0; // indicate end of message
unsigned int pos_cnt = 0; // position counter
unsigned int lat_cnt = 0; // latitude data counter
unsigned int log_cnt = 0; // longitude data counter
unsigned int flg    = 0; // GPS flag
unsigned int com_cnt = 0; // comma counter
char lat[20];            // latitude array
char lg[20];             // longitude array
#define ADC_ref 5
//volatile float zero_x = 0;
//volatile float zero_y = 0;
//volatile float zero_z = 0;
float last_angle_y=180;
#define zero_x 1.66
#define zero_y 1.63
#define zero_z 1.39

#define sensitivity_x 0.3
#define sensitivity_y 0.3
#define sensitivity_z 0.3

unsigned int value_x;
unsigned int value_y;
unsigned int value_z;

float xv;
float yv;
float zv;

float angle_x;
float angle_y;
float angle_z;
float angle_difference=0;
void Receive_GPS_Data();
int dist = 0, strength, check, j = 0, i, uart[9], a[3];
const int HEADER = 0X59;
boolean state = 0;




void setup()
{
  Serial.begin(9600);
  pinMode(4,OUTPUT);
  digitalWrite(4,HIGH);
}

void loop()
{

  Receive_GPS_Data();
  tiltdatafrom_accelerometer();
  //key_press();

}


void key_press()
{
  if (digitalRead(5) == 0)
  {
    delay(100);
    while (!digitalRead(5));
    delay(200);
    Serial.print("start");
    Serial.print("\n");
  }
}
void tiltdatafrom_accelerometer()
{
    value_x = analogRead(A5);
    delay(10);
    xv = (value_x / 1024.0 * ADC_ref - zero_x) / sensitivity_x;
    yv = (value_y / 1024.0 * ADC_ref - zero_y) / sensitivity_y;
    zv = (value_z / 1024.0 * ADC_ref - zero_z) / sensitivity_z;
    angle_x = atan2(-yv, -zv) * 57.2957795 + 180;
    angle_y = atan2(-xv, -zv) * 57.2957795 + 180;
    angle_z = atan2(-yv, -xv) * 57.2957795 + 180;
    angle_difference=last_angle_y-angle_y;
    Serial.print("orientation=");
    Serial.print('\n');
    Serial.print(angle_difference);
    Serial.print('\n');
  
} 

void tfmini()
{
  lidarSerial.begin(115200);
  j = 0;
  while (j < 1)
  {
    //Serial.println("state");
    if (lidarSerial.available())
    {
      //Serial.println("1");
      if (lidarSerial.read() == HEADER)
      {
        // Serial.println("2");
        uart[0] = HEADER;
        if (lidarSerial.read() == HEADER)
        {
          //Serial.println("3");
          uart[1] = HEADER;
          for (i = 2; i < 9; i++)
            uart[i] = lidarSerial.read(); //Serial.println(uart[i]);}
          check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
          if (uart[8] < 500)
          {
            dist = uart[2] + uart[3] * 256;
            uart[2] = 0, uart[3] = 0;
            if (dist < 1000 && dist > 30)
            {
              a[j] = dist;
              dist = 0;
              j++;
            }
          }
          else
          {
            Serial.println("error data/");
          }
        }
      }
    }
  }////end of tfmini data


  dist = 0;
  lidarSerial.end();
  for ( j = 0; j < 1; j++)
  {
    dist += a[j];
  }
  dist = dist / 1;
  
  if (dist < 1000 && dist > 30)
  {
    Serial.print("dist=");
    Serial.print('\n');
    Serial.print(dist);
    Serial.print('\n');
  }
  else
  {
    Serial.println("dist=");
    Serial.print('\n');
    Serial.println("error reading");
    Serial.print('\n');
    dist = 0;
  }
}
void Receive_GPS_Data()
{
  //Serial.end();
  mySerial.begin(9600);
  while (finish == 0) 
  {
    while (mySerial.available() > 0) {     // Check GPS data
      Gpsdata = mySerial.read();
      //Serial.print(char(Gpsdata));
      flg = 1;
      if ( Gpsdata == '$' && pos_cnt == 0) // finding GPRMC header
        pos_cnt = 1;
      if ( Gpsdata == 'G' && pos_cnt == 1)
        pos_cnt = 2;
      if ( Gpsdata == 'P' && pos_cnt == 2)
        pos_cnt = 3;
      if ( Gpsdata == 'R' && pos_cnt == 3)
        pos_cnt = 4;
      if ( Gpsdata == 'M' && pos_cnt == 4)
        pos_cnt = 5;
      if ( Gpsdata == 'C' && pos_cnt == 5 )
        pos_cnt = 6;
      if (pos_cnt == 6 &&  Gpsdata == ',') { // count commas in message
        com_cnt++;
        flg = 0;
      }

      if (com_cnt == 3 && flg == 1) {
        lat[lat_cnt++] =  Gpsdata;         // latitude
        flg = 0;
      }

      if (com_cnt == 5 && flg == 1) {
        lg[log_cnt++] =  Gpsdata;         // Longitude
        flg = 0;
      }

      if ( Gpsdata == '*' && com_cnt >= 5) {
        com_cnt = 0;                      // end of GPRMC message
        lat_cnt = 0;
        log_cnt = 0;
        flg     = 0;
        finish  = 1;
        mySerial.end();
        //Serial.begin(9600);
      }
    }
  }
    Serial.print("Latitude  :");
    Serial.print('\n');
    Serial.print(lat);
    Serial.print('\n');
    Serial.print("Longitude :");
    Serial.print('\n');
    Serial.print(lg);
    Serial.print('\n');
    finish = 0; pos_cnt = 0;
}

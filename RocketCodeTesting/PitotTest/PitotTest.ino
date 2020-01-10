
#include <Wire.h>   //I2C library 0x28H 
byte fetch_pressure(unsigned int *p_Pressure); //convert value to byte data type

#define TRUE 1
#define FALSE 0

void setup(void)
{
  Serial.begin(115200);
  Wire.begin();
  delay(500);
  Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>");  // just to be sure things are working
}

void loop()
{
  byte _status;
  unsigned int P_dat;
  unsigned int T_dat;
  double PR;
  double TR;
  double V;
  double VV;
  unsigned int count = 0;
  long total = 0;
  
  while (1)
  {
    _status = fetch_pressure(&P_dat, &T_dat);

    switch (_status)
    {
      case 0:
        break;
      case 1: Serial.println("Busy");
        break;
      case 2: Serial.println("Slate");
        break;
      default: Serial.println("Error");
        break;
    }

    PR = (double)((P_dat-8130.8)/(14744.7)) ;
    Serial.println(PR,10);
    total = total + P_dat;
    count++;
    
     PR = abs(PR);
  
     V = ((PR*13789.5144)/1.225);
    VV = (sqrt(V));

    
    TR = (double)((T_dat*0.09770395701));
    TR = TR-50;

   Serial.print("raw Pressure:");
   Serial.println(P_dat);
   //Serial.println(P_dat,DEC);
   //Serial.println(P_dat,BIN);
   Serial.print("pressure psi:");
   Serial.println(PR,10);
   Serial.print("raw Temp:");
   Serial.println(T_dat);
   Serial.print("speed m/s : ");
   Serial.println(VV,5);

    delay(100);
  }
}

byte fetch_pressure(unsigned int *p_P_dat, unsigned int *p_T_dat)
{
  byte address, Press_H, Press_L, _status;
  unsigned int P_dat;
  unsigned int T_dat;

  address = 0x28;
  //Wire.beginTransmission(address);
  //Wire.endTransmission();
  //delay(5);
  Wire.requestFrom((int)address, (int) 4);//Request 4 bytes need 4 bytes are read
  Press_H = Wire.read();
  Press_L = Wire.read();
  byte Temp_H = Wire.read();
  byte  Temp_L = Wire.read();
  //Wire.endTransmission();


  _status = (Press_H >> 6) & 0x03;
  Press_H = Press_H & 0x3f;
  P_dat = (((unsigned int)Press_H) << 8) | Press_L;
  *p_P_dat = P_dat;

  Temp_L = (Temp_L >> 5);
  T_dat = (((unsigned int)Temp_H) << 3) | Temp_L;
  *p_T_dat = T_dat;
  return (_status);
}

#include <DHT.h>
#include <OneWire.h>
#include <DS3231.h>
#include <BMP180.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#define DHTPIN 2     // what pin we're connected to
#define DHTTYPE DHT11   // DHT 11

BMP180  sensor;
OneWire  ds(3);  // подключение ds18s20 на pin 3 (a 4.7K resistor is necessary) Можно подключать к любому цифровому пину
DS3231  rtc(SDA, SCL);
//iarduino_RTC time(RTC_DS3231);
DHT dht(DHTPIN, DHTTYPE);

  LiquidCrystal lcd(4, 5, 10, 11, 12, 13);
int   msec=5000;
long period=0;
float pressureArray[4];
//float pres_1;    
float delta_press=0; //переменная для сохранения разницы в давлении
   
   byte degree[8]={
    B10000,
    B00110,
    B01001,
    B01000,
    B01000,
    B01001,
    B00110,
    B00000,
   };
    byte temp[8]={
    B01001,
    B01000,
    B01000,
    B11100,
    B01000,
    B01001,
    B00110,
    B00000,
   }; 
    void setup() 
    {
        Serial.begin(9600);
        delay(1000);
        sensor.begin(205); //высота над уровнем моря
  //  Serial.begin(115200);
      rtc.begin();
      dht.begin();
//      x=0;  
        lcd.createChar(0,degree);
        lcd.createChar(1,temp);
        lcd.begin(16, 2);
       // lcd.print("t");
        lcd.write((byte)1);
        lcd.setCursor(0, 0);
        //lcd.setCursor(9, 0);
       //lcd.write((byte)0);
        lcd.setCursor(0, 1);
        lcd.print("p         h  %");
    }
    void pressure_arrow()
      {      
            lcd.setCursor(5, 1);
            if (delta_press>1)
              {
                lcd.print("\xDA");
              }
              else if (delta_press<-1)
              {
                lcd.print("\xD9");
              }
              else 
              {
                lcd.print("\x20"); //если давл.не изм, то пробел
              }
        }
        
    void loop() 
    {
  //установки для ds18b20
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius;
  //для DHT
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  // check if returns are valid, if they are NaN (not a number) then something went wrong!
  if (isnan(t) || isnan(h)) {
    Serial.println("Failed to read from DHT");
  }
      // Send Day-of-Week
  Serial.print(rtc.getDOWStr());
  Serial.print(" ");
  // Send date
  Serial.print(rtc.getDateStr());
  Serial.print(" -- ");
  // Send time
  Serial.println(rtc.getTimeStr());
// char x=rtc.getTimeStr();
//  if x==23:10 {Serial.print("zzzzzzzzzzzzzzzzzzzzzzzz");}
  //Serial.print("Temperature: ");
  //Serial.print(rtc.getTemp());
  //Serial.println(" C");
  if ( !ds.search(addr)) {
//    Serial.println("No more addresses.");
//    Serial.println();
    ds.reset_search();
    delay(250);
    return;
  } 

 if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  ds.reset();
  ds.select(addr);
  ds.write(0x44);        // start conversion, with parasite power on at the end
  
  delay(950);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad
for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
//   Serial.print(data[i], HEX);
//   Serial.print(" ");
  }
  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0; // гр.Ц. для ds18b20
  
     if(sensor.read()) {Serial.println((String)"CEHCOP BMP180: P="+sensor.pres+" MM.PT.CT - T="+sensor.temp+" *C - B="+sensor.alti+" M."  "+delta_press+" +celsius);}
      else {Serial.println(  "CEHCOP BMP180: HET OTBETA");}
      lcd.setCursor(2, 0);
      lcd.print(round(sensor.temp));
      lcd.setCursor(2, 1);
      lcd.print(round(sensor.pres));
      lcd.setCursor(12, 1);
      lcd.print(round(h)); //влажность с датчика DHT11
      lcd.setCursor(6, 0);
    //  lcd.print(round(dht.readTemperature())); //температура с датчика DHT11 на LCD, можно испльзовать переменную t в которую считывается показание датчика
      lcd.print("\x20\x20\x20"); //затираем тремя пробелами, чтобы при переходе от десятков градусов к единицам не оставались единицы во втором разряде
      lcd.setCursor(6, 0); // устанавливаем курсор обратно в позицию вывода темп.с датчика 18b20
      lcd.print(round(celsius)); //температура с ds18b20
      lcd.setCursor(11, 0);
      lcd.print(rtc.getTimeStr(FORMAT_SHORT));

      if (period==0) 
        {
            pressureArray[0]=sensor.pres;
            delta_press=pressureArray[0]-pressureArray[3];
            void pressure_arrow();
         }
      
      if (period==3600000)
          {
            pressureArray[1]=sensor.pres;
            delta_press=pressureArray[1]-pressureArray[2];
            void pressure_arrow();
          }
      if (period==7200000)
          {
            pressureArray[2]=sensor.pres;
            delta_press=pressureArray[2]-pressureArray[1];
            void pressure_arrow();
          }      
      if (period==10800000) //period 3 hours
          {
            pressureArray[3]=sensor.pres;
            delta_press=pressureArray[3]-pressureArray[0];
            void pressure_arrow();       
            period=0;
          }
      period=period+msec;
      
      lcd.setCursor(6, 1);
      if (sensor.pres>755)
        {
          lcd.print("H");
        }
        else if (sensor.pres<745)
        {
          lcd.print("L");
        }
        else
        {
          lcd.print("N");
          }
          delay(msec);
      }
// в библиотеке доступны всего две функции:
// sensor.begin();  ИНИЦИАЛИЗАЦИЯ СЕНСОРА (функция проверяет наличие сенсора и читает из него калибровочные коэффициенты для расчетов)
// sensor.begin(160); функция может принимать параметр float начальная высота, например над уровнем моря (по умолчанию = 0)
//      функцию достаточно вызвать 1 раз, но не ранее чем через 10мс после подачи напряжения питания на BMP180
// sensor.read(); ЧТЕНИЕ ПАРАМЕТРОВ СЕНСОРА (функция читает данные сенсора и пересчитывает их в соответствии с калибровочными коэффициентами)
// sensor.read(0);  функция может принимать параметр 0 , 1 , 2 или 3 , по умолчанию 3 (точность расчетов 0-минимальная ... 3-максимальная), чем меньше точность, тем быстрее происходит обработка и чтение параметров
//      функция возвращает true или false в зависимости от реакции сенсора. Так же возвращает false если ни разу не вызывалась функция sensor.begin();
// результаты хранятся в следующих переменных:
// sensor.pres    float - давление в мм.рт.ст.
// sensor.temp    float - температура в *С
// sensor.alti    float - высота относительно начальной (например над уровнем моря)


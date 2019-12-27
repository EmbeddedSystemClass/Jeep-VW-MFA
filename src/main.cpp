#define GPS_Port Serial2
#include <M5Stack.h>

// #include <LoRaWan.h>

#include <TinyGPS++.h>
#include <HardwareSerial.h>

#include <ESP32CAN.h>
#include <CAN_config.h>

#include "images/glowplug.h"
#include "images/cel.h"
#include "images/cctrl.h"
#include "images/battery.h"
#include "images/warning.h"

CAN_device_t CAN_cfg;             // CAN Config
unsigned long previousMillis = 0; // will store last time a CAN Message was send
const int interval = 1000;        // interval at which send CAN Messages (milliseconds)
const int rx_queue_size = 10;     // Receive Queue size
CAN_frame_t rx_frame;

//initial values must be different to force initial rendering
bool genLamp = true,
     _genLamp = false,
     glwLamp = true,
     _glwLamp = false,
     sysLamp = true,
     _sysLamp = false,
     checkEngineLamp = true,
     _checkEngineLamp = false,
     cruiseLamp = true,
     _cruiseLamp = false,
     gpsPolled = true;
int coolantTemp = 0,
    _coolantTemp = 1,
    idleSpeed = 0,
    _idleSpeed = 1,
    intakeTemp = 0,
    _intakeTemp = 1;
// canVersion = 999,
// accPetalPos = 999,
// _accPetalPos = 0;
uint16_t engineSpeed = 0, _engineSpeed = 1;
uint16_t gpsPollCount = 0;
float boost = -0.1, _boost = 1.1;

//fuel consumption
uint16_t fuel_last = 0;
unsigned long time_last = 0;
const int FUEL_AVG_BUFFER = 20;
double fuel_buffer[FUEL_AVG_BUFFER]; // buffer locations will be 0 - 9
int fuel_pointer = 0;
double fuel_consumption = 0.0, _fuel_consumption = 1.1;

TinyGPSPlus gps;
static const uint32_t GPSBaud = 9600;

// char lorabuffer[256];

void updateScreen()
{
  if (cruiseLamp != _cruiseLamp)
  {
    _cruiseLamp = cruiseLamp;
    M5.Lcd.fillRect(0, 0, 50, 50, BLACK);
    if (cruiseLamp)
    {
      M5.Lcd.drawXBitmap(0, 0, cctrl_bits, cctrl_width, cctrl_height, DARKGREEN);
    }
  }

  if (sysLamp != _sysLamp)
  {
    _sysLamp = sysLamp;
    M5.Lcd.fillRect(50, 0, 50, 50, BLACK);
    if (sysLamp)
    {
      M5.Lcd.drawXBitmap(50, 0, warning_bits, warning_width, warning_height, ORANGE);
    }
  }

  if (checkEngineLamp != _checkEngineLamp)
  {
    _checkEngineLamp = checkEngineLamp;
    M5.Lcd.fillRect(100, 0, 50, 50, BLACK);
    if (checkEngineLamp)
    {
      M5.Lcd.drawXBitmap(100, 5, cel_bits, cel_width, cel_height, ORANGE);
    }
  }

  if (_glwLamp != glwLamp)
  {
    _glwLamp = glwLamp;
    M5.Lcd.fillRect(150, 0, 50, 50, BLACK);

    if (glwLamp)
    {
      M5.Lcd.drawXBitmap(150, 0, glowplug_bits, glowplug_width, glowplug_height, TFT_ORANGE);
    }
  }

  if (genLamp != _genLamp)
  {
    _genLamp = genLamp;
    M5.Lcd.fillRect(200, 0, 50, 50, BLACK);
    if (genLamp)
    {
      M5.Lcd.drawXBitmap(200, 5, battery_bits, battery_width, battery_height, RED);
    }
  }

  //draw border of status lights
  M5.Lcd.drawRect(0, 0, 250, 50, TFT_DARKGREY);

  // fuel consumption
  if (_fuel_consumption != fuel_consumption)
  {
    _fuel_consumption = fuel_consumption;
    M5.Lcd.fillRect(250, 0, 70, 50, BLACK);
    M5.Lcd.setCursor(250, 0);
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(TFT_DARKGREEN);
    M5.Lcd.printf("%2.1f", fuel_consumption);
    M5.Lcd.setCursor(250, 30);
    M5.Lcd.setTextSize(2);
    M5.Lcd.printf("l/h");
  }

  //write out the engine speed
  if (_engineSpeed != engineSpeed)
  {
    _engineSpeed = engineSpeed;
    M5.Lcd.fillRect(0, 50, 160, 65, BLACK);
    M5.Lcd.drawRect(0, 50, 160, 65, TFT_DARKGREY);
    M5.Lcd.setTextColor(TFT_DARKGREEN);
    M5.Lcd.setCursor(8, 50);
    M5.Lcd.setTextSize(5);
    M5.Lcd.printf("%i", engineSpeed);
    M5.Lcd.setCursor(60, 100);
    M5.Lcd.setTextSize(2);
    M5.Lcd.printf("RPM");
  }

  //draw outline around coolant temp
  if (_coolantTemp != coolantTemp)
  {
    _coolantTemp = coolantTemp;
    M5.Lcd.fillRect(160, 50, 160, 65, BLACK);
    M5.Lcd.drawRect(160, 50, 160, 65, TFT_DARKGREY);
    M5.Lcd.setTextColor(TFT_DARKGREEN);
    M5.Lcd.setCursor(190, 50);
    M5.Lcd.setTextSize(6);
    M5.Lcd.printf("%i", coolantTemp);
    M5.Lcd.setCursor(220, 100);
    M5.Lcd.setTextSize(2);
    M5.Lcd.printf("CTS");
  }

  //write out intake temp
  if (_intakeTemp != intakeTemp)
  {
    _intakeTemp = intakeTemp;
    M5.Lcd.fillRect(0, 115, 160, 65, BLACK);
    M5.Lcd.drawRect(0, 115, 160, 65, TFT_DARKGREY);
    M5.Lcd.setTextColor(TFT_DARKGREEN);
    M5.Lcd.setCursor(8, 115);
    M5.Lcd.setTextSize(6);
    M5.Lcd.printf("%i", intakeTemp);
    M5.Lcd.setCursor(60, 165);
    M5.Lcd.setTextSize(2);
    M5.Lcd.printf("IAT");
  }

  //accel_pos
  // if (_accPetalPos != accPetalPos)
  // {
  //   _accPetalPos = accPetalPos;
  //   M5.Lcd.fillRect(160, 115, 160, 65, BLACK);
  //   M5.Lcd.drawRect(160, 115, 160, 65, TFT_DARKGREY);
  //   M5.Lcd.setTextColor(TFT_DARKGREEN);
  //   M5.Lcd.setCursor(190, 115);
  //   M5.Lcd.setTextSize(6);
  //   M5.Lcd.printf("%i", accPetalPos);
  //   M5.Lcd.setCursor(210, 165);
  //   M5.Lcd.setTextSize(2);
  //   M5.Lcd.printf("AccPos");
  // }
  if (gpsPolled)
  {
    gpsPolled = false;
    M5.Lcd.fillRect(160, 115, 160, 65, BLACK);
    M5.Lcd.drawRect(160, 115, 160, 65, TFT_DARKGREY);
    if (gps.speed.isValid())
    {
      M5.Lcd.setTextColor(TFT_DARKGREEN);
      M5.Lcd.setCursor(163, 115);
      M5.Lcd.setTextSize(8);
      M5.Lcd.printf("%3.0f", gps.speed.mph());
      M5.Lcd.setTextSize(2);
      //160 is the beginning
      //each character is 5x5? (*scale)

      M5.Lcd.setCursor(184, 166);
      M5.Lcd.printf("GPS MPH");
    }
    else
    {
      M5.Lcd.setTextColor(RED);
      M5.Lcd.setCursor(163, 115);
      M5.Lcd.setTextSize(4);
      M5.Lcd.printf("NO FIX");
      uint16_t color = (uint16_t) random(65535);
      M5.Lcd.setTextColor(color);
      M5.Lcd.setCursor(163, 145);
      M5.Lcd.printf("sats:%i", gps.satellites.value());
    }
  }

  if (_boost != boost)
  {
    _boost = boost;
    M5.Lcd.fillRect(0, 180, 80, 60, BLACK);
    M5.Lcd.setTextColor(TFT_DARKGREEN);
    M5.Lcd.setCursor(3, 182);
    if (boost > 10)
    {
      M5.Lcd.setTextSize(3);
      M5.Lcd.printf("%2.1f", boost);
    }
    else if (boost < 0)
    {
      M5.Lcd.setTextSize(4);
      M5.Lcd.printf("0");
    }
    else
    {
      M5.Lcd.setTextSize(4);
      M5.Lcd.printf("%1.1f", boost);
    }
    M5.Lcd.setCursor(13, 217);
    M5.Lcd.setTextSize(2);
    M5.Lcd.printf("Boost");
    M5.Lcd.drawRect(0, 180, 80, 60, TFT_DARKGREY);
  }

  if (_idleSpeed != idleSpeed)
  {
    _idleSpeed = idleSpeed;
    M5.Lcd.fillRect(80, 180, 80, 60, BLACK);
    M5.Lcd.setTextColor(TFT_DARKGREEN);
    M5.Lcd.setCursor(83, 183);
    M5.Lcd.setTextSize(3);
    M5.Lcd.printf("%i", idleSpeed);
    M5.Lcd.setCursor(83, 207);
    M5.Lcd.setTextSize(2);
    M5.Lcd.println("idle");
    M5.Lcd.setCursor(83, 222);
    M5.Lcd.println("speed");
    M5.Lcd.drawRect(80, 180, 80, 60, TFT_DARKGREY);
  }
  M5.Lcd.fillRect(160, 180, 160, 60, BLACK);
  M5.Lcd.setCursor(160, 180);
  M5.Lcd.setTextSize(2);
  M5.Lcd.printf("Sat count:%i", gps.satellites.value());
  M5.Lcd.setCursor(160, 200);
  M5.Lcd.printf("GPSTime:%02d:%02d", gps.time.hour(), gps.time.minute());
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(160, 220);
  // M5.Lcd.printf("%02d:%02d:%02d", , gps.time.second());
}

bool maskToBool(int message, int mask)
{
  return (message & mask) == mask;
}

//GPS Delay
static void gpsRead(unsigned long ms)
{
  unsigned long start = millis();
  do
  {

    while (GPS_Port.available())
    {
      int res = GPS_Port.read();
      gps.encode(res);
      // printf("%c",(char)res);
      gpsPollCount++;
    }
  } while (millis() - start < ms);
}

// the setup routine runs once when M5Stack starts up
void setup()
{
  //lorawan
  // rx = 5
  // tx = 26
  // lora.init();
  // lora.setDeviceReset();
  // memset(lorabuffer, 0, 256);
  // lora.getVersion(lorabuffer, 256, 10);
  // printf("LoRaWan Version:");
  // for(char x : lorabuffer){
  //   printf("%c", x);
  // }
  // printf("\n");

  // initialize the M5Stack object
  M5.begin();
  M5.Power.begin();

  M5.Lcd.setTextSize(8);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.println("Initializing...");
  printf("initializing\n");

  // Init CAN Module
  printf("Setting up CAN\n");
  CAN_cfg.speed = CAN_SPEED_500KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_5;
  CAN_cfg.rx_pin_id = GPIO_NUM_2;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  ESP32Can.CANInit();
  printf("CAN set up\n");

  // LCD Init section
  printf("Setting up initial LCD layout\n");
  M5.Lcd.fillScreen(BLACK);
  updateScreen();
  //used for random color when waiting for GPS fix
  randomSeed(analogRead(0));
  printf("LCD set up!\n");

  //gps
  // rx = 17
  // tx = 16
  printf("Setting up GPS\n");
  GPS_Port.begin(GPSBaud, SERIAL_8N1, 16, 17);
  printf("GPS Set up!\n");
  while (!GPS_Port)
    ;

  printf("Fully initialized!!\n");
}
void canRx()
{
  if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
  {
    switch (rx_frame.MsgID)
    {
    case 0x280:
    {
      int rpm_high = rx_frame.data.u8[3];
      int rpm_low = rx_frame.data.u8[2];
      engineSpeed = ((rpm_high << 8) | rpm_low) / 4;

      // accPetalPos = (rx_frame.data.u8[5] * 100) / 255; //0x00 == 0%, 0xff == 100%
      updateScreen();
    }
    break;
    case 0x288:
    {
      // int eng2MuxInfo = rx_frame.data.u8[0];
      // int eng2MuxData = eng2MuxInfo & 0x3f;            //first 6 bits
      // int eng2MuxDataType = (eng2MuxInfo & 0xc0) >> 6; // bits 7-8
      // if (eng2MuxDataType == 0x00)
      // {
      //   canVersion = eng2MuxData;
      //   //FrmMng_dMulinfo2_0_C - This is the CAN version number == 16 (0x10) on BHW
      // }
      // Bit addr. 8, bit no. 8, initial value 0,
      // Corresponds to the message CTSCD_tClnt, which contains the fuel temperature FTSCD_tFuel (broken link) in the event of a defective coolant temperature sensor and corresponding application. FrmMng_t_Offset_C and FrmMng_facT_Slope_C are used as conversion parameters.
      // The statuses “coolant temperature ok”, “coolant temperature contains substitute value fuel temperature” and “coolant temperature defective” are determined via CTSCD_stCanSndTemp. In the event of an error (“Coolant temperature defective”) the error value FrmMng_RawErr_C is sent via CAN.
      coolantTemp = ((float)rx_frame.data.u8[1] * 0.75) - 48;
      // Idle speed
      // Bit addr. 40, bit no. 8, initial value 0,
      // Corresponds to the message LIGov_nSetpoint. FrmMng_nLI_Offset_C and FrmMng_facNLI_Slope_C are used as conversion parameters.
      idleSpeed = rx_frame.data.u8[5] * 10;
      updateScreen();
    }
    break;
    case 0x380:
    {
      intakeTemp = ((float)rx_frame.data.u8[1] * 0.75) - 48;
      updateScreen();
    }
    break;
    case 0x480: //message that contains glow lamp
    {
      genLamp = maskToBool(rx_frame.data.u8[1], 0x01);
      glwLamp = maskToBool(rx_frame.data.u8[1], 0x02);          // S_LGLW
      sysLamp = maskToBool(rx_frame.data.u8[1], 0x04);          // S_LSYS
      checkEngineLamp = maskToBool(rx_frame.data.u8[1], 0x080); // S_LODBII
      cruiseLamp = maskToBool(rx_frame.data.u8[6], 0x04);       // L_CCTL

      //                bool coolantTempLamp = maskToBool(rx_frame.data.u8[1],0x010); //S_CLNTTMP
      //                bool s_aco = maskToBool(rx_frame.data.u8[1],0x20);
      //                bool s_clg = maskToBool(rx_frame.data.u8[1],0x40);
      //                bool s_aclow = maskToBool(rx_frame.data.u8[1],0x80);
      //Fuel consumption math is hard
      double fuel_used;
      uint16_t fuel_now = (((rx_frame.data.u8[3] & 0x7f) << 8) + rx_frame.data.u8[2]); // get current fuel reading
      unsigned long time_now = millis();                                               // get curent time

      if (time_last != 0)
      {
        if (fuel_now >= fuel_last)
        { // to allow for roll over
          fuel_used = fuel_now - fuel_last;
        }
        else
        {
          fuel_used = (fuel_now + (32767 - fuel_last));
        }

        double time_elapsed = time_now - time_last;

        // fuel_used is in ul, so we need devide it with 1e6 to get litres.
        // time_elapsed is ms, so devide it with 3.6e6 to get hours.
        // fuel_rate = ( (fuel_used / 1e6) / ( time_elapsed/3.6e6 ) );
        double fuel_rate = (fuel_used / time_elapsed) * 3.6;

        fuel_buffer[fuel_pointer] = fuel_rate; // store latest reading in buffer (10 deep).
        fuel_pointer++;                        // increment buffer pointer ready for next time

        if (fuel_pointer == FUEL_AVG_BUFFER)
          fuel_pointer = 0; // if end of buffer reached, point to start

        double fuel_average = 0;
        for (int i = 0; i < FUEL_AVG_BUFFER; i++)
        { // do this 10 times
          fuel_average += fuel_buffer[i];
        } // add all 10 numbers stored in buffer
        fuel_consumption = fuel_average / FUEL_AVG_BUFFER;
      }
      time_last = time_now;
      fuel_last = fuel_now;
    }
      updateScreen();
      break;
    case 0x588:
    {
      // mbar to psi, compensating for atmosheric pressure
      boost = ((float)rx_frame.data.u8[4] * 0.145037738) - 14.7;
      updateScreen();
    }
    break;
    default:
      break;
    }
  }
}
// the loop routine runs over and over again forever
void loop()
{
  // Receive next CAN frame from queue
  canRx();
  gpsRead(100);
  gpsPolled = true;
  if (gps.speed.isUpdated())
  {
    updateScreen();
  }
  
}
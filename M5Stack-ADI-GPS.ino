#define M5STACK_MPU6886
#include <M5Stack.h>

#include <TinyGPSPlus.h>
TinyGPSPlus gps;

#define LGFX_AUTODETECT
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>
static LGFX lcd;
static LGFX_Sprite horizon(&lcd);
static LGFX_Sprite altimeter(&horizon);
static LGFX_Sprite pointer(&horizon);
static LGFX_Sprite variometer(&horizon);
static LGFX_Sprite heading(&horizon);
static LGFX_Sprite gspeed(&horizon);

#define XORG  160
#define YORG  120
#define PITCH 220

#define PBLACK  0
#define PSKY    1
#define PGND    2
#define PNEEDLE 3
#define PWHITE  255

uint8_t displayOption;
uint8_t debugOption;
uint16_t gpsValidCount;
uint16_t fps;
uint16_t debugCount;

float pitch, pitch0;
float roll, roll0;
float yaw, yaw0;
float Sin, Cos, Psin;

float altitude;
float climbRate;

void Btn()
{
  M5.update();
  if(M5.BtnC.wasPressed()){
    IMUReset();
  }
  else if(M5.BtnB.wasPressed()){
    ++debugOption %= 3;
    if(debugOption == 2){
      lcd.clear();
      lcd.setCursor(0, 0);
      debugCount = 0;
    }
  }
  else if(M5.BtnA.wasPressed()){
    ++displayOption %= 2;
  }
}

void Imu()
{
  float p, r, y;

  M5.IMU.getAhrsData(&r, &p, &y);
  
  pitch = (pitch * 4 + p - pitch0) / 5;   // moving average
  roll  = (roll  * 4 + r -  roll0) / 5;
  yaw   = y - yaw0;

  Sin = sin(PI * -roll / 180);
  Cos = cos(PI * -roll / 180);
  Psin =sin(PI * pitch / 180);
}

void IMUReset()
{
  float p, r, y;

  M5.IMU.getAhrsData(&r, &p, &y);
  pitch0 = p;
  roll0  = r;
  yaw0   = y;
}

void Gps()
{
  static unsigned long start;
  static float prevAltitude;

  while(Serial2.available() > 0)
    gps.encode(Serial2.read());

  if(debugOption == 1){
    altitude += 0.1;
  }
  else{
    if(gps.altitude.isValid()){
      altitude = gps.altitude.meters();
      gpsValidCount++;
    }
  }

  if(altitude > 999.0)
    altitude = 999.0;
  else if(altitude < -10)
    altitude = -10.0;

  if(millis() - start > 1000){
    climbRate = (climbRate*4 + altitude - prevAltitude) / 5;
    prevAltitude = altitude;
    start = millis();
  }
}

void gpsMonitor()
{
  lcd.setTextSize(2);
  lcd.setCursor(50, 140);
  if (gps.time.isValid())
  {
    lcd.printf("%02d:", gps.time.hour());
    lcd.printf("%02d:", gps.time.minute());
    lcd.printf("%02d", gps.time.second());
  }
  else
  {
    lcd.print(F("INVALID"));
  }

  lcd.setCursor(50, 160);
  lcd.printf("%d", gpsValidCount);
  lcd.setCursor(50, 180);
  lcd.printf("fps:%d", fps);
}

void rotate(int *X, int *Y, int x, int y)
{
  *X = x * Cos - y * Sin;
  *Y = x * Sin + y * Cos;
}

uint8_t Zero[] = {
  0,0,0,1,1,0,0,0,
  0,0,1,1,1,1,0,0,
  0,1,1,0,0,1,1,0,
  1,1,0,0,0,0,1,1,
  1,1,0,0,0,0,1,1,
  1,1,0,0,0,0,1,1,
  1,1,0,0,0,0,1,1,
  1,1,0,0,0,0,1,1,
  1,1,0,0,0,0,1,1,
  0,1,1,0,0,1,1,0,
  0,0,1,1,1,1,0,0,
  0,0,0,1,1,0,0,0,
};

uint8_t One[] = {
  0,0,0,1,1,0,0,0,
  0,0,1,1,1,0,0,0,
  0,0,1,1,1,0,0,0,
  0,0,0,1,1,0,0,0,
  0,0,0,1,1,0,0,0,
  0,0,0,1,1,0,0,0,
  0,0,0,1,1,0,0,0,
  0,0,0,1,1,0,0,0,
  0,0,0,1,1,0,0,0,
  0,0,0,1,1,0,0,0,
  0,0,0,1,1,0,0,0,
  0,0,0,1,1,0,0,0,
};

uint8_t Two[] = {
  0,1,1,1,1,1,0,0,
  1,1,1,1,1,1,1,0,
  1,1,0,0,0,1,1,1,
  0,0,0,0,0,0,1,1,
  0,0,0,0,0,0,1,1,
  0,0,1,1,1,1,1,0,
  0,1,1,1,1,1,0,0,
  1,1,1,0,0,0,0,0,
  1,1,0,0,0,0,0,0,
  1,1,0,0,0,0,0,0,
  1,1,1,1,1,1,1,1,
  1,1,1,1,1,1,1,1,
};

uint8_t Three[] = {
  0,1,1,1,1,1,0,0,
  1,1,1,1,1,1,1,0,
  1,1,0,0,0,1,1,1,
  0,0,0,0,0,0,1,1,
  0,0,0,1,1,1,1,0,
  0,0,0,1,1,1,1,0,
  0,0,0,0,0,1,1,0,
  0,0,0,0,0,0,1,1,
  0,0,0,0,0,0,1,1,
  1,1,0,0,0,1,1,0,
  1,1,1,1,1,1,1,0,
  0,1,1,1,1,1,0,0,
};

void draw_Char(int x, int y, uint8_t c[])
{
  int ix, iy, X, Y;
  int p = PITCH * Psin;

  if(y+7+p < -90 || y+7+p > 90)
    return;

  for(iy = 0; iy < 12; iy++){
    for(ix = 0; ix < 8; ix++){
      if(c[iy*8+ix] != 0){
        rotate(&X, &Y, x+ix, y+iy+p);
        horizon.drawPixel(XORG+X, YORG+Y, PWHITE);
      }
    }
  }
}

void draw_Line(int y, int len)
{
  int x = - len / 2;
  int p = PITCH * Psin;
  int X0, X1, Y0, Y1;

  if(y+p < -90 || y+p > 90)
    return;

  rotate(&X0, &Y0, -len/2, y+p);
  rotate(&X1, &Y1,  len/2, y+p);
  horizon.drawLine(XORG+X0, YORG+Y0, XORG+X1, YORG+Y1, PWHITE);
}

#define PX1 32
#define PX2 42

#define PY0 0
#define PY1 20
#define PY2 40
#define PY3 60
#define PY4 80
#define PY5 100
#define PY6 120

#define PL1 28
#define PL2 56

void draw_Pitch()
{
  draw_Char( PX1  ,  PY6-6, Three);
  draw_Char( PX2  ,  PY6-6, Zero);
  draw_Line(         PY6, PL2);
  draw_Char(-PX2-8,  PY6-6, Three);
  draw_Char(-PX1-8,  PY6-6, Zero);

  draw_Line(         PY5, PL1);

  draw_Char( PX1,    PY4-6, Two);
  draw_Char( PX2,    PY4-6, Zero);
  draw_Line(         PY4, PL2);
  draw_Char(-PX2-8,  PY4-6, Two);
  draw_Char(-PX1-8,  PY4-6, Zero);

  draw_Line(         PY3, PL1);

  draw_Char( PX1,    PY2-6, One);
  draw_Char( PX2,    PY2-6, Zero);
  draw_Line(         PY2, PL2);
  draw_Char(-PX2-8,  PY2-6, One);
  draw_Char(-PX1-8,  PY2-6, Zero);

  draw_Line(         PY1, PL1);

  draw_Char( PX1,    PY0-6, Zero);
  draw_Line(         0, PL2);
  draw_Char(-PX1-8,  PY0-6, Zero);

  draw_Line(        -PY1, PL1);

  draw_Char( PX1,   -PY2-6, One);
  draw_Char( PX2,   -PY2-6, Zero);
  draw_Line(        -PY2, PL2);
  draw_Char(-PX2-8, -PY2-6, One);
  draw_Char(-PX1-8, -PY2-6, Zero);

  draw_Line(        -PY3, PL1);

  draw_Char( PX1,   -PY4-6, Two);
  draw_Char( PX2,   -PY4-6, Zero);
  draw_Line(        -PY4, PL2);
  draw_Char(-PX2-8, -PY4-6, Two);
  draw_Char(-PX1-8, -PY4-6, Zero);

  draw_Line(        -PY5, PL1);

  draw_Char( PX1,   -PY6-6, Three);
  draw_Char( PX2,   -PY6-6, Zero);
  draw_Line(        -PY6, PL2);
  draw_Char(-PX2-8, -PY6-6, Three);
  draw_Char(-PX1-8, -PY6-6, Zero);
}

#define RADIUS 120

void draw_Mark1(int angle)
{
  int x, y, X0, Y0, X1, Y1, X2, Y2;

  x = (RADIUS-13) * cos(PI * angle / 180);
  y = (RADIUS-13) * sin(PI * angle / 180);
  rotate(&X0, &Y0, x, y);
  x = (RADIUS -5) * cos(PI * (angle-2) / 180);
  y = (RADIUS -5) * sin(PI * (angle-2) / 180);
  rotate(&X1, &Y1, x, y);
  x = (RADIUS -5) * cos(PI * (angle+2) / 180);
  y = (RADIUS -5) * sin(PI * (angle+2) / 180);
  rotate(&X2, &Y2, x, y);
  horizon.fillTriangle(XORG+X0, YORG+Y0, XORG+X1, YORG+Y1, XORG+X2, Y2+YORG, PWHITE);
}

void draw_Mark2(int angle)
{
  int x, y, X0, Y0, X1, Y1;

  x = (RADIUS -5) * cos(PI * angle / 180);
  y = (RADIUS -5) * sin(PI * angle / 180);
  rotate(&X0, &Y0, x, y);
  x = (RADIUS-14) * cos(PI * angle / 180);
  y = (RADIUS-14) * sin(PI * angle / 180);
  rotate(&X1, &Y1, x, y);
  horizon.drawLine(XORG+X0, YORG+Y0, XORG+X1, YORG+Y1, PWHITE);
}

void draw_Mark3(int angle)
{
  int x, y, X0, Y0, X1, Y1, X2, Y2, X3, Y3;

  x = (RADIUS -5) * cos(PI * (angle+1) / 180);
  y = (RADIUS -5) * sin(PI * (angle+1) / 180);
  rotate(&X0, &Y0, x, y);
  x = (RADIUS-14) * cos(PI * (angle+1) / 180);
  y = (RADIUS-14) * sin(PI * (angle+1) / 180);
  rotate(&X1, &Y1, x, y);
  x = (RADIUS -5) * cos(PI * (angle-1) / 180);
  y = (RADIUS -5) * sin(PI * (angle-1) / 180);
  rotate(&X2, &Y2, x, y);
  x = (RADIUS-14) * cos(PI * (angle-1) / 180);
  y = (RADIUS-14) * sin(PI * (angle-1) / 180);
  rotate(&X3, &Y3, x, y);
  horizon.fillTriangle(XORG+X0, YORG+Y0, XORG+X1, YORG+Y1, XORG+X2, YORG+Y2, PWHITE);
  horizon.fillTriangle(XORG+X1, YORG+Y1, XORG+X2, YORG+Y2, XORG+X3, YORG+Y3, PWHITE);
}

void draw_Roll()
{
  horizon.drawArc(XORG, YORG, RADIUS-15, RADIUS-15, 180-roll, -roll, PWHITE);

  draw_Mark1(-45);
  draw_Mark1(-135);

  draw_Mark2(-70);
  draw_Mark2(-80);
  draw_Mark2(-100);
  draw_Mark2(-110);

  draw_Mark3(0);
  draw_Mark3(-30);
  draw_Mark3(-60);
  draw_Mark3(-90);
  draw_Mark3(-120);
  draw_Mark3(-150);
  draw_Mark3(-180);
}

void draw_Bezel()
{
  horizon.fillTriangle(XORG+0, YORG-105, XORG-10, YORG-85, XORG+10, YORG-85, PBLACK);
  horizon.drawTriangle(XORG+0, YORG-105, XORG-10, YORG-85, XORG+10, YORG-85, PWHITE);

  horizon.fillRect(XORG-69, YORG-4, 50, 8, PBLACK);
  horizon.drawRect(XORG-69, YORG-4, 50, 8, PWHITE);
  horizon.fillRect(XORG+20, YORG-4, 50, 8, PBLACK);
  horizon.drawRect(XORG+20, YORG-4, 50, 8, PWHITE);

  horizon.fillTriangle(XORG-20, YORG-4, XORG+0, YORG+16, XORG-20, YORG+ 4, PBLACK);
  horizon.fillTriangle(XORG-20, YORG+4, XORG+0, YORG+16, XORG+ 0, YORG+24, PBLACK);
  horizon.drawLine(XORG-20, YORG-4, XORG+0, YORG+16, PWHITE);
  horizon.drawLine(XORG-20, YORG+4, XORG+0, YORG+24, PWHITE);
  horizon.fillTriangle(XORG+20, YORG-4, XORG+0, YORG+16, XORG+20, YORG+ 4, PBLACK);
  horizon.fillTriangle(XORG+20, YORG+4, XORG+0, YORG+16, XORG+ 0, YORG+24, PBLACK);
  horizon.drawLine(XORG+20, YORG-4, XORG+0, YORG+16, PWHITE);
  horizon.drawLine(XORG+20, YORG+4, XORG+0, YORG+24, PWHITE);
}

void draw_BackPlate()
{
  int32_t P;
  int32_t x0, x1, x2, y0, y1, y2;
  int32_t X0, X1, X2, Y0, Y1, Y2;

  horizon.fillSprite(PBLACK);

  horizon.fillRect(0, 0, 320, 240, PGND);
  P = PITCH * sin(PI * pitch / 180);
  x0 = 0; x1 = -480; x2 = 480;
  y0 = -2400; y1 = 0; y2 = 0;
  rotate(&X0, &Y0, x0, y0);
  rotate(&X1, &Y1, x1, y1);
  rotate(&X2, &Y2, x2, y2);
  horizon.fillTriangle(XORG+X0, Y0+P+YORG, XORG+X1, Y1+P+YORG, XORG+X2, Y2+P+YORG, PSKY);
  if(debugOption == 1){
    horizon.setTextSize(2);
    horizon.setTextColor(PWHITE);
    horizon.setCursor(41, 5); horizon.printf("%4.1f", roll);
    horizon.setCursor(215, 5); horizon.printf("%4.1f", pitch);
  }
}

void draw_Altimeter()
{
  char buf[16];
  int alt = altitude * 10.0;

  altimeter.fillRect(0, 0, 40, 240, PBLACK);
  altimeter.setTextSize(2);
  alt -= 120;
  for(int y = 0; y < 240 + 20; y++, alt++){
    if((alt % 100) == 0){
      sprintf(buf, "%03d", alt / 10);
      altimeter.drawString(buf, 4, 239 - y);
    }
    else if((alt % 50) == 0)
      altimeter.drawLine(19, 239 - y, 39, 239 - y, PWHITE);
    else if((alt % 10) == 0)
      altimeter.drawLine(29, 239 - y, 39, 239 - y, PWHITE);
  }
  altimeter.drawRect(0, 0, 40, 240, PWHITE);
}

void draw_Pointer()
{
  char buf[5][4];
  int alt = altitude * 10.0;
  int digit;

  pointer.clear();

  sprintf(buf[0], "%04d", alt-20);
  sprintf(buf[1], "%04d", alt-10);
  sprintf(buf[2], "%04d", alt   );
  sprintf(buf[3], "%04d", alt+10);
  sprintf(buf[4], "%04d", alt+20);

  pointer.setTextSize(3);
  digit = (buf[2][3] - '0') * 1.8;

  if(buf[2][1] - '0' > 8 && buf[2][2] - '0' > 8){
    pointer.drawChar(buf[3][0],  4, 4 -28 + digit);
    pointer.drawChar(buf[2][0],  4, 4     + digit);
    pointer.drawChar(buf[1][0],  4, 4 +28 + digit);
  }
  else{
    pointer.drawChar(buf[2][0],  4, 4);
  }

  if(buf[2][2] - '0' > 8){
    pointer.drawChar(buf[3][1], 22, 4 -28 + digit);
    pointer.drawChar(buf[2][1], 22, 4     + digit);
    pointer.drawChar(buf[1][1], 22, 4 +28 + digit);
  }
  else{
    pointer.drawChar(buf[2][1], 22, 4);
  }

  pointer.setTextSize(2);
  pointer.drawChar(buf[4][2], 42, 8 -32 + digit);
  pointer.drawChar(buf[3][2], 42, 8 -16 + digit);
  pointer.drawChar(buf[2][2], 42, 8     + digit);
  pointer.drawChar(buf[1][2], 42, 8 +16 + digit);
  pointer.drawChar(buf[0][2], 42, 8 +32 + digit);
//  if(debugOption == 1)
//    horizon.drawChar(buf[1][3], 250, 160);
  pointer.drawLine( 0,  0, 58,  0, PWHITE);
  pointer.drawLine(58,  0, 58, 10, PWHITE);
  pointer.drawLine(58, 10, 68, 15, PWHITE);
  pointer.drawLine(69, 15, 58, 20, PWHITE);
  pointer.drawLine(58, 20, 58, 29, PWHITE);
  pointer.drawLine(58, 29,  0, 29, PWHITE);
  pointer.drawLine( 0, 29,  0,  0, PWHITE);
}

void draw_Variometer()
{
  int v = climbRate * 5;

  variometer.setTextSize(2);

  variometer.fillRect(0, 0, 40, 240-2, PBLACK);     // 204 -> 238 (bottom line problem)
  variometer.fillRect(0, 115 - v, 40, 10, PNEEDLE);
  variometer.drawRect(0, 0, 40, 240-2, PWHITE);     // 204 -> 238 (bottom line problem)
  for(int i = -120; i < 120; i++){
    if(((i + 200) % 50) == 0)
      variometer.drawLine(0, 120 - i, 15, 120 - i, PWHITE);
    else if((i % 10) == 0)
      variometer.drawLine(0, 120 - i,  8, 120 - i, PWHITE);
  }
  variometer.drawChar('2', 22,  20-7);
  variometer.drawChar('1', 22,  70-7);
  variometer.drawChar('0', 22, 120-7);
  variometer.drawChar('1', 22, 170-7);
  variometer.drawChar('2', 22, 220-7);
}

void draw_Heading()
{
  heading.fillRect(0, 0, 70, 30, PBLACK);
  heading.drawRect(0, 0, 70, 30, PWHITE);
  heading.setTextSize(3);
  heading.setCursor(5, 3);
  if(debugOption == 1)
    heading.printf("%03d", 59);
  else
    heading.printf("%03d", (int)gps.course.deg());
  heading.drawRect(60, 6, 6, 4, PWHITE);
  heading.drawRect(61, 5, 4, 6, PWHITE);
}

void draw_Gspeed()
{
  gspeed.fillRect(0, 0, 95, 30, PBLACK);
  gspeed.drawRect(0, 0, 95, 30, PWHITE);
  gspeed.setTextSize(3);
  gspeed.setCursor(3, 3);
  if(debugOption == 1)
    gspeed.printf("%3dkm", 118);
  else
    gspeed.printf("%3dkm", (int)gps.speed.kmph());
}

void Draw_All()
{
  draw_BackPlate();
  draw_Pitch();
  draw_Roll();
  draw_Bezel();

  draw_Altimeter();
  draw_Pointer();
  draw_Variometer();
  draw_Heading();
  draw_Gspeed();

  heading.pushSprite(70, 209);
  gspeed.pushSprite(160, 209);
  if(displayOption == 0){
    altimeter.pushSprite(280, 0);
    pointer.pushSprite(249, 105);
    variometer.pushSprite(0, 0);
  }

  horizon.pushSprite(0, 0);
}

void Terminal()
{
  if(debugCount > 26*15)      // size(2):12*26=312(<320) 16*15=240
    return;
  while (Serial2.available()){
    lcd.printf("%c", Serial2.read());
    debugCount++;
  }
}

void setup(void)
{
  M5.begin();
  Wire.begin();
  M5.Power.begin();
//  M5.Power.setPowerBoostSet(true);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);

  M5.IMU.Init();
  for(int i = 0; i < 100; i++)
  {
    Imu();
    delay(10);
  }
  IMUReset();

  lcd.init();
  lcd.setRotation(1);
  lcd.setBrightness(128);
  lcd.setColorDepth(16);

  lcd.clear();
  lcd.startWrite();

  horizon.setColorDepth(8);
  horizon.createPalette();
  horizon.setPaletteColor(PSKY, 0x02F8);
  horizon.setPaletteColor(PGND, 0x8000);
  horizon.createSprite(320, 240);
  
  altimeter.setColorDepth(8);
  altimeter.createPalette();
  altimeter.createSprite(40, 240);
  
  pointer.setColorDepth(8);
  pointer.createPalette();
  pointer.createSprite(69, 30);
  
  variometer.setColorDepth(8);
  variometer.createPalette();
  horizon.setPaletteColor(PNEEDLE, 0x07E0);
  variometer.createSprite(40, 240);

  heading.setColorDepth(8);
  heading.createPalette();
  heading.createSprite(70, 30);

  gspeed.setColorDepth(8);
  gspeed.createPalette();
  gspeed.createSprite(95, 30);
}

void loop(void)
{
  static int start, count;

  Btn();
  if(debugOption == 0){
    Gps();
    Imu();
    Draw_All();
  }
  else if(debugOption == 1){
    Gps();
    Imu();
    Draw_All();
    gpsMonitor();
  }
  else if(debugOption == 2){
    Terminal();
  }

  if(millis() - start > 1000){
    fps = count;
    count = 0;
    start = millis();
  }
  else
    count++;
}

/*
 * 7 segment display driven by arduino mega and ds3231
 *  -   5
 * | |  7 4
 *  -   6
 * | |  1 3
 *  -   2
 *  
 *  : 9
 *  left dot 10
 *  right dot 8
 */
 
#include <Wire.h>
#include "RTClib.h"

#define DIG8_PIN 9
#define DIG9_PIN 10
#define DIG10_PIN 11
#define COM1_PIN 37

#define duration 4000 

#define A 6
#define B 5
#define C 4
#define D 3
#define E 2
#define F 8
#define G 7
#define DP 13

#define disp1 33
#define disp2 34
#define disp3 35
#define disp4 36

#define KEY1 29
#define KEY2 32
#define KEY3 31
#define KEY4 30

#define numbersegments { \
{1,1,1,1,1,1,0,0},\
{0,1,1,0,0,0,0,0},\
{1,1,0,1,1,0,1,0},\
{1,1,1,1,0,0,1,0},\
{0,1,1,0,0,1,1,0},\
{1,0,1,1,0,1,1,0},\
{1,0,1,1,1,1,1,0},\
{1,1,1,0,0,0,0,0},\
{1,1,1,1,1,1,1,0},\
{1,1,1,0,0,1,1,0},\
}

byte numbers[10][8] = numbersegments; 
const int segments[8] = {A, B, C, D, E, F, G, DP};

RTC_DS3231 rtc;

const char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

long lastMsg = 0;

int count = 0;

int digit1 = 0;
int digit2 = 0;
int digit3 = 0;
int digit4 = 0;

void setup() 
{
  Serial.begin(115200);

  

  if (! rtc.begin()) 
  {
    Serial.println("Couldn't find RTC");
    while (1);
  }

/*
  if (rtc.lostPower()) 
  {
    Serial.println("RTC lost power, lets set the time!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
*/

  pinMode(DIG8_PIN, OUTPUT);
  pinMode(DIG9_PIN, OUTPUT);
  pinMode(DIG10_PIN, OUTPUT);

  pinMode(COM1_PIN, OUTPUT);

  pinMode(KEY1, INPUT_PULLUP);
  pinMode(KEY2, INPUT_PULLUP);
  pinMode(KEY3, INPUT_PULLUP);
  pinMode(KEY4, INPUT_PULLUP);

  digitalWrite(DIG8_PIN, LOW);
  digitalWrite(DIG9_PIN, HIGH);
  digitalWrite(DIG10_PIN, LOW);

  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);
  pinMode(D, OUTPUT);
  pinMode(E, OUTPUT);
  pinMode(F, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(DP, OUTPUT);
  pinMode(disp1, OUTPUT);
  pinMode(disp2, OUTPUT);  
  pinMode(disp3, OUTPUT);
  pinMode(disp4, OUTPUT);
  digitalWrite(A, LOW);
  digitalWrite(B, LOW);
  digitalWrite(C, LOW);
  digitalWrite(D, LOW);
  digitalWrite(E, LOW);
  digitalWrite(F, LOW);
  digitalWrite(G, LOW);
  digitalWrite(DP, LOW);
  digitalWrite(disp1, LOW);
  digitalWrite(disp2, LOW);
  digitalWrite(disp3, LOW);
  digitalWrite(disp4, LOW);
}

void loop() 
{
  long currentTime = millis();
  if (currentTime - lastMsg > 1000) 
  {
    lastMsg = currentTime;

    DateTime now = rtc.now();

    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();

    digit1 = now.minute() % 10;
    digit2 = now.minute() / 10;
    digit3 = now.hour() % 10;
    digit4 = now.hour() / 10;

    count ++;

    if((count % 2) == 0)
    {
      digitalWrite(COM1_PIN, LOW);
    }
    else
    {
      digitalWrite(COM1_PIN, HIGH);
    }
  }

  if(digitalRead(KEY1) == LOW)
  {
    Serial.println("press1");
  }

  if(digitalRead(KEY2) == LOW)
  {
    Serial.println("press2");
  }

  if(digitalRead(KEY3) == LOW)
  {
    Serial.println("press3");
  }

  if(digitalRead(KEY4) == LOW)
  {
    Serial.println("press4");
  }

  setsegments(digit1, disp1, duration);
  setsegments(digit2, disp2, duration);
  setsegments(digit3, disp3, duration);
  setsegments(digit4, disp4, duration);  
}

void setsegments(int number, int digit, int ontime)
{ 
  for (int seg=0; seg<8; seg++)
  { 
    if(numbers[number][seg]==1)
    { 
      digitalWrite(segments[seg], HIGH);
    }
    else 
    {
      digitalWrite(segments[seg], LOW);
    }
  }
  digitalWrite(digit, HIGH);
  delayMicroseconds(ontime);
  digitalWrite(digit, LOW);
}

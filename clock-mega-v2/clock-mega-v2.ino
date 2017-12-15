/*
Arduino MEGA clock
Copyright (C) 2017  Victor Serov 

info@vitjka.com

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

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
#include <EEPROM.h>

#define DIG8_PIN 9
#define DIG9_PIN 10
#define DIG10_PIN 11
#define COM1_PIN 37
#define BUZZ_PIN 12

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

#define SET_ALARM 29
#define SET_CLOCK 32
#define HOURS 31
#define MINUTES 30

#define DISABLE_BUZZER 19

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

long lastSec = 0;
long lastmSec = 0;

int count = 0;

int digit1 = 0;
int digit2 = 0;
int digit3 = 0;
int digit4 = 0;

int requestSetHours = 0;
int requestSetMinutes = 0;

int requestSetAlarmHours = 0;
int requestSetAlarmMinutes = 0;

int alarmOn;

TimeSpan alarmTime;
#define MAGIC_PATTERN 0xAB
#define HOURS_ADDR 2
#define MINUTES_ADDR 3

TimeSpan alarmDuration = TimeSpan(0, 0, 1, 0);
DateTime alarmStart;

void setup() 
{
  Serial.begin(115200);

  if(EEPROM.read(0) != MAGIC_PATTERN)
  {
    EEPROM.write(0, MAGIC_PATTERN);
    EEPROM.write(HOURS_ADDR, 0);
    EEPROM.write(MINUTES_ADDR, 0);
  }
  byte alarmHours = EEPROM.read(HOURS_ADDR);
  byte alarmMinutes = EEPROM.read(MINUTES_ADDR);
  
  alarmTime = TimeSpan(0, alarmHours, alarmMinutes, 0);

  alarmOn = 0;

  Serial.print("Alarm: ");
  Serial.print(alarmHours);
  Serial.print(" : ");
  Serial.println(alarmMinutes);

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

  pinMode(SET_ALARM, INPUT_PULLUP);
  pinMode(SET_CLOCK, INPUT_PULLUP);
  pinMode(HOURS, INPUT_PULLUP);
  pinMode(MINUTES, INPUT_PULLUP);

  pinMode(DISABLE_BUZZER, INPUT_PULLUP);

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
  if (currentTime - lastSec > 1000) 
  {
    lastSec = currentTime;

///////////////////////////////////////////
// Print time to serial
///////////////////////////////////////////
    /*
    DateTime currentDate = rtc.now();

    Serial.print(currentDate.year(), DEC);
    Serial.print('/');
    Serial.print(currentDate.month(), DEC);
    Serial.print('/');
    Serial.print(currentDate.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[currentDate.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(currentDate.hour(), DEC);
    Serial.print(':');
    Serial.print(currentDate.minute(), DEC);
    Serial.print(':');
    Serial.print(currentDate.second(), DEC);
    Serial.println();
    */

    count ++;

    if((count % 2) == 0)
    {
      if(digitalRead(SET_ALARM) != LOW)
        digitalWrite(COM1_PIN, LOW);
      soundAlarm();
    }
    else
    {
      if(digitalRead(SET_ALARM) != LOW)
        digitalWrite(COM1_PIN, HIGH);
      disableAlarm();
    }
  }

  if(currentTime - lastmSec > 250)
  {
    lastmSec = currentTime;

    DateTime currentDate = rtc.now();
    
    digit1 = currentDate.minute() % 10;
    digit2 = currentDate.minute() / 10;
    digit3 = currentDate.hour() % 10;
    digit4 = currentDate.hour() / 10;
  }

/////////////////////////////////////////////
//  set alarm
/////////////////////////////////////////////
  if(digitalRead(SET_ALARM) == LOW)
  {
    digit1 = alarmTime.minutes() % 10;
    digit2 = alarmTime.minutes() / 10;
    digit3 = alarmTime.hours() % 10;
    digit4 = alarmTime.hours() / 10;

    digitalWrite(COM1_PIN, HIGH);

    //Set Hours
    if(digitalRead(HOURS) == LOW)
    {
      requestSetAlarmHours = 1;
    }
    else
    {
      if(requestSetAlarmHours)
      {
        requestSetAlarmHours = 0;
        Serial.println("Set Alarm Hours");
        alarmTime = alarmTime + TimeSpan(0, 1, 0, 0);
        EEPROM.write(HOURS_ADDR, alarmTime.hours());
      }
    }

    //Set Minutes
    if(digitalRead(MINUTES) == LOW)
    {
      requestSetAlarmMinutes = 1;
    }
    else
    {
      if(requestSetAlarmMinutes)
      {
        requestSetAlarmMinutes = 0;
        Serial.println("Set Alarm Minutes");
        alarmTime = alarmTime + TimeSpan(0, 0, 1, 0);
        EEPROM.write(MINUTES_ADDR, alarmTime.minutes());
      }
    }
  }


/////////////////////////////////////////////
//  set clock
/////////////////////////////////////////////
  if(digitalRead(SET_CLOCK) == LOW)
  {
    //Set Hours
    if(digitalRead(HOURS) == LOW)
    {
      requestSetHours = 1;
    }
    else
    {
      if(requestSetHours)
      {
        requestSetHours = 0;
        Serial.println("Set Hours");
        DateTime newTime = rtc.now() + TimeSpan(0, 1, 0, 0);
        rtc.adjust(newTime);
      }
    }

    //Set Minutes
    if(digitalRead(MINUTES) == LOW)
    {
      requestSetMinutes = 1;
    }
    else
    {
      if(requestSetMinutes)
      {
        requestSetMinutes = 0;
        Serial.println("Set Minutes");
        DateTime newTime = rtc.now() + TimeSpan(0, 0, 1, 0);
        rtc.adjust(newTime);
      }
    }
  }

  //Display digits
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

void soundAlarm()
{
  if(digitalRead(DISABLE_BUZZER))
  {
    DateTime currentTime = rtc.now();
    if(currentTime.hour() == alarmTime.hours() && currentTime.minute() == alarmTime.minutes())
    {
      alarmOn = 1;
      alarmStart = currentTime;
    }
    if(alarmOn)
    {
      DateTime alarmEnd = alarmStart + alarmDuration;
      if(currentTime.hour() == alarmEnd.hour() && currentTime.minute() == alarmEnd.minute())
      {
        alarmOn = 0;
      }
    }

    if(alarmOn)
    {
      Serial.println("Alarm On");
      tone(BUZZ_PIN, 500);
    }
  }
}

void disableAlarm()
{
  noTone(BUZZ_PIN);
}


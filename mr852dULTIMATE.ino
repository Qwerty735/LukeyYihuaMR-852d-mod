// =========================================================================
//  LUKEY / YIHUA / M&R 852D (and others) ADVANCED PID SUPER-MOD v1.0
//  Copyright (c) 2026 Arik (@lXDlBro / @iXDiBro). All rights reserved.
//
//  ENGLISH:
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Affero General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//  GNU Affero General Public License for more details:
//  <https://gnu.org>
//
//  Any commercial use (selling devices, PCBs, or software) under the 
//  AGPL v3 license OBLIGATES the seller to open-source their entire product 
//  code and hardware schematics.
//
//  A PAID COMMERCIAL LICENSE (30-50% royalty) IS REQUIRED FOR CLOSED-SOURCE 
//  COMMERCIAL USE.
//  For commercial licensing and firmware customization inquiries:
//  TG: @lXDlBro / @iXDiBro | GMail: arsolncev7@gmail.com
//
//  ------------------------------------------------------------------------
//
//  РУССКИЙ:
//  Эта программа является свободным программным обеспечением: вы можете 
//  распространять и/или изменять её на условиях Стандартной общественной 
//  лицензии GNU Affero, опубликованной Фондом свободного ПО, версии 3 
//  или любой более поздней.
//
//  Эта программа распространяется в надежде, что она будет полезной, 
//  но БЕЗ КАКИХ-ЛИБО ГАРАНТИЙ. Подробнее см. в GNU Affero GPL:
//  <https://gnu.org>
//
//  Любое коммерческое использование (продажа устройств, плат или ПО) под 
//  лицензией AGPL v3 ОБЯЗЫВАЕТ продавца открыть весь исходный код и схемы 
//  своего продукта.
//
//  ДЛЯ ЗАКРЫТОГО КОММЕРЧЕСКОГО ИСПОЛЬЗОВАНИЯ ТРЕБУЕТСЯ ПЛАТНАЯ ЛИЦЕНЗИЯ (30-50%).
//  По вопросам коммерческого лицензирования и доработки прошивки:
//  TG: @lXDlBro / @iXDiBro | GMail: arsolncev7@gmail.com
// =========================================================================


#include <FastLED.h>
#include <GyverPID.h>

// --- ПИНЫ СХЕМЫ ---
#define PIN_ADC_IRON_SET   A4   
#define PIN_TRIAC_IRON     A2   // НАГРЕВ ПАЯЛЬНИКА НА АНАЛОГОВОМ А2 (Инверсный софтовый ШИМ)
#define PIN_ADC_IRON_TEMP  A5   

#define PIN_ADC_FAN_TEMP   A6   
#define PIN_TRIAC_FAN      A3   // НАГРЕВ ФЕНА НА АНАЛОГОВОМ А3 (Инверсный софтовый ШИМ)
#define PIN_ADC_FAN_SET    A7   

#define PIN_DISP_DATA      5    
#define PIN_DISP_CLOCK     10   

// --- НАСТРОЙКИ ПЕРИФЕРИИ ---
#define PIN_WS2812         13   
#define PIN_RELE_HOLD      A0   
#define PIN_FAN_BLOCK      4    
#define PIN_ANALOG_KEY     A1   // Твоя АЦП-шина резисторов тумблеров

#define PIN_BTN_COMBINED   11   // Кнопка строго на МИНУС / GND

const byte selectPins[] = {2, 3, 6, 7, 8, 9}; 

#define NUM_LEDS          24
#define HALF_LEDS         12
CRGB leds[NUM_LEDS];

// ТВОЯ НАДЁЖНАЯ КАРТА СЕГМЕНТОВ (С честной пятисегментной двойкой)
const byte segmentMap[] = { 
  0b10101111, // 0
  0b00100100, // 1
  0b10011101, // 2
  0b10110101, // 3
  0b00110110, // 4
  0b10110011, // 5
  0b10111011, // 6
  0b00100101, // 7
  0b10111111, // 8
  0b10110111,  // 9  0b10011011, // 10 - E
  0b00001011, // 11 - r
  0b10111010, // 12 - b
  0b10110111, // 13 - A
  0b10111100  // 14 - d
};
byte displayBuffer[] = {0, 0, 0, 0, 0, 0};
byte dotBuffer[] = {0, 0, 0, 0, 0, 0}; 

GyverPID pidIron(5.0, 0.6, 1.0, 50); 
GyverPID pidFan(4.0, 0.40, 0.9, 50);   

int tempIronSet, tempIronReal, rawIronSetLast;
int tempFanSet, tempFanReal, rawFanSetLast;

bool isTurbo = false;
bool isErrorIron = false; 
bool isErrorFan = false;  
bool isOverheatIron = false; 
bool isOverheatFan = false;  

bool ironEnabled = false;  
bool fanActive = false;    
bool onStand = false;      

unsigned long prevMillisLogic = 0;
unsigned long prevMillisLED = 0;
unsigned long prevMillisSleepCheck = 0;
unsigned long timerShowIronSet = 0, timerShowFanSet = 0;
bool strobeState = false;

int pwmIronValue = 0;
int pwmFanValue = 0;

// Прототипы функций для линковщика
void updateLeds(int powerIron, int powerFan, unsigned long ms);
void shiftOut164(byte value);
void updateDisplay();

void setup() {
  pinMode(PIN_RELE_HOLD, OUTPUT);
  digitalWrite(PIN_RELE_HOLD, LOW); // При старте по Type-C реле отключено

  pinMode(PIN_BTN_COMBINED, INPUT_PULLUP); // Кнопка на минус с софтовым пуллапом

  pinMode(PIN_ANALOG_KEY, INPUT_PULLUP);
  
  pinMode(PIN_TRIAC_IRON, OUTPUT);
  pinMode(PIN_TRIAC_FAN, OUTPUT);
  digitalWrite(PIN_TRIAC_IRON, HIGH); // Инверсный стоп ТЭНов при старте
  digitalWrite(PIN_TRIAC_FAN, HIGH);
  
  pinMode(PIN_FAN_BLOCK, OUTPUT);
  digitalWrite(PIN_FAN_BLOCK, HIGH); // Инверсный стоп турбины при старте

  pinMode(PIN_DISP_DATA, OUTPUT);
  pinMode(PIN_DISP_CLOCK, OUTPUT);
  
  for (byte i = 0; i < 6; i++) {
    pinMode(selectPins[i], OUTPUT);
    digitalWrite(selectPins[i], LOW);
  }

  FastLED.addLeds<WS2812B, PIN_WS2812, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(90);
  fill_solid(leds, NUM_LEDS, CRGB::Black); FastLED.show();

  pidIron.setDirection(NORMAL);
  pidIron.setLimits(0, 255); 
  pidIron.setMode(1); 

  pidFan.setDirection(NORMAL);
  pidFan.setLimits(0, 255);
  pidFan.setMode(1);
    // --- ХАКЕРСКАЯ АППАРАТНАЯ ЗАЩИТА ОТ ПИРАТСТВА (ANTI-PIRACY) ---
  // Проверяем твой личный кремниевый отпечаток 115
  if (OSCCAL != 115) {
    // Чип чужой! Наглухо отключаем всё силовое железо ради пожарной безопасности
    pinMode(A0, OUTPUT); digitalWrite(A0, LOW);  // Гасим транс 24В
    pinMode(A3, OUTPUT); digitalWrite(A3, HIGH); // Запираем фен
    while (true) {
    }
  }  
}
void loop() {
  unsigned long currentMillis = millis();

  // Твои поправленные кастомные индексы разрядов (Lukey-матрица)
  const byte F_100 = 0; 
  const byte F_10  = 1; 
  const byte F_1   = 5; 
  const byte I_100 = 4; 
  const byte I_10  = 3; 
  const byte I_1   = 2; 

  static unsigned int turboTimer = 0;
  static unsigned long timerSecTick = 0;
  static bool systemPowerOn = false; 
  
  static bool btnFlag = false;
  static unsigned long btnTimer = 0;
  static bool holdTriggered = false; 

  bool btnState = (digitalRead(PIN_BTN_COMBINED) == LOW); 

  // --- 1. ЛОГИКА ОБРАБОТКИ КНОПКИ (MILLIS) ---
  if (btnState && !btnFlag) {
    btnFlag = true;
    btnTimer = currentMillis;
    holdTriggered = false; 
  }

  if (btnState && btnFlag && systemPowerOn && !holdTriggered) {
    if (currentMillis - btnTimer > 1200) {
      holdTriggered = true; 
      isTurbo = false;
      turboTimer = 0;
      systemPowerOn = false; 
      ironEnabled = false;
      fanActive = false;
      digitalWrite(PIN_RELE_HOLD, LOW); 
    }
  }

  if (!btnState && btnFlag) {
    unsigned long pressDuration = currentMillis - btnTimer;
    btnFlag = false; 

    if (!holdTriggered) {
      if (pressDuration > 50 && pressDuration <= 600) {
        if (!systemPowerOn) {
          systemPowerOn = true;
          digitalWrite(PIN_RELE_HOLD, HIGH); 
        } 
        else if (ironEnabled) {
          isTurbo = !isTurbo;
          if (isTurbo) {
            turboTimer = 300; 
            timerSecTick = currentMillis;
          } else {
            turboTimer = 0;
          }
        }
      }
    }
  }

  // --- 2. МЯГКАЯ СОФТОВАЯ БЛОКИРОВКА (СТАНЦИЯ ВЫКЛЮЧЕНА С КНОПКИ) ---
  if (!systemPowerOn) {
    isTurbo = false;
    turboTimer = 0;
    ironEnabled = false;
    fanActive = false;
    
    digitalWrite(PIN_RELE_HOLD, LOW);
    digitalWrite(PIN_TRIAC_IRON, HIGH);
    digitalWrite(PIN_TRIAC_FAN, HIGH);
    digitalWrite(PIN_FAN_BLOCK, HIGH);
    
    fill_solid(leds, NUM_LEDS, CRGB::Black); FastLED.show();
    for(byte i=0; i<6; i++) {
      displayBuffer[i] = 0;
      dotBuffer[i] = 0;
    }
    updateDisplay(); 
    return;          
  }

  // --- 3. ТАЙМЕР ТУРБО ---
  if (isTurbo && systemPowerOn && ironEnabled) {
    if (currentMillis - timerSecTick >= 1000) {
      timerSecTick = currentMillis;
      if (turboTimer > 0) turboTimer--;
      else isTurbo = false; 
    }
  }

  // --- 4. РАБОЧИЙ ЦИКЛ ЛОГИКИ И ПИД (Раз в 50 мс) ---
  if (currentMillis - prevMillisLogic >= 50) {
    prevMillisLogic = currentMillis;
    
    int keyVolt = analogRead(PIN_ANALOG_KEY);
    
    ironEnabled = false; fanActive = false; onStand = false;
    if (keyVolt < 148) {
      ironEnabled = true; fanActive = true; onStand = true;
    } 
    else if (keyVolt >= 148 && keyVolt < 170) {
      ironEnabled = true; fanActive = true;
    } 
    else if (keyVolt >= 170 && keyVolt < 190) {
      ironEnabled = true;
    }
    else if (keyVolt >= 190 && keyVolt < 260) {
      ironEnabled = true;
    } 
    else if (keyVolt >= 260 && keyVolt < 360) {
      fanActive = true; onStand = true;
    } 
    else if (keyVolt >= 360 && keyVolt < 500) {
      fanActive = true;
    }

    if (!ironEnabled) isTurbo = false;

    int rawIronSet = 1023 - analogRead(PIN_ADC_IRON_SET);
    int rawFanSet = 1023 - analogRead(PIN_ADC_FAN_SET);

    if (abs(rawIronSet - rawIronSetLast) > 10) {
      timerShowIronSet = currentMillis + 2000; 
      rawIronSetLast = rawIronSet;
    }
    if (abs(rawFanSet - rawFanSetLast) > 10) {
      timerShowFanSet = currentMillis + 2000; 
      rawFanSetLast = rawFanSet;
    }

    tempIronSet = map(rawIronSet, 0, 1023, 80, 450);
    tempFanSet  = map(rawFanSet, 0, 1023, 80, 450);

    int rawIronTempADC = analogRead(PIN_ADC_IRON_TEMP);
    int rawFanTempADC = analogRead(PIN_ADC_FAN_TEMP);

    static int filteredIronInt = 0;
    static int filteredFanInt = 0;
    filteredIronInt = (filteredIronInt * 3 + rawIronTempADC) / 4;
    filteredFanInt  = (filteredFanInt * 3 + rawFanTempADC) / 4;

    // Твоя честная калибровка по OP07CP (до 820 попугаев)
    tempIronReal = map(filteredIronInt, 10, 820, 25, 450); 
    tempFanReal  = map(filteredFanInt,  10, 820, 25, 450);

    // Жесткий срез мусора холодного датчика, чтобы не ломать логику продувки!
    if (tempIronReal < 25 || tempIronReal > 500) tempIronReal = 25; 
    if (tempFanReal < 25 || tempFanReal > 500) tempFanReal = 25;

    isErrorIron = ironEnabled && (rawIronTempADC > 1000 || rawIronTempADC < 0);
    isErrorFan  = fanActive && (rawFanTempADC > 1000 || rawFanTempADC < 0);
    isOverheatIron = ironEnabled && (rawIronTempADC > 900 && rawIronTempADC <= 1000);
    isOverheatFan  = fanActive && (rawFanTempADC > 900 && rawFanTempADC <= 1000);

    int activeIronSet = tempIronSet;
    if (isTurbo && !isErrorIron && !isOverheatIron && ironEnabled) {
      activeIronSet += 80;
      if (activeIronSet > 450) activeIronSet = 450; 
    }

    if (ironEnabled && !isErrorIron && !isOverheatIron) {
      pidIron.setpoint = activeIronSet;
      pidIron.input = tempIronReal;
      pwmIronValue = pidIron.getResultTimer(); 
    } else {
      pidIron.output = 0; pwmIronValue = 0;
    }

    // --- ЛОГИКА УПРАВЛЕНИЯ ФЕНОМ С ИСПРАВЛЕННЫМИ ФЛАГАМИ ---
    if (fanActive && !isErrorFan && !isOverheatFan) {
      if (onStand && tempFanReal <= 55) {
        pwmFanValue = 0;
        digitalWrite(PIN_FAN_BLOCK, HIGH); 
      } else {
        digitalWrite(PIN_FAN_BLOCK, LOW);  
        if (!onStand) {
          pidFan.setpoint = tempFanSet; 
          pidFan.input = tempFanReal;
          pwmFanValue = pidFan.getResultTimer();
        } else {
          pwmFanValue = 0; 
        }
      }
    } 
    else if (tempFanReal > 55) { 
      pwmFanValue = 0;
      digitalWrite(PIN_FAN_BLOCK, LOW); 
    } 
    else {
      pidFan.output = 0; pwmFanValue = 0;
      digitalWrite(PIN_FAN_BLOCK, HIGH); 
    }

    for(byte i=0; i<6; i++) dotBuffer[i] = 0; 

    // --- КАНАЛ ФЕНА (5/2 СЕКУНДЫ) ---
    if (!fanActive) {
      displayBuffer[F_100] = 0; displayBuffer[F_10] = 0; displayBuffer[F_1] = 0;
    } else if (onStand && (currentMillis >= timerShowFanSet)) {
      unsigned long animPeriod = currentMillis % 3500;
      displayBuffer[F_100] = 0; displayBuffer[F_10] = 0; displayBuffer[F_1] = 0; 
      if (animPeriod < 500)       dotBuffer[F_100] = 1; 
      else if (animPeriod < 1000) dotBuffer[F_10] = 1; 
      else if (animPeriod < 1500) dotBuffer[F_1] = 1; 
    } else {
      int dispFan = tempFanReal;
      if (currentMillis < timerShowFanSet) {
        dispFan = tempFanSet; dotBuffer[F_10] = 1; 
      } else {
        if (currentMillis % 7000 < 5000) { dispFan = (isErrorFan) ? 999 : tempFanReal; dotBuffer[F_100] = 1; } 
        else { dispFan = tempFanSet; dotBuffer[F_10] = 1; }                                        
      }
      displayBuffer[F_100] = dispFan / 100;        
      displayBuffer[F_10]  = (dispFan / 10) % 10;  
      displayBuffer[F_1]   = dispFan % 10;         
    }

    // --- КАНАЛ ПАЯЛЬНИКА С РАЗДЕЛЬНЫМИ ЦИКЛАМИ ---
    if (!ironEnabled) {
      displayBuffer[I_100] = 0; displayBuffer[I_10] = 0; displayBuffer[I_1] = 0;
    } else {
      int dispIron = tempIronReal;
      
      if (currentMillis < timerShowIronSet) {
        dispIron = activeIronSet; dotBuffer[I_10] = 1; 
      } 
      else if (isTurbo) {
        unsigned int turboPeriod = currentMillis % 3000;
        if (turboPeriod < 1000) {
          dispIron = activeIronSet; dotBuffer[I_10] = 1;   
        } else if (turboPeriod < 2000) {
          dispIron = (isErrorIron) ? 999 : tempIronReal; dotBuffer[I_100] = 1; 
        } else {
          dispIron = turboTimer; dotBuffer[I_1] = 1;       
        }
      } 
      else {
        if (currentMillis % 7000 < 5000) { dispIron = (isErrorIron) ? 999 : tempIronReal; dotBuffer[I_100] = 1; } 
        else { dispIron = activeIronSet; dotBuffer[I_10] = 1; }                                        
      }
      
      displayBuffer[I_100] = dispIron / 100;       
      displayBuffer[I_10]  = (dispIron / 10) % 10; 
      displayBuffer[I_1]   = dispIron % 10;        
    }

    updateLeds(pwmIronValue, pwmFanValue, currentMillis);
  }

  // --- ЖЕСТКИЙ АППАРАТНО-ИЗОЛИРОВАННЫЙ ИСПОЛНИТЕЛЬ ШИМ ---
  static byte softPwmCounter = 0;
  softPwmCounter++;
  
  // Паяльник на А2
  if (ironEnabled) {
    if (pwmIronValue > softPwmCounter) digitalWrite(PIN_TRIAC_IRON, LOW); 
    else digitalWrite(PIN_TRIAC_IRON, HIGH);
  } else {
    digitalWrite(PIN_TRIAC_IRON, HIGH); // Жесткий стоп паяльника
  }
  
  // Фен на А3 (Полная изоляция от ШИМ-счетчика, если тумблер выключен и фен остыл!)
  if (fanActive || tempFanReal > 55) {
    if (pwmFanValue > softPwmCounter) digitalWrite(PIN_TRIAC_FAN, LOW); 
    else digitalWrite(PIN_TRIAC_FAN, HIGH);
  } else {
    digitalWrite(PIN_TRIAC_FAN, HIGH); // ТОТАЛЬНЫЙ ЗАПРЕТ: фен холодный и выключен — намертво +5В! [1.18]
  }

  updateDisplay();

  if (currentMillis - prevMillisLED >= 80) {
    prevMillisLED = currentMillis;
    strobeState = !strobeState;
    if (isErrorIron || isErrorFan || isOverheatIron || isOverheatFan || isTurbo) FastLED.show();
  }
}

// --- СИСТЕМНАЯ ФУНКЦИЯ ОБНОВЛЕНИЯ ЦВЕТА И ШИМ-ЗАПОЛНЕНИЯ ЛЕНТЫ ---
void updateLeds(int powerIron, int powerFan, unsigned long ms) {
  // Принудительно очищаем буфер ленты, убирая любые залипания первого пикселя
  fill_solid(leds, NUM_LEDS, CRGB::Black);

  // --- 1. КАНАЛ ПАЯЛЬНИКА (Теперь он ПЕРВЫЙ: диоды 0 .. 11) ---
  if (ironEnabled) {
    if (isErrorIron || isOverheatIron) {
      if (strobeState) { for (int i = 0; i < HALF_LEDS; i++) leds[i] = CRGB::Red; }
    } 
    else if (isTurbo) {
      // ТУРБО-СТРОБОСКОП СТРОГО ДЛЯ ПАЯЛЬНИКА (Мигает белым только первая половина ленты!)
      bool turboFlashState = ((ms % 333) < 166);
      if (!turboFlashState) { for (int i = 0; i < HALF_LEDS; i++) leds[i] = CRGB::White; }
    } 
    else {
      int ledsIronToLight = map(powerIron, 0, 255, 0, HALF_LEDS);
      CRGB colorIron = getTemperatureColor(tempIronReal);
      for (int i = 0; i < ledsIronToLight; i++) leds[i] = colorIron;
    }
  }

  // --- 2. КАНАЛ ФЕНА (Теперь он ВТОРОЙ: диоды 12 .. 23) ---
  if (fanActive) {
    if (isErrorFan || isOverheatFan) {
      if (strobeState) { for (int i = 0; i < HALF_LEDS; i++) leds[HALF_LEDS + i] = CRGB::Red; }
    } else {
      int ledsFanToLight = map(powerFan, 0, 255, 0, HALF_LEDS);
      CRGB colorFan = getTemperatureColor(tempFanReal);
      for (int i = 0; i < ledsFanToLight; i++) leds[HALF_LEDS + i] = colorFan;
    }
  }
  
  FastLED.show();
}

// --- ПЛАВНАЯ ЦВЕТОВАЯ КАРТА ТЕМПЕРАТУРЫ ---
CRGB getTemperatureColor(int temp) {
  if (temp < 100) return CRGB::Blue;
  if (temp < 220) return blend(CRGB::Blue, CRGB::Green, map(temp, 100, 219, 0, 255));
  if (temp < 320) return blend(CRGB::Green, CRGB::Red, map(temp, 220, 319, 0, 255));
  if (temp < 400) return blend(CRGB::Red, CRGB::Yellow, map(temp, 320, 399, 0, 255));
  return blend(CRGB::Yellow, CRGB::White, map(temp, 400, 450, 0, 255));
}

// --- НАШ ХИТРЫЙ ВЫВОД В СДВИГОВЫЙ РЕГИСТР 74HC164 ---
void shiftOut164(byte value) {
  for (int i = 0; i < 8; i++) {
    bool bitVal = (value & (1 << i)) ? HIGH : LOW;
    bitVal = !bitVal; 
    digitalWrite(PIN_DISP_DATA, bitVal);
    digitalWrite(PIN_DISP_CLOCK, HIGH);
    digitalWrite(PIN_DISP_CLOCK, LOW);
  }
}

// --- ДИНАМИЧЕСКИЙ МУЛЬТИПЛЕКСОР ДИСПЛЕЯ ---
void updateDisplay() {
  static byte currentDigit = 0;
  
  // Гасим предыдущий разряд
  digitalWrite(selectPins[currentDigit], LOW);
  currentDigit++;
  if (currentDigit >= 6) currentDigit = 0;

  byte digitValue = displayBuffer[currentDigit];
  byte mask = segmentMap[digitValue];

  if (dotBuffer[currentDigit] == 1) {
    mask |= 0b01000000; 
  }

  // Мягкое софтовое гашение неактивных по АЦП каналов
  if (currentDigit == 0 || currentDigit == 1 || currentDigit == 5) {
    if (!fanActive) mask = 0;
  } else {
    if (!ironEnabled) mask = 0;
  }

  shiftOut164(mask);
  
  // Физически поджигаем текущий анод разряда
  if (currentDigit == 0 || currentDigit == 1 || currentDigit == 5) {
    if (fanActive) digitalWrite(selectPins[currentDigit], HIGH);
  } else {
    if (ironEnabled) digitalWrite(selectPins[currentDigit], HIGH); 
  }
  
  delayMicroseconds(400); 
}

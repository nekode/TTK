#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RotaryEncoder.h>
#include "max6675.h"
#include <EEPROM.h>

/*
#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
*/

OneWire oneWire(2); // Шина 1-Wire для датчиков температуры DS18B20 на пине 2.

DallasTemperature sensors(&oneWire);

/*
#if defined(ARDUINO_ARCH_SAMD)
#define Serial SerialUSB
#endif
#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif
*/
#define key_pressed_encoder 1
#define key_pressed_rezhim 2
#define key_holded_encoder 3
#define key_holded_rezhim 4
#define keys_not_pressed 0


Servo zaslonka;// Servo
RTC_DS3231 rtc; // Clok + EEPROM
LiquidCrystal_I2C lcd(0x3F, 20, 4); // Устанавливаем дисплей 2004 Evro

byte rus_D[8]  = // Д - матрица
{
  0b00110,
  0b01010,
  0b01010,
  0b01010,
  0b01010,
  0b01010,
  0b11111,
  0b10001
};

byte rus_i[8] = // И - матрица
{ B10001,
  B10001,
  B10011,
  B10101,
  B11001,
  B10001,
  B10001,
  B00000,
};

byte rus_yy[8] = // Ы - матрица
{ B10001,
  B10001,
  B10001,
  B11101,
  B10011,
  B10011,
  B11101,
  B00000,
};

byte rus_p[8] = // П - матрица
{ B11111,
  B10001,
  B10001,
  B10001,
  B10001,
  B10001,
  B10001,
  B00000,
};

byte rus_ya[8] = // Я - матрица
{ B01111,
  B10001,
  B10001,
  B01111,
  B00101,
  B01001,
  B10001,
  B00000,
};

byte rus_g[8] = // Г - матрица
{ B11111,
  B10001,
  B10000,
  B10000,
  B10000,
  B10000,
  B10000,
  B00000,
};

byte rus_l[8] = // Л - матрица
{ B00111,
  B01001,
  B01001,
  B01001,
  B01001,
  B01001,
  B10001,
  B00000,
};

byte rus_zg[8] = // Ж - матрица
{ B10101,
  B10101,
  B01110,
  B00100,
  B01110,
  B10101,
  B10101,
  B00000,
};

uint8_t key_data = 0; // переменная функции опроса кнопок
boolean summer_mode = 0; // Летний режим
double temp_gaz = 20.0; //Температура носителя на выходе газового котла
double temp_carbon = 10.0; //Температура носителя на выходе твердотопливного котла
double temp_obratka = 10.0; //Температура носителя In
double temp_logging = 30.0;
int16_t temp_max = 95; // Максимальная температура аварии
int16_t temp_min = 35; // Минимальная температура (котел холодный) насос выключяется
int16_t temp_val = 45; //Установленная температура
int16_t temp_ttk_diff = 10; // полоса мастштабирования температуры в коэффициент открытия заслонки
int16_t tempthermocouple_value = 0; // Переменная температуры дымохода
int16_t tempthermocouple_value_limit = 105; // Значение ограничения температуры дымохода
int16_t tempthermocouple_value_limit_low = 90; // Значение гистерезиса ограничения температуры дымохода
double temp_old = 25; // Переменная предыдущей температуры подачи твердотопливного котла
long temp_oldtime = 0; // Предыдущее время замера для определения остывания
uint32_t period_millis = 0; //переменная времени для периодического обновления данных
uint32_t seconds_millis = 0; //переменная времени для периодического обновления данных
uint32_t smoke_overtemp_millis = 0; // переменная для интервала проверки перегрева дымохода
uint32_t temp_logging_millis = 0;
uint32_t temp_logging__47_millis = 0;
uint16_t period = 3500; //период обновления данных термометров(миллисекунды)
uint16_t zaslonka_interval = 30000; // Интервал обновления положения заслонки  (миллисекунды)
uint32_t zaslonka_millis_old = 0;
uint16_t zaslonka_val; // положение заслонки
uint16_t zaslonka_min = 33; // Заслонка закрыта
uint16_t zaslonka_max = 165; // Заслонка открыта
uint16_t zaslonka_val_old = zaslonka_min; // переменная хранения предыдущего значения заслонки
uint16_t zaslonka_temp = zaslonka_min; // переменная для плавного изменения положения заслонки
uint16_t cooling_counter = 0; // Переменная отсчёта остывания (сек)
uint16_t cooling_limit_1 = 1800; // Переменная времени детектирования остывания (сек)
uint16_t cooling_limit_2 = 5400; // Переменная окончания индикации остывания (сек)
boolean cooling_alarm = 0; // Переменная аварии остывания
boolean max_alarm = 0; // Переменная аварии перегрева
boolean smoke_temp_max_state = 0; // Переменная включения режима ограничения температуры дымохода
boolean mode_set_temp = 0; // Переменная режима установки рабочей температуры
boolean mode_menu = 0; // Переменная режима меню
boolean blink_display = 0; // Мигание данных на дисплее
const uint8_t knopka_rezhim = 12; // кнопка выбора режима				   
const uint8_t knopka = 5; // кнопка энкодера
const uint8_t POMPA = 6; // пин насоса (управление низким уровнем)
const uint8_t BEEPER = 4; // пин пьезобузера
const uint8_t RELE = 7; // резерв
const uint8_t SERVO = 3; // пин управления сервой
const uint8_t thermoDO = 8; //подключение max6675
const uint8_t thermoCS = 9; //подключение max6675
const uint8_t thermoCLK = 10; //подключение max6675

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

int16_t pos = 0; // для хранения текущего положения энкодера																							
RotaryEncoder encoder(A2, A3); //устанавливаем A2, A3 порты для энкодера
int16_t result = -1;
long lcdUpdate;

DeviceAddress sensor1 = {  0x28, 0xFF, 0x96, 0x2D, 0xA1, 0x15, 0x03, 0x22  }; // адреса градусников.
DeviceAddress sensor2 = { 0x28, 0xFF, 0x5B, 0x05, 0xA1, 0x15, 0x03, 0x80  };
DeviceAddress sensor3 = { 0x28, 0xFF, 0x01, 0x11, 0xA1, 0x15, 0x03, 0x9E  };
//DeviceAddress sensor4 = { 0x28, 0xFF, 0x4E, 0x12, 0xA1, 0x15, 0x03, 0x90  };

void setup()
{
  sensors.begin(); // Запуск датчиков температуры
  sensors.setResolution(sensor1, 11); // 11 bit (9, 10, 11 и 12)
  sensors.setResolution(sensor2, 11); // 11 bit
  sensors.setResolution(sensor3, 11); // 11 bit
  Serial.begin(9600);

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1){	
      tone(BEEPER, 900, 10);
      }; // stop! No time!
}
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }  
  
  
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, rus_D); // - прописываем символы
  lcd.createChar(1, rus_i);
  lcd.createChar(2, rus_yy);
  lcd.createChar(3, rus_ya);
  lcd.createChar(4, rus_p);
  lcd.createChar(5, rus_g);
  lcd.createChar(6, rus_l);
  lcd.createChar(6, rus_zg); 

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("3A\x0DP\xBF""3KA...");

/*
  zaslonka.attach(SERVO); // Иницилизация серво
  zaslonka.write(zaslonka_max); // Установка заслонки в открыто
  delay(1000);
  zaslonka.detach();
*/  
  pinMode(knopka_rezhim, INPUT);  // кнопка режима
  pinMode(knopka, INPUT);
//  digitalWrite(knopka, HIGH);
  pinMode(POMPA, OUTPUT);
  pinMode(13, OUTPUT);  
  digitalWrite(POMPA, HIGH);

  lcd.clear();

  EEPROM.get(0, temp_min);  //Чтение из ЕЕПРОМ 
  EEPROM.get(2, temp_val); //Чтение из ЕЕПРОМ 

    zaslonka.attach(SERVO); // Иницилизация серво
    zaslonka.write(zaslonka_min);  // Перемещение заслонки
    delay(1500); // Ждём установки сервы
    zaslonka_val = zaslonka_min;
    zaslonka.detach();
}


void loop()
{
  encoder.tick();	// считывание состояния энкодера
  key_data = get_key();  // вызываем функцию определения нажатия кнопок, присваивая возвращаемое ней значение переменной, которую далее будем использовать в коде

  if ((millis()-period_millis)>period) // Считываем показания температуры с датчиков с интервалом "period"
      {
    sensors.requestTemperatures(); // запрос температуры
    temp_gaz = sensors.getTempCByIndex(0); // Температура выхода теплоносителя газового котла
    temp_carbon = sensors.getTempCByIndex(2); // Температура выхода теплоносителя твердотопливного котла
    temp_obratka = sensors.getTempCByIndex(1); // Температура обратки
    // Serial.println (temp_carbon);
    tempthermocouple_value = thermocouple.readCelsius(); // Температура дымохода
    period_millis = millis();
      }

//___________________________________________________________________________________
// детектирование погасания твердотопливного котла
	if (millis() - seconds_millis > 1000) // секундный софт-таймер
	{
			if ((temp_carbon - temp_obratka)<1)
			{
			cooling_counter++; // Инкрементируем переменную погасания.
			}
		else 
			{
			cooling_counter = 0; // Если температура в норме или выше уставки - сбрасываем отсчёт погасания.
			}
    seconds_millis = millis();     
	}
//___________________________________________________________________________________

  if ((temp_carbon > temp_min) || (temp_gaz > temp_min)) {
    digitalWrite(POMPA, LOW); //Включение помпы отопления
//    digitalWrite(13, HIGH);
  }
  else if ((temp_carbon < (temp_min-2)) && (temp_gaz < (temp_min-2))) {
    digitalWrite(POMPA, HIGH); //Выключение помпы отопления
 //   digitalWrite(13, LOW);    
  }
  
    if (millis() - lcdUpdate >= 500) // обновление дисплея
  {
    blink_display = !blink_display;
    dislay();
    lcdUpdate = millis(); // Обновляем время для дисплея
  }
  
  if (max_alarm) { // Eсли взведён флаг аварии перегрева
	return; // Блокируем дальнейшую работу котла
  }    
  
  if (temp_carbon > temp_max) { // Eсли температура выше max то взводим флаг аварии перегрева и закрываем поддувало
	max_alarm = 1;
    tone(BEEPER, 500, 20);
	zaslonka.attach(SERVO); // Иницилизация серво	
	zaslonka.write(zaslonka_min); // Установка заслонки в закрыто
	delay(1000); // Ждём установки сервы
	zaslonka.detach(); // Отключаем серву
  }
  
  if (cooling_alarm)  // Eсли взведён флаг аварии погасания
	{
		return; // Блокируем дальнейшую работу котла
	}  
  
   	if (tempthermocouple_value >= tempthermocouple_value_limit) // если температура дымохода выше верхнего установленного порога
		{
			smoke_temp_max_state = 1; // включаем режим ограничения температуры дымохода
		}
	else if ((tempthermocouple_value <= tempthermocouple_value_limit_low) && (smoke_temp_max_state)) // если дымоход остыл ниже установленного лимита при включённом режиме ограничения температуры дымохода
		{
			smoke_temp_max_state = 0; // выключаем режим ограничения температуры дымохода
		}

  if (cooling_counter > cooling_limit_1)  // Eсли температура ниже уставки на значение остывания и таймер остывания заполнен  
	{
		cooling_alarm = 1; // Взводим флаг аварии погасания 
		tone(BEEPER, 500, 20);
//		zaslonka.attach(SERVO,2300,800); // Иницилизация серво
		zaslonka.attach(SERVO); // Иницилизация серво	
		zaslonka.write(zaslonka_min); // Установка заслонки в закрыто
		delay(1000); // Ждём установки сервы
		zaslonka.detach(); // Отключаем серву
		if (cooling_counter > cooling_limit_2) {cooling_counter = cooling_limit_2;} // не выходим за пределы второго лимита счётчика остывания
	}
  if ((millis()-zaslonka_millis_old) > zaslonka_interval)
	{
		zaslonka_millis_old = millis();
		zaslonka_move(); // перемещение заслонки от температуры	  
	}
	if (key_data == key_pressed_encoder) // Если нажата кнопка энкодера
	{
		key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
		set_temp();
	}
	
	if (mode_set_temp) // Если поднят флаг установки температуры
	{
		set_temp();
	}	
	
	if (key_data == key_holded_encoder)
	{
		
	}	
}

void zaslonka_move() 
{
zaslonka.attach(SERVO); // Иницилизация серво
zaslonka_temp = zaslonka.read(); // Считываем с сервы последнее записаное значение
	if (smoke_temp_max_state)
	{
		uint8_t temp_temp = 0;
		if (tempthermocouple_value_limit > tempthermocouple_value)
			{
				temp_temp = tempthermocouple_value_limit - tempthermocouple_value;
				zaslonka_val=map(temp_temp, tempthermocouple_value_limit_low, tempthermocouple_value_limit, zaslonka_min, zaslonka_max);	// Переносим значение температуры разбаланса в диапазон движения сервы
			}
		else
			{
				zaslonka_val=0;	
			}

	}
	 else
	{
		uint8_t temp_temp = 0;
		if (temp_val > temp_carbon)
			{
				temp_temp = temp_val - temp_carbon;
				zaslonka_val=map(temp_temp, (temp_val - temp_ttk_diff), temp_val, zaslonka_min, zaslonka_max);	// Переносим значение температуры разбаланса в диапазон движения сервы
			}
		else
			{
				zaslonka_val = 0;
			}
	}
		if (abs(zaslonka_val-zaslonka_val_old)>3) // если разница между старым и новым значением меньше 3, серву зря не дёргаем
		// для уменьшения рывков перемещаем серву плавно, с задержками
			{
				if (zaslonka_val>zaslonka_temp) // если требуемое значение больше предыдущего
					{
						while (zaslonka_val>=zaslonka_temp) // пока новое значение сервы меньше старого инкрементироуем значение zaslonka_val на единицу, записываем в серву, выжидаем паузу
						{
							if (zaslonka_temp>zaslonka_max) {break;} 	// если вылезли за границы регулирования, то выходим из цикла	
							zaslonka.write(zaslonka_temp);  // Перемещение заслонки
							zaslonka_temp++;
							delay(45);
						}
					}					
						else if (zaslonka_val<zaslonka_temp) // если требуемое значение меньше предыдущего
					{
						while (zaslonka_val<=zaslonka_temp)// пока новое значение сервы больше старого декрементируем значение zaslonka_val на единицу, записываем в серву, выжидаем паузу
							{
								if (zaslonka_temp<zaslonka_min) {break;} 	// если вылезли за границы регулирования, то выходим из цикла				
								zaslonka.write(zaslonka_temp);  // Перемещение заслонки
								zaslonka_temp--;
								delay(45);
							}
					}		
		zaslonka_val_old=zaslonka_val;	// 				
		delay(100);
			}
			delay(15);
			zaslonka.detach();	// отключаем серву	
}

void time_out()  // Вывод Времени
{
  lcd.setCursor(15, 0);
  DateTime now = rtc.now();

  if (now.hour() < 10 ) {
    lcd.print('0');
  }
  lcd.print(now.hour(), DEC);

  lcd.print(':');

  if (now.minute() < 10 ) {
    lcd.print('0');
  }
  lcd.print(now.minute(), DEC);
}

void set_temp()
{
static uint32_t timeout_mode_set_temp = 0;
int newPos = encoder.getPosition();
if (!mode_set_temp)  // Если зашли в функцию первый раз - включаем повторные вызовы и запоминаем время входа
 {
	mode_set_temp = 1; 
	timeout_mode_set_temp = millis();
 }
    if (((pos - 2) >= newPos) || ((pos + 2) <= newPos))
  {
  timeout_mode_set_temp = millis();
    if (newPos > pos)
    {
		temp_val = temp_val + 1;
    dislay();
    lcdUpdate = millis(); // Обновляем время для дисплея
    }
    if (newPos < pos)
    {
		temp_val = temp_val - 1;	
    dislay();
    lcdUpdate = millis(); // Обновляем время для дисплея	
    }    
    pos = newPos;
  }
  if (((millis() - timeout_mode_set_temp) > 7000) || (key_data == key_pressed_encoder))
  {
  	mode_set_temp = 0; 
  	EEPROM.put(2, temp_val); //Запись в ЕЕПРОМ 
	delay (30);	
  	lcd.clear();
    lcd.setCursor(5, 1);
    lcd.print("COXPAHEHO");
	lcdUpdate = millis(); // Обновляем время для дисплея (на задержку отображения подтверждения)
	pos = newPos;
  }
  
}

void menu()
{
static uint32_t timeout_mode_menu = 0;
int newPos = encoder.getPosition();
    if (((pos - 2) >= newPos) || ((pos + 2) <= newPos))
  {
    if (newPos > pos)
    {
		temp_val = temp_val + 1;
    }
    if (newPos < pos)
    {
		temp_val = temp_val - 1;		
    }    
    pos = newPos;
  }
}

void dislay() //Обновление основного дисплэя
{
  time_out();
  lcd.setCursor(0, 0);
  lcd.print("\x0D""A3: ");
  lcd.print(temp_gaz, 1); // Выводм тепературу xx.x Комнатная
  lcd.print ("\xDF ");

  lcd.setCursor(0, 1);
//  lcd.print("B\x0A""X: ");
  lcd.print("TTK: ");
  lcd.print(temp_carbon, 1); // Выводм тепературу xx.x Выход из котла
  lcd.print ("\xDF ");
  
  lcd.setCursor(11, 1);
  lcd.print ("\xBF""CT: ");
  
  lcd.print(temp_val, 1); // Выводм тепературу xx.x Установленная рабочяя
  if (!mode_set_temp || blink_display)
  {
  lcd.print ("\xDF");  
  }
  else if (!blink_display && mode_set_temp)
  {
  lcd.print(" "); // Выводм тепературу xx.x Установленная рабочяя  
  }


  lcd.setCursor(0, 2);
  lcd.print("BXO: ");
  lcd.print(temp_obratka, 1); // Выводм тепературу xx.x Вход в котел
  lcd.print ("\xDF");
  
 /*
  lcd.setCursor(12, 2);
  lcd.print ("M\x09H: ");
  lcd.print(temp_min, 1); // Выводм тепературу xx.x Минимальная для отключения насоса
  lcd.print ("\xDF");
*/

  lcd.setCursor(11, 2);
  lcd.print ("\x08\x0A""M: ");
  lcd.print(tempthermocouple_value); // Выводим тепературу дымохода
  lcd.print ("\xDF");
  if (tempthermocouple_value<99) // Если значение температуры ниже 99 градусов
  {
  lcd.print (" "); // На всяк случай затираем знакоместо, где мог остаться символ градуса от трёхзначных значений  температуры 
  }


//  lcd.setCursor(0, 3);
//  lcd.print(rezhim_t[rezhim]); // Вывод режима работы
  if (zaslonka_val == zaslonka_min)
  {
  lcd.setCursor(16, 3);
  lcd.print("3AKP");    
  }
  else if (zaslonka_val == zaslonka_max)
  {
  lcd.setCursor(16, 3);
  lcd.print("OTKP");    
  }
  else
  {
  lcd.setCursor(16, 3);   
  lcd.print(zaslonka_val - zaslonka_min);
  if (((zaslonka_val - zaslonka_min) <= 99) && ((zaslonka_val - zaslonka_min) > 9))
   {
    lcd.print("  ");  
   }
  else if ((zaslonka_val - zaslonka_min) <= 9)
   {
    lcd.print("   ");  
   }
  else
   {
    lcd.print(" "); 
   }
  }
  
    if ((cooling_counter < cooling_limit_2)  &&  (cooling_alarm))  // Eсли взведён флаг погасания
	{
  lcd.setCursor(0, 3);
  lcd.print("OCT\x0A""BAH\x09""E...");	// Индицируем "ОСТЫВАНИЕ"
  /*
  lcd.setCursor(12, 3);
  lcd.print(cooling_counter);  
  lcd.setCursor(18, 3);
  lcd.print(cooling_alarm);   
  */
	}
	else if (smoke_temp_max_state)
	{
  lcd.setCursor(0, 3);
  lcd.print("\x08\x0A""MOXO""\x08 > ");	// Индицируем "ДЫМОХОД!"	
  lcd.setCursor(10, 3);
  lcd.print(tempthermocouple_value_limit);  
  lcd.setCursor(14, 3);
  lcd.print ("\xDF");	
	}
	else	
	{
	lcd.setCursor(0, 3);
	lcd.print("                ");	//	затираем строку до вывода угла поворота
	}
}

byte get_key()
{
// версия 2 - значение возвращается только при отпускании кнопки (как для короткого нажатия, так и для удержания)
uint8_t trigger_push_hold_counter = 10; // задержка триггера кратковременного/длительного нажатия (проходов функции, умноженных на задержку)
static uint8_t val_key_encoder, val_key_rezhim;
static uint32_t key_delay_millis;
if ((millis() - key_delay_millis) > 50) //обрабатываем нажатия инкрементом переменной только если после предыдущей обработки прошло не менее 50 миллисекунд
{
  if (!(PIND & (1 << PIND5))) {val_key_encoder++;} //нажатие кнопки энкодера
  if (!(PIND & (1 << PINB4))) {val_key_rezhim++;} //нажатие кнопки режима
  key_delay_millis = millis();
}
if ((val_key_encoder > 0) && (PIND & (1 << PIND5))) //если клавиша энкодера отпущена, но перед этим была нажата 
{
  if (val_key_encoder <= trigger_push_hold_counter) {val_key_encoder = 0; return key_pressed_encoder;} //кратковременно - возвращаем 1
  else if (val_key_encoder > trigger_push_hold_counter) {val_key_encoder = 0; return key_holded_encoder;} //длительно - возвращаем 3
}
if ((val_key_rezhim > 0) && (PIND & (1 << PINB4))) //если клавиша режима отпущена, но перед этим была нажата 
{
  if (val_key_rezhim <= trigger_push_hold_counter) {val_key_rezhim = 0; return key_pressed_rezhim;} //кратковременно - возвращаем 2
  else if (val_key_rezhim > trigger_push_hold_counter) {val_key_rezhim = 0; return key_holded_rezhim;} //длительно - возвращаем 4
}
return 0; // если ни одна из кнопок не была нажата - возвращаем 0

}

/*
06.12.2019
Выброшен PID алгоритм, заменён на пропорциональное регулирование с задаваемой полосой. Авария перегрева дымохода вместо триггерной сделана также с полосой регулирования, с приоритетом выше основного регулирования по температуре теплоносителя.

13.11.2018
-добавил затирание нижней строки, т.к. оставались надписи ДЫМОХОД и ОСТАНОВКА даже после снятия режимов

12.11.2018 
- попытка добавить ограничение температуры дымохода

24.03.2018
-плавное движение заслонки


TODO:

-корректировка ручного изменения положения сервы с заданным интервалом

-запись лога с привязкой ко времени на флеш-память RTC и вывод его в сериал, с возможностью включения/отключения

-сброс аварий остывания и перегрева кнопкой "РЕЖИМ"

-переписать полностью меню

-сделать попеременный вывод параметров и полноэкранных часов (возможно)

-реализовать летний режим (постоянный вывод часов, периодическое включение насоса на минуту как защита от закисания ротора)

-реализовать контроль включения насоса (потребуется добавление узла контроля тока с опторазвязкой)

-возможно добавление PIR детектора движения для отключения дисплея когда в помещении никого нет
*/

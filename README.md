#
#                  Отладочный модуль
#     CC3200 SimpleLink LaunchPad (i2s) + SimCom5320 (pcm)
#


*  Порты по-умолчанию :

```
5000 - tcp порт для команд управления устройством
5001 - tcp порт для подачи АТ-команд на 3G-модуль sim5320
5002 - udp порт для приема разговорных rtp-пакетов от клиента
5050 - udp порт для передачи breadcast сообщений, тело сообщения :'sim5320_cc3200:192.168.0.103',
      где 192.168.0.103 - ip адрес устройства (может быть другим)
```


## I.  Режим STA (Station)

  После подачи питания устройство начинает выполнять программу, записанную в dataflash (файл /sys/mcuimg.bin)

На консоли это выглядит примерно так :

1. Начало работы

```
Init UARTA0 (CONSOLE) done.
Init UARTA1 (GSM) done.
                Version 2.9
[MAIN] Create Tx Buffer Play Ok (8000)
[MAIN] Create Rx Buffer Record Ok (4000)
```

2. Из файла www/ap.conf читаются данные для подключения к точке доступа (далее AP),
  если такой файл отсутствует - работа в режиме STA невозможна, нужно перевести устройство
  в режим AP (Access Point) и командой управления cmd=0 записать данные в файл www/ap.conf.
  Если файл присутствует - устройство подключится к AP с параметрами, указанными в файле www/ap.conf
  например так :

```
[IP] Connect to AP 'ssid_dlink' start...
[FILE] file 'www/ap.conf' info: flag=0 len=66/3656
[FILE]  cmd=0
        ssid='ssid_dlink'
        key='key_dlink'
        type=2 (wpa/wpa2)
[WLAN] STA Connected to AP 'ssid_dlink', BSSID: 64:66:b3:c6:e6:0d
```

3. После успешного подключения, устройство получает по DHCP IP,Gateway,dsn :

```
[NET] Acquired: IP=192.168.0.103, Gateway=192.168.0.1, dns=192.168.0.1
[IP] Device connected to AP 'ssid_dlink'
        IP 192.168.0.103
        MASK 255.255.255.0
        BCAST 192.168.0.255:5050
```

4. Далее устройство читает из файла /sys/shadow.conf логин и пароль для выполнения процедуры
  авторизации клиентов (команда cmd=12 по порту 5000), если файл отсутствует, то он будет автоматически
  создан с такими значениями по-умолчанию : login=def_login passwd=def_passwd.
  Без авторизации доступ к порту 5001 НЕ предоставляется :

```
[FILE] file '/sys/shadow.conf' info: flag=0 len=66/3656
[FILE]  cmd=11
        login='def_login'
        passwd='def_passwd'
```

5. Запускаются "нитки" для обслуживания портов 5000,5001 и включается модуль SIM5320 :

```
[CTL] server start, listen port 5000
[TCP] server start, listen port 5001
[VIO] 0
[TCP] GSM ON.(1)
[TCP] Wait auth. client...
[VIO] 1
```

6. Запускается "нитка" передачи broadcast сообщений :

```
[BCAST] Start broadcast task...
[BCAST] Send broadcast msg 'sim5320_cc3200:192.168.0.103'
START
[BCAST] Send broadcast msg 'sim5320_cc3200:192.168.0.103'
+STIN: 25
+STIN: 24
+CPIN: READY
SMS DONE
[BCAST] Send broadcast msg 'sim5320_cc3200:192.168.0.103'
PB DONE
[BCAST] Send broadcast msg 'sim5320_cc3200:192.168.0.103'
[BCAST] Send broadcast msg 'sim5320_cc3200:192.168.0.103'
[BCAST] Send broadcast msg 'sim5320_cc3200:192.168.0.103'
```

  Сообщения выдаются каждые 5 секунд, до тех пор пока на устройство не придет
любая валидная команда управления (порт 5000), например команда авторизации :

```
[MD5] Access granted for 192.168.0.109
[CTL]   cmd=12
        time='1440493267'
        md5='70ff4601ce53873e639e0b4a690cd638'
        mode=0
        auth_ip=192.168.0.109
```

  После чего выдача broadcast сообщений прекращается, если же устройство "потеряет"
связь с AP, будет выполнена полная перезагрузка устройства, с предварительным выключение
модуля SIM5320.

```
[BCAST] Stop broadcast task...
```

7. После успешного выполнения команды cmd=12 следует выполнить команду cmd=5

```
[CTL]   cmd=5 - udp:192.168.0.109:5004 port=5002
```

  Эта команда назначит адрес:порт rtp-клиента, которому будут отправляться разговорные пакеты с устройства,
а также rtp-порт, на котором устройство будет принимать разговорные пакеты от клиента (по-умолчанию это 5002)

8. После чего клиент, получивший авторизацию (по результату команды cmd=12) может подключаться
  к порту 5001 и начинать передачу АТ-команд на модуль SIM5320 :

```
[TCP] Auth. client 192.168.0.109:45231 connected.
ATE0
OK
```

Если нет ответа на команду, возможно модуль SIM5320 не включен, для его включения есть команда
управления cmd=1, а для проверки включился ли модуль SIM5320 есть команда управления cmd=7


## II.  Режим AP (Access Point)

```
  Отличается от режима STA только тем что сообщения broadcast не выдаются,
а ip адрес устройства всегда 192.168.1.1.
В остальном он не отличается от редима STA.
```

## III.  Рекомендуемая последовательность АТ-команд на модуль SIM5320 :

```
ATE0
AT+CMGF=0
AT+CREG?
AT+CNUM
AT+COPS?
AT+CPSI?
```


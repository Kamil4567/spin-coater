# ESC

## Fabryczne ustawienia ESC

Throthle center 1500 = stop

1500->1000 = spin left

1500->2000 = spin right

### Zmiana ustawień ESC

Dodany tryb programowania ESC do menu serwisowego. Połączenie z komputerem tylko używając starej wersji SerialPortConnector(https://drive.google.com/file/d/1XACXV4w1ldtp71KJtZOjKVYttHv50o5C/view) na ustawieniach 19200 i ECHO ON. 

Oprogramowanie/firmware w tej wersji nie ma ustawień dotyczących sterowania prędkością przez PWM. Potrzebna zmiana firmware.

## Modyfikacja firmware

Procesor ESC to Artery AT32F421K8U7, obok 4 nieopisane pady(sygnały do ustalenia). 

Programator klon STLINK. Programowanie przez pakiet pythona(3.12) **pyOCD(0.46.1)** z specjalną doinstalowaną paczką `ArteryTek.AT32F421_DFP.2.2.0.pack`(https://raw.githubusercontent.com/ArteryTek/PACKs/master/Keil5/ArteryTek.AT32F421_DFP.2.2.0.pack). 

Wyświetlenie zawartości paczki
```
py -3.12 -m pyocd list --targets --pack .\ArteryTek.AT32F421_DFP.2.2.0.pack
```

Szukany target to `_at32f421k8u7` (znajduje się na liście)

Pady obok(w kolejności od układu):
- SWCLK
- SWDIO
- 3.3V
- GND

Sprawdzenie czy python widzy programator
```
py -3.12 -m pyocd list
```

Sprawdzenie czy programator widzi układ
```
py -3.12 -m pyocd cmd -t _at32f421k8u7 --pack ArteryTek.AT32F421_DFP.2.2.0.pack -O connect_mode=attach -c "read32 0x08000000 32"
```

Zrzut pamięci do pliku(dwa zrzuty i porównanie plików) + obszar konfiguracji
```
py -3.12 -m pyocd cmd -t _at32f421k8u7 --pack ArteryTek.AT32F421_DFP.2.2.0.pack -O connect_mode=attach -c "savemem 0x08000000 0x10000 esc_backup.bin"

py -3.12 -m pyocd cmd -t _at32f421k8u7 --pack ArteryTek.AT32F421_DFP.2.2.0.pack -O connect_mode=attach -c "savemem 0x08000000 0x10000 esc_backup2.bin"

Get-FileHash esc_backup.bin, esc_backup2.bin

py -3.12 -m pyocd cmd -t _at32f421k8u7 --pack ArteryTek.AT32F421_DFP.2.2.0.pack -O connect_mode=attach -c "read32 0x1FFFF800 16"
```

Obszar konfiguracji z odczytu: `1ffff800:  00ff5aa5 00ff00ff 00ff00ff 00ff00ff    |..Z.............|`


Pobranie nazwy firmware:
```
py -3.12 -c "import re; [print(m.decode()) for m in re.findall(rb'[ -~]{6,}', open('esc_backup.bin','rb').read())[:60]]"
```

Nazwa firmware:
```
jakieś śmieci...
AORC_K_F421
AORC_K_A_F421
AORC_K_F421
```

## Kompilacja firmware

Kod firmware znajduje się w `firmware_esc`. Kompilacja(wewnątrz folderu):
```
& .\tools\windows\make\bin\make.exe "ARM_SDK_PREFIX=$((Resolve-Path .\tools\windows\xpack-arm-none-eabi-gcc-10.3.1-2.3\bin).Path)\arm-none-eabi-" AORC_K_A_RPM_F421
```

Wgranie używając SWD(z katalogu 'programowanie ESC'):
```
py -3.12 -m pyocd flash -t _at32f421k8u7 --pack ArteryTek.AT32F421_DFP.2.2.0.pack ../firmware_esc/obj/AM32_AORC_K_A_RPM_F421_2.22.hex
```

Udało się wgrać nowy firmware.

Nowe ustawienia:
- Napięcie odcięcia: 10V
- Maksymalny prąd: 12A 
- Servo low: 810
- Servo high: 2190
- Servo neutral: 1500
- Servo dead: 10

# Uchwyt
```
   rurka Ø6 (wiruje)
        ║
  ┌─────╫─────┐
  │ [łożysko] │  MR106 6x10x3
  │ [simmer ] │  6x12x4, warga na zewnątrz (w stronę atmosfery)
  │  ○──╫─────┼──→ króciec boczny → wąż do pompy
  │ [simmer ] │  6x12x4, warga na zewnątrz
  │ [łożysko] │  MR106 6x10x3
  └─────╫─────┘   obudowa: wydruk 3D lub toczone aluminium
        ║          kołek przeciw obracaniu oparty o ramę
```

Pompa podciśnienia: https://allegro.pl/produkt/pompa-prozniowa-podcisnieniowa-ze-stojakiem-80kpa-10w-dc-12v-8ac11ec4-97c9-4a44-89d6-9ad8165243d5?offerId=18887387625

Mocowanie próbki: twarda płaska odporna chemicznie powierzchnia, na środku otwór z podciśnieniem i oring wystający 0.1-0.2 mm do uszczelnienia. 

Oś silnika ma 3.75x11mm(średnica x wysokość) - możliwa potencjalna wymiana na dłuższą
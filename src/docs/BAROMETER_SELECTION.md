# Barometre secimi

Firmware ayni anda tek bir barometre kullanir. Takili sensoru `src/hardware_config.h`
icindeki `VARIO_BAROMETER_TYPE` satirindan secin:

```cpp
#define VARIO_BAROMETER_TYPE BAROMETER_MS5607
```

Desteklenen secimler ve varsayilan I2C adresleri:

| Sensor | Secim degeri | Varsayilan adres | Alternatif adres |
|---|---|---:|---:|
| MS5607 | `BAROMETER_MS5607` | `0x77` | `0x76` |
| MS5611 | `BAROMETER_MS5611` | `0x77` | `0x76` |
| BMP280 | `BAROMETER_BMP280` | `0x76` | `0x77` |
| BME280 | `BAROMETER_BME280` | `0x76` | `0x77` |
| LPS22HB | `BAROMETER_LPS22HB` | `0x5C` | `0x5D` |
| DPS310 | `BAROMETER_DPS310` | `0x77` | `0x76` |
| HP303B | `BAROMETER_HP303B` | `0x77` | `0x76` |
| BMP388 | `BAROMETER_BMP388` | `0x76` | `0x77` |
| BMP390 | `BAROMETER_BMP390` | `0x76` | `0x77` |
| BMP580 | `BAROMETER_BMP580` | `0x47` | `0x46` |
| BMP581 | `BAROMETER_BMP581` | `0x47` | `0x46` |

Kartiniz alternatif adresi kullaniyorsa secim satirinin altina adresi de yazin:

```cpp
#define VARIO_BAROMETER_TYPE BAROMETER_BMP280
#define VARIO_BAROMETER_ADDRESS 0x77
```

Firmware secilen modele ait olmayan bir I2C adresini derleme sirasinda hata
olarak bildirir. Boylece yanlis adresle uretilen bir yazilim cihaza yuklenmez.

Tum sensorler 3.3 V, GND, SDA GPIO 8 ve SCL GPIO 9 baglantilarini kullanir.
BME280 secildiginde yalnizca basinc ve sicaklik okunur; nem verisi vario
hesabina katilmaz. Yanlis sensor veya adres secilirse seri porttaki baslangic
gunlugu algilama hatasini bildirir ve vario gorevi guvenli bicimde bekler.

DPS310 ile HP303B ayni register ve kompanzasyon duzenini, BMP388 ile BMP390
BMP3 ailesi duzenini, BMP580 ile BMP581 ise BMP5 ailesi duzenini kullanir.
Secim sensor ailesine gore yapilir; firmware baslangicta gercek cip kimligini
kontrol eder.

Ortak register ve kompanzasyon yapisini kullanan modeller tek aile surucusunde
toplanir: `ms56xx`, `bmp28x`, `dps3xx`, `bmp3xx` ve `bmp5xx`. Model secimi,
beklenen cip kimligini ve modele ozel telafi yolunu derleme sirasinda belirler.
LPS22HB farkli bir yapi kullandigi icin ayri surucude kalir.
Fabrika kalibrasyon katsayilari kodda veya NVS'de sabit tutulmaz;
her acilista sensorun kendi PROM/NVM alanindan okunur ve dogrulanir. Kullaniciya
ait kalici basinc referansi QNH ayaridir ve normal ayar deposunda saklanir.

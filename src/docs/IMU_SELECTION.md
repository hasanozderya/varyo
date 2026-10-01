# IMU seçimi

Takılı IMU'yu `src/hardware_config.h` içindeki `VARIO_IMU_TYPE` satırından seçin:

```cpp
#define VARIO_IMU_TYPE IMU_MPU6050
```

Desteklenen seçenekler ve varsayılan I2C adresleri:

| Seçenek | Varsayılan adres | Alternatif adres |
|---|---:|---:|
| `IMU_MPU6050` | `0x68` | `0x69` |
| `IMU_LSM6DS3` | `0x6A` | `0x6B` |
| `IMU_LSM6DS3TR` | `0x6A` | `0x6B` |
| `IMU_LSM6DSOX` | `0x6A` | `0x6B` |
| `IMU_BMI160` | `0x68` | `0x69` |
| `IMU_BMI270` | `0x68` | `0x69` |
| `IMU_BNO055` | `0x28` | `0x29` |

Alternatif adres gerekiyorsa ayrıca şu satırı tanımlayın:

```cpp
#define VARIO_IMU_ADDRESS 0x69
```

Tüm sürücüler ivmeyi `g`, jiroskobu `°/s` cinsinden ortak füzyon katmanına
aktarır. Örnekleme hızı yaklaşık 100 Hz, ivme aralığı yaklaşık ±8 g ve
jiroskop aralığı ±500 °/s olarak ayarlanır. BNO055, cihazın kendi yönelim
sonucunu kullanmaz; projenin mevcut yönelim ve Kalman zincirine ham AMG
ivme/jiroskop verisi verir.

Kodda hiçbir IMU'ya ait cihaz-özel kalibrasyon katsayısı bulunmaz. Zemin
kalibrasyonu jiroskop sıfır kaymasını, ivme büyüklüğü düzeltmesini ve kalibrasyon
sıcaklığını her sensör tipi için ayrı bir NVS kaydında saklar. Sensör değişince
yalnız o sensörün kaydı yüklenir; kaydı yoksa IMU füzyonu açılmaz ve barometrik
vario güvenli yedek olarak çalışır. Yeni IMU takıldıktan sonra cihaz sabit ve
titreşimsizken uygulamadaki zemin kalibrasyonunu bir kez çalıştırın. Açılışta
otomatik kalibrasyon yapılmaz; böylece cihaz hareket halindeyken yanlış ofset
öğrenmez.

BMI270 açılışta Bosch yapılandırma dosyasına ihtiyaç duyduğu için PlatformIO,
bu dosyayı içeren SparkFun BMI270 Arduino paketini bağımlılık olarak indirir.

Ortak kalibrasyon ve NVS işlemleri `src/imus/imu.cpp/.h` içindedir.
LSM6DS3, LSM6DS3TR-C ve LSM6DSOX ortak register yapıları nedeniyle
`lsm6ds_family.cpp/.h` içinde birleştirilmiştir. Başlatma ve veri yapısı farklı
olan MPU6050, BMI160, BMI270 ve BNO055 ayrı sürücülerde kalır.

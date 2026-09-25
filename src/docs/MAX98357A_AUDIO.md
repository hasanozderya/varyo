# MAX98357A hoparlör çıkışı

ESP32-S3 firmware artık pasif buzzer PWM çıkışı yerine MAX98357A I²S amfi kullanır.
Hoparlör: 8 Ω, 1,5 W, 20×30 mm. MAX98357B farklı veri biçimi kullanır; bu ayarlar A modeli içindir.

## Bağlantı

| MAX98357A | Bağlantı |
|---|---|
| DIN | ESP32 GPIO4 |
| BCLK | ESP32 GPIO5 |
| LRC / LRCLK / WS | ESP32 GPIO6 |
| VIN | 5 V besleme |
| GND | ESP32 ve besleme ile ortak GND |
| GAIN | VIN'e bağlanarak 6 dB kazanç; modülün direnç/jumper düzenini kontrol edin |
| SD / EN | VIN'e bağlanarak etkin; GPIO7 kullanılmıyor |
| SPK+ ve SPK− | Hoparlörün iki ucu |

GPIO4 üzerindeki eski buzzer bağlantısını kaldırın. Hoparlör uçlarından hiçbirini GND'ye
bağlamayın; çıkış köprülüdür. Amfinin 5 V hatlarını ESP32 GPIO'larına bağlamayın.
Kart üzerindeki GAIN ve SD dirençleri modüle göre değişebilir; bağlantıyı modül şemasıyla
doğrulayın. Boşta GAIN tipik olarak 9 dB'dir, 6 dB değildir.

Amfiyi sensörlerin 3,3 V hattından beslemeyin. Kısa besleme kablosu ve amfi yakınında
100 nF seramik ile yaklaşık 220 µF ek besleme kondansatörü önerilir; mevcut modül
kondansatörlerini de hesaba katın. Hoparlör kablolarını birlikte büküp I²C ve barometreden
uzak geçirin. Akustik/mekanik titreşimin basınç ölçümüne etkisini cihaz kasasında sınayın.

## Yazılım davranışı

- 16 kHz, 16 bit stereo Philips I²S; iki kanala aynı sinüs gönderilir. MCLK gerekmez.
- Ayrı Core 1 ses görevi, dört adet 128 çerçeveli DMA tamponu kullanır.
- Sensör görevi yalnızca frekans ve ses seviyesini bildirir; I²S yazmasını beklemez.
- Başlangıç/bitiş genliği en fazla 5 ms içinde yumuşatılır.
- Komut 200 ms güncellenmezse ses susturulur. Kuyruktaki DMA sesi ve sönüm nedeniyle
  fiziksel çıkış daha sonra susar; bu süre gerçek cihazda doğrulanmalıdır.
- I²S hatasında sürücü iki saniye sonra tekrar açılır; hata teknik günlüğe yazılır.
- Android ses seviyesi ve sessiz ayarı mevcut BLE komutlarıyla çalışır.
- Yükseliş varsayılanı 700–2200 Hz, normal alçalış başlangıcı 450 Hz'dir.
  Eski 600/1800/300 Hz üçlüsü yüklenirken güncellenir; diğer özel ton ayarları korunur.
  Geçiş sonraki ayar kaydında kalıcı olur; QNH ve sensör ayarları değiştirilmez.
- Açılışta zorunlu test sesi yoktur; mevcut uçuş ve sensör hata sesleri korunur.

Uygulamada %100 ses, PCM tepe genliğinin %80'i anlamına gelir. Üretici formülüne göre
6 dB kazançta bu sinüsün nominal gücü 8 Ω üzerinde yaklaşık 0,52 W'tır; tam PCM
genliğinde yaklaşık 0,81 W olurdu. Bunlar ideal hesap değerleridir, güç ölçümü değildir.
Kazanç bağlantısı farklıysa hesap geçerli olmaz. 1,5 W hoparlör etiketi sürekli tüketimi
değil taşıyabildiği nominal gücü belirtir.

## Doğrulama

Host kontrolleri sinüs genliği/frekansı, stereo eşitliği, bloklar arası süreklilik,
susturma, komut zaman aşımı ve ayar geçişini kapsar. PlatformIO derlemesi ayrıca
gerçek ESP32 I²S API uyumunu kontrol eder. I²S hoparlör varlığını algılamaz;
başarılı sürücü açılışı amfinin bağlı olduğunu kanıtlamaz.

Donanım bağlıyken önce düşük sesle yükseliş/alçalış/sessiz modlarını deneyin;
ardından azami seste besleme kararlılığını ve masada vario ölçümünü karşılaştırın.
Bu değişiklik kapsamında karta yükleme ve fiziksel ses/güç ölçümü yapılmadı.

Kaynaklar:
- [MAX98357A/B veri sayfası](https://www.analog.com/media/en/technical-documentation/data-sheets/max98357a-max98357b.pdf)
- [ESP-IDF I²S sürücüsü](https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32s3/api-reference/peripherals/i2s.html)

# Firmware 2.0.0 — uygulanan düzenlemeler

**11 Eylül 2026:** Sonraki sürüm 2.1.0'a XCTrack BLE desteği eklendi.
[Bağlantı adımları ve 2.1.0 doğrulaması](XCTRACK_SETUP.md). Aşağıdaki kayıt
2.0.0 değişikliklerini ve o sürümün test sonuçlarını anlatır.

Tarih: 10 Eylül 2026. [İlk inceleme](FIRMWARE_REVIEW.md) sonrasında ölçüm,
zamanlama, arıza davranışı, ayar saklama, ses ve BLE düzeltmeleri uygulandı.
Android karşılığı `C:/Users/ozder/StudioProjects/vario_ble` projesindedir.

## Ölçüm ve zamanlama

- Yönelim hesabı artık üç jiroskop eksenini quaternion ile birlikte kullanır.
  IMU okumaları arasındaki gerçek süre kullanılır; ivme normu ve yönü uygun
  değilse ivmeölçer düzeltmesinin etkisi azaltılır. Cihaza ait mevcut eksen
  ölçekleri ve kayıtlı yer kalibrasyonu korunur.
- MS5607 dönüşümleri ardışık yürütülür; örnek zamanı dönüşümün orta noktasına
  atanır. Basınç regresyon tamponu 900 ms pencereyi yaklaşık 100 Hz'de tutacak
  kapasitededir. Gerçek örnek hızı cihazdaki sayaçlarla ölçülmelidir.
- OLED aktarımı 64 baytlık parçalara bölünür ve sensör çevriminden sonra
  gönderilir. I²C işlem zaman aşımı 4 ms'dir. Büyük framebuffer aktarımı boyunca
  sensörleri dışarıda bırakan eski kilit kaldırılmıştır.
- Ana ölçüm 500 ms, barometre 500 ms, IMU 200 ms içinde yenilenmelidir.
  Eski veri geçerli sıfır gibi gösterilmez; BLE ölçümün kendi zamanını taşır.
  Sensörler açılışta bulunamasa veya sonradan cevap vermese yeniden denenir.
  Reset/bekleme adımları çevrimlere yayılır; sıkışmış SDA için mevcut hat üzerinde
  kurtarma darbesi denenir. Ölçüm görevi watchdog'a kaydedilir.
- Düzenli metin logları ve IMU kalibrasyonu NVS yazımı bakım görevine taşındı.
  BLE trend sayfalaması istenen sayıda noktayı alır; tüm geçmişi tek uzun
  kritik bölümde kopyalamaz.

## Uçuşta açılış ve kalibrasyon

- Açılışta cihazın sabit tutulması şartı ve hareketten otomatik sıfır öğrenme
  yoktur. İlk basınç örneklerinden vario hesaplanır. Kaydedilmiş kalibrasyonla
  IMU uygun hale gelince barometrik hıza oturan yumuşak füzyon geçişi yapılır.
- Bilinen irtifa/QNH değişiminde iki kestirimci ve barometrik geçmiş yeni
  referansa dönüştürülür. Dikey hız sıfırlanmaz. Fiziksel modelde QNH değişiminin
  getirdiği küçük hız ölçeği değişimi korunur.
- Android grafiği ilk yeni referanslı örnekte yeni bölüm açar; kalkışa göre
  irtifa farkına kalibrasyon sıçraması eklenmez.
- Yalnız IMU yer kalibrasyonu ve ham filtre katsayısı değişikliği için 8 saniye
  sakin ölçüm gerekir. Yeni IMU verisi olmayan tek bir yoklama bu sayacı bozmaz;
  100 ms taze IMU kanıtı yoksa izin düşer. Uçuş durumu etkinse izin verilmez.
  Bu bir yerde olma tahminidir; GPS olmadan düz/sakin uçuşu kesin ayıramaz.
  IMU kalibrasyonu yine yalnız yerde, kullanıcı komutuyla yapılmalıdır.
- IMU kayıt işlemi tamamlanmadan başarı bildirilmez. Ayarlar tek sürümlü,
  sağlama toplamlı NVS blob'unda saklanır. Yazma başarısızsa çalışan ayar
  değiştirilmez. Eski NVS anahtarları ilk geçişte okunur ve silinmez.

## Ses ve uçuş durumu

- Normal çöküş sesi gerçekten ayarlanan `sinkAlarm` eşiğinde başlar.
  Ayrı güçlü çöküş bildirimi, eşik histerezisi, ses seviyesi, bip aralıkları ve
  isteğe bağlı zayıf kaldırıcı bildirimi eklendi; Android ayarlarında görünür.
- Veri geçersizleşince uçuş tonu kesilir. Arıza 3 saniye sürerse ayırt edilebilir
  kısa çift bildirim, sonra 10 saniye aralıkla tekrar verilir. Ses seviyesi 0
  bu bildirimi de kapatır. PWM yüzdesi ölçülmüş akustik dB karşılığı değildir.
- Cihazda belirsiz/yerde/uçuşta/tamamlandı durumları ve süre vardır. Geçerli
  GPS yer hızı ≥5 m/s, 3 saniye sürerse; GPS yoksa |vario| ≥1 m/s, 8 saniye
  sürerse uçuş başlar. İniş için 60 saniye sakinlik ve GPS hızı <1.5 m/s gerekir.
  GPS kaybı iniş sayılmaz. Elle durdurma devam eden hareketle hemen geri açılmaz.
- Android kayıt başlat/bitir düğmesi yeni firmware'e uçuş durumu komutu da
  gönderir. Telefon kaydı elle başlar; cihazın otomatik durum algısı tek başına
  Android'de kayıt başlatmaz. OLED'de ölçüm durumu, BLE ve uçuş süresi görünür.

## BLE ve Android

- Ana telemetri 20 bayt/v1 olarak korundu. GPS ve cihaz sağlığı iki ayrı
  20 baytlık karakteristikle 2 Hz iletilir. Protokol ayrıntıları
  [BLE_ANDROID.md](../BLE_ANDROID.md) içindedir.
- GPS konumu/kalitesi/hızı eskirse kullanılmaz. Aynı FIX'in tekrar gönderilmesi
  konumu tazelemez. Android önce cihaz GPS'ini, yoksa güncel telefon GPS'ini
  kullanır. Kaynak değişiminde ekranda ve GPX dışa aktarımında iz bölünür;
  CSV'de kaynak, HDOP ve uydu sayısı korunur. HDOP metre olarak sunulmaz.
- Komut kanalı eşleşmiş, şifreli ve MITM doğrulamalı bağlantı ister. İlk
  eşleşmede OLED'deki PIN Android penceresine girilir. Eşleşme sonraki bağlantıda
  kullanılır; otomatik denemeler yeniden PIN penceresi açmaz. Eşleşme silinirse
  elle bağlantı gerektiği belirtilir. Bu sürümde tek sahip telefona özel bir
  izin listesi veya fiziksel eşleştirme düğmesi yoktur.
- Teşhis ekranında ölçüm sağlığı, firmware sürümü, kalibrasyon hazırlığı vardır.
  `status` ayrıca örnek/iyileşme/gecikme sayaçları, boş bellek, görev stack'i,
  reset nedeni ve GPS ayrıntılarını verir.

## Doğrulama

- `python src/checks/run_host_checks.py`: üretim C++ modülleri MSVC ile derlenip
  çalıştırıldı; **80 kontrol geçti**. Donanım/RTOS/NVS için sahte arayüz kullanır.
  Yatık dönüş, örnek kaybı, sayaç taşması, QNH sürekliliği, eski veri, uçuş/iniş,
  ses eşikleri, protokol yerleşimi ve NVS hata/bozulma senaryolarını kapsar.
- PlatformIO ESP32-S3 hedef derlemesi başarılı. Platform, yerel çalışan
  [pioarduino 55.03.311 / Arduino 3.3.11](https://github.com/pioarduino/platform-espressif32/releases/tag/55.03.311)
  sürümüne; U8g2 2.36.18 ve TinyGPSPlus 1.1.0'a sabitlendi. Flash/PSRAM/pin
  seçimi değiştirilmedi.
  Son derlemede RAM 42.560/327.680 bayt (%13), uygulama flash kullanımı
  823.494/1.310.720 bayt (%62,8). Çıktı:
  `../../.pio/build/esp32-s3-devkitc-1/firmware.bin`.
- Android: **62 test geçti**. Test edilen kaynaklar SHA-256 ile kontrol edilerek
  asıl projeye aktarıldı; asıl projede `flutter analyze --no-pub` temiz geçti.
  Test paketi eski ve yeni BLE, ilk PIN/eşleşmiş yeniden bağlantı, GPS kaynağı
  geçişi, GPX/CSV, irtifa referansı ve yatay ses ayarlarını kapsar.
- Kotlin BLE yardımcısı bağımsız derlendi; paket sürümü, genişletilmiş bayraklar,
  işaretsiz zaman ve eski telemetri uyumluluğu için 4 davranış kontrolü geçti.
- Cihaza yükleme, APK üretimi ve fiziksel uçuş testi yapılmadı. Sahte donanımla
  geçen testler gerçek I²C arıza toparlanmasını, PIN penceresini veya akustik
  davranışı ölçmez. Flash yazımının sistem genelindeki duraklaması ve OLED/BLE
  yükündeki örnek süreleri gerçek kartta kaydedilmelidir.

## Ayrı geliştirme ve ölçüm gerektiren işler

Pil için ADC pini, voltaj bölücü ve hücre bilgileri henüz verilmediği için
ölçüm uydurulmadı. Dahili kalıcı uçuş dosyası/IGC, termik merkezi ve rüzgâr
tahmini, doğrulanmış ses profilleri ve OTA bu düzeltme paketine eklenmedi.
Mevcut kalıcı uçuş kaydı telefondadır; cihazın trendi RAM/PSRAM'da uçucudur.
Bu ürün özellikleri ilk rapordaki sonraki aşamalardır. Sıcaklık modeli ve
filtre performansı için gerçek uçuş/tezgâh kayıtları gereklidir.

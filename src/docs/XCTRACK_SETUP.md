# XCTrack bağlantısı — firmware 2.1.0

11 Eylül 2026. Vario-BLE artık XCTrack'in desteklediği Nordic UART BLE servisi
üzerinden LK8EX1 basınç/dikey hız ve RMC/GGA cihaz GPS verisi yayınlar.
Servis ve cümle türleri [XCTrack'in harici cihaz belgesine](https://xctrack.org/External_Devices.html)
göre seçildi. Mevcut Android Vario uygulaması aynı firmware ile çalışır;
cihazda ayrıca XCTrack modu seçmek gerekmez.

## Telefonda bağlantı

1. **Firmware 2.1.0'ı cihaza yükle.** Bu çalışma sırasında derleme yapıldı,
   fiziksel cihaza yükleme yapılmadı. PlatformIO projesindeki çıktı:
   `.pio/build/esp32-s3-devkitc-1/firmware.bin`.
2. Bizim Vario uygulamasında **Bağlantıyı kes** seçeneğini kullan. Uygulamayı
   yalnızca arka plana atmak yeterli olmayabilir: otomatik bağlantı yeniden
   devreye girebilir. Cihaz aynı anda tek uygulama bağlantısı için tasarlandı.
3. XCTrack'te **Tercihler → Bağlantı ve sensörler → Ekle → Bluetooth sensörü**
   yolundan **Vario-BLE** cihazını seç. Sürüme/dile göre bu bölüm
   **Preferences → Connection & sensors → External sensor → Bluetooth sensor**
   olarak da görünebilir. Telefonun klasik seri port eşleştirmesini kullanma.
4. Eklenen sensörün özelliklerinden **barometreyi** etkinleştir ve harici
   sensörü öncelikli kaynak yap. Cihaz GPS'ini de kullanacaksan GPS kaynağını
   aynı cihazdan seç. Telefon GPS'ini tercih edersen GPS kaynağı telefonda,
   barometre kaynağı Vario-BLE'de kalabilir.
5. Bağlantı göstergesini ve gelen sensör verisini kontrol et. XCTrack'te yeşil
   bağlantı göstergesi bağlantıyı, kalın veriler alımı, yeşil kalın veriler
   kullanılan kaynağı gösterir. Barometre kalibrasyonu ekranında basınç ve veri
   hızı izlenebilir. Kaynak seçimi ve gösterge açıklamaları
   [XCTrack bağlantı ve sensör ayarları rehberinde](https://www.fly-air3.com/en/support/air3-xctrack-manual/xctrack-manual/preferences4/)
   anlatılır.

XCTrack'in salt okunur verisi PIN istemez. Bizim uygulamadan cihaz ayarı
değiştirmek için mevcut OLED PIN'iyle güvenli eşleşme devam eder. XCTrack'e
geçiş için kayıtlı eşleşmeyi silmek gerekmez.

Cihaz listede görünmüyorsa önce diğer uygulamanın bağlantısını kesip taramayı
yenile. Firmware güncellemesinden sonra eski servis özellikleri önbellekte
kalmışsa bağlantıyı kapatıp yeniden kur; gerekirse telefon Bluetooth'unu
kapatıp aç ve tekrar tara.

## Hangi veri nereden geliyor?

| Veri | Kaynak ve davranış |
|---|---|
| Basınç | Cihaz barometresi, Pa; hedef hız 5 Hz |
| Dikey hız | Cihazın füzyon çıkışı; LK8EX1 içinde cm/s, hedef hız 5 Hz |
| GPS | Cihazın alıcısından gelen RMC/GGA, hedef hız 1 Hz |
| İrtifa referansı | XCTrack kendi basınç/irtifa kalibrasyonunu uygular |
| Sıcaklık / cihaz pili | Ölçüm bağlanmadığı için protokolde mevcut değil olarak gönderilir |

Bizim uygulamada bilinen irtifa girilmesi XCTrack'in QNH ayarını değiştirmez.
XCTrack irtifasını kendi kalibrasyon ekranından ayarla. XCTrack'in seçili sensör
ve filtre ayarları ekrandaki/işitilen vario tepkisini etkileyebilir; cihazın
buzzer döngüsü bağımsız çalışmaya devam eder.

Cihaz GPS'i ancak konum güncelse (3 saniyeden yeni), en az 4 uydu varsa ve HDOP
0–5 aralığında geçerliyse aktarılır. RMC/GGA cümleleri checksum doğrulamasından
sonra orijinal zaman, tarih, fix ve yükseklik alanlarıyla gönderilir. Fix yoksa
veya veri eskimişse RMC `V`, GGA kalite `0` yayınlanır; eski konum geçerli gibi
tekrarlanmaz. Telefon GPS'ine geçiş kararı XCTrack'in kaynak ayarlarına bağlıdır.

## Protokol ve uygulama notları

- Nordic UART servisi: `6e400001-b5a3-f393-e0a9-e50e24dcca9e`.
  TX: `6e400003-b5a3-f393-e0a9-e50e24dcca9e`.
- XCTrack TX **notification** aboneliği kullanır. Bizim uygulama aynı TX'te
  **indication** seçerek JSON komut yanıtlarını alır. İki abonelik birden
  seçilirse JSON önceliklidir; veri akışları karıştırılmaz.
- Ana ikili telemetri/GPS/durum karakteristikleri ve güvenli komut RX'i
  korunur. Android üretim kodunda değişiklik gerekmemiştir.
- LK8EX1 basıncı Pa, dikey hızı işaretli cm/s taşır. İrtifa alanı `99999`
  (kullanılmıyor), sıcaklık `99`, pil `999` olarak gönderilir. Basınç geçersizse
  `999999`, vario hazır değilse `9999` kullanılır. Cümleler `$...*HH\r\n`
  biçimindedir; checksum son virgül dahil hesaplanır. Birimler ve boş değerler
  [LK8EX1 protokol tanımına](https://github.com/LK8000/LK8000/blob/master/Docs/LK8EX1.txt)
  uygundur.
- Eski/bozuk barometre verisi 500 ms sonrasında kullanılabilir ölçüm olarak
  gönderilmez. GPS için yalnızca checksum'u doğru GP/GN RMC ve GGA kabul edilir.
- Akış BLE MTU'suna göre bölünür; varsayılan MTU 23'te 20 baytlık parçalar da
  desteklenir. Sabit 256 bayt tampon kullanılır. Yığına teslim edilemeyen parça
  yeniden denenir; 250 ms'den eski yarım akış bırakılıp yeni satırla başlanır.
  Tıkanıklık sırasında gerçekleşen veri hızı hedefin altında kalabilir.
- Yeni üretim modülü: `xctrack_protocol.cpp/.h`; GPS UART'ından doğrulanmış
  cümleleri alan görev ve BLE görevi bu modüle bağlandı. Ölçüm görevine BLE
  aktarımı eklenmedi. `status` yanıtı `xcTrackCompatible:true` bildirir.

## Doğrulama

- **136 C++ kontrolü başarılı:** üretim modülleri üzerinden birimler,
  checksum, GPS kaybı, bozuk/uzun satırlar, MTU parçalama, aktarım tıkanıklığı,
  abonelik seçimi ve sayaç taşması dahil.
- **10 Android BLE bağlantı testi başarılı:** yeni çift özellikli TX'te
  uygulamanın JSON indication seçmesi ve otomatik bağlantı davranışları dahil.
  Android'e özgü BLE çağrısı masaüstü testinde taklit edildi.
- Flutter statik analiz: **sorun yok**. APK üretilmedi.
- PlatformIO ESP32-S3 derlemesi: **başarılı**. Flash 826.114 / 1.310.720 bayt
  (%63,0); statik RAM 42.848 / 327.680 bayt (%13,1).

Gerçek telefon + XCTrack + cihaz bağlantısı henüz denenmedi. Cihaza yükledikten
sonra yerde basınç alımını, GPS fix kazanma/kaybetmeyi ve iki uygulama arasında
bağlantı geçişini kontrol etmek kalan donanım doğrulamasıdır.

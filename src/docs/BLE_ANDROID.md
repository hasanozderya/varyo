# Android uygulaması için Vario BLE protokolü — firmware 2.1.0

Ana telemetri paketinin sürümü ve uzunluğu değişmedi: v1, 20 bayt. GPS ve cihaz
durumu ayrı, isteğe bağlı karakteristiklerdir; eski firmware ile bağlantı korunur.

Firmware `Vario-BLE` adıyla BLE çevre birimi açar. Wi-Fi desteği firmware'den
tamamen kaldırılmıştır. Telefonun Bluetooth ayarlarındaki seri bağlantı yerine uygulama
içinden BLE taraması ve GATT bağlantısı kullanılır. ESP32-S3 yalnız BLE destekler
([Espressif](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-guides/ble/overview.html)).

## Bağlantı

| İşlev | UUID son bölümü* | Özellik |
|---|---|---|
| Servis | 0001 | Servis UUID'siyle tara |
| RX: telefondan komut | 0002 | Write with response, şifreli ve kimliği doğrulanmış |
| TX: komut yanıtı / XCTrack | 0003 | Indicate: JSON; Notify: NMEA |
| Canlı telemetri | 0004 | Read + Notify |
| Cihaz GPS | 0005 | Read + Notify, 2 Hz |
| Cihaz durumu | 0006 | Read + Notify, 2 Hz |

*Tam UUID'ler: `6e400001-b5a3-f393-e0a9-e50e24dcca9e`; diğerleri için ilk
bölümün son dört hanesini tabloda gösterilen değerle değiştir.

1. Servis UUID'siyle tara, `connectGatt(..., false, ..., TRANSPORT_LE)` ile bağlan.
2. Servis keşfini tamamla. Varsa durum karakteristiğini oku. Güvenli komut biti
   etkinse Android bonding tamamlanmalı; ilk eşleşmede OLED'deki altı haneli PIN
   Android eşleşme penceresine girilir. Kayıtlı eşleşme sonraki bağlantıda kullanılır.
   Ardından TX için yerel `setCharacteristicNotification(tx, true)`
   ve 0x2902 CCCD'ye `ENABLE_INDICATION_VALUE` (`02 00`) yazımını tamamla.
3. Telemetri için aynı şekilde `ENABLE_NOTIFICATION_VALUE` (`01 00`) yaz.
4. Her GATT işleminin callback'ini bekle; descriptor/MTU/characteristic yazımlarını
   paralel gönderme. RX için `WRITE_TYPE_DEFAULT` kullan.
5. `getSettings` ve `status` komutlarını sırayla gönder. Firmware tek uygulama
   bağlantısı için tasarlanmıştır. Kopunca yeniden reklam verir; üç dakika sınırı yoktur.

TX'nin JSON yanıtları için bildirim türü **indication** olmalıdır. Firmware 2.1.0
aynı karakteristikte **notify** aboneliğini XCTrack NMEA akışı için kullanır.
CCCD `02 00` JSON, `01 00` NMEA seçer; iki bit birlikte (`03 00`) yazılırsa JSON
önceliklidir. Abonelik türü değişince yarım paketler ve eski modun komutları
bırakılır. JSON ile NMEA aynı akışa karıştırılmaz. Flutter uygulamamız TX için
`setNotifyValue(true, forceIndications: tx.properties.indicate)` kullanır;
bu Android davranışı korunmalıdır. [XCTrack bağlantı rehberi](docs/XCTRACK_SETUP.md).

ATT onayını Android Bluetooth yığını verir;
uygulamanın ayrıca yanıt ACK komutu göndermesi gerekmez.

Android 12+ için manifest ve çalışma zamanında `BLUETOOTH_SCAN` ile
`BLUETOOTH_CONNECT` gerekir. Eski Android sürümlerinde tarama konum izni de
isteyebilir. [Android izinleri](https://developer.android.com/develop/connectivity/bluetooth/bt-permissions)
ve [GATT API](https://developer.android.com/reference/android/bluetooth/BluetoothGatt).

Firmware 2.0.0 ve sonrası komut yazımında BLE bonding, MITM koruması ve Secure Connections
kullanır. Ana telemetri/GPS/durum okunabilir; bu kanalların gizliliği vaat edilmez.
Android uygulaması `C:/Users/ozder/StudioProjects/vario_ble` projesindedir.
Otomatik bağlantı yeni eşleşme penceresi açmaz; eşleşme kaydı yoksa açıklama
gösterip durur. İlk geçişte uygulamadan bir kez elle güvenli bağlantı kurulur.

## Komut ve yanıt çerçevesi

RX'e UTF-8, tek satır JSON ve sonunda gerçek LF (`0x0A`) gönder. JSON dizgesindeki
iki karakterli `\n` ile karıştırma. Komut LF hariç en fazla **511 bayt**. BLE yazımı
komutun tamamını içermek zorunda değil: 20 baytlık parçalar her MTU ile çalışır.
Firmware parçaları LF gelene kadar biriktirir. CRLF de kabul edilir. Taşmış veya
NUL içeren satır tamamen reddedilir; kalan kısmı yeni komut olarak çalıştırılmaz.

Uygulama aynı anda **bir komut** göndersin. GATT write onayı yalnız alımı doğrular;
komutun sonucu TX'teki aynı `id` değerli JSON yanıtıdır. `id`: 1..2147483647.
Yanıt da LF ile biter; indication boyutu `min(MTU−3, 182)` bayttır (MTU 23 ise 20). UTF-8
çözümlemeyi/JSON ayrıştırmayı tam satır biriktikten sonra yap. Kopuşta parçaları sil.

Başarı: `{"id":1,"ok":true,"result":{...}}`

Hata: `{"id":1,"ok":false,"error":"invalid_settings"}`

Ayrıştırılamayan/taşan komutlarda `id:null` olabilir. Komut kuyruğu sınırlıdır.
Yanıt için örneğin 60 saniye zaman aşımı kullan; özellikle log ve trend yanıtları
çok sayıda indication içerir. Zaman aşımı/kopuş sonrasında **değişiklik komutlarını
körü körüne tekrarlama**: önce `getSettings` / `status` ile sonucu sorgula.
`id` sadece eşleştirme içindir; firmware idempotency/tekrar engelleme uygulamaz.

## Komutlar

Aşağıdaki her satırın sonuna LF ekle. Sayılarda ondalık ayırıcı noktadır.

```json
{"id":1,"cmd":"status"}
{"id":2,"cmd":"getSettings"}
{"id":3,"cmd":"setSettings","params":{"fusion":{"alpha":0.98,"kfA":2.5,"kfAB":0.0001,"kfBaro":0.35,"qnh":1013.25,"kAdapt":1},"audio":{"deadband":0.1,"climbMax":5,"sinkAlarm":-2,"toneMin":700,"toneMax":2200,"sinkTone":450,"strongSink":-5,"hysteresis":0.05,"volume":80,"periodMin":120,"periodMax":700,"weakLift":false,"altitudeAlert":true,"altitudeLimit":2500,"altitudeMargin":100}}}
{"id":5,"cmd":"calibrateAltitude","params":{"altitude":975}}
{"id":6,"cmd":"calibrateImu","params":{"ground":true}}
{"id":7,"cmd":"getLog"}
{"id":8,"cmd":"getTrend","params":{"since":0,"limit":30}}
{"id":10,"cmd":"startFlight"}
{"id":11,"cmd":"stopFlight"}
```

- `setSettings`: `fusion` ve `audio` bölümlerini tek komutta gönderir. Eksik
  alanlar mevcut değeri korur; iki bölüm birlikte doğrulanıp atomik olarak uygulanır. Bilinmeyen
  alanlar, yinelenen anahtarlar ve string olarak gönderilen sayılar reddedilir.
  `getSettings` sonucu `fusion` ve `audio` nesnelerini içerir. Mevcut NVS kayıt
  mekanizması kullanılır; yeniden açılışta bu ayarlar yüklenir. Ayarlar yalnızca
  `settings` adlı, sağlama toplamlı tek kayıt olarak saklanır. Başarısız yazma
  `settings_save_failed` döndürür; çalışan RAM ayarları değiştirilmez.
- `alpha`: 0 dahil, 1 hariç; `kfA`: (0,1000]; `kfAB`: [0,10];
  `kfBaro`: (0,1000]; `qnh`: [800,1100] hPa; `kAdapt`: [0,10].
- `deadband` >=0; `climbMax` > deadband ve <=20; `sinkAlarm` [-20,0);
  frekanslar tam sayı [150,5000] Hz; `toneMax` >= `toneMin`.
  `sinkAlarm` normal çöküş sesinin gerçek başlangıç eşiğidir. `strongSink` ayrı
  güçlü çöküş bildirimi: 0 kapalı, aksi halde [-25,sinkAlarm) m/s.
  `hysteresis`: [0,0.3] m/s; `volume`: tamsayı [0,100] (0 tüm sesi kapatır);
  `periodMin`/`periodMax`: tamsayı [60,2000] ms, min ≤ max; `weakLift`: boolean.
  `altitudeAlert`: boolean; `altitudeLimit`: [-500,9000] m;
  `altitudeMargin`: [20,1000] m. Uyarı, yalnız uçuş durumunda
  `altitudeLimit-altitudeMargin` seviyesine çıkıldığında bir kez çalar ve eşikten
  50 m aşağı inilmeden yeniden kurulmaz.
  Eski firmware bu yeni alanları kabul etmez; `getSettings.audio.strongSink`
  varlığıyla destek denetlenmelidir. Ses yüzdesi elektriksel PWM seviyesidir.
- `calibrateAltitude`: metre cinsinden [-500,9000], güncel barometre ölçümü
  gerekir. QNH'yi hesaplar; ölçüm görevi filtreleri yeni referansa dönüştürür,
  dikey hızı sıfırlamaz. İlk yeni referanslı telemetride bit6 değişir; uygulama
  o örnekte irtifa geçmişini ve bağıl irtifa referansını yeniden başlatmalıdır.
- `calibrateImu`: yalnız yerde hareketsizken başlat. Yanıttaki `accepted:true`
  işlemin bittiği anlamına gelmez. `status.imuCalibrationStatus` izle:
  0=boşta, 1=örnek topluyor, 2=kaydedildi, 3=reddedildi, 4=kayıt hatası,
  5=arka planda kaydediliyor. `ground:true` tek başına yeterli değildir;
  cihazda 8 saniyelik sakinlik ve uçuş durumu kontrolü vardır.
  Açılışta yeniden kalibrasyon gerekmez. Başlangıçta kayıt varsa `imuCalibrated`
  true olabilir ve durum 0'dır.
- `status`: irtifa, vario, basınç, ham barometrik irtifa, düşey ivme, açı,
  Kalman bias ve sensör/kalibrasyon durumları. Birimler: m, m/s, Pa, m/s², derece.
  Ayrıca firmware/build, reset nedeni, bellek, örnek sayıları/yaşları, gecikme
  sayaçları, uçuş durumu/süresi, GPS kalite/geçerlilik/UTC bilgileri içerir.
  Firmware 2.1.0, `xcTrackCompatible:true` alanını da döndürür.
- `startFlight` / `stopFlight`: cihazın uçuş durumunu değiştirir. `accepted`
  komutun kuyruğa alındığını belirtir; durum kanalından sonucu izle.
  Sensörleri sıfırlamaz, sesi kapatmaz, kalıcı uçuş dosyası oluşturmaz.
- `getLog`: `result.text` içinde dönen son log metni.
- `getTrend`: 1 Hz geçmiş; satır `[timeMs, altitudeM, varioMps, pressurePa]`.
  `limit`: 1..100, varsayılan30. `hasMore:true` ise sonraki komutta
  `since=nextSince` kullan. Bellekteki en eski erişilebilir kayıttan ilerler.
  Yeniden açılış veya QNH/irtifa kalibrasyonu sonrasında uygulama kendi grafiğini
  silip `since=0` ile başlasın. `timeMs` 32 bit sayaçtır; farkları modulo 2^32 al.

## Canlı telemetri — 20 bayt

Yaklaşık **5 Hz** notify. Komut yanıtlarının onay beklemesi bu hızı geçici olarak
düşürebilir; sensör ve buzzer döngüsü bağımsız çalışır. Tüm sayılar little endian.

| Ofset | Tür | İçerik |
|---|---|---|
| 0 | uint8 | Protokol sürümü: 1 |
| 1 | uint8 | Bit0 kullanılabilir barometrik ölçüm, bit1 imuOk, bit2 imuCalibrated, bit3 imuFusionActive, bit4 IMU kalibrasyonu sürüyor, bit5 genişletilmiş firmware, bit6 irtifa referansı paritesi |
| 2 | uint16 | Paket sıra numarası; 65535'ten sonra 0 |
| 4 | uint32 | Ölçümün üretildiği açılıştan beri ms; 2^32'de taşar |
| 8 | float32 | İrtifa, m |
| 12 | float32 | Vario, m/s |
| 16 | float32 | Basınç, Pa |

Bağlantı kesilince ekrandaki telemetriyi eski olarak işaretle. `baroOk=false`
iken irtifa/varioyu geçerli ölçüm gibi gösterme. İlk örneklerin toplanması sırasında
bit0 kapalıdır. Tekrarlanan ölçüm zamanı yeni veri sayılmaz. Ölçüm görevi durmuşsa
pakete yeni bir saat yazılmaz. Bit6 yalnız bit5 etkinse yorumlanmalıdır.

## GPS — 20 bayt, v1

Tüm çok baytlı sayılar little endian. GPS yaşı 3 saniyeyi geçtiğinde fix/motion
bayrakları söner. Uydu ≥4 ve 0 < HDOP ≤5 gerekir; HDOP metre doğruluğu değildir.

| Ofset | Tür | İçerik |
|---|---|---|
| 0 | uint8 | Sürüm 1 |
| 1 | uint8 | Bit0 fix, bit1 hız/yön geçerli; bit4–7 fix yaşı (200 ms adım, yukarı yuvarlanır, en fazla 15) |
| 2 | uint8 | Uydu sayısı |
| 3 | uint8 | HDOP ×10; 255 doygun/bilinmiyor |
| 4 | int32 | Enlem ×10⁷ |
| 8 | int32 | Boylam ×10⁷ |
| 12 | uint32 | FIX örneğinin cihaz uptime ms değeri |
| 16 | uint16 | Yer hızı, cm/s; yalnız bit1 ile geçerli |
| 18 | uint16 | Hareket yönü, 0.01 derece; yalnız bit1 ile geçerli |

Aynı FIX zamanı yeniden gelirse konumun tazeliğini uzatma. Android uygulaması
3 saniyeden eski cihaz konumundan güncel telefon konumuna geçer ve rotada yeni
segment açar. GPS irtifası ve gerçek alıcı UTC değeri `status.gps` içindedir;
bu küçük paket UTC taşımaz. Android rota zamanı telefon saati eksi paket yaşıdır.

## Cihaz durumu — 20 bayt, v1

| Ofset | Tür | İçerik |
|---|---|---|
| 0 | uint8 | Sürüm 1 |
| 1 | uint8 | Bit0 güvenli komut zorunlu, bit1 yer kalibrasyonuna hazır, bit2 ölçüm hazır |
| 2 | uint8 | 0 füzyon, 1 eski ölçüm, 2 baro arızası, 3 hazırlanıyor, 4 IMU arızası, 5 kalibrasyonsuz, 6 barometrik yedek |
| 3 | uint8 | Uçuş: 0 belirsiz, 1 yerde, 2 uçuşta, 3 tamamlandı |
| 4 | uint32 | Ölçüm uptime ms |
| 8 | uint16 | Barometre yaşı ms, en fazla 65535 |
| 10 | uint16 | IMU yaşı ms, en fazla 65535 |
| 12 | uint32 | İrtifa referans revizyonu |
| 16 | uint16 | Ayrılmış, 0; pil yüzdesi değildir |
| 18 | uint8 | Firmware ana sürümü: 2 |
| 19 | uint8 | Firmware alt sürümü: 1 |

Durum bildirimleri 2 saniyedir alınmıyorsa yer kalibrasyonu iznini geçersiz say.
Bit6 ana telemetriyle zaman eşleşmesi içindir; tam revizyon burada teşhis edilir.

`../examples/android/VarioBleCodec.kt`: UUID'ler, paket çözümleme, komut parçalama ve JSON yanıt
biriktirme yardımcısı. Bildirim callback'inde karakteristik UUID'sine göre
`telemetry(value)` veya `responses(value)` çağır; kopuşta `reset()` çağır.
Android 13+ callback'inde verilen `byte[] value` kopyasını kullan; eski sürümde
`characteristic.value.copyOf()` ile callback içinde kopyala.

Firmware yalnız BLE uygulama bağlantısını kullanır; Wi-Fi erişim noktası veya
web ayar arayüzü başlatılmaz.

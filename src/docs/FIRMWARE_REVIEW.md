# Varyo donanım yazılımı incelemesi

Tarih: 9 Eylül 2026

**11 Eylül 2026:** Firmware 2.1.0 XCTrack desteği ve bağlantı adımları
[XCTRACK_SETUP.md](XCTRACK_SETUP.md) dosyasındadır.

**10 Eylül 2026 uygulama durumu:** Aşağıdaki bulgular inceleme öncesindeki kodu
anlatır. Öncelikli firmware düzeltmeleri ve bunların Android uyarlaması uygulandı.
Güncel değişiklikler, test sonuçları ve sonraki işler
[FIRMWARE_UPDATE.md](FIRMWARE_UPDATE.md) dosyasındadır. Bu belgedeki eski satır
numaraları uygulama sonrasında kaymış olabilir.

Kapsam: ESP32-S3 projesinin sensör sürücüleri, füzyon, ses, OLED/GPS, BLE, ayar saklama ve görev düzeni. Android uygulamasındaki mevcut uçuş özellikleri bu raporda yeniden yapılacak işler olarak sayılmadı.

Bu bölüm 9 Eylül tarihli kaynak kod incelemesidir. Aşağıdaki masaüstü hesapları
gerçek kart ölçümü değildir. O inceleme sırasında firmware değiştirilmemişti;
10 Eylül uygulaması ayrı değişiklik belgesinde kayıtlıdır. Cihaza yükleme ve APK
üretimi yapılmadı.

## Değerlendirme

ESP32-S3, MS5607, MPU6050 ve kodda NEO-M8N olarak belirtilen GPS iyi bir geliştirme temeli oluşturuyor. Profesyonel seviyeye yaklaşmak için öncelik; hareket sırasında doğru ölçüm, tutarlı gecikme, anlaşılır ses ve arızayı görünür kılmak olmalı. İşlemci gücü tek başına sensör doğruluğunu, pil ömrünü veya basınç portunun rüzgârdan etkilenmesini çözmez.

Kodda 100 Hz ölçüm görevi, barometrik yedek hesap, kayıtlı IMU kalibrasyonu, sıcaklık kontrolü ve telefon bağlantısından bağımsız ses zaten var. Bunlar korunmalı.

## Önce düzeltilmesi gerekenler

### 1. Dönüşlerde üç eksenli yönelim hesabı — yüksek öncelik

**Bulgu:** MPU6050'nin `gz` ölçümü okunuyor fakat yönelim filtresine aktarılmıyor. Filtre doğrudan `pitch += gy * dt`, `roll += gx * dt` kullanıyor. Gövde eksenindeki jiroskop hızları, cihaz yatıkken Euler açı hızlarıyla aynı değildir. Ayrıca kaçırılan IMU okumalarından sonra gerçek örnek aralığı yerine görev döngüsünün `dt` değeri kullanılıyor.

**Etkisi:** Yatık durumda dönüş, gerçekte olmayan eğim değişimine ve yerçekimi çıkarılırken hatalı dikey ivmeye neden olabilir. Barometre hatayı zamanla düzeltebilir; yine de hızlı ses tepkisine yanlış bir ivme girişi taşınmış olur.

**Öneri:** Üç jiroskop eksenini kullanan quaternion tabanlı yönelim hesabı, gerçek IMU örnek zamanları ve hareket sırasında ivmeölçere duyulan güveni denetleyen düzeltme. Quaternion kullanmak tek başına dönüş ivmesini yerçekiminden ayırmaz; bu durum ayrıca sınanmalı.

**Kanıt:** [Filtre denklemleri](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/sensor_fusion.cpp:38), [IMU çağrısı](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/vario_task.cpp:203).

### 2. OLED ile sensörlerin aynı I²C hattını paylaşması — yüksek öncelik

**Bulgu:** 128×64 ekranın tam tamponu, I²C kilidi bırakılmadan gönderiliyor. Bu sırada sensörlerin sıfır beklemeli kilit denemeleri başarısız oluyor ve okumalar atlanıyor.

**Etkisi:** 400 kHz hatta yalnızca 1024 bayt görüntü ve ACK bitleri en az **23,04 ms** sürer; komutlar ve işlem yükü bu süreye eklenir. 10 ms görev aralığına göre bu kayda değer bir örnek boşluğudur. Kurulu U8g2 SSD1306 sürücüsünün varsayılanı da 400 kHz olarak doğrulandı. Gerçek süre kartta ölçülmeli.

**Öneri:** Mevcut bağlantıyla önce küçük parçalara bölünmüş ekran aktarımı ve aralarda sensöre hat erişimi. Donanım bağlantısı değiştirilecekse OLED için ikinci I²C hattı veya uygun ekranla SPI. IMU için veri hazır/FIFO kullanımı ayrıca değerlendirilmeli. `Wire` işlemlerinin zaman aşımı açıkça sınırlandırılmalı; sıfır beklemeli mutex, I²C işleminin tamamını asenkron yapmıyor.

**Kanıt:** [Ekran aktarımı](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/display.cpp:55), [IMU okuma](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/mpu6050.cpp:37), [I²C başlangıcı](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/i2c_bus.cpp:8).

### 3. Eski veya henüz hazır olmayan veri, geçerli sıfır gibi görünebiliyor — yüksek öncelik

**Bulgu:** `VarioState` içinde ölçüm zamanı, ölçüm sıra numarası ve `outputReady` yok. BLE, eldeki son durumu yeni paket sıra numarası ve güncel `millis()` ile gönderiyor. Ölçüm görevi ilerlemeyi bırakır ama BLE devam ederse eski ölçümün paketleri yeni görünür. Ölçüm görevi çalışırken barometre zaman aşımı denetleniyor; bu mevcut koruma, görevin kendisinin durmasını kapsamıyor.

Başlangıçta, QNH değişiminden sonra veya veri yetersizken dikey hız `0.0` yapılıyor. OLED bunu normal sayı olarak gösteriyor. Barometre haberleşmesinin sağlıklı olması, dikey hız hesabının hazır olduğu anlamına gelmiyor.

**Öneri:** Ölçüm zamanı/sayacı, barometre ve IMU örnek yaşları, çıktı geçerliliği, çalışma modu ve arıza nedeni eklenmeli. BLE ve OLED bunları kendi saatleriyle denetlemeli. Geçersiz hız `--` ve kısa açıklamayla gösterilmeli. Sensör arızası için normal varyo sesinden ayırt edilen, sınırlı bir bildirim olmalı. VarioTask ilerleme denetimi ve görev oluşturma sonuçlarının kontrolü de eklenmeli.

**Kanıt:** [Paylaşılan durum](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/shared_state.h:9), [BLE paket zamanı](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/ble_task.cpp:209), [Geçerlilik ve yayın](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/vario_task.cpp:273), [Görev oluşturma](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/main.cpp:46).

### 4. Başlangıçta başarısız olan sensör için tekrar başlatma yok — yüksek öncelik

**Bulgu:** `baro.begin()` ve `imu.begin()` bir kez çağrılıyor. Başlangıç sonucu başarısızsa ilgili sensörün normal okuma yolu kalıcı olarak atlanıyor. Başarıyla başlamış bir sensörde geçici okuma hatasından dönüş mümkün; ancak sensörün yeniden yapılandırılmasını gerektiren durumlar için bir kurtarma akışı görünmüyor.

**Öneri:** Sağlıklı / geçici hata / yeniden deneme / kalıcı arıza durumları; sınırlı sıklıkla yeniden tanıma ve yapılandırma; I²C hat kurtarma. IMU kimliği ve ayar yazımlarının geri okunması da doğrulanmalı. Kurtarma sırasında kalan sağlıklı ölçüm yolu çalışmalı ve yeniden otomatik yer kalibrasyonu yapılmamalı.

**Kanıt:** [Tek başlangıç denemesi](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/vario_task.cpp:36), [Barometre koşulu](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/vario_task.cpp:149), [IMU koşulu](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/vario_task.cpp:201).

### 5. Çöküş sesi eşiği ayarın adıyla uyuşmuyor — yüksek öncelik

**Bulgu:** Varsayılan `sinkAlarm = -2.0 m/s`, fakat sürekli çöküş sesi `climbRate < -climbDeadband` koşulunda başlıyor. Varsayılan deadband 0,1 olduğundan örneğin **−0,5 m/s'de ses var**. `sinkAlarm` yalnızca frekans eğrisini ölçekliyor. Ayrı güçlü çöküş alarmı veya eşik çevresinde aç/kapa titremesini engelleyen histerezis yok.

**Öneri:** Tırmanış başlangıcı, normal çöküş sesi başlangıcı ve güçlü çöküş alarmı ayrı parametreler olmalı. Bunlara histerezis, ayarlanabilir ses eğrisi/ritmi, isteğe bağlı zayıf kaldırıcı bildirimi ve önizleme eklenmeli. Ses seviyesi kontrolü mevcut buzzer sürme devresine göre tasarlanmalı; PWM görev oranı her devrede düzgün bir ses seviyesi ayarı sağlamaz.

**Kanıt:** [Ses koşulu](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/buzzer.cpp:35), [Varsayılan eşikler](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/config.h:101).

Flymaster'ın kılavuzunda çöküş sesi eşiği, güçlü çöküş alarmı, ses seviyesi ve ritim ayrı işlevler olarak tanımlanıyor. Buradaki öneri bu işlev ayrımını temel alıyor; üreticinin varsayılanlarını aynen kopyalamak gerekmiyor. [Flymaster VARIO SD kılavuzu, 4.4–4.5](https://dnl.flymaster.net/Flymaster%20VARIO%20SD%20manual%20EN%20v2.pdf)

### 6. İrtifa kalibrasyonu dikey hız hesabını yeniden başlatıyor

**Bulgu:** QNH değişince Kalman ve barometrik regresyon sıfırlanıyor. Regresyon yeniden hazır oluncaya kadar hız sıfır olarak çıkıyor. Geçmiş trend de siliniyor.

**Öneri:** Mutlak irtifa referansı değişikliği ile hareket tahminini ayırmak. Yeni referansa geçerken dikey hız sürekliliğini koruyacak şekilde estimator ve geçmiş örnekleri dönüştürmek; kalibrasyonu kayıt üzerinde olay olarak işaretlemek. QNH dönüşümü tam olarak sabit irtifa farkı olmadığından bu işlem yalnızca ekrana sabit sayı eklemekle sınırlanmamalı.

**Kanıt:** [QNH sıfırlaması](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/vario_task.cpp:121), [Kalibrasyon komutu](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/ble_commands.cpp:161).

### 7. Kalibrasyon ve ayar değişiklikleri cihaz tarafında korunmalı

**Bulgu:** BLE yazma karakteristiği için eşleşme/şifreleme veya istemci yetkilendirme şartı kurulmamış. IMU kalibrasyonu için gönderilen `ground:true` yeterli; firmware kendi uçuş durumunu denetlemiyor. Kalibrasyonun düşük varyans kontrolü sabit açısal hızı bias sanabilir; kaynak kod da bu sınırlamayı belirtiyor.

**Öneri:** İlk kurulumda tanıtılan sahibin ayar değiştirebilmesi, sonrasında kayıtlı telefona otomatik bağlantının korunması. Yeni sahip tanıtımı için cihaz üzerinde veya açıkça başlatılan kurulum akışı. Firmware içinde uçuş durumu ve yer kalibrasyonu kilidi; kararsız başlangıç durumunda otomatik olarak yerde kabul etmeme. Sayısal filtre ayarlarını uçuşta korumak; ses ve irtifa referansı gibi kullanıcının uçuşta ihtiyaç duyduğu ayarları uygun doğrulamayla ayrı ele almak.

**Kanıt:** [BLE karakteristikleri](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/ble_task.cpp:106), [Yer kalibrasyonu isteği](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/ble_commands.cpp:176), [Kalibrasyon sınırlaması](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/mpu6050.cpp:128).

## İkinci aşamadaki teknik iyileştirmeler

| Konu | Mevcut durum ve yapılacak iş |
| --- | --- |
| Barometre örnekleme | MS5607 durum makinesi okuma tamamlanınca `Idle` durumuna dönüyor; yeni dönüşüm sonraki görev turunda başlıyor. Kusursuz 100 Hz görev zamanlaması varsayımıyla sıcaklık okumaları dahil yaklaşık **47,6 basınç örneği/s** elde edilir; OLED boşlukları buna dahil değildir. Yeni dönüşümü okuma sonrası başlatarak boş aralık azaltılabilir. Hız/gürültü/enerji ölçülerek seçim yapılmalı. [Kod](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/ms5607.cpp:153) |
| Filtre gecikmesi | Basınçta 0,25 s alçak geçiren filtre, barometrik yedek yolda 900 ms regresyon penceresi var. IMU füzyonu ayrı çalışıyor; bu nedenle tüm modlara tek bir toplam gecikme sayısı vermek doğru olmaz. Basamak/rampa tekrarlarıyla gürültü ve tepki süresi ölçülmeli. Daha fazla filtreyi varsayılan çözüm olarak eklememeli. [Ayarlar](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/config.h:107) |
| Basınç aykırı değerleri | Sürücüde ham örnek kontrolü var. Kalman tarafında 50 m yenilik farkında sert sıfırlama kullanılıyor. Daha küçük basınç darbeleri için tahmin belirsizliğine göre kabul/red ve bozulmuş ölçüm modu değerlendirilmeli; gerçek hızlı tırmanış reddedilmemeli. [Kod](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/sensor_fusion.cpp:218) |
| Cihaza özel kalibrasyon | Üç eksen ölçek katsayısı derleme sabiti. Bunlar önceki ölçümden geliyor; rastgele kaldırılmamalı. Çok konumlu ivmeölçer kalibrasyonu ve birden fazla sıcaklıkta gyro bias ölçümüyle cihaz başına sürümlü kalibrasyon verisi hazırlanmalı. Sıcaklık modeli doğrulanana kadar mevcut barometreye geçiş korunmalı. [Kod](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/config.h:69) |
| Kalıcı ayarlar | Ses/füzyon ayarları ayrı NVS anahtarlarına yazılıyor ve yazma sonuçları denetlenmiyor. Enerji kesilirse aynı ayar grubunda eski/yeni değerler karışabilir; BLE yine başarılı yanıt verebilir. Sürümlü, bütün olarak doğrulanan kayıt ve gerçek saklama sonucunu döndüren API kullanılmalı. IMU kalibrasyonunun mevcut `putBytes` sonucunu kontrol etmesi iyi örnek. [Kod](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/tunables.cpp:79) |
| Ölçüm görevinin işi | 1 Hz metin loglama 20 ms'ye kadar mutex bekleyebiliyor; kalibrasyon sonunda NVS yazımı ölçüm görevi içinde yapılıyor. Logları beklemeyen bir kuyrukla dışarı taşımak ve kalıcı yazımları zaman bütçesiyle yönetmek gerekir. [Log](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/debug_log.cpp:55), [Kalibrasyon kaydı](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/mpu6050.cpp:143) |
| BLE geçmiş aktarımı | `getTrend` en fazla 100 nokta döndürse de 1800 noktalık tüm geçmişi geçici belleğe kopyalayabiliyor. Yalnızca istenen aralık kopyalanmalı; yanıt parçaları anlaşılmış MTU'ya göre seçilmeli. Bağlantı aralığına bağlı aktarım süresi test edilmeli. [Kod](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/ble_commands.cpp:186) |
| Tekrarlanabilir derleme | Arduino-ESP32 doğrudan sabitlenmemiş Git adresinden alınıyor. BLE tarafında bu çekirdeğin davranışına bağlı doğrudan NimBLE çağrısı var. Çalıştığı doğrulanan platform/çekirdek/kütüphane sürümleri sabitlenmeli ve firmware sürüm/derleme kimliği yayınlanmalı. [PlatformIO](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/platformio.ini:6), [NimBLE](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/ble_task.cpp:169) |

Flash yazımını başka göreve taşımak tek başına zamanlama etkisini yok etmez. ESP32-S3'te flash, önbellek ve PSRAM eşzamanlılığı kullanılan çip/SDK seçeneklerine bağlıdır; uçuşta kayıt eklenirken bu ayarlar ve en uzun duraklamalar ölçülmeli. [Espressif flash eşzamanlılık açıklaması](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/spi_flash/spi_flash_concurrency.html)

## Profesyonel kullanım için eklenecek işlevler

| İşlev | Yerleşim ve kapsam |
| --- | --- |
| Cihazın GPS'ini kullanma | Şu an GPS yalnızca OLED'e aktarılıyor. Konum, yer hızı, iz açısı, UTC, uydu/kalite ve örnek yaşı ortak durum ve BLE'ye eklenmeli. `isValid()` yanında `age()` kontrolü gerekli; eski FIX kalmamalı. Android, cihaz ve telefon GPS'i arasında açık kaynak/kalite seçimi yapmalı. [Mevcut GPS yolu](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/gps_ui_task.cpp:34) |
| Bağımsız uçuş kaydı | Mevcut cihaz geçmişi RAM/PSRAM'de 30 dakika ve güç kesilince kayboluyor. Telefona ihtiyaç duymayan konum/irtifa/basınç kaydı, güvenli kapatma, doluluk yönetimi, uçuş kimliği ve devam ettirilebilir BLE indirme eklenmeli. Depolama bütçesine göre dahili flash veya harici kayıt ortamı seçilmeli. IGC biçiminde dosya üretmek tek başına yarışma onayı anlamına gelmez. [Mevcut tampon](C:/Users/ozder/OneDrive/Belgeler/PlatformIO/Projects/varyo/src/trend_buffer.h:12) |
| Uçuş durum makinesi | Başlangıç/belirsiz, yerde, uçuşta ve iniş durumları; GPS/baro/hareket kanıtıyla geçiş, elle düzeltme, havada açılışı destekleme. GPS yokluğu sesli varyoyu durdurmamalı. Uçuş durumu kayıt ve kritik kalibrasyonları yönetmeli. |
| Pil ölçümü | Cihaz pili için ADC ölçümü, yük altında filtreleme, düşük pil bildirimi ve güç kesilmeden kayıt sonlandırma. Kullanıcı ölçüm bağlantısı olduğunu belirtti; **ADC pini, direnç bölücü değerleri ve pil türü/kapasitesi henüz belli değil**. Voltajdan yüzde yaklaşık hesaplanabilir; kalan süreyi güvenilir göstermek için tüketim ölçümü gerekir. |
| Uçuş ses profilleri | Hızlı/denge/sakin gibi ölçülmüş profiller; tırmanış/çöküş ayrı eşikleri, histerezis ve zayıf kaldırıcı bildirimi. Grafik yumuşatması ses yoluna ek gecikme taşımamalı. |
| Termik yardımcısı | Android'de son dönüşlerin konum ve dikey hız izi, termik merkezi tahmini, ortalama kazanım ve güven göstergesi. Önce doğru zamanlanmış GPS/vario verisi gerekir. |
| Rüzgâr ve süzülüş | Yeterli ve kaliteli dönüş verisinde rüzgâr tahmini; düz süzülüşte filtreli yer esaslı süzülüş oranı. Sonuçlar yalnızca koşullar uygunsa gösterilmeli. Yer hızı, hava hızı veya dururken pusula yönü olarak sunulmamalı. |
| OLED uçuş ekranı | Büyük dikey hız/irtifa korunmalı; pil, kayıt, BLE ve açık ölçüm durumu eklenmeli. Pitch/roll gibi teşhis alanları teknik sayfaya taşınmalı. Telefon olmadan temel kullanım sürmeli. |
| Sürüm ve güncelleme | Cihaz kimliği, sensör/kalibrasyon sürümü, reset nedeni, en düşük boş bellek ve görev zamanlaması teşhisi. Daha sonra bütünlüğü doğrulanan ve yarım kalınca açılışı bozmayan güncelleme akışı; uçuş sırasında güncelleme engeli. |

Bu yönelim profesyonel ürünlerdeki işlevlerle uyumlu: XC Tracer Maxx II ayarlanabilir ses, GPS/BLE, uçuş kaydı ve termik ekranı sunuyor. Naviter'ın termik yardımcısı geçmiş dönüşlerin konum ve kaldırıcı bilgisini kullanıyor. Bunlar örnek işlevlerdir; bu projenin henüz aynı performansı verdiği iddia edilmiyor. [XC Tracer Maxx II](https://www.xctracer.com/en/xctracermaxx), [Naviter termik yardımcısı](https://kb.naviter.com/en/kb/thermal-assistant/)

## Sadeleştirilecekler ve korunacaklar

- Eski Wi-Fi arayüzü BLE sürümünde zaten derleme dışında. Arşivlenmesi bakım yükünü azaltabilir; mevcut uçuş performansında büyük bir kazanç varmış gibi değerlendirilmemeli.
- Kullanıcıya açık ham Kalman katsayıları teknik bölüme alınmalı; günlük kullanımda ölçülmüş profiller sunulmalı.
- Tam geçmiş kopyaları ve ölçüm görevi içindeki biçimlendirilmiş loglar azaltılmalı. Teşhis olanağı korunmalı.
- Açılışta otomatik sıfır hız/yer kalibrasyonu geri eklenmemeli. Mevcut barometrik başlangıç, kayıtlı kalibrasyon ve yumuşak füzyon geçişi korunmalı.
- Barometrik yedek yol, PROM CRC kontrolü, sıcaklık sınırı, BLE komut boyutu/sayı doğrulamaları ve eski bağlantının yanıtını yeni bağlantıya taşımayan oturum kontrolü korunmalı.
- FANET/FLARM için uygun radyo ve protokol donanımı gerekir. İncelenen projede böyle bir sürücü/bağlantı tanımı yok; yalnızca işlemci gücüyle eklenebilecek özellik olarak planlanmamalı.
- MPU6050 üç eksen ivmeölçer ve üç eksen jiroskop içerir. Bu projede ayrı manyetometre sürücüsü yok; sabitken gerçek kuzey pusulası vaat edilmemeli. [TDK MPU-6000/MPU-6050 ürün belgesi](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet.pdf)

## İncelemede yapılan sayısal doğrulama

Python ile mevcut yönelim denklemleri 100 Hz'de 10 saniye tekrarlandı. İdeal düzenek: dönme merkezindeki cihaz 30° yatık, gerçek pitch 0°, dünya düşeyi etrafında 30°/s dönüyor; doğrusal ivme yok. Gövde jiroskop değerleri `gx=0`, `gy=15°/s`, `gz≈25,98°/s`; ivmeölçer sürekli `(0; 0,5; 0,8660254) g`.

Gerçek Euler pitch hızı `gy*cos(roll) - gz*sin(roll) = 0`. Mevcut `alpha=0,98` filtresi ise yaklaşık **7,35° hatalı pitch** ve **−0,080579 m/s² sahte dikey ivme** üretiyor. Bu, `gz` ve eksen bağlaşımının eksikliğini gösteren sentetik bir örnektir; Kalman çıkışındaki gerçek uçuş hatasının büyüklüğünü ölçmez. Firmware C++ kodu derlenip çalıştırılmadı; ilgili denklemler tekrarlandı.

I²C alt sınırı: `1024 × 9 / 400000 = 0,02304 s`.

İdeal basınç örnek sıklığı: `20 / (42 × 0,01) = 47,619 Hz`. Hesap, 20 basınç + 1 sıcaklık dönüşümünün her birinde başlatma/okuma için iki görev turu ve kusursuz zamanlama varsayar.

## Uygulama sırası ve doğrulama

| Sıra | Paket | Tamamlanma ölçütü |
| --- | --- | --- |
| 1 | Örnek zamanları, OLED aktarımı, sensör kurtarma, ölçüm geçerliliği | OLED ve BLE yükünde örnek aralıklarının dağılımı ölçülür; veri durduğunda eski ölçüm geçerli gösterilmez; başlangıç hatasından kontrollü geri dönüş denenir. |
| 2 | Üç eksenli füzyon ve irtifa referansı sürekliliği | Sabit/dönen/eğilen düzenek, basınç rampası, örnek kaybı, sıcaklık değişimi ve havada açılış tekrarları yapılır. QNH değişiminde hızda sıfırlama veya yapay tepe oluşmaz. Gürültü ve gecikme aynı kayıt üzerinde karşılaştırılır. |
| 3 | Ses eşikleri/profilleri ve cihazda uçuş/ayar koruması | Eşik çevresindeki gürültü, normal çöküş, güçlü çöküş, termiğe giriş ve BLE kopması senaryoları ses kaydıyla doğrulanır. Sahip telefonun otomatik bağlantısı korunur. |
| 4 | GPS/BLE genişletmesi, pil ve bağımsız kayıt | GPS kaybı/taze olmayan FIX, telefon kopması, yeniden başlatma, dolu depolama ve kayıt sırasında enerji kesilmesi sınanır. Gerçek pil ömrü ölçülür. |
| 5 | Android termik/rüzgâr/süzülüş ve güncelleme | Kaliteli zaman eşleşmiş uçuş kayıtlarında yardımcı göstergeler doğrulanır; yetersiz veri açıkça belirtilir. Güncelleme kesintisinden kurtarma denenir. |

Projenin `test` klasöründe yalnızca PlatformIO şablon açıklaması bulundu. İlk iki paketle birlikte sensör kayıtlarını yeniden oynatabilen test altyapısı kurulmalı. Fiziksel doğrulamada basınç portu/rüzgâr etkisi, besleme gürültüsü, montaj titreşimi ve sıcaklık da kontrol edilmeli; bunlar kaynak koddan ölçülemez.

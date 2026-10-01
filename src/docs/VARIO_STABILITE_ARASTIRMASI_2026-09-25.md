# Açık kaynak variometrelerde stabilite araştırması

Araştırma tarihi: 25 Eylül 2026.

## Kısa sonuç

24 ilgili GitHub deposu tarandı. Bunların hepsi bağımsız algoritma değil: Hari Nair ve GNUVario ailelerinde ortak kod kullanan portlar var; iki depo ağırlıklı olarak web/donanım/kasa projesi. Aşağıda kaynak kodundan doğrulanan yöntemlerle yalnızca proje açıklamasında belirtilenler ayrıldı. Ek olarak PX4 ve x-io Fusion belgeleri incelendi.

Bizim cihaz için en değerli ders: barometreyi giderek daha ağır filtrelemek tek başına yeterli değil. İvmenin dünya dikey eksenine doğru dönüştürülmesi, kalibrasyon/ofset, sensörlerin zaman uyumu, sıcaklık ve fiziksel gürültü birlikte ele alınmalı. Dikey ivmeyi kapatmak veya küçük ivmede hızı zorla sıfırlamak önerilmiyor.

Bu çalışma kaynak incelemesidir. Projelerin uçuşta ne kadar iyi çalıştığını bağımsız olarak ölçmedim. Bizim cihazda bu turda yeni sensör kaydı alınmadı; aşağıdaki yerel neden adayları henüz deneyle doğrulanmış arıza teşhisi değildir. Firmware veya Android kodu değiştirilmedi, derleme/yükleme yapılmadı.

## İncelenen 24 depo

K: ilgili kaynak kodu/parametre dosyası incelendi. B: README/proje belgeleri incelendi. K, deponun tamamının denetlendiği anlamına gelmez.

| No | Depo / doğrudan kaynak | İnceleme | Stabilite yaklaşımı ve değerlendirme |
|---|---|---|---|
| 1 | [har-in-air/ESP32_IMU_BARO_GPS_VARIO](https://github.com/har-in-air/ESP32_IMU_BARO_GPS_VARIO/blob/master/src/sensor/kalmanfilter4d.cpp) | K+B | Yükseklik, hız, ivme ve ivme ofsetini izleyen KF4D. İvme ile yükseklik ayrı ölçümler. Gürültü analizi ve aynı kayıtta filtre karşılaştırması mevcut. En yararlı referanslardan biri. |
| 2 | [har-in-air/ESP32C3_BLUETOOTH_AUDIO_VARIO](https://github.com/har-in-air/ESP32C3_BLUETOOTH_AUDIO_VARIO) | B | Aynı KF4D ailesi. Kullanıcıya koşullara göre tepki ayarı; montaj gerilimi ve kalibrasyon için önemli deneyimler. `adapt` seçeneği belgede deneysel nitelikte. |
| 3 | [har-in-air/ESP8266_BLUETOOTH_AUDIO_VARIO](https://github.com/har-in-air/ESP8266_BLUETOOTH_AUDIO_VARIO) | B | KF4D portu. Hoparlör titreşiminin sensöre taşınmaması, MEMS modülünün sıkıştırılmaması ve kayıtlı kalibrasyona dönüş vurgulanıyor. Bağımsız yeni filtre değil. |
| 4 | [har-in-air/ESP8266_MPU9250_MS5611_VARIO](https://github.com/har-in-air/ESP8266_MPU9250_MS5611_VARIO) | B | IMU + barometre füzyonu. Yazar sensör beslemesinden ESP beslemenin baro gürültüsünü artırdığını raporluyor. Hareketli açılışta kayıtlı gyro ofsetleri kullanılıyor. |
| 5 | [har-in-air/STM32F103_MAX21105_MS5611_VARIO](https://github.com/har-in-air/STM32F103_MAX21105_MS5611_VARIO) | B | MAX21105 + MS5611 füzyonu; depoda üç durumlu Kalman uygulaması bulunuyor. Aynı geliştiricinin farklı donanım sürümü. |
| 6 | [har-in-air/Kalmanfilter_altimeter_vario](https://github.com/har-in-air/Kalmanfilter_altimeter_vario) | B | Üç durum: yükseklik, dikey hız, kalibrasyon sonrası kalan ivme ofseti. Ofsetin çevre koşullarıyla değişebileceği açıklanıyor. Bizdeki modelin amaçlarıyla çok yakın. |
| 7 | [har-in-air/ESP32_IMU_BARO_GPS_LOGGER](https://github.com/har-in-air/ESP32_IMU_BARO_GPS_LOGGER) | B | Gerçek sensör kaydını bilgisayarda tekrar işleyerek yalnız baro ve baro+ivme filtrelerini karşılaştırıyor. Bir filtre seçmek kadar test yöntemini örnek almak da değerli. |
| 8 | [qubolino/flybaby](https://github.com/qubolino/flybaby) / [Kalman](https://github.com/qubolino/flybaby/blob/master/src/KalmanVario.cpp) / [MS56XX](https://github.com/qubolino/flybaby/blob/master/src/MS56XX.cpp) | K+B | Bizimle aynı sensör ailesi: ESP32 + MPU6050 + MS5607. DMP quaternion, ofsetli Kalman ve MS5607/MS5611 için farklı sıcaklık telafisi. README yazıldığı sırada uçuş denemesi yapılmadığı belirtiliyor. |
| 9 | [mwesterm/esp32-vario](https://github.com/mwesterm/esp32-vario) | B | MPU6050 DMP ile yönelim. Yazar kendi donanımında 200 Hz yerine 100 Hz DMP okumada daha az gürültü; I2C veri parçası boyutuna hassasiyet raporluyor. Bunlar bizim yazılım için otomatik geçerli sonuçlar değil. |
| 10 | [prunkdump/arduino-variometer](https://github.com/prunkdump/arduino-variometer/blob/master/libraries/kalmanvert/kalmanvert.cpp) | K+B | GNUVario `kalmanvert`: yükseklik/hız durumları, ivme ile tahmin, barometre ile düzeltme; zaman damgasından gerçek dt. Girişteki standart sapmalar kare alınarak varyansa çevriliyor. |
| 11 | [prunkdump/GNUVario-TTGO-T5](https://github.com/prunkdump/GNUVario-TTGO-T5/blob/master/Sources/Beta%20Code/Ide%20Arduino/libraries/kalmanvert/kalmanvert.cpp) | K+B | Aynı `kalmanvert` yaklaşımını ESP32/e-paper ailesine taşıyor. Ayrı bağımsız algoritma başarısı olarak sayılmamalı. |
| 12 | [prunkdump/GNUVario](https://github.com/prunkdump/GNUVario) | B | Projenin web/dokümantasyon deposu; kaynak firmware'e yönlendiriyor. Yeni stabilite algoritması kanıtı değil. |
| 13 | [antoine5974/GNU-vario-E-SLIM](https://github.com/antoine5974/GNU-vario-E-SLIM) | B | GNUVario E tabanlı PCB/kasa varyantı. Yeni filtre yöntemi doğrulanmadı. |
| 14 | [openXsensor/openXsensor](https://github.com/openXsensor/openXsensor/blob/master/openXsensor/oXs_config_description.h) | K+B | Değişimin büyüklüğüne göre hassasiyet, çıkış histerezisi, isteğe bağlı baro+IMU, ek sıcaklık sürüklenmesi telafisi. Ölçüm filtresi ile ses/telemetri sakinleştirmesini ayırmak için değerli. |
| 15 | [mstrens/oXs_on_RP2040](https://github.com/mstrens/oXs_on_RP2040/blob/main/src/vario.cpp) | K+B | İki farklı hızlı alçak geçiren yükseklik filtresinin farkından vario; ardından değişime bağlı yumuşatma ve çıkış histerezisi. IMU varsa ayrı füzyon yolu. openXsensor ailesi. |
| 16 | [Openvario/sensord](https://github.com/Openvario/sensord/blob/master/KalmanFilter1d.c) / [örnekleme](https://github.com/Openvario/sensord/blob/master/main.c) | K+B | Basınç ve basınç değişimini Kalman ile izleyip variometreye dönüştürüyor. MS5611 örnekleme düzensizliğine bağlı sıçramalar ve D1/D2 tutarlılığı için özel önlemler var. Planörün toplam enerji basınç sistemi bizim el tipi cihazla aynı değil. |
| 17 | [FreeVario/FreeVario](https://github.com/FreeVario/FreeVario/blob/master/FreeVario/Src/kalman/kalman.c) | K+B | Hari Nair'in üç durumlu ofsetli filtresinin C portu. Depoda Madgwick de var. Ana uygulama `useKalman=0` ile başlıyor; kodun depoda olması her zaman varsayılan aktif yol olduğu anlamına gelmiyor. |
| 18 | [SkyDrop — bubeck fork'u](https://github.com/bubeck/SkyDrop/blob/master/skydrop/src/fc/vario.cpp) / [Kalman](https://github.com/bubeck/SkyDrop/blob/master/skydrop/src/fc/kalman.cpp) | K | Baro+ivme ile iki durumlu Kalman. Anlık, dijital ve ortalama vario ayrı tutuluyor; ekran değerlerine ayrı sönümleme var. Asıl `fhorinek/SkyDrop` adresi erişimde 404 verdi; incelenen bu fork güncel ürün yazılımı kabul edilmemeli. |
| 19 | [lebipbip/le-BipBip](https://github.com/lebipbip/le-BipBip/blob/master/filter.c) | K+B | Sabit örnekleme hızına göre tasarlanmış Butterworth/bant geçiren karakterli IIR ile basıncın türevini ve gürültü süzmeyi birleştiriyor. Barometre-only yaklaşım için örnek. |
| 20 | [glydrfreak/vSpeedVario-mini](https://github.com/glydrfreak/vSpeedVario-mini/blob/master/vSpeed_mini/vSpeed_mini.ino) | K+B | Basınç, yükseklik, sıcaklık ve hız için ayarlanabilir süreli hareketli ortalamalar. Basit ama birden çok aşamanın gecikmesi hesaba katılmalı. |
| 21 | [XT95/bipbip](https://github.com/XT95/bipbip/blob/master/program.ino) | K+B | BMP390L; yaklaşık 500 ms'lik yükseklik ortalamalarının farkıyla ses üretimi. Basit barometrik örnek; düşük gecikmeli gelişmiş IMU füzyonuna denk değil. |
| 22 | [zmolnar/sensorio](https://github.com/zmolnar/sensorio) | B | Gerçek veride UKF denemesi; BNO055 yönelime bağlı ofset ve sürekli ivmeli termik dönüşte yanlış dikey yön raporu. Geliştirici dört barometre fikrine yönelmiş ama bunu bitmiş/kanıtlanmış çözüm olarak sunmuyor. |
| 23 | [juangallostra/AltitudeEstimation](https://github.com/juangallostra/AltitudeEstimation) | B | İki aşamalı Kalman/tamamlayıcı yükseklik kestirimi. Önemli sakınca: 12 küçük ivme ölçümünden sonra dikey hızı sıfırlayabiliyor. Sabit hızlı tırmanışta ivme sıfıra yakın olabileceğinden bu mantık uçuş variosuna doğrudan taşınmamalı. |
| 24 | [iltis42/XCVario](https://github.com/iltis42/XCVario/blob/master/main/BMPVario.cpp) | K+B | İncelenen dosyada tahmin/ölçüm farkına bağlı kazanç, toplam enerji vario ortalaması ve ayrı sönümleme bulunuyor. Anlık ve ortalama göstergeleri ayırma açısından yararlı; planör TE ölçümünü el tipi varioyla karıştırmamak gerekir. |

## Bizim probleme en doğrudan karşılık veren bulgular

### 1. İvme ofseti ve yönelim: yalnızca filtre şiddeti değil

KF4D yükseklik, hız, ivme ve ivme ofsetini ayrı durumlar olarak izliyor. Kodda barometre ve ivme ölçüm gürültüleri de ayrılmış. Yüksek ivmede ofset süreç gürültüsü küçültülüyor; ofsetin hareketi yanlışlıkla öğrenmesi sınırlandırılmak isteniyor. `adapt` davranışı olduğu gibi kopyalanmamalı: kaynakta ölçüm yenilik kovaryansını değiştiriyor ve C3 belgelerinde deneysel olduğu belirtiliyor. Dört durumlu olması tek başına bizim üç durumlu filtreden daha iyi sonuç garantisi değildir. [KF4D kaynak kodu](https://github.com/har-in-air/ESP32_IMU_BARO_GPS_VARIO/blob/master/src/sensor/kalmanfilter4d.cpp)

Sensorio'nun raporu, yönelim hatasının sahte tırmanışa dönüşebileceğine somut bir örnek. Fakat onlar BNO055, biz MPU6050 kullanıyoruz; rapor bizim cihazın arızasını kanıtlamıyor. [Sensorio deneyimi](https://github.com/zmolnar/sensorio)

Tamamlayıcı referans [x-io Fusion](https://github.com/xioTechnologies/Fusion), hareket ivmesinin yönelim hesabını bozmasını sınırlayan açısal tutarlılık kontrolü ve toparlanma mekanizması içeriyor. Burada yönelim düzeltmesinde ivmeye güveni azaltmak ile variometrenin dikey ivme girişini kapatmak farklı işlemlerdir. Bizdeki norm/yön güven kapıları da benzer amacı taşıyor; önce davranışları ölçülmeli.

### 2. Sensörleri aynı zamana aitmiş gibi kullanmamak

[PX4 EKF2](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf) sensörleri tamponlayıp farklı gecikmelerini hesaba katıyor. Ayrıca ölçüm ile tahmin arasındaki farkı istatistiksel olarak kontrol ediyor. Tam PX4'ü taşımak gerekmiyor; zaman eşleme ve ölçüm tutarlılığı prensipleri değerli.

Bizde basınç önce 1,20 s zaman sabitli LPF'den geçiyor, sonra Kalman düzeltmesinde kullanılıyor. İvme yolu çok daha güncel. Bu, hareket başlangıcı/sonunda iki girdinin uyumsuzlaşmasına katkı yapabilir. LPF sabit bir zaman kaydırması değildir; zaman damgasından yalnızca 1,20 s çıkarmak da tam çözüm olmaz. Ön süzmeyi azaltma, gecikmeyi modelleme veya ölçüm yapısını değiştirme seçenekleri kayıt üzerinde karşılaştırılmalı.

### 3. Kalman ayarını ölçülen gürültüye dayandırmak

Hari Nair'in [gürültü analizi](https://github.com/har-in-air/ESP32_IMU_BARO_GPS_VARIO/blob/master/offline/noise/README.md) sabit cihazdan kayıt alıyor; uzun kayıtlardaki sürüklenmeyi rastgele gürültüden ayırıyor. [Filtre karşılaştırması](https://github.com/har-in-air/ESP32_IMU_BARO_GPS_VARIO/tree/master/offline/kf) aynı veri üzerinde farklı modelleri deniyor.

Bizim için çıkarım: ayarları yalnızca elde sallayıp beğenerek seçmek yerine aynı ham kayıt üzerinde durgun gürültü, hareket tepki süresi, durduktan sonra kuyruk ve eğme hatasını birlikte ölçmek gerekir. Ham ölçüm varyansı ile önceden filtrelenmiş, ardışık örnekleri ilişkili verinin varyansı aynı şey değildir. Başka projedeki sayılar birimleri ve örnekleme oranı doğrulanmadan kopyalanmamalı; örneğin cm² ile m² arasında 10.000 katsayı vardır.

### 4. Sıcaklık, besleme ve montaj da araştırılmalı

OpenXsensor, MS5611'in açılışta ısınmasına bağlı yükseklik sürüklenmesi için sensöre özgü ek telafi tanımlıyor; ışık etkisini de uyarıyor. Bu MS5607'ye aynı katsayıyla uygulanamaz. [Parametre açıklamaları, bölüm 4.5](https://github.com/openXsensor/openXsensor/blob/master/openXsensor/oXs_config_description.h)

Hari Nair'in eski ESP8266 projesinde ESP beslemesini sensör modülü regülatöründen almanın gürültüyü artırdığı raporlanıyor. Bluetooth audio projesinde hoparlör titreşimi ve MEMS montaj gerilimi vurgulanıyor. Bunlar bizim MAX98357A'lı cihazda ses açık/kapalı ve farklı besleme koşullarını kontrollü karşılaştırmak için gerekçe, mevcut devrenin hatalı olduğunun kanıtı değil. [Besleme deneyimi](https://github.com/har-in-air/ESP8266_MPU9250_MS5611_VARIO), [montaj ve hoparlör](https://github.com/har-in-air/ESP8266_BLUETOOTH_AUDIO_VARIO)

PX4, kasa etrafındaki hava akışının barometrede gerçek irtifa dışı basınç değişikliği oluşturabileceğini ve hava geçirgen korumayı açıklıyor. Bu yüzden sensör hava geçirmeyecek biçimde kapatılmamalı; fiziksel koruma da gecikme açısından test edilmeli. [Statik basınç etkisi](https://docs.px4.io/v1.15/en/advanced_config/static_pressure_buildup)

### 5. Sakin ekran ile hızlı uçuş ölçümü ayrı amaçlar

SkyDrop anlık, dijital ve ortalama variometreyi ayırıyor. openXsensor çıkış değişimlerine histerezis uyguluyor. Bunlardan alınacak fikir ölçümü sıfıra yapıştırmak değil, doğru ölçümün ekran ve ses sunumunu ayrı düzenlemek. Zayıf ama sürekli tırmanışı kaybetmemek gerekir. [SkyDrop](https://github.com/bubeck/SkyDrop/blob/master/skydrop/src/fc/vario.cpp), [openXsensor](https://github.com/openXsensor/openXsensor/blob/master/openXsensor/oXs_config_description.h)

## Mevcut yerel kodla karşılaştırma

Bu bölümdeki değerler internetten veya eski notlardan değil, bu turda açık çalışma dizininden okundu.

| Yerel bulgu | Anlamı / yapılacak doğrulama |
|---|---|
| `src/config.h:141`: basınç LPF zaman sabiti 1,20 s | Tek başına bu filtrenin basamak yanıtının %90'a erişmesi yaklaşık 2,76 s sürer. Bu, bütün cihazın vario gecikmesi değildir; fakat uzun kuyruğa aday bir etkidir. |
| `src/config.h:109`: çıkış LPF 0,25 s | Basınç ön süzmesine ek bir çıkış yumuşatması var. Yalnızca bunu küçültmek önceki aşamanın gecikmesini ortadan kaldırmaz. |
| `src/sensor_fusion.cpp:226`: yükseklik/hız/ivme-ofseti Kalman | Ofset takibi zaten mevcut. Araştırma sonucu “ilk kez bias ekleyelim” değil; mevcut modelin ayarını ve gözlenebilirliğini doğrulayalım. |
| `src/sensor_fusion.cpp:107`: dünya dikeyi boyunca izdüşüm ve yerçekimi çıkarma | Dikey ivme zaten hesaplanıyor. Altı yüz kalibrasyonundan sonra ara açılardaki artık hata, gyro ofseti, dinamik yönelim ve mekanik etki ayrı incelenmeli. |
| `src/barometers/ms56xx_barometer.cpp`: her 20 basınç örneğinde D2, alpha=0,10 ile süzülmüş sıcaklık üzerinden telafi | Ani sıcaklık değişiminde gecikmiş D2'nin basınca etkisi test edilmeli. Sabit alpha'nın saniye cinsinden etkisi gerçek D2 hızına bağlı; yorumdaki yaklaşık süreyi ölçülmüş sabit kabul etmemek gerekir. |
| `src/config.h:134` ve `src/sensor_fusion.h:58`: 3000 ms regresyon penceresi, fakat 128 örneklik tampon | Pencere en fazla 127 örnek aralığını kapsar: 50 Hz'de yaklaşık 2,54 s, 95 Hz'de 1,34 s. Bunlar örnek hesaplar, ölçülmüş mevcut hız değil. 3 s'lik kapsam 42,3 Hz üzerindeki sürekli örneklemede mümkün olmaz. |
| `src/vario_task.cpp:217-225,323-337`: regresyon ve füzyon yolları | Tampon sınırı baro referansı/fallback/ilk hız tohumlamasını etkiler; aktif füzyonun sürekli çıkışındaki her sorunu tek başına açıklamaz. |
| `src/vario_task.cpp:190,225,291`: baro zaman damgası var; düzeltme ve tahmin ana döngüde ayrı | Zaman damgası LPF ve regresyonda kullanılıyor; Kalman düzeltmesi gecikmiş ölçüm zamanı argümanı almıyor. Örneklerin fiziksel zamanları ve hesaplama sırası birlikte incelenmeli. Sırf `correct` önce diye hata ilan edilmemeli. |

## Önerilen sonraki çalışma — henüz uygulanmadı

1. Ham kayıt: D1/D2 veya mümkün olan en erken basınç, sıcaklık, filtreli basınç, accel XYZ, gyro XYZ, quaternion, dünya-Z ivme, bias, Kalman hızı, baro hızı, füzyon durumu ve gerçek zaman damgaları. Zaten var olan alanlar yeniden kullanılmalı.
2. Kontrollü, tekrarlanabilir yer testleri: sabit cihaz, yüksekliği değişmeden kontrollü eğme, belirli yükseklik değişimi ve duruş, ısınma, ses açık/kapalı. Elde eğme sırasında farkında olmadan dikey hareket yapılabildiği için mümkünse sabit eksenli tutucu kullanılmalı.
3. Aynı veriyi çevrimdışı tekrar işleme: mevcut filtre; daha hafif baro ön süzme ve yeniden ayarlanmış mevcut Kalman; daha sonra gerekirse KF4D adayı. Dikey ivme korunmalı.
4. Her adayda durgun hız standart sapması, eğme sırasındaki sahte hız, gerçek hareketin tepki süresi, duruş sonrası toparlanma ve zayıf sürekli tırmanış birlikte raporlanmalı. Sakin görüntü tek başına başarı ölçütü değil.
5. Ölçüm sonuçlarına göre sıcaklık telafisi/kalibrasyon, örnekleme zamanlaması, baro regresyon kapasitesi ve ekran/ses süzmesi hedefli düzenlenmeli. Kullanıcının stabilite–tepki kaydırıcısı ölçülmüş, güvenli aralıklara bağlanmalı.

Öncelik sıram: zaman uyumu ve filtre gecikmesi; eğimde ivme/ofset doğruluğu; sıcaklık ve fiziksel etkiler; ölçüme dayalı Q/R ayarı. Kalman türünü değiştirmek bunlardan sonra değerlendirilir. İncelenen algoritmalar uçuş güvenilirliği garantisi değildir; değişiklikler yer testleri ve bağımsız referansla doğrulanmadan uçuşta tek referans kabul edilmemeli.

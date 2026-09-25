# Masa üstünde dikey hız oynaması — 14 Eylül 2026

OLED ve Android aynı telemetriyi gösterdiğinden düzenleme firmware tarafındadır.

## Değişiklikler

- MS5607 basınç hesabının son bölmesi tamsayı yerine kayan noktayla yapılır.
  Böylece Pascal altı çözünürlük korunur. Önceki tam-Pascal basamakları deniz
  seviyesi yakınında yaklaşık 8 cm irtifa adımına karşılık geliyordu.
- Hesaplanan dikey hız için 0,45 saniye zaman sabitli birinci dereceden alçak
  geçiren filtre eklendi. OLED, BLE/XCTrack, ses ve trend geçmişi aynı değeri alır.
- Filtre sıfır bandı veya hareketsizlik varsayımı kullanmaz. Sürekli küçük
  yükseliş ve düşüşler korunur. İlk geçerli ölçüm doğrudan alınır; uçuş sırasında
  açılışta yapay sıfır hız eklenmez.
- Uçuş algılama ve yer kalibrasyonuna uygunluk kontrolü süzülmemiş kestirimi
  kullanır. Geçersiz çıktı filtre belleğini temizler. QNH değişiminde filtre
  hızına aynı ölçek dönüşümü uygulanır.

## Tepki ve test

Bu filtre tek başına bir hız basamağının %63'üne 0,45 s, %90'ına yaklaşık 1,04 s
sonra ulaşır; bu, mevcut sensör filtrelerinin gecikmesine eklenir. Filtrelenmiş
ses de aynı gecikmeden etkilenir. Zaman sabiti config.h içindeki
VarioTuning::OUTPUT_LPF_TAU_S değeridir; kayıtlı NVS ayarlarına bağlı değildir.

163 üretim C++ kontrolü geçti. Sentetik 1 Hz ±0,3 m/s sinüs gürültüsünün kararlı
çıkışı ±0,11 m/s altında kaldı. Sabit ±0,05 m/s hareketin sıfıra bastırılmadığı,
QNH dönüşümü, veri kesintisi ve hareketli başlangıç test edildi. Bu sonuçlar
fiziksel sensörde aynı gürültü azalmasını garanti etmez; gerçek kayıt alınmadı.

Tam sıfır garanti edilmez. Yavaş basınç değişimi veya sıcaklık kaynaklı sürüklenme
bu filtrenin hedefi değildir. Yeni firmware ile oynama sürerse ham basınç,
barometrik vario, Kalman vario, dünya-Z ivmesi ve IMU füzyon durumu birlikte
kaydedilerek ayrıştırılmalıdır. Cihaza otomatik yükleme yapılmadı.

# Sallama sonrası kalıcı negatif vario — 14 Eylül 2026

## Yeniden üretilen hata

AttitudeFilter içindeki directionTrust, ölçülen ve hesaplanan yerçekimi yönleri
30 dereceden fazla ayrıldığında sıfıra iniyordu. Bu durumda normal 1 g ölçümü ve
sıfır açısal hız geri gelse bile yön düzeltmesi çalışmıyordu. Hatalı yerçekimi
izdüşümü sürekli negatif dünya-Z ivmesi oluşturabiliyordu. Önceden eklenen
ivme doygunluğu ve Kalman inovasyon sınırı bu yön kilitlenmesini çözmüyordu.

Üretim kodunu kullanan yeni regresyon testi, düzeltme öncesinde başarısız oldu.
Bu, yazılımda doğrulanmış bir hata yoludur; kullanıcının fiziksel cihazındaki
olayın kesin eşleştirmesi için sensör kaydı/saha denemesi henüz yapılmadı.

## Değişiklik

- Yerçekimi referansı güvenilir değilse IMU füzyonu devreden çıkar; barometrik
  dikey hız kullanılmaya devam eder.
- Yaklaşık 1 g ve düşük açısal hız 0,5 saniye sürerse büyük yön hatasının
  düzeltmesi yeniden açılır. 180 derecedeki sıfır çapraz çarpım için kaçış ekseni
  kullanılır. Bu işlem kalibrasyon kaydetmez ve hızı sıfıra zorlamaz.
- Yön referansı tekrar güvenilir olduğunda mevcut iki saniyelik bekleme ve
  bir saniyelik karıştırma uygulanır. Kalman irtifası/hızı barometreden başlatılır.
- Jiroskop doygunluğu da algılanır. Kırpılmış dönüş verisiyle yön entegrasyonu
  yapılmaz; geçersiz ivme örnekleri füzyona ve yer kalibrasyonuna alınmaz.
- İlk yön kurulumu çok-g darbelerinden yapılmaz. Açılışta sabit durma veya
  zorunlu yer kalibrasyonu eklenmedi.

## Kontroller

`python checks/run_host_checks.py`: 154 üretim C++ kontrolü geçti.
60, 100, 179,9 ve 180 derece yön kaybından sonra 10 saniyelik normal örnek akışıyla
dikey ivmenin 0,02 m/s² altına döndüğü doğrulandı. Bu süre fiziksel cihaz için
garanti değildir. -3, 0 ve +2 m/s barometrik hızla yeniden füzyona giriş kontrol edildi.

Donanım denemesi: yeni firmware yüklendikten sonra sabit durumda, sallama
sonrasında ve farklı sabit açılarda dikey hızın toparlandığı kontrol edilmeli.
Devam eden kaymada barometrik vario, dünya-Z ivmesi, pitch/roll ve KF bias
değerleri birlikte incelenmeli. Cihaza otomatik yükleme yapılmadı.

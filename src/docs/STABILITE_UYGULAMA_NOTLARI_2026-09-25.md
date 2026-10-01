# Stabilite araştırması sonrası uygulanan genel iyileştirmeler

Tarih: 25 Eylül 2026

Bu turda kullanıcı isteğiyle sensör filtrelerinin karakteri yeniden ayarlanmadı.
Mevcut değerler korunarak güvenilirlik, gözlenebilirlik ve ayar uyumluluğu
iyileştirildi.

## Uygulananlar

- Barometrik regresyon tamponu 128'den 512 örneğe çıkarıldı. Böylece ayarlı
  3 saniyelik pencere yaklaşık 94-100 Hz gerçek örnekleme hızında gerçekten
  saklanabiliyor.
- Kalman barometre düzeltmesinde sayısal olarak daha kararlı Joseph kovaryans
  güncellemesi kullanılıyor. Büyük tekil basınç sıçraması uçuş hızını zorla
  sıfırlamıyor; ölçümün belirsizliği geçici olarak yükseltiliyor.
- Geçersiz/sonsuz belirsizlik parametrelerinin filtre durumunu bozması engellendi.
- Firmware ayarları yalnızca sağlama toplamlı `settings` kaydında tutuluyor.
  Sensör ve ses değerleri birlikte doğrulanıp tek
  atomik yazmayla kaydediliyor. Eski veya bozuk ayar alanı varsayılanlarla
  temizlenip yalnız bu kayıt oluşturuluyor; ayrı IMU kalibrasyonu korunuyor.
- Android sensör filtresi sayfasındaki denge kaydırıcısı kaldırıldı. Alfa,
  ivme belirsizliği, barometre belirsizliği ve ivme sapması uyum hızı virgül
  veya noktayla elle giriliyor. Ses ayarlarının kaydırıcıları korundu.
- Android ses ve sensör bölümleri tek `VarioSettings` nesnesi, tek `setSettings`
  BLE komutu ve tek "Tüm ayarları kaydet" düğmesi kullanıyor.
- Açılış loguna etkin filtre parametreleri eklendi. Kalman ivme sapması sınırın
  %90'ına ulaşırsa en fazla 10 saniyede bir tanı uyarısı yazılıyor.
- Üretim varsayılanında sürekli CSV kapalı ve ses açık. İstenirse kontrollü
  masa testinde CSV geçici olarak açılabilir.
- Ham seri kaydı güvenli biçimde alan ve özetleyen `checks/capture_vario.py`
  eklendi. Araç var olan dosyanın üzerine yazmaz ve cihazı bilerek resetlemez.

## Mevcut cihazdan alınan başlangıç kaydı

Dosya: `checks/captures/before-stability-20260925.log`

20 saniyelik kayıt tanı amaçlı ve yaklaşık 11,1 Hz'e seyreltilmiş akıştır;
tam hızlı filtre tekrarı değildir. Kayıt sırasında cihazın tamamen hareketsiz
olduğu yazılım tarafından kanıtlanamaz.

- Barometre üretim hızı: yaklaşık 93,7 örnek/s
- IMU okuma kaçırma artışı: 0
- Barometre veri yolu kaçırma artışı: 0
- Uzun vario döngüsü artışı: 0
- Basınç detrend sonrası standart sapması: yaklaşık 5,77 Pa
- Çıkış standart sapması: yaklaşık 0,45 m/s
- Kalman ivme sapması aralığı: -0,50 ile +0,50 m/s²

Son madde filtre ayarı yapma gerekçesi olarak tek başına kullanılmadı. Gerçek
hareket ve sabit masa bölümleri etiketlenmeden sebep ayrıştırılamaz.

## Donanım kontrol listesi

Araştırılan projelerin ortak pratiklerinden çıkarılmıştır:

- Barometreyi doğrudan ışık ve hava akımından koru; hava geçişini tamamen
  kapatma. Kullanılacak köpük nefes alabilir olmalı.
- Hoparlörün ve kasanın mekanik titreşimini IMU/barometre kartına taşımayacak
  yumuşak bağlantı kullan. MAX98357A sesi açık ve kapalıyken kayıtları kıyasla.
- MPU6050 kartını vida veya sert baskıyla germeden sabitle. Montaj değişirse
  altı yüz kalibrasyonunu yeniden kontrol et.
- Sensör beslemesindeki dalgalanmayı ses açık/kapalı ve BLE bağlı/bağsız
  koşullarda karşılaştır. Ortak toprak ve kısa besleme yollarını denetle.
- Uçuşta açılış desteklenmeye devam eder; cihazın açılışta zorunlu olarak
  sabit tutulmasına dayanan otomatik ofset öğrenmesi eklenmedi.

## Doğrulama

- 218 üretim C++ kontrolü geçti.
- Sentetik testler tam 3 saniyelik regresyon penceresini, zaman sayacı
  taşmasını, büyük barometre sıçramasını, uzun süreli sayısal kararlılığı,
  eğimli konumda dikey ivmenin açık kalmasını ve tek NVS kaydını kapsıyor.
- ESP32-S3 release firmware derlemesi geçti. Cihaza yükleme yapılmadı.
- Android filtre/ses ayarlarının 8 hedefli widget testi ve 101 testlik tam takım
  geçti; statik analizde sorun bulunmadı. Eski irtifa düğmesi varsayımı, güncel
  60 saniyelik grafik penceresi ve zamana bağlı termik testi güncellendi.
  APK derlenmedi.

# P0 — Quest girdi probu: ölçümler

Tarih: 2026-11-12. Headset: Quest 3S, Quest Browser (WebXR, `local-floor`). Sunucu: MSI, Wi-Fi (aksoy-lab).

Bu belge ham loglardan ve headset kaydından (§6) çıkarılan ölçümleri ve tasarıma sonuçlarını (§7) listeler. Kafa
akışı için alınan karar (Vuer 0.1.6 `Head` bileşeni) P3'te uygulandı; gerekçesi §4'te.

## 1. Kaynaklar

| Log | Araç | Vuer | Koşul |
|---|---|---|---|
| `data/quest-runs/input-probe-20261112-170509.jsonl` (442 s, 70 053 olay) | `tools/quest_input_probe.py --stereo` | 0.0.60 | Hands + MotionControllers 90 Hz istendi; sentetik stereo 2×960×540 JPEG, 30 Hz. 12–48 s: sağ kontrolcü elde, kafa hareketli. 50–442 s: hiçbir şey takipte değil (headset çıkarılmış, oturum açık). |
| `input-probe-20261112-165520.jsonl`, `-172607.jsonl` | aynı | 0.0.60 | Boş (headset bağlanmadı). |
| `server_1..3.log` (6 Kasım) | `operator/quest/server.py` | 0.0.60 | Sadece Hands, 90 Hz istendi. `server_3`: stereo test kaynağı 2×1280×720, 30 Hz açık. 5 s pencerelik olay oranları (ayrık örnek değil). |

Analiz: olay başına MSI alım zamanı (`t_mono`) ve tarayıcı zaman damgası (`ts`, `Date.now()`).
"Ayrık örnek" = bir öncekinden farklı payload; Vuer olayları bir zamanlayıcıyla gönderir ve XR karesi
gelmediyse aynı payload'ı tekrar yollar, bu yüzden gerçek örnekleme hızı ayrık örnek hızıdır.

## 2. Ölçüm tablosu (`input-probe-20261112-170509`, aktif pencere 12–48 s)

| Akış | Hız | Aralık p50 / p90 / p99 / maks (ms) | Boşluk > 50 / > 100 ms | Zaman damgası |
|---|---|---|---|---|
| Kafa `CAMERA_MOVE` (alım) | 30.5 Hz ort., 20–44 Hz | 23.6 / 68.9 / 196.5 / 436 | 153 / 43 (28 s'de) | **Yok** — `ts` sunucuda atanıyor |
| Kontrolcü olayları (tarayıcı `ts`) | 81.8 Hz (90 istendi) | 11 / 18 / 28 / 72 | 1 / 0 | Var (ms) |
| Kontrolcü olayları (alım) | 81.8 Hz | 10.1 / 19.8 / 139.5 / 162 | 84 / 29 | — |
| Sağ kontrolcü, ayrık poz (tarayıcı `ts`) | **42.9 Hz** | 22 / 33 / 56.7 / 90 | 29 / 0 | — |
| Sağ kontrolcü, ayrık poz (alım) | 42.9 Hz | 20.3 / 42.1 / 155.5 / 177 | 102 / 29 | — |
| `HAND_MOVE` olayları | 82.1 Hz | ≈ kontrolcü ile aynı | ≈ aynı | Var (ms) |
| Takipteki el | 3 olay (sol, t≈90 s) | — | — | — |

Tüm koşu (442 s): kontrolcü/el olayları 81.5 Hz; alımda > 100 ms boşluk 450 kez (≈ 1/s, kümeler halinde, aralarında
p50 0.24 s). Tarayıcı tarafında aynı olaylar düzenli (p99 28 ms). Tek yönlü gecikmenin yayılımı
(`t_rx − ts − min`): p50 7.8, p90 58.9, p99 133, maks 151 ms.

6 Kasım (stereo yok, sadece eller): `HAND_MOVE` 91 Hz, iki el de takipte 91 Hz; `CAMERA_MOVE` 5 s pencerelerde
0–119 Hz (p10/p50/p90 = 41/73/99 Hz hareket varken; dururken 0); jitter p99 3–5 ms. `server_3` (stereo 2×1280×720
açık): el olayları 91 → ~80 Hz.

## 3. Bulgular

1. **`CAMERA_MOVE` kontrol için kullanılamaz.** Vuer 0.0.60 istemcisinde bu olay XR'a özgü değil: OrbitControls'un
   `onChange`'inden gelir, sadece kamera matrisi değişince ve kare başına en fazla bir kez gönderilir. Kafa durunca
   akış durur (50–442 s arası 3 olay), hız harekete bağlıdır (0–119 Hz) ve tarayıcı zaman damgası yoktur.
2. **Ayrık örnek hızı XR kare hızıdır ve stereo yük onu yarıya düşürüyor.** Stereo görüntü açıkken kontrolcü pozu
   42.9 Hz'de değişiyor (tarayıcı aralığı p50 22 ms); stereo yokken eller ve kafa 90–119 Hz'e çıkıyordu. Vuer'in
   `createXRStore`'u `frameRate: "high"` ile en yüksek kare hızını istiyor (IWER Quest 3 profilinde 120 Hz;
   gerçek cihazda ölçülmedi). Render kareyi yetiştiremeyince kare hızı alt kata (≈45 Hz) düşüyor. Sonuç: hangi
   olay mekanizması kullanılırsa kullanılsın **kafa ≥ 50 Hz, ancak headset'te kare hızı ≥ 50 ise** mümkün. Bu P5'in
   (robot görüntüsü) sorumluluğu: görüntü yolu kare hızını 50'nin altına düşürmemeli.
3. **Ağ teslimatı kümeli.** Tarayıcı olayları düzenli gönderiyor, MSI'a ise 100–160 ms'lik takılmalarla ve sonra
   toplu geliyor. Neden ayrıştırılmadı: Wi-Fi uplink'i ya da probun stereo döngüsünün JPEG'leri asyncio döngüsünde
   senkron kodlaması. Sonuç (tasarıma girdi): kaynak zamanı alım zamanından ayrı taşınmalı; bayatlık kaynak zamanına
   göre ölçülmeli. P3'teki `xr_frame` bunu yapıyor.
4. **Kontrolcü yokken payload bir yer tutucu.** `left`/`right` alanı msgpack `ExtType(0, b"\x00")` (JS `undefined`),
   `leftState` `{}`. El için de aynısı. Varlık = 16 (kontrolcü) ya da 400 (el) float'lık geçerli bir matris.
5. **Thumbstick** `thumbstickValue = [axes[2], axes[3]]`; ileri itmek `y < 0` (WebXR gamepad kuralı).
6. **Sol/sağ kontrolcü butonları:** `aButton`/`bButton` sol kolda X/Y'dir. `trigger`, `squeeze` hem bool hem değer.
7. **Kontrolcü pozu `targetRaySpace`'ten geliyor**, `gripSpace`'ten değil (Vuer kaynağı). Park/ışın için doğru,
   el pozu yerine kullanılacaksa ofset gerekir.

## 4. Kafa hızı kararı (P3)

Değerlendirilen yollar:

| Yol | Sonuç |
|---|---|
| 0.0.60'ta kalıp `CAMERA_MOVE` | Reddedildi: değişim tetikli, zaman damgasız (bulgu 1). |
| Kafa pozunu el/kontrolcü paketine eklemek | Vuer istemcisini yamalamayı gerektirir (özel bileşen yükleme yok; D10'a aykırı). |
| Kendi istemci bileşenimiz | Vuer 0.0.60–0.1.6'da harici bileşen yükleme yolu yok; aynı nedenle reddedildi. |
| **Vuer ≥ 0.0.80 `Head` bileşeni** | **Seçildi.** `XRFrame.getViewerPose(originReferenceSpace)` her XR karesinde örnekleniyor, `fps` ile ayarlanan zamanlayıcıyla `HEAD_MOVE` olarak gönderiliyor, tarayıcı zaman damgası var. El ve kontrolcüyle **aynı referans uzayı ve aynı mekanizma**. 0.0.79'da şema var ama istemci yok; ilk çalışan sürüm 0.0.80. |

Sürüm: **0.1.6** (en yeni). Ayrı venv `data/quest-env` (`envs/quest-env.freeze.txt`); `data/quest-venv` (0.0.60 +
televuer) dokunulmadan duruyor, sadece `tools/quest_probe.py` (TeleVuer probu) onu kullanıyor.

Başsız Chrome + Meta IWER emülatörüyle (`tools/quest_emulator.mjs`, `XR_E2E=1 pytest tests/test_quest_e2e.py`)
uçtan uca doğrulandı: gerçek Vuer 0.1.6 istemcisi → sunucu → `xr_frame` → kayıt. Kafa, iki el ve iki kontrolcü
geliyor; kafa yüksekliği, çömelme, ±30° yaw, stick ileri = +1, A butonu, kontrolcü → el geçişi ve recenter senaryoyla
birebir. Hız tarayıcının kare hızı: yazılım GL ile ~36–43 Hz; aynı ortamda stereo görüntü (2×960×540, 30 Hz)
açılınca **~1.7 Hz**'e düştü. Bu Quest'i temsil etmez ama bulgu 2'yi doğruluyor: örnek hızını render belirliyor.
Headset'te beklenen: kafa ayrık örnek hızı = XR kare hızı (0.0.60 + stereo yükünde ≈ 43 Hz ölçüldü; yük yokken
90–120 Hz). Headset kaydı: §6 (görüntü açıkken 36 Hz; 50 Hz kapısı bayatlık ve jitter sınırlarıyla değiştirildi, §7).

0.1.6'nın bilinen bedelleri:
- **Yeniden bağlanma:** `?reconnect=&interval=` sorgusu artık okunmuyor; istemci 5 deneme yapıyor (1, 2, 4, 8, 16 s)
  ve yalnızca `error` olayında yeniden deniyor. Çözüldü: `operator/quest/vuer_app.py` sayfaya küçük bir WebSocket
  sarmalayıcısı enjekte ediyor; sayfa sunucu ne kadar kapalı kalırsa kalsın kendiliğinden yeniden bağlanıyor, VR
  oturumu sürüyor (emülatörle 65 s kesintide doğrulandı, `tests/test_quest_e2e.py`).
- Zaman damgası birimleri tutarsız: `HEAD_MOVE` ms, `HAND_MOVE`/`CONTROLLER_MOVE` s. `convert.client_time_s` ikisini
  de çözer.
- İstemci `bodyTracking` ve `meshDetection`'ı opsiyonel özellik olarak istiyor; Quest Browser Enter VR'da bir izin
  sorabilir.

## 5. Açık kalanlar

Karışık mod, el ↔ kontrolcü geçişi, recenter ve 0.1.6 kafa hızı §6'daki headset kaydıyla ölçüldü. Açık kalanlar:

- Kafa hızı stereo görüntü **kapalıyken** (kayıt sadece görüntü açıkken yapıldı; karar buna bağlı değil).
- Kafa hareketinden görüntüye gecikme (P5).
- Sessiz bir 22.6 s'nin ve 4 s'lik XR kare duruşlarının sistem tarafındaki nedeni (sistem menüsü, takip kaybı,
  headset'in çıkarılması). Vuer `visibilitychange`'i sunucuya iletmiyor; kayıttan ayırt edilemez.

## 6. Headset kaydı (Vuer 0.1.6, P3 sunucusu)

Kayıt: `data/fullbody-runs/20261112-195840` (`deploy/xr_frame_record.sh`, 300 s canlı, oturum 4). Sentetik stereo
görüntü açık: 2×960×540 JPEG, 30 Hz, ego. Operatör protokolü serbest uyguladı: kontrolcüleri bıraktı ve aldı, iki
elle çalıştı, tek kontrolcüyü iki tarafta da denedi. Doğrulama: `tools/xr_frame_validate.py` (düğümün kuralıyla:
50 Hz tik, `Stamp.fresh`). Kayıt eski assembler'la alındı; kontrolcü satırları el işaretçisi düşürülerek
(`assembler.py`'nin yeni kuralı kayda uygulanarak) hesaplandı.

| Ölçüm | Değer |
|---|---|
| Kafa, ayrık örnek | 8 702 örnek; 2 s pencerelerde kararlı **36 Hz**, tüm kayıtta 29.0 Hz (duruşlar dahil). Aralık p50 / p90 / p99 **29 / 41 / 72 ms** |
| Kafa, `HEAD_MOVE` olayı | 74–83 olay/s (90 Hz istendi). Fazlası, XR karesi gelmeden yeniden gönderilen aynı payload |
| Ağ gecikmesi (`t_rx − t`) | p50 / p99 **2.0 / 4.6 ms**. 0.0.60 probundaki 100–160 ms teslimat takılmaları bu sunucuyla görülmedi |
| Kafa > 200 ms (PAUSED) | **10 duraklama**, hepsi kaynak boşluğu (XR karesi yok), 0 teslimat. 6'sı el ↔ kontrolcü geçişinde (20 ms – 4.8 s), 3'ü sakin kullanımda (4.1 s el/kontrolcü yokken; 0.56 s iki el takipteyken; 0.4 s son takipten hemen önce), 1'i kayıt sonunda (22.4 s) |
| Kontrolcü > 150 ms (yürüme IDLE) | Taraf başına **1** (Enter VR anında, 60 ms), 0 teslimat. Kontrolcü 33.9 Hz |
| Recenter | 1; algılandı (kafa xy 0.40 m → 0.0001 m). Kare aralığı 23 ms: **recenter XR karelerini durdurmuyor** |
| Modlar (s) | iki el 125.8, iki kontrolcü 70.2, sol yok + sağ el 35.7, sol yok + sağ kontrolcü 12.1, sol el + sağ yok 1.1, hiçbiri 55.3 (22.6 s kayıt sonu dahil). **Bir tarafta el, diğerinde kontrolcü: 0.0 s** |
| Geçiş süresi | kontrolcü → el **114–130 ms** (8 kez), el → kontrolcü **60–1527 ms** (6 kez; arada hiçbir giriş yok) |

Bulgular:

1. **Kafa hızı render'a bağlı, Vuer kısmıyor.** `Head` her XR karesinde `getViewerPose` okuyor ve son değeri
   `setInterval(1000/fps)` ile yolluyor (`vuer/client_build`, `function Head`); değişince-gönder ya da başka bir
   kısma yok. Olaylar 74–83/s gelirken ayrık örnek 36/s: XR kare hızı ≈ 72/2. Görüntü açıkken 50 Hz'e
   ulaşılmıyor. **Karar (kullanıcı):** headset'in verdiği hız kabul; kriter hız değil bayatlık ve jitter (§7).
2. **4.2 s'lik kafa boşluğu bir kaynak boşluğu.** Oturum değişimi değil (canlı kayıt baştan sona oturum 4;
   oturum 3, Enter VR'dan önce bitti) ve recenter değil. Sayfa canlıydı (olaylar 47/s sürdü), ama yeni XR karesi
   yoktu; o sırada ne el ne kontrolcü takipteydi. Tüm uzun boşluklar böyle: XR kare döngüsü duruyor, ağ değil.
3. **Quest el ↔ kontrolcü geçişinde XR karelerini durduruyor:** 10 duraklamanın 6'sı bir geçişin ±1 s içinde.
4. **Tracked bir el, Vuer'de aynı zamanda "kontrolcü".** `MotionControllers` ≥ 6 butonlu gamepad'i olan her
   input source'u alıyor; Quest'te takipteki el de böyle (pinch tetikte, poz el işaretçisi, bilekten ~13 cm). Bu
   yüzden eski doğrulayıcı "aynı tarafta el + kontrolcü 162 s" ve toplamı süreyi aşan modlar gösterdi. Düzeltme:
   `assembler.py` takipteki elin tarafındaki kontrolcüyü düşürüyor; xr_frame'deki kontrolcüler artık fiziksel.
5. **Karışık mod yok (Quest 3S, Quest Browser, primary input sources).** Tek kontrolcü elde, diğer el boştayken
   Quest ya kontrolcü modunda kaldı (boş el takip edilmedi: "sol yok + sağ kontrolcü") ya da el moduna geçti
   (kontrolcüyü tutan el görünmez, tutulan kontrolcü raporlanmaz: "sol yok + sağ el"). Quest global olarak ya
   el ya kontrolcü modunda.
   - WebXR'da bir yol var: Meta Browser 37.1 sürüm notları (deneysel): "If hands are enabled in WebXR, you can
     now get one hand and one controller. In addition, the non-primary inputs can now be tracked in the session's
     trackedSources attribute." Kaynaklar: [Browser Developer Release Notes](https://developer.oculus.com/documentation/web/browser-release-notes)
     (şimdi https://developers.meta.com/vr/release-notes/web/ altında, eski notlar orada listelenmiyor) ve
     [Babylon.js forumu](https://forum.babylonjs.com/t/support-for-latest-webxr-features-in-babylon-js/54642).
     Meta'nın native multimodal belgeleri Quest 3, Quest Pro ve Pro kontrolcülü Quest 2'yi listeliyor; 3S için
     tarayıcıda destek bulunamadı.
   - Vuer 0.1.6 bunu kullanamaz: `Hands` ve `MotionControllers` sadece `session.inputSources`'u okuyor. Altındaki
     @react-three/xr store `trackedsourceschange`'i dinliyor ama Vuer bileşenleri kullanmıyor. Okumak istemciyi
     değiştirmeyi gerektirir (D10).
   - **Karar:** P0'ın yedeği. Yürüme iki kontrolcüyle, kontrolcüler bırakılınca iki el (§7).
6. **El → kontrolcü geçişi uzun:** kontrolcü alındıktan sonra 0.6–1.5 s hiçbir giriş yok.

## 7. Tasarıma sonuçlar (önerilen PLAN değişiklikleri)

- **Kafa kriteri:** düğüm 50 Hz'de son örnekle çalışır. Gereken: kafa > 200 ms bayatsa PAUSED (değişmedi),
  örnek aralığı p50 ≤ 40 ms ve p99 ≤ 100 ms. 100 ms, en hızlı kafa döngüsünün (heading bekleme 0.3 s) üçte biri;
  hız sınırları (60°/s, 0.5 rad/s) bir aralıkta en fazla 6° / 0.05 rad adım bırakır. Sakin kullanımda en fazla
  0.2 duraklama/dk (geçiş, recenter ve oturum sonundakiler ayrı sayılır). Bu kayıt: jitter geçer (29 / 72 ms),
  sakin duraklama 0.6/dk ile geçmez; bu platformun XR kare duruşları, bizim kodumuzun değil.
- **Bayatlıktan dönüş otomatik olmalı:** her el ↔ kontrolcü geçişi kafayı 200 ms'den uzun durdurabiliyor.
  Bayatlık PAUSED'ı butonla çözülürse iki ele geçen operatörün basacağı buton kalmaz. Öneri: bayatlıkla girilen
  PAUSED, aynı oturum ve epoch'ta kafa 0.5 s taze kalınca kendiliğinden ACTIVE'e döner (engage geçişiyle). Buton,
  recenter ve oturum değişimiyle girilen PAUSED elle çözülür.
- **D8:** taraf başına kural aynı kalır: el takipteyse TRACKED, değilse PARKED (kontrolcü olsun olmasın; kayıp
  için §5'in 0.25 s / 1 s kuralı). Platform karışık modu vermediği için operatöre kural: **"Yürümek için iki
  kontrolcü elde. Ellerle çalışmak için ikisini de bırak; yeniden yürümek için ikisini de al."** Tek kontrolcü
  tutulursa sistem yine tutarlı: Quest kontrolcü modunda kalırsa o stick yürütür, diğer kol park; el moduna
  geçerse boş el TRACKED, diğer kol park, yürüme yok. Karışık mod bir gün gelirse taraf başına kural onu da
  kendiliğinden kapsar.
- **Kontrolcü bayatlığı:** 150 ms kuralı örnek zamanına göre, düğümün tikinde (`Stamp.fresh`). Bu kayıtta sadece
  Enter VR anında tetiklendi; ağ kaynaklı hiç yok.

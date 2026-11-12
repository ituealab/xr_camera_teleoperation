# P0 — Quest girdi probu: ölçümler

Tarih: 2026-11-12. Headset: Quest 3S, Quest Browser (WebXR, `local-floor`). Sunucu: MSI, Wi-Fi (aksoy-lab).

Bu belge mevcut ham loglardan çıkarılan ölçümleri ve henüz ölçülmemiş olanları listeler. Kafa akışı için alınan
karar (Vuer 0.1.6 `Head` bileşeni) P3'te uygulandı; gerekçesi §4'te.

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
90–120 Hz). 50 Hz kapısı headset kaydıyla ölçülecek (§5).

0.1.6'nın bilinen bedelleri:
- **Yeniden bağlanma:** `?reconnect=&interval=` sorgusu artık okunmuyor; istemci 5 deneme yapıyor (1, 2, 4, 8, 16 s)
  ve yalnızca `error` olayında yeniden deniyor. Çözüldü: `operator/quest/vuer_app.py` sayfaya küçük bir WebSocket
  sarmalayıcısı enjekte ediyor; sayfa sunucu ne kadar kapalı kalırsa kalsın kendiliğinden yeniden bağlanıyor, VR
  oturumu sürüyor (emülatörle 65 s kesintide doğrulandı, `tests/test_quest_e2e.py`).
- Zaman damgası birimleri tutarsız: `HEAD_MOVE` ms, `HAND_MOVE`/`CONTROLLER_MOVE` s. `convert.client_time_s` ikisini
  de çözer.
- İstemci `bodyTracking` ve `meshDetection`'ı opsiyonel özellik olarak istiyor; Quest Browser Enter VR'da bir izin
  sorabilir.

## 5. Henüz ölçülmedi (headset ve kullanıcı gerekir)

- **Karışık mod (D8):** bir el kontrolcüde, diğeri serbest el takibinde. 12–48 s'de sol el serbestti ama takip
  edilmedi; bu, Quest'in kontrolcü etkinken el takibini kapattığını düşündürüyor, fakat kayıtta sol elin gerçekten
  görüş alanında olup olmadığı bilinmiyor. **Karar verilemedi.**
- Taraf başına el ↔ kontrolcü geçişi (kontrolcüyü bırakma/alma) ve geçiş süresi.
- Recenter: Vuer XR `reset` olayını sunucuya iletmiyor (0.0.60 ve 0.1.6 istemcileri incelendi). P3 sunucusu
  recenter'ı kafa pozunun kendi hareketinden sapan ve orijine/ileriye düşen bir adımdan algılıyor
  (`xr_frame.epoch`, emülatörde doğrulandı). 200 ms'den uzun bir kafa boşluğunun içinde olan recenter ayırt
  edilemez; supervisor o boşlukta zaten PAUSED'a geçer. Quest'te recenter hareketi (Meta tuşunu basılı tutmak)
  sırasında XR karelerinin durup durmadığı headset'te ölçülmeli.
- Vuer 0.1.6 `HEAD_MOVE` gerçek hızı ve düzeni, stereo görüntü açıkken ve kapalıyken.
- Kafa hareketinden görüntüye gecikme (P5).
- Bunların hepsi P3'ün kayıt komutu ve doğrulayıcısı ile ölçülür: `deploy/xr_frame_record.sh` (bkz. `RUN.md`).

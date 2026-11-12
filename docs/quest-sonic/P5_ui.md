# P5 — Robot gözüyle görüntü ve headset arayüzü

Tarih: 2026-11-12. Makine: MSI Raider 16 (GPU 145 W), SONIC `a0732b6` prebuilt deploy, MuJoCo 3.15, Vuer 0.1.6.
Doğrulama insansız yapıldı: birim testleri ve headless Chrome + IWER Meta Quest emülatörü (GPU'da), gerçek sim ve SONIC
ile (§5). Headset bu gece kullanılmadı.

## 1. Operatör ne görür, ne yapar

**Çalıştırma (tek komut):**

```bash
deploy/xr_fullbody_sim.sh start -- --source quest     # sim, SONIC, kamera köprüsü, Quest sunucusu, node
deploy/xr_fullbody_sim.sh stop
```

Quest'te `https://192.168.1.105` → **Enter VR**.

**Görüntü.** Robotun kafasındaki ZED stereo kameranın görüntüsü (simde `zed_left` / `zed_right`, 960×540, 30 fps).
Varsayılan **Ego**: 2 m önde bir pencere. **İmmersif**: görüntü kameranın tüm görüş alanını kaplar. **Kapalı**:
görüntü yok. Sol üst köşede sim saati ve görüntünün çekildiği anın saati yazar (`sim 123.456  23:12:05.123`).
Görüntü 0.5 s gelmezse kaldırılır, donmuş görüntü canlı gibi görünmez.

**Durum şeridi.** Panel kapalıyken görüşün altında küçük bir satır: robot durumu (KAPALI / HAZIR / AKTİF /
DURAKLADI), mod (YÜRÜME / ELLER), yön kilidi (KİLİT), SONIC yoksa uyarı, kalibrasyon gerekiyorsa "kalibre et",
kafa verisi yoksa uyarı, açık bir kayıt bölümü varsa "● bölüm", ve "Menü: stick / sol avuç".

**Paneli açmak:**
- Kontrolcüyle: herhangi bir **thumbstick'e bas** (tıkla). Quest Browser menü (≡) tuşunu WebXR'a vermiyor; stick
  basma §6'da başka bir işe ayrılmamış tek tuş.
- Elle: **sol avucunu yüzüne çevir**, 0.4 s tut. Avuç tekrar dönene kadar panel yeniden açılmaz.

Panel açıldığı yerde dünyaya sabitlenir: başının 0.45 m önünde, göz hizasının 0.28 m altında (göğüs), dik ve sana
dönük.

**Seçmek:**
- Kontrolcüyle: kontrolcünün ışını + **tetik**.
- Elle: omuzdan işaret parmağı köküne giden ışın + **başparmak-işaret parmağı pinch**.
- Işın beyaz çizgi olarak görünür, pinch/tetik basılıyken yeşil. Işının değdiği düğme beyaz çerçeveyle vurgulanır.
- **Durdur**, sekmeler ve **X** basınca hemen çalışır. Robota ya da görüntüye giden her şey **0.6 s basılı tutmayla**
  onaylanır: düğmenin sağında sarı halka dolar. Bırakırsan ya da ışın düğmeden çıkarsa iptal olur. +/– düğmeleri
  basılı tuttukça 0.3 s'de bir tekrar eder.

**Panel açıkken robot donar.** Robot AKTİF'se panel açılınca DURAKLADI'ya geçer (yürüme IDLE, yön donuk, kollar 1.5 s'de
nötre). Panel kapanınca robot kaldığı yerden devam eder. Panelden **Duraklat**'a basarsan duraklatma senin olur:
panel kapanınca robot duraklı kalır, **Devam** ile sürer.

**Sayfalar:**

| Sayfa | İçerik |
|---|---|
| Üstte her zaman | **DURDUR** (kırmızı, tek tık: SONIC durur, robot sönüm moduna geçer), durum satırı, **X** (kapat) |
| **Robot** | **Başlat** (SONIC'i başlatır, robot ayağa kalkar: HAZIR), **Kalibre et ve devral** (nötr duruşta: dik dur, ileri bak; robot AKTİF olur, panel kapanır), **Duraklat / Devam**; durum: robot, SONIC, kalibrasyon, mod, yön kilidi, giriş cihazları |
| **Hareket** | `fullbody/settings.py` `KNOBS` tablosundan: Hız, Stick ölü bölge, Yön kilidi, Çömelme, Eğilme kazancı |
| **Eller** | Aynı tablodan: Kol takibi, Parmaklar; mod ve giriş cihazları |
| **Görüntü** | Ego / İmmersif / Kapalı |
| **Sistem** | **Yeniden kalibre et** (recenter sonrası), **Bölüm başlat / bitir** (run log'a deneme işareti), hızlar: kafa, el, kontrolcü, kamera fps ve yaşı, node hızı, kafa yaşı; kayıt dizini |

Ayar satırları node'un o anki değerini gösterir (node'dan gelen durum); +/– değeri knob adımıyla değiştirir, aralık
dışına çıkamaz. Panel ayarları `configs/fullbody.yaml` `operator_settings` varsayılanlarının üstüne çalışır, node
yeniden başlayınca varsayılana döner ve her tick run log'a yazılır.

**Kayıt.** Node her koşuyu baştan sona kaydeder (`data/fullbody-runs/<zaman>/`). "Bölüm başlat/bitir" kaydı açıp
kapatmaz, denemeleri işaretler (`MARK` → node notu ve `tick.ui`).

## 2. Parçalar

| Yol | Tek görevi |
|---|---|
| `sim/camera_bridge.py` | Sim kamera akışı (:5555, upstream `ImageUtils`: RGB BGR gibi kodlanmış base64 JPEG) → upstream `decode_image` ile çöz, saati yaz, doğru renkli JPEG çifti → :5605 (`deploy/zed_stereo_stream.py` formatı). `stamp_ns` = render edilen fizik durumunun duvar saati. |
| `operator/quest/ui.py` | Panelin tüm davranışı, saf: yerleşim (knob satırları `KNOBS`'tan), ışınlar, raycast, hover, basılı tutma onayı, sayfalar, açma/kapama, açıkken dondurma. |
| `operator/quest/ui_draw.py` | Panel ve şeridi JPEG'e çizer (TrueType font: Türkçe harfler headset'te birebir). |
| `operator/quest/connections.py` | Hangi headset bağlantısı akışları besler: VR'da render eden (farklı kafa örnekleri gönderen). |
| `operator/quest/server.py` | Bağlama: paneli 60 Hz adımlar, istekleri UiAction olarak yayınlar, node durumunu alır, resimleri upsert eder; `/state`, `/frames`. |
| `fullbody/operator_link.py` | Panel ↔ node tel formatı ve teslim kuralları (saf). |
| `fullbody/node.py` | Ayarların sahibi: SET'i `with_value` ile uygular, MARK'ı kendisi tutar, kalanını kaynağa verir; durumunu yayınlar. |
| `tools/quest_emulator.mjs --puppet`, `tools/quest_puppet.py` | Testler için emülatörü adım adım süren operatör: kontrolcü/el ışınıyla hedefleme, tetik, pinch, avuç. |
| `tools/view_latency.py` | Görüntü gecikmesi aşamaları ve node'da kafa örneği yaşı. |

Panel çizimi: dünyaya sabit bir Vuer `Image` (0.44 × 0.28 m, 2400 px/m), şerit kafaya sabit `ImageBackground`
(0.9 m'de), ışınlar `Line`. Panel resmi yalnız değişince ve en çok 15 Hz yeniden çizilir (~70 kB JPEG), şerit 2 Hz.

## 3. Portlar ve taşıma

| Port | Yön | İçerik |
|---|---|---|
| :5605 | camera_bridge PUB → sunucu SUB | stereo JPEG (`v, sequence, stamp_ns, left, right, sim_time, state_wall_time`) |
| :5597 | sunucu PUB → node SUB (`node.ui_addr`) | `ui_actions`: son 2 s'nin tüm UiAction'ları; her yeni eylemde ve 0.2 s'de bir |
| :5595 | node PUB → sunucu SUB (`node.status_addr`) | `node_status` (`NodeStatus`), 10 Hz |
| :8771 | HTTP | `/state` (node durumu + panel: açık mı, öğelerin WebXR konumları, hover, basılı tutma), `/frames` (gönderilen görüntü kareleri: damgalar, boyutlar), `/health`, `/view?mode=` |

**Teslim kuralları (`operator_link.py`):** PUB/SUB kayıplı olduğu için her mesaj son 2 s'deki tüm eylemleri taşır;
node her `(server, seq)`'i bir kez alır, gelişte 1 s'den eski eylemi atar (geç uygulama yok, sonradan açılan node eski
basışa göre davranmaz). Yeniden başlayan sunucu yeni bir sıra başlatır. Node durumu 1 s gelmezse panel robota giden
her şeyi kapatır; **Durdur her zaman gider**.

**Node tarafı:** her tick alınan eylemler `NodeCore.step`'e girer. SET ayarları değiştirir (aralığa kırpılır,
bilinmeyen anahtar not düşer), MARK node'da kalır, START/STOP/CALIBRATE/PAUSE/RESUME ve SET `OperatorData.ui` ile
kaynağa gider, `OperatorData.settings` o anki ayarlardır. `tick` kaydına `ui` (alınan eylemler) ve `settings`
yazılır; `tools/fullbody_replay.py` eylemleri geri besler ve ayarları da karşılaştırır.

**Durum (`NodeStatus`):** supervisor durumu ve kendiliğinden dönüp dönmeyeceği, lifecycle fazı, SONIC
(absent/waiting/controlling), live, ready (kalibre), `operator` (kaynağın durumu: mode, heading_lock, calibrating, …),
ayarlar, tick hızı, gecikmiş tikler, kaynak başına örnek yaşları ve telemetri yaşı, run dizini, bölüm, son eylem
sırası, son notlar.

## 4. Ölçümler

Koşu: `data/fullbody-runs/20261113-001352` (sim + SONIC + köprü + Quest sunucusu + node, emülatör GPU'da; §5).
Araç: `tools/view_latency.py`.

**Görüntü yolu** (fizik durumu → headset sayfasında çözülmüş resim; 454 kare, sol göz, boyuta göre eşlendi):

| Aşama | p50 | p90 | p99 |
|---|---|---|---|
| fizik durumu → sunucu (render, sim JPEG, köprü çöz+saat+kodla, ZMQ) | 10.8 ms | 12.7 ms | 13.7 ms |
| sunucuda bekleme (4 ms gönderim döngüsü) | 2.0 | 3.6 | 4.0 |
| sunucu → sayfa (websocket, istemci) | 9.6 | 11.9 | 58.3 |
| JPEG çözme (sayfada) | 4.2 | 5.8 | 10.0 |
| **fizik durumu → çözülmüş resim** | **27.0** | **30.2** | **77.6** |

Köprü kendi başına: dönüştürme p50/p99 3.3/4.4 ms, 30.0 fps. Kamera 30 fps olduğu için kare yaşı buna ortalama 17 ms
(0–33) daha ekler; headset'te bir sonraki XR karesine kadar ≤ 14 ms (72 Hz) ve Wi-Fi p50/p99 2/5 ms (P0) gelir.

**Kafa → node:** yeni kafa örneğinin, onu ilk kullanan tick'teki yaşı p50/p90/p99 **8.6 / 16.6 / 20.6 ms** (1852 örnek;
emülatör localhost'ta, Quest'te Wi-Fi'nin 2–5 ms'si eklenir).

**Kafa hareketi → robot görüntüsünün hareketi, doğrudan:** robot AKTİF iken emülatörde kafa 0.3 s'de 15 cm öne/geri
(4 adım). Kafanın ilk 1 cm hareket ettiği örnekten, robot görüntüsünün önceki karelerin titreşiminin 3 katından fazla
değiştiği ilk karenin fizik zamanına: **89, 127, 149, 198 ms** (ortanca ~138 ms). Bu, posture eşlemesinin hız sınırını
(0.5 rad/s, PLAN §5) ve SONIC'in gövdeyi oynatma dinamiğini içerir; görüntü yolu değil, robotun tepkisi baskın.

**Toplam ve < 150 ms hedefi:**
- Bizim taşıma yolumuz (kafa → node + fizik → çözülmüş resim + gösterim): p50 ≈ 9 + 27 + 17 (kare yaşı) + 7 (XR karesi)
  ≈ **60 ms**, p99 ≈ 21 + 78 + 33 + 14 ≈ **146 ms**. Hedefin altında.
- Robotun tepkisi dahil "kafayı eğdim → görüntü kıpırdadı": 89–198 ms + görüntü yolu (≈ 50 ms) ≈ **140–250 ms**. Bu
  hedefin üstünde; kaldıraçlar robot tarafında: posture hız sınırı ve filtreleri (P7'nin), SONIC'in gövde tepkisi.
  Taşıma yolunda kısaltılacak anlamlı bir şey yok (kamerayı 60 fps'e çıkarmak ortalama 8 ms kazandırır).
- Gerçek headset'te ölçülmedi: Quest'in JPEG çözme ve kompozitör süresi emülatörden farklı olacak.

**Headset render hızı:** robot görüntüsü açıkken emülatör (GPU) kafa örneklerini 60 Hz'de verdi, görüntü 30 Hz geldi.
Köprünün çıkışı P0'daki sentetik yükle aynı (2×960×540 JPEG q80, 30 fps, ~50 kB/göz, ~23 Mbit/s); P0 bu yükle
Quest'te 36 Hz kafa ölçtü, P5 bunun üstüne yalnız panel açıkken ≤ 15 Hz'lik bir resim (~70 kB, değişince) ve 2 Hz
şerit ekliyor. Headset'te doğrulanmadı.

**Diğer:** Başlat → HAZIR 0.3 s (binary bekliyorken); Durdur → node KAPALI aynı tick, SONIC kontrolü 0.1 s'de
bırakıyor; panel 60 Hz adımlanıyor, panel resmi çizimi 5–8 ms (ayrı thread).

## 5. Uçtan uca doğrulama (emülatör + gerçek sim)

```bash
data/xr-env/bin/python -m pytest -q xr_camera_teleop/tests/test_quest_ui.py xr_camera_teleop/tests/test_fullbody_operator_link.py \
    xr_camera_teleop/tests/test_quest_connections.py xr_camera_teleop/tests/test_view_latency.py      # birim
XR_E2E=1 data/xr-env/bin/python -m pytest -q xr_camera_teleop/tests/test_quest_panel_e2e.py -k drives   # simsiz, ~10 s
XR_E2E_SIM=1 XR_E2E_OUT=<dizin> data/xr-env/bin/python -m pytest -q -s xr_camera_teleop/tests/test_quest_panel_e2e.py -k sim
```

Sim'li test (`test_panel_runs_the_sim_robot`, ~90 s, tek yığın kuralı yüzünden kilit altında) gerçek launcher'ı
(`start --no-viewer -- --source quest`) ve emüle edilmiş Quest'i kullanır. Geçen koşu (`20261113-001352`):

| Adım | Nasıl | Sonuç |
|---|---|---|
| Panel aç | sağ thumbstick basma | açıldı |
| **Başlat** | kontrolcü ışını + tetik 0.8 s | 0.3 s'de HAZIR; bant düştükten sonraki 3 s'de gövde eğimi hep < 15° (ayakta) |
| Ayar | **el ışını + pinch**: Hareket sekmesi, Hız – | node `speed_scale` 1.0 → 0.9; durumda ve her tick'in `settings`'inde |
| **Kalibre et ve devral** | el ışını + pinch 0.8 s | panel kapandı, kalibre, AKTİF |
| Kafa eğme ölçümü | kafa 15 cm öne/geri ×4 | §4 |
| Avuçla aç | sol avuç yüze | panel açıldı, robot DURAKLADI; X ile kapanınca AKTİF |
| Görüntü | kontrolcü: Ego → İmmersif → Kapalı → Ego | sunucu görünümü değişti; Kapalı'da 0 Hz, Ego'da 30 Hz |
| **Durdur** | kontrolcü, tek tık | node KAPALI, SONIC kontrolü 0.1 s'de bıraktı |
| Robot görüntüsü | `/frames` + :5605 | 454 kare sim kamerasından (`sim_time` dolu), çözülüp sayfada gösterildi |
| Kayıt | run log | `start, set, calibrate, pause, resume, pause, stop` tick'lerde; `tools/fullbody_replay.py` birebir |

Robot görüntüsünden bir kare (köprünün çıkışı): zemin, robotun kendi elleri, sol üstte saat.

Simsiz test (`test_panel_drives_the_node`): kontrolcüyle Hız, elle Parmaklar, elle Durdur; node her birini uygular,
log'a yazar, replay birebir. P3'ün emülatör testleri (`test_quest_e2e.py`, 3/3) yeni bağlantı kuralıyla da geçiyor.

**Bir koşuda robot düştü (P5 dışı, risk):** Başlat'tan ~1.5 s sonra, bant bırakılınca. Bant pelvisi 0.965 m'de tutuyor,
ayakta 0.786 m: robot 18 cm yüksekten ayaklarının üstüne düşüyor. Ayakta kalan koşularda iniş anında eğim 16°'ye
çıktı. Node bu sırada aynı komutları gönderiyordu (STANDBY: IDLE, VR kapalı); SONIC zamanlaması normaldi (gözlem→komut
0.5 ms). Upstream düşme reset'i robotu ~20 s sonra ayağa dikti. Bandı bırakmadan önce indirmek `sim/gantry.py`'nin
(P1) işi.

## 6. Quest ekran görüntüsü

`adb screencap` VR katmanlarını siyah veriyor: yalnız SurfaceFlinger'ın 2D yüzeylerini okuyor, immersive WebXR oturumu
VR compositor'da birleşiyor. `deploy/quest_adb.sh shot` artık Meta'nın kendi yakalama servisini kullanıyor (Quest'in
ekran görüntüsü tuşunun yaptığı):

```bash
adb shell am startservice -n com.oculus.metacam/.capture.CaptureService -a TAKE_SCREENSHOT
# yeni dosya /sdcard/Oculus/Screenshots/ altında; script bekler ve çeker, yoksa screencap'e düşer
```

Bu gece denenmedi (headset uyuyor). Çalışmazsa sırayla denenecekler:
1. `adb shell am startforegroundservice -n com.oculus.metacam/.capture.CaptureService -a TAKE_SCREENSHOT`
   (yeni Horizon OS sürümleri arka plan servis başlatmayı kısıtlıyor).
2. Video: aynı servis, `-a START_CAPTURE` / `-a STOP_CAPTURE`; dosya `/sdcard/Oculus/VideoShots/`.
3. `scrcpy` (Quest'te çalıştığı biliniyor; iki göz yan yana gelir, `--crop` ile tek göz).
4. Tarayıcının kendisi: sayfaya enjekte edilen bir script XR karesinde `gl.readPixels` ile kendi katmanını okuyup
   sunucuya gönderebilir. Yalnız bizim çizdiğimizi yakalar (passthrough yok), ama sim ve panel için yeter.

## 6a. Bulgu: uyuyan headset akışı çalıyordu

Uyuyan Quest'in sayfası sunucuyu ~5 s'de bir arıyor (pasif `tcpdump`: 192.168.1.106 → :443 SYN). "En son bağlanan
besler" kuralıyla sunucu açılınca uyuyan headset bağlanıp akışı ele geçiriyor, son kafa pozunu sürekli tekrarlıyordu
(XR karesi yok, Vuer son matrisi yeniden gönderir): node `live` olamıyordu. Kural değişti (`connections.py`): VR'da
render eden, yani farklı kafa örnekleri gönderen bağlantı besler; o canlıyken başkası devralamaz, sustuğunda canlı
olan devralır. Aynı headset'in Wi-Fi kopması sonrası yarım açık soketi de böylece kendiliğinden bırakılır.
`xr_frame.session` artık bağlantıda değil, bir bağlantı beslemeye başladığında artar.

## 7. PLAN'dan sapmalar

1. **Panel dik, göze eğik değil.** Göğüs hizasından göze eğik bir panelin alt kısmı operatöre yaklaşıyor; bel
   hizasında tutulan eller düzlemin arkasında kalıyor ve ışın yüzeye ulaşamıyor (emülatörde görüldü).
2. **Menü butonu yerine thumbstick basma**: Quest Browser ≡ tuşunu WebXR'a vermiyor.
3. **El ışını**: "işaret parmağı ışını" yerine Quest'in kendi modeli, omuzdan işaret parmağı köküne. Parmak ucu
   yönü pinch sırasında kayıyor; kök sabit kalıyor.
4. **Panel açıkken kollar o anki hedefte kalmaz, PAUSED gibi 1.5 s'de nötre yumuşar** (görev "PAUSE" dedi; supervisor
   PAUSED'da bunu yapıyor).
5. **Basılı tutma**: sekmeler ve X hemen çalışır (robota bir şey gitmez); geri kalan her şey 0.6 s.
6. **Kayıt düğmesi kaydı açıp kapatmaz** (node her şeyi zaten kaydediyor; parçalı kayıt replay'i bozardı), bölüm
   işaretler. Bunun için `UiKind.MARK` eklendi; node'da kalır, kaynağa gitmez.
7. **Panel mesafesi ayarı yok** (Görüntü sayfası yalnız modlar).
8. **PLAN §3'teki `POST :8771/state`** yerine node durumunu ZMQ PUB :5595 ile yayınlar; sunucu `/state`'i GET olarak
   sunar (panel ve testler için).
9. Supervisor: otomatik (bayat kafa) duraklama sırasında gelen PAUSE duraklamayı operatöre geçirir; panel açıkken geri
   gelen kafa robotu kendiliğinden yürütmesin diye.
10. Emülatör GPU'da (`--gl gl-egl`) koşuyor: yazılım GL'de robot görüntüsü açıkken XR kare hızı ~2/s'ye düşüyordu.

## 8. Eşleme tarafıyla sözleşme (P4/P6/P7)

- `fullbody/settings.py`: `OperatorSettings`, `KNOBS`, `UiAction`; P5 `UiKind.MARK`'ı ekledi (node'da kalır).
- `fullbody/types.py`: `OperatorInput.status` (P5): kaynak `mode` ("walk"/"hands"), `heading_lock` (geçerli kilit),
  `calibrating` doldurur; node değiştirmeden yayınlar ve log'a yazar. Panelin "Mod" ve şeridin "KİLİT"i buradan.
- `quest_source.py` panel eylemlerini olaya çevirir (CALIBRATE → kalibre + ENGAGE, START/STOP/PAUSE/RESUME aynen);
  SET'i node uygular, kaynak `OperatorData.settings`'i okur.
- Hareket sayfasındaki "Yön kilidi" satırı **ayarı** gösterir; B/Y ya da çömelme kilidi değiştirdiyse geçerli kilit
  şeritte ve Robot sayfasında (`status.heading_lock`) görünür. Kaynağın kuralı "son yazan kazanır": düğmeye basmak
  ayarı değiştirir ve kilit ona uyar.

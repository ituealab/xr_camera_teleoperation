# P1 — Simülasyonu ayağa kaldırma: kurulum, ölçümler, bulgular

Tarih: 2026-11-12. Makine: MSI Raider 16 (Core Ultra 9 290HX, RTX 5090 Laptop, sürücü 580). SONIC kaynağı
`a0732b6`, prebuilt `g1_deploy_onnx_ref`, MuJoCo 3.15.0. Tüm ölçümler bu belgedeki araçlarla tekrar üretilebilir;
ham sonuçlar `data/sonic-p1/*.json` (gitignore).

## 1. Çalıştırma

Bir kerelik kurulum (uv PATH'te olmalı):

```bash
deploy/setup_sonic_sim_env.sh                                   # data/sonic-sim-venv (py3.10, torch yok)
data/xr-env/bin/python -m xr_camera_teleop.sonic.assets verify  # kaynak commit'i + tüm sha256'lar
```

Sim + SONIC (tmux, oturum başına bir süreç, loglar `~/.local/state/xr_fullbody/`):

```bash
deploy/xr_fullbody_sim.sh start            # [--no-viewer] [--no-cameras]; robot elastik bantta asılı başlar
data/sonic-sim-venv/bin/python -m xr_camera_teleop.tools.sonic_drive start   # start + planner, bandı bırak
deploy/xr_fullbody_sim.sh stop
```

`sonic_drive start` SONIC'e `command{start, planner}` gönderir, telemetri (`g1_debug`, :5557) gelince bandı sim'in
tuş ucundan (`9`) bırakır; robot IDLE'da ayakta kalır. Diğer senaryolar: `speeds`, `heights`, `hands`, `vr`,
`endurance --minutes 10`. Elle: `deploy/xr_fullbody_sim.sh key 9|backspace|up|...` (sim tuşları),
`deploy/xr_fullbody_sim.sh attach sonic` + `o` (SONIC acil durdurma).

Ölçüm: `data/sonic-sim-venv/bin/python -m xr_camera_teleop.tools.sim_perf --duration 60` (§7a hedefleri, PASS/FAIL).

Upstream klavye girişi de denendi (`--input-type keyboard`, tmux send-keys ile `]`, sim `9`, `Enter`, `1`, `w`,
`q`, `n`, `-`): başlatma, yürüme, yerinde dönüş, squat set'i ve yükseklik çalışıyor. Not: planner modunda başlangıç
modu IDLE'dır, `w`'den önce `1` gerekir; klavyenin SLOW_WALK hızı 0.2 m/s'ye kırpılır.

## 2. Yapı (ne neyin işini yapar)

| Parça | Görev |
|---|---|
| `configs/sonic_assets.yaml` + `sonic/assets.py` | Upstream kaynak, modeller, planner, binary, lib'ler, referans klipler: kök + yol + commit/sha256. `verify`, `path`, `dds-domain`. Planner yolu, binary'nin `find("V0")→"V1"→"V2"` sırasıyla kontrol edilir. |
| `deploy/setup_sonic_sim_env.sh` + `envs/sonic-sim.freeze.txt` | venv; SONIC kaynağı `.pth` ile (upstream ağaca hiçbir şey yazılmaz). |
| `configs/sim_courses.yaml` + `sim/scene_builder.py` | Upstream `scene_43dof.xml`'i MjSpec ile yükler; ZED çifti, render ayarları, parkur kutuları → `data/sim/scenes/<parkur>.xml`. |
| `configs/sonic_sim.yaml` | Sim/SONIC çalışma ayarları: dt, bant, tuş ucu, viewer/kamera hızları, portlar, istatistik ucu. DDS domain'i burada değil, binary'nin (manifest). |
| `sim/run_sonic_sim.py` | Upstream `SimLoopConfig` + `SimulatorFactory` ile `DefaultEnv`'i kurar; fizik döngüsü bizim (mutlak deadline). Her adımda durum anlık görüntüsü. |
| `sim/state_snapshot.py` | Fizik → render süreçleri, kilitsiz (seqlock, paylaşımlı bellek). |
| `sim/camera_publisher.py` | Alt süreç: EGL ile ZED çifti, upstream `ImageUtils` formatında :5555. |
| `sim/sim_viewer.py` | Alt süreç: pasif MuJoCo viewer, anlık görüntüden 60 Hz; pencere tuşları sim'e. |
| `sim/perf.py`, `tools/sim_perf.py` | Süreçlerin döngü zamanları → istatistik ucu; araç toplar, kamera akışını ve `rt/lowstate`'i dışarıdan ölçer. |
| `sonic/run_deploy.py` | Prebuilt binary'yi pinli varlıklarla `exec` eder (upstream `deploy.sh` argüman sırası). |
| `sonic/protocol.py` | zmq_manager mesajları; upstream `zmq_planner_sender.py` ile bayt bayt aynı (golden test). P2'nin `protocol.py`'si budur. |
| `tools/sonic_drive.py` | Senaryolu sürüş + ölçüm (DDS + telemetri), düşme tespiti. |
| `deploy/xr_fullbody_sim.sh` | Süreç grafiğinin launcher'ı; şimdilik sim + sonic. |

Upstream'in `BaseSimulator.start()` döngüsü kullanılmıyor: kamera render'ı ve `viewer.sync()` fizik adımının içinde,
uyku göreli (`sleep(dt - elapsed)`), dolayısıyla RTF < 1 ve adım süresi render'la sıçrıyor. Fizik, PD, bant, düşme
reset'i ve DDS köprüsü upstream'in aynısı.

## 3. Performans (§7a)

`tools/sim_perf.py`, SONIC ayakta/yürürken, makine sakinken (başka ağır süreç yok):

| Hedef | Ölçülen | Durum |
|---|---|---|
| RTF ≥ 1.0 | 1.000 (iç döngü ve `rt/lowstate` tick'i/duvar saati, ikisi de); `lag_resets` 0 | geçer |
| Adım p99 < 2.5 ms | yürürken 1.38–1.41 ms (en kötü 1 s pencere), pencere medyanı 1.1 ms, en kötü adım 1.9 ms; ortalama 0.8 ms | geçer |
| `rt/lowstate` 200 Hz | 200.0 Hz, varış aralığı p99 5.3 ms | geçer |
| Robot kamerası 30 fps, 2×960×540 | 30.0 fps her göz, düşen kare 0; durum→gönderim p50 6.0 / p99 8.9 ms, render+kodlama p99 7.1 ms (§3a, 145 W) | geçer |
| Viewer ≥ 60 fps | senkron döngüsü 60.0 Hz (Xvfb üzerinde, yazılım GL); fizik etkilenmedi (p99 1.2 ms) | gözle doğrulanmalı (§6) |

- Adımın kendisi ucuz: `mj_step` 0.08 ms. Asıl maliyet upstream'in Python PD döngüsü ve DDS yayını (~0.7 ms).
- SONIC `rt/lowcmd` ve iki `rt/dex3/*/cmd`'yi 500'er Hz yayınlar; upstream köprüsü bunları Python dinleyici
  thread'lerinde çözer. GIL geçiş aralığı 0.5 ms'ye indirildi (`physics.gil_switch_interval_s`); A/B (2×2 tur):
  pencere medyanı p99 1.40 → 1.09 ms, en kötü adım 2.3 → 1.9 ms.
- **Makine yüklüyken** (başka bir kullanıcının headless Chrome'u %1000 CPU, CPU 100–106 °C) 1 s pencerelerde
  p99 2.5–4.3 ms, tek adımlar 7 ms'ye kadar görüldü; RTF yine 1.000 ve SONIC düşmedi. Hedef sakin makinede tutuyor.
- Render: robot çarpışma mesh'leri (görsellerle aynı ~620 bin üçgen) geom grubu 3'e alındı, gölge haritası 2048.
  960×540 göz başına 3.1 ms (gölge 4096 + çarpışma mesh'leri görünürken 4.7 ms).

### 3a. dGPU 20 W'a kilitliydi (çözüldü: uykudan dönüş hatası)

**Neden (2026-11-12 akşamı bulundu):** s2idle uykusundan dönüş başarısız olmuş; çekirdek günlüğünde
`NVRM: rm_power_source_change_event: Failed to handle Power Source change event` ve `RmHandleDNotifierEvent: Failed
to handle ACPI D-Notifier event`. GPU pil seviyesindeki 20 W bütçesinde kalmış. Yeniden başlatma düzeltti: enforced
limit 145–150 W. Tekrarlamasın diye sim açılırken GPU ön kontrolü yapıyor (`sim/gpu.py`): enforced limit < 50 W ise
yüksek sesle uyarır (neden, çözüm: yeniden başlat ya da şarj aletini çıkar-tak; günlük okunabiliyorsa NVRM hatalarını da
belirtir), `sim_perf` o durumda numaraları basar ama geçti/kaldı demez (çıkış kodu 2). Not: `emin` çekirdek günlüğünü
okuyamıyor (`systemd-journal` grubunda değil); uyarı bunu söylüyor.

**Kameralar açık, 145 W (yeniden başlatmadan sonra), `sim_perf --duration 90` yürürken (`sonic_drive endurance`):**

| Ölçüm | 20 W kilitliyken | 145 W |
|---|---|---|
| SONIC politika çıkarımı (medyan / p99) | ~8 ms | 0.23 / 2.9 ms |
| SONIC gözlem→motor komutu (medyan / p99 / maks) | 28 ms (kameralar açık), 11 ms (kapalı) | 0.59 / 3.2 / 3.4 ms (bütçe 20 ms) |
| Planner modeli (10 Hz thread) | 63 ms | 2.9 ms |
| GPU sürekli saat / güç | P8, 180 MHz | 1740 MHz medyan (1725–1762), 31 W, kullanım %4, throttle nedeni yok (`0x0`) |
| Fizik RTF / adım p99 | — | 1.000 / en kötü pencere 1.75 ms, lag reset 0 |
| Kamera | düşmeler | 30.0 fps her göz, düşen kare 0 |

Bu yükte GPU kısa 2.7 GHz patlamalarından sonra ~1.6–1.7 GHz'de oturuyor; gözlem→komut p99 3.2 ms olduğuna göre bu
saat SONIC için sorun değil. Kameralar açık 3 dk dayanıklılık (`sonic_drive endurance --minutes 3 --tag cameras-145w`):
4 tur, **0 düşme**. Ham sonuçlar `data/sonic-p1/perf-cameras-145w.json`, `endurance-cameras-145w.json`.

Aşağıdaki paragraf kilitliyken yazıldı (tarihçe):


`nvidia-smi -q -d POWER`: *Current/Requested Power Limit 20 W* (varsayılan 95, max 175), *SW Power Cap: Active*,
GPU P8 / 180 MHz'de takılı. `nvidia-smi -pl` bu laptopta desteklenmiyor (denendi, değişmedi); sınırı platform
(EC / nvidia-powerd Dynamic Boost) koyuyor. Aynı sırada CPU paketi hafif yükte 94–100 °C.

Sonuç: SONIC'in politika çıkarımı ~1 ms yerine 8 ms, gözlem→komut 11 ms (bütçe 20 ms); planner modeli 63 ms
(10 Hz thread'de, tolere ediliyor). **Kameralar da açıkken** GPU paylaşılıyor: gözlem→komut medyan 28 ms, robot
ayakta bile duramadı (60 s'de 3 düşme). Kameralar kapalıyken her şey çalışıyor (§4, 10 dk dayanıklılık). Yani §7a'nın "kameralar
açıkken" hedefi bu donanım durumunda karşılanamıyor; sim tarafı değil, GPU gücü darboğaz. Kullanıcının kontrol
etmesi gerekenler: orijinal (yüksek watt) adaptör mü takılı, kapak açık ve fanlar çalışıyor mu, MSI'ın performans
modu. Düzelince `deploy/xr_fullbody_sim.sh start` (kameralar açık) ile `sim_perf` tekrar koşulmalı.

## 4. SONIC ölçümleri (P2/P4/P7 için)

### DDS domain

Binary **42**'de (manifest `deploy.dds_domain: 42`, sha256 `d194e7fe…`). Doğrulama: sim sadece domain 42'de yayın
yapıyor; binary `rt/lowstate`'i aldı (INIT'te "LowState is not available" yok) ve robot SONIC'in `rt/lowcmd`'siyle
ayağa kalktı/yürüdü. Sim domain'i manifestten okur; binary değişmedi.

### Planner modlarının hızları

`sonic_drive speeds`: 6 s komut, son 3 s'deki yer değiştirme/süre (odometri). İleri:

| Komut `speed` (m/s) | -1 | 0.1 | 0.2 | 0.3 | 0.5 | 0.7 | 0.8 | 1.0 | 1.2 | 1.5 | 2.0 | 2.5 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| SLOW_WALK (1) | 0.17 | 0.00 | 0.10 | 0.18 | 0.46 | 0.61 | 0.69 | 0.88 | 1.04 | | | |
| WALK (2) | 0.88 | | | | 0.44 | | 0.72 | 0.85 | 1.04 | 1.22 | 1.48 | 2.08 |

- Yana (SLOW_WALK, facing sabit): 0.2 → 0.02, 0.3 → 0.10, 0.4 → 0.37, 0.6 → 0.59 m/s.
- Geri: SLOW_WALK 0.2 → 0.04, 0.4 → 0.18, 0.6 → 0.51; WALK 0.8 → 0.71, 1.0 → 0.92 m/s.
- **Ölü bölge:** ~0.3 m/s altı komutlar neredeyse hareket üretmiyor (planner kısa sürede hızlanmıyor). P4'te
  yürüme eşiği ve en düşük hız buna göre seçilmeli (≥ 0.35–0.4 m/s).
- SLOW_WALK ve WALK 0.5–1.2 m/s'de aynı hızı veriyor; fark yürüyüş stili. Varsayılanlar: SLOW_WALK 0.17, WALK 0.88.
- RUN (3) ölçülmedi (kabul senaryosunda yok; `--run` ile ölçülebilir).
- **Durma mesafesi** (IDLE'a geçince): SLOW_WALK 0.5 → 0.25 m, WALK 1.0 → 0.48 m, WALK 1.5 → 0.87 m. §0 madde 3'ün
  "0.3 m içinde durur" şartı yalnız ~0.5 m/s'ye kadar sağlanıyor; P4'te `v_fwd_max` buna göre (ya da şart gevşetilir).

### Squat / kneel yükseklikleri

`sonic_drive heights`: 6 s, son 3 s'nin pelvis z ortalaması.

| Komut `height` | 0.8 | 0.7 | 0.6 | 0.5 | 0.4 | 0.3 | 0.2 |
|---|---|---|---|---|---|---|---|
| SQUAT (4), pelvis z (m) | 0.785 | 0.722 | 0.656 | 0.561 | 0.501 | 0.437 | 0.427 |
| KNEEL_TWO_LEGS (5) | 0.41 (height yok sayılıyor) | | | | | | |
| KNEEL one leg (6) | 0.43–0.46 (height yok sayılıyor) | | | | | | |

- Ayakta pelvis 0.786 m. SQUAT kullanılabilir aralık `height` 0.3–0.8 (pelvis 0.44–0.79); 0.3 altı doyuyor.
- zmq_manager squat modlarında `height < 0.2`'yi 0.2'ye çeker: `-1` "varsayılan" değil, en alçak squat demektir.
- Squat'a giriş 1.2–1.3 s'de oturuyor; kalkış (IDLE) sorunsuz, eğilme (up_z) ≥ 0.94.
- Squat'tan IDLE'a geçişte ilk anlarda pelvis 0.44 m'de: düşme tespiti yüksekliğe değil eğime bakmalı.

### Dex3 el modeli

`sonic_drive hands`: 2.5 s'lik aç/kapa döngüleri.

- Kollar öne (VR 3-nokta ile bilekler gövde önünde): iki el de hedefi 0.034 rad içinde izliyor, durağan |dq| 0.005
  rad/s. Titreşim, patlama ya da NaN yok. Upstream'in "el modeli kararsız" uyarısı MuJoCo 3.15'te görülmedi;
  sahne üreticide kontak/sönüm ayarı **gerekmedi**.
- Kollar SONIC'in IDLE pozunda (eller kalça yanında): sağ işaret parmağı kalçaya (`right_hip_roll_link`) takılıp
  açılamıyor (1.2–1.6 rad hata), sol başparmak 0.17 rad. Bu el modeli değil, el-kalça kontağı. P6'nın park pozu
  elleri kalçadan uzak tutmalı.
- Planner mesajında el alanı yoksa SONIC elleri **yumruk** yapar (`input_interface.hpp` varsayılanı). D4 gereği her
  mesajda `left/right_hand_joints` gönderilmeli.
- **thumb_0 sıfırı** (`sim/dex3.py`): gerçek ellerimizin thumb_0 açısı = URDF/MJCF açısı − 1.10 rad (iki el de).
  Gerçek elde öğretilen pozlar (`robot/thumb_shaping.py`: açık −1.15/−1.08, işaret kıstırma −1.77…−1.99, orta
  kıstırma −0.37, yumruk −1.13) URDF'in ±1.047'sinin çoğunlukla dışında; upstream MJCF bunları −1.047'de kırpıp
  başparmağı ~60° işaret parmağı tarafına çeviriyordu, büküm başparmağı yana savuruyordu. Sahne üretici thumb_0'a
  `ref = −1.10` verip aralığını kaydırır: qpos, PD ve `rt/dex3/*/state` gerçek elin biriminde; gönderilen komut ve
  upstream değişmez. Not: SONIC deploy (`dex3_hands.hpp` `clipToMaxOpen`) thumb_0'ı [−1.05, 1.05]'e kırpar, yani
  SONIC yolunda işaret kıstırmanın −1.99'u gerçek robotta da −1.05 (başparmak ortada) olur.

### 10 dk dayanıklılık (çıkış kriteri)

`sonic_drive endurance --minutes 10`: döngü = WALK 1.0 ileri 5 s, 15°'lik adımlarla +90° dönüş, SLOW_WALK 0.5,
yana sol/sağ 0.3, geri 0.4, squat 0.5 (4 s) ve kalkış, yürürken 4×15° dönüş, −90° dönüş. Düşme = pelvis eğimi > 60°
veya pelvis < 0.2 m veya sim reset'i.

| Koşu | Süre | Tur | Düşme | Sim reset | Fizik (eş zamanlı `sim_perf`, 540 s) |
|---|---|---|---|---|---|
| son ayarlar, kameralar kapalı | 607 s | 13 | **0** | 0 | RTF 1.000, adım p99 en kötü pencere 1.67 ms (medyan 1.07), en kötü adım 2.25 ms, lag reset 0, `rt/lowstate` 200 Hz |
| ilk koşu (GIL aralığı 5 ms) | 607 s | 13 | **0** | 0 | 120 s ölçüm: p99 1.72 ms |

Çıkış kriteri (10 dk yürü/dön/squat, düşme yok) iki kez geçti; ikisi de kameralar kapalı. GPU kilidi kalktıktan sonra
kameralar açık 3 dk koşu da düşmesiz (§3a); 10 dk'lık kameralar açık koşu: `deploy/xr_fullbody_sim.sh start` +
`sonic_drive endurance --minutes 10 --tag cameras`.

## 5. zmq_manager tel formatı ve davranışı (P2)

Kaynak: `zmq_manager.hpp`, `zmq_packed_message_subscriber.hpp` @ a0732b6; `sonic/protocol.py` golden testleri.

- SONIC SUB olarak `tcp://localhost:5556`'ya **bağlanır**; gönderen PUB'u **bind** eder. Üç konu aynı portta:
  `command`, `planner`, `pose`. Mesaj tek frame: `konu + 1280 bayt JSON başlık (NUL dolgulu) + alanlar`.
  Başlık `{"v":1,"endian":"le","count":1,"fields":[{name,dtype,shape}]}` (upstream Python docstring'indeki "1024"
  eskimiş; sabit 1280).
- `command`: `start`, `stop`, `planner` (u8, üçü zorunlu). start/stop kontrol döngüsü okuyana kadar OR'lanır.
  `delta_heading` zmq_manager tarafından okunmaz. **`stop` binary'yi kapatır** (süreç çıkar); yeniden başlatmak
  için süreç yeniden açılmalı.
- `planner`: `mode` (i32; dtype ne yazılırsa yazılsın 4 bayt int okunur), `movement`, `facing` (f32[3], binary
  normalize eder), `speed`, `height` (f32, -1 = varsayılan). Opsiyonel: `upper_body_position/velocity` [17],
  `left/right_hand_joints` [7, Dex3 motor sırası], `vr_position` [9], `vr_orientation` [12, wxyz], `vr_compliance`
  [3]. Statik modlarda speed -1'e zorlanır; squat modlarında height ≥ 0.2.
- Başlatma: planner mesajları akarken `command{start=1, planner=1}` → 0.2 s içinde telemetri (`g1_debug`, :5557,
  kontrol döngüsü her tick'te msgpack) gelmeye başlıyor; planner init ~140 ms. Telemetrinin gelmesi "SONIC kontrol
  ediyor" sinyali olarak kullanılabilir (P2'deki "planner init bekleme" sorusu).
- Planner timeout 1 s → IDLE, el ve üst gövde bayrakları temizlenir; **VR bayrağı temizlenmez** (gözlendi: sürücü VR
  açıkken kapandı, timeout oldu, VR bir sonraki VR'sız mesaja kadar açık kaldı). §6'daki kabul edilen risk doğrulandı.
- VR 3-nokta mesaj başına kenar tetikli: `vr_position` içeren ilk mesaj "VR 3-point control enabled" + encoder
  modu 1 (`teleop`), içermeyen ilk mesaj "disabled" + mod 0 (`g1`). 3 aç/kapa döngüsü düşmesiz.
- Politika uyumluluk (compliance) gözlemlemiyor: `vr_compliance` ve g/h/b/v tuşları bu modelde etkisiz.
- Binary'nin TensorRT motorları ONNX'lerin yanındaki önbellekten yükleniyor (hash eşleşti, yeniden derleme yok).

## 6. Gözle kontrol edilmesi gerekenler

- **Viewer**: `emin` mevcut ekrana pencere açamıyor (oturum aksoy-msi'nin GNOME Wayland'i; `DISPLAY` yok, X0 soketi
  emin'e yazılabilir değil). Viewer kendi Xvfb ekranımda (yazılım GL) açıldı, 60 Hz senkron, fizik etkilenmedi.
  GPU'lu gerçek ekranda akıcılık gözle bakılmalı: masaüstü oturumundan `deploy/xr_fullbody_sim.sh start` (ya da
  aksoy-msi oturumunda `xhost +SI:localuser:emin` sonrası `DISPLAY=:0`).
- Viewer ayrı süreçte: fare ile itme (Ctrl+sürükle) fiziğe gitmez; itme için ok tuşları.
- ZED görüntüsü: `zed_left` robotun orta hattında, 50° aşağı; eller ve önü görünüyor (render kontrol edildi).
  Gerçek montajla karşılaştırma P5'te.
- §3a'daki GPU güç sınırı çözüldü; ön kontrol her sim açılışında bakıyor.

## 7. PLAN'a düzeltmeler / yeni bilgiler

- D11 ile uyumlu: sim süreç grafiği tek kutu (`run_sonic_sim`), render alt süreçleri onun içinde.
- §7 başlatma: `SimulatorFactory.start_simulator` / `BaseSimulator.start` kullanılmıyor (yukarıdaki gerekçe);
  `SimLoopConfig.load_wbc_yaml` + `SimulatorFactory.create_simulator` kullanılıyor. `camera_configs` upstream'e
  verilmiyor; kameralar bizim alt sürecimizde, upstream'in görüntü formatıyla.
- §7 sahne: temel `g1_29dof_with_hand.xml` değil, onu içeren upstream sim sahnesi `scene_43dof.xml` (zemin, ışık,
  sürtünme upstream sim2sim ile aynı kalsın diye).
- §7 venv: `unitree_sdk2_python` editable değil, `.pth` ile (editable kurulum upstream ağaca egg-info yazardı);
  `pin`, `tyro`, `pygame`, `glfw` (mujoco zaten çekiyor), `loguru`, `termcolor` gerekmiyor (sim yolu import etmiyor).
- §4: kamera/viewer için `sim/camera_publisher.py`, `sim/sim_viewer.py`, `sim/state_snapshot.py`, `sim/perf.py`,
  `sim/sim_config.py`, `sonic/assets.py`, `sonic/run_deploy.py`, `tools/sonic_drive.py`, `configs/sonic_sim.yaml`
  eklendi. Sim'e ait portlar (:5555 kamera, :5598 istatistik, :5599 sim tuşları) `configs/sonic_sim.yaml`'da; SONIC
  portları da (:5556, :5557) orada, P2'nin `fullbody.yaml`'ı aynı değerleri buradan okumalı (tek kaynak).
- `sonic/protocol.py` P1'de yazıldı (sürücü gerektirdi); P2 bunu kullanır, golden test hazır.
- Kamera akışı upstream formatında JPEG + base64 (upstream `ImageUtils`, RGB dizisini BGR gibi kodlar). §4'teki
  "JPEG kodlama camera_bridge'de" yerine: sim kodlar, `camera_bridge` (P5) upstream `ImageUtils.decode_image` ile
  çözüp stereo formatına çevirir. Mesajda `sim_time` ve `state_wall_time` var (P5 gecikme ölçümü için).
- §0 madde 3 (0.3 m'de durma) ve P4 hız aralıkları için §4'teki ölçümler; squat `height` aralığı 0.3–0.8.
- §7a "kameralar açıkken" hedefi 20 W kilidiyle karşılanamıyordu; kilit uykudan dönüş hatasıydı, yeniden başlatmadan sonra
  karşılanıyor (§3a).

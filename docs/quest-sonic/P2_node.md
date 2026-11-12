# P2 — SONIC adaptörü ve başsız node: çalıştırma, yapı, ölçümler

Tarih: 2026-11-12. Makine: MSI Raider 16, GPU 145 W (P1 §3a: 20 W kilidi uykudan dönüş hatasıydı, yeniden
başlatmayla çözüldü). SONIC `a0732b6` prebuilt deploy, MuJoCo 3.15. Ölçümler kameralar **açık** yapıldı. Ham
sonuçlar: `data/fullbody-runs/<zaman>/{run.msgpack, meta.json, odom.msgpack, metrics.json, sim_test.json}`
(gitignore).

## 1. Çalıştırma

Tek komut: sim + SONIC + node (her biri kendi tmux oturumunda, loglar `~/.local/state/xr_fullbody/`):

```bash
deploy/xr_fullbody_sim.sh start [--no-viewer] [--no-cameras] -- --source script --scenario all      # script'li operatör
deploy/xr_fullbody_sim.sh start -- --source quest --xr-replay data/fullbody-runs/20261112-195840     # kayıtlı headset
deploy/xr_fullbody_sim.sh start -- --source quest                                                   # canlı Quest (:5596)
deploy/xr_fullbody_sim.sh node --source script --scenario circles   # sim/SONIC açıkken sadece node'u (yeniden) başlat
deploy/xr_fullbody_sim.sh status | logs | attach node | stop
```

- `--` sonrası `fullbody/node.py`'nin argümanlarıdır (`--help`). Node `data/xr-env`'de çalışır. Her koşu
  `data/fullbody-runs/<zaman>/`'a yazılır (`--run-dir`, `--no-log`).
- Senaryolar (`fullbody/scripted.py`): `square`, `turn`, `circles`, `bend`, `squat`, `all` (beşi art arda, 128.5 s).
  Hepsi 4 s'lik `settle` ile başlar; sonunda SONIC'e STOP gönderilir (`--keep-running` ile robot ayakta bırakılır).
- `--xr-replay RUN_DIR`: bir koşunun `xr_frame` kayıtları Quest sunucusu yerine gerçek zamanlı oynatılır. Zamanlar
  şimdiye kaydırılır, böylece bayatlık kuralları kayıttaki örnek yaşlarını görür. Kayıt bitince node çıkar.
- Sim'de STOP sonrası her şey kendiliğinden toparlanır: `run_deploy --respawn` binary'yi yeniden açar,
  `sim/gantry.py` bandı SONIC'in kontrol durumuna göre indirir/kaldırır. Yani koşular arka arkaya, elle müdahalesiz.

Ölçüm ve doğrulama:

```bash
data/sonic-sim-venv/bin/python -m xr_camera_teleop.tools.fullbody_sim_test all --repeat 3        # senaryo + ölçüm + replay
data/sonic-sim-venv/bin/python -m xr_camera_teleop.tools.fullbody_sim_test square --kill-at 6.5 --watch 30   # kill testi
data/sonic-sim-venv/bin/python -m xr_camera_teleop.tools.fullbody_metrics data/fullbody-runs/<zaman>
data/xr-env/bin/python -m xr_camera_teleop.tools.fullbody_replay data/fullbody-runs/<zaman>      # regresyon (simsiz)
XR_E2E=1 data/xr-env/bin/python -m pytest -q xr_camera_teleop/tests/test_quest_e2e.py             # emülatörlü uçtan uca
```

`fullbody_sim_test` sim venv'inde çalışır (pinli `unitree_sdk2py` `rt/odostate`'i biliyor; xr-env'deki eski kopya
bilmiyor), node'u xr-env ile alt süreç olarak açar.

## 2. Yapı: girdi kaynağı ara yüzü (P4/P6/P7 bunun arkasına takılır)

Bir tick (`NodeCore.step`, saf, 50 Hz):

```
OperatorData ─► InputSource.step(now, data, state) ─► OperatorInput ─┬─ START/STOP ─► sonic/lifecycle ─► command
 (xr_frame)        quest_source | scripted             live, ready,   │                    │ phase
                                                       events, intent └────► supervisor ─► FullBodyCommand ─► sonic/adapter ─► planner
```

**Ara yüz** (`fullbody/types.py`, değişmeyecek kısım):

```python
class InputSource(Protocol):
    def describe(self) -> dict                    # meta.json'a; replay kaynağı bundan yeniden kurar
    def step(self, now: float, data: OperatorData, state: State) -> OperatorInput   # saf, deterministik, I/O yok
    result: SourceResult | None                   # script bitince dolar; Quest için hep None

OperatorData(frame: XRFrame | None)               # o ana kadar gelen son xr_frame (P5 buraya UiAction ekler)
OperatorInput(live: bool, ready: bool, events: tuple[Event, ...], intent: Intent)
Event: START, STOP, ENGAGE, DISENGAGE, PAUSE, RESUME
Intent(base: BaseCmd | None,          # BaseCmd(mode IDLE|SLOW_WALK|WALK|SQUAT, velocity (vx, vy) m/s, height)
       heading: float | None,         # rad, ENGAGE anındaki facing'e göre (CCW +)
       torso, left_wrist, right_wrist: 4x4 | None,   # torso_link / *_wrist_yaw_link, pelvis frame'inde
       hands: Hands | None)           # Dex3 7+7, motor sırası
```

- `velocity` komut edilen facing frame'indedir (x ileri, y sol); dünyaya çevirmek adaptörün işi. Yürüme modu ⇔ hız
  sıfırdan farklı, SQUAT ⇔ `height` var (`BaseCmd` doğrular).
- `None` bırakılan parça "operatörden hedef yok" demektir. Supervisor doldurur: base IDLE, heading tutulur,
  gövde/bilekler `postures.neutral` (kollar önde, eller kalçadan uzak), eller açık.
- `live`: operatör girdisi canlı (Quest: bağlı ve kafa örneği ≤ 0.2 s, örnek zamanı `t`'ye göre). `ready`: kaynak
  hedef üretebilir (Quest: kalibrasyon bu `session`/`epoch` ile eşleşiyor). ACTIVE'de ikisinden biri düşerse PAUSED.
- Kaynak, bir önceki tick'in supervisor durumunu görür (ör. ENGAGE ancak STANDBY'de anlamlı).

**Quest kaynağı** (`fullbody/quest_source.py`): P2'de sadece canlılık kuralı var; `ready` hep False, yani robot
STANDBY'de kalır. P4 buraya kalibrasyonu (buton ~1 s → `calibrate` → ENGAGE), buton jestlerini ve `heading.py` +
`locomotion.py`'yi; P6 `arms.py` + `hands.py`'yi; P7 `posture.py`'yi ekler. Her modül Intent'in kendi parçasını
doldurur; node, supervisor ve adapter değişmez. Kalibrasyon ve butonlar kaynağa aittir (operatör tarafı), supervisor
cihaz bilmez.

**Supervisor** (`fullbody/supervisor.py`): OFF (SONIC kontrol etmiyor) → STANDBY (SONIC ayakta tutuyor: IDLE, VR
kapalı, eller açık) → ENGAGE → ACTIVE ⇄ PAUSED; DISENGAGE → STANDBY; SONIC düşerse → OFF.
- ACTIVE'e girişte üst gövde bulunduğu yerden (STANDBY'den: `postures.idle`, SONIC'in kendi duruşu) hedefe 1.0 s'de
  yarı-kosinüsle geçer; VR açılması zıplamaz.
- PAUSED'da IDLE, facing donuk, üst gövde 1.5 s'de nötre yumuşar.
- Bayat kafa (`live` False) ile girilen PAUSED, girdi 0.5 s canlı ve hazır kalınca kendiliğinden ACTIVE'e döner
  (PLAN §6, `supervisor.auto_resume_s`). Buton, ya da `ready` düşmesi (recenter, yeni oturum) ile girilen PAUSED
  RESUME/ENGAGE ister. Arada `ready` düşerse otomatik dönüş iptal olur.
- Heading ENGAGE anında çapalanır (`facing = çapa + intent.heading`): devralma ya da devam robotu döndürmez. SONIC her
  start'ta planner yönünü robotun o anki yaw'ına sıfırladığı için facing de start'ta 0'a döner.

**Adapter** (`sonic/adapter.py`), SONIC kodlamasının tek sahibi:
- Mod numaraları; facing frame'indeki hızın planner dünyasına çevrilip normalize edilmesi; `speed`/`height` -1
  kuralları; squat'ta her zaman açık yükseklik.
- VR noktaları: link pozu + upstream ofsetleri (bilek `[0.18, ∓0.025, 0]`, gövde `[0, 0, 0.35]`), wxyz, sıra sol,
  sağ, gövde.
- El alanları her mesajda.

**Kayıt** (P3 formatı, `fullbody/runlog.py`):
- Tick başına `tick`: kaynağa ait olmayan girdiler (telemetri alım zamanları, node'un eklediği olaylar) ve çıktılar
  (durum, faz, telde giden planner alanları, command'lar, notlar).
- Kullanılan her yeni `xr_frame`.
- Her yeni telemetri örneği için `robot`: eklemler, IMU, eller, SONIC'in gördüğü VR hedefleri.
- `meta.json` kaynak tanımını ve etkin config'leri tutar. `tools/fullbody_replay.py` node'u bunlardan kurar ve her
  tick'i birebir karşılaştırır.

**Çıkış davranışı:** script bitince ya da SIGINT/SIGTERM'de, SONIC duruyorsa durana kadar tick'ler sürer. Aksi halde
DISENGAGE → STANDBY 0.5 s, sonra çıkar; SONIC ayakta kalır. Node SONIC'i kendiliğinden durdurmaz, STOP operatörün
(ya da script'in) açık isteğidir.

## 3. Lifecycle (`sonic/lifecycle.py`)

| Faz | Ne olur | Ölçülen |
|---|---|---|
| STARTING | `command{start, planner}` her 0.2 s, `g1_debug` gelene kadar (binary kapalıyken giden mesaj kaybolur, tekrar edilir) | start → kontrol 0.20–0.34 s |
| ARMING | Kontrol görüldü; supervisor OFF'ta tutulur, sadece VR'sız mesajlar, 0.5 s | SONIC logu: "planner_motion" sonra "VR 3-point control enabled" |
| RUNNING | VR serbest; `g1_debug` 0.3 s susarsa → OFF ("SONIC stopped controlling") | — |
| STOPPING | `stop` her 0.1 s, telemetri susana kadar | 0.32 s |
| OFF + SONIC zaten kontrol ediyor | (node, çalışan bir SONIC'in altında yeniden açıldı) → ARMING: eski VR bayrağı VR'sız mesajla sıfırlanır | — |

- "Binary ayakta, bekliyor" `robot_config` konusundan anlaşılır (INIT ve WAIT_FOR_CONTROL'de ~2 s'de bir). Durum
  `absent | waiting | controlling` olarak log'a ve konsola yazılır.
- `stop` binary'yi kapatır. stop → yeniden açılan binary'nin kontrolü alması **~28 s** sürdü (INIT + TensorRT
  yükleme), bu yüzden start timeout'u 60 s. Sim'de yeniden açma `run_deploy --respawn`'ın işidir. Gerçek robotta
  binary açılışta tüm eklemleri varsayılan poza sürdüğü için yeniden açma açık bir operatör eylemi kalmalı (§12).

## 4. Ölçümler

### Tekrarlanabilirlik: `all` × 3 (kameralar açık, her koşu STOP → yeniden açma → START ile başlıyor)

Üç koşu da `OK`, düşme yok (zaman çizelgesi boyunca en küçük up_z 0.973, en alçak pelvis 0.475 m squat'ta), replay
3/3 birebir. Segment başına sonuçlar koşular arasında birkaç mm/derece içinde:

| Ölçüm | Koşu 1 | Koşu 2 | Koşu 3 |
|---|---|---|---|
| Bilek dairesi r=8 cm T=4 s, konum medyan L/R (cm) | 1.42 / 1.29 | 1.39 / 1.20 | 1.50 / 1.28 |
| aynı, p95 L/R (cm) | 2.41 / 2.64 | 2.46 / 2.16 | 2.50 / 2.31 |
| aynı, rotasyon medyan L/R (°) | 5.0 / 5.6 | 4.6 / 4.2 | 5.0 / 5.2 |
| Bilek dairesi r=8 cm T=2 s, konum medyan L/R (cm) | 1.78 / 1.71 | 1.51 / 1.31 | 1.54 / 1.72 |
| aynı, p95 L/R (cm) | 2.99 / 3.47 | 3.10 / 2.97 | 2.91 / 2.86 |
| aynı, rotasyon medyan L/R (°) | 7.5 / 8.3 | 6.6 / 6.5 | 6.4 / 7.8 |
| En iyi gecikme (her iki daire) | 0.06 s | 0.06–0.08 s | 0.06 s |
| Squat pelvis z: 0.6 / 0.45 / 0.35 (m) | 0.654 / 0.543 / 0.485 | 0.664 / 0.546 / 0.486 | 0.660 / 0.543 / 0.484 |
| +90° dönüşlerde segment sonu hata (°) | −12.7 −8.6 −12.7 −8.5 | −11.6 −7.3 −11.9 −9.3 | −11.4 −9.2 −11.7 −10.3 |
| +180° / −180° segment sonu hata (°) | −0.2 / +17.1 | +0.1 / +15.3 | +0.2 / +15.0 |
| Gövde eğme pitch 20° / roll ±10°, rotasyon medyan (°) | 1.2 / 2.5, 1.0 | 1.3 / 3.3, 1.5 | 1.3 / 2.7, 1.1 |
| Gövde **yaw** ±30°, rotasyon medyan (°) | 26.0 / 23.3 | 25.6 / 22.8 | 26.2 / 23.3 |
| Kare: başlangıç ↔ bitiş (m) | 0.36 | 0.37 | 0.51 |

- **Bilek dairesi VR modunda simde görülüyor.** Hata: ileri kinematik (ölçülen eklemler → link → SONIC VR noktası) ile
  komut edilen nokta karşılaştırılıyor. Yürürken bilek hatası medyan 1.6–2.1 cm, ayakta dururken 0.2–1.4 cm. SONIC'e
  verilen VR oryantasyonları gönderilenle birebir aynı (kablolama kontrolü 0.0).
- **Dönüşler:** planner dönüşü komutun gerisinden tamamlıyor; segment sonunda 90°'de 8–13° eksik, sonraki segmentte
  kapanıyor (tüm zaman çizelgesinde net yaw +1.3…+3.5°). P4'ün "90° dönüşü ±5° izler" kapısı oturma süresiyle
  ölçülmeli.
- **Bulgu, P7 için:** gövde yaw hedefi (±30°) neredeyse izlenmiyor (~5° gerçekleşiyor). Pitch ve roll iyi izleniyor.
  P7'de bakışı bel yaw'ıyla izleme (§5 Heading, waist-yaw payı) bu modelde VR gövde noktasıyla çalışmayabilir. Ölçüp
  karar vermek gerek (gerekirse yaw facing'e kalır).
- **Kare:** kenarlar 1.22–1.47 m (0.6 m/s × 3 s), dönüş eksikleri yüzünden kapanmıyor (0.36–0.51 m). Senaryo
  tekrarlanabilirlik içindir, kapalı çevrim yörünge değildir.
- SONIC'in telemetride bildirdiği `vr_3point_position`, heading'e göre döndürülmüş bir görselleştirme frame'inde (90°
  dönüşten sonra 90° dönük); politikanın izlediği bizim pelvis-frame hedeflerimiz. Ölçüm araçları bunu kullanmıyor.

### Kill testi (SIGKILL, node yok, 30 s izlendi)

| Ne zaman öldürüldü | Sonuç |
|---|---|
| `square`'de SLOW_WALK 0.6 m/s yürürken | **Ayakta kaldı** (pelvis min 0.762, up_z ≥ 0.996). SONIC kontrolü sürdürdü (%100). 1 s planner timeout'undan sonra IDLE; robot öldürmeden 2.0 s sonra durdu, 0.57 m daha gitti. |
| `circles`'ta VR modunda | Ayakta kaldı, kıpırdamadı (1 cm). |

İkisinde de:
- **VR bayrağı açık kaldı**, kollar son hedefte durdu (stok SONIC timeout'u VR'ı temizlemiyor; §6'daki kabul edilen risk).
- **Eller ~1 s sonra yumruk oldu** (timeout el bayrağını temizliyor, SONIC varsayılanı yumruk: `[0, 0, 1.75, −1.57,
  −1.75, −1.57, −1.75]`).

PC2 için anlamı (§12):
- Node ölürse robot düşmüyor.
- Ama ~1 s daha son komutla yürüyor (0.6 m/s'de 0.57 m).
- Kollar son VR hedefinde donuyor, eller kapanıyor; elde bir nesne varsa sıkılır.

SONIC tarafı watchdog'u (VR'ı ve hareketi kısa sürede bırakan, elleri açık tutan) gerçek robottan önce gerekli.

### Kayıtlı headset oturumu (`20261112-195840`, 548 s, Quest 3S) node'dan

`--source quest --xr-replay` ile oynatıldı:
- 27 455 tick, gecikme yok (tick aralığı p99 20.6 ms). Replay birebir.
- Headset 271. saniyeden sonra canlı; canlı oranı %95.2.
- 13 "canlı değil" aralığı (kafa > 200 ms bayat): 4'ü uzun (1.9–4.1 s), diğerleri 0.14–0.3 s.

P4 için: her takılma bir PAUSED demek. Bayatlıkla girilen PAUSED artık 0.5 s taze kafadan sonra kendiliğinden
dönüyor (supervisor, PLAN §6), yani kısa takılmalar ~0.5–0.8 s'lik duraklamalara dönüşür. Uzunlar kalibrasyonu
bozmuyorsa aynı şekilde döner.

### Emülatör (headless Chrome + IWER, `XR_E2E=1`)

Gerçek Quest sunucusu → `xr_frame` → gerçek node süreci:
- Emülatör oynarken girdi canlı, ayrılınca değil.
- Kullanılan her kare loglanıyor, koşu replay'de birebir.
- SONIC başlatılmadı (P2'de Quest kaynağı START üretmez).

### Kameralar açık performans (P1 §3a'nın 145 W tekrarı)

| Ölçüm | Değer |
|---|---|
| SONIC politika (medyan / p99) | 0.23 / 2.9 ms (20 W'ta ~8 ms) |
| SONIC gözlem→komut (medyan / p99 / maks) | 0.59 / 3.2 / 3.4 ms (20 W'ta 28 ms), bütçe 20 ms |
| GPU sürekli | 1740 MHz medyan (1725–1762), 31 W, throttle nedeni yok |
| Fizik / kamera | RTF 1.000, adım p99 1.75 ms; 30.0 fps × 2, düşen kare 0 |
| 3 dk dayanıklılık | 0 düşme |

**GPU ön kontrolü** (`sim/gpu.py`):
- Sim her açılışta `nvidia-smi enforced.power.limit`'e bakıyor.
- < 50 W ise yüksek sesle uyarıyor (launcher da tekrarlar): neden (uykudan dönüşte bayat güç bütçesi), çözüm
  (yeniden başlat ya da şarj aletini çıkar-tak). Çekirdek günlüğünde NVRM D-Notifier hatası varsa onu da söylüyor.
  `emin` günlüğü okuyamıyorsa bunu da söylüyor; `systemd-journal` grubuna eklemek kullanıcının kararı.
- O durumda `sim_perf` geçti/kaldı demez (çıkış 2). `sim_perf` artık SONIC'in kendi Loop timing satırlarını
  (politika, gözlem→komut < 20 ms hedefi, planner) ve GPU saatini de raporlar.

### CPU (ORT) ile TensorRT

Prebuilt deploy'da CPU seçeneği yok:
- Kaynak `a0732b6` ve binary'nin argümanları incelendi: politika, encoder ve planner koşulsuz TensorRT.
- ORT sadece hiç kurulmayan `LocalMotionPlannerONNX` için bağlı.
- Var olan seçenekler `--policy-precision 16` / `--planner-precision 16` (FP16 TRT).

GPU 145 W'a döndükten sonra soru anlamını yitirdi (gözlem→komut p99 3.2 ms), ölçülmedi.

## 5. PLAN'dan sapmalar

1. `FullBodyCommand` SONIC dizileri (`vr_position[9]`…) değil, robot terimleri taşıyor: link pozları, facing
   frame'inde hız. Bütün SONIC kodlaması adaptörde. §5'teki "proxy çevirisinin tek sahibi adapter" kuralı böylece
   gerçekten tek yerde.
2. Yürüme: `locomotion.py` `movement = Rz(ψ_cmd)·v̂` hesaplamaz; facing frame'indeki hızı verir, adapter çevirir.
   Heading mutlak değil, ENGAGE'e göre bağıl (supervisor çapalar); `ψ_plan0` gerekmiyor.
3. Durumlar: PLAN'daki DISCONNECTED → `OFF` ve sadece SONIC'e bağlı. STANDBY Quest bağlantısı istemez. Quest
   kopması ACTIVE'de PAUSED'a götürür (senaryo 9'un istediği gibi robot ayakta). Buton ve kalibrasyon mantığı
   supervisor'da değil Quest kaynağında.
4. §6 Lifecycle 4: node kapanışta `stop` göndermez (robot düşmesin, öldürme ile kapanma aynı davransın); STOP açık
   istektir. Script'ler sonunda STOP gönderir.
5. Planner init bekleme: sabit gecikme değil, `g1_debug` + 0.5 s VR'sız ARMING.
6. Yeni modüller (§4'te yok):
   - `sonic/telemetry.py`, `fullbody/scripted.py`, `fullbody/quest_source.py`, `sim/gantry.py`, `sim/gpu.py`;
   - `tools/fullbody_sim_test.py`; `tools/fullbody_metrics.py` (PLAN'da adı vardı) ve `tools/fullbody_replay.py`;
   - `runlog.RecordFile` (yan kayıt dosyaları), `xr_frame.shifted`, `frames.interpolate_pose`/`rpy_pose`,
     `run_deploy --respawn`;
   - launcher'da `node` süreci.
7. Upstream hatasına bizim tarafta çözüm: upstream reset'i (`mj_resetData`) `xquat`'ı sıfır bırakıyor ve bant
   okuyunca sim çöküyordu. Döngü reset'ten sonra `mj_forward` çağırıyor; upstream'e dokunulmadı.

## 6. P4/P6/P7 için

- Sadece `fullbody/quest_source.py`'yi genişletin; node, supervisor ve adapter'a dokunmayın. Yeni saf modüller
  (`heading.py`, `locomotion.py`, …) Intent'in kendi alanını üretir.
- Kaynak saf kalmalı: zaman sadece `now`, girdi sadece `OperatorData` (+ önceki `State`). Replay buna dayanır.
  Kalibrasyon ve filtre durumu kaynağın içinde tutulabilir; aynı girdi dizisi aynı çıktıyı vermeli.
- Ayarlar `configs/fullbody.yaml`'da kendi bölümünde ve sıkı (`from_config` bilinmeyen anahtarı reddeder).
- Test girdileri:
  - `--xr-replay data/fullbody-runs/20261112-195840` (gerçek headset; stick, butonlar, recenter, eller içeriyor);
  - emülatör (`tools/quest_emulator.mjs`: kafa ±30°, çömelme, stick ileri, A, el↔kontrolcü, recenter);
  - sentetik `XRFrame`'lerle birim testi (`tests/test_fullbody_node.py` örneği).
- Hız ve durma: §0 notu, P1 §4. Yürüme komutunun 1 s timeout'ta 0.57 m sürdüğünü unutmayın.
- Gövde yaw'ı VR gövde noktasıyla izlenmiyor (§4); waist-yaw payını P7 ölçerek tasarlamalı.

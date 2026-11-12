# P4 / P6 / P7 — Operatör eşlemesi: yürüme ve yön, kollar ve eller, duruş

Tarih: 2026-11-12. Makine: MSI Raider 16 (GPU 145 W), SONIC `a0732b6` prebuilt deploy, MuJoCo 3.15, kameralar açık.
Kod: `xr_camera_teleop/fullbody/{quest_source,operator_mode,gestures,heading,locomotion,posture,arms,hands}.py`,
`operator/hand_geometry.py`; ayarlar `configs/fullbody.yaml` `quest:`. Ham sonuçlar
`data/fullbody-runs/<zaman>/{run.msgpack, odom.msgpack, metrics.json, mapping_metrics.json, replay.json}` (gitignore).

## 1. Çalıştırma

```bash
deploy/xr_fullbody_sim.sh start -- --source quest                     # canlı headset (yarınki deneme)
deploy/xr_fullbody_sim.sh start -- --source quest --xr-replay DIR     # kayıtlı ya da sentetik bir akış

# sentetik operatör (insansız doğrulama)
data/xr-env/bin/python -m xr_camera_teleop.tools.xr_synth --list                  # senaryolar
data/xr-env/bin/python -m xr_camera_teleop.tools.xr_synth p6_hands                # -> data/xr-synth/p6_hands-s0/
data/xr-env/bin/python -m xr_camera_teleop.tools.quest_offline data/xr-synth/p6_hands-s0    # simsiz, gerçek zamandan hızlı
data/sonic-sim-venv/bin/python -m xr_camera_teleop.tools.fullbody_sim_test --xr p4_turn90 p6_hands   # sim + ölçüm
data/sonic-sim-venv/bin/python -m xr_camera_teleop.tools.fullbody_sim_test --xr-run data/fullbody-runs/<kayıt>
data/sonic-sim-venv/bin/python -m xr_camera_teleop.tools.mapping_metrics data/fullbody-runs/<zaman>  # sonradan ölçüm

# d2bba3e öncesi kayıtlar (el işaretçisi kontrolcü gibi görünüyordu)
data/xr-env/bin/python -m xr_camera_teleop.tools.xr_frame_drop_pointers data/fullbody-runs/20261112-195840 [--start 265]
```

`fullbody_sim_test --xr` her sentetik akışı node'a `--source quest --xr-replay` ile verir: gerçek `QuestSource` →
node → SONIC → MuJoCo. Akış A tuşunu 1.3 s basılı tutarak kalibre eder, SONIC'i başlatır ve devralır; sonunda iki
grip + tetik ile STOP gönderir. Her koşudan sonra replay testi node'un venv'inde çalışır (Dex3 retarget'ı
dex-retargeting ister, sim venv'inde yok).

## 2. Yapı

Bir tick'te `QuestSource.step` (saf, deterministik; replay birebir):

```
OperatorData ─┬─ gestures.py ───────────┐  A/X 1 s → kalibre + devral; A/X kısa → duraklat/devam; B/Y → yön kilidi;
 frame         │ (kontrolcü butonları)   ├─► events   iki grip + tetik 0.5 s → STOP
 ui (panel) ───┤ panel UiAction ─────────┘            START/STOP/PAUSE/RESUME aynen; CALIBRATE → kalibre + devral
 settings      ├─ calibration.py (CalibrationWindow: son 1 s) → Calibration (ready)
               ├─ operator_mode.py → WALK (kontrolcü var) | HANDS (kontrolcü yok)          (D8)
               ├─ locomotion.py  stick (WALK) → BaseCmd yürüme / IDLE
               ├─ heading.py     kafa yaw → ψ_body (histerezis) → heading (kavrama, kilit, 3°, yeniden gönderme)
               ├─ posture.py     kafa pozu → boyun pivotu → eğilme → torso hedefi; çömelme → SQUAT + height;
               │                 operatör omuz frame'i ve bileklerin konduğu torso frame'i
               ├─ arms.py        eller (HANDS) → bilek hedefleri; WALK/kayıp → park; rampalar; güvenlik
               └─ hands.py       el eklemleri → Dex3 7+7 (kolun moduna göre)
                                              ▼
                                OperatorInput(live, ready, events, intent, status)
```

| Modül | Tek görevi |
|---|---|
| `quest_source.py` | Modülleri bağlamak; butonlar/panel → Event; kalibrasyon isteği, devralma sırası (OFF ise önce START); `status` |
| `operator_mode.py` | WALK/HANDS: taze kontrolcü → WALK, yalnız taze el → HANDS, hiçbiri → mod korunur; 0.1 s teyit |
| `gestures.py` | Kontrolcü buton jestleri (bas-tut, kısa basma, kenar, kombinasyon) |
| `heading.py` | Kafa yaw → operatör gövde yaw'ı, robot heading'i |
| `locomotion.py` | Stick → mod + facing frame'inde hız |
| `posture.py` | Kafa pozu → gövde eğimi, torso hedefi, operatör omuz frame'i, çömelme durumu |
| `arms.py` | Takip edilen el → pelvis frame'inde bilek linki; park/takip/tut; geometrik güvenlik |
| `hands.py` | El eklemleri → Dex3 motor sırası hedefleri |
| `operator/hand_geometry.py` | Kameralı wrapper ile ortak el geometrisi (bilek-yerel noktalar, Unitree el dönüşümü, ego yön ofseti) |
| `calibration.py` (+) | `R_head` (ortalama kafa rotasyonu) ve `CalibrationWindow` (son 1 s'nin kareleri) |

Node, supervisor, types, settings, sonic/* değişmedi. Ayarların hepsi `OperatorData.settings`'ten canlı okunur
(speed_scale, stick_deadzone, heading_lock, crouch_enabled, lean_gain, arm_tracking, fingers); yeni ayar eklenmedi.

## 3. Operatörün kontrolleri (D8)

- **YÜRÜME** (en az bir kontrolcü takipte): hangi kontrolcünün stick'i daha çok itilmişse o yürütür. İki kol park
  pozuna 1 s'de gider, parmaklar dinlenme pozunda. Gövde eğilmesi ve çömelme bu modda da çalışır.
- **ELLER** (hiç kontrolcü yok): takip edilen her el aynalanır, takip edilmeyen taraf 1 s tutulur sonra park eder,
  bacaklar IDLE. Buton yok; panel kullanılır.
- Geçişler: kontrolcü → el geçişinde Quest girdiyi ~0.6 s, el → kontrolcüde ~1.5 s durdurur. Kafa bayatlarsa
  PAUSED, kafa 0.5 s taze kalınca kendiliğinden ACTIVE (supervisor). Sim'de: 1.1–2.1 s sonra yeniden ACTIVE.

| Giriş | Ne olur |
|---|---|
| A veya X 1 s basılı (dik dur, ileri bak) | Son 1 s'nin karelerinden kalibrasyon; SONIC kapalıysa START, ayakta durunca ENGAGE; zaten ACTIVE ise PAUSE + ENGAGE (yön yeniden çapalanır, robot dönmez). Kalibrasyon reddedilirse buton basılıyken yeniden denenir. |
| A veya X kısa (< 0.5 s) | ACTIVE → PAUSE, PAUSED → RESUME |
| B veya Y | Yön kilidi aç/kapa (panel ayarıyla "son yazan kazanır"; squat'ta kendiliğinden kilitli) |
| İki grip + tetik 0.5 s (iki kontrolcü) | STOP |
| Panel | START / STOP / PAUSE / RESUME aynen; "Kalibre et ve devral" = A/X 1 s ile aynı (4 s içinde uygun poz beklenir) |

`OperatorInput.status` (panel ve log için, düz değerler): `mode` (walk/hands; ilk girdiden önce yok),
`heading_lock`, `calibrating` (A/X basılıyken 0→1; panel isteği denenirken 1), `calibrated`, `calibration_error`
(son red nedeni), `engage_pending`, `hand_left`/`hand_right` (tracked/hold/parked), `crouch` (stand/squat/rising),
`body_turning`, `lean_pitch_deg`/`lean_roll_deg`.

## 4. Tasarım ayrıntıları

### Kalibrasyon ve devralma
- Kalibrasyon `CalibrationWindow`'un son 1 s karesinden (`calibrate`, P3 kuralları: ≥ 20 kafa örneği, ≥ 0.8 s,
  konum yayılımı ≤ 2 cm, yaw ≤ 3°, eğim ≤ 20°, boy 1.0–2.2 m). İstek açıkken her yeni karede yeniden denenir:
  buton basılı olduğu sürece, panel isteğinde 4 s. Başarıda heading ve posture sıfırlanır.
- Bilek nötrleri kullanılmaz (bkz. §4 Kollar). `R_head` boyun pivotunun nötr konumu içindir.
- `session` ya da `epoch` değişince kalibrasyon düşer → `ready` False → PAUSED; yeniden A/X ya da panel gerekir.
- **Gerçek headset bulgusu** (`20261112-195840`, t = 359.3 s): X basılı tutulurken tarayıcı iki kontrolcüyü de
  2.1 s raporlamadı (xr_frame `valid=False`, son örnekte X basılı), kafa akmaya devam etti. Bas-tut kuralı bu
  kesintiye dayanıklı: geçersizleşmiş, son görüldüğünde A/X basılı, 3 s'den genç ve o tarafta taze el olmayan
  kontrolcü basılı sayılır. Bu kural olmadan gerçek headset'te A/X ile kalibrasyon hiç tetiklenmezdi; kayıtta
  bu kuralla X basışı kalibre etti ve START + ENGAGE üretti.

### Yön (`heading.py`)
- ψ_body kafayı histerezisle izler: kafa 25°'den uzaklaşınca bir sayaç başlar, 20°'nin içine dönünce iptal olur;
  0.3 s dolunca ψ_body 60°/s ile kafaya kayar ve 1° içinde durur (PLAN: 5°; ±5° kriteri için 1°).
- Robot heading'i kavrama gibi: ψ_body'nin her değişimi kilit açıkken heading'e eklenir, kilitliyken eklenmez.
  Kilidi açmak robotu döndürmez. 3° adım, 0.6 adım histerezisli. ψ_body kilitliyken de operatörü izler (kolların
  frame'i).
- **Yeniden gönderme:** SONIC'in planner'ı yerinde dönüşü facing başına bir kez planlıyor ve dururken başka
  planlamıyor (upstream: dururken periyodik replan yok). Simde 90° dönüş facing'in 10° gerisinde kalıyordu.
  Dururken, heading son değiştikten 3 s sonra 4° ileriye (dönüş yönünde), 4.5 s sonra tam değerine yeniden
  gönderilir. Yürürken gönderilmez. Robot yaw'ı okunmaz (açık çevrim).
- Bel-yaw payı (`waist_yaw_max_deg`) **kapalı (0)**: SONIC VR modu gövde yaw komutunun ~%6'sını uyguluyor (±15°
  bakış → gövde −2.2…+0.8°). Komut ile gerçek gövde arasında boşuna fark yaratıyordu. Mekanizma duruyor (q17 yolu
  gibi bel yaw'ını izleyen bir model için).

### Yürüme (`locomotion.py`)
- Radyal ölü bölge `stick_deadzone` (ayar), dışı 0–1'e ölçeklenir. Yön (y, −x), 10° adım, hız 0.05 m/s adım,
  ikisi de histerezisli (yürürken her değişim replan tetikliyor).
- Hız: v = v_min + (tepe − v_min)·m; tepe = clamp(speed_scale·v_yön_max, v_min, v_yön_max), elips üzerinde
  ileri 0.5 / geri 0.55 / yan 0.45 m/s, v_min 0.4 m/s (SONIC 0.3 m/s altında neredeyse yürümüyor).
- **Seçim (0.3 m durma şartı):** ileri azami hız 0.5 m/s'de kapalı. Stick bırakılınca hemen IDLE; yavaşlama
  rampası yok (IDLE SONIC'in dur komutu, rampa sadece daha uzun yürütür). Ölçülen durma mesafeleri §6'da: hepsi
  0.3 m'nin altında. Daha yüksek hız `v_fwd_max` ile config'ten açılır; o zaman şart hız başına raporlanmalı.
- Mod: 0.75 m/s'den itibaren WALK, 0.65'in altında SLOW_WALK (varsayılan aralıkta hep SLOW_WALK).
- Kontrolcü 150 ms'den bayatsa stick yok sayılır (IDLE).

### Duruş (`posture.py`)
- Boyun pivotu n = p_kafa + R_kafa·(−0.08, 0, −0.10): başı sallamak, yana eğmek, etrafa bakmak n'yi oynatmaz.
  Eğilme n'nin kalibre konumundan kaymasından (ψ_body frame'inde): pitch = asin(Δx/Ln), roll = −asin(Δy/Ln),
  Ln = L − 0.10. Kafanın açısı eğilme olarak okunmaz.
- Komut: kazanç × `lean_gain`, sınırlar ileri 25° / geri 10° / yana ±15°, 0.5 rad/s hız sınırı; torso_link nötr
  konumu etrafında döner (VR torso noktası adapter'da).
- Çömelme: Δh = (n0_z − n_z) − Ln(1 − cos pitch·cos roll). Stick 0.5 s boşta, `crouch_enabled` ve Δh > 0.12 m
  0.25 s sürerse SQUAT; height = clamp(0.8 − Δh, 0.3, 0.75), 3 cm adım histerezisli. Δh < 0.07 m, stick ya da
  ayar kapanınca RISING: bacaklar 1.5 s IDLE (SONIC kalkışı ~1.3 s), sonra yürüme serbest. Squat sürerken yön
  kilitli.

### Kollar (`arms.py`)
- **Operatör omuz frame'i** (posture): yaw ψ_body, operatörün kendi eğimi, orijin omuz hattında. Bilek bu frame'e
  alınır; robotta torso_link frame'inde omuz ortasına göre `p = c_robot + 0.7·p_op` (tek ortak merkez: eller
  birleşince robot elleri birleşir), `R = R_op·C_taraf` (`hand_geometry.ego_wrist_offset`; avuç içe, başparmak
  yukarı el → robot bilek nötrü R = I). Sonra komut edilen torso **eğimiyle** (bel-yaw payı olmadan) pelvise.
  Eğilme iki kez yansımaz (D9); bakış elleri oynatmaz.
- Mutlak gövde-frame eşlemesi: bilek kalibrasyonu gerekmez (D8 ile kalibrasyon çoğunlukla kontrolcü eldeyken).
- 0.7 = robot kolu (omuz → bilek 0.37 m) / insan kolu (~0.53 m); robot omuz ortası torso_link'te (0.004, 0, 0.235).
- Filtre: wrapper'ın One-Euro değerleri, sadece yeni el örneğinde. Güvenlik (torso frame'inde, bu sırayla): bilek
  omzundan ≤ 0.36 m; bilek gövdenin önünde (x ≥ 0.08 m: robotun arkasına ya da kalçanın yanına hiç gitmez) ve orta
  hattı en fazla 5 cm geçer; bilek ≥ 0.16 m, el noktası (+0.10 m) ≥ 0.12 m gövde kapsülünden (kapsülden ileri
  çıkılır, yan ve yükseklik korunur); pelvis frame'inde z ≥ −0.15 m; iki bilek ≥ 0.15 m (Dex3 başparmağı ~6 cm
  içeri uzanıyor). İhlal sınıra izdüşürülür. Sonra 1.2 m/s, 4 rad/s. Rampalar ve hız sınırları adım başına en fazla
  0.03 s entegre eder (`filters.step_dt`): döngü takılırsa hedef sıçramaz.
- Modlar: TRACKED (ELLER modu, ayar açık, el taze ≤ 0.25 s; girişte 0.7 s yarım-kosinüs), HOLD (el kayıp: son hedef
  1 s tutulur), PARKED (YÜRÜME, ayar kapalı ya da 1 s'den uzun kayıp: park pozuna 1 s yarım-kosinüs).
- Park pozu (pelvis frame'i, bilek linki): sol (0.162, 0.222, 0.004) rpy (17.2, 33.2, 9.6)°, sağ ayna. FK: omuz
  pitch 0, roll ±0.25, dirsek 0.6; parmaklar kalçanın önünde ve dışında (P1'deki kalçaya takılma yok).

### Parmaklar (`hands.py`)
- Repodaki Dex3 hattı aynen: WebXR eklemleri → bilek-yerel Unitree el noktaları (`hand_geometry`, wrapper ile
  ortak) → DexPilot retarget → thumb shaping → motor sırası. TRACKED'da her yeni el örneği, HOLD'da son değer,
  PARKED/ayar kapalı → dinlenme (`postures.hands_open`, config'te YAML alias'ı). Eklem başına 8 rad/s.
- dex-retargeting torch ister: retarget'ı kaynakla birlikte, node açılışında kurulur (~0.7 s). Döngü içinde
  kurulduğunda 50 Hz döngüyü 0.7–0.9 s durduruyordu (§7).

## 5. Ayarlanabilirler (`configs/fullbody.yaml` `quest:`)

| Bölüm | Anahtarlar (varsayılan) |
|---|---|
| üst | head_timeout_s 0.2, controller_timeout_s 0.15, hand_timeout_s 0.25, calibrate_window_s 1.0, calibrate_timeout_s 4.0, engage_timeout_s 90 |
| calibration | k_L 0.48 (+ calibration.py'nin diğer eşikleri) |
| gestures | hold_s 1.0, short_s 0.5, stop_s 0.5, dropout_s 3.0 |
| mode | confirm_s 0.1 |
| heading | hold_deg 20, turn_deg 25, turn_delay_s 0.3, rate_deg_s 60, settle_deg 1, step_deg 3, step_band 0.6, waist_yaw_max_deg 0, reissue_s [3.0, 4.5], reissue_rad 0.0698 |
| locomotion | v_fwd_max 0.5, v_back_max 0.55, v_lat_max 0.45, v_min 0.4, walk_above 0.75, walk_band 0.1, speed_step 0.05, direction_step_deg 10, band 0.6 |
| posture | neck_in_head [−0.08, 0, −0.10], shoulders_below_neck 0.09, torso_neutral (*alias*), gain 1, pitch_fwd_max_deg 25, pitch_back_max_deg 10, roll_max_deg 15, rate_rad_s 0.5, stick_idle_s 0.5, enter_m 0.12, enter_hold_s 0.25, exit_m 0.07, rise_s 1.5, h_stand 0.8, k_h 1.0, h_min 0.3, h_max 0.75, height_step 0.03 |
| arms | scale 0.7, robot_shoulders, reach_m 0.36, front_min_m 0.08, cross_m 0.05, torso_axis/radius, hand_point/radius, z_min_m −0.15, wrist_gap_m 0.15, hold_s 1, park_s 1, track_s 0.7, max_lin 1.2, max_ang 4, One-Euro, park |
| hands | rest (*alias* postures.hands_open), max_rate_rad_s 8, thumb_shaping |

Panel ayarları (`operator_settings`): speed_scale tepe hızı ölçekler (v_min altına inmez), stick_deadzone, heading_lock,
crouch_enabled, lean_gain (0 = gövde dik), arm_tracking (kapalı: iki kol park), fingers (kapalı: dinlenme).

## 6. Ölçümler (çıkış kriterleri)

Hepsi gerçek hat üzerinden: sentetik operatör akışı (`tools/xr_synth.py`, §1) → `--xr-replay` → `QuestSource` →
node → SONIC → MuJoCo; ölçüm `tools/mapping_metrics.py` (odometri, FK, telemetri). Her koşu replay'de birebir
(`replay.json`), node tick aralığı en fazla 20.6 ms (düzeltmeden sonra, §7). Sentetik operatörün vücut
ölçüleri eşlemenin modelinden kasten farklı (boyun pivotu (−0.075, 0, −0.11), gövde 0.70 m).

### P4 — yürüme ve yön

| Kriter | Ölçüm | Sonuç |
|---|---|---|
| Sekiz + işarette durma | İki tam tur (sol + sağ, gövdeyle dönerek, stick ileri) sonra 4 s düz, bırak. Bırakıştan durana: **0.20 / 0.25 / 0.27 m** (3 koşu, 0.41–0.48 m/s'den). Turların başlangıca kapanışı 0.64–0.77 m (açık çevrim; facing gövdeyi kademeli izliyor). | geçer (< 0.3 m) |
| Düz durma (hız seçimi) | Tam stick 4 s, bırak: ileri 0.49 m/s → **0.12 m**, geri 0.48 → 0.22, sol 0.43 → 0.18, sağ 0.38 → 0.13, yarım stick 0.41 → 0.05 | geçer; `v_fwd_max` 0.5 korundu |
| 60 s etrafa bakma (±15° yaw, ±12° baş sallama, ±2 cm salınım) | Komut edilen facing değişimi **0**; robot yaw kayması 0.35°, en fazla sapma 1.24°; taban kayması 4 mm | geçer |
| 90° vücut dönüşü ±5° (oturunca) | 4 dönüş (+90, −90, −90, +90; 45°/s), dönüşten 6 s sonra robot yaw'ı − komut edilen facing: yeniden gönderme olmadan en fazla 10.5°; 4° darbe ile **2.9° ve 3.5°** (iki koşu: −1.8/+2.8/+2.9/−2.6, −1.0/+3.5/+3.4/−2.6) | geçer (yeniden gönderme ile) |

Yeniden gönderme A/B (aynı senaryo, dönüş başına mutlak hata, en büyüğü): yok 10.5°; 1e-3 rad dürtme 1 / 2 s'de
5.7°; 3 / 4 s'de 6.7°; 1…6 s'de 4.7°; 4° darbe 3 s'de, 4.5 s'de geri 3.6° (seçilen; dururken).

### P6 — kollar ve eller (`p6_hands`, ayakta ELLER modu)

| Kriter | Ölçüm | Sonuç |
|---|---|---|
| Bilek konum hatası medyan < 3 cm, rotasyon < 10° | Segment başına sol/sağ medyan (p95): nötr 1.09/1.34 cm (1.8/1.7), daireler r = 7.7 cm robotta T = 4 s 1.18/1.04 (2.2/2.1), T = 2.5 s 1.28/1.27 (2.7/2.1), uzan-bük 0.99/1.08 (1.6/2.1), parmak hareketleri 0.98/2.14, ellere dönüş 0.91/2.75. Rotasyon medyanı 1.8–7.0°. Gecikme 0.06–0.18 s. | geçer |
| Park ↔ takip sarsıntısız | Kol eklemleri tepe hız / ivme, geçişten sonraki 3 s: kontrolcü → el 1.19 rad/s, 18 rad/s²; el kayıp (tut → park) 1.39, 15; el geri 1.83, 31; el → kontrolcü (park) 1.32, 13; yeniden eller 1.08, 14. Karşılaştırma: daire takibi 2.38 rad/s, 27 rad/s². | geçer (geçişler normal takipten yumuşak) |
| Parmaklar izler (aç / yumruk / pinch) | Ölçülen Dex3 − komut: medyan 0.059 rad, p95 0.72 rad (komut aralığı 1.81 rad; p95 hızlı kapanışlarda gecikme) | geçer |
| YÜRÜME ↔ ELLER geçişleri (kafa duruşlarıyla) | Kontrolcü → el (0.6 s duruş): PAUSED, duruş başından 1.12 s sonra ACTIVE; el → kontrolcü (1.5 s duruş): 2.06 s sonra ACTIVE. Düşme yok. | geçer |
| Küp al-bırak | Sahne P8'de (masa + küp yok). Kollar hazır: erişim küresi, ön çalışma alanı, bilekler arası 0.15 m. | P8 |

### P7 — duruş

| Kriter | Ölçüm | Sonuç |
|---|---|---|
| 20 yürü ↔ squat döngüsü | 20/20 squat girişi, düşme yok; squat'ta pelvis 0.556–0.569 m (komut ~0.48 height), en küçük up_z 0.957 | geçer |
| Bel limitine kadar eğilme kararlı | Operatör ileri 35°, geri 15°, yana 25° (limitlerin ötesi); komut 25 / −10 / ∓15° → gerçekleşen gövde 24.4 / −10.8 / −14.3 / +14.0°, tutuşta yayılım ≤ 0.05° | geçer |
| Baş sallama gövdeyi < 3° eğer | ±30° baş sallama, ±20° yana eğme, 40° aşağı bakma, ±40° hızlı bakışlar: komut edilen gövde eğimi en fazla 0.62°, gerçekleşen 1.31°; facing değişimi 0 | geçer |
| 10 dk'da yanlış çömelme < 1 | Sentetik normal hareket (bakınma, aşağı bakma, baş sallama, ağırlık aktarma, küçük eğilmeler, yürürken sallanma, el işi): 3 vücut ölçüsü × 3 hareket tohumu, **90 dk'da 0**. Gerçek kayıt (`20261112-195840`, 4 farklı kalibrasyon anı, ~4.4 dk takip): **0**. | geçer |

### Diğer

- **Kabul senaryosu akışı** (`acceptance`, §0'ın masa/rampa dışı kısmı: ileri/geri/yan yürüme, +90° dönüş, bakınma,
  eğilme, squat, eller, tek kontrolcüyle yürüme, bağlantı kopması): 3 koşu, düşme yok; kopmada robot ayakta durdu.
- **Gerçek headset kaydı simde** (`20261112-195840-nopointers-from265`, `fullbody_sim_test --xr-run`): X basılı
  tutma → kalibrasyon, START, ENGAGE; gerçek kafa duruşlarında PAUSED → kendiliğinden ACTIVE; recenter'da PAUSED
  (yeniden kalibrasyon gerekir). Düşme yok, tick aralığı en fazla 34 ms. ELLER modunda iki el takipteyken bilek
  hatası: sakin, oturarak çalışılan 7.4 s'de medyan sol/sağ 3.3/2.8 cm, rotasyon 8.1/8.8°; operatörün ayağa kalkıp
  etrafında döndüğü 15 s'de 5–10 cm (hedefler ön çalışma alanı sınırında, §8 madde 4). Kol eklemleri tepe hızı
  takip sırasında 7.7 rad/s (düzeltmelerden önce 26 rad/s, §7).
- **Emülatör** (headless Chrome + IWER, gerçek Quest sunucusu): uçtan uca test geçti; kaydı Quest kaynağından
  geçince modlar emüle edilen geçişi izliyor (YÜRÜME 0.1–15.1 s, ELLER 15.1–25.1 s, YÜRÜME). Emülatörün A basışı
  tam 1.0 s ve kafası sürekli ±30° salındığı için kalibrasyon tetiklenmiyor (beklenen).
- **Bel-yaw payı:** ±15° bakışta gövde yaw'ı −2.2…+0.8° (komutun ~%6'sı) → kapatıldı.

## 7. PLAN'dan sapmalar

1. **Bilek frame'i:** operatör gövde frame'i boyun pivotundan omuz hattına iner ve eğimle döner (PLAN: kafa
   − Rz(ψ)·boyun ofseti). `frames.operator_body_frame` buna göre yeniden tanımlandı (kalça pivotu, eğimle dönen
   segment).
2. **Kafa açısı boyun pivotunu bulmak için kullanılır** (eğilme olarak değil): PLAN "kafa açısı kullanılmaz" diyor;
   pivot olmadan 30° baş sallama ~4 cm ileri kayma = ~3° sahte eğilme.
3. **Bilekler bel-yaw payı olmadan** torso frame'ine konur (PLAN: `T_pelvis_torso(R_torso_cmd)`): bakış elleri
   döndürmesin. Bel-yaw payı zaten kapalı (§4).
4. **Bilek nötrleri kalibrasyonda kullanılmaz:** mutlak gövde-frame eşlemesi (D8 ile kalibrasyon kontrolcü elde).
5. **ψ_body durma eşiği 1°** (PLAN 5°), ±5° dönüş kriteri için; **heading yeniden gönderme** (SONIC planner'ının
   yerinde dönüşü yarım bırakması) PLAN'da yok.
6. **Bas-tut kontrolcü kesintisine dayanıklı** (gerçek headset bulgusu, §4).
7. **A/X bas-tut SONIC kapalıysa START da gönderir** (tek jestle devralma; panel olmadan da çalışır).
8. **Squat çıkışında zaman pencereli RISING** (1.5 s): kaynak pelvis yüksekliğini göremez (PLAN: "pelvis toparlanınca").
9. **Dex3 retarget'ı node açılışında kurulur** ve hız sınırları/rampalar adım başına en fazla 0.03 s entegre eder:
    tembel yükleme gerçek kayıtta 50 Hz döngüyü 0.7–0.9 s durdurdu, kollar sıçradı (26 rad/s).
10. **Kol güvenliğine ön çalışma alanı ve orta hat sınırı** eklendi (PLAN: küre, kapsül, bilek-bilek).
11. **Yeni modüller:** `operator_mode.py`, `gestures.py`; araçlar `xr_synth.py`, `quest_offline.py`,
   `mapping_metrics.py`, `xr_frame_drop_pointers.py`; `filters.HysteresisQuantizer`.
12. **D8 ile "yürürken taşıma" (P6 çıkışı, §0 madde 7) mümkün değil:** kontrolcü varken iki kol park ve parmaklar
    dinlenmede, eldeki küp düşer. Kullanıcı kararı gereği; bkz. §8.

## 8. Açık riskler

1. **Yürürken taşıma D8 ile mümkün değil** (P6 çıkışı "yürürken taşıma 3/3", §0 madde 7): kontrolcü alınınca iki
   kol park eder, parmaklar dinlenmeye döner; eldeki küp düşer. Çözüm kullanıcı kararı gerektirir (ör. YÜRÜME'de
   parmakları tutmak, kolu park yerine son hedefte dondurmak ya da tetikle kavrama).
2. **A/X bas-tut gerçek headset'te tek gözleme dayanıyor:** X basılıyken kontrolcülerin akıştan düşmesi bir kez
   görüldü; kural buna dayanıklı, ama başka bir davranış (ör. sistem menüsü) olursa kalibrasyon panelden yapılır.
3. **Yerinde dönüş yeniden gönderme ile ±5° içinde** (2.9° ve 3.5°, iki koşu); SONIC planner'ının davranışına
   bağlı açık çevrim bir çare. Yürürken dönüşler SONIC'in kendi replan'ıyla kapanıyor.
4. **Gövde dönüşünde kollar:** ELLER modunda operatör yerinde dönerse ψ_body histerezis yüzünden geride kalır,
   eller o frame'de döner görünür; ön çalışma alanı ve orta hat sınırı hedefleri robotun önünde tutar ama dönüş
   sürerken eller doğru yerde değildir. Dönüş bitince (ψ_body yetişince) düzelir.
5. **Eğilme tahmini adım atmaya duyarlı:** operatör fiziksel olarak adım atarsa (yerinde durmazsa) kafa kayması
   eğilme sayılır (20 cm ≈ 16°). Komut limitleri (25° / 15°) ve hız sınırı sınırlar; kalibrasyon yenilenince düzelir.
6. **Kalibrasyon şartları:** dik dur, ileri bak (eğim ≤ 20°), 1 s kıpırdama (2 cm, 3°). Panelden kalibrede operatör
   panele bakıyorsa reddedilir; 4 s içinde ileri bakınca kabul edilir (`calibration_error` status'ta).
7. **Parmak dinlenme pozu sıfırlar** (`postures.hands_open`): retarget'ın "açık el"inden farklı (başparmak −1.05 rad);
   ELLER → park geçişinde başparmak hareket eder (hız sınırlı).
8. **Dex3 retarget yükü:** node açılışında ~0.7 s + torch import; tick başına el örneği geldiğinde 1–3 ms/el.

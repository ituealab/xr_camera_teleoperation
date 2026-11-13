# Quest 3S → SONIC: tam vücut teleop (MuJoCo) — uygulama planı

Branch: `quest_emin` (`quest-xr`'dan, 3144d6d). Tarih: 2026-11-12.

**Durum (2026-11-13):**

| Faz | Durum |
|---|---|
| P0 | Tamam. Gerçek headset kaydı `data/fullbody-runs/20261112-195840` (300 s); bulgular `P0_quest_probe.md` §6–7. Kafa ~36 Hz (robot görüntüsü açıkken 72 Hz / 2); Quest 3S Quest Browser'da karışık el+kontrolcü modu yok (D8). |
| P1 | Tamam: 10 dk dayanıklılık düşmesiz, §7a fizik hedefleri geçiyor (`P1_sim.md`). GPU 20 W kilidi uykudan dönüş hatasıydı; yeniden başlatmadan sonra kameralar açık da geçiyor (`P1_sim.md` §3a). |
| P2 | Tamam (`P2_node.md`): node, girdi kaynağı ara yüzü, lifecycle, script senaryoları, kill ve replay testleri; sapmalar orada §5. |
| P3 | Tamam. Gerçek kayıt §6 kurallarıyla doğrulandı; takip edilen elin işaretçisi artık kontrolcü akışına girmiyor. |
| P4, P6, P7 | Tamam (`P4_P6_P7_mapping.md`); P6'nın "yürürken taşıma" çıkışı D8 ile imkânsız (§0 madde 7). |
| P5 | Tamam (`P5_ui.md`). |
| P8 | Kısmen (`P8_results.md`). Parkurlar ve sabah yerleşimi, gantry (20 başlatma, 0 düşme), tek komutla açılış (`deploy/xr_fullbody_sim.sh morning`, viewer masaüstünde), 5° rampa 5/5, §7a tam sahnede geçiyor. Park pozu düzeltildi (kollar yanlarda; eller artık masaya takılmıyor). §0 kuru koşusu (emülatör, 2×3 koşu): bir koşu her adımı geçti, düşme 0; **üç üst üste geçmedi**: küp 6'da 1 (robotun masadaki duruşu), 90° dönüş 6'da 4 (±5°). Sabah kılavuzu `SABAH.md`. |

Faz belgelerinin PLAN'dan sapmaları her belgenin kendi "PLAN'dan sapmalar" bölümündedir (P1 §7, P2 §5, P4_P6_P7 §7, P5 §7, P8 `P8_results.md` §9).

## 0. Bitiş tanımı

Plan şu **kabul senaryosu** üç kez üst üste, düşmeden geçtiğinde biter. Senaryo MSI'da, MuJoCo'da çalışır; Quest operatörün kafasındadır.

1. Operatör Quest'te sayfayı açar ve Enter VR'a basar. Simülasyondaki robotun gözünden stereo görüntüyü görür. Ayrıca bir kontrol paneli görür: robotu başlatma/durdurma ve operatör ayarları. Panel elle ya da kontrolcüyle seçilir (§6a).
2. Nötr duruşta kalibre olur: dik durur, ileri bakar, butonu basılı tutar.
3. Thumbstick ile ileri, geri ve yana yürür. Stick'i bırakınca robot 0.3 m içinde durur (aşağıdaki nota bakın).
4. Vücuduyla 90° döner, robot da döner. ±15° etrafa bakınca robot dönmez; sadece gövde yaw'ı (bel) bakışı takip eder.
5. Öne ve yana eğilince robotun gövdesi eğilir. Çömelince robot squat yapar. Kalkınca robot da kalkar, sonra tekrar yürür.
6. Kontrolcüyü bırakır, iki el serbest kalır. Robotun kolları bilekleri, Dex3 parmakları parmakları takip eder. Masadan bir küpü alıp yerine koyar.
7. ~~Kontrolcüyü bir eline alır; o kol yana park eder. Küp diğer elindeyken başka bir masaya yürür ve küpü bırakır.~~
   **D8 ile geçersiz (P8):** kontrolcü alınınca iki kol park eder ve parmaklar açılır, küp düşer; yürürken taşıma bu
   kuralla mümkün değil. Kabul senaryosunda bu adım yok; ne yapılacağı kullanıcı kararı (`P4_P6_P7_mapping.md` §8 madde 1).
8. 5°'lik rampayı çıkar ve iner. 10° ve 15° rampalar ile basamaklar ölçülür ve raporlanır; bunlar geçme kapısı değildir.
9. Quest bağlantısını keser (headset'i çıkarır ya da sayfayı kapatır). Robot ayakta, güvenli şekilde durur.

Senaryo boyunca MuJoCo gerçek zamanlı ve akıcı çalışır (§7a): takılma ya da yavaşlama yoktur.

**Madde 3 için not (P1 ölçümü; karar kullanıcıda):** Durma mesafesi SONIC planner'ının özelliğidir ve hıza bağlıdır. IDLE'a geçişten sonra 0.5 m/s'den 0.25 m, 1.0 m/s'den 0.48 m, 1.5 m/s'den 0.87 m. Yani 0.3 m şartı yalnız ~0.5 m/s'ye kadar sağlanıyor. Bizim tarafımızdaki tek kaldıraç hız sınırıdır. Stick bırakılınca uygulanan bir yavaşlama rampası mesafeyi kısaltmaz, en fazla sarsıntıyı azaltır. Öneri: kabul senaryosunda operatörün azami ileri hızı `v_fwd_max = 0.5 m/s` (config). Daha yüksek hız istenirse şart "≤ 0.5 m/s'de 0.3 m; üstünde hız başına ölçülüp raporlanır" olur. Şart kullanıcı onayı olmadan gevşetilmez.

Her koşuda xr_frame, giden komutlar ve robot durumu kaydedilir. Kayıt sonradan replay edilebilir olmalıdır.

## 1. Kapsam

**Kapsamda:**
- Quest kafa, kontrolcü ve el verisini toplamak.
- Operatör niyetini çıkarmak.
- SONIC v1.1 adaptörü.
- MuJoCo sahnesi: eğim, masa ve nesneler, robot kamerası.
- Headset içi HUD.
- Kayıt ve replay.
- Testler.

**Kapsam dışı (bu plan bittikten sonra):** PC2'ye taşıma, PC2 döngüde simülasyon, gerçek robot, SONIC tarafında watchdog, arazi uyarlaması (ör. GRAFT).

Tasarım bunları zorlaştırmayacak şekilde yapılır (bkz. §12).

## 2. Kararlar

| # | Karar | Gerekçe |
|---|---|---|
| D1 | **SMPL yok.** SONIC'in planner hibriti kullanılır: zmq_manager `planner` topic'i, 50 Hz. | Kafa ve iki bilekten güvenilir insan vücudu çıkmaz. SMPL modunda planner bacakları yoktur. Resmi PICO teleop'u da bu yolu kullanır. |
| D2 | **Üst gövde VR 3-nokta (encoder mod 1).** Sol bilek, sağ bilek ve gövde proxy'si, pelvise göre 6D. | Bel, kol ve denge koordinasyonunu SONIC'in eğitildiği tek model çözer. |
| D3 | **Kontrol yolunda kol IK'sı yok.** | Bileğin nerede olması gerektiğini biz söyleriz, eklemleri SONIC çözer. q17+IK yolu sadece yedektir (§11). |
| D4 | **Parmaklar tamamen bizim Dex3 retarget'ımız.** Planner mesajındaki `left/right_hand_joints` ile gider, her mesajda. | Dex3 komutunu zaten SONIC yayınlar. Her aktüatörün tek sahibi olur, upstream'e yama gerekmez. Alan yoksa SONIC elleri yumruk yapar (P1). |
| D5 | **Yürüme thumbstick'ten:** sol stick yürütür, sağ stick'in x'i robotu döndürür (tam itince 60°/s; tek kontrolcüde onun stick'i yürütür, dönüş yok). Stick boşken mode `IDLE`. ELLER modunda ek olarak operatörün kendi adımları (D15). | `speed<=0`, "modun varsayılan hızı" anlamına gelir; dur komutu değildir. |
| D6 | **Robot yönü kafa yaw'ından gelir.** Mutlak ve kalibre edilmiş; histerezisli, hız sınırlı, kademeli. Bir kilit butonu vardır. Sağ stick heading'e doğrudan eklenir (kilitliyken de; ψ_body'ye değil). | Etrafa bakmak robotu döndürmemeli. Facing'deki her float değişikliği yeniden planlama tetikler. |
| D7 | **Çömelme dururken ve kademelidir** (planner mode 4 + `height`). Operatör yere kadar inerse (düşüş > 0.55 m, 0.5 s) **KNEEL** (planner mode 5, iki diz): SONIC dizlerinin üstünden öne eğilip elleri yere getirir; squat'ı getiremez (en alçak squat'ta eller yerden ≥ 0.34 m). Kollar diz çökerken genişletilmiş sınırlarla (`arms.py`). | Stok SONIC'te çömelerek yürüme yoktur; `height` sadece statik modlarda çalışır. |
| D8 | **İki çalışma modu, operatörün elinden çıkar (kullanıcı kararı, 2026-11-12).** **YÜRÜME:** en az bir fiziksel kontrolcü takipteyse. Stick (hangi kontrolcüdeyse) yürütür (sol yürür, sağ döndürür). **ELLER:** hiç kontrolcü yoksa. Takip edilen her el aynalanır (TRACKED), takip edilmeyen taraf PARKED; yürüme IDLE. Geçişler park↔takip rampalarıyla (§5) yumuşar. **Kumandayla kol (ayar `controller_arms`, varsayılan açık, kullanıcı kararı 2026-11-13):** YÜRÜME'de kollar kontrolcüleri izler; tetik basılı tutuldukça o taraftaki Dex3 eli yumruğa kapanır (analog), bırakınca açılır (`hands.py` `closed`). Tetiğin başka görevi yok (STOP: iki grip + tetik). Bilek pozu target-ray'den; stick yürütmeye devam eder (yürürken taşıma). Quest 3S Quest Browser karışık el+kontrolcü vermiyor (P0 §6); kural buna dayanıklı: platform verirse de kontrolcü varken kollar park kalır. | Tek kontrolcü yürümeye yeter; yürürken kollar sabit, bırakınca eller hemen devam eder. |
| D9 | **Bilek hedefleri sanal operatör gövdesine göre tanımlanır.** Gövde hedefiyle aynı frame kullanılır. | Eğilme hem gövdeye hem kola iki kez yansımaz. |
| D10 | **Upstream kopyalanmaz ve yamalanmaz.** SONIC kaynağı, modeller ve binary bir config yolu ve manifest (commit + sha256) ile referanslanır. | Tasarım borcu olmaz, güncelleme tek yerden yapılır. |
| D11 | **Sim ve gerçek robotta aynı süreç grafiği çalışır.** Fark sadece DDS ucu, yani MuJoCo köprüsü ya da robot. | İleride PC2'ye taşımak sadece config değişikliği olur. |
| D12 | **Operatör arayüzü headset'tedir:** robot kamerası + seçilebilir kontrol paneli (başlat/durdur, ayarlar) + kısa yol butonları. Kontrol yolunda MSI GUI'si yoktur. | Son halde MSI yok. |
| D13 | **Panel etkileşimi sunucu tarafında çözülür:** el pinch ışını ya da kontrolcü ışını ile panel geometrisine raycast; hover ve tıklama bizim kodumuzda. Vuer sadece çizer. | Vuer'in XR içi tıklama desteğine bağımlı kalmayız. Davranış test edilebilir ve PC2'ye aynen taşınır. |
| D14 | **Simülasyon akıcı olmalı:** fizik gerçek zamanlı (RTF ≥ 1.0), görüntüleyici ≥ 60 fps, robot kamerası 30 fps. Kamera render'ı fizik döngüsünü yavaşlatmamalı. | Kasan bir sim hem kullanıcı deneyimini hem SONIC'in 50 Hz döngüsünü bozar. |
| D15 | **ELLER modunda operatörün adımları robotu yürütebilir (ayar `step_walk`; kullanıcı kararı 2026-11-13: varsayılan kapalı, ELLER'de robot xy hedefi almaz, sadece eğilme ve çömelme).** Duruş referansı (eğilmenin sıfırı) boyun pivotunu tasmayla izler: kaymanın eğilmenin düşüşüyle açıklanamayan kısmı bir elipsi (ileri 8, geri 6, yana 8 cm) aşınca referans **her tick, hemen** sürüklenir; ≥ 5 cm sürüklenince operatör yürüyor sayılır, elips 4 cm'ye iner, durunca kuyruk emilir. Sürüklenen yer değiştirme robot gövde frame'inde bekler; ≥ 8 cm (yan ×0.5) olunca robot SLOW_WALK 0.4 m/s (geride kalırsa 0.5) **operatörle birlikte** yürür, kat ettiği yol darbe modeliyle D = kazanç·v·(T − gecikme) tahmin edilip düşülür (açık çevrim, `stepping.py`). Önceki sürüm adımı ancak kafa durulunca sayıyordu: kullanıcının ilk denemesinde sürekli yürüyüş hiç adım sayılmadı, eğilme 90°'ye doydu ve eller savruldu (kayıt 20261113-135937). Oturan operatörde (boy < 1.35 m) yürüme yok. | Manipülasyon sırasında küçük yer düzeltmeleri için joystick'i alıp bırakmak zahmetli; kafa xy takibi iyi. |

## 3. Mimari

```
Quest 3S (Quest Browser, WebXR)
  │  wss :8012 (Vuer)
  ▼
quest_server  ─────────────── (mevcut, genişletilir)
  ├─ HEAD_MOVE (Vuer 0.1.6 `Head`), HAND_MOVE, CONTROLLER_MOVE
  ├─ PUB :5594  perception mesajı (mevcut live_ui için, değişmez)
  ├─ PUB :5596  xr_frame (P3: tipli anlık görüntü, kaynak başına zaman/geçerlilik)
  ├─ SUB :5605  stereo JPEG (gerçek: ZED, sim: camera_bridge) → headset
  ├─ ui.py: kontrol paneli (çizim + raycast); tıklamalar UiAction olarak node'a
  └─ HTTP :8771 view modu + YENİ /state (node durumu girer, panelde gösterilir)
        │ xr_frame (CONFLATE)
        ▼
fullbody_node  (YENİ, başsız, sabit 50 Hz)
  calibration → heading → locomotion → posture → arms → hands → supervisor
  ├─ PUB :5556  SONIC zmq_manager (command / planner)
  ├─ POST :8771/state  durum (panele)   ◄─ PUB :5597 UiAction (panelden)
  └─ log: xr_frame + FullBodyCommand + durum (msgpack, replay edilebilir)
        ▼
SONIC g1_deploy_onnx_ref  (upstream, değişmez; --input-type zmq_manager)
        │ DDS (rt/lowcmd, rt/lowstate, rt/dex3/*, rt/odostate)
        ▼
MuJoCo sim (upstream fizik + DDS köprüsü, bizim gerçek zamanlı döngümüz) ──JPEG stereo :5555──► camera_bridge (çözer) ──► :5605
```

Port çakışması kontrolü: sim kamerası :5555, SONIC telemetrisi :5557, fullbody → SONIC :5556, sim istatistik :5598, sim tuşları :5599. Sim ve SONIC portları `configs/sonic_sim.yaml`'dadır (tek kaynak); `configs/fullbody.yaml` SONIC portlarını oradan okur.

## 4. Modüller ve sorumluluklar

Her modülün tek bir görevi vardır. Saf matematik modülleri I/O yapmaz ve birim testlidir.

| Yol | Tek görevi | Girdi → çıktı |
|---|---|---|
| `operator/quest/server.py` (değişir) | Quest'ten veri almak ve yayınlamak; görüntü ve HUD sunmak | Vuer olayları → `perception` (mevcut) + `xr_frame` |
| `operator/quest/assembler.py` (P3) | Vuer olayları → `xr_frame` anlık görüntüleri: saat eşleme, recenter tespiti (`epoch`). Saf, replay edilebilir. | olay + alım zamanı → `XRFrame` |
| `operator/quest/vuer_app.py` (P3) | Vuer 0.1.6 uygulaması; sonsuz yeniden bağlanma için sayfaya WebSocket sarmalayıcısı enjekte eder | — |
| `operator/quest/ui.py` | Headset kontrol paneli: düzen, widget'lar, raycast, hover/tıklama; durum gösterimi. Çizim Vuer elemanlarıyla. | xr_frame (ışın kaynakları) + node durumu → panel sahnesi + `UiAction` olayları |
| `operator/quest/convert.py` (değişir) | XR çözme ve frame dönüşümü (kafa, kontrolcü, el) | ham olay → tipli poz/eksen/buton |
| `fullbody/xr_frame.py` | `xr_frame` şeması: encode, decode, doğrulama | dict ↔ `XRFrame` (kaynak başına `t`, `valid`, `session`) |
| `fullbody/frames.py` | Tek yerde frame tanımları: XR(Y-up) → Z-up, operatör gövde frame'i, robot pelvis/torso nötr pozları | poz → poz |
| `fullbody/runlog.py` (P3) | Koşu dizini: `run.msgpack` (kayıt akışı) + `meta.json`; yazma ve okuma | kayıtlar ↔ dosya |
| `fullbody/calibration.py` | Nötr duruşu kaydetmek: kafa yüksekliği h0, yaw ψ0, gövde boyu L, bilek nötrleri | XRFrame'ler → `Calibration` |
| `fullbody/heading.py` | Kafa yaw'ı → operatör gövde yaw'ı ve robot facing'i (histerezis, kilit, kademe) | ψ_head → ψ_body, facing, waist-yaw payı |
| `fullbody/locomotion.py` | Stick → mode, movement, speed | eksenler, ψ_facing → `LocomotionCmd` |
| `fullbody/posture.py` | Kafa pozu → gövde proxy hedefi + çömelme durumu | XRFrame, Calibration → `TorsoTarget`, `CrouchState` |
| `fullbody/arms.py` | El modu (TRACKED/PARKED) ve bilek 6D hedefleri (pelvise göre); filtre, limit ve ucuz geometrik güvenlik | XRFrame, TorsoTarget → 2× bilek pozu |
| `fullbody/hands.py` | Dex3 7+7 hedefleri (mevcut `HandRetargeting` + `thumb_shaping`) | el eklemleri → 14 hedef |
| `fullbody/supervisor.py` | Durum makinesi, bayatlık kuralları, butonlar | her şey → durum + komut maskesi |
| `fullbody/node.py` | 50 Hz döngü: bağlama, loglama, HUD | süreç |
| `operator/hand_geometry.py` (çıkarılır) | El eklemlerinden bilek-yerel parmak koordinatları; saf fonksiyonlar, `wrapper.py` ile ortak | 25×3 → yerel |
| `sonic/protocol.py` (P1) | SONIC paket formatı (konu + 1280 bayt JSON başlık + diziler); upstream gönderici ile bayt bayt aynı (golden test) | alanlar → bytes |
| `sonic/adapter.py` | `FullBodyCommand` → planner alanları (wxyz, sıralar, offset'ler) | komut → alan dict'i |
| `sonic/lifecycle.py` | start, planner, VR yeniden kurma sırası; yeniden bağlanma | durum → command mesajları |
| `sim/scene_builder.py` | Upstream sim sahnesi `scene_43dof.xml` + stereo kamera + parkur → üretilmiş sahne (MjSpec ile, kopya yok) | `configs/sim_courses.yaml` → `data/sim/scenes/*.xml` (gitignore) |
| `sim/run_sonic_sim.py` | Upstream `DefaultEnv`'i (`SimLoopConfig` + `SimulatorFactory.create_simulator`) bizim sahne ve domain ayarımızla kurar; fizik döngüsü bizim (mutlak deadline), render alt süreçlerde | config → süreç |
| P1 ek modülleri | `sim/{camera_publisher,sim_viewer,state_snapshot,perf,sim_config}.py`, `sonic/{assets,run_deploy}.py`, `tools/{sonic_drive,sim_perf}.py`; görevleri `P1_sim.md` §2 | — |
| `sim/camera_bridge.py` | Sim'in upstream formatındaki JPEG akışı (:5555, `ImageUtils`) → çöz → bizim stereo formatı (:5605) | ZMQ → ZMQ |
| `tools/fullbody_replay.py` | Log'dan node'u başsız tekrar oynatmak | log → komutlar |
| `deploy/xr_fullbody_sim.sh` | tmux launcher: sim, SONIC, kamera köprüsü, quest_server, fullbody_node | — |
| `configs/fullbody.yaml` | Tüm eşikler, kazançlar, limitler, node portları, park pozları; SONIC portları `sonic_sim.yaml`'dan | — |
| `configs/sonic_sim.yaml` (P1) | Sim ve SONIC çalışma ayarları: dt, bant, viewer/kamera hızları, sim ve SONIC portları | — |
| `configs/sonic_assets.yaml` | SONIC kaynak, model ve binary yolları + commit + sha256 | — |

Mevcut `live_ui` ve `ee_runtime` yolu (sadece kol modu) değişmez. Tek fark, ortak el geometrisinin `hand_geometry.py`'den gelmesi.

**Ortak veri tipleri** (`fullbody/types.py`, dataclass):
- `FullBodyCommand`: mode, movement[3], facing[3], speed, height, `vr_position[9]`, `vr_orientation[12]` (wxyz), `vr_enabled`, `hand_joints[2][7]`, stamp.
- `TorsoTarget`, `WristTarget` (pelvise göre `T` 4×4), `LocomotionCmd`, `CrouchState`, `HandMode`.

## 5. Frame'ler ve eşlemeler

Bu bölümün kodu `frames.py`'de yaşar. Diğer modüller frame'lere sadece onun üzerinden erişir.

**XR → kontrol frame'i.** WebXR local-floor (Y yukarı, −Z ileri), Z yukarı / X ileri / Y sol frame'e çevrilir. Tüm XR pozları (kafa, bilek, kontrolcü) aynı dönüşümden geçer. Rotasyon iki taraftan dönüştürülür; mevcut `convert.py`'deki tek taraflı `S·R` kafa için düzeltilir.

**Kalibrasyon** (`calibration.py`): Operatör dik durur ve ileri bakar, buton 1 s basılı tutulur. Şunlar kaydedilir:
- h0: kafa yüksekliği
- ψ0: kafa yaw'ı
- p0: kafanın yer izdüşümü
- L: kalça-kafa uzunluğu, `L = k_L·h0` (k_L config'te, ilk değer ≈0.48)
- Bilek nötrleri (iki el TRACKED ise)

Vuer, WebXR `reset` olayını sunucuya iletmez. Recenter sunucuda kafa pozundaki adımdan algılanır ve `xr_frame.epoch`'u artırır (P3). `session` ya da `epoch` değişince kalibrasyon düşer (`Calibration.matches`); sistem PAUSED'a geçer ve yeniden kalibrasyon ister.

**Heading** (`heading.py`):
- ψ_body (operatörün gövde yaw tahmini) kafayı histerezisle izler. |ψ_head − ψ_body| ≤ 20° iken sabittir. 25°'yi en az 0.3 s aşarsa ψ_body, en fazla 60°/s ile ψ_head'e kayar; fark 5°'nin altına inince durur.
- Robot facing'i şöyle hesaplanır: `ψ_cmd = ψ_plan0 + (ψ_body − ψ0)`, 3°'lik adımlara kuantize edilir, `facing = [cos ψ_cmd, sin ψ_cmd, 0]`. ψ_plan0 SONIC planner dünyasında başlangıç yönüdür (ilk gönderilen facing). Ölçülen robot yaw'ı kullanılmaz.
- Waist-yaw payı: `clamp(ψ_head − ψ_body, ±yaw_waist_max)`. Gövde proxy'sinin yaw'ına girer; robot gövdesi ve kamerası bakışı izler.
- Kilit: buton ile aç/kapa. Squat'ta kilit otomatik olarak açıktır. Kilitliyken ψ_body donar.
- Sağ stick: x'in ölü bölge dışı kısmı × 60°/s heading'e eklenir (kilitliyken de, squat/kneel'de değil).

**Yürüme** (`locomotion.py`):
- Sol stick (x, y) (tek kontrolcüde onunki), radyal deadband 0.15. xr_frame'de stick ileri = +y = +1 (P3). `v_local = [v_fwd_max·y, −v_lat_max·x]`.
- `|v| < ε` → mode `IDLE`, movement 0. Değilse eşiğin altında `SLOW_WALK`, üstünde `WALK` (histerezisli).
- speed = |v|, modun ölçülmüş aralığına kırpılır. P1 ölçümü:
  - ~0.3 m/s altındaki komutlar neredeyse hareket üretmiyor; en düşük yürüme hızı ≥ 0.35–0.4 m/s olmalı.
  - SLOW_WALK ve WALK 0.5–1.2 m/s'de aynı hızı veriyor; fark yürüyüş stili.
  - `speed = -1` varsayılanları: SLOW_WALK 0.17, WALK 0.88 m/s.
  - Durma mesafesi hıza bağlı (§0 notu); `v_fwd_max` buna göre seçilir.
- `movement = Rz(ψ_cmd)·v̂_local`. İleri yön kafaya değil, komut edilen facing'e göredir.

**Gövde proxy'si** (`posture.py`, VR noktası 3):
- Operatörün gövde eğim tahmini kafa *pozisyonundan* yapılır; kafa açısı kullanılmaz, çünkü baş sallamak gövde eğmek değildir. Kafanın ψ_body frame'indeki kayması Δx_fwd ve Δy_lat ile:
  - `θ_pitch = asin(clamp(Δx_fwd / L, ±1))`
  - `θ_roll = asin(clamp(Δy_lat / L, ±1))`
- Komut kazanç, limit (config, robot bel limitleri içinde) ve hız sınırından (≤0.5 rad/s) geçer.
- Hedef rotasyon: `R_torso = Rz(waist_yaw_payı)·Ry(θ_pitch)·Rx(θ_roll)`. Pozisyon: `p = p_torso_nötr + R_torso·[0, 0, 0.35]`. İkisi de pelvise göredir; nötrler robot FK'sinden, ayakta duruşta.
- Kuantizasyon yoktur: VR alanları yeniden planlama tetiklemez, sadece facing ve height tetikler.

**Çömelme** (`posture.py`):
- Eğilmeden düzeltilmiş düşüş: `Δh = (h0 − h_head) − L(1 − cos θ_pitch)`.
- **Giriş:** stick en az 0.5 s boşta ve Δh > 0.12 m. Mode `SQUAT`, `height = clamp(h_stand − k_h·Δh, h_min, h_max)`, 3 cm adımlarla.
- **Aralık (P1):** `height` 0.3–0.8 → pelvis 0.44–0.79 m (ayakta 0.786); 0.3'ün altı doyuyor. Squat'a giriş ~1.3 s'de oturuyor. zmq_manager squat'ta `height < 0.2`'yi 0.2'ye çeker; yani `-1` "varsayılan" değil en alçak squat demektir, her zaman açık bir yükseklik gönderilir. Kneel modları `height`'ı yok sayar.
- **Çıkış:** Δh < 0.07 m (histerezis) ya da stick sapması. Önce `IDLE`, ardından pelvis yüksekliği toparlanınca yürüyüşe izin verilir.

**Bilekler** (`arms.py`):
- Operatör gövde frame'i: `T_op = (Rz(ψ_body)·R_torso_op, p_head − Rz(ψ_body)·neck_offset)`. Burada R_torso_op, θ_pitch ve θ_roll'dan kurulur.
- Bilek XR pozu bu frame'e alınır: `T_op⁻¹·T_wrist`. Sonra mevcut "ego" eşlemesinin mantığı uygulanır (ortak orta nokta, ölçek k, taraf başına oryantasyon offset'i), robotun torso frame'ine taşınır ve `T_pelvis_torso(R_torso_cmd)` ile pelvise göre ifade edilir.
- Sonuç SONIC'in bilek proxy'sine çevrilir: `wrist_yaw_link + [0.18, ∓0.025, 0]`. Bu çevirinin tek sahibi `sonic/adapter.py`'dir.
- Mevcut One-Euro filtreleri, hız limitleri (1.2 m/s, 4 rad/s) ve 0.40 m küre sınırı yeniden kullanılır. Kontrol yolunda IK olmadığı için çarpışma kapısı yerine ucuz geometrik kontroller vardır:
  - bilek-bilek mesafesi > d_min
  - bilek-gövde kapsülü > d_min
  - çalışma alanı küresi

  İhlalde hedef, sınır yüzeyine yansıtılır.
- WebXR'ın gerçek bilek rotasyonu doğrudan kullanılır; 21 noktadan yeniden kurulmaz.

**El modları** (`arms.py` + `supervisor.py`):
- Her taraf için giriş kaynağı türüne bakılır. El takibi varsa TRACKED, kontrolcü varsa PARKED.
- PARKED: kol, config'teki park pozuna (pelvise göre sabit, yan taraf) yarı-kosinüs geçişle 1.0 s'de gider. Parmaklar nötr-açık. Park pozu elleri kalçadan uzak tutmalı: P1'de SONIC'in IDLE kol pozunda Dex3 işaret parmağı kalçaya takılıp açılamıyor.
- Kontrolcü pozu WebXR target-ray pozudur, grip pozu değil (P0/P3). Işın ve park için doğrudan kullanılır; el yerine kullanılacaksa ofset gerekir.
- TRACKED'a dönüşte, o anki hedeften takip hedefine 0.7 s'de geçilir.
- El kaybı: 0.25 s son hedefi tut; 1 s sonra PARKED gibi davran.

## 6. Durum makinesi ve güvenlik (`supervisor.py`, `lifecycle.py`)

```
DISCONNECTED ─(SONIC hazır + Quest bağlı)→ STANDBY ─(kalibrasyon)→ ACTIVE ⇄ PAUSED
     ▲                                                     │
     └──────────── Quest koptu / SONIC koptu ◄─────────────┘
```

- **STANDBY:** `IDLE`, VR kapalı, sadece facing ψ_plan0. SONIC robotu ayakta tutar.
- **ACTIVE:** VR 3-nokta, yürüme, duruş ve eller açık.
- **PAUSED:** `IDLE`, facing donuk. VR hedefleri 1.5 s'de nötr duruşa yumuşar ve öyle kalır. Şunlarla girilir: pause butonu, kafa verisi bayatlaması (>200 ms), recenter (`epoch`) ya da oturum değişimi. Bayatlıkla girilen PAUSED, aynı oturum ve epoch'ta kafa 0.5 s taze kalınca kendiliğinden ACTIVE'e döner (el↔kontrolcü geçişi kafayı 0.6–1.5 s durdurur ve iki el serbestken buton yoktur). Buton, recenter ve oturum değişimiyle girilen PAUSED elle çözülür.
- **Bayatlık kuralları:**
  - Bayatlık her kaynağın örnek zamanına (`t`) göre ölçülür, alım zamanına göre değil.
  - Düğüm 50 Hz'de son örnekle çalışır; kafa > 200 ms bayatsa PAUSED. Kafa örnek aralığı p50 ≤ 40 ms, p99 ≤ 100 ms; sakin kullanımda ≤ 0.2 duraklama/dk (geçiş, recenter ve oturum sonundakiler ayrı raporlanır). 50 Hz kafa şartı kaldırıldı (kullanıcı kararı, 2026-11-12): robot görüntüsü açıkken headset ~36 Hz örnekliyor.
  - Kontrolcü verisi > 150 ms bayatsa yürüme `IDLE` olur, diğer her şey devam eder.
  - El verisi için §5'teki kural geçerlidir.
  - fullbody_node her durumda 50 Hz yayın yapar.
- **Lifecycle:**
  1. `command{start=1, planner=1}` gönderilir.
  2. Planner init beklenir. P1: telemetri (`g1_debug`, :5557) start'tan ~0.2 s sonra akmaya başlıyor, planner init ~140 ms. Sabit gecikme yerine bu sinyal kullanılır (P2).
  3. **VR yeniden kurma:** VR alanları olmadan bir mesaj, ardından VR alanlarıyla mesajlar gönderilir. Bu, upstream'deki "init encoder modunu sıfırlar, VR enable kenar tetikli" davranışına karşı gerekli.
  4. Durdurmada `command{stop=1}` gönderilir. `stop` binary'yi kapatır (süreç çıkar, motorlar sönüm moduna geçer); yeniden başlatmak için süreç yeniden açılmalı (sim'de `run_deploy --respawn`, ~28 s). P2: node kapanırken stop göndermez; STOP operatörün açık isteğidir (`P2_node.md` §5).
- **Kabul edilen risk:** stok SONIC 1 s planner timeout'unda VR bayrağını temizlemez (P1'de gözlendi: VR modu mesaj başına `vr_position` varlığıyla açılıp kapanıyor, timeout ona dokunmuyor). Simde bacaklar `IDLE`'a geçer, üst gövde son hedefte kalır; senaryo 9 bunu ölçer. Gerçek robot öncesi SONIC tarafı watchdog'u gerekir (§12).

**Kontrolcü düzeni** (eşleme config'te):

| Giriş | Görev |
|---|---|
| Thumbstick | Yürüme |
| A/X 1 s basılı | Kalibre ol / ACTIVE'e geç |
| A/X kısa | Pause / devam |
| B/Y | Heading kilidi aç/kapa |
| Tetik/grip | Kullanılmıyor; ileride park edilen elin kavraması için ayrılmış |

## 6a. Headset arayüzü (`operator/quest/ui.py`)

**Görüntü:** Robot kamerasının stereo görüntüsü ana görüntüdür (mevcut ego/immersive modları). Panel görüntüyü kapatmaz.

**Panel:**
- Çağrılınca açılır: thumbstick basma ya da el jesti (sol avuç açık yüze, sonra sağ işaret parmağıyla avuca dokunma; tek başına avuç çevirmek manipülasyonda sık olduğu için açmaz, `P5_ui.md` §9). Dünyaya sabitlenir: açıldığı anda göğüs hizasında, 0.45 m önde.
- Açıkken yürüme ve heading donar, kollar o anki hedefte kalır. Böylece panelle uğraşırken robot hareket etmez.
- Kapalıyken köşede küçük bir durum şeridi görünür: durum, mod, kilit, el modları, uyarılar.

**Seçim:**
- Elle: işaret parmağı ışını, pinch ile tıklama.
- Kontrolcüyle: kontrolcü ışını, tetik ile tıklama.
- Hover vurgusu var; tıklamada kısa görsel geri bildirim.
- Kritik komutlar (Robotu durdur hariç) basılı tutmayla onaylanır (0.6 s halka dolar).

**Panel içeriği** (sayfalar sekme olarak):

| Sayfa | Öğeler |
|---|---|
| **Robot** | `Başlat` (SONIC start + planner; STANDBY'e geçer) · `Durdur` (SONIC stop, her zaman tek tık, en üstte, kırmızı) · `Kalibre et ve devral` (ACTIVE) · `Duraklat/Devam` · durum: SONIC bağlantısı, mod, encoder modu, son komut yaşı |
| **Hareket** | Hız ölçeği (0.3–1.0) · heading kilidi · heading dead zone (10/20/30°) · çömelme açık/kapalı · eğilme kazancı (0.5/0.75/1.0) |
| **Eller** | Kol ölçeği k · el modu: otomatik / sol park / sağ park / iki el park · parmaklar açık/kapalı |
| **Görüntü** | Robot görüntüsü: ego / immersive / kapalı · panel mesafesi |
| **Sistem** | Recenter sonrası yeniden kalibrasyon · kayıt başlat/durdur · gecikme ve oran göstergeleri (kafa, el, kontrolcü, kamera) |

**Kurallar:**
- Panel ayarları `configs/fullbody.yaml` varsayılanlarının üstüne çalışma zamanında yazılır ve log'a girer.
- Kalıcı varsayılanlar sadece config'ten değişir.
- Kısa yol butonları (§6) panelden bağımsız çalışır. `Durdur` hem panelde hem bir kontrolcü kombinasyonunda vardır (iki grip + tetik 0.5 s).

## 7. Simülasyon ortamı

**Kaynaklar** (`configs/sonic_assets.yaml`, sha256 + commit manifest'i; repoya kopyalanmaz):
- SONIC kaynağı: `/home/aksoy-msi/code/groot_n17_apple_wbc/third_party/GR00T-WholeBodyControl` @ `a0732b642c0333077e127a2f56ab0014c196bca4`
- Modeller: `humanoid-lab-main/data/models/sonic/sonic_v1_1/{model_encoder.onnx, model_decoder.onnx, observation_config.yaml}`
- Planner: `humanoid-lab-main/data/models/sonic-isaac/planner/target_vel/V2/planner_sonic.onnx`. Yolda `V2` olmalı.
- Referans klibi: `humanoid-lab-main/data/models/sonic-isaac/reference/example`. Planner modunda da başlangıçta gerekiyor.
- Deploy binary: `humanoid-lab-main/data/models/sonic-deploy/g1_deploy_onnx_ref` + `lib/` (x86, TensorRT 10.9, ORT 1.20.1). P1'de doğrulandı: DDS domain'i **42** (binary'ye derlenmiş; sim buna uyar, binary değişmez) ve `planner` mesaj formatı a0732b6 ile aynı. Başlık sabit 1280 bayt; upstream Python docstring'indeki 1024 eskimiş.

**Python ortamı:** `data/sonic-sim-venv`, Python 3.10, uv. Sadece simin import ettiği paketler kurulur:
- mujoco, numpy 1.26.4, scipy, pyyaml, pyzmq, msgpack, msgpack-numpy, opencv-python-headless, cyclonedds 0.10.2
- `pin`, `tyro`, `pygame`, `glfw` (mujoco zaten çekiyor), `loguru`, `termcolor` gerekmiyor; sim yolu import etmiyor (P1).
- `gear_sonic` ve pinli kaynaktaki `unitree_sdk2_python` `.pth` ile yol olarak eklenir. Editable kurulum upstream ağaca egg-info yazardı.

Torch kurulmaz; sim yolu torch import etmiyor, doğrulandı. Kurulum listesi `envs/sonic-sim.freeze.txt` dosyasına yazılır.

**Sahne** (`sim/scene_builder.py`, MjSpec ile upstream sim sahnesi `scene_43dof.xml` yüklenir; içinde `g1_29dof_with_hand.xml` var, zemin, ışık ve sürtünme upstream sim2sim ile aynı kalır; kopya yok):
- **Stereo kamera:** torso_link'e, ZED 2i montaj pozunda, 120 mm baz ile, 960×540 göz başına. Montaj pozu `g1_zed_mount` CAD'inden ya da ZED kalibrasyonundan alınır.
- **Parkurlar** (`configs/sim_courses.yaml`): flat; rampa 5°/10°/15° çıkış ve iniş; yan eğim 5°; basamak 3/6 cm; pürüzlü heightfield (2 cm); iki masa ve bir küp (5 cm, 0.1 kg).
- **El kararlılığı:** upstream, 43 DoF el modelinin kararsız olduğunu söylüyor. P1: MuJoCo 3.15'te kararsızlık görülmedi, kontak/sönüm ayarı gerekmedi.

**7a. Performans hedefleri** (P1'de ölçülür, P8'e kadar regresyon olarak izlenir; `tools/sim_perf.py`):
- Fizik gerçek zamanlı: RTF ≥ 1.0; 0.005 s adım, adım başına süre p99 < 2.5 ms.
- Görüntüleyici ≥ 60 fps. Upstream varsayılanı 50 Hz; config ile yükseltilir.
- Robot kamerası 30 fps, 2×960×540. Render ve JPEG kodlama fizik döngüsünün dışında olmalı.
- Bunları sağlamak için:
  - offscreen render GPU'da (EGL);
  - render ayrı bir thread ya da süreçte, MjData kopyası üzerinde, upstream sim API'sinin izin verdiği şekilde;
  - JPEG kodlama sim'in kamera alt sürecinde, upstream formatında (`ImageUtils`); `camera_bridge` (P5) çözer ve stereo formatına çevirir;
  - sahnede gereksiz kontak çifti yok;
  - mesh'ler sadeleştirilmiş görsel/çarpışma ayrımıyla.
- Ölçülen RTF ya da fps hedefin altına düşerse ilgili faz kapanmaz.
- **GPU güç bütçesi (P1, çözüldü):** dGPU uykudan başarısız dönüşte 20 W'ta kalmıştı. O halde kameralar açıkken SONIC'in gözlem→komut adımı ~28 ms (bütçe 20 ms) ve robot düşüyordu. Yeniden başlatmadan sonra 145 W, gözlem→komut p99 3.2 ms. Sim açılışta güç limitini kontrol eder ve < 50 W'ta uyarır (`sim/gpu.py`, `P1_sim.md` §3a).

**Başlatma:** `sim/run_sonic_sim.py`, `SimLoopConfig.load_wbc_yaml` + `SimulatorFactory.create_simulator` ile upstream `DefaultEnv`'i kurar (ROBOT_SCENE, DOMAIN_ID); döngü bizimdir. Upstream'in `BaseSimulator.start()` döngüsü kullanılmaz: kamera render'ı ve viewer senkronu fizik adımının içinde, uyku göreli, dolayısıyla RTF < 1. `camera_configs` upstream'e verilmez; kameralar ve viewer bizim alt süreçlerimizde, durum anlık görüntüsünden render eder. Upstream dosyalarına dokunulmaz.

## 8. Kayıt ve replay

- fullbody_node her döngüde şunları yazar: kullanılan `XRFrame`, `FullBodyCommand`, supervisor durumu ve olaylar (buton kenarları, mod geçişleri). Format: `data/fullbody-runs/<zaman>/run.msgpack` (msgpack akışı) + `meta.json` (git sha, config hash'i, asset manifest'i).
- `tools/fullbody_replay.py` bir log'u node'a başsız olarak besler ve çıkan komutları kayıttakilerle karşılaştırır. Bu bir regresyon testidir; P2'den itibaren kullanılır.
- Robot durumu (bilek hatası, düşme) için `tools/fullbody_metrics.py` sim DDS `rt/lowstate` ve `rt/odostate`'i dinler, FK ile bilek pozunu hesaplar ve hedefle karşılaştırır. Kontrol yolunda değildir, sadece ölçüm içindir.

## 8a. Mevcut altyapı ve P0 ilk bulguları (2026-11-12)

**Altyapı (hazır):**
- **Ağ:** MSI ve Quest aksoy-lab Wi-Fi'ında. MSI `192.168.1.105`'te; aksoy-lab'ın otomatik bağlanma önceliği eduroam'dan yüksek. Quest `192.168.1.106`'da.
- **Kısa adres:** Quest'te `https://192.168.1.105` yeterli. systemd `quest-https-redirect` servisi 443'ü 8012'ye yönlendiriyor. Vuer 0.1.6 `?reconnect=` sorgusunu okumuyor; `operator/quest/vuer_app.py` sayfaya bir WebSocket sarmalayıcısı enjekte ediyor, böylece sunucu ne kadar kapalı kalırsa kalsın sayfa VR'dan çıkmadan yeniden bağlanıyor (P3).
- **ADB:** Kablosuz, `192.168.1.106:5555`. Kendi ADB sunucumuz port 5039'da (`QUEST_ADB_PORT`); 5037'deki başka kullanıcı sunucusuna dokunulmuyor. udev kuralı Quest USB'sini sadece `emin`'e veriyor.
- **`deploy/quest_adb.sh`:** `keepalive`, `watch`, `open`, `shot`, `status`. systemd `quest-keepalive` servisi `watch` çalıştırıyor: ekran kapanmıyor, yakınlık sensörü yok sayılıyor, Quest yeniden başlarsa ayarlar geri uygulanıyor. Kablosuz ADB'yi yeniden açmak için USB kablosu takılı olmalı.
- **Bilinen eksik:** `screencap` VR katmanını siyah veriyor. Quest'e özgü yakalama yolu P5 araçlarında çözülecek.

**P0 ilk ölçüm** (`tools/quest_input_probe.py`, Vuer 0.0.60, stereo test görüntüsü açık):
- Kafa (CAMERA_MOVE) 20–44 Hz, düzensiz. 50 Hz kontrol için yetersiz. **Karar (P3):** Vuer 0.1.6 `Head` bileşeni (her XR karesinde örnekler, tarayıcı zaman damgalı), ayrı venv `data/quest-env`. Hız headset'in render hızıyla sınırlı (§6).
- Kontrolcü (CONTROLLER_MOVE) 90 Hz istendi, ~40–47 Hz geldi. Yürüme için yeterli.
- Kontrolcü eldeyken HAND_MOVE event'leri geliyor ama içleri boş.
- **Henüz ölçülmedi:** "bir el kontrolcüde, diğeri takipte" (D8). Kullanıcının 5 dk'lık headset kaydını (`deploy/xr_frame_record.sh`) bekliyor.

## 9. Fazlar

Her fazın çıktısı birim testleriyle ve bir commit ile kapanır.

**Doğrulama ilkesi (kullanıcı kararı, 2026-11-12):** Hiçbir faz kullanıcının headset denemesini beklemez. Her çıkış kriteri insansız doğrulanır: sim + script girdileri, headless Chrome Meta headset emülatörü (`tools/quest_emulator.mjs`) ve gerçek kaydın (`data/fullbody-runs/20261112-195840`) replay'i. Kullanıcı sistemi bitince bir kez, baştan sona §0 senaryosuyla dener. Aşağıdaki "headset ile" geçen çıkışlar emülatör/replay ile karşılanır; §0'ın kendisi kullanıcının denemesidir.

### P0 — Quest girdi probu (headset gerekir; P1 ile paralel) — kısmen tamam
- `tools/quest_probe.py` genişletilir: CAMERA_MOVE oranı ve gecikmesi, CONTROLLER_MOVE alanları, taraf başına el↔kontrolcü geçişi, bir kontrolcü + bir el aynı anda, recenter/reset olayı. Hepsi stereo görüntü açıkken ölçülür.
- **Çıkış:** ölçüm tablosu `docs/quest-sonic/P0_quest_probe.md`'ye yazılır.
- **El modu kararı:** karışık mod (taraf başına) çalışıyor mu? Çalışmıyorsa yedek: iki kontrolcü ile yürüme, kontrolcüler bırakılınca iki el. Bu durumda tasarım değişmez, sadece D8'deki eşleme değişir.

### P1 — Simülasyonu ayağa kaldırma — tamam (kameralar kapalı)
- `envs/sonic-sim.freeze.txt` ve venv; `configs/sonic_assets.yaml` + doğrulama script'i; `sim/scene_builder.py` (flat + kamera); `sim/run_sonic_sim.py`.
- Upstream deploy klavye ile denenir.
- Ölçülecekler: modların hız aralıkları, squat ve kneel yükseklik aralığı, DDS domain'i, el modeli kararlılığı.
- **Çıkış:** klavye ile 10 dk yürü, dön, squat; düşme yok. §7a performans hedefleri karşılanır (kameralar açıkken dahil). Ölçümler `docs/quest-sonic/P1_sim.md`'ye yazılır.

### P2 — SONIC adaptörü ve başsız node (script ile girdi) — tamam
- `sonic/protocol.py`: upstream `zmq_planner_sender.py` çıktısıyla bayt bayt karşılaştıran golden test (P1'de yapıldı).
- `sonic/adapter.py`, `sonic/lifecycle.py`, `fullbody/types.py`, `fullbody/supervisor.py`, `fullbody/node.py` (girdi kaynağı soyut: Quest ya da script).
- Script senaryoları: kare yürüyüş, yerinde 180° dönüş, bilek daireleri, gövde eğme, squat.
- **Çıkış:**
  - Senaryolar tekrarlanabilir.
  - VR modu aktif: bilek dairesi simde görülür ve ölçülür.
  - Node öldürülünce robot ayakta durur.
  - Replay testi geçer.

### P3 — Quest verisi — tamam
- `convert.py`'ye kontrolcü ve kafa çözümü eklenir; kafa rotasyon dönüşümü düzeltilir. `server.py` `xr_frame` yayınlar; Hands + MotionControllers birlikte açılır. `fullbody/xr_frame.py`, `fullbody/frames.py`, `fullbody/calibration.py`.
- **Çıkış:** Quest'ten 5 dk'lık xr_frame logu alınır; tüm kaynaklar zaman damgalı ve doğrulanmış. Frame testleri geçer: bilinen pozlar → beklenen sonuçlar.

### P4 — Quest ile yürüme ve yön (kollar park)
- `locomotion.py`, `heading.py`; supervisor buton eşlemesi.
- **Çıkış:**
  - Sekiz çizilir; işaretin 0.3 m içinde durulur (§0 notu: bu ≤ 0.5 m/s'de geçerli).
  - 60 s etrafa bakarken istemsiz dönüş yok.
  - 90° vücut dönüşünü robot ±5° ile izler.

### P5 — Robot gözüyle görüntü ve headset arayüzü
- `scene_builder` stereo kamera, `sim/camera_bridge.py`, `operator/quest/ui.py` (§6a), quest_server `/state` ve UiAction yayını.
- **Çıkış:**
  - Robot, panelden başlatılır ve durdurulur. Tüm sayfalar elle ve kontrolcüyle kullanılabilir; ui birim testleri (raycast, hover, basılı tutma onayı) geçer.
  - Kafa hareketinden görüntüye gecikme < 150 ms. Sim saati görüntüye yazılır, xr_frame zamanıyla karşılaştırılır.
  - P4 görevleri sadece robot görüntüsüyle yapılır.

### P6 — Kollar ve eller
- `arms.py` (ego eşleme operatör gövde frame'inde, park/takip, geometrik güvenlik), `hands.py`, `operator/hand_geometry.py` (wrapper ile ortak).
- **Çıkış:**
  - Ayakta bilek pozisyon hatası medyan < 3 cm, rotasyon < 10°.
  - Küp al-bırak 5/5.
  - Park↔takip geçişleri sarsıntısız.
  - Yürürken taşıma 3/3.

Bilek hatası yetmezse §11'e geçilir.

### P7 — Duruş
- `posture.py` (eğilme, çömelme, geçişler).
- **Çıkış:**
  - 20 yürü↔squat döngüsü düşmeden.
  - Bel limitine kadar eğilme kararlı.
  - Baş sallamak gövdeyi < 3° eğiyor.
  - 10 dk'da yanlış çömelme < 1.

### P8 — Parkur ve kabul
- Tüm parkurlar sahneye eklenir. `deploy/xr_fullbody_sim.sh` tek komutla açar.
- **Çıkış:**
  - §0 kabul senaryosu üç kez üst üste geçer.
  - Eğim ve basamak başarı oranları `docs/quest-sonic/P8_results.md`'ye yazılır.
  - `RUN.md`'ye kullanım bölümü eklenir.

## 10. Testler

- **Birim:** frames, calibration, heading (histerezis, kuantizasyon, açı sarması), locomotion, posture (θ ve Δh), arms (geçiş, güvenlik projeksiyonu), protocol (golden), adapter (sıralar, wxyz), supervisor (bayatlık, reset).
- **Entegrasyon:** script senaryoları ve replay regresyonu (simsiz çalışabilir; SONIC'e gitmeden komut karşılaştırması).
- **Sim içinde:** `fullbody_metrics.py` ile her fazın çıkış ölçümleri.

## 11. Riskler ve yedekler

| Risk | Belirti | Yedek |
|---|---|---|
| Mod 1 bilek hassasiyeti yetersiz | P6'da > 3 cm | `arms.py` çıktısı aynı kalır. `sonic/adapter.py`'ye q17 yolu eklenir: bel açısı posture'dan, kollar IK'dan (`g1_kinematics.ik`, komut edilen bel ile). Seçim config'ten yapılır. |
| Karışık el/kontrolcü modu yok | P0 | İki kontrolcü ↔ iki el modları (§9 P0) |
| VR yeniden kurma güvenilmez | P2 | lifecycle'da telemetriye dayalı doğrulama; gerekirse upstream'e issue |
| El modeli kararsız | P1 | Sahne üreticide kontak ayarları; olmazsa sabit (kinematik) parmaklarla, kavrama kaynaklı |
| Prebuilt binary ile kaynak uyumsuz | P1 | Pinli kaynaktan x86 build tarifi (`docs/quest-sonic/build_sonic.md`) |
| Eğimlerde düşme | P8 | Karakterizasyon olarak raporlanır; kabul kapısı sadece 5° |
| Squat aralığı beklenenden dar | P1: `height` 0.3–0.8 (pelvis 0.44–0.79) | Çömelme kademeleri ölçüme göre ayarlanır |
| dGPU 20 W güç kilidi | P1: kameralar açıkken politika adımı ~28 ms, robot düşüyor | Uykudan dönüş hatası; yeniden başlat. Sim açılışta uyarır (`sim/gpu.py`). |

## 12. Sonrası için tasarım garantileri (PC2 ve gerçek robot)

- **Süreçler mimariden bağımsız:** quest_server, fullbody_node ve camera kaynağı PC2'ye config değişikliğiyle taşınır. Gerçekte camera kaynağı `deploy/zed_stereo_stream.py`, SONIC ise PC2'deki mevcut kurulum (v1.1 ve zmq_manager doğrulanacak).
- **Kontrol yolunda MSI'a özgü bir şey yok:** GUI ve klavye kullanılmıyor.
- **Gerçek robot öncesi yapılacaklar** (bu planın dışında):
  - SONIC tarafında watchdog: 1 s timeout'un VR bayrağını temizlememesi.
  - PC2'de hesap bütçesi ölçümü.
  - "PC2 döngüde, sim MSI'da" testi.
  - Bel kilidi kontrolü.
  - Gantry testleri.

## 13. Açık sorular

1. G1'in bel roll/pitch eklemleri açık mı? Simde sorun değil; gerçek robotta duruş tasarımını etkiler.
2. Mevcut sadece kol modu (live_ui + ee_runtime) kalacak mı? Plan onu korur; kaldırılacaksa ayrı bir iş olur.
3. Son halde Quest PC2'ye hangi ağ üzerinden bağlanacak? P5'teki görüntü bant genişliği ölçümünü etkiler.

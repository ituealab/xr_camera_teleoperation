# P8 — Parkur ve kabul: sonuçlar

Tarih: 2026-11-13 (gece). Makine: MSI Raider 16 (GPU 145–150 W), SONIC `a0732b6` prebuilt deploy, MuJoCo 3.15,
kameralar açık. Doğrulama insansız: sentetik operatör akışları (`tools/xr_synth.py`), headless Chrome + IWER Quest
emülatörü puppet modunda (`tools/quest_emulator.mjs` + `tools/quest_puppet.py`) ve replay. Ham sonuçlar
`data/p8/`, `data/sonic-p1/starts-gantry.json`, `data/fullbody-runs/<zaman>/` (gitignore). Sabah denemesinin
kılavuzu: `SABAH.md`.

## 1. Çalıştırma

```bash
deploy/xr_fullbody_sim.sh morning [--no-viewer]     # tüm yığın, sabah parkuru, viewer masaüstünde; idempotent
deploy/xr_fullbody_sim.sh status | stop [NAME]

data/xr-env/bin/python -m xr_camera_teleop.tools.acceptance --runs 3                 # §0 kuru koşu (emülatör)
data/xr-env/bin/python -m xr_camera_teleop.tools.acceptance --only cube --runs 5     # küp al-bırak oranı
data/sonic-sim-venv/bin/python -m xr_camera_teleop.tools.course_test ramp5 --repeat 5          # parkur, sentetik operatör
data/sonic-sim-venv/bin/python -m xr_camera_teleop.tools.sonic_drive starts --count 20         # gantry
data/sonic-sim-venv/bin/python -m xr_camera_teleop.tools.sim_perf --duration 60                # §7a
```

Yığın `emin` kullanıcısıyla çalışır (sim venv'inin yorumlayıcısı emin'in evinde). Masadaki kullanıcı:
`sudo -u emin -H deploy/xr_fullbody_sim.sh morning`.

## 2. Gantry: bant bırakılmadan önce indirilir (`sim/gantry.py`)

**Sorun (P5):** bant pelvisi 0.965 m'de tutuyordu, robot ayakta 0.786 m. Bırakılınca 18 cm düşüyordu: 4 başlatmada bir
düşme, ayakta kalanlarda iniş eğimi 16°.

**Çözüm:** SONIC kontrolü aldıktan 0.5 s sonra bandın çapa yüksekliği (upstream `ElasticBand.length`) 1.5 s'de
yarım-kosinüsle ayakta pelvis yüksekliğine (0.79 m) iner; ayaklar yükü alır. Sonra bant birden kapatılmaz: sertliği ve
sönümü (upstream'in `kp_pos/kd_pos/kp_ang/kd_ang` değerleri × güç) 1.0 s'de sıfıra iner, ardından `9` ile kapanır.
İkinci adımın nedeni: SONIC indirilmiş bantta her zaman 0.786 m'de oturmuyor (0.77 m görüldü); sert bir bant altından
çekilince robot biraz düşüp adım atıyordu (sabit yükseklikte bırakmada bir koşuda 23 cm kayma). Upstream'e
dokunulmadı; değerler `configs/sonic_sim.yaml` `gantry`.

**Ölçüm** (`sonic_drive starts --count 20`: başlat → ayakta 8 s izle → stop → yeniden açılma ~28 s → tekrar):

| Ölçüm | min | medyan | maks |
|---|---|---|---|
| Düşme | **0 / 20** | | |
| İniş eğimi (bant gevşemeye başladıktan sonra) | 2.8° | 4.3° | **5.2°** |
| Son 2 s eğim (duruş) | 1.5° | 3.0° | 5.0° |
| Ayakta pelvis z | 0.772 | 0.785 | 0.788 m |
| Bırakıştan sonra kayma | 0.9 | 2.9 | 6.7 cm |
| Asılıyken eğim (SONIC'in açılış hareketi, bant tutuyor) | 5.2° | 10.9° | 12.8° |

P5'te 16° olan iniş eğimi en fazla 5.2°; ayakta duruşun kendisi ~3°. Asılıyken görülen ~11°, SONIC kontrolü alırken
bacakları varsayılan poza sürmesinden; robot bantta, iniş değil.

## 3. Parkurlar (`configs/sim_courses.yaml`, `sim/courses.py`)

Her engel kendi çerçevesinde tanımlı (x: üstüne yürünen yön, yakın kenar x = 0; y ortalı); parkur bir yerleşim
listesi. Sahne üretici parçaları robottan **sonra** ekler: upstream `DefaultEnv` qpos'u eklem kimliğiyle adresliyor,
robotun indeksleri değişmez (test). Küp serbest gövde; masa, rampa, basamak statik kutular; pürüzlü zemin ve yan eğim
heightfield (kenarlarda sıfır, dudak yok). İşaretler yalnız görsel (kontak yok).

| Engel | Tanım |
|---|---|
| ramp5 / ramp10 / ramp15 | 15 cm yükselir, 1.2 m düzlük, iner; eğim boyu 1.71 / 0.85 / 0.56 m; 2 m geniş |
| cross5 | 4 m şerit, yana 5° (sol kenar 17.5 cm, orta 8.7 cm), boyuna 1.2 m'de yumuşak giriş/çıkış; 2 m geniş |
| steps3 / steps6 | 2 basamak çık (40 cm basış), 0.8 m sahanlık, 2 in; rise 3 / 6 cm; 2 m geniş |
| rough2 | 2.4 × 2.0 m, 0–2 cm tümsekler (6 cm ölçek, kenarda 0.2 m'de sıfıra iner) |
| table | 0.5 × 0.8 m, üst 0.82 m (0.92 denendi, geri alındı: §6) |
| cube | 6 cm, 0.1 kg, sürtünme 2.0, condim 4 |

**Sabah yerleşimi (`morning`):** robot başlangıçta +x'e bakar. Düz bir koridor, 1.5 / 4.1 / 6.7 / 9.3 m'de sarı
işaretler. Her işaretin 0.7 m solunda bir şerit (5°, 10°, 15° rampa, yan eğim), sağında (masa A + küp ve masa B,
3 cm basamak, 6 cm basamak, pürüzlü zemin). İşarette dön, yürü.

**Neden 2 m geniş:** sentetik operatör stick'i düz tutup hiç düzeltmeden 6 m yürüyünce robot yana 0.4–0.8 m kayıyor
(açık çevrim, yön hatası birkaç derece). 1.2 m'lik rampada geri dönüşlerde robot 15 cm'lik yan kenardan düştü (3 koşuda
2 düşme, hepsi kenardan). Rampalar 2 m'ye genişletildi, karakterizasyon her başlatmada tek geçiş (düzeltme yapmayan
operatör için dönüş yolu merkezden başlamıyor). İnsan operatör görüntüye bakarak çapraz stick ile düzeltir.

**Masa ve küp neden değişti (sahne tarafı ayar; robot ve SONIC değişmedi):** grasp laboratuvarı (SONIC VR bileği +
Dex3, sim'den gerçek değer):
- Masa 0.78 m'de küp kolun erişiminin sonundaydı (omuz → bilek 0.35 m); SONIC bileği son 2–3 cm'yi indiremiyor.
- Dex3'ün iki parmağı merkezden ~5.7 cm aralıklı. Avuç dikken 5 cm küpte üst parmak ve başparmak küpün üstünde,
  alt parmak masaya değiyor; avuç aşağıyken iki parmak 5 cm küpün iki yanından geçiyor, bastırmıyor.
- Masa 0.82 m ve küp 6 cm ile **avuç aşağı** tutuş (küp avucun altında, başparmak arkada, parmaklar önünde, yumruk)
  küpü 5–9 cm kaldırıp masaya geri koydu. Pinch ile tutmuyor (başparmak yeterince dönmüyor). PLAN §7: 5 cm.

## 4. Parkur karakterizasyonu (`tools/course_test.py`, sentetik operatör, gerçek hat)

Sentetik operatör (`xr_synth p8_course`): kalibre + başlat (A basılı), tam stick ileri 16 s, bırak. Gerçek
QuestSource → node → SONIC → MuJoCo; her koşu replay'de birebir. Geçiş başarılı: robot engelin tamamını (çık, düzlük,
in) aştı, düşmedi, yandan çıkmadı. Her koşu başlangıçtan tek geçiş.

| Parkur | Başarılı geçiş | Düşme | Not |
|---|---|---|---|
| **ramp5** (kabul kapısı) | **5 / 5** | **0** | Çıkış, düzlük, iniş; en büyük eğim 7.1°, yana kayma ≤ 0.61 m (2 m şeritte) |
| ramp10 | 3 / 3 | 0 | en büyük eğim 8.0° |
| ramp15 | 2 / 3 | 0 | Kalan koşu 16 s'de inişi bitiremedi (yokuşta yavaş), düşmedi; eğim ≤ 10.2° |
| steps3 | 3 / 3 | 0 | Bir koşuda 43° yalpalayıp toparladı, diğerleri ≤ 6.4° |
| steps6 | 1 / 3 | **2** | İkisi ilk basamakta düştü (biri yandan). 6 cm stok SONIC için fazla |
| rough2 | 3 / 3 | 0 | eğim ≤ 6.6° |
| cross5 | 0 / 3 (3 / 3 düşmeden geçti) | 0 | Hepsi aştı ama yokuş aşağı 1.1–1.25 m kaydı ve alçak kenardan şeritten çıktı (alçak kenar zemin hizasında, tehlike yok). Stok planner yan eğimde yönünü tutmuyor |

Kabul kapısı yalnız 5°'lik rampa (PLAN §11): geçti. Diğerleri bilgi içindir. Kabul kuru koşusunda (§7) rampa ayrıca
emüle edilmiş operatörle, kapalı çevrim yönle 3 kez çıkılıp inildi.

## 5. Performans (§7a), tam sahne

`sim_perf --duration 150`, sabah sahnesinin tamamı (rampalar, basamaklar, iki heightfield, masalar, küp), kabul kuru
koşusu sırasında: robot yürüyor, eğiliyor, çömeliyor, elleri çalışıyor; aynı GPU'da emülatörün Chrome'u da var.

| Hedef (§7a) | Ölçülen | Durum |
|---|---|---|
| Fizik RTF ≥ 1.0 | 1.000, lag reset 0 | geçer |
| Adım p99 < 2.5 ms | 1.40 ms (pencere medyanı 1.09, en kötü adım 2.86, ortalama 0.81) | geçer |
| `rt/lowstate` 200 Hz | 200 Hz, varış p99 5.4 ms, sim/duvar saati 1.000 | geçer |
| Robot kamerası 30 fps × 2 | 30.0 fps her göz, düşen kare 0; render+kodlama p99 13 ms, durum yaşı p50/p99 5.8/10.3 ms | geçer |
| SONIC gözlem→komut < 20 ms | medyan 0.53, p99 1.49, maks 1.67 ms; politika p99 1.18 ms | geçer |
| GPU | limit 145 W, ~2.0 GHz, 46 W, kısma nedeni yok | — |
| Viewer ≥ 60 fps | kuru koşu `--no-viewer`; masaüstünde ayrıca ölçüldü (§8) | — |

P1'e göre heightfield'lar ve küp adım süresini ölçülebilir değiştirmedi (P1 yürürken p99 1.38–1.41 ms).

## 6. Küp al-bırak (ELLER modu, gerçek hat)

**Park pozu düzeltildi (`P4_P6_P7_mapping.md` §4 Kollar).** Eski park pozu bilekleri pelvisin 0.16 m önüne, pelvis
yüksekliğine koyuyordu; eller 0.36 m öne, 0.64–0.87 m'ye uzanıyordu. Bu kullanıcının D8 tanımına ("kontrolcü eldeyken
kollar yanda dursun") aykırıydı ve küp sorununun ana nedeniydi: robot masaya küpe uzanacak kadar yaklaşınca park eden
eller masa üstünün altına giriyor, ELLER'e geçince kalkamıyordu (o koşularda el 10 cm kalkınca robot bileği 0.1–3.4 cm
kalktı; kuru koşu 2'deki el aynalama hatası da buydu: sol el de masa A'nın altındaydı). Yeni poz: omuz pitch 0, roll
±0.40, dirsek 1.40 (FK): bilekler 0.74 m'de uyluğun yanında, eller pelvisin en fazla 0.11 m önünde, yürürken
uyluğa ≥ 3.8 cm. Simde ramp5 4/4 düşmesiz; AKTİF YÜRÜME'de el-bacak mesafesi hiç 1 cm'nin altına inmedi (1 cm altı
yalnız SONIC kapalıyken, bantta asılı: SONIC'in kendi duruşu).

**Bileğin kalkamaması, kök neden ve test:** (1) park pozu masanın içindeydi (yukarıda); (2) yan park pozundan bile düz
bir park→takip karışımı parmakları yeterince yükselmeden masa kenarının altına süpürüyordu. `arms.py` rampaları artık
hedef yüksekse önce yüksekliği, alçaksa önce yatayı karıştırıyor. Testler: `test_fullbody_park_pose.py` (FK ile:
el masa ayak izinin dışında, uyluğa mesafe; eski pozla kalır) ve `test_the_hand_never_travels_forward_while_low` (eski
düz karışımla kalır, denendi). Sonuç: el aynalama park düzeltmesinden sonraki 35 masa koşusunun hepsinde geçti (robot bileği 5.8–8.4 cm kalktı).

**Kalan sorun: tutuşun kendisi.** Park düzeltmesinden sonra emüle edilmiş operatörle tam hat (`acceptance --only cube`,
5'er koşu, her koşuda en çok 3 deneme):

| Değişiklik (operatör tarafı, birikimli) | Koşu başarısı | Not |
|---|---|---|
| park pozu + rampa | 0 / 5 | Operatörün dinlenme elleri robotun elini küp hizasına koyuyordu: ELLER'e geçince el küpü 19 cm öteye itti |
| dinlenme elleri 10 cm yukarıda (masanın üstünde) | 2 / 5 | Kalan üçünde robot 0.20–0.24 m'ye kadar yaklaşmıştı |
| yumruk 2.5 s, yavaş kaldırma | 2 / 5 | |
| durma noktası 0.36 m | 2 / 5 | Kısa yaklaşmada robot 0.03–0.15 m fazla gidiyor (SONIC ≥ 0.4 m/s) |
| çok yakınsa kısa geri adım | 0 / 5 | 0.3 s'lik geri darbe robotu yeterince geri almıyor |
| küp el çerçevesinde 1.0 cm işaret parmağı tarafına, 0.5 s geri darbe, en az 0.30 m | 1 / 5 | Başarılı koşu ilk denemede tuttu (küp 0.8 cm'de); 0.5 s'lik geri darbe robotu 0.46–0.52 m'ye, menzil dışına attı |
| son ayar: 1.0 cm, geri adım yok, durma 0.36 m (iki kuru koşu, §7) | 1 / 6 | Biri ilk denemede; kalanlarda küp robotun solunda ya da menzil dışında kaldı |

Kayıtlardan çıkan koşullar: küp robotun **≥ 0.29 m önünde** değilse avuç-aşağı bilek küpün üstüne gelemiyor (servo
4–10 cm'de takılıyor); geldiğinde (servo ≤ 0.5 cm) tutuş küp el çerçevesinde işaret parmağı tarafına 1.0–1.6 cm
kaymışsa tuttu (5.0–6.5 cm kaldırıp 10–13 cm taşıdı), 2.2–2.6 cm'de tutmadı. Başarılı tutuşların hepsi 2. ya da 3.
denemede geldi. Grasp lab'da (SONIC VR bileği doğrudan) 3 / 3; avuç dik, pinch ve 5 cm küp hiç tutmadı; masa 0.92 m'de
de tutmadı (0 / 4).

İnsan operatör için: küp robotun ~30–35 cm önünde olacak şekilde dur; elleri önce masanın üstüne kaldır; avuç aşağı,
küpün üstüne otur; yumruk, ~2 s tut, yavaş kaldır; tutmazsa tekrar (`SABAH.md`).

## 7. §0 kabul kuru koşusu (`tools/acceptance.py`)

`tools/acceptance.py --runs 3`, sabah parkuru, taze yığın, üç koşu art arda (her koşu Başlat'tan Durdur'a; arada
SONIC yeniden açılır, gantry robotu ve küpü sıfırlar). Koşu 2'de bağlantı kesme sayfa + Quest sunucusu durdurularak
yapıldı, sunucu `morning` ile geri geldi. Yeni park pozu ve son operatör ayarlarıyla iki kez koşuldu.

**Son koşu** (`data/p8/acceptance-20261113-060353/`):

| Adım | Koşu 1 | Koşu 2 | Koşu 3 |
|---|---|---|---|
| Sayfa + robot görüntüsü (2 s'de çözülen resim, görüntü Hz) | 133, 30.0 | 120, 29.5 | 124, 30.0 |
| Başlat → HAZIR | 0.31 s | 0.3 s | 0.31 s |
| A basılı tutarak kalibrasyon → AKTİF | geçti | geçti | geçti |
| Panelden kalibrasyon → AKTİF | geçti | geçti | geçti |
| Stick ileri/geri/sol/sağ; işarette durma (≤ 0.3 m) | 0.21 m | 0.20 m | 0.08 m |
| 90° vücut dönüşü (±5°); ±15° bakış | -89.9°; 0.57° | **kaldı** (robot turned -83.1 deg for a -90 deg body turn) | **kaldı** (robot turned -84.5 deg for a -90 deg body turn) |
| Eğilme / squat pelvis / kalkış pelvis | 12.1°, 0.597, 0.782 m | 11.7°, 0.596, 0.783 m | 11.4°, 0.597, 0.783 m |
| Masaya yürü (küp önde), ELLER, el aynalanıyor | 0.31 m; 6.5 cm | 0.32 m; 5.9 cm | 0.39 m; 6.6 cm |
| Küp al-bırak (deneme, kalkış) | **kaldı** (cube not lifted in 3 attempts (last 1.9 cm)) | **kaldı** (cube not lifted in 3 attempts (last 0.4 cm)) | **kaldı** (cube not lifted in 3 attempts (last 1.1 cm)) |
| Kontrolcüler geri → YÜRÜME, koridor, 180° | 176.7° | 178.3° | 177.1° |
| 5° rampa çık-in (pelvis maks, eğim maks) | 0.930 m, 9.8° | 0.931 m, 9.2° | 0.925 m, 10.5° |
| Bağlantı kesme: robot ayakta, kayma | 0.021 m | 0.021 m (sunucu da) | 0.023 m |
| Durdur → SONIC bıraktı, gantry yakaladı | 0.0 s | 0.0 s | 0.1 s |
| **Düşme** | **0** | **0** | **0** |

**Bir önceki** (`data/p8/acceptance-20261113-055143/`; koşu 3'teki rampa ve bağlantı hataları ölçüm aracının
hatasıydı: canlı durum okuyucusu vazgeçti, adım stick basılıyken koptu; düzeltildi, sonraki koşu bunu göstermiyor):

| Adım | Koşu 1 | Koşu 2 | Koşu 3 |
|---|---|---|---|
| Sayfa + robot görüntüsü (2 s'de çözülen resim, görüntü Hz) | 135, 30.0 | 124, 30.0 | 120, 29.5 |
| Başlat → HAZIR | 0.31 s | 0.3 s | 0.3 s |
| A basılı tutarak kalibrasyon → AKTİF | geçti | geçti | geçti |
| Panelden kalibrasyon → AKTİF | geçti | geçti | geçti |
| Stick ileri/geri/sol/sağ; işarette durma (≤ 0.3 m) | 0.17 m | 0.16 m | 0.20 m |
| 90° vücut dönüşü (±5°); ±15° bakış | -86.6°; 1.63° | -87.1°; 1.39° | -88.2°; 1.17° |
| Eğilme / squat pelvis / kalkış pelvis | 11.3°, 0.596, 0.784 m | 11.4°, 0.594, 0.783 m | 12.2°, 0.596, 0.782 m |
| Masaya yürü (küp önde), ELLER, el aynalanıyor | 0.27 m; 6.0 cm | 0.35 m; 6.8 cm | 0.29 m; 6.0 cm |
| Küp al-bırak (deneme, kalkış) | 1. denemede, 6.2 cm | **kaldı** (cube not lifted in 3 attempts (last 0.7 cm)) | **kaldı** (cube not lifted in 3 attempts (last 2.4 cm)) |
| Kontrolcüler geri → YÜRÜME, koridor, 180° | 176.0° | 177.6° | 174.0° |
| 5° rampa çık-in (pelvis maks, eğim maks) | 0.931 m, 9.2° | 0.928 m, 11.1° | **kaldı** (the sim's state snapshot has no complete record) |
| Bağlantı kesme: robot ayakta, kayma | 0.020 m | 0.022 m (sunucu da) | **kaldı** (robot moved 0.20 m after the disconnect) |
| Durdur → SONIC bıraktı, gantry yakaladı | 0.0 s | 0.11 s | 0.0 s |
| **Düşme** | **0** | **0** | **0** |

**Kapı:** §0 "üç kez üst üste" geçmedi. Bir koşu (05:51 koşu 1) baştan sona her adımı geçti, küp ilk denemede
6.2 cm kalktı. Hiçbir koşuda düşme yok. Kalanlar:
- **Küp** (6 koşuda 1): tutuş için robot küpün ≥ 0.29 m arkasında ve küp sağ elinin önünde olmalı. Robot 90° dönüşü
  birkaç derece eksik bitirince (−83…−88°) masaya yürüyüşü küpü sol tarafına (+0.05…+0.10 m) ya da menzil dışına
  (0.385 m) getiriyor; emüle edilmiş operatör bunu masada düzeltmiyor (yan adım SONIC'te ≥ 0.4 m/s, ince ayar yok).
- **90° dönüş ±5°** (6 koşuda 4): −83.1° ve −84.5° (P4'ün açık çevrim yeniden-gönderme çaresi, risk 5).
- El aynalama artık 6/6 (park düzeltmesinden önce 2/3).

Emülatörün kafa hızı 60 Hz (gerçek Quest'te 36 Hz, P0); A tuşu yolu ve panel yolu ikisi de test edildi.

## 8. Viewer masaüstünde

Orkestratör yığını `emin` olarak, ekransız bir kabuktan başlatır. Launcher masadaki oturumun Xwayland'ini (`:0`) bulur:
`xhost +SI:localuser:emin` verilmişse doğrudan, değilse `emin`'in şifresiz sudo'su ile masadaki kullanıcının X
çerezini kendi dosyasına kopyalar (`~emin/.local/state/xr_fullbody/desk.xauth`, `stop` siler). Kullanıcının hiçbir
şey yapması gerekmez. Denendi (2026-11-13 04:05):

- `deploy/xr_fullbody_sim.sh morning` DISPLAY'siz kabuktan: "Viewer: the desk display :0 (with the desk user's X
  cookie)"; pencere `:0`'da açıldı ("MuJoCo : g1_43dof scene", 1706×1066), `xwininfo` ile görüldü, sonra `stop` ile
  kapandı.
- `sim_perf`: viewer senkron döngüsü **60 Hz (PASS)**, senkron p99 6.9 ms; fizik etkilenmedi (RTF 1.000, adım p99
  1.33 ms), kameralar 30 fps.
- Masaüstü Intel iGPU'da (Mesa, Arrow Lake): viewer orada render ediyor, RTX SONIC'e ve kameralara kalıyor. Sahne bu
  pencere boyutunda iGPU'da 5.4 ms/kare (~184 fps kapasite).
- Masadaki kullanıcı yığını kendi açamaz (sim venv yorumlayıcısı emin'in evinde); `sudo -u emin -H
  deploy/xr_fullbody_sim.sh morning|stop|status` çalışıyor (denendi: aksoy-msi → emin, `status`).

## 9. PLAN'dan ve faz belgelerinden sapmalar

1. **§0 madde 7 (yürürken taşıma) D8 ile geçersiz** (PLAN §0'da işaretlendi): kontrolcü alınınca iki kol park eder,
   parmaklar açılır. Kabul senaryosunda yok; karar kullanıcıda.
2. **Küp 6 cm** (PLAN §7: 5 cm) ve **masa 0.82 m**: Dex3 tutuşu için sahne ayarı (§3).
3. **Parkurlar 2 m geniş**, karakterizasyon tek geçiş (§3).
4. **Gantry** indirip yavaşça bırakıyor (P2 §1: "bandı indirir/kaldırır" anahtarla anında bırakıyordu).
5. **Park pozu değişti** (P4_P6_P7 §4 Kollar): kollar yanlarda sarkık (D8 tanımı); eski poz elleri öne uzatıyordu.
   **Kol rampaları yükseklik önce** (park ↔ takip): `arms.py` `_Ramp`.
6. **Yeni modüller:** `sim/courses.py` (parkur geometrisi), `sim/live.py` (çalışan sim'in gerçek değerleri,
   araçlar için; kontrol yolunda değil), `tools/course_test.py`, `tools/acceptance.py`; `sonic_drive starts`;
   emülatörde `grip` el pozu ve kalibrasyon için hareketsiz kafa penceresi; launcher'da `morning`, `stop NAME`,
   masaüstü ekranı bulma.
7. **Emülatör (P5 §5'teki not):** betik modunda A artık 1.5 s, kafa 9–12 s arası durgun: kalibrasyon yolu uçtan uca
   test ediliyor (`test_quest_e2e.py::test_node_follows_the_emulated_quest`).

## 10. Açık riskler

Yarınki denemeyi bozma olasılığına göre sıralı:

1. **Küp al-bırak (§0 madde 6) zor olacak.** Park sorunu çözüldü (kollar yanlarda, eller masaya takılmıyor), ama tutuş
   dar: küp robotun ≥ 0.29 m önünde olmalı, avuç küpün üstüne oturmalı, küp avucun işaret parmağı tarafında; emüle
   edilmiş operatörle tutuş çoğu zaman 2.–3. denemede geldi, en iyi ayarda 5 koşuda 2. İnsan görüntüye bakarak düzeltir;
   `SABAH.md` adımları söylüyor. Robot ≥ 0.4 m/s yürüdüğü için masaya kısa yaklaşmada 0.03–0.15 m fazla gidiyor.
2. **§0 madde 7 D8 ile imkânsız** (yürürken taşıma): kullanıcı kararı; PLAN'da işaretlendi.
3. **Gerçek headset emülatörden farklı:** kafa 36 Hz (60 değil), Quest'in JPEG çözmesi, Wi-Fi, A basılıyken
   kontrolcülerin akıştan düşmesi (tek gözleme dayanan kural, P4 §8). Kalibrasyon 1 s hareketsizlik ister; olmazsa
   panelden "Kalibre et ve devral".
4. **Rampadan yana kayma:** açık çevrimde 6 m'de 0.4–0.6 m; rampa 2 m geniş, kenar 15 cm. Kullanıcı çapraz stick ile
   düzeltmeli; 10°/15° rampa, 6 cm basamak (3'te 2 düşme) ve yan eğim (yokuş aşağı kayma) yalnız bilgi için.
5. **Yerinde dönüş açık çevrim:** 90°'de −84…−89° görüldü (bir kez ±5° dışında); 180°'de 170–178°.
6. **Yeni park pozunun gerçek headset'le denenmemesi:** simde yürüme ve rampa düşmesiz, el-bacak teması yok; YÜRÜME ↔
   ELLER geçişleri artık "önce yüksel" rampası ile, sarsıntı ölçümü (P6 tablosu) eski pozla yapılmıştı.
7. **Viewer:** masaüstü kilitliyse pencere kilidin arkasında; kullanıcı oturumu kapatıp açarsa Xwayland çerezi
   değişir: `stop` + `morning` yeniden bulur. Viewer kapanırsa fizik sürer.
8. **Quest kablosuz ADB'si** USB'yi bir kez takmayı gerektirir (orkestratör); sayfa açma ADB'siz de olur.
9. **Robot düşerse:** upstream ~20 s sonra robotu diker; ya da DURDUR (bant yakalar, küp de sıfırlanır) ve ~30 s
   sonra Başlat.

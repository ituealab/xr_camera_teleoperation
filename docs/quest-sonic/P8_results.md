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

**Sonuç: küp al-bırak insansız doğrulanamadı.** Tutuşun kendisi simde çalışıyor, ama emüle edilmiş operatörle tam hat
üzerinden güvenilir değil. Nedenleri aşağıda; en büyüğü D8 park pozu ile masanın çakışması (§10 risk 1).

| Deneme | Nasıl | Sonuç |
|---|---|---|
| Grasp lab, masa 0.82 m, 6 cm küp, avuç aşağı, yumruk | SONIC VR bileği doğrudan (node yok), sim'den kapalı çevrim | **3 / 3 kaldırdı** (9.3 / 8.2 / 5.1 cm), masaya geri koydu |
| aynı, pinch | | 0 / 2 (başparmak yeterince dönmüyor) |
| aynı, avuç dik | | 0 / 1 (küpü masadan düşürdü) |
| 5 cm küp (0.78 ve 0.82 m masa, avuç dik ve aşağı) | | 0 / 19 (Dex3 parmakları küpün iki yanından geçiyor ya da masaya değiyor) |
| masa 0.92 m (park eden eller altından geçsin diye) | | 0 / 4: bilek avuç-aşağı poza gelmiyor; geri alındı |
| **Tam hat**: emülatör (sayfa, Quest sunucusu, xr_frame) → node → SONIC → MuJoCo, operatör sim'e bakarak elini kapalı çevrimle götürür, yumruk (`grip`), en çok 3 deneme | `tools/acceptance.py --only cube` ve kuru koşunun küp adımı, son araç | **0 / 7 koşu** (21 deneme; en iyisi 3.2 cm kaldırıp düşürdü) |
| aynı, aracın önceki sürümleri (gece boyunca) | | 3 başarılı al-bırak (8.1 / 6.7 / 6.2 cm kaldırıp 10–13 cm taşıdı) |

Neden tutmuyor (kayıtlardan):
1. **Park eden eller masaya giriyor.** YÜRÜME'de eller robotun 0.36 m önüne ve 0.64–0.87 m yüksekliğe park ediyor;
   küpü almak için robot masa kenarına ~0.2 m yaklaşmalı, masa üstü 0.78–0.82 m. Eller masa üstünün hizasında/altında
   kalıyor; ELLER'e geçince bilekler kalkamıyor: bu koşularda sol el 10 cm kalkınca robot bileği 0.1–3.4 cm kalktı,
   sağ bilek hedefin 35–48 cm uzağında kaldı (son 7 koşunun 4'ünde). Masa yüksekte (0.92 m) eller altından geçiyor ama
   tutuş pozu olmuyor; alçakta erişim yetmiyor. Sahne tarafında temiz bir çözüm bulamadım; park pozu robotun
   davranışı (P4_P6_P7 §4), değiştirmedim. Karar kullanıcıda.
2. **Bilek oraya gelince bile tolerans dar:** küp el çerçevesinde ~1 cm doğrulukta olmalı; SONIC'in bilek yönü
   hatası (5–20°) küpü avuca göre kaydırıyor. Başarılı denemelerde küp el çerçevesinde (9–11, 5–6, 1–2) cm'deydi.
3. **Emüle edilen yumruk** IWER'in kendi pozlarından kuruldu (`point` parmakları, işaret parmağı da orta parmak
   şeklinde, `pinch` başparmağı): retarget gerçek yumruğa yakın (`[-1.13, -0.69, -1.54, 0.79, 1.61, 0.60, 1.37]`
   / kanonik `[-1.13, -0.69, -1.54, 0.51, 1.26, 0.61, 1.25]`), ama aynı değil.

İnsan operatör için anlamı: tutuş mümkün (lab), ama küp robotun sağ önünde ~30 cm'deyken avuç aşağı, küpün üstüne
oturtup yumruk gerekiyor; park eden eller masaya takılırsa geri çekilip tekrar yaklaşmak gerekiyor (`SABAH.md`).

## 7. §0 kabul kuru koşusu (`tools/acceptance.py`)

`tools/acceptance.py --runs 3`, sabah parkuru, taze yığın, üç koşu art arda (her koşu Başlat'tan Durdur'a; arada
SONIC yeniden açılır, gantry robotu ve küpü sıfırlar). Koşu 2'de bağlantı kesme sayfa + Quest sunucusu durdurularak
yapıldı, sunucu `morning` ile geri geldi. Sonuç: `data/p8/acceptance-20261113-034310/`.

| Adım | Koşu 1 | Koşu 2 | Koşu 3 |
|---|---|---|---|
| Sayfa + robot görüntüsü + panel (2 s'de çözülen resim, görüntü Hz) | 133, 30.1 | 124, 30.0 | 122, 30.0 |
| Başlat → HAZIR | 0.3 s | 0.3 s | 0.3 s |
| A basılı tutarak kalibrasyon → AKTİF | geçti | geçti | geçti |
| Panelden kalibrasyon → AKTİF | geçti | geçti | geçti |
| Stick ileri/geri/sol/sağ; işarette durma (≤ 0.3 m) | 0.12 m | 0.23 m | 0.03 m |
| 90° vücut dönüşü (±5°); ±15° bakış | −87.5°; 0.45° | −88.7°; 0.86° | −89.1°; 0.11° |
| Eğilme / squat / kalkış | 11.4°, 0.596, 0.785 m | 11.6°, 0.596, 0.784 m | 11.5°, 0.594, 0.785 m |
| Masaya yürü, ELLER, el aynalanıyor | geçti (8.7 cm) | **kaldı** (3.3 cm) | geçti (8.9 cm) |
| Küp al-bırak | **kaldı** | **kaldı** | **kaldı** (3.2 cm kalktı, düştü) |
| Kontrolcüler geri → YÜRÜME, koridor, 180° | geçti | geçti | geçti |
| 5° rampa çık-in (pelvis maks, eğim maks) | 0.933 m, 9.5° | 0.931 m, 10.2° | 0.934 m, 9.0° |
| Bağlantı kesme: robot ayakta, kayma | 0.007 m | 0.008 m (sunucu da) | 0.008 m |
| Durdur → SONIC bıraktı, gantry yakaladı | 0.0 s | 0.0 s | 0.1 s |
| **Düşme** | **0** | **0** | **0** |

**Kapı:** §0 "üç kez üst üste" geçmedi: küp adımı üç koşuda, masadaki el aynalama koşu 2'de kaldı; ikisinin nedeni
aynı (§6). Diğer her adım üç koşuda da geçti, hiç düşme yok. Önceki bir 3 koşuluk kuru koşu (emüle elin yönü
düzeltilmeden önce) aynı tabloyu verdi: küp 0/3, el aynalama 2/3, gerisi 3/3, düşme 0.

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
5. **Yeni modüller:** `sim/courses.py` (parkur geometrisi), `sim/live.py` (çalışan sim'in gerçek değerleri,
   araçlar için; kontrol yolunda değil), `tools/course_test.py`, `tools/acceptance.py`; `sonic_drive starts`;
   emülatörde `grip` el pozu ve kalibrasyon için hareketsiz kafa penceresi; launcher'da `morning`, `stop NAME`,
   masaüstü ekranı bulma.
6. **Emülatör (P5 §5'teki not):** betik modunda A artık 1.5 s, kafa 9–12 s arası durgun: kalibrasyon yolu uçtan uca
   test ediliyor (`test_quest_e2e.py::test_node_follows_the_emulated_quest`).

## 10. Açık riskler

Yarınki denemeyi bozma olasılığına göre sıralı:

1. **Küp al-bırak (§0 madde 6) büyük olasılıkla zorlayacak.** Park eden eller masa kenarına/altına giriyor; ELLER'e
   geçince kollar kalkamayabilir; tutuş toleransı ~1 cm (§6). Kullanıcıya söylenecek: avuç aşağı, küpün üstüne otur,
   yumruk; takılırsa kontrolcüyü al, geri çekil, tekrar. Kalıcı çözüm kullanıcı kararı: park pozunu alçaltmak/
   yanlara açmak, ya da masa/görev tasarımı.
2. **§0 madde 7 D8 ile imkânsız** (yürürken taşıma): kullanıcı kararı; PLAN'da işaretlendi.
3. **Gerçek headset emülatörden farklı:** kafa 36 Hz (60 değil), Quest'in JPEG çözmesi, Wi-Fi, A basılıyken
   kontrolcülerin akıştan düşmesi (tek gözleme dayanan kural, P4 §8). Kalibrasyon 1 s hareketsizlik ister; olmazsa
   panelden "Kalibre et ve devral".
4. **Rampadan yana kayma:** açık çevrimde 6 m'de 0.4–0.6 m; rampa 2 m geniş, kenar 15 cm. Kullanıcı çapraz stick ile
   düzeltmeli; 10°/15° rampa, 6 cm basamak (3'te 2 düşme) ve yan eğim (yokuş aşağı kayma) yalnız bilgi için.
5. **Yerinde dönüş açık çevrim:** 90°'de −84…−89° görüldü (bir kez ±5° dışında); 180°'de 170–178°.
6. **Viewer:** masaüstü kilitliyse pencere kilidin arkasında; kullanıcı oturumu kapatıp açarsa Xwayland çerezi
   değişir: `stop` + `morning` yeniden bulur. Viewer kapanırsa fizik sürer.
7. **Quest kablosuz ADB'si** USB'yi bir kez takmayı gerektirir (orkestratör); sayfa açma ADB'siz de olur.
8. **Robot düşerse:** upstream ~20 s sonra robotu diker; ya da DURDUR (bant yakalar, küp de sıfırlanır) ve ~30 s
   sonra Başlat.

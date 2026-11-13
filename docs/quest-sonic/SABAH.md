# Sabah denemesi: Quest 3S ile simülasyondaki G1

Tek deneme, baştan sona PLAN §0 senaryosu. MuJoCo'yu gerçek robot gibi düşün. Ayrıntılar ve ölçümler:
`P8_results.md`.

## 1. Başlatma

Sistemi orkestratör senin gelmenden önce açar. Masaüstünde MuJoCo penceresi açık olmalı: robot, önünde sarı işaretli
bir koridor, solda rampalar, sağda iki masa ve kırmızı küp. Robot havada, bantta asılı durur; bu normal.

Kendin açman ya da yeniden başlatman gerekirse (kendi terminalinden, şifre sormaz):

```bash
sudo -u emin -H /home/aksoy-msi/code/xr_camera_teleoperation/deploy/xr_fullbody_sim.sh morning   # aç (açıksa dokunmaz)
sudo -u emin -H /home/aksoy-msi/code/xr_camera_teleoperation/deploy/xr_fullbody_sim.sh stop      # her şeyi kapat
sudo -u emin -H /home/aksoy-msi/code/xr_camera_teleoperation/deploy/xr_fullbody_sim.sh status    # ne çalışıyor
```

`morning` sonunda üç şey yazar: Quest adresi, ilk ne yapılacağı, nasıl kapatılacağı. Pencere ekranında açılmadıysa
bunu da söyler (çare: kendi oturumunda bir kez `xhost +SI:localuser:emin`, sonra `stop` ve `morning`).

## 2. Quest

1. Quest'i aç.
2. USB kablosunu bir kez tak, orkestratöre söyle: kablosuz ADB'yi geri açar. Sonra kabloyu çıkarabilirsin.
3. Quest Browser'da `https://192.168.1.105` aç, **Enter VR**.
4. Robotun kafa kamerasından stereo görüntüyü görürsün (önünde 2 m'de bir pencere). Sol üstte sim saati yazar.

## 3. Kontroller

**İki mod (D8):**
- **YÜRÜME:** en az bir kontrolcü elinde. Stick yürütür. Robotun iki kolu yanlarda sarkar, parmaklar açık.
- **ELLER:** iki kontrolcüyü de bırak. Robotun bilekleri ellerini, Dex3 parmakları parmaklarını izler. Bacaklar durur.
- Geçişte Quest girdiyi 0.6–1.5 s keser; robot kısa bir an DURAKLADI'ya geçer, sonra kendiliğinden devam eder.

| Ne | Nasıl |
|---|---|
| Paneli aç | Herhangi bir thumbstick'e **bas** (kontrolcü) ya da **sol avucunu yüzüne çevir**, 0.4 s tut (el) |
| Panelde seç | Kontrolcü ışını + **tetik**, ya da el ışını + **başparmak-işaret pinch**. Robota giden her şey **0.6 s basılı tutma** ister (sarı halka dolar); DURDUR, sekmeler ve X hemen çalışır |
| Kalibre et ve devral | Dik dur, ileri bak, kıpırdama. **A veya X'i 1 s basılı tut**, ya da panelde **Kalibre et ve devral** |
| Duraklat / devam | A veya X kısa bas |
| Yön kilidi | B veya Y |
| Yürü | Stick (ileri, geri, yan). En fazla 0.5 m/s; bırakınca ~0.1–0.25 m'de durur |
| Dön | Vücudunla dön (kafa 25°'den fazla ve 0.3 s). Etrafa bakmak (±20°) robotu döndürmez |
| Eğil, çömel | Öne/yana eğil: robotun gövdesi eğilir. Çömel (stick boşta): robot squat yapar; kalkınca kalkar |
| Acil durdur | Panelde kırmızı **DURDUR**, ya da iki grip + tetik 0.5 s |

Panel açıkken robot donar (DURAKLADI); panel kapanınca devam eder. Panel sayfaları: Robot (Başlat, Kalibre et ve
devral, Duraklat/Devam), Hareket (hız, stick ölü bölge, yön kilidi, çömelme, eğilme kazancı), Eller (kol takibi,
parmaklar), Görüntü (Ego / İmmersif / Kapalı), Sistem (yeniden kalibre, bölüm işareti, hızlar).

## 4. Sırayla dene (PLAN §0)

Sahne: robot başlangıçta +x'e bakar. Koridorda 1.5, 4.1, 6.7 ve 9.3 m'de sarı işaretler var. **İlk işaretin**
sağında masa A (üstünde kırmızı küp) ve masa B, solunda 5°'lik rampa. Daha ileride solda 10° ve 15° rampalar ile yan
eğim, sağda 3 ve 6 cm basamaklar ile pürüzlü zemin.

1. Paneli aç → **Başlat**. Robot bantla yere iner ve ayakta durur (~3 s). Paneli kapat (X).
2. Dik dur, ileri bak, **A'yı 1 s basılı tut** → robot AKTİF. (Panelden **Kalibre et ve devral** de aynı.)
3. Stick ile ileri, geri, yana yürü. Sonra ilk sarı işarete yürü ve üstünde dur.
4. Vücudunla **90° sağa** dön: robot döner, masa A önünde. Sonra ±15° etrafa bak: robot dönmez.
5. Öne eğil, geri gel; çömel, kalk. Robot kalkınca tekrar yürür.
6. Masaya yaklaş: küp robotun **sağ elinin önünde**, gövdesinden ~30–35 cm ileride olsun (çok yaklaşma: daha yakında
   avuç küpün üstüne gelemiyor). Robot dönüşü birkaç derece eksik bitirebilir: masaya yürümeden önce görüntüde küpün
   sağ önde olduğuna bak, değilse stick'i hafif çapraz tutarak hizala. **İki kontrolcüyü bırak** → ELLER. Ellerini önce **masanın üstüne kaldır**, sonra uzan
   (alçakta uzanırsan robotun eli küpü iter). Küpü **sağ elinle, avuç aşağı** al: avucun küpün üstüne otursun,
   başparmak arkasında, parmaklar önünde; **yumruk yap, ~2 s tut, yavaşça kaldır**, biraz yana taşı, indir, elini aç.
   Tutmazsa elini aç, kaldır, tekrar dene (simde çoğu zaman 2.–3. denemede tuttu).
7. (§0 madde 7, "kontrolcüyü alıp küpü taşıyarak yürü", D8 ile mümkün değil: kontrolcü alınınca iki kol park eder,
   parmaklar açılır, küp düşer. Bu adımı atla; karar senin.)
8. **Kontrolcüleri geri al** → YÜRÜME. Geri geri koridora dön, vücudunla **180° sola** dön: 5°'lik rampa önünde.
   Rampayı çık, düzlükte yürü, öbür taraftan in. (İstersen 10°/15° rampaları ve basamakları dene; bunlar kabul kapısı
   değil, `P8_results.md`'de ölçümleri var.)
9. Bağlantıyı kes: headset'i çıkar ya da sayfayı kapat. Robot ayakta, yerinde durur.
10. Tekrar bağlan (sayfayı aç, Enter VR), paneli aç → **DURDUR**. Robot bantla yakalanır.

## 5. Bir şey ters giderse

| Belirti | Ne yap |
|---|---|
| Robot görüntüsü yok | Sol üstteki saat donuksa sim/köprü durmuş olabilir: `status`, sonra `morning` (eksikleri açar) |
| Panel "kalibre et" diyor, A çalışmıyor | Dik dur, ileri bak, 1 s kıpırdama; ya da panelden **Kalibre et ve devral**. Recenter (Meta tuşu) kalibrasyonu düşürür |
| Robot DURAKLADI'da kaldı | Kısa A/X (devam) ya da paneli aç/kapat. Kafa verisi gelmiyorsa şeritte uyarı çıkar |
| Robot düştü | Sim ~20 s sonra robotu kendisi diker; ya da panelde **DURDUR** (bant robotu yakalar), ~30 s sonra **Başlat** |
| Robot yürümüyor | ELLER modundasın (kontrolcü yok) ya da panel açık. Kontrolcüyü al |
| Küp masadan düştü | DURDUR + Başlat sahneyi sıfırlar (küp masaya döner) |
| Sayfa açılmıyor / takılı | Quest Browser'da sayfayı yenile, Enter VR. Sunucu yeniden başlarsa sayfa kendiliğinden bağlanır |
| Her şey karıştı | `stop`, sonra `morning` (1 dk). Quest'te sayfayı yenile |

Her koşu kaydedilir: `data/fullbody-runs/<zaman>/` (replay edilebilir).

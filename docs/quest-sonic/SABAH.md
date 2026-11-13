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
- **YÜRÜME:** en az bir kontrolcü elinde. **Sol stick yürütür, sağ stick (sağa/sola) robotu döndürür.** Tek
  kontrolcü tutuyorsan onun stick'i yürütür. Robotun iki kolu yanlarda sarkar, parmaklar açık. Panelde Eller →
  **Kumandayla kol** açıksa kollar kontrolcüleri izler, parmaklar olduğu gibi kalır (yürürken bir şey taşımak için).
- **ELLER:** iki kontrolcüyü de bırak. Robotun bilekleri ellerini, Dex3 parmakları parmaklarını izler. Joystick'siz
  yer değiştirme: **kendin yürü** (ileri, geri, yana): robot seninle birlikte yürür (başın ~13 cm ilerleyince
  başlar, yana ~20 cm; durunca o da kalan yolu bitirip durur). Yürümek ellerin yerini değiştirmez. Öne eğilmek,
  uzanmak, ağırlığını bir bacağa vermek robotu yürütmez. Robot en az ~20 cm yürür; çok küçük adımlar izlenmez. Panelde
  Eller → **Adımla yürüme** ile kapatılır.
- Geçişte Quest girdiyi 0.6–1.5 s keser; robot kısa bir an DURAKLADI'ya geçer, sonra kendiliğinden devam eder.

| Ne | Nasıl |
|---|---|
| Paneli aç | Herhangi bir thumbstick'e **bas** (kontrolcü) ya da elle: **sol avucunu açık olarak yüzüne çevir** (alttaki şerit sararır), sonra **sağ işaret parmağınla sol avucuna dokun**. Avucu yüze çevirmek tek başına artık paneli açmaz, robotu durdurmaz |
| Panelde seç | Kontrolcü ışını + **tetik**, ya da el ışını + **başparmak-işaret pinch**. Robota giden her şey **0.6 s basılı tutma** ister (sarı halka dolar); DURDUR, sekmeler ve X hemen çalışır |
| Kalibre et ve devral | Dik dur, ileri bak, kıpırdama. **A veya X'i 1 s basılı tut**, ya da panelde **Kalibre et ve devral** |
| Duraklat / devam | A veya X kısa bas |
| Yön kilidi | B veya Y |
| Yürü | Sol stick (ileri, geri, yan). En fazla 0.5 m/s; bırakınca ~0.1–0.25 m'de durur |
| Dön | Sağ stick sağa/sola (tam itince 60°/s; yön kilidi açıkken de çalışır), ya da vücudunla dön (kafa 25°'den fazla ve 0.3 s). Etrafa bakmak (±20°) robotu döndürmez |
| Eğil, çömel | Öne/yana eğil: robotun gövdesi eğilir (öne en fazla 30°). Çömel (stick boşta): robot squat yapar; kalkınca kalkar |
| Yerden al | Çömel, sonra **yere kadar in** (kalibrasyondaki boyundan ~55 cm aşağı, 0.5 s): robot iki dizinin üstüne çöker (KNEEL) ve öne eğilerek elleri yere ulaşır. Kalkınca (40 cm'nin üstüne) robot ~2 s'de ayağa kalkar |
| Yürüyerek yer değiştir | ELLER modunda kendin yürü: robot seninle yürür |
| Acil durdur | Panelde kırmızı **DURDUR**, ya da iki grip + tetik 0.5 s |

Panel açıkken robot donar (DURAKLADI); panel kapanınca devam eder. Panel sayfaları: Robot (Başlat, Kalibre et ve
devral, Duraklat/Devam), Hareket (hız, stick ölü bölge, yön kilidi, çömelme, eğilme kazancı), Eller (kol takibi,
parmaklar, adımla yürüme, kumandayla kol), Görüntü (Ego / İmmersif / Kapalı), Sistem (yeniden kalibre, bölüm işareti, hızlar).

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
7. (İsteğe bağlı, §0 madde 7) Panelde Eller → **Kumandayla kol**'u aç, kontrolcüleri al: kollar kontrolcüleri izler,
   parmaklar olduğu gibi kalır. Not: kontrolcüye geçiş 1 s'den uzun sürerse parmaklar o arada açılır.
7b. **Yerden al:** başlangıçta robotun 0.8 m solunda yerde ikinci bir küp var. Ona dön, ~40 cm gerisine yürü,
   kontrolcüleri bırak. Çömel ve yere kadar in: robot diz çöker. Sağ elinle avuç aşağı küpü almayı dene, kalk.
   (Simde diz çökme kararlı ama emüle operatörle küp alınamadı; deneysel.)
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

# Gerçek robotta tam vücut Quest teleop

Çalışırken sadece robot (PC2, Jetson) ve Quest 3S var; MSI döngüde değil. Süreç grafiği sim'dekiyle aynı (PLAN D11).
Farklar şunlar:

- DDS ucu `eth0` üzerinden gerçek robot (`configs/sonic_robot.yaml`).
- Kamera RealSense D435i'nin IR stereo çifti.
- SONIC, `~/sonic_jp5`'teki JetPack build'i (`configs/sonic_assets_robot.yaml`).

```
Quest ──Wi‑Fi aksoy-lab──> https://192.168.1.164  (443 → 8012)
PC2:  realsense_ir_stream :5605 → quest server :5596 → fullbody_node → SONIC :5556 → eth0 → motorlar, Dex3
```

## Tek seferlik kurulum (robotta)

```
git clone -b quest_emin https://github.com/ituealab/xr_camera_teleoperation ~/repos/xr_camera_teleoperation
cd ~/repos/xr_camera_teleoperation && bash deploy/setup_robot_fullbody.sh
```

Script tekrar çalıştırılabilir; şunları kurar:

- `data/xr-env` (node), `data/quest-env` (sunucu) ve `data/camera-env` (RealSense, py3.9) ortamları
- headset için TLS sertifikası
- 443 → 8012 yönlendirmesi (`quest-https-redirect.service`)
- `g1-free-camera.service`: Unitree'nin video servislerini açılışta durdurur, RealSense boş kalır
- `~/.zshrc`'ye `sonic-quest` ve `quest` alias'ları

Quest'te `https://192.168.1.164` adresini ilk açışta sertifika uyarısı çıkar. "Gelişmiş → devam et" deyip adresi yer imine ekle.

Güncelleme: `cd ~/repos/xr_camera_teleoperation && git pull`. Robotta yerel değişiklik yapma.

## Oturum

1. Robot askıda, elinde Unitree kumandası olan ikinci bir kişi var.
2. `developer start` (motion servisi bırakılır, denge kontrolü kapanır).
3. **Terminal 1:** `sonic-quest`.
   - Developer modunu kontrol eder, "askıda mı?" diye sorar.
   - SONIC açılınca eklemler varsayılan duruşa gider.
   - **Bu terminalde `o` = acil durdurma.**
4. **Terminal 2:** `quest`. Kamera, Quest sunucusu ve node açılır, sonunda adres yazılır.
5. Quest: yer imini aç, Enter VR, panelden **Başlat**. Dik dur, ileri bak, **A'yı 1 sn tut** (kalibre et ve devral).
6. Bitiş:
   - paneldeki **DURDUR** (ya da iki grip + tetik)
   - `quest stop`
   - SONIC terminalinde Ctrl-C
   - robot desteklenmiş haldeyken `developer exit`

Diğer komutlar: `quest status`, `quest logs`, `quest attach node|quest|camera`. Loglar `~/.local/state/xr_fullbody` altında; her koşu `data/fullbody-runs/<zaman>` altına kaydedilir.

## Güvenlik

- SONIC'i hiçbir şey kendiliğinden yeniden başlatmaz (sim'deki respawn robot profilinde yasak). Her başlatma operatörün komutudur.
- **Açık soru, ilk iş askıda test edilecek:** DURDUR SONIC'i kapattığında robot fiziksel olarak ne yapıyor? Sönümlemeye mi geçiyor, gevşeyip çöküyor mu? Bu öğrenilmeden askıyı gevşetme.
- Wi-Fi koparsa ya da kafa verisi 0.2 sn'den eskiyse node PAUSED'a geçer. Bunu askıda bir kez dene: Quest'i çıkar, ya da Wi-Fi'ı kapat.
- Sıra: askıda (duruş, kollar, Dex3 aç/kapa, kamera, DURDUR), sonra askı gevşek (eğil, çömel, adım, dön), sonra serbest.

## Ağ

- Robot aksoy-lab'da sabit `192.168.1.164` adresinde (NetworkManager profili `aksoy-lab`, öncelik 30, Wi-Fi güç tasarrufu kapalı).
- eduroam'ın önceliği -10. Robot başka bir ağa bağlıyken eduroam'a geçmez; eduroam'ı sadece görünür başka ağ yokken açar.
- Uzaktan erişim: tailscale (`unitree`) ya da `ssh unitree@192.168.1.164`.

## Kontrol

```
data/xr-env/bin/python -m xr_camera_teleop.sonic.assets --manifest configs/sonic_assets_robot.yaml verify
```

Bu komut SONIC dosyalarının pinlenmiş sha256'larla aynı olduğunu doğrular.

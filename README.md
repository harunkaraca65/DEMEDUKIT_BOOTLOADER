# STM32 OTA Bootloader Sistem - Algoritma Akışı
# STM32 OTA Bootloader System - Algorithm Flow

---

## 🇹🇷 Türkçe

Bu proje, ESP32 ve STM32 mimarisi üzerine kurulu uçtan uca bir Firmware Güncelleme (OTA) sistemidir.
Sistemde ESP32 işlemcisi yönetici (Host), STM32G030 ise hedef (Target) cihaz olarak çalışır.
Performans optimizasyonu için standart HAL kütüphaneleri kullanılmamıştır.
Tüm Flash yönetimi, UART ve I2C sürücüleri Low-Layer (LL) ve Register seviyesinde yazılmıştır.
STM32 Flash hafızası, Linker Script düzenlemesi ile iki bölüme ayrılmıştır.
İlk bölüm (0x0800 0000), silinmeyen Bootloader kodunu barındırır.
İkinci bölüm (0x0800 3000), kullanıcı uygulamasının (App) çalıştığı alandır.
Sistem açılışında ESP32, Wi-Fi üzerinden sunucuya bağlanarak versiyon kontrolü yapar.
Sunucudaki versiyon cihazdakinden yüksekse, güncelleme süreci başlatılır.
STM32 açılışta 3 saniye boyunca UART hattını dinler.
ESP32'den gelen başlatma sinyalini (0xAB) yakalarsa güncelleme moduna girer.
Veri transferi, RAM kısıtlamaları nedeniyle küçük paketler halinde yapılır.
Her paket gönderiminde "Dur-Bekle" (Stop-and-Wait) protokolü uygulanır.
STM32G0 serisinin 64-bit yazma zorunluluğu için özel veri hizalama algoritması kullanılır.
İndirilen dosyanın bütünlüğü CRC32 algoritması ile doğrulanır.
İmzalar uyuşmazsa güncelleme reddedilir.
Süreç boyunca OLED ekran üzerinde ilerleme durumu gösterilir.
Güncelleme tamamlandığında Vektör Tablosu (VTOR) kaydırılır ve yeni uygulamaya atlanır.

### Kurulum ve Kullanım

Bu projeyi Demedukit kartınızda çalıştırmak için aşağıdaki adımları sırasıyla uygulayın.

**1. Donanım Ayarı (Hardware Setup)**
Kart üzerindeki Channel Select Jumper ayarlarını kontrol edin.
Jumper'ları, ESP32 ve Serial hattının haberleşebileceği konuma getirin.
Tx ve Rx hatlarının doğru eşleştiğinden emin olun.

**2. Sunucu Kurulumu (Server Setup)**
Bilgisayarınızı güncelleme sunucusuna dönüştürmek için Python gereklidir.
Python 3.x sürümünü indirin ve kurun.
Kurulum sırasında "Add Python to PATH" seçeneğini mutlaka işaretleyin.
Masaüstünde `OTA_Server` adında yeni bir klasör oluşturun.
Derlediğiniz `app.bin` dosyasını bu klasörün içine kopyalayın.
Yeni bir metin belgesi oluşturun.
İçine sadece versiyon numarasını (Örn: `2`) yazın.
Dosyayı `version.txt` adıyla kaydedin.
Klasör içinde boş bir yere Shift tuşuna basılı tutarak sağ tıklayın.
"PowerShell penceresini buradan aç" veya Terminal seçeneğini seçin.
Komut satırına `python -m http.server 8000` yazıp Enter'a basın.
Güvenlik duvarı uyarısı gelirse erişime izin verin.
Yeni bir komut penceresi açıp `ipconfig` komutunu yazın.
IPv4 adresinizi not edin.

**3. Yazılım Yükleme (Flashing)**
STM32CubeIDE programını açın.
Bootloader projesini derleyin (Build).
Kartı bağlayın ve kodu STM32'ye yükleyin.
ESP-IDF projesindeki `main.c` dosyasını açın.
`WIFI_SSID` ve `WIFI_PASS` tanımlarını kendi ağ bilgilerinizle güncelleyin.
`SERVER_URL` kısmına bilgisayarınızın IP adresini girin (Örn: `http://192.168.1.25:8000/app.bin`).
ESP-IDF terminalinde `idf.py build flash monitor` komutunu çalıştırın.

**4. Çalıştırma (Execution)**
Tüm yüklemeler bittiğinde sisteme güç verin veya Reset atın.
ESP32 sunucuya bağlanacak ve güncellemeyi kontrol edecektir.
Güncelleme varsa, OLED ekranda ilerleme çubuğunun dolduğunu izleyebilirsiniz.

---

## 🇬🇧 English

This project is an end-to-end Firmware Update (OTA) system based on ESP32 and STM32 architecture.
The ESP32 processor operates as the Host, and the STM32G030 operates as the Target device.
Standard HAL libraries were not used for performance optimization.
All Flash management, UART, and I2C drivers were written at Low-Layer (LL) and Register levels.
STM32 Flash memory is split into two sections via Linker Script modification.
The first section (0x0800 0000) hosts the immutable Bootloader code.
The second section (0x0800 3000) is the area where the user application (App) runs.
Upon startup, the ESP32 connects to the server via Wi-Fi and performs a version check.
If the server version is higher than the device version, the update process is initiated.
The STM32 listens to the UART line for 3 seconds upon boot.
If it captures the start signal (0xAB) from ESP32, it enters update mode.
Data transfer is performed in small chunks due to RAM constraints.
A "Stop-and-Wait" protocol is applied for each packet transmission.
A custom data alignment algorithm is used for the STM32G0 series' 64-bit write requirement.
The integrity of the downloaded file is verified using the CRC32 algorithm.
The update is rejected if signatures do not match.
Progress status is displayed on the OLED screen throughout the process.
When the update is complete, the Vector Table (VTOR) is offset, and the system jumps to the new application.

### Setup and Usage

Follow these steps sequentially to run this project on your Demedukit board.

**1. Hardware Setup**
Check the Channel Select Jumper settings on the board.
Set the jumpers to a position that allows ESP32 and Serial line communication.
Ensure Tx and Rx lines are matched correctly.

**2. Server Setup**
Python is required to turn your PC into an update server.
Download and install Python 3.x.
Make sure to check "Add Python to PATH" during installation.
Create a new folder named `OTA_Server` on your desktop.
Copy your compiled `app.bin` file into this folder.
Create a new text document.
Write only the version number (e.g., `2`) inside it.
Save the file as `version.txt`.
Shift + Right Click on an empty space inside the folder.
Select "Open PowerShell window here" or Terminal.
Type `python -m http.server 8000` in the command line and press Enter.
Allow access if a firewall warning appears.
Open a new command window and type `ipconfig`.
Note down your IPv4 address.

**3. Flashing Firmware**
Open STM32CubeIDE.
Build the Bootloader project.
Connect the board and flash the code to the STM32.
Open the `main.c` file in the ESP-IDF project.
Update `WIFI_SSID` and `WIFI_PASS` definitions with your network credentials.
Enter your PC's IP address in the `SERVER_URL` section (e.g., `http://192.168.1.25:8000/app.bin`).
Run the `idf.py build flash monitor` command in the ESP-IDF terminal.

**4. Execution**
Power up the system or Reset it when all flashing is done.
The ESP32 will connect to the server and check for updates.
If an update is available, you can watch the progress bar fill up on the OLED screen.
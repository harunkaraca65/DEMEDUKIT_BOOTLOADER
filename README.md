\# Professional STM32 OTA Bootloader System

\### (Endüstriyel Seviye Uzaktan Güncelleme Sistemi / Industrial Grade Remote Update System)


---


\## 🇹🇷 TÜRKÇE



\### Proje Hakkında

Bu proje, gömülü sistemlerde fiziksel erişim zorunluluğunu ortadan kaldıran uçtan uca bir sistemdir.

Wi-Fi üzerinden güvenli ve akıllı bir \*\*Firmware Güncelleme (OTA)\*\* ekosistemidir.

Geleneksel yöntemlerin aksine, bu sistemde bir bilgisayar yazılımı değil, ESP32 işlemcisi yönetici (Host) rolünü üstlenir.

Hedef cihaz olan STM32G030 ise, kendi hafızasını yönetebilen akıllı bir uç birim (Target) olarak çalışır.



Bu projede performans ve kod boyutu optimizasyonu için standart \*\*HAL kütüphaneleri kullanılmamıştır.\*\*

Bunun yerine, donanıma tam hakimiyet sağlayan \*\*Low-Layer (LL) sürücüler\*\* ve \*\*Register seviyesinde\*\* kodlama teknikleri tercih edilmiştir.

Bu sayede Flash yönetimi ve I2C ekran sürme işlemleri maksimum hızda ve minimum boyutta gerçekleştirilmiştir.



---



\### Sistem Mimarisi ve Algoritma Akışı



Sistem enerjilendiği andan itibaren rastgele bir akış izlemez.

Güvenlik ve kararlılığın ön planda tutulduğu katı bir protokol zincirini takip eder.



\#### 1. Hafıza Bölümleme (Memory Partitioning)

STM32'nin 64KB Flash hafızası, sanal sınırlarla iki stratejik bölgeye ayrılmıştır.

\*\*Bootloader Alanı (0x0800 0000 - 12KB):\*\* Cihazın "tapusu" buradadır.

Asla silinmeyen ve değiştirilmeyen çekirdek alandır.

Cihaz her açıldığında kontrol Bootloader'dadır.

\*\*Uygulama Alanı (0x0800 3000 - 52KB):\*\* Kullanıcının asıl yazılımının (App) koştuğu yerdir.

Güncellemeler buraya yazılır.



\#### 2. Akıllı Karar Mekanizması (Smart Decision Logic)

Sistemin beyni olan ESP32, açılışta Wi-Fi ağına bağlanarak uzaktaki bir sunucuyu sorgular.

Ancak körü körüne işlem yapmaz.

Kendi kalıcı hafızasında (NVS) tuttuğu \*"En son hangi sürümü yükledim?"\* bilgisini kontrol eder.

Bunu sunucudaki güncel sürümle karşılaştırır.

Eğer sunucudaki sürüm daha yeniyse, ESP32 \*\*"Avcı Moduna"\*\* geçer ve STM32'yi yakalamaya çalışır.

Eğer sistem zaten güncelse, STM32'yi rahatsız etmez ve sistem normal açılır.



\#### 3. Yakalama ve El Sıkışma (Interception \& Handshake)

Güncelleme kararı verildiğinde süreç başlar.

STM32 açıldığında 3 saniye boyunca UART hattını dinler.

ESP32 `0xAB` (Başlat) sinyalini gönderir.

STM32 bunu yakalar ve uygulamayı başlatmayı iptal eder.

Ardından `0xC1` (ACK) cevabı vererek \*\*"Emrini bekliyorum"\*\* moduna girer.



\#### 4. Veri Transferi ve Görsel Geri Bildirim

Dosya internetten RAM kısıtlamalarına uygun olarak küçük paketler (Chunks) halinde indirilir.

ESP32, veriyi özel bir paket yapısıyla (`\[CMD] + \[LEN] + \[DATA]`) STM32'ye iletir.

\*\*OLED Arayüzü:\*\* Süreç boyunca STM32, üzerindeki ekranı sürerek kullanıcıyı bilgilendirir.

Ekranda \*\*"SİLİNİYOR"\*\*, \*\*"YÜKLENİYOR"\*\* mesajları gösterilir.

Ayrıca canlı dolan bir \*\*İlerleme Çubuğu (Progress Bar)\*\* ile süreç görselleştirilir.



\#### 5. Askeri Seviye Güvenlik (CRC32 Integrity Check)

Dosya inip yazıldıktan sonra işlem bitmez.

ESP32, indirdiği dosyanın matematiksel imzasını (CRC32) hesaplar.

STM32, kendi hafızasına yazdığı veriyi okuyup kendi imzasını çıkarır.

İki imza eşleşmezse güncelleme \*\*REDDEDİLİR\*\*.

Bu, internet kopması veya veri bozulması durumunda cihazın çökmesini (Brick olmasını) %100 engeller.



---



\### Geliştirme Süreci ve Mühendislik Çözümleri



Bu proje, adım adım ilerleyen bir mühendislik serüvenidir.

Karşılaşılan her darboğaz, özel bir teknikle çözülmüştür.



\* \*\*Problem 1: Standart Kütüphanelerin Hantallığı\*\*

&nbsp;   \* \*\*Çözüm:\*\* HAL kütüphanesinin Bootloader için çok yer kapladığı fark edildi.

&nbsp;   \* Tüm Flash yazma, UART ve I2C sürücüleri \*\*LL (Low-Layer)\*\* ve \*\*Register\*\* manipülasyonu ile sıfırdan yazıldı.



\* \*\*Problem 2: Çift Yazılımın Çakışması\*\*

&nbsp;   \* \*\*Çözüm:\*\* Linker Script (`.ld`) dosyaları manipüle edildi.

&nbsp;   \* Hafıza fiziksel olarak bölündü.

&nbsp;   \* Uygulama kodunun başlangıç vektörü (`SCB->VTOR`) kaydırılarak çakışma önlendi.



\* \*\*Problem 3: Yüksek Hızda Veri Kaybı\*\*

&nbsp;   \* \*\*Çözüm:\*\* "Dur-Bekle" (Stop-and-Wait) protokolü tasarlandı.

&nbsp;   \* STM32, \*"Yazma ve ekran çizme işim bitti (ACK)"\* demeden, ESP32 bir sonraki paketi göndermez.

&nbsp;   \* Sistem 5 saniyelik bir zaman aşımı (timeout) korumasına sahiptir.



\* \*\*Problem 4: Flash Yazma Kısıtlamaları\*\*

&nbsp;   \* \*\*Çözüm:\*\* STM32G0 serisinin "64-bit (Double Word) Yazma" zorunluluğu vardır.

&nbsp;   \* Bunun için özel bir sürücü yazıldı.

&nbsp;   \* Gelen veri byte-byte birleştirilerek hizalanır ve `PG` biti donanım seviyesinde yönetilir.



\* \*\*Problem 5: Kısıtlı Alanda Grafik Arayüzü\*\*

&nbsp;   \* \*\*Çözüm:\*\* Hazır grafik kütüphaneleri (u8g2 vb.) Bootloader için çok büyüktü.

&nbsp;   \* Sadece gerekli piksel, karakter ve bar çizim fonksiyonlarını içeren \*\*Ultra-Lite SSD1306 Sürücüsü\*\* yazıldı.



---



### Kurulum ve Kullanım

Bu projeyi kendi Demedukit kartınızda çalıştırmak için aşağıdaki adımları sırasıyla uygulayın:

#### 1. Donanım Ayarı (Hardware Setup)
Kart üzerindeki **Channel Select Jumper** ayarlarını kontrol edin.
Jumper'ları, **ESP32 ve Serial hattının haberleşebileceği** konuma getirin.
(Genellikle Rx ve Tx pinlerinin çaprazlandığı moddur).

#### 2. Sunucu Kurulumu (Server Setup)
Bilgisayarınızı bir güncelleme sunucusuna dönüştürmek için:

* **Python Kurulumu:**
    * Microsoft Store'a girin ve "Python 3.11" (veya daha yeni bir sürüm) aratıp indirin.
    * Veya `python.org` adresinden indirip kurun.
    * *Önemli:* Kurulum sırasında **"Add Python to PATH"** seçeneğini mutlaka işaretleyin.

* **Klasör Hazırlığı:**
    * Masaüstünde `OTA_Server` adında yeni bir klasör oluşturun.
    * Derlediğiniz `app.bin` dosyasını bu klasörün içine atın.
    * Yeni bir metin belgesi oluşturun, içine sadece versiyon numarasını (Örn: `2`) yazın ve adını `version.txt` olarak kaydedin.

* **Sunucuyu Başlatma:**
    * Klasörün içinde boş bir yere `Shift` tuşuna basılı tutarak sağ tıklayın.
    * "PowerShell penceresini buradan aç" (veya Terminal) seçeneğini seçin.
    * Açılan mavi ekrana şu komutu yazıp Enter'a basın:
      `python -m http.server 8000`
    * Güvenlik duvarı uyarısı gelirse "Erişime İzin Ver" deyin.

* **IP Adresini Öğrenme:**
    * Yeni bir PowerShell penceresi açın.
    * `ipconfig` yazın ve Enter'a basın.
    * `IPv4 Address` satırındaki IP adresini not edin (Örn: `192.168.1.25`).

#### 3. Yazılım Yükleme (Flashing)

* **STM32 Tarafı:**
    * `STM32CubeIDE` programını açın.
    * `Bootloader` projesini import edin ve derleyin (Build).
    * Kartı bağlayın ve "Run" butonuna basarak kodu yükleyin.

* **ESP32 Tarafı:**
    * `ESP-IDF` projesindeki `main.c` dosyasını açın.
    * `WIFI_SSID` ve `WIFI_PASS` kısımlarına kendi Wi-Fi bilgilerinizi girin.
    * `SERVER_URL` kısmına, az önce not ettiğiniz IP adresini girin:
      `http://192.168.1.25:8000/app.bin`
    * ESP-IDF terminalini açın ve şu komutu girin:
      `idf.py build flash monitor`

#### 4. Çalıştırma (Execution)
Tüm yüklemeler bittiğinde sisteme güç verin veya Reset atın.
ESP32 sunucuya bağlanacak ve güncellemeyi kontrol edecektir.
Güncelleme varsa, OLED ekranda ilerleme çubuğunun dolduğunu izleyebilirsiniz.


---

---



\## 🇬🇧 ENGLISH



\### About the Project

This project is an end-to-end \*\*Wireless Firmware Update (OTA)\*\* ecosystem.

It is designed to eliminate the bottleneck of "physical access requirement" in embedded systems.

Unlike traditional methods relying on PC software, this system utilizes an \*\*ESP32\*\* processor.

The ESP32 acts as an intelligent Host that makes autonomous decisions.

The target device, \*\*STM32G030\*\*, operates as a smart endpoint.

It is capable of managing its own memory, verifying incoming data, and reprogramming itself.



To optimize performance and code size, standard \*\*HAL libraries were NOT used\*\* in this project.

Instead, \*\*Low-Layer (LL) drivers\*\* and \*\*Register-level\*\* coding techniques were employed to ensure full hardware control.

This allowed for high-speed Flash management and I2C display driving with minimal footprint.



---



\### Architecture \& Algorithm Flow



The system follows a strict protocol chain prioritizing security and stability rather than a random flow.



\#### 1. Memory Partitioning

The STM32's 64KB Flash memory is logically split into two strategic regions.

\*\*Bootloader Section (0x0800 0000 - 12KB):\*\* The immutable core.

This section runs first on power-up and is never erased.

\*\*Application Section (0x0800 3000 - 52KB):\*\* User application space.

This is where updates are flashed.



\#### 2. Smart Decision Logic

The ESP32 does not update blindly.

Upon startup, it connects to Wi-Fi and queries the server.

It checks its Non-Volatile Storage (NVS) for the "Last Installed Version".

It compares this with the server version.

Only if `Server Version > Device Version` does it enter \*\*"Hunter Mode"\*\* to intercept the STM32.

Otherwise, it lets the STM32 boot normally.



\#### 3. Interception \& Handshake

Upon reset, STM32 listens on UART for 3 seconds.

ESP32 sends a magic byte `0xAB`.

STM32 intercepts this and aborts the jump to the main app.

It sends an `ACK` (`0xC1`) and enters Update Mode.



\#### 4. Data Transfer \& UI Feedback

Firmware is downloaded in chunks via HTTP.

Data is piped to STM32 using a robust packet format.

\*\*OLED Integration:\*\* The process is visualized on the OLED screen.

Status messages like "ERASING" and "LOADING" are displayed.

A \*\*Real-time Progress Bar\*\* provides visual feedback.



\#### 5. Military-Grade Safety (CRC32 Verification)

Before running the new code, the system performs an integrity check.

ESP32 calculates the CRC32 of the downloaded file.

STM32 reads its flash memory and calculates its own CRC32.

If they don't match, the update is \*\*REJECTED\*\*.

This prevents a "bricked" device due to corrupted data or connection loss.



---



\### Development Journey \& Solutions



This project was an engineering journey involving step-by-step problem solving.



\* \*\*Problem 1: Overhead of Standard Libraries\*\*

&nbsp;   \* \*\*Solution:\*\* HAL libraries were found to be too heavy for the Bootloader.

&nbsp;   \* All Flash writing, UART, and I2C drivers were written from scratch using \*\*LL (Low-Layer)\*\* and \*\*Register\*\* manipulation.



\* \*\*Problem 2: Memory Conflict\*\*

&nbsp;   \* \*\*Solution:\*\* Modified Linker Scripts (`.ld`) to physically split flash memory.

&nbsp;   \* Adjusted Vector Table Offsets (`SCB->VTOR`) to allow two programs to coexist.



\* \*\*Problem 3: Data Loss at High Speed\*\*

&nbsp;   \* \*\*Solution:\*\* Designed a \*\*Stop-and-Wait\*\* protocol.

&nbsp;   \* ESP32 waits for an explicit `ACK` from STM32 before sending the next chunk.

&nbsp;   \* Includes a safety timeout mechanism.



\* \*\*Problem 4: Flash Alignment Restrictions\*\*

&nbsp;   \* \*\*Solution:\*\* STM32G0 requires 64-bit aligned writes.

&nbsp;   \* Developed a custom \*\*Low-Layer Flash Driver\*\*.

&nbsp;   \* It reconstructs incoming byte streams into aligned words and manages hardware flags directly.



\* \*\*Problem 5: UI Overhead\*\*

&nbsp;   \* \*\*Solution:\*\* Standard graphics libraries were too heavy.

&nbsp;   \* Wrote a custom \*\*Ultra-Lite SSD1306 Driver\*\* containing only essential drawing functions.



---



### How to Use

Follow these steps to deploy the system on your Demedukit board:

#### 1. Hardware Setup
Check the **Jumpers** on the board.
Configure the jumpers so that **ESP32 and Serial** lines can communicate.
(Ensure Tx and Rx lines are correctly linked for inter-chip communication).

#### 2. Server Setup
Turn your PC into an update server:

* **Install Python:**
    * Download Python 3.x from the Microsoft Store or python.org.
    * Ensure you check **"Add Python to PATH"** during installation.

* **Prepare Folder:**
    * Create a folder named `OTA_Server` on your desktop.
    * Copy your compiled `app.bin` firmware file into this folder.
    * Create a text file named `version.txt` containing only the version number (e.g., `2`).

* **Start Server:**
    * Shift + Right Click inside the folder and select "Open PowerShell window here".
    * Type the following command and hit Enter:
      `python -m http.server 8000`
    * Allow access if Firewall prompts appear.

* **Get IP Address:**
    * Open a new PowerShell window.
    * Type `ipconfig` and note down your `IPv4 Address`.

#### 3. Flashing Firmware

* **STM32 Side:**
    * Open `STM32CubeIDE`.
    * Build the `Bootloader` project.
    * Connect the board and click "Run" to flash the STM32.

* **ESP32 Side:**
    * Open `main.c` in your ESP-IDF project.
    * Update `WIFI_SSID` and `WIFI_PASS` with your credentials.
    * Update `SERVER_URL` with your PC's IP address:
      `http://YOUR_IP_HERE:8000/app.bin`
    * Run the following command in the terminal:
      `idf.py build flash monitor`

#### 4. Execution
Power up or Reset the board.
The system will automatically detect the update server.
Watch the OLED screen as the progress bar fills up and the device updates itself.

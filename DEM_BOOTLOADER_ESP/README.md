| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-H2 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- | -------- | -------- |

# ESP-IDF Asenkron UART RX/TX Görevleri - Algoritma Akışı
# ESP-IDF Asynchronous UART RX/TX Tasks - Algorithm Flow

---

## 🇹🇷 Türkçe

Bu ESP-IDF uygulaması, iki ayrı FreeRTOS görevi kullanarak asenkron UART iletişimini yönetir.
`app_main` fonksiyonu, sistemi başlatan `init` fonksiyonunu çağırarak başlar.
`init` fonksiyonu, `UART_NUM_2`'yi (TX=25, RX=26) 115200 baud hızında yapılandırır.
Ayrıca, TX (GPIO 32) ve RX (GPIO 33) için iki adet LED pini çıkış olarak ayarlar.
`app_main`, daha sonra `rx_task` (yüksek öncelikli) ve `tx_task` (düşük öncelikli) olmak üzere iki görev oluşturur.
`tx_task`, her 6 saniyede bir `UART_NUM_2` üzerinden "Hello STM\n" mesajını gönderir.
Mesajı gönderdikten sonra, TX LED'ini 5 saniyeliğine yakar ve ardından 1 saniye bekler.
Ayrıca, gönderilen mesajı ana konsol (UART0) üzerine de yazdırır.
`rx_task`, sürekli olarak `UART_NUM_2`'den veri okumaya çalışır (200ms zaman aşımı ile).
Herhangi bir veri alındığında, bu veriyi null-sonlandırır.
Alınan veriyi hem ESP-LOGI ile hem de ana konsola (UART0) yazdırır.
Veri alımını göstermek için RX LED'ini 200 milisaniye süreyle yakar.

---

## 🇬🇧 English

This ESP-IDF application manages asynchronous UART communication using two separate FreeRTOS tasks.
The `app_main` function begins by calling the `init` function to initialize the system.
The `init` function configures `UART_NUM_2` (TX=25, RX=26) at 115200 baud.
It also sets up two LED pins as outputs for TX (GPIO 32) and RX (GPIO 33).
`app_main` then creates two tasks: `rx_task` (with a higher priority) and `tx_task` (with a lower priority).
The `tx_task` sends the message "Hello STM\n" via `UART_NUM_2` every 6 seconds.
After sending the message, it illuminates the TX LED for 5 seconds and then waits for 1 second.
It also prints the sent message to the main console (UART0).
The `rx_task` continuously attempts to read data from `UART_NUM_2` (with a 200ms timeout).
When any data is received, it null-terminates the data.
It prints the received data to both ESP-LOGI and the main console (UART0).
It flashes the RX LED for 200 milliseconds to indicate data reception.
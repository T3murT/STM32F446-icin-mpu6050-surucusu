# STM32F446 MPU6050 Driver with Kalman Filter

![Language](https://img.shields.io/badge/language-C-blue.svg)
![Platform](https://img.shields.io/badge/platform-STM32-green.svg)
![License](https://img.shields.io/badge/license-MIT-orange.svg)

## 🇹🇷 Proje Hakkında (Turkish)

Bu depo, **STM32F446** mikrodenetleyicisi için geliştirilmiş, **MPU6050** IMU (Inertial Measurement Unit) sensör sürücüsünü ve **Kalman Filtresi** uygulamasını içerir.

MPU6050'den alınan ham ivmeölçer (accelerometer) ve jiroskop (gyroscope) verileri genellikle gürültülüdür ve titreşimlerden kolayca etkilenir. Bu projede, sensörden I2C protokolü ile okunan veriler, **Kalman Filtresi** algoritmasından geçirilerek gürültüden arındırılmış, kararlı **Pitch (Yunuslama)** ve **Roll (Yuvarlanma)** açıları elde edilmiştir.

### Özellikler
* STM32 HAL Kütüphanesi üzerine kurulu yapı.
* I2C üzerinden hızlı veri okuma (400kHz Fast Mode).
* Gerçek zamanlı açı kestirimi için optimize edilmiş Kalman Filtresi.
* Ofset (kalibrasyon) hesaplama fonksiyonları.
* Modüler kod yapısı (`mpu6050.c`, `kalman.c`).

### Bağlantı Şeması (Pinout)
STM32F446RE Nucleo (veya özel kart) ile MPU6050 bağlantısı aşağıdaki gibidir:

| MPU6050 Pin | STM32F446 Pin | Açıklama |
| :--- | :--- | :--- |
| VCC | 3.3V / 5V | Güç Beslemesi |
| GND | GND | Toprak |
| SCL | PB8 (veya uygun I2C pini) | Clock Hattı |
| SDA | PB9 (veya uygun I2C pini) | Data Hattı |

---

## 🇺🇸 Project Description (English)

This repository contains an **MPU6050** IMU driver and **Kalman Filter** implementation specifically designed for the **STM32F446** microcontroller.

Raw data from the accelerometer and gyroscope is inherently noisy and prone to drift. This driver interfaces with the MPU6050 via I2C and applies a **Kalman Filter** to fuse the sensor data, providing stable and noise-free **Pitch** and **Roll** angle estimation.

### Key Features
* Built on STM32 HAL Library.
* High-speed I2C communication (400kHz).
* Lightweight Kalman Filter implementation for real-time angle estimation.
* Built-in calibration (offset calculation) routines.
* Modular design for easy integration.

## 🚀 Kurulum ve Kullanım / Installation & Usage

1.  Proje dosyalarını `Core/Src` ve `Core/Inc` klasörlerine kopyalayın.
2.  `main.c` dosyanıza kütüphaneyi dahil edin:
    ```c
    #include "mpu6050.h"
    ```
3.  Başlatma ve döngü (Init & Loop):

```c
/* Global Variables */
MPU6050_t MPU6050;

int main(void) {
  /* MCU Configuration... */
  HAL_Init();
  SystemClock_Config();
  MX_I2C1_Init(); // I2C başlatılmalı

  /* MPU6050 Başlatma / Init */
  while (MPU6050_Init(&hi2c1) == 1);

  while (1) {
    /* Veriyi oku ve filtrele / Read and filter data */
    MPU6050_Read_All(&hi2c1, &MPU6050);

    /* Erişim / Access :
       MPU6050.KalmanAngleX -> Roll
       MPU6050.KalmanAngleY -> Pitch
    */

    HAL_Delay(4); // Örnekleme zamanı önemli / Sampling time matters
  }
}

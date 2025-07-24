# RTOS Smartwatch

This project implements a **Smartwatch using FreeRTOS on ESP32** for real-time clock display, heart rate and SpO₂ monitoring, temperature and humidity measurement, alarm management, health alerts, and MQTT data upload.

## Features

* **Display:**

  * Time: hours, minutes, seconds, day, month, year.
  * Temperature and humidity.
  * Heart rate and SpO₂.
  * Alarm setup screen.
* **Alarm Management:**

  * Set alarms using push buttons.
  * Buzzer alarm notification at the set time.
* **Health Monitoring:**

  * Measure heart rate and SpO₂ using MAX30100.
  * Measure temperature and humidity using DHT22.
  * Display data on LCD and upload to MQTT cloud.
* **Health Alerts:**

  * Trigger buzzer if thresholds are exceeded.
  * Send email alerts when health parameters are out of safe range.

##  Hardware Used

* **ESP32** (WiFi + BLE, 5VDC)
* **LCD 16x2 I2C**
* **TTP223B Touch Sensor** (for screen switching)
* **Push Buttons** (for alarm threshold setup)
* **DHT22** (temperature and humidity)
* **DS1307 RTC** (real-time clock)
* **MAX30100** (heart rate and SpO₂)
* **Buzzer**

##  FreeRTOS Configuration

* Tick rate: `configTICK_RATE_HZ = 1000Hz`
* Preemptive scheduling: `configUSE_PREEMPTION = 1`
* Time slicing: `configUSE_TIME_SLICING = 1`
* Max priority: `configMAX_PRIORITIES = 4`
* **8 Tasks:**

  * Heart rate measurement
  * Screen display management
  * Alarm triggering
  * Alarm setup
  * Alert management
  * Threshold monitoring
  * Temperature and humidity measurement
  * Time update
* **1 SoftTimer** (1s interval) for MQTT data upload
* **2 semaphores and 1 mutex**

##  System Block Diagram

![Schematic](link_to_block_diagram_image_if_available.png)

## Demo

* [Demo video: code, tasks, and execution](https://www.youtube.com/watch?v=j3JlXlQpY2I)
* [Demo video: hardware in operation](https://www.youtube.com/watch?v=rDHCBnv8ca0)

## Applications

* Wearable device for users to monitor time, heart rate, SpO₂, temperature, and humidity anytime.
* Provides timely health alerts with buzzer and email notifications.
* Uploads health data to MQTT for cloud storage and analysis.

---

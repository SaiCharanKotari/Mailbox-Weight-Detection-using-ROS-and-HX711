# Mailbox Weight Detection using ROS and HX711  

This project demonstrates how to use an **Arduino + HX711 load cell amplifier** to measure weight and publish the data to a **ROS (Robot Operating System)** environment. It simulates a **smart mailbox** that detects when mail is added or removed based on changes in measured weight.  

---

## 📌 Features  
- Uses **HX711 weight sensor** with Arduino.  
- Publishes live **weight values** to ROS topic:  
  - `/mailbox_weight` → `std_msgs/Float32`  
- Publishes **status messages** when weight changes beyond a threshold:  
  - `/mailbox_status` → `std_msgs/String`  
- Automatic tare and scaling for sensor calibration.  
- ROS node written in Arduino using `rosserial`.  

---

## 🛠️ Hardware Requirements  
- Arduino board (e.g., Uno, Mega, Nano)  
- HX711 module  
- Load cell sensor (any compatible weight sensor)  
- USB cable (for Arduino–PC connection)  

---

## 🔌 Wiring  
| HX711 Pin | Arduino Pin |  
|-----------|-------------|  
| DT        | 3           |  
| SCK       | 4           |  
| VCC       | 5V          |  
| GND       | GND         |  

---

## ⚙️ Software Requirements  
- ROS 1 (tested on **Noetic**)  
- `rosserial_arduino` package  
- Arduino IDE with `ros_lib` and `HX711.h` library  

---

## 🚀 Setup Instructions  

### 1. Install ROS and rosserial  
```bash
sudo apt-get install ros-noetic-rosserial ros-noetic-rosserial-arduino
```

### 2. Generate and copy ros_lib for Arduino  
```bash
cd ~/Arduino/libraries
rosrun rosserial_arduino make_libraries.py .
```

### 3. Upload Arduino Code  
- Open the provided Arduino sketch in Arduino IDE.  
- Connect Arduino and upload the code.  

### 4. Start rosserial  
Connect Arduino to ROS:  
```bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600
```

*(Replace `/dev/ttyUSB0` with your Arduino port.)*  

### 5. Subscribe to Topics  
Check weight values:  
```bash
rostopic echo /mailbox_weight
```

Check status updates:  
```bash
rostopic echo /mailbox_status
```

---

## 📊 Example Output  

**Weight topic:**  
```
data: 12.35
data: 12.38
```

**Status topic (when mail is detected):**  
```
data: "Mail detected"
```

---

## 🔧 Calibration  
- Adjust the `scale.set_scale(-7050);` value in the code based on your load cell calibration.  
- Perform a **tare** with the `tare()` function before use.  

---

## 📂 Project Structure  
```
├── mailbox_weight_detection.ino   # Arduino code
├── README.md                      # Documentation
```

---
## 📜 License  
This project is released under the **MIT License**.  
Screenshots:  

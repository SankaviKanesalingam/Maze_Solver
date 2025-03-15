# 🏆 Maze Solver Challenge Project 2024

📌 **Course:** EE4360 - Embedded Systems Design and Programming  
📌 **Platform:** PlatformIO + ATmega328P  
📌 **Hardware:** Ultrasonic sensors, motors, microcontroller  

## 🚀 Overview  
This project is designed to autonomously navigate and solve a maze using ultrasonic sensors for obstacle detection. The system uses an ATmega328P microcontroller programmed in **C++** with **PlatformIO**.  

## 🛠️ Features  
✅ Real-time obstacle detection using ultrasonic sensors  
✅ Pathfinding algorithm to navigate the maze  
✅ Motor control for movement and turning  
✅ Serial communication for debugging and logging  

## 🏗️ Hardware Components  
- ATmega328P Microcontroller  
- Ultrasonic Sensors (HC-SR04)  
- Motor Driver (L298N or similar)  
- DC Motors with wheels  
- Power Supply (Battery or Adapter)  
- Chassis for mounting components  

## 🖥️ Software Setup  
### 1️⃣ Install Dependencies  
Ensure you have PlatformIO installed:  
```bash
pip install platformio
```  
### 2️⃣ Clone the Repository  
```bash
git clone https://github.com/your-repo-name.git
cd your-repo-name
```  
### 3️⃣ Build & Upload Code  
```bash
pio run --target upload
```  
### 4️⃣ Monitor Serial Output  
```bash
pio device monitor
```  

## ⚙️ Algorithm  
The maze-solving logic is based on the **Right-Hand Rule** (or Wall-Following Algorithm):  
1️⃣ Move forward until an obstacle is detected  
2️⃣ Turn right if possible; otherwise, check left  
3️⃣ If both directions are blocked, move backward  
4️⃣ Repeat until the exit is found  

## 📂 Project Structure  
```
📦 MazeSolverProject  
 ┣ 📂 src          # Source code  
 ┃ ┣ 📜 main.cpp   # Main program logic  
 ┣ 📂 include      # Header files  
 ┣ 📂 lib          # Custom libraries  
 ┣ 📂 docs         # Documentation & images  
 ┣ 📜 platformio.ini  # PlatformIO config file  
 ┣ 📜 README.md    # Project details  
```

## 🔧 Future Improvements  
🔹 Optimize pathfinding algorithm  
🔹 Implement dynamic mapping  
🔹 Improve motor speed control  

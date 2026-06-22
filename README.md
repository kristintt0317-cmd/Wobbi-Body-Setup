# Wobbi Robot

**Wobbi** is a character-inspired legged robot companion prototype developed for the *Creative Making and Applied Robotics* unit.  
The project explores how a small physical robot can express personality through walking, wobbling, weight shifting, and simple voice-triggered interaction.

This repository documents the final physical gait prototype, including Arduino gait control, laptop-based voice control, project images, and demo videos.  

Only walking: https://youtube.com/shorts/8o6FKWRD7j8  
Voice control video: https://youtu.be/k7S-Folz8xo  
Camera recognition video:  https://youtu.be/LPa320tfHLo  


---

## Project Overview

The current version of Wobbi focuses on the lower-body walking platform, with additional voice control and camera-based person recognition. It uses two 3D-printed robotic legs, eight bus servos for walking, one optional camera pan servo, two bus servo controllers, two separate batteries, one Arduino UNO, and a laptop for voice and camera processing.

The final walking behaviour is based on a continuous **weight-shift gait**:

1. Shift weight to the left support leg  
2. Bend and lift the right non-support leg  
3. Place the right foot forward while the left support leg bends  
4. Shift weight to the right support leg  
5. Bend and lift the left non-support leg  
6. Place the left foot forward while the right support leg bends  
7. Transfer weight back to the left leg and repeat

Instead of simply swinging both legs, this gait separates the roles of the **support leg** and the **swing leg**, making the motion more readable and character-like.

### Camera Update

A camera module has been added to Wobbi to support basic person recognition and future gaze-based interaction.  
The camera is connected directly to the laptop through USB, while the Arduino continues to control the servos through serial communication.

The current camera script can detect a person and use the visual input as a first step towards responsive behaviour.  
An additional servo motor, ID9, is used as a camera pan servo, allowing the robot to rotate the camera left and right.

This update expands Wobbi from a walking and voice-controlled prototype into a more interactive character-based robot that can begin to notice and respond to people.

---

## Repository Structure

```text
WOBBI-ROBOT/
├── Arduino/
│   ├── Wobbi_gait_v1/
│   │   └── Wobbi_gait_v1.ino
│   └── Wobbi_gait_with_camera_pan_ID9.ino/
│       └── Wobbi_gait_with_camera_pan_ID9.ino
│
├── Image/
│   ├── body_photo/
│   │   ├── body_01.JPG
│   │   ├── body_02.JPG
│   │   ├── body_03.JPG
│   │   ├── final_body.JPG
│   │   └── hardware_setup.JPG
│   │
│   └── python_photo/
│       ├── arduino_test.jpg
│       ├── record1.jpg
│       └── serial_ports_check.jpg
│
├── Python/
│   ├── wobbi_voice_control.py
│   └── wobbi_person_depth_pan_follow.py
│
└── requirements.txt

```

---

## Hardware

The prototype uses:

- Arduino UNO
- 2 × Hiwonder / bus servo controllers
- 8 × bus servos
- 2 × external batteries, one for each controller
- 3D-printed leg and foot parts
- Servo brackets and mechanical fasteners
- USB cable for Arduino serial communication
- Laptop microphone for optional voice control

The Arduino does **not** power the servos. It only sends serial control commands.  
Servo power is supplied separately through the two servo controller boards.

### Added Camera Components

- USB camera / webcam
- Camera mount or 3D-printed bracket
- Camera pan servo, ID9
- Hiwonder bus servo controller
- Laptop for camera processing and voice recognition  
The camera is connected directly to the laptop through USB.  
The Arduino does not process camera images directly.  
The laptop handles camera input and sends movement commands to Arduino through serial communication.
---

## Final Hardware Architecture

```text
Laptop / USB
    ↓
Arduino UNO
D11 TX → Left controller RX
D11 TX → Right controller RX

Arduino GND → Left controller GND
Arduino GND → Right controller GND

Left battery  → Left servo controller → Servo IDs 1–4 + ID9 camera pan servo
Right battery → Right servo controller → Servo IDs 5–8  

ID9 is used as the camera pan servo and is connected to the same bus servo controller as the leg servos.
```

Important wiring rules:

- Arduino D11 is connected to both controller RX pins.
- Arduino GND must be connected to both controller GND pins.
- The two battery positive lines are not connected together.
- Servo power must not pass through a breadboard.
- Emergency stop should still be done by physically cutting servo power.

---

## Servo ID Mapping

| Servo ID | Joint Name | Description |
|---|---|---|
| 1 | L_Ankle_Pitch | Left ankle pitch |
| 2 | L_Knee | Left knee / lower leg joint |
| 3 | L_Hip_Pitch | Left hip forward-backward movement |
| 4 | L_Hip_Yaw | Left hip left-right rotation |
| 5 | R_Hip_Yaw | Right hip left-right rotation |
| 6 | R_Hip_Pitch | Right hip forward-backward movement |
| 7 | R_Knee | Right knee / lower leg joint |
| 8 | R_Ankle_Pitch | Right ankle pitch |
| 9 | Camera_Pan | Camera left-right pan movement |

---

## Calibrated Neutral Pose

The final neutral pose used in the stable gait version is:

| Servo ID | Neutral Position |
|---|---|
| ID1 | 540 |
| ID2 | 470 |
| ID3 | 550 |
| ID4 | 520 |
| ID5 | 540 |
| ID6 | 500 |
| ID7 | 550 |
| ID8 | 550 |
| ID9 | 500 |

---

## Final Gait Parameters

The final Arduino gait version uses the following main tuning values:

```cpp
// ==========================
// Safety range for leg servos
// ==========================
const int SERVO_MIN = 180;
const int SERVO_MAX = 820;

// ==========================
// Gait tuning parameters
// ==========================
int SHIFT_TO_LEFT  = 50;
int SHIFT_TO_RIGHT = 55;

int LEFT_HIP_FORWARD  = 230;
int RIGHT_HIP_FORWARD = 230;

int LEFT_KNEE_LIFT  = 55;
int RIGHT_KNEE_LIFT = 55;

int LEFT_TOE_UP  = 100;
int RIGHT_TOE_UP = 100;

int LEFT_KNEE_PLACE  = 20;
int RIGHT_KNEE_PLACE = 20;

int LEFT_SUPPORT_KNEE_BEND  = 25;
int RIGHT_SUPPORT_KNEE_BEND = 25;

int LEFT_SUPPORT_ANKLE_COMP  = 20;
int RIGHT_SUPPORT_ANKLE_COMP = 20;

int MOVE_TIME_SHIFT = 2400;
int MOVE_TIME_LIFT  = 2500;
int MOVE_TIME_PLACE = 2600;
int MOVE_TIME_FAST  = 1600;
```

---

## Arduino Control

The main Arduino code is located at:

```text
Arduino/Wobbi_gait_with_camera_pan_ID9.ino/Wobbi_gait_with_camera_pan_ID9.ino
```

The Arduino receives single-character serial commands:

| Command | Function                                                                                 |
| ------- | ---------------------------------------------------------------------------------------- |
| `n`     | Move all servos to neutral pose: legs return to neutral and camera pan returns to centre |
| `w`     | Run weight-shift test                                                                    |
| `g`     | Run one complete gait cycle                                                              |
| `r`     | Start repeated gait loop                                                                 |
| `x`     | Stop walking and return the legs to a stable stop pose                                   |
| `u`     | Unload knee servos while the robot is supported                                          |

To run the Arduino code:

1. Open `Wobbi_gait_with_camera_pan_ID9.ino` in Arduino IDE.
2. Select **Arduino UNO** as the board.
3. Select the correct Arduino COM port.
4. Upload the code.
5. Open Serial Monitor at `9600` baud.
6. Type one of the commands above and press Enter.

---
## Python Setup

Install all required Python libraries:

```bash
pip install -r requirements.txt
```

## Voice Control

The voice-control script is located at:

```text
Python/wobbi_voice_control.py
```

It uses the laptop microphone and sends serial commands to Arduino.

Final voice mapping:

| Voice Command | Arduino Command | Behaviour |
|---|---|---|
| `stop` | `n` | Return legs and camera pan to neutral pose |
| `let's go` | `r` | Start repeated walking |
| `hello` | `g` | Run one gait cycle |

The phrase **“let’s go”** was used instead of **“go”** because the single word “go” was not recognised reliably during testing.

Install dependencies:

```bash
pip install pyserial SpeechRecognition pyaudio
```

If `pyaudio` fails to install on Windows, try:

```bash
pip install pipwin
pipwin install pyaudio
pip install pyserial SpeechRecognition
```

Run the script:

```bash
python Python/wobbi_voice_control.py
```

Make sure the Arduino Serial Monitor is closed before running the Python script, because only one program can use the Arduino COM port at a time.


## Camera Recognition and Pan Control

```bash
pip install pyserial opencv-python numpy
```

The camera recognition script is located at `Python/wobbi_person_depth_pan_follow.py`. The camera is connected directly to the laptop through USB, and the laptop processes the camera input. The Arduino does not process image data directly; it only receives serial commands from the Python script. In the current version, the camera system is used for basic person recognition and camera pan behaviour.

The camera pan servo is assigned as **ID9**, allowing Wobbi to rotate the camera left and right. Before running the script, make sure the USB camera is connected, the Arduino is connected to the correct COM port, the servo controllers are powered by external batteries, and the Arduino Serial Monitor is closed. Run the script with `python Python/wobbi_person_depth_pan_follow.py`.

---

## Demo

The updated prototype includes:
- basic walking gait
- voice command control
- camera module installation
- camera pan movement using servo ID9

---

## Images

Project images are stored in the `Image/` folder.

### Body and Hardware Photos

```text
Image/body_photo/
```

This folder includes body and hardware setup images:

- `body_01.JPG`
- `body_02.JPG`
- `body_03.JPG`
- `final_body.JPG`
- `hardware_setup.JPG`

### Python and Serial Testing Screenshots

```text
Image/python_photo/
```

This folder includes screenshots from Arduino and Python testing:

- `arduino_test.jpg`
- `record1.jpg`
- `serial_ports_check.jpg`

---

## Safety Notes

This prototype uses high-torque servos and external batteries. Always test carefully.

Important safety rules:

- Support the robot during early gait tests.
- Keep fingers away from moving joints.
- Stop testing if a servo becomes hot.
- Do not let servo power pass through a breadboard.
- Do not join the positive terminals of the two batteries.
- Use physical power-off as the real emergency stop.
- Use the `u` command only when the robot is supported, because unloading the knee servos will make the legs soft.

---

## Current Status

The current prototype has achieved:

- Two physical robotic legs assembled
- Eight bus servos tested and mapped
- Arduino serial control established
- Two-controller / two-battery architecture working
- Stable neutral pose calibrated
- Continuous weight-shift gait implemented
- Simple laptop-based voice control tested
- Camera module added to the robot body/head area
- Basic camera-based person recognition tested
- Camera pan servo ID9 added and tested
- Demo videos recorded

The prototype is not yet a fully autonomous robot. It is an open-loop physical gait prototype intended to explore how walking can become a character-like robotic behaviour.

---

## Next Steps

Future development could include:

- Improving foot friction and stability
- Adding an external character shell
- Adding expressive eyes
- Adding head movement
- Adding sound feedback
- Improving cable management
- Connecting walking to approach or follow behaviour
- Testing more robust gait timing and sensor feedback
- Improving camera-based person tracking
- Connecting person detection to walking or following behaviour
- Making camera pan movement more expressive

---

## Project Context

This project was developed for the **Creative Making and Applied Robotics** unit as part of the MSc Creative Robotics programme.

The project explores how a robot can use body movement, weight shifting, and gait rhythm to suggest personality and presence before a complete character shell or face is added.

---

## Author

**Tingting Liao**  
MSc Creative Robotics  
University of the Arts London

# Wobbi Robot

**Wobbi** is a character-inspired legged robot companion prototype developed for the *Creative Making and Applied Robotics* unit.  
The project explores how a small physical robot can express personality through walking, wobbling, weight shifting, and simple voice-triggered interaction.

This repository documents the final physical gait prototype, including Arduino gait control, laptop-based voice control, project images, and demo videos.

---

## Project Overview

The current version of Wobbi focuses on the lower-body platform. It uses two 3D-printed robotic legs, eight bus servos, two bus servo controllers, two separate batteries, and one Arduino UNO.

The final walking behaviour is based on a continuous **weight-shift gait**:

1. Shift weight to the left support leg  
2. Bend and lift the right non-support leg  
3. Place the right foot forward while the left support leg bends  
4. Shift weight to the right support leg  
5. Bend and lift the left non-support leg  
6. Place the left foot forward while the right support leg bends  
7. Transfer weight back to the left leg and repeat

Instead of simply swinging both legs, this gait separates the roles of the **support leg** and the **swing leg**, making the motion more readable and character-like.

---

## Repository Structure

```text
WOBBI-ROBOT/
├── Arduino/
│   └── Wobbi_gait_v1/
│       └── Wobbi_gait_v1.ino
│
├── Image/
│   ├── body_photo/
│   │   ├── body_01.JPG
│   │   ├── body_02.JPG
│   │   ├── body_03.JPG
│   │   └── hardware_setup.JPG
│   │
│   └── python_photo/
│       ├── arduino_test.jpg
│       ├── record1.jpg
│       └── serial_ports_check.jpg
│
├── Python/
│   └── wobbi_voice_control.py
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

Left battery  → Left servo controller → Servo IDs 1–4
Right battery → Right servo controller → Servo IDs 5–8
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

---

## Final Gait Parameters

The final Arduino gait version uses the following main tuning values:

```cpp
const int SERVO_MIN = 180;
const int SERVO_MAX = 820;

int SHIFT_TO_LEFT  = 50;
int SHIFT_TO_RIGHT = 55;

int LEFT_HIP_FORWARD  = 200;
int RIGHT_HIP_FORWARD = 200;

int LEFT_KNEE_LIFT  = 55;
int RIGHT_KNEE_LIFT = 55;

int LEFT_TOE_UP  = 100;
int RIGHT_TOE_UP = 100;

int LEFT_KNEE_PLACE  = 20;
int RIGHT_KNEE_PLACE = 20;

int LEFT_SUPPORT_KNEE_BEND  = 25;
int RIGHT_SUPPORT_KNEE_BEND = 25;

int LEFT_SUPPORT_ANKLE_COMP  = 15;
int RIGHT_SUPPORT_ANKLE_COMP = 15;

int MOVE_TIME_SHIFT = 2400;
int MOVE_TIME_LIFT  = 2500;
int MOVE_TIME_PLACE = 2600;
int MOVE_TIME_FAST  = 1600;
```

---

## Arduino Control

The main Arduino code is located at:

```text
Arduino/Wobbi_gait_v1/Wobbi_gait_v1.ino
```

The Arduino receives single-character serial commands:

| Command | Function |
|---|---|
| `n` | Move to neutral pose |
| `w` | Run weight-shift test |
| `g` | Run one complete gait cycle |
| `r` | Repeat gait loop |
| `x` | Stop repeat gait and return to neutral |
| `u` | Unload knee servos while the robot is supported |

To run the Arduino code:

1. Open `Wobbi_gait_v1.ino` in Arduino IDE.
2. Select **Arduino UNO** as the board.
3. Select the correct Arduino COM port.
4. Upload the code.
5. Open Serial Monitor at `9600` baud.
6. Type one of the commands above and press Enter.

---

## Voice Control

The voice-control script is located at:

```text
Python/wobbi_voice_control.py
```

It uses the laptop microphone and sends serial commands to Arduino.

Final voice mapping:

| Voice Command | Arduino Command | Behaviour |
|---|---|---|
| `stop` | `n` | Return to neutral pose |
| `let's go` | `r` | Start repeated walking |
| `hello` | `g` | Run one gait cycle |

The phrase **“let’s go”** was used instead of **“go”** because the single word “go” was not recognised reliably during testing.

### Python Setup

Create and activate a Python environment:

```bash
conda create -n wobbi_voice python=3.10 -y
conda activate wobbi_voice
```

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

---

## Demo Videos

Demo videos are stored in the `Video/` folder.

| File | Description |
|---|---|
| `move_01.MP4` | Walking / gait test video |
| `move_02.MP4` | Additional movement test |
| `move_voice.MP4` | Voice-control demo |

Recommended demo sequence:

1. Neutral pose  
2. Weight-shift test  
3. One complete gait cycle  
4. Repeat gait loop  
5. Voice control: “stop”, “let’s go”, “hello”

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

---

## Project Context

This project was developed for the **Creative Making and Applied Robotics** unit as part of the MSc Creative Robotics programme.

The project explores how a robot can use body movement, weight shifting, and gait rhythm to suggest personality and presence before a complete character shell or face is added.

---

## Author

**Tingting Liao**  
MSc Creative Robotics  
University of the Arts London

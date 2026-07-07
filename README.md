# Hand-Controlled Drone

A fully custom quadcopter built from raw components — no pre-made flight controller, no off-the-shelf RC transmitter. The drone is stabilized by a self-written PID control loop running on an ESP32, and flown by tilting your hand: a second ESP32 + IMU worn on a glove/wrist strap reads your hand's orientation and sends it wirelessly to the drone in real time.

Built for [Hack Club Outpost](https://outpost.hackclub.com/).

## How it works

- A **hand controller** (ESP32 + MPU6050 + nRF24L01) reads your hand's tilt (roll/pitch) and wrist twist (yaw rate), plus throttle from a potentiometer and an arm/disarm button, and transmits it over a 2.4GHz radio link.
- The **drone** (ESP32 + MPU6050 + nRF24L01 + 4-in-1 ESC) receives that packet, reads its own current attitude from its onboard IMU, runs a PID loop comparing the two, and mixes the correction into 4 individual motor speeds.
- A **failsafe** cuts the motors automatically if the radio signal is lost for more than 300ms.

## Features

- Fully custom PID stabilization — no Betaflight/existing flight controller firmware
- Custom wireless control protocol over nRF24L01+PA+LNA (long-range variant)
- Hand-gesture flight control instead of a standard RC transmitter
- Signal-loss failsafe
- Pendulum-style frame design: flight electronics isolated from motor vibration on damped standoffs, battery mounted low for passive self-leveling stability

## Hardware

| | |
|---|---|
| MCU | ESP32-WROOM-32 (x2 — one per drone, one for the hand controller) |
| IMU | MPU6050 (x2) |
| Radio | nRF24L01+PA+LNA (x2) |
| Motors | 2205 2300KV brushless (x4) |
| ESC | 30A 4-in-1, BLHeli_S |
| Battery | 4S 1500mAh LiPo |
| Frame | Custom-designed two-tier stacked frame, 3D printed |

Full bill of materials with part links: [`docs/drone_bom.csv`](docs/drone_bom.csv)

## Repository structure

```
├── firmware/
│   ├── drone_flight_controller/
│   │   └── drone_flight_controller.ino   # Runs on the drone's ESP32
│   └── hand_controller/
│       └── hand_controller.ino           # Runs on the glove/handheld ESP32
├── hardware/
│   ├── schematics/                       # KiCad schematics (drone + glove)
│   └── cad/                              # Frame CAD files
├── docs/
│   └── drone_bom.csv                     # Full bill of materials
└── README.md
```

## Firmware

Both sketches are written for the Arduino IDE (ESP32 board package).

**Libraries required:**
- [RF24](https://github.com/nRF24/RF24) by TMRh20
- [MPU6050_light](https://github.com/rfetick/MPU6050_light) by rfetick
- [ESP32Servo](https://github.com/madhephaestus/ESP32Servo) by Kevin Harrington (drone side only)

**Pin assignments** are defined as `#define`s at the top of each `.ino` file — update them to match your own wiring if it differs from the schematics in `hardware/schematics/`.

### Hand controller (`firmware/hand_controller/`)
Reads hand tilt (roll/pitch) and wrist twist rate (yaw) from the MPU6050, reads throttle from a potentiometer, and toggles an armed/disarmed state on each button press. Packages all of it into a struct and transmits it over the radio at ~50Hz.

### Drone flight controller (`firmware/drone_flight_controller/`)
Receives the control packet, reads its own attitude from its onboard IMU, and runs independent PID loops for roll, pitch, and yaw rate. Outputs are mixed using standard X-quad motor mixing and written to the ESC as PWM (1000–2000µs). Motors are held at minimum signal whenever the drone is disarmed or the signal has timed out.

## Building it

1. Solder the ESP32, MPU6050, and nRF24L01 onto perfboard per the schematics in `hardware/schematics/`
2. Mount the electronics stack and ESC/PDB on the 3D-printed frame using vibration-damping standoffs
3. Flash `hand_controller.ino` to the hand controller's ESP32 and `drone_flight_controller.ino` to the drone's ESP32
4. **Remove propellers** and bench-test: verify the IMU calibrates, the radio link connects, and each motor spins the correct direction when armed
5. Tune the PID gains at the top of `drone_flight_controller.ino`, starting with P only, then adding D, then I if needed
6. Only attach propellers once motor response and PID behavior look correct on the bench

## Safety notes

- Always test with propellers **off** first
- Keep hands and body clear of the propeller plane during any powered test
- The arm/disarm button and failsafe are there for a reason — don't bypass them
- LiPo batteries can be dangerous if punctured, overcharged, or short-circuited — always use a balance charger and never leave a charging battery unattended

## Status

Work in progress, built for the Hack Club Outpost hardware program (2026).

## Pictures

<img width="1613" height="779" alt="image" src="https://github.com/user-attachments/assets/54587ae7-fde4-4836-a1b2-5c0367ac513a" />
<img width="1456" height="827" alt="image" src="https://github.com/user-attachments/assets/9992bacd-de86-47b3-b679-e8b23a92c7a5" />
<img width="1384" height="868" alt="image" src="https://github.com/user-attachments/assets/0a3fcb33-52e6-4a00-8a83-8c7dd3a592c7" />



## License

MIT

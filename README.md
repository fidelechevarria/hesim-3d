# hesim-3d ⚡🎥

**A high-performance 3D trajectory simulator for Hybrid Event-Frame Sensors and Event Cameras.**

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![Render: Google Filament](https://img.shields.io/badge/Render-Google_Filament-orange.svg)](https://github.com/google/filament)
[![Sensor: H-ESIM ECCV'26](https://img.shields.io/badge/Sensor-H--ESIM_ECCV'26-green.svg)](https://yunfanlu.github.io/HESIM)

`hesim-3d` connects high-speed 3D PBR offscreen rendering (Google Filament / Vulkan) with the physics-based statistical noise models of **H-ESIM** (ECCV 2026), generating synchronized **RAW frames, event streams, synthetic IMU telemetry, and 6-DoF ground truth poses** with zero disk overhead.
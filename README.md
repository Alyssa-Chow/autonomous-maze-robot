Autonomous Maze-Navigating Robot
An autonomous ground robot built with a Raspberry Pi, developed as part of the Global Emergency Autonomous Response System (GEARS) project. The robot navigates a simulated disaster zone maze, detects and avoids environmental hazards, generates a path map, and delivers a cargo payload to a target destination.
What it does:

Navigates an unknown maze in real time using three ultrasonic sensors (front, left, right)
Detects and avoids two hazard types: heat sources via IR sensor and electromagnetic sources via IMU sensor
Generates a CSV grid map of the path taken and a hazard report with coordinates and field strength readings
Delivers a 3D-printed cargo container via a motorized sliding ramp upon exiting the maze

Tech stack: Python, Raspberry Pi, BuildHAT/BaseHAT motor and sensor libraries, NumPy
I designed and wrote all software for this project, including the navigation and maze logic, sensor fusion and hazard detection algorithms, mapping and CSV output code, all motor control functions, and the testing methodology. Built as part of a team project where I led the software development end to end.
Course: ENGR 16200 — Purdue University

    # Arduino_Robot
Arduino based Robot
Uses Code from previous projectfor ELEC 291 - UBC
- For testing purposes, I will only be using the original code
- After extensively testing the Ultrasonic sensors and possibly new IR sensors,
I will start with fresh

Goals:

1. Autonomous robot that does not need any input to move
2. Does not bump into any walls
3. Moves around the room "smartly"
    1. Current thoughts: Move through a room once using object detection
    2. Want to also save an image of the ceiling for each room and after one run,
    and will be able to differentiate between rooms and apply previous movements
    3. Be able to recognize where is a cliff, and stop before falling off
4. Control the robot via Bluetooth using my Android Phone

The project is very similar to a Roomba, as it does most of those things. I have always
been interested in robotics and would love to apply the knowledge I learnt about embedded
software systems into my own projects. This is simply my own personal project.

Plan:
- Start to test new sensors during the month of October
- Create new code from scratch that does not use test code from previous project
- Aim to finish basic movement in straight lines and turns by the end of November
    - Aim to look into turning efficiently with the right angle with improved Ultrasonic and IR sensors
- Aim to start cliff hanging by mid December
- Aim to start Android application by mid December after cliff recognition
- Aim to develop algorithm for mapping rooms with MATLAB / Octave by the Feburary (2017)

This plan is not a strict plan per se, but rather a guideline for me to remind myself of
my project in my busy school days.

Research:
- http://spectrum.ieee.org/automaton/robotics/robotics-software/how_roomba_moves 
- http://spectrum.ieee.org/automaton/robotics/home-robots/review-irobot-roomba-980

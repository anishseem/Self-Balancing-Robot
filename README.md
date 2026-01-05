This is a personal project focused on building a inverted pendulum robot, also known as a self balancing robot. 
The robot utilizes PID control to maintain balance and stay upright.
Currently the code only operates to maintain the upgright position, however, I will be adding positional control (forward/backward & rotation) using a remote and the RF transciever module on board.
Attached are all the related project files including the custom PCB gerber files, aseembly STEP files, balance code, and visual processing code.
The 3d processing, real time visualization code does not properly work in conjunction with the balancing code due to the constraints of the arduino processor. So right now, I recommend commenting out the update motor function when using it. 
<img width="1079" height="666" alt="Main_Assembly" src="https://github.com/user-attachments/assets/a9bf515c-cad4-43cf-880c-d456867bb550" />
<img width="1216" height="728" alt="Screenshot 2026-01-04 at 11 31 42 PM" src="https://github.com/user-attachments/assets/a725f97b-183e-4ac1-ae29-1e97b4f9ea11" />
<img width="1218" height="901" alt="Screenshot 2026-01-04 at 11 32 13 PM" src="https://github.com/user-attachments/assets/f393e695-ddfd-4b82-9ac9-bd527c1f0b1a" />
[![Watch the video](https://img.youtube.com/vi/x7obqyywPWY/hqdefault.jpg)](https://www.youtube.com/watch?v=x7obqyywPWY)

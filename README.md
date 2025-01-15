# Line-following Robot-Car with Crash Detection
This repository hosts the code for the line-following robot car with crash detection built for the ME 461 (Computer Control of Mechanical Systems) course at UIUC in the Fall semester of 2022.

The project included 2 subsystems (the TI microcontroller and Raspberry Pi).

### OpenCV Blob detection
This is implemented in ME461FinalProject_RPi_code\finalproject-master\blob_new.cpp<br>
Note: The final2 directory includes the blob-detect code for the 2-line following implementation<br>

### Line-following + Crash Detection
This is implemented in the main TI microcontroller code found in ME461FinalProject_TI_code\ME461_repo-main\workspace\FinalProject\FinalProject_main.c<br>
Note: the FinalProject2line_following directory includes the code for the two-line following implementation<br>

I worked on the two-line following code that. The centroids of the two largest blobs are detected by the RPi. Their X coordinates are received by the TI Launchpad microcontroller. The one-line following proportional control law is used by averaging these two values<br>
```c
cent_avg = (cx1+cx2)/2;
centerr = cent_avg -80;

// line-following case
Vref = 0.25; // keep moving forward when following a line
turn = -0.013*(centerr);
```

<br>
Team members are: Ananda Sangli, Anisha Shukla, Christopher Conway, Jiwon Shin
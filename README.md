# SDP2027-Team34 wooo

## Aamir: main.c, acq.c, baord.c (Aamir and Nimon parts shared w/ eachother)
Do the first real idf.py build of the skeleton, fix the compile errors, and get it running in Wokwi. Nobody else can test on the device until this is done.
On the Feather, bring up the IMU: confirm the chip ID reads 0xE1 and find the I²C address.
Prove 1 kHz sampling: overruns = 0 and max_loop well under 1000 µs. If the loop is too slow, switch the FSRs to DMA ADC reads or use the IMU’s FIFO.
Act as integration owner: review merges and keep the main branch building.

## Nimon: fsr.c, imu.c
Choose the divider resistor. Start at 10 kΩ, then check that a real kick doesn’t push the reading to the 4095 maximum while a light touch still registers.
Add a baseline step to fsr.c: record each FSR’s resting value at boot, so boot pressure doesn’t shift the thresholds.
Co-lead the data-collection session with Alex.

## Alex: detect.c, test/, and laptop-tools/
Replay tool (most useful item). A small laptop program that compiles the same detect.c, runs it over a recorded SG0001.BIN, and prints the events it finds. Threshold changes can then be tested on real recordings in seconds, without reflashing or needing the one board. He can build it this week on fake or Wokwi data.
Labeling protocol and dataset. Write the script for the recording session, for example 20 shots and 20 touches per zone plus 2 minutes of walking and jogging. Also define a simple label file (a timestamp and the true kind for each event).
Accuracy script. Compare detector output against the labels and report touch/shot/step accuracy and a confusion matrix. This is your S1 evidence for MDR.
Threshold tuning. Run the replay tool across a range of threshold values, choose the numbers for config.h, and decide whether peak acceleration alone is enough. If it isn’t, add another feature to detect.c, such as peak gyro rate or how fast the FSR reading rises.
Filtering. If foot bending or noise causes false contacts, add smoothing or hysteresis to detect.c. Diego can review this, given his DSP background.
After MDR: the stretch ML classifier, trained on the same labeled dataset.


## Neel: logger.c, log_format.h, sg_decode.py
Get SD logging working on hardware. Record 10+ minutes with dropped_records = 0 and check the file decodes cleanly.
Add a start/stop button on the Feather’s BOOT button (GPIO0) that closes the file cleanly. It touches acq.c, so coordinate with Aamir.
Grow sg_decode.py into a plotting tool: FSR channels and acceleration over time, with touch/shot markers. This is literally the MDR demo item (“show zone and acceleration plots”).
Then start the app GUI.

## Diego: pin map in config.h, board.c changes based on PCB
Keep config.h in sync with the PCB schematic, so the firmware runs on PCB v1 unchanged except for that file.
Measure the dev board’s real current draw while logging. That checks the PDR’s 118 mA estimate and battery-life risk.


## Upload Rules:
config.h: anyone can propose a change; the owner of the affected value reviews it.
log_format.h: changes need both Neel and Alex to agree, since both their tools read the file.
Branches: one git branch per person, merged through pull requests that Aamir reviews.


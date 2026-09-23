# concretePrinter
concrete printer for arduino

WIP untested still

for 3knucklepicker
*   **Joint Commands (`G1 D2500`):** This will command the 4th lever to extend the boom to 2500mm.
*   **Cartesian Commands (`G1 X100 Y100 Z100`):** This will move the nozzle to that exact location using Slew, Boom, and Knuckle, while keeping the telescoping extension perfectly still.

**Important Hardware Calibration Step:**
Be sure to measure exactly how many stepper motor pulses (steps) it takes to move the extension cylinder 1 millimeter, and update `COUNTS_PER_UNIT[3]` with that number!
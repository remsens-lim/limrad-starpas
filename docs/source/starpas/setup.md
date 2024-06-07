(setup)=
# Setup
Here, the setup procedure of STARPAS is described. The subsections are sorted in the order of intended setup procedure steps:

1. Hardware is attached to the stabilization platform of the radar
2. The stabilization platform + radar + STARPAS is fixed on the ship
3. Set up the ethernet connection and processing software
3. STARPAS initial calibration of the magnetic sensor


(setup-hardware)=
## Hardware
* **When?**: Before the radar is attached to the stabilization platform!
* **What?**: The backplate of STARPAS will be attached to the inner stabilization platform frame (right below the radar)
* **How?**:
  1. The backplate is placed in between the stickers at the inner frame of the stabilization platform below the radar
  2. The cables face downward
  3. Fix the screw clamps tight to hold it in place
  4. The GPS-module can be placed somewhere at the frame for now. Later it will be attached to the ships railing

(setup-software)=
## Software

(setup-calibration)=
## Calibration
* **When?**: Stabilization platform and radar fully assembled and fixed on the ship, software setup done.
* **What?**: Two steps: (i) Magnetic sensor calibration, (ii) retrieve Offset-Angle 
* **How?**:
  1. **Magnetic sensor calibration**
     1. Open the STARPAS box and connect the USB cable to the ESP32 and Laptop
     > See also USB-Driver and software https://www.olimex.com/Products/IoT/ESP32/ESP32-POE-ISO/open-source-hardware
     2. If Arduino IDE is installed on laptop, install the ESP32 library and connect to the board (**OLIMEX ESP32-PoE-ISO**), if no Arduino IDE identify the USB-COM port
     > The proper setup of the Arduino IDE to reprogramm or update the STARPAS is described in [](arduino-setup) 
     3. Open serial monitor (e.g., via Arduino IDE) - 115200 baud
     4. The board now enters calibration mode, now move the platform around until the "Algorithm Status" byte changes from 0 to 8. See below.
     > From https://github.com/gregtomasch/EM7180_SENtral_Calibration :
     > ### Basic Operation and Screen Messaging
     > * Select the appropriate sketch for your microcontroller development board (STM432L4, ESP32 or Teensy 3.x) and USFS board (Invensense or ST sensors). **A word to the wise: Running an ST USFS with an Invensense sketch (or the other way around) WILL NOT WORK**. *Either the sketch will hang or the AHRS data will be nonsense*
     > * Follow the hardware-specific interconnection instructions in the "Readme.md" file for the specific sketch you have chosen.
     > Build/upload the sketch from the [Arduino IDE](https://www.arduino.cc/en/main/software). Power-cycle the board and open the Arduino serial monitor (or any terminal emulator you prefer)
     > * You should see a startup sequence on the Arduino serial monitor similar to fht following screen capture:
     > 
     > ![alt text](https://user-images.githubusercontent.com/5760946/53260187-bfec5e80-3685-11e9-80a2-7922921492f4.png)
     > 
     > * The accelerometer calibration data are displayed as part of the Sentral startup procedure. The data are in ADC counts where 1g = 2048. If all three accelerometers were perfect, maxima and minima would all be +/- 2048, respectively. If any of the data vary more than +/-~10% from their target values, the entire calibration is judged as invalid and is not loaded into the Sentral.
     > * When the Warm Start parameters are retrieved from the EEPROM, the final byte of the sequence is a "Validation byte". This byte is erased immediately before Warm Start data is written to the EEPROM and is re-written once Warm Start data is successfully saved. This byte is checked at startup; if the value is correct, the Warm Start parameter data is loaded into the Sentral. Otherwise, Warm Start parameters are not loaded into the Sentral and will be reflected in the startup screen messaging.
     > Once Sentral Startup is complete, the main loop will start running and sensor/AHRS data will stream across the Arduino serial monitor as shown in the following screen capture:
     > 
     > ![alt text](https://user-images.githubusercontent.com/5760946/53261426-d8aa4380-3688-11e9-96fa-e353c9dec2d8.png)
     > 
     > * In this serial monitor screen capture, the "Algorithm Status" field is highlighted. It will initially assume a value of "0" when the USFS first starts up and has not experienced any significant motion
     > * The balance of the update screen includes:
     >     - X, Y, and Z accelerometer data in milli-g's (mg)
     >     - X, Y, and Z gyroscope data in degrees per second (deg/s)
     >     - X, Y, and Z magnetometer data in micro-Teslas (uT)
     >     - Unit quaternion coefficients (dimensionless)
     >     - Euler angles (Yaw, Pitch and Roll) in degrees
     >     - Barometric pressure in milli-bars (mbar)
     >     - Ambient temperature in degrees Celsius (deg C)
     >     - Loop cycle time in microseconds (us). The loop cycle time will fluctuate significantly because it is dominated by I2C data transfer time for whatever new data is available at any given cycle of the loop...
     > * The accelerometer data is also specifically highlighted in this screen capture as this data is crucial for the accelerometer calibration procedure. In this example, the Z-accelerometer is aligned parallel to gravity; az ~ 1000mg and ax, ay <+/-30mg. This orientation of the USFS would be satisfactory for collecting the "Z-acc max" accelerometer calibration data to be stored in the EEPROM...
     > * Next, randomly rotate the USFS throughout 3-space while taking note of the "Algorithm status" byte:
     > 
     > ![alt text](https://user-images.githubusercontent.com/5760946/53261503-0c856900-3689-11e9-8f64-ae64f5c1c9af.png)
     > 
     > * When this byte toggles from "0" (Indeterminate) to "8" (Calibration stable) the SpacePoint algorithm should be converged to a stable operating point
     5. When Algorithm Status shows "8" send "2"  via the serial monitor to save the warm start parameters. The System is now stable
     6. Disconnect the USB and close the box. 
  2. **Retrieve Offset-Angle**
  > Do this while in port, when the ship does not move!
     1. Connect the STARPAS to the Ethernet (PoE) and wait 10min (see [](setup-software))
     2. Loosen the straps of the stabilization platform and switch on the radar.
     3. Give the radar a good swing - so it will level in neutral position. 
     3. Write down the angles reported by the radar software (Elevation Angle and Elevation Axis Angle) and the time.
     4. Retrieve the STARPAS data of the noted time

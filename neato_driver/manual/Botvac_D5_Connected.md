#Documentation of Neato's serial port interface
## Communication through the USB port

Remove dust filter and plug in a USB cable to the USB port and to 
your computer. Then you can use a terminal to communicate with the 
robot. For example you can use screen, with the device as argument. 
The device can be found in /dev folder, usually it is ttyACM0, 
sometimes it is ttyACM1. You can type ''screen /dev/ttyACM0'' to 
start the communication. Type the first command, for example help, 
and press Enter.  

## Command syntax

Every commant starts with its name, followed by arguments. The 
arguments are either flags or key value pairs. The arguments are 
seperated by spaces, keys and values too. Commands and arguments 
are not case insensitive, even partial commands are supported.

## Response syntax

Command responses are in csv (comma seperated values), beginning 
with a title line. Each response ends with control-z

## List of Commands

The following commands are supported by the robot:

###Help

**Description:** Without any argument, this prints a list of all possible cmds.

**Options:** 

| Flag | Description |
|------|-------------|
|     Cmd | (Optional) Next argument is command to show help for. |


###Clean

**Description:** Starts a cleaning by simulating press of start button.

**Options:** 

| Flag | Description |
|------|-------------|
|     Spot | (Optional) Starts a spot clean. (Not available with AutoCycle) <br> (Choose only 1 of House,Spot,Stop) |
|     Stop | Stop Cleaning. <br> (Choose only 1 of House,Spot,Stop) |
|     Height | (Optional) Spot Height in CM (100-400)(-1=use default). |
|     House | (Optional) Equivalent to pressing 'Start' button once. <br> Starts a house cleaning. <br> (House cleaning mode is the default cleaning mode.) <br> (Choose only 1 of House,Spot,Stop) |
|     Width | (Optional) Spot Width in CM (100-400)(-1=use default). |


###ClearFiles

**Description:** Erases Black Box, and other Logs

**Options:** 

| Flag | Description |
|------|-------------|
|     BB | (Optional) Clears Managed Logs in BlackBox Directory. <br>  |
|     All | (Optional) Additionally clears unmanaged files (Crash, ...) in the specified directories. <br>  |


###DiagTest

**Description:** Executes different test modes. Once set, press Start button to engage. (Test modes are mutually exclusive.)

**Options:** 

| Flag | Description |
|------|-------------|
|     AutoCycle | DropTest argument to enable automatic restart of the test. The robot will drive backwards and then forward until a drop is detected until the test is over. <br>  |
|     BrushSpeed | DropTest argument to set the speed of the brush in rpm. <br>  |
|     DisablePickupDetect | Ignores pickup (wheel suspension). By default, pickup detect is enabled and stops the test. <br>  |
|     DriveForeverRightDist | Use next arg to set right wheel dist for DriveForever test. Requires DriveForeverLeftDist as well. The ratio of this value to DriveForeverLeftDist determines turn radius. <br>  |
|     MoveAndBump | Enables 'Move and Bump' TestMode. Executes canned series of motions, but will react to bumps <br> (DrivePath|DriveForever|MoveAndBump|Drop) <br>  |
|     OneShot | Only executes test once. <br>  |
|     DriveForeverSpeed | Use next arg to set turn speed of outer wheel for DriveForever test in mm/s. <br>  |
|     AllMotorsOn | Turns on brush, vacuum, and lds during test. May conflict with motor commands of test so use carefully! <br>  |
|     Speed | DropTest argument to set the robot speed in mm/s. <br>  |
|     BrushOn | Turns on brush during test. May conflict with motor commands of test so use carefully! <br>  |
|     TestsOff | Stop Diagnostic Test and clear all diagnostic test modes.  <br>  |
|     DropTest | Enables 'Drop' Test. Robot drives forward until a drop is detected. <br> (DrivePath|DriveForever|MoveAndBump|Drop) <br>  |
|     DrivePathDist | Distance in mm <br>  |
|     VacuumOn | Turns on vacuum during test. May conflict with motor commands of test so use carefully! <br>  |
|     DrivePath | Enables 'DrivePath' TestMode. Robot travels straight by commanded distance as path. <br> (DrivePath|DriveForever|MoveAndBump|Drop) <br>  |
|     SideBrushOn | Turns on side brush during test. May conflict with motor commands of test so use carefully! <br>  |
|     DriveForever | Enables 'DriveForever' TestMode.Robot drives continuously. <br> (DrivePath|DriveForever|MoveAndBump|Drop) <br>  |
|     DriveForeverLeftDist | Use next arg to set left wheel dist for DriveForever test. Requires DriveForeverRightDist as well. The ratio of this value to DriveForeverRightDist determines turn radius. <br>  |
|     LDSOn | Turns on LDS during test. May conflict with motor commands of test so use carefully! <br>  |


###GenerateRobotLinkCode

**Description:** Generate and send robot linking code to server.

**Options:** 

| Flag | Description |
|------|-------------|
|     brief | Start the linking process.  This will initiate a link robot command. <br>       SerialNumber -> Serial number of this robot <br>  |
|     SerialNumber | The serial number of the robot |


###GetConfiguredWifiNetworks

**Description:** Get the list of configured wifi networks.

**Options:** 

| Flag | Description |
|------|-------------|
|     brief | Returns Wifi Configuration. Data order: <br>       SSID <br>       signal strength <br>  |


###GetRobotLinkCode

**Description:** Get the robot linking code

**Options:** 

| Flag | Description |
|------|-------------|
|     brief | Get the robot linking code. <br>       Value: <value to enter at server> <br>       Expiry Time: <how long it will be valid for in seconds>  <br>  |


###CancelRobotLink

**Description:** Cancel link request.

**Options:** 

| Flag | Description |
|------|-------------|
|     brief | Cancel the link request. <br>  |
|     SerialNumber | The serial number of the robot |


###SetNTPTime

**Description:** Set system time using the NTP servers (WIFI must be up for this to work)

**Options:** 

| Flag | Description |
|------|-------------|
|     brief | Instruct Astro to get the time from an NTP Server <br>  |


###GetAccel

**Description:** Get the Accelerometer readings.

**Options:** 

| Flag | Description |
|------|-------------|
|     brief | Returns a single-line summary. Data order: <br>       Revision(0) <br>       Pitch <br>       Roll <br>       X Axis <br>       Y Axis <br>       Z Axis <br>  |


###GetAnalogSensors

**Description:** Get the A2D readings for the analog sensors.

**Options:** 

| Flag | Description |
|------|-------------|
|     stats | Return stats (avg,max,min,dev,cnt) of raw analog sensor values as milliVolts. <br>   (Implies 'raw' option) |
|     AllData | Return all readings followed by stats (avg,max,min,dev,cnt) of raw analog sensor values as milliVolts. <br>   (Implies both 'stats' and 'raw' options) |
|     raw | Return raw analog sensor values as milliVolts.  <br> (Default is sensor values in native units of what they measure.) |
|     nSamples | Number of samples of data to take for stats. Default (N_STAT_SAMPLES) is 50. Limit is 200. |


###GetButtons

**Description:** Get the state of the UI Buttons.

**Options:** 

| Flag | Description |
|------|-------------|
|     brief | Returns a single-line summary. Data order: <br>       Revision(0) <br>       Soft key <br>       Up <br>       Start <br>       Back <br>       Down <br>       Spot <br>  |


###GetCalInfo

**Description:** Prints out the cal info from the System Control Block.

###GetCharger

**Description:** Get the diagnostic data for the charging system.

###GetDigitalSensors

**Description:** Get the state of the digital sensors.

**Options:** 

| Flag | Description |
|------|-------------|
|     brief | Returns a single-line summary. Data order: <br>       Data Format Revision <br>       DC Jack <br>       Dustbin <br>       Left Wheel <br>       Right Wheel <br>       Left Side Bumper <br>       Left Front Bumper <br>       Right Side Bumper <br>       Right Front Bumper <br>  |


###GetErr

**Description:** Get Error Message.

**Options:** 

| Flag | Description |
|------|-------------|
|     Clear | Dismiss the reported error. |


###GetLDSScan

**Description:** Get scan packet from LDS.

###GetMotors

**Description:** Get the diagnostic data for the motors.

**Options:** 

| Flag | Description |
|------|-------------|
|     brief | Returns a single-line WHEEL ONLY summary. Data order: <br>       Revision(0) <br>       Left Wheel RPM <br>       Left Wheel Position MM <br>       Left Wheel Speed <br>       Right Wheel RPM <br>       Right Wheel Position MM <br>       Right Wheel Speed <br>  |
|     LeftWheel | Return LeftWheel Motor stats. |
|     SideBrush | Return Side Brush stats. |
|     LDS | Return LDS Motor stats. |
|     Brush | Return Brush Motor stats. |
|     Vacuum | Return Vacuum Motor stats. |
|     RightWheel | Return RightWheel Motor stats. |


###GetSensor

**Description:** Gets the sensors status ON/OFF (Wall Follower and Ultra Sound Only)

**Options:** 

| Flag | Description |
|------|-------------|
|     Flight | Reports flight sensors status |
|     US | Reports ultra sound sensor status |
|     Wall | Reports wall follower sensor status |
|     Drop | Reports drop sensor status |


###GetTime

**Description:** Get Current Scheduler Time.

###GetVersion

**Description:** Get the version information for the system software and hardware.

###GetWarranty

**Description:** Get the warranty data.

###GetWifiInfo

**Description:** Get a list of available wifi networks.

###GetWifiStatus

**Description:** Get the current status of the wifi.

###GetUserSettings

**Description:** Get the user settings.

**Options:** 

| Flag | Description |
|------|-------------|
|     EcoMode | Returns status of ECO cleaning Mode(ON/OFF)  |
|     Schedule | Returns Scheduled cleanings |
|     WiFi | Returns status of WiFi (ON/OFF) |
|     ButtonClick | Returns status of button click sound (ON/OFF)  |
|     FilterChange | Returns time interval in seconds before sending alert to change filter |
|     Warnings | Returns status of warning sounds (ON/OFF) |
|     BinFullDetect | Returns status of dust bin full detect option (ON/OFF) |
|     IntenseClean | Returns status of Intesne clean option(ON/OFF)  |
|     BrushChange | Returns time interval in seconds before sending alert to change brush |
|     Day | Day of the week to get schedule for. Sun=0,Sat=6. <br>  If not specified, then all days are given. |
|     Melodies | Returns status of melody sounds(ON/OFF) |
|     DirtBin | Returns time interval in minutes before sending alert to change dirt bin |
|     StealthLED | Returns status of LEDs in standby mode (ON/OFF) |


###GetUsage

**Description:** Get usage settings

**Options:** 

| Flag | Description |
|------|-------------|
|     SideBrushArea | Returns the remaining life time of side brush for replacing. |
|     FilterArea | Returns the remaining life time of filter for replacement. |
|     DustbinTime | Returns the remaining life time of dustbin untill next emptying. |
|     TotalCleanArea | Returns total area(in mm) cleaned so far. |
|     TotalCleanTime | Returns total amount of time(in sec) spent cleaning so far. |
|     MainBrushArea | Returns the life of main brush in area. |


###PlaySound

**Description:** Play the specified sound in the robot.

**Options:** 

| Flag | Description |
|------|-------------|
|     Stop | Stop playing sound. |
|     SoundID | Play the sound library entry specified by the number in the next argument. <br>  <br>     Legal values are: <br>  <br>        0 - Waking Up <br>        1 - Starting Cleaning <br>        2 - Cleaning Completed <br>        3 - Attention Needed <br>        4 - Backing up into base station <br>        5 - Base Station Docking Completed <br>        6 - Test Sound 1 <br>        7 - Exploring <br>        8 - ShutDown <br>        9 - Picked Up <br>        10 - Going to sleep <br>        11 - Returning Home <br>        12 - User Canceled Cleaning <br>        13 - User Terminated Cleaning <br>        14 - Slipped Off Base While Charging <br>        15 - Alert <br>        16 - Thank You <br>        17 - Button Press <br>        18 - US speed modifier engaged <br>        19 - US bump engaged <br>        20 - Find me <br>        21 - Easy Connect Success <br>  |


###SetBatteryTest

**Description:** Sets California Energy Commission 10-CFR-430 Battery Charging System Test mode.

###SetButton

**Description:** Simulates a button press.

**Options:** 

| Flag | Description |
|------|-------------|
|     spot | Simulate pressing the spot button |
|     back | Simulate pressing the back button |
|     IRhome | Simulate pressing the down button |
|     IRstart | Simulate pressing the down button |
|     up | Simulate pressing the up button |
|     IRleft | Simulate pressing the down button |
|     IReco | Simulate pressing the down button |
|     IRright | Simulate pressing the down button |
|     soft | Simulate pressing the soft button |
|     IRfront | Simulate pressing the down button |
|     IRback | Simulate pressing the down button |
|     IRspot | Simulate pressing the down button |
|     down | Simulate pressing the down button |
|     start | Simulate pressing the start button |


###SetFuelGauge

**Description:** Set Fuel Gauge Level.

**Options:** 

| Flag | Description |
|------|-------------|
|     Percent | Fuel Gauge percent from -100 to 100 |


###SetIEC

**Description:** Sets the IEC Cleaning Test parameters

**Options:** 

| Flag | Description |
|------|-------------|
|     FloorSelection | Next Arg is the floor type < carpet | hard > |
|     CarpetSpeed | Next Arg is test speed on carpet (10-300mm/s) |
|     Distance | Next Arg is test distance (200-4000 mm, default 1200) |
|     HardSpeed | Next Arg is test speed on hard floors (10-300mm/s) |


###SetLCD

**Description:** Sets the LCD to the specified display. (TestMode Only)

###SetLED

**Description:** Sets the specified LED to on,off,blink, or dim. (TestMode Only)

**Options:** 

| Flag | Description |
|------|-------------|
|     Led12BlinkFast | Set the 1st and 2nd LEDs (BATT Yellow) Blinking |
|     Led12Off | Set the 1st and 2nd LEDs Off |
|     Led4DimSolid | Set the 4th LED (INFO Red) to solid with dim |
|     Led12Pulse | Set the 1st and 2nd LEDs (BATT Yellow) to glowing |
|     Led2DimSolid | Set the 2nd LED to solid with Dim |
|     Led34Off | Set the 3rd and 4th LEDs Off |
|     Led3Solid | Set the 3rd LED (INFO Blue) to Solid |
|     Led12Blink | Set the 1st and 2nd LEDs (BATT Yellow) Slow Blinking |
|     Led4Pulse | Set the 4th LED (INFO Red) to glowing |
|     Led1Pulse | Set the 1st LED (BATT Green) to glowing |
|     Led2Pulse | Set the 2nd LED (BATT Red) to glowing |
|     Led1Blink | Set the 1st LED (BATT Green) to blinking |
|     Led1Solid | Set the 1st LED (BATT Green) to ON |
|     Led2Solid | Set the 2nd LED (BATT Red) to  solid |
|     Led4Solid | Set the 4th LED (INFO Red) to solid |
|     Led34Blink | Set the 3rd and 4th LEDs (INFO Purple) Slow Blinking |
|     Led34Pulse | Set the 3rd and 4th LEDs (INFO Purple) to glowing |
|     Led34BlinkFast | Set the 3rd and 4th LEDs (INFO Purple) Blinking |
|     Led4Blink | Set the 4th LED (INFO Red)to Blinking |
|     Led12Solid | Set the 1st and 2nd LEDs (BATT Yellow) to  solid |
|     Led3DimSolid | Set the 3rd LED (INFO Blue) to a solid with dim |
|     Led3Pulse | Set the 3rd LED (INFO Blue) to glowing |
|     Led3Blink | Set the 3rd LED (INFO Blue) to Blinking |
|     Led2Blink | Set the 2nd Led (BATT Red) to blinking |
|     Led34Solid | Set the 3rd and 4th LEDs (INFO Purple) to  solid |
|     Led1DimSolid | Set the 1st LED (BATT Green) to solid with dim |


###SetLDSRotation

**Description:** Sets LDS rotation on or off. Can only be run in TestMode.

**Options:** 

| Flag | Description |
|------|-------------|
|     Off | Turns LDS rotation off. Mutually exclusive with On. <br>  |
|     On | Turns LDS rotation on. Mutually exclusive with Off. <br>  |


###SetMotor

**Description:** Sets the specified motor to run in a direction at a requested speed. (TestMode Only)

**Options:** 

| Flag | Description |
|------|-------------|
|     RWheelDisable | Disable Right Wheel motor |
|     BrushEnable | Enable Brush motor |
|     VacuumSpeed | Vacuum speed in percent (1-100). |
|     LWheelDist | Distance in millimeters to drive Left wheel. <br> 		(Pos = forward, neg = backward) (-10000<Dist<10000) |
|     SideBrushPower | Side Brush maximum power in milliwatts |
|     SideBrushOff | Disable the Side Brush |
|     SideBrush | SideBrush motor forward (Mutually exclusive with wheels and vacuum.) |
|     VacuumOff | Vacuum motor off (Mutually exclusive with VacuumOn) |
|     Speed | Speed in millimeters/second (0<=Speed<=350). (Required only for wheel movements) |
|     SideBrushOn | Enable the Side Brush |
|     BrushDisable | Disable Brush motor |
|     VacuumOn | Vacuum motor on (Mutually exclusive with VacuumOff) |
|     SideBrushDisable | Enable Side Brush Motor motor |
|     RPM | Next argument is the RPM of the motor. (0<=RPM<=10000) <br> 		Not used for wheels, but applied to all other motors specified in the command line. |
|     Brush | Brush motor forward (Mutually exclusive with wheels and vacuum.) |
|     RWheelEnable | Enable Right Wheel motor |
|     LWheelEnable | Enable Left Wheel motor |
|     RWheelDist | Distance in millimeters to drive Right wheel. <br> 		(Pos = forward, neg = backward) (-10000<Dist<10000) |
|     LWheelDisable | Disable Left Wheel motor |
|     SideBrushEnable | Enable Side Brush Motor motor |


###SetSystemMode

**Description:** Set the operation mode of the robot. (TestMode Only)

**Options:** 

| Flag | Description |
|------|-------------|
|     PowerCycle | Power cycles the entire system. (mutually exclusive of other options) |
|     Shutdown | Shut down the robot. (mutually exclusive of other options) |
|     Standby | Start standby operation. (mutually exclusive of other options) |
|     Hibernate | Start hibernate operation.(mutually exclusive of other options) |


###SetTime

**Description:** Sets the current day, hour, and minute for the scheduler clock.

**Options:** 

| Flag | Description |
|------|-------------|
|     Sec | Seconds value 0..59 (Optional, defaults to 0) |
|     Day | Day of week value Sunday=0,Monday=1,... (required) |
|     Hour | Hour value 0..23 (required) |
|     Min | Minutes value 0..59 (required) |


###SetUserSettings

**Description:** Sets user settings

**Options:** 

| Flag | Description |
|------|-------------|
|     OFF | Disable (Mutually exclusive with ON) |
|     EcoMode | Set ECO cleaning Mode(ON/OFF)  |
|     Schedule | Set scheduled cleanings (Must give a day and time) |
|     House | Schedule to Clean whole house (default) <br> (Mutually exclusive with None) |
|     WiFi | Set WiFi (ON/OFF) |
|     Hour | Hour value 0..23 (required) |
|     ButtonClick | Set Button click sound (ON/OFF)  |
|     Reset | Resets all the above user settings to factory and clears all scheduled cleanings |
|     None | Remove Scheduled Cleaning for specified day. Time is ignored. <br> (Mutually exclusive with House) |
|     FilterChange | Set time interval in seconds before sending alert to change filter |
|     Min | Minutes value 0..59 (required) |
|     BinFullDetect | Set dust bin full detect option (ON/OFF) |
|     IntenseClean | Set Intense clean State(ON/OFF)  |
|     BrushChange | Set time interval in seconds before sending alert to change brush |
|     Day | Day of the week to set schedule for. Sun=0,Sat=6. <br>  If not specified, then all days are given. |
|     Warnings | Set warning sounds (ON/OFF) |
|     Melodies | Set melody sounds(ON/OFF) |
|     DirtBin | Set time interval in minutes before sending alert to change dirt bin |
|     StealthLED | Set LEDs in standby mode (ON/OFF) |
|     ON | Enable  (Mutually exclusive with OFF) |


###SetUsage

**Description:** Sets usage settings

**Options:** 

| Flag | Description |
|------|-------------|
|     Dustbin | Set dustbin life time. |
|     MainBrush | Set mainbrush life time. |
|     SideBrush | Set sidebrush life time. |
|     Filter | Set filter life time. |


###SetWifi

**Description:** SetWifi variables

###TestMode

**Description:** Sets TestMode on or off. Some commands can only be run in TestMode.

**Options:** 

| Flag | Description |
|------|-------------|
|     Off | Turns Testmode off. Mutually exclusive with On. <br>  |
|     On | Turns Testmode on. Mutually exclusive with Off. <br>  |


###Upload

**Description:** Uploads new program to the robot.


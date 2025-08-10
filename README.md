# Transport Robot code documentation, refined with Gemini 2.5 Pro:

### **Project Overview and Purpose**

The robot's primary objective is to navigate a predefined course, identify specific objects (mission targets and command cards) using a camera and a neural network, and perform tasks based on a complex set of rules and states.

The robot operates in a mission-based environment with three distinct routes:
1.  **Supply Route:** Collect and deliver supplies.
2.  **Rescue Route:** Transport a "trapped individual."
3.  **Obstacle Route:** Clear and transport debris.

---

### **Hardware Platform and Libraries**

The code is written for a specific hardware platform, likely the **CocoPi**, which appears to be an embedded system featuring an AI-capable processor (like the Allwinner V831, hinted at by function names), camera, display, and I/O for controlling motors and servos.

**Core Libraries Used:**

*   **`CocoPi`:** A custom, proprietary library for the CocoPi board. It provides high-level abstractions for controlling DC Motors (`extDcMotor`) and reading GPIO pins (`multiFuncGpio`).
*   **`maix`:** The Sipeed MaixPy AI/ML library. This is the cornerstone of the robot's intelligence, used for:
    *   `maix.camera`: Capturing images from the camera.
    *   `maix.display`: Showing the camera feed and detection results on the built-in screen.
    *   `maix.image`: Performing image manipulation tasks like cropping, rotating, drawing shapes, and loading fonts.
    *   `maix.nn`: Loading and running the neural network model for object detection.
*   **`smbus2`:** A standard Python library for I2C communication. It is used to interface with the PCA9685 servo driver chip.
*   **Standard Libraries:** `time`, `sys`, `os`, `base64`.

---

### **Code Architecture and Structure**

The program is structured around a central state machine managed by global variables and a main execution loop.

#### **1. Global Variables (State Management)**

The `initVariables()` function initializes all the global variables that define the robot's state at any given time.

*   `currentPath` (`str`): The current mission route the robot is on ("SUPPLY", "RESCUE", "OBSTACLE"). This is the primary state variable.
*   `junctionCount` (`int`): A counter that tracks how many intersections (black cross-lines) the robot has passed on its current path. This is crucial for localization.
*   `doTravel` (`str`): A flag ("OPENED" or "CLOSED") that enables or disables the line-following logic. It's used to prevent sensor interference during complex maneuvers like turns.
*   `doWait` (`int`): A flag (0 or 1) that makes the robot pause at a specific junction, waiting for a command card to be detected.
*   `doDetect` (`int`): A flag (0 or 1) that enables or disables object detection. This is an important optimization to prevent false detections when the robot is not in a designated detection zone.
*   `containerFilled` (`str`): Tracks the payload status ("EMPTY" or "FILLED").
*   `vehicleLocation` (`str`): Stores the name of the location card currently detected by the camera (e.g., "SUPPLY_STATION").
*   `vehicleCommand` (`str`): Stores the name of the command card currently detected (e.g., "GO_RESCUE").
*   **Configuration & Calibration Values:**
    *   `movementSpeed`, `correctionSpeed`: Define the base speeds for various movements.
    *   `lineFollowThreshold`: The analog sensor value that distinguishes a black line from a white surface.
    *   `servoP0...`, `servoP1...` angles: Pre-calibrated angle values for the servo motors.
    *   `my_90deg`, `my_180deg`: Time durations (in milliseconds) required for the robot to complete 90-degree and 180-degree turns. These are calibrated time-based turns, not gyro-based.

#### **2. Motor and Movement Functions**

These functions translate high-level commands into low-level motor speed controls for the four mecanum wheels (C, D, E, F). Mecanum wheels allow for omnidirectional movement.

*   `stopMoving()`: Sets all motor speeds to 0.
*   `moveFront()`, `moveBack()`: All wheels rotate in the same direction.
*   `turnLeft()`, `turnRight()`: Wheels on opposite sides rotate in opposite directions to pivot the robot.
*   `shiftLeft()`, `shiftRight()`: Wheels on diagonal axes rotate in opposite directions to move the robot sideways without changing its orientation.
*   `correctLeft()`, `correctRight()`: Subtle turning movements for line following, using different speed multipliers for inner and outer wheels.

#### **3. Servo Control (PCA9685 Driver)**

The code includes a robust class-based driver for the PCA9685 16-channel I2C servo controller.

*   **`class PCA9685`**: A low-level driver.
    *   `__init__()`: Initializes the I2C bus and sets default frequency and pulse width ranges.
    *   `write()`, `read()`: Basic I2C communication functions with built-in retry logic for reliability.
    *   `freq()`: Sets the PWM frequency for all channels, essential for servo operation.
    *   `pwm()` / `duty()`: The core functions that set the pulse width for a specific channel to control the servo's position.
*   **`class extServo(PCA9685)`**: A high-level abstraction built on `PCA9685`.
    *   It maps a logical servo ID (0 or 1 for P0, P1) to a physical pin on the PCA9685 board.
    *   `position(degrees)`: Translates a desired angle (in degrees) into the corresponding PWM duty cycle value and sends it to the servo.

#### **4. Computer Vision and AI**

This is the most complex part of the system, responsible for interpreting the visual world.

*   **`class Yolo`**: Manages the object detection model.
    *   **`labels`**: A list of all possible object names the model can recognize.
    *   **`m`**: A dictionary containing the file paths to the pre-trained neural network model (`.param` and `.bin` files).
    *   **`options`**: A dictionary that configures the neural network for the `maix.nn` library, specifying input dimensions, output shape, and normalization values.
    *   **`__init__()`**: This is where the magic happens. `nn.load()` loads the model into memory. A `decoder.Yolo2` object is created to translate the raw numerical output of the neural network into human-readable bounding boxes and class probabilities.
*   **`doDetection()`**: The primary function for vision processing.
    1.  **Capture & Preprocess:** It captures an image from the camera, rotates it to match the screen orientation, and crops it to a 224x224 square, which is the required input size for the YOLO model.
    2.  **Inference:** `Yolo.model.forward()` runs the image through the neural network. This is the "inference" step.
    3.  **Decode:** `Yolo.decoder.run()` takes the raw output and finds potential objects.
    4.  **Process Detections:**
        *   If objects are found and the `doDetect` flag is active, it iterates through each detected object (`box`).
        *   It draws the bounding box and the object's label on the image for debugging on the display.
        *   **Crucially, it updates the `vehicleLocation` or `vehicleCommand` global variables based on the detected label and sometimes its position (e.g., `(i[1]) < 200` ensures the object is in the upper part of the view).**
    5.  **High-Level Logic:** After updating the state, it contains a block of `if/elif` statements that make major strategic decisions, like changing the `currentPath` based on the detected command card.
    6.  **Display:** It calls `v831_display_show_imageDetection` to show the processed image on the screen.

#### **5. Mission Logic Functions**

These functions contain the step-by-step instructions for each of the three main routes. They are called from the main loop when a junction is detected.

*   **`doSupply()`**: Implements the logic for the supply run. It uses `junctionCount` to determine actions like turning left, turning right, stopping to collect supplies (a complex sequence of movements and servo actions), and enabling detection (`doDetect = 1`) when approaching the drop-off zone.
*   **`doRescue()`**: Logic for the rescue mission. It involves turning, pausing at a junction (`doWait = 1`) to wait for the "trapped individual," and enabling detection.
*   **`doObstacle()`**: Logic for the obstacle clearing mission, similar in structure to the other two.

---

### **Main Execution Loop (`while True:`)**

This is the heart of the program, running continuously.

1.  **Vision First:** The loop always starts by calling `doDetection()`. This prioritizes reacting to visual cues (command cards, location markers) over line following.
2.  **Junction Check:** It reads the two grayscale sensors. If both values are below the `lineFollowThreshold`, it signifies a black cross-line (a junction).
    *   It increments `junctionCount`.
    *   It calls the function corresponding to the `currentPath` (`doSupply`, `doRescue`, or `doObstacle`), which then executes the maneuver for that specific junction.
3.  **Line Detection & State Reset:** If *at least one* sensor detects the line and the robot is not in a `doWait` state, it sets `doTravel` to "OPENED". This ensures that after a turn or maneuver, the robot re-engages its line-following behavior.
4.  **Line Following:** If `doTravel` is "OPENED":
    *   If the right sensor is on black and the left is on white, it calls `correctRight()`.
    *   If the left sensor is on black and the right is on white, it calls `correctLeft()`.
    *   If both are on white, it calls `moveFront()`.

### **Core Concept: State Change vs. Immediate Action**

It's important to understand that detecting these cards primarily changes the value of two key global variables:
*   `vehicleCommand`: Stores the next intended mission (set by A, B, C).
*   `vehicleLocation`: Stores the robot's current location, but is uniquely set to "RETURN" by card D to trigger a specific action.

The robot's actual physical response is a result of a combination of these variables, along with others like `currentPath` and `containerFilled`.

---

### **Detection of Card "D" (The "Return / Continue" Command)**

Card "D" is the most direct action trigger. Its purpose is to signal that a waiting period is over and the robot can proceed with its return journey.

**Direct Code Actions:**
When the camera detects the label "D":
1.  `time.sleep(2000 / 1000)`: The code pauses for 2 seconds. This is a debouncing measure to ensure the card is read correctly and not just a fleeting glance.
2.  `vehicleLocation = "RETURN"`: The robot's location state is set to "RETURN". This is a unique state that doesn't correspond to a physical place but rather a command to resume movement.
3.  `doDetect = 0`: Object detection is immediately turned **off**. This is a critical step to prevent the robot from repeatedly detecting the "D" card and getting stuck in a logic loop.

**Resulting Robot Behavior:**
The main logic block in `doDetection()` checks for this state change: `elif vehicleLocation == "RETURN":`
*   `doWait = 0`: This is the most important consequence. The robot is often paused at a junction (`doWait = 1`) while waiting for another robot to load its cargo. Setting `doWait` to 0 "un-pauses" the robot. The main `while True:` loop's line-following condition will now be met, and the robot will start moving again.
*   `containerFilled = "FILLED"`: If the robot is on the "RESCUE" or "OBSTACLE" path, its container status is updated to "FILLED". This logically signifies that the payload (the rescued person or the obstacle) has been successfully loaded during the wait.
*   The robot then begins its return journey by executing the appropriate movement function based on its path (e.g., `moveFront()`).

**In Simple Terms:** When the robot sees **Card D**, it understands that its partner robot has finished loading the cargo. It waits two seconds, acknowledges the command, stops looking for more cards, and resumes its journey with its cargo now considered "FILLED".

---

### **Detection of Cards "A", "B", and "C" (The "Next Path" Commands)**

These three cards function as instructions for the robot's *next* mission. They set a "command" state that is only acted upon when the robot is at a specific delivery location.

#### **Card "A"**

*   **Direct Code Action:** `vehicleCommand = "GO_RESCUE"`
*   **Resulting Robot Behavior:** This command tells the robot that its **next path should be the "RESCUE" route**. This instruction is stored and only used when the robot is at a drop-off point.
    *   **Example Scenario:** The robot is on the "SUPPLY" path and has a "FILLED" container. It arrives and detects the "Supply\_Station" location card. At the same time, it sees **Card "A"**. The code checks the condition: `if vehicleLocation == "SUPPLY_STATION" and currentPath == "SUPPLY" and containerFilled == "FILLED" and vehicleCommand == "GO_RESCUE":`
    *   Because the condition is met, the robot will first execute `doUnload()` to deliver its supplies, and then immediately change its master state: `currentPath = "RESCUE"`. From this point forward, it will use the logic in the `doRescue()` function to navigate.

#### **Card "B"**

*   **Direct Code Action:** `vehicleCommand = "GO_CLEAR"`
*   **Resulting Robot Behavior:** This command tells the robot that its **next path should be the "OBSTACLE" route**.
    *   **Example Scenario:** The robot is at the "Supply\_Station" to deliver supplies. It sees **Card "B"**.
    *   The code will match the condition for `vehicleCommand == "GO_CLEAR"`. The robot will perform `doUnload()` and then set `currentPath = "OBSTACLE"`. Its next journey will be to the obstacle zone.

#### **Card "C"**

*   **Direct Code Action:** `vehicleCommand = "GO_COLLECT"`
*   **Resulting Robot Behavior:** This command tells the robot that its **next path should be the "SUPPLY" route**. This is typically used when the robot is on a different mission (like "RESCUE") and needs to be told to go back to collecting supplies next.
    *   **Example Scenario:** The robot is on the "RESCUE" path and has just dropped off the rescued person at the "Rescue\_Station". It sees **Card "C"**.
    *   The code will match the condition `vehicleCommand == "GO_COLLECT"`. It will perform `doUnload()` and then set `currentPath = "SUPPLY"`.

### **Summary Table**

| Card Detected | Direct State Change | Condition for Action | Robot's Ultimate Action |
| :-----------: | :------------------------------------------------------------------- | :-------------------------------------------------------------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **A** | `vehicleCommand = "GO_RESCUE"` | Robot must be at a valid drop-off location (e.g., "Supply\_Station"). | After unloading its current cargo, the robot will switch its mission path to **"RESCUE"** for its next journey. |
| **B** | `vehicleCommand = "GO_CLEAR"` | Robot must be at a valid drop-off location. | After unloading, the robot will switch its mission path to **"OBSTACLE"** for its next journey. |
| **C** | `vehicleCommand = "GO_COLLECT"` | Robot must be at a valid drop-off location. | After unloading, the robot will switch its mission path to **"SUPPLY"** for its next journey. |
| **D** | `vehicleLocation = "RETURN"`, `doDetect = 0` | The robot is currently paused at a junction (`doWait = 1`). | The robot **resumes movement** from its paused state, now considering its cargo container to be "FILLED", and begins its return trip. It also stops looking for any more cards. |

### **Summary of Operation**

1.  **Initialization:** The robot starts, initializes all variables, sets servos to their home positions, and enters the main loop. `currentPath` is "SUPPLY" by default.
2.  **Navigation:** It follows a black line on a white surface using two grayscale sensors. `correctLeft` and `correctRight` keep it centered.
3.  **Junction Handling:** When it crosses an intersection, `junctionCount` increases. It then executes a pre-programmed sequence of actions based on its `currentPath` and the new `junctionCount`. This might be a simple turn or a complex task like simulating supply collection.
4.  **Object Detection:** In specific parts of the course (controlled by the `doDetect` flag), the camera becomes active. The YOLO model identifies command and location cards.
5.  **Decision Making:** A detected card updates the `vehicleLocation` and `vehicleCommand` variables. This can trigger a major change in strategy, such as performing an unloading sequence (`doUnload`) and then changing the `currentPath` from "SUPPLY" to "RESCUE".
6.  **Loop:** This process repeats, allowing the robot to navigate the entire course, react to visual information, and complete its multi-stage mission.

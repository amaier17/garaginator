// This #include statement was automatically added by the Particle IDE.
#include <MQTT_HASS.h>
#include <HallEffectSI7210.h>

// -----------------------------------------
// Function and Variable with Photoresistors
// -----------------------------------------
// In this example, we're going to register a Particle.variable() with the cloud so that we can read brightness levels from the photoresistor.
// We'll also register a Particle.function so that we can turn the LED on and off remotely.

#include <Wire.h>

// -----------------------------------------
// Pins
// -----------------------------------------
int led = D7; // This is where your LED is plugged in. The other side goes to a resistor connected to GND.
int value = 0;
int DEV = 0x33;
int BUTTON_DELAY = 200;

int WALL_SENSE = A0;
int LED_EY = A3;
int LED_G = A4;
int LED_Y = A5;
int CTRL_WALLDRIVE = D7;
int CTRL_COIL0 = D2;
int CTRL_COIL1 = D3;
int CTRL_COIL2 = D4;
int CTRL_COIL3 = D5;
int CTRL_COIL4 = D6;
// -----------------------------------------

// MQTT
byte mqtt_server[] = {192, 168, 0, 3};
MQTT_HASS& client = MQTT_HASS::getInstance(mqtt_server, 1883);
String mqtt_username = "<replace this username>";
String mqtt_password = "<replace this password>";

Device dev {
    .name = "garaginator",
    .model = "Andrew's Argon",
};

void coverDoorCallback(char *topic, uint8_t *payload, unsigned int length);
void lockOpenerCallback(char *topic, uint8_t *payload, unsigned int length);
void buttonToggleLightCallback(char *topic, uint8_t *payload, unsigned int length);
void buttonCalibrateCallback(char *topic, uint8_t *payload, unsigned int length);

BinarySensor binarySensorTamper("tamper", "Tamper Sensor", client, dev, BinarySensor::DeviceClasses::tamper);
Sensor sensorHeThreshold("heThreshold", "Hall Effect Threshold", client, dev, Sensor::DeviceClasses::None, "", Sensor::EntityCategories::diagnostic);
Sensor sensorHeValue("heValue", "Hall Effect Value", client, dev, Sensor::DeviceClasses::None, "", Sensor::EntityCategories::diagnostic);
Sensor sensorWallState("wallState", "Wall Button State", client, dev, Sensor::DeviceClasses::None, "", Sensor::EntityCategories::diagnostic);
Cover garageDoor("door", "Garage Door", client, dev, coverDoorCallback, Cover::DeviceClasses::garage);
Lock lockOpener("lockOpener", "Lock Opener", client, dev, lockOpenerCallback);
Button buttonToggleLight("buttonToggleLight", "Toggle Light", client, dev, buttonToggleLightCallback);
Button buttonCalibrate("buttonCalibrate", "Calibrate", client, dev, buttonCalibrateCallback);

void joint_print(String message) {
    Serial.println(message);
    Particle.publish(message);
}

enum EEPROMLayout : int {
    EEPROMHallEffect,
};

enum LockState: int {
    LockState_Unknown = -1,
    LockState_Unlocked,
    LockState_Locked,
};

LockState lockState = LockState_Unknown;

int he_threshold = 5000;
int he_value = 0;
int wall_state = 0;
HallEffectSI7210 hallEffect(HallEffectSI7210::Devs::SI7210_B_04_IV);

void enableBoard() {
    digitalWrite(CTRL_COIL0, HIGH);
    delay(100);
}

void disableBoard() {
    digitalWrite(CTRL_COIL0, LOW);
    delay(100);
}

enum ReqState : int32_t {
    Unlocked,
    Lock,
    ToggleLight,
    Activate,
    Invalid,
};

ReqState curState = Invalid;

bool lockDoor(String extra = "") {
    binarySensorTamper.updateState(BinarySensor::States::OFF); // Clear tamper if door is locked
    if (lockState != LockState_Locked) {
        lockOpener.updateState(Lock::States::LOCKING);
        delay(1);
        lockOpener.updateState(Lock::States::LOCKED);
        lockState = LockState_Locked;
    }
    enableBoard();
    digitalWrite(CTRL_COIL2, LOW);
    digitalWrite(CTRL_COIL3, HIGH);
    digitalWrite(CTRL_COIL4, LOW);
    delay(100);
    return true;
}

bool unlockDoor(String extra = "") {
    binarySensorTamper.updateState(BinarySensor::States::OFF); // Clear tamper if door is unlocked
    // Check if the wall still says locked (if so we can't unlock)
    if (curState == Lock)
        return true;

    if (lockState != LockState_Unlocked) {
        lockOpener.updateState(Lock::States::UNLOCKING);
        delay(1);
        lockOpener.updateState(Lock::States::UNLOCKED);
        lockState = LockState_Unlocked;
    }
    enableBoard();
    digitalWrite(CTRL_COIL2, LOW);
    digitalWrite(CTRL_COIL3, LOW);
    digitalWrite(CTRL_COIL4, LOW);
    delay(100);
    return true;
}

int toggleOverheadLight(String extra = "") {
    binarySensorTamper.updateState(BinarySensor::States::OFF); // Clear tamper if light is toggled
    enableBoard();
    digitalWrite(CTRL_COIL2, LOW);
    digitalWrite(CTRL_COIL3, LOW);
    digitalWrite(CTRL_COIL4, HIGH);
    delay(BUTTON_DELAY);
    digitalWrite(CTRL_COIL4, LOW);
    return 0;
}

enum DoorState: int {
    DoorState_Unknown = -1,
    DoorState_Closed,
    DoorState_Open,
};

DoorState doorState = DoorState_Unknown;
String doorStateStr = "Unknown";

DoorState getDoorState() {
    int value;
    if (!hallEffect.measure(value))
        return DoorState_Unknown;
    
    value = abs(value);
    he_value = value;
    if (value <= he_threshold)
        return DoorState_Open;
    else
        return DoorState_Closed;
}

void activateDoor() {
    enableBoard();
    digitalWrite(CTRL_COIL2, HIGH);
    delay(BUTTON_DELAY);
    digitalWrite(CTRL_COIL2, LOW);
}

DoorState closeDoor(String extra = "") {
    int numRetries = 10;
    if (doorState == DoorState_Closed)
        return DoorState_Closed;

    garageDoor.updateState(Cover::States::CLOSING);
    lockDoor();
    client.publishAvailabilities();
    while (numRetries-- && getDoorState() != DoorState_Closed) {
        unsigned long endTime = millis() + 20000;
        activateDoor();
        while(getDoorState() != DoorState_Closed && endTime > millis()) {
            yield();
            Particle.process();
            delay(10);
        }
    }

    if (curState != Lock)
        unlockDoor();
    
    doorState = getDoorState();
    if (doorState == DoorState_Closed) {
        garageDoor.updateState(Cover::States::CLOSED);
        binarySensorTamper.updateState(BinarySensor::States::OFF); // Clear tamper if door is closed from our requests
    } else {
        garageDoor.updateState(Cover::States::STOPPED);
        binarySensorTamper.updateState(BinarySensor::States::ON);
    }
    
    return doorState;
}

DoorState openDoor(String extra = "") {
    if (curState == Lock || lockState == LockState_Locked || doorState == DoorState_Open)
        return doorState;

    activateDoor();
    garageDoor.updateState(Cover::States::OPENING);
    client.publishAvailabilities();
    unsigned long endTime = millis() + 20000;
    while (endTime > millis()) {
        yield();
        Particle.process();
        delay(10);
    }
    doorState = getDoorState();
    if (doorState == DoorState_Open) {
        garageDoor.updateState(Cover::States::OPEN);
    } else {
        garageDoor.updateState(Cover::States::STOPPED);
        binarySensorTamper.updateState(BinarySensor::States::ON);
    }

    return doorState;
}

bool calibrateHallEffect(String extra = "") {
    int closeValue, openValue;
    joint_print("Running calibrate");
    lockDoor();
    garageDoor.updateState(Cover::States::CLOSING);
    activateDoor();
    delay(20000);
    if (!hallEffect.measure(closeValue))
        goto failed;
    closeValue = abs(closeValue);
    doorState = getDoorState();
    garageDoor.updateState(Cover::States::CLOSED);

    unlockDoor();
    garageDoor.updateState(Cover::States::OPENING);
    activateDoor();
    delay(20000);
    if (!hallEffect.measure(openValue))
        goto failed;
    openValue = abs(openValue);
    doorState = getDoorState();
    garageDoor.updateState(Cover::States::OPEN);

    if (openValue > closeValue || (closeValue - openValue) < 500)
        goto failed;
    
    he_threshold = (closeValue - openValue) / 2;
    EEPROM.put(EEPROMHallEffect, he_threshold);
    joint_print("he_threshold" + String::format("%d", he_threshold));
    closeDoor(extra);
    return true;

failed:
    binarySensorTamper.updateState(BinarySensor::States::ON);
    lockDoor();
    activateDoor();
    return false;
}

void coverDoorCallback(char *topic, uint8_t *payload, unsigned int length) {
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = NULL;
    String message(p);

    if (message.equalsIgnoreCase("OPEN"))
        openDoor();
    else if (message.equalsIgnoreCase("CLOSE"))
        closeDoor();
    else if (message.equalsIgnoreCase("STOP"))
        joint_print("STOP Not Supported");
    else
        joint_print("main: " + message);
}

void buttonCalibrateCallback(char *topic, uint8_t *payload, unsigned int length) {
    calibrateHallEffect();
}

void buttonToggleLightCallback(char *topic, uint8_t *payload, unsigned int length) {
    toggleOverheadLight();
}

void lockOpenerCallback(char *topic, uint8_t *payload, unsigned int length) {
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = NULL;
    String message(p);

    if (message.equalsIgnoreCase("LOCK"))
        lockDoor();
    else if (message.equalsIgnoreCase("UNLOCK"))
        unlockDoor();
    else
        joint_print("lock: " + message);
}

void update_sensors() {
    String wall, thresh, val;
    thresh = String::format("%d", he_threshold);
    val = String::format("%d", he_value);
    if (curState == Unlocked)
        wall = "Unlocked";
    else if (curState == Lock)
        wall = "Lock";
    else if (curState == ToggleLight)
        wall = "Toggle Light";
    else if (curState == Activate)
        wall = "Activate";
    else
        wall = "Invalid";
    
    sensorHeThreshold.updateState(thresh);
    sensorHeValue.updateState(val);
    sensorWallState.updateState(wall);
}

ReqState getWallState() {
    const int DEBOUNCE_THRESHOLD = 200;
    const int WALL_ACTIVATE = 290;
    const int WALL_LOCK = 1286;
    const int WALL_TOGGLELIGHT = 1800;
    const int WALL_UNLOCKED = 2100;
    int val1 = analogRead(WALL_SENSE);
    delay(10);
    int val2 = analogRead(WALL_SENSE);
    if (abs(val2 - val1) > DEBOUNCE_THRESHOLD)
        return Invalid;

    if (val2 < (WALL_ACTIVATE + WALL_LOCK) / 2)
        return Activate;
    else if (val2 < (WALL_LOCK + WALL_TOGGLELIGHT) / 2)
        return Lock;
    else if (val2 < (WALL_TOGGLELIGHT + WALL_UNLOCKED) / 2)
        return ToggleLight;
    else
        return Unlocked;
}

void init_door_state() {
    doorState = getDoorState();
    if (doorState == DoorState_Open)
        garageDoor.updateState(Cover::States::OPEN);
    else if (doorState == DoorState_Closed)
        garageDoor.updateState(Cover::States::CLOSED);
    else
        garageDoor.updateState(Cover::States::STOPPED);
    
    binarySensorTamper.updateState(BinarySensor::States::OFF); // Initially clear to no tamper
}

bool mqtt_start(int retries = 5) {
    client.connect(mqtt_username, mqtt_password);
    
    int i = 0;
    while (!client.isConnected() && i < retries) {
        delay(1000);
        client.connect(mqtt_username, mqtt_password);
        i++;
    }
    
    if (client.isConnected()) {
        client.registerEntity(&binarySensorTamper);
        client.registerEntity(&sensorHeThreshold);
        client.registerEntity(&sensorHeValue);
        client.registerEntity(&sensorWallState);
        client.registerEntity(&garageDoor);
        client.registerEntity(&lockOpener);
        client.registerEntity(&buttonToggleLight);
        client.registerEntity(&buttonCalibrate);
        client.publishAvailabilities();
        delay(100);
        update_sensors();
        init_door_state();
    }

    return client.isConnected();
}

void setup() {
    waitUntil(Particle.connected);
    pinMode(LED_EY, OUTPUT);
    pinMode(LED_G , OUTPUT);
    pinMode(LED_Y, OUTPUT);
    pinMode(CTRL_COIL0, OUTPUT);
    pinMode(CTRL_COIL1, OUTPUT);
    pinMode(CTRL_COIL2, OUTPUT);
    pinMode(CTRL_COIL3, OUTPUT);
    pinMode(CTRL_COIL4, OUTPUT);
    pinMode(CTRL_WALLDRIVE, OUTPUT);
    digitalWrite(CTRL_COIL0, HIGH);
    digitalWrite(CTRL_COIL1, HIGH);
    digitalWrite(CTRL_WALLDRIVE, HIGH);
    digitalWrite(LED_G, HIGH);
    EEPROM.get(EEPROMHallEffect, he_threshold);
    if (he_threshold == 0 || he_threshold == -1) {
        he_threshold = 5000;
        EEPROM.put(EEPROMHallEffect, he_threshold);
    }
    Particle.function("calibrate", calibrateHallEffect);
    Particle.function("closeDoor", closeDoor);
    Particle.function("toggleLight", toggleOverheadLight);
    Particle.function("openDoor", openDoor);
    Particle.function("lockDoor", lockDoor);
    Particle.function("unlockDoor", unlockDoor);
    Particle.variable("he_threshold", he_threshold);
    Particle.variable("he_value", he_value);
    Particle.variable("wall_state", wall_state);
    
    // MQTT
    bool connected = mqtt_start();
    if (connected)
        joint_print("MQTT Connected Successfully");
}

bool ignore_next_tamper = false;

void loop() {
    delay(100);
    ReqState state = getWallState();
    if (state != Invalid && state != curState) {
        ignore_next_tamper = true;
        binarySensorTamper.updateState(BinarySensor::States::OFF);
        switch (state) {
            case Unlocked:
                unlockDoor();
                break;
            case Lock:
                lockDoor();
                break;
            case ToggleLight:
                if (curState != ToggleLight)
                    toggleOverheadLight("");
                break;
            case Activate:
                if (curState != Activate)
                    activateDoor();
                break;
            default:
                unlockDoor();
                break;
        }
        curState = state;
        wall_state = curState;
    }
    
    digitalWrite(LED_Y, hallEffect.isEnabled());
    digitalWrite(LED_EY, hallEffect.isEnabled());
    
    
    static int i = 0;
    if (i % 200 == 0) {
        client.publishAvailabilities();
        update_sensors();
        
        DoorState temp = getDoorState();
        if (temp != doorState) {
            if (ignore_next_tamper) {
                ignore_next_tamper = false;
            } else {
                binarySensorTamper.updateState(BinarySensor::States::ON);
            }
        }
        
        doorState = temp;
        if (doorState == DoorState_Open)
            garageDoor.updateState(Cover::States::OPEN);
        else if (doorState == DoorState_Closed)
            garageDoor.updateState(Cover::States::CLOSED);
        
        i = 0;
    }
    i++;
    
    if (!client.isConnected())
        mqtt_start();
        
    client.loop();
}

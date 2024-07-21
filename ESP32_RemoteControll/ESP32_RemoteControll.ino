#include <Bluepad32.h>
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
#define TXD_PIN (GPIO_NUM_17) 
#define RXD_PIN (GPIO_NUM_16) 
#define DEBOUNCE_DELAY 200
unsigned long lastDebounceTime = 0;
int Vx;
int Vy;
int Omega;
int vx;
int vy;
int omega;
int Speed;
int Mode;
int TakeRice;
int DropRice;
int Position;
int Shooter;
int StepperM;
int PushRice;
int Pick;
int Put;
// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
long int map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void decimalToHex(int decimalValue, char* hexString, size_t hexStringSize) {
  snprintf(hexString, hexStringSize, "%04X", decimalValue);
}
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}


void Transmit(ControllerPtr ctl) {
  // Number to TxData
    byte TxData[11] = {0};
   
    Vx = map(-1*ctl->axisY(),-511,512,-100,100);
    Vy = map(-1*ctl->axisX(),-511,512,-100,100);
    Omega = map(-1*ctl->axisRX(),-511,512,-100,100);
    if (Vx >= 50)
    {
      vx = 2;
    }
    else if (Vx <= -50)
    {
      vx = 1;
    }
    else 
    {
      vx = 0;
    }
    if (Vy >= 50)
    {
      vy = 2;
    }
    else if (Vy <= -50)
    {
      vy = 1;
    }
    else 
    {
      vy = 0;
    }
    if (Omega >= 50)
    {
      omega = 2;
    }
    else if (Omega <= -50)
    {
      omega = 1;
    }
    else 
    {
      omega = 0;
    }
    
    
    if (ctl->buttons() == 0x0001 && (millis()- lastDebounceTime > DEBOUNCE_DELAY))
    {
      Position++;
       if (Position>11)
      {
        Position=0;
      }
      lastDebounceTime = millis();
     
    }
    if (ctl->buttons() == 0x0010 && (millis()- lastDebounceTime > DEBOUNCE_DELAY))
    {
      Speed++;
      if (Speed>5)
      {
        Speed = 0;
      }
       
      lastDebounceTime = millis();
      
    }
    if (ctl-> buttons() == 0x0020 && (millis()- lastDebounceTime > DEBOUNCE_DELAY))
    {
      Mode++;
      if (Mode>1)
      {
        Mode = 0;
      }
      
      lastDebounceTime = millis();
      
    }
    
    if (ctl-> buttons() == 0x0004)
    {

        DropRice = 1;
    }
    else 
    {
      DropRice = 0;
    }
    if (ctl-> buttons() == 0x0002 && (millis()- lastDebounceTime > 200))
    {
      Shooter++;
      if (Shooter>6)
      {
        Shooter = 0;
      }
      
      lastDebounceTime = millis();
    }
    if (ctl-> buttons() == 0x0008)
    {
        PushRice = 1;
    }
    else if (ctl->dpad() == 0x01 )
    {
      PushRice= 2;
      
    }
    else if (ctl->dpad() == 0x02 )
    {
      PushRice = 3;
      
    }
    else 
    {
      PushRice = 0;
    }
    if (ctl->dpad() == 0x04 )
    {
      StepperM= 1;
      
    }
    else if (ctl->dpad() == 0x08 )
    {
      StepperM = 2;
      
    }
    else
    {
      
      StepperM = 0;
 
    }
    if (ctl->throttle() == 1020)
    {
      Pick = 1;
    }
    else
    {
      Pick = 0;
    }
//    if (ctl->brake() == 1020)
//    {
//      Put = 1;
//    }
//    else
//    {
//      Put = 0;
//    }
    TxData[0] = byte(vx);
    TxData[1] = byte(vy);
    TxData[2] = byte(omega);
    TxData[3] = byte(Speed);
    TxData[4] = byte(Mode);
    TxData[5] = byte(Position);
    TxData[6] = byte(PushRice);
    TxData[7] = byte(Shooter);
    TxData[8] = byte(StepperM);
    TxData[9] = byte(DropRice);
    TxData[10] = byte(Pick);
//    TxData[11] = byte(Put);
    Serial2.write(TxData, 11);
    Serial.printf("Vx=%d, Vy=%d, Omega=%d, Speed=%d, Mode=%d, Position=%d, TakeRice=%d, DropRice=%d,Shooter=%d,PushRice=%d,StepperM=%d,Pick=%d,Push=%d\n",
    vx,vy,omega,Speed,Mode,Position,TakeRice,DropRice,Shooter,PushRice,StepperM,Pick,Put);
}



void processGamepad(ControllerPtr ctl) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...
    Transmit(ctl);
    if (ctl->a()) {
        static int colorIdx = 0;
        // Some gamepads like DS4 and DualSense support changing the color LED.
        // It is possible to change it by calling:
        switch (colorIdx % 6) {
            case 0:
                // Red
                ctl->setColorLED(255, 0, 0);
                break;
            case 1:
                // Green
                ctl->setColorLED(0, 255, 0);
                break;
            case 2:
                // Blue
                ctl->setColorLED(0, 0, 255);
                break;
            case 3:
                // Yellow
                ctl->setColorLED(255,255,0);
                break;
            case 4:
                // Orange
                ctl->setColorLED(255,165,0);
                break;
            case 5:
                // Purple
                ctl->setColorLED(143,0,255);
                break;
        }
        colorIdx++;
    }

    if (ctl->b()) {
        // Turn on the 4 LED. Each bit represents one LED.
        static int led = 0;
        led++;
        // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
        // support changing the "Player LEDs": those 4 LEDs that usually indicate
        // the "gamepad seat".
        // It is possible to change them by calling:
        ctl->setPlayerLEDs(led & 0x0f);
    }

    if (ctl->x()) {
        // Duration: 255 is ~2 seconds
        // force: intensity
        // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S support
        // rumble.
        // It is possible to set it by calling:
        ctl->setRumble(0xc0 /* force */, 0xc0 /* duration */);
    }

    // Another way to query controller data is by getting the buttons() function.
    // See how the different "dump*" functions dump the Controller info.
//    dumpGamepad(ctl);
}



// Arduino setup function. Runs in CPU 1
void setup() {
  
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RXD_PIN, TXD_PIN); // Use UART2
    
    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "vritual device", is a mouse
    // By default it is disabled.
    BP32.enableVirtualDevice(false);
}

// Arduino loop function. Runs in CPU 1
void loop() {
    // This call fetches all the gamepad info from the NINA (ESP32) module.
    // Just call this function in your main loop.
    // The gamepads pointer (the ones received in the callbacks) gets updated
    // automatically.
    BP32.update();
    
//    Serial2.write(values, 12);
    // It is safe to always do this before using the gamepad API.
    // This guarantees that the gamepad is valid and connected.
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        ControllerPtr myController = myControllers[i];

        if (myController && myController->isConnected()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
//                Transmit(myController);
            } 
             else {
                Serial.printf("Data not available yet\n");
                continue;
            }
            // See ArduinoController.h for all the available functions.
        }
    }
    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    // vTaskDelay(1);
    delay(10);
}

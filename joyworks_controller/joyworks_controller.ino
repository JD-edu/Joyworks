#include <U8x8lib.h>
#include <ESP32Servo.h>
#include "BluetoothSerial.h"
#include "ESP32MotorControl.h"
#include <SPI.h>
#include "LedMatrix.h"

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

#define SCL 22
#define SDA 21

#define SOUND_SPEED 0.034

U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // OLEDs without Reset of the Display

// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
ESP32MotorControl MotorControl = ESP32MotorControl();
BluetoothSerial SerialBT;

int MOTOR_GPIO_IN1 = 25; 
int MOTOR_GPIO_IN2 = 26;
int MOTOR_GPIO_IN3 = 5; 
int MOTOR_GPIO_IN4 = 18;  

int echoPin = 2;
int trigPin = 16;

int IR1 = 13;
int IR2 = 14;

#define SERVO1  4

Servo myservo1;

// Reading ultrasonic sensor 
float readUltraSensor(){
    float distance = 0;
    long duration = 0;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = duration * SOUND_SPEED/2;
    Serial.println(distance);
    return distance;
}

void readIR(){
    int ir1 = digitalRead(IR1);
    int ir2 = digitalRead(IR2);
    String IR1_STR  = String(ir1);
    String IR2_STR  = String(ir2);
    String result = IR1_STR+" "+IR2_STR;
    Serial.println(result);
}

int readIR1(){
    int ir1 = digitalRead(IR1);
    return ir1;
}

int readIR2(){
    int ir2 = digitalRead(IR2);
    return ir2;
}

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
    if(event == ESP_SPP_SRV_OPEN_EVT){
        Serial.println("Client Connected");
    }else if(event == ESP_SPP_CLOSE_EVT){
        Serial.println("Client closed");
    }
}

void setup() {

    // sets the pins as outputs:
    pinMode(MOTOR_GPIO_IN1, OUTPUT);
    pinMode(MOTOR_GPIO_IN2, OUTPUT);
    pinMode(MOTOR_GPIO_IN3, OUTPUT);
    pinMode(MOTOR_GPIO_IN4, OUTPUT);

    pinMode(trigPin, OUTPUT);

    pinMode(IR1, INPUT);
    pinMode(IR2, INPUT);

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    Serial.begin(115200);
    Serial.println("Starting Joywork ... ");

    SerialBT.register_callback(callback);
    SerialBT.begin("Polycar"); //Bluetooth device name

    MotorControl.attachMotors(MOTOR_GPIO_IN1, MOTOR_GPIO_IN2, MOTOR_GPIO_IN3, MOTOR_GPIO_IN4);    
    MotorControl.motorsStop();

    myservo1.setPeriodHertz(50);    // standard 50 hz servo
    myservo1.attach(SERVO1, 500, 2400);

    myservo1.write(90);

    u8x8.begin();
    u8x8.setPowerSave(0);
    u8x8.setFont(u8x8_font_chroma48medium8_r);
}

void loop() {

    char inCommand;
    
    String inString = "";
    if(SerialBT.available() >0){
        if(SerialBT.available()){
            inString = SerialBT.readStringUntil('\n');
            inCommand = inString[0];
        }    
    }
    inString = "";
    if(Serial.available() >0){
        if(Serial.available()){
            inString = Serial.readStringUntil('\n');
            SerialBT.println(inString);
            const char * cmd = inString.c_str();
            inCommand = inString[0];
        }    
    }

    if(inCommand == 'a'){
        Serial.println("Motor forward ");
        MotorControl.motorForward(0,50);
        MotorControl.motorForward(1,50);
    }else if(inCommand == 'b'){
        Serial.println("Motor backward ");
        MotorControl.motorReverse(0,50);
        MotorControl.motorReverse(1,50);
    }else if(inCommand == 'c'){
        Serial.println("Motor stop ");
        MotorControl.motorsStop();
    }else if(inCommand == 'd'){
        myservo1.write(0);
    }else if(inCommand == 'e'){
        myservo1.write(180);
    }
    //if(readUltraSensor() < 10){
    //  MotorControl.motorsStop();
    //  delay(1000);
    //  MotorControl.motorReverse(0,50);
    //  MotorControl.motorReverse(1,50);
    //}
    //readIR();
    if(readIR1() == 1){
        MotorControl.motorForward(0,50);
        MotorControl.motorForward(1,0);
        delay(300);
    }else{
        MotorControl.motorForward(0,50);
        MotorControl.motorForward(1,50);
    }
    if(readIR2() == 1){
        MotorControl.motorForward(0,0);
        MotorControl.motorForward(1,50);
        delay(300);
    }else{
      MotorControl.motorForward(0,50);
        MotorControl.motorForward(1,50);
    }
    u8x8.drawString(0,0,"PolyCar");
}

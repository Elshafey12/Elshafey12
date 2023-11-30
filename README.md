
#include <stdio.h>
#include <unistd.h>
#include <wiringPi.h> // Assuming you're using WiringPi for GPIO control
#include <softPwm.h>  // Soft PWM for motor control

#define TRIG_PIN 1     // GPIO pin for Ultrasonic sensor trigger
#define ECHO_PIN 4     // GPIO pin for Ultrasonic sensor echo
#define SERVO_PIN 5    // GPIO pin for Servo motor
#define MOTOR1_PWM 6   // GPIO pin for Motor 1 PWM control
#define MOTOR1_IN1 0   // GPIO pin for Motor 1 direction control
#define MOTOR1_IN2 2   // GPIO pin for Motor 1 direction control
#define MOTOR2_PWM 3   // GPIO pin for Motor 2 PWM control
#define MOTOR2_IN1 21  // GPIO pin for Motor 2 direction control
#define MOTOR2_IN2 22  // GPIO pin for Motor 2 direction control
#define LCD_RS 26      // GPIO pin for LCD RS
#define LCD_E 27       // GPIO pin for LCD E
#define LCD_D4 28      // GPIO pin for LCD D4
#define LCD_D5 29      // GPIO pin for LCD D5
#define LCD_D6 25      // GPIO pin for LCD D6
#define LCD_D7 24      // GPIO pin for LCD D7

void setup() {
    wiringPiSetup();
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(SERVO_PIN, OUTPUT);

    // Motor control setup
    softPwmCreate(MOTOR1_PWM, 0, 100); // Setup soft PWM for Motor 1
    pinMode(MOTOR1_IN1, OUTPUT);
    pinMode(MOTOR1_IN2, OUTPUT);

    softPwmCreate(MOTOR2_PWM, 0, 100); // Setup soft PWM for Motor 2
    pinMode(MOTOR2_IN1, OUTPUT);
    pinMode(MOTOR2_IN2, OUTPUT);

    // LCD setup
    pinMode(LCD_RS, OUTPUT);
    pinMode(LCD_E, OUTPUT);
    pinMode(LCD_D4, OUTPUT);
    pinMode(LCD_D5, OUTPUT);
    pinMode(LCD_D6, OUTPUT);
    pinMode(LCD_D7, OUTPUT);

    lcdInit(2, 16, 4, LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7, 0, 0, 0, 0);
    lcdPuts("Self-Driving Car");
}

void moveForward() {
    digitalWrite(MOTOR1_IN1, HIGH);
    digitalWrite(MOTOR1_IN2, LOW);
    softPwmWrite(MOTOR1_PWM, 50); // Adjust PWM for speed control

    digitalWrite(MOTOR2_IN1, HIGH);
    digitalWrite(MOTOR2_IN2, LOW);
    softPwmWrite(MOTOR2_PWM, 50); // Adjust PWM for speed control
}

void stop() {
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, LOW);
    softPwmWrite(MOTOR1_PWM, 0);

    digitalWrite(MOTOR2_IN1, LOW);
    digitalWrite(MOTOR2_IN2, LOW);
    softPwmWrite(MOTOR2_PWM, 0);
}

void turnLeft() {
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, LOW);
    softPwmWrite(MOTOR1_PWM, 0);

    digitalWrite(MOTOR2_IN1, HIGH);
    digitalWrite(MOTOR2_IN2, LOW);
    softPwmWrite(MOTOR2_PWM, 50); // Adjust PWM for speed control
}

void scanWithServo() {
    for (int angle = 0; angle <= 180; angle += 10) {
        digitalWrite(SERVO_PIN, HIGH);
        delayMicroseconds(500 + angle * 100);
        digitalWrite(SERVO_PIN, LOW);
        delayMicroseconds(20000 - angle * 100);
    }
}

int getDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    while (digitalRead(ECHO_PIN) == LOW);
    long startTime = micros();

    while (digitalRead(ECHO_PIN) == HIGH);
    long travelTime = micros() - startTime;

    // Calculate distance in cm
    int distance = travelTime / 58;

    return distance;
}

int main() {
    setup();

    while (1) {
        int distance = getDistance();

        if (distance < 20) {
            stop();
            turnLeft();
        } else {
            moveForward();
        }

        // Add more logic as needed based on the actual scenario
        // ...

        delay(100); // Adjust the delay based on your system requirements
    }

    return 0;
}

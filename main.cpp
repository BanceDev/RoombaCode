#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHRPS.h>
#include <cmath>
#include <algorithm>
#include <FEHServo.h>
using namespace std;

#define RED 0
#define BLUE 1
#define OFF 2
#define CPI 40.386 //actual counts per inch of each wheel
#define MAXIPS 30.7
#define FORWARD 1
#define REVERSE -1
#define CLRCHCKYES 1
#define CLRCHCKNO 0


// class for the Drive train and its functions
class DriveTrain {
    private:
        // declare all of the motors and inputs
        // must be done like this since FEHMotor has no default constructor
        // 0 - front
        // 1 - back left
        // 2 - back right
        // positive speed turns cw.
        FEHMotor motor0 = FEHMotor(FEHMotor::Motor0,9.0);
        FEHMotor motor1 = FEHMotor(FEHMotor::Motor1,9.0);
        FEHMotor motor2 = FEHMotor(FEHMotor::Motor2,9.0);
        DigitalEncoder motorZeroEncoder = DigitalEncoder(FEHIO::P0_0);
        DigitalEncoder motorOneEncoder = DigitalEncoder(FEHIO::P0_1);
        DigitalEncoder motorTwoEncoder = DigitalEncoder(FEHIO::P0_2);

        float prevTime, prevError, firstTimeInMove;
        float pConst, iConst, dConst;
        float errorSum;
        int motorZeroCounts, motorOneCounts, motorTwoCounts;

        AnalogInputPin CdS = AnalogInputPin(FEHIO::P3_0);
                

    public:
        float minCdSValue;

        // Delcaration for all the functions below
        DriveTrain();
        void DriveForward(float speed, int forwardMotor, float distance, int direction, int checkColorYesNo);
        void DriveStrafe(float speed, int forwardMotor, float distance, int direction, int checkColorYesNo);
        void DriveRotate(float speed);
        void DriveCombined(float angle, int speed);
        void StopDriving();
        void Initialize();
        float PIDAdjustment(float expectedSpeed, int motor);
        void ResetPID();
        int GetStartColor();
        void checkMinCdSValue();
        
        FEHServo armServo = FEHServo(FEHServo::Servo7);

};

// Default Constructor
DriveTrain::DriveTrain() {
    prevTime = 0;
    firstTimeInMove = 0;
    prevError = 0;
    motorZeroCounts = 0;
    motorOneCounts = 0;
    motorTwoCounts = 0;
    errorSum = 0;
    pConst = 0.45;
    iConst = 0.05;
    dConst = 0.2;
    minCdSValue = 3.3;
}

// Function to stop all the motors
void DriveTrain::StopDriving() {
    motor0.SetPercent(0);
    motor1.SetPercent(0);
    motor2.SetPercent(0);
}

// function to reset all the PID variables
void DriveTrain::ResetPID() {
    prevTime = TimeNow();
    firstTimeInMove = TimeNow();
    prevError = 0;
    motorZeroCounts = 0;
    motorOneCounts = 0;
    motorTwoCounts = 0;
    errorSum = 0;
    motorZeroEncoder.ResetCounts();
    motorOneEncoder.ResetCounts();
    motorTwoEncoder.ResetCounts();
    Sleep(0.05);
}

//calculate PID terms

float DriveTrain::PIDAdjustment(float expectedSpeed, int motor) {
    int deltaCounts;
    float deltaTime;
    float actSpeed;
    float errorSpeed;
    float pTerm, iTerm, dTerm;
    if (motor == 0) {
        deltaCounts = motorZeroEncoder.Counts() - motorZeroCounts;
    } else if (motor == 1) {
        deltaCounts = motorOneEncoder.Counts() - motorOneCounts;
    } else {
        deltaCounts = motorTwoEncoder.Counts() - motorTwoCounts;
    }

    deltaTime = TimeNow() - prevTime;

    actSpeed = (1/CPI) * (deltaCounts/deltaTime);
    errorSpeed = expectedSpeed - actSpeed;
    errorSum += errorSpeed;

    pTerm = errorSpeed * pConst;
    iTerm = errorSum * iConst;
    dTerm = (errorSpeed - prevError) * dConst;
    prevError = errorSpeed;
    prevTime = TimeNow();
    motorZeroCounts = motorZeroEncoder.Counts();
    motorOneCounts = motorOneEncoder.Counts();
    motorTwoCounts = motorTwoEncoder.Counts();

    LCD.WriteAt("actSpeed",0,60);
    LCD.WriteAt(actSpeed,200,60);
    LCD.WriteAt("errorSpeed",0,80);
    LCD.WriteAt(errorSpeed,200,80);
    LCD.WriteAt("elapsedTime", 0, 100);
    LCD.WriteAt(prevTime - firstTimeInMove, 200, 100);
    LCD.WriteAt("P", 0, 140);
    LCD.WriteAt(pTerm, 200, 140);

    LCD.WriteAt(minCdSValue, 200, 20);

    return expectedSpeed + pTerm + iTerm + dTerm;
}

// Drive forward in the direction of the given motor
void DriveTrain::DriveForward(float speed, int forwardMotor, float distance, int direction, int checkColorYesNo) {
    ResetPID();
    LCD.Clear();
    if (forwardMotor == 0) { // Motor 0
        while(((motorOneEncoder.Counts() + motorTwoEncoder.Counts())/2) < (35.07 * distance)) {
            float motor1PID = -PIDAdjustment(speed, 1) * direction;
            float motor2PID = PIDAdjustment(speed, 2) * direction;
            motor1.SetPercent((motor1PID/MAXIPS)*100);
            motor2.SetPercent((motor2PID/MAXIPS)*100);
            if (checkColorYesNo == 1) {
                checkMinCdSValue();
            }
            Sleep(0.05);
        }
        
    } else if (forwardMotor == 1) { // Motor 1
        while(((motorZeroEncoder.Counts() + motorTwoEncoder.Counts())/2) < (35.07 * distance)) {
            float motor0PID = PIDAdjustment(speed, 0) * direction;
            float motor2PID = -PIDAdjustment(speed, 2) * direction;
            motor0.SetPercent((motor0PID/MAXIPS)*100);
            motor2.SetPercent((motor2PID/MAXIPS)*100);
            LCD.WriteAt("zeroCounts", 0, 0);
            LCD.WriteAt(motorZeroEncoder.Counts(), 200, 0);
            LCD.WriteAt("oneCounts", 0, 20);
            LCD.WriteAt(motorOneEncoder.Counts(), 200, 20);
            LCD.WriteAt("twoCounts", 0, 40);
            LCD.WriteAt(motorTwoEncoder.Counts(), 200, 40);
            if (checkColorYesNo == 1) {
                checkMinCdSValue();
            }
            Sleep(0.05);
        }
    } else {  // Motor 2
        while(((motorZeroEncoder.Counts() + motorOneEncoder.Counts())/2) < (35.07 * distance)) {
            float motor0PID = -PIDAdjustment(speed, 0) * direction;
            float motor1PID = PIDAdjustment(speed, 1) * direction;
            motor0.SetPercent((motor0PID/MAXIPS)*100);
            motor1.SetPercent((motor1PID/MAXIPS)*100);
            if (checkColorYesNo == 1) {
                checkMinCdSValue();
            }
            Sleep(0.05);
        }
    }
    StopDriving();
    
}

// Drive Horizontol
// Currently only parallel to motor 0
// TODO: Update it to be like DriveForward
void DriveTrain::DriveStrafe(float speed, int forwardMotor, float distance, int direction, int checkColorYesNo) {
    ResetPID();
    LCD.Clear();
    if (forwardMotor == 0) { // Motor 0
        while(motorZeroEncoder.Counts() < (CPI * distance)) {
            float motor0PID = -PIDAdjustment(speed, 0) * direction;
            float motor1PID = PIDAdjustment(speed/2, 1) * direction;
            float motor2PID = PIDAdjustment(speed/2, 2) * direction;

            motor0.SetPercent((motor0PID/MAXIPS)*100);
            motor1.SetPercent((motor1PID/MAXIPS)*100);
            motor2.SetPercent((motor2PID/MAXIPS)*100);
            if (checkColorYesNo == 1) {
                checkMinCdSValue();
            }
            Sleep(0.05);
        }
        
    } else if (forwardMotor == 1) { // Motor 1
        while(motorOneEncoder.Counts() < (CPI * distance)) {
            float motor0PID = PIDAdjustment(speed/2, 0) * direction;
            float motor1PID = -PIDAdjustment(speed, 1) * direction;
            float motor2PID = PIDAdjustment(speed/2, 2) * direction;

            motor0.SetPercent((motor0PID/MAXIPS)*100);
            motor1.SetPercent((motor1PID/MAXIPS)*100);
            motor2.SetPercent((motor2PID/MAXIPS)*100);
            if (checkColorYesNo == 1) {
                checkMinCdSValue();
            }
            Sleep(0.05);
        }
    } else {  // Motor 2
        while(motorTwoEncoder.Counts() < (CPI * distance)) {
            float motor0PID = PIDAdjustment(speed/2, 0) * direction;
            float motor1PID = PIDAdjustment(speed/2, 1) * direction;
            float motor2PID = -PIDAdjustment(speed, 2) * direction;

            motor0.SetPercent((motor0PID/MAXIPS)*100);
            motor1.SetPercent((motor1PID/MAXIPS)*100);
            motor2.SetPercent((motor2PID/MAXIPS)*100);
            if (checkColorYesNo == 1) {
                checkMinCdSValue();
            }
            Sleep(0.05);
        }
    }
    StopDriving();
    
}

// Rotate the robot
void DriveTrain::DriveRotate(float speed) {
    motor0.SetPercent(speed);
    motor1.SetPercent(speed);
    motor2.SetPercent(speed);
}

// Drive in any direction and with rotation.
// TODO: Add a way to make this field centric and not robot centric
void DriveTrain::DriveCombined(float angle, int speed) {
    float motorZeroSpeed = speed * sin(angle);
    float motorOneSpeed = 0.5 * speed * sin(angle) + (sqrt(3)/2) * speed * cos(angle);
    float motorTwoSpeed = -0.5 * speed * sin(angle) + (sqrt(3)/2) * speed * cos(angle);

    motor0.SetPercent(motorZeroSpeed);
    motor1.SetPercent(motorOneSpeed);
    motor2.SetPercent(motorTwoSpeed);
}


// Returns the color that the CdS Cell is seeing
int DriveTrain::GetStartColor() {
    if (CdS.Value() < 0.3) {
        return RED;
    } else if (0.3 < CdS.Value() && CdS.Value() < 1.5) {
        return BLUE;
    }

    return OFF;
}

void DriveTrain::checkMinCdSValue() {
    if (CdS.Value() < minCdSValue) {
        minCdSValue = CdS.Value();
    }
}

// initializes the drive train when the light turns on
void DriveTrain::Initialize() {
    while (GetStartColor() == OFF || GetStartColor() == BLUE) {}
}


// Class for the Robot
// Will bring in all the classes for the mechanisms
// Functions will be the different routines
class Robot {
    private:
        DriveTrain dt;
        FEHServo armServo = FEHServo(FEHServo::Servo7);
    public:
        Robot();
        void Lever();
        void LEDButton();
        void Passport();
        void Luggage();
        void FinalRoutine();

        FEHServo luggageServo = FEHServo(FEHServo::Servo6);
};

// Default Constructor
Robot::Robot() {

}

// Routine for the first checkpoint
void Robot::LEDButton() {
    //Strafe over to align with light
    dt.DriveStrafe(7, 0, 2, FORWARD, CLRCHCKNO);
    // Drive to light
    dt.DriveForward(7, 0, 21, REVERSE, CLRCHCKYES);
    dt.DriveForward(7, 0, 4, FORWARD, CLRCHCKNO);

    LCD.Clear();

    int thisisavar = -1;

    if (dt.minCdSValue < 0.3) { // RED
        thisisavar = 200;
        LCD.SetBackgroundColor(0xFF0000);
        LCD.Clear();
    } else if (0.3 < dt.minCdSValue && dt.minCdSValue < 1.5) { // BLUE
        thisisavar = 100;
        LCD.SetBackgroundColor(0x0000FF);
        LCD.Clear();
    }
    

    //move to the right button
    if (thisisavar == 200) { // RED
        dt.DriveStrafe(7, 0, 11, REVERSE, CLRCHCKNO);
        LCD.Clear();
    } else { // BLUE
        dt.DriveStrafe(7, 0, 6, REVERSE, CLRCHCKNO);
        LCD.Clear();
    }

    dt.DriveForward(7, 0, 5, REVERSE, CLRCHCKNO);
    dt.DriveForward(7, 0, 3, FORWARD, CLRCHCKNO);

}

// Routine for the first checkpoint
void Robot::Lever() {

    // Initialize on the light
    dt.Initialize();
    // position 1, wait 2 seconds, position 2, 
    // Get correct lever from the RPS
    int correctLever = RPS.GetCorrectLever();
    // Drive of launchpad
    dt.DriveForward(7, 0, 4, FORWARD, CLRCHCKNO);
    // Rotate to face ramp
    dt.DriveRotate(-30);
    Sleep(0.72);
    dt.StopDriving();
    dt.DriveForward(9, 1, 6, FORWARD, CLRCHCKNO);
     
    // Check which lever to flip and perform some action
    if(correctLever == 0) {
        dt.DriveStrafe(9, 1, 18, REVERSE, CLRCHCKNO);
    } else if(correctLever == 1) {
        dt.DriveStrafe(9, 1, 23, REVERSE, CLRCHCKNO);
    } else if(correctLever == 2) {
        dt.DriveStrafe(9, 1, 26, REVERSE, CLRCHCKNO);
    }
    // Flip the lever with 5 second delay
    Sleep(1.0);
    armServo.SetDegree(80);
    Sleep(0.6);
    dt.DriveForward(7, 1, 2, FORWARD, CLRCHCKNO);
    armServo.SetDegree(100);
    Sleep(4.0);
    dt.DriveForward(7, 1, 1.5, REVERSE, CLRCHCKNO);
    Sleep(0.6);
    armServo.SetDegree(35);
    Sleep(1.0);
    // Leave lever and align with wall
    dt.DriveForward(7, 1, 1, FORWARD, CLRCHCKNO);
    dt.DriveRotate(30);
    Sleep(0.4);
    dt.StopDriving();
    armServo.SetDegree(15);
    Sleep(0.5);
    dt.DriveForward(7, 0, 12, REVERSE, CLRCHCKNO);
}

void Robot::Passport() {
    dt.DriveRotate(30);
    Sleep(1.3);
    dt.StopDriving();
    armServo.SetDegree(100);
    dt.DriveStrafe(7, 1, 6, REVERSE, CLRCHCKNO);
    dt.DriveForward(4, 1, 2, REVERSE, CLRCHCKNO);
    armServo.SetDegree(50);
    Sleep(0.5);
    dt.DriveStrafe(7, 1, 3, FORWARD, CLRCHCKNO);
}

void Robot::Luggage() {
    // Leave wall and rotate to face ramp
    dt.DriveForward(7, 0, 3, FORWARD, CLRCHCKNO);
    dt.DriveRotate(30);
    Sleep(0.8);
    dt.StopDriving();
    dt.DriveForward(10, 0, 22, FORWARD, CLRCHCKNO);
    // Drive into wall at the top
    dt.DriveRotate(-30);
    Sleep(0.8);
    dt.StopDriving();
    dt.DriveForward(7, 0, 5, REVERSE, CLRCHCKNO);
    // drive off wall and rotate to face luggage drop
    dt.DriveForward(7, 0, 3, FORWARD, CLRCHCKNO);
    dt.DriveRotate(-30);
    Sleep(0.8);
    dt.StopDriving();
    //Strafe over to drop
    dt.DriveStrafe(7, 0, 6, REVERSE, CLRCHCKNO);
    // Drive into luggage drop
    dt.DriveForward(7, 0, 5, FORWARD, CLRCHCKNO);
    //Lift Servo Block
    luggageServo.SetDegree(10);
    Sleep(0.5);
    //Reset Servo
    luggageServo.SetDegree(100);
    Sleep(0.5);

}

void Robot::FinalRoutine() {
    Lever();
    Luggage();
    LEDButton();
    Passport();
}


int main(void) {

    // declare robot class
    Robot robot;

    RPS.InitializeTouchMenu();

    robot.FinalRoutine();

        
    return 0;
}
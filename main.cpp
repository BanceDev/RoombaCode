#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHRPS.h>
#include <cmath>
#include <algorithm>
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
        void DriveForward(float speed, int forwardMotor, int distance, int direction, int checkColorYesNo);
        void DriveHorizontal(float speed);
        void DriveRotate(float speed);
        void DriveCombined(float angle, int speed);
        void StopDriving();
        void Initialize();
        float PIDAdjustment(float expectedSpeed, int motor);
        void ResetPID();
        int GetStartColor();
        void checkMinCdSValue();

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
    minCdSValue = CdS.Value();
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
void DriveTrain::DriveForward(float speed, int forwardMotor, int distance, int direction, int checkColorYesNo) {
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
void DriveTrain::DriveHorizontal(float speed) {
    motor2.SetPercent(speed/2);
    motor1.SetPercent(speed/2);
    motor0.SetPercent(-speed);
    
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
    public:
        Robot();
        void Checkpoint3();
        void Checkpoint1();
        void PIDDebug();
};

// Default Constructor
Robot::Robot() {

}

// Routine for the first checkpoint
void Robot::Checkpoint1() {
    // Initialize on the light
    dt.Initialize();
    // Drive of launchpad
    dt.DriveForward(5, 0, 4, FORWARD, CLRCHCKNO);
    // Rotate to face ramp
    dt.DriveRotate(30);
    Sleep(0.4);
    dt.StopDriving();
    // Drive up ramp
    dt.DriveForward(10, 0, 32, FORWARD, CLRCHCKNO);
    // Rotate to face wall
    dt.DriveRotate(-30);
    Sleep(0.4);
    dt.StopDriving();
    // Align with wall
    dt.DriveForward(7.25, 1, 10, REVERSE, CLRCHCKNO);


    
    // Drive out some distance
    dt.DriveForward(7, 1, 11.5, FORWARD, CLRCHCKNO);
    
    // Drive to light
    dt.DriveForward(7, 2, 19, REVERSE, CLRCHCKYES);

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
    
    // back away from light
    dt.DriveForward(7, 2, 4.5, FORWARD, CLRCHCKNO);

    //move to the right button
    if (thisisavar == 200) { // RED
        dt.DriveForward(7, 1, 8, REVERSE, CLRCHCKNO);
    } else { // BLUE
        dt.DriveForward(7, 1, 1, REVERSE, CLRCHCKNO);
        LCD.Clear();
    }

    //turn towards sides to hit the button
    dt.DriveRotate(-30);
    Sleep(0.325);
    dt.StopDriving();
    
    //back up from light
    dt.DriveForward(7, 2, 7, REVERSE, CLRCHCKNO);


    /*
    // Drive out to hit opposite wall
    dt.DriveForward(7, 1, 31, FORWARD, CLRCHCKNO);
    // back up a bit to be aligned with light
    dt.DriveForward(6.5, 1, 8.5, REVERSE, CLRCHCKNO);
    // Rotate to orient towards light
    dt.DriveRotate(30);
    Sleep(0.26);
    dt.StopDriving();
    // Drive into light
    dt.DriveForward(6, 0, 18, FORWARD, CLRCHCKYES);
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
    //back up from light
    dt.DriveForward(7, 0, 5, REVERSE, CLRCHCKNO);
    //turn towards kiosk
    dt.DriveRotate(-30);
    Sleep(0.275);
    dt.StopDriving();
    //move to the right button
    if (thisisavar == 200) { // RED
        dt.DriveForward(7, 1, 10, REVERSE, CLRCHCKNO);
    } else { // BLUE
        dt.DriveForward(7, 1, 5, REVERSE, CLRCHCKNO);
        LCD.Clear();
    }
    //turn towards sides to hit the button
    dt.DriveRotate(-30);
    Sleep(0.275);
    dt.StopDriving();
    
    //back up from light
    dt.DriveForward(7, 2, 7, REVERSE, CLRCHCKNO);*/

    /*
    // Drive off kiosk
    dt.DriveForward(40, 2);
    Sleep(2.0);
    dt.StopDriving();
    // Rotate to face wall
    dt.DriveRotate(-30);
    Sleep(0.4);
    dt.StopDriving();
    // Drive into wall
    dt.DriveForward(40,0);
    Sleep(2.0);
    dt.StopDriving();
    // Pull off wall
    dt.DriveForward(-40, 0);
    Sleep(0.3);
    dt.StopDriving();
    // Rotate to face ramp
    dt.DriveRotate(-30);
    Sleep(0.9);
    dt.StopDriving();
    // Drive down ramp
    dt.DriveForward(30, 0);
    Sleep(3.5);
    dt.StopDriving();*/

}

// Routine for the first checkpoint
void Robot::Checkpoint3() {
     
    // Get correct lever from the RPS
    int correctLever = RPS.GetCorrectLever();

    dt.DriveForward(7, 0, 7, FORWARD, CLRCHCKNO);
    dt.DriveRotate(-30);
    Sleep(1.2);
    dt.StopDriving();
    dt.DriveForward(7, 2, 7, REVERSE, CLRCHCKNO);
     
    // Check which lever to flip and perform some action
    if(correctLever == 0)
    {
        // Perform actions to flip left lever
        dt.DriveForward(7, 2, 19, FORWARD, CLRCHCKNO);
    } 
    else if(correctLever == 1)
    {
        // Perform actions to flip middle lever
        dt.DriveForward(7, 2, 22, FORWARD, CLRCHCKNO);
    }
    else if(correctLever == 2)
    {
       // Perform actions to flip right lever
       dt.DriveForward(7, 2, 26, FORWARD, CLRCHCKNO);
    }
}

void Robot::PIDDebug() {
    //dt.DriveForward(7, 2, 32, FORWARD);
}


int main(void) {

    // declare robot class
    Robot robot;

    RPS.InitializeTouchMenu();

    robot.Checkpoint3();
        
    return 0;
}
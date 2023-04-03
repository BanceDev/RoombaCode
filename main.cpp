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
        FEHServo luggageServo = FEHServo(FEHServo::Servo6);
    public:
        Robot();
        void Checkpoint3();
        void Checkpoint1();
        void FlipLever();
        void StrafeTest();
        void Checkpoint4();
        void Checkpoint5();
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

}

void Robot::StrafeTest() {
    dt.DriveStrafe(7, 0, 10, FORWARD, CLRCHCKNO);
    dt.DriveStrafe(7, 0, 10, REVERSE, CLRCHCKNO);
}

// Routine for the first checkpoint
void Robot::Checkpoint3() {

    // Initialize on the light
    dt.Initialize();

    // position 1, wait 2 seconds, position 2, 
    // Get correct lever from the RPS
    int correctLever = RPS.GetCorrectLever();

    dt.DriveForward(7, 0, 9, FORWARD, CLRCHCKNO);
    dt.DriveRotate(-30);
    Sleep(1.3);
    dt.StopDriving();
    dt.DriveForward(7, 2, 7, REVERSE, CLRCHCKNO);
     
    // Check which lever to flip and perform some action
    if(correctLever == 0)
    {
        // Perform actions to flip left lever
        dt.DriveForward(7, 2, 19, FORWARD, CLRCHCKNO);
        dt.DriveRotate(30);
        Sleep(0.2);
        dt.StopDriving();
        dt.DriveForward(7, 1, 1, FORWARD, CLRCHCKNO);
        FlipLever();
    } 
    else if(correctLever == 1)
    {
        // Perform actions to flip middle lever
        dt.DriveForward(7, 2, 23, FORWARD, CLRCHCKNO);
        dt.DriveRotate(30);
        Sleep(0.2);
        dt.StopDriving();
        dt.DriveForward(7, 1, 1, FORWARD, CLRCHCKNO);
        FlipLever();
    }
    else if(correctLever == 2)
    {
        // Perform actions to flip right lever
        dt.DriveForward(7, 2, 26, FORWARD, CLRCHCKNO);
        dt.DriveRotate(30);
        Sleep(0.2);
        dt.StopDriving();
        dt.DriveForward(7, 1, 1, FORWARD, CLRCHCKNO);
        FlipLever();
    }
}

void Robot::FlipLever() {
    Sleep(1.0);
    armServo.SetDegree(80);
    Sleep(0.6);
    dt.DriveForward(7, 1, 2, FORWARD, CLRCHCKNO);
    armServo.SetDegree(85);
    Sleep(4.0);
    dt.DriveForward(7, 1, 1, REVERSE, CLRCHCKNO);
    Sleep(0.6);
    armServo.SetDegree(35);
    Sleep(1.0);
}

void Robot::Checkpoint4() {
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
    dt.DriveRotate(30);
    Sleep(0.2);
    dt.StopDriving();
    // Align with wall
    dt.DriveForward(7, 2, 10, FORWARD, CLRCHCKNO);
    // Drive off of wall
    dt.DriveForward(7, 2, 2, REVERSE, CLRCHCKNO);
    // Rotate to face passport
    dt.DriveRotate(30);
    Sleep(0.25);
    dt.StopDriving();
    // Drive back to give room for arm
    dt.DriveForward(7, 1, 2, FORWARD, CLRCHCKNO);
    // Drop arm
    armServo.SetDegree(100);
    Sleep(0.5);
    // Drive to get under lever
    dt.DriveForward(7, 1, 2, REVERSE, CLRCHCKNO);
    // Raise arm a little
    armServo.SetDegree(85);
    Sleep(0.5);
    // Drive Forward
    dt.DriveForward(7, 1, 2, REVERSE, CLRCHCKNO);
    // Raise arm more
    armServo.SetDegree(75);
    Sleep(0.5);
    // Drive forward to complete stamping
    dt.DriveForward(7, 1, 3, REVERSE, CLRCHCKNO);
    armServo.SetDegree(10);
    Sleep(0.5);
    dt.DriveForward(7, 1, 1, FORWARD, CLRCHCKNO);
    dt.DriveStrafe(7, 1, 5, FORWARD, CLRCHCKNO);
    dt.DriveForward(7, 1, 6, REVERSE, CLRCHCKNO);
    armServo.SetDegree(50);
    Sleep(0.5);
    dt.DriveRotate(-30);
    Sleep(0.7);
    dt.StopDriving();

}

void Robot::Checkpoint5() {
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
    dt.DriveRotate(30);
    Sleep(0.2);
    dt.StopDriving();
    // Align with wall
    dt.DriveForward(7, 2, 10, FORWARD, CLRCHCKNO);
    // Drive off of wall
    dt.DriveForward(7, 2, 17, REVERSE, CLRCHCKNO);
    // Rotate to orient with luggage
    dt.DriveRotate(30);
    Sleep(1.3);
    dt.StopDriving();
    //Drive into luggage
    dt.DriveForward(7, 0, 5, FORWARD, CLRCHCKNO);
    // Strafe back to ramp
    dt.DriveStrafe(7, 0, 15, REVERSE, CLRCHCKNO);
    // drive down ramp
    dt.DriveForward(7, 0, 32, FORWARD, CLRCHCKNO);

}


int main(void) {

    // declare robot class
    Robot robot;

    //RPS.InitializeTouchMenu();

    robot.Checkpoint5();
        
    return 0;
}
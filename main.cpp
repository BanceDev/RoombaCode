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

struct Vector2 {
    float x, y;

    // a function to subtract one vector from another
    // used to get vector line between two points
    struct Vector2 Subtract(struct Vector2 other) {
        struct Vector2 result;
        result.x = other.x - x;
        result.y = other.y - y;
        return result;
    }
    
    /* this function normalizes the vector
    this makes it's length equal to 1, useful for maintaining a constant velocity*/
    void Normalize()
    {
        float magnitude = sqrt(pow(x,2) + pow(y,2));
        x = x / magnitude;
        y = y / magnitude;
    }
    
    // this function returns the magnitude of the vector
    float Magnitude() {
        return sqrt(pow(x,2) + pow(y,2));
    }
    
};

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
        // Delcaration for all the functions below
        DriveTrain();
        void DriveForward(float speed, int forwardMotor, int distance, int direction);
        void DriveHorizontal(float speed);
        void DriveRotate(float speed);
        void DriveCombined(float angle, int speed);
        void DriveToPoint(Vector2 currentPos, Vector2 targetPos, int speed);
        void StopDriving();
        void Initialize();
        float PIDAdjustment(float expectedSpeed, int motor);
        void ResetPID();
        int GetColor();

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
    pConst = 0.5;
    iConst = 0.05;
    dConst = 0.2;
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

    return expectedSpeed + pTerm + iTerm + dTerm;
}

// Drive forward in the direction of the given motor
void DriveTrain::DriveForward(float speed, int forwardMotor, int distance, int direction) {
    ResetPID();
    LCD.Clear();
    if (forwardMotor == 0) { // Motor 0
        while(((motorOneEncoder.Counts() + motorTwoEncoder.Counts())/2) < (35.07 * distance)) {
            float motor1PID = -PIDAdjustment(speed, 1) * direction;
            float motor2PID = PIDAdjustment(speed, 2) * direction;
            motor1.SetPercent((motor1PID/MAXIPS)*100);
            motor2.SetPercent((motor2PID/MAXIPS)*100);
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
            Sleep(0.05);
        }
    } else {  // Motor 2
        while(((motorZeroEncoder.Counts() + motorOneEncoder.Counts())/2) < (35.07 * distance)) {
            float motor0PID = -PIDAdjustment(speed, 0) * direction;
            float motor1PID = PIDAdjustment(speed, 1) * direction;
            motor0.SetPercent((motor0PID/MAXIPS)*100);
            motor1.SetPercent((motor1PID/MAXIPS)*100);
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

// Use the Drive Combined function to drive to a point
// For this to work the robot needs to be oriented with motor zero at the top
// TODO: Make this field centric
void DriveTrain::DriveToPoint(Vector2 currentPos, Vector2 targetPos, int speed) {
    // get the vector between the two points
    Vector2 direction = targetPos.Subtract(currentPos);
    direction.Normalize();
    float angle = atan(direction.y/direction.x);
    DriveCombined(angle, speed);
}


// Returns the color that the CdS Cell is seeing
int DriveTrain::GetColor() {
    if (CdS.Value() < 0.3) {
        return RED;
    } else if (0.3 < CdS.Value() && CdS.Value() < 1.5) {
        return BLUE;
    } 

    return OFF;
}

// initializes the drive train when the light turns on
void DriveTrain::Initialize() {
    while (GetColor() == OFF || GetColor() == BLUE) {}
}



// Class for the Robot
// Will bring in all the classes for the mechanisms
// Functions will be the different routines
class Robot {
    private:
        DriveTrain dt;
    public:
        Robot();
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
    dt.DriveForward(7, 0, 4, FORWARD);
    // Rotate to face ramp
    dt.DriveRotate(30);
    Sleep(0.4);
    dt.StopDriving();
    // Drive up ramp
    dt.DriveForward(10, 0, 32, FORWARD);
    // Rotate to face wall
    dt.DriveRotate(-30);
    Sleep(0.4);
    dt.StopDriving();
    // Align with wall
    dt.DriveForward(12, 1, 5, REVERSE);
    // Drive out to hit opposite wall
    Sleep(5.0);
    dt.DriveForward(10, 1, 32, FORWARD);
    // Drive to face kiosk
    dt.DriveForward(7, 1, 6, REVERSE);
    // Rotate to orient towards kiosk
    dt.DriveRotate(30);
    Sleep(0.3);
    dt.StopDriving();
    // Drive into kiosk
    dt.DriveForward(7, 0, 10, FORWARD);
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

void Robot::PIDDebug() {
    dt.DriveForward(7, 2, 32, FORWARD);
}


int main(void) {

    // declare robot class
    Robot robot;

    robot.Checkpoint1();
        
    return 0;
}
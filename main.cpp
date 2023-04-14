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

#define PULSESPEED 25
#define PULSEDELAY 0.05
#define PULSEWAIT 0.2


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
        float robotX, robotY, robotH; //heheh more pain.
        float LEDX, LEDY;

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

        void UpdateRPS();
        void PulseRotate(int speed, float seconds);
        void PulseForward(int speed, float seconds, int checkColorYesNo);
        void PulseStrafe(int speed, float seconds, int checkColorYesNo);
        void CheckHeading(float heading);
        void CheckX(float x_coordinate, int orientation, int checkColorYesNo);
        void CheckY(float y_coordinate, int orientation, int checkColorYesNo);
        
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

    robotX = -1;
    robotY = -1;
    robotH = -1;
    LEDX = 11.63; // actual coordinates
    LEDY = 60.43;
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

    LCD.WriteAt("X",0,60);
    LCD.WriteAt(RPS.X(),200,60);
    LCD.WriteAt("Y",0,80);
    LCD.WriteAt(RPS.Y(),200,80);
    LCD.WriteAt("H", 0, 100);
    LCD.WriteAt(RPS.Heading(), 200, 100);
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

void DriveTrain::UpdateRPS() {
    robotX = RPS.X();
    robotY = RPS.Y();
    robotH = RPS.Heading();
}


void DriveTrain::PulseRotate(int speed, float seconds) {
    motor0.SetPercent(speed);
    motor1.SetPercent(speed);
    motor2.SetPercent(speed);

    Sleep(seconds);

    StopDriving();
}

void DriveTrain::PulseForward(int speed, float seconds, int checkColorYesNo) {
    motor1.SetPercent(-speed);
    motor2.SetPercent(speed);

    Sleep(seconds);

    if (checkColorYesNo == 1) {
        checkMinCdSValue();
    }

    StopDriving();
}

void DriveTrain::PulseStrafe(int speed, float seconds, int checkColorYesNo) {
    motor0.SetPercent(-speed);
    motor1.SetPercent(speed/2);
    motor2.SetPercent(speed/2);

    Sleep(seconds);

    if (checkColorYesNo == 1) {
        checkMinCdSValue();
    }

    StopDriving();
}

/*
 * Use RPS to move to the desired x_coordinate based on the orientation of the QR code
 */
void DriveTrain::CheckX(float x_coordinate, int orientation, int checkColorYesNo)
{
    // Determine the direction of the motors based on the orientation of the QR code
    int power = PULSESPEED;
    if (orientation == -1)
    {
        power = -PULSESPEED;
    }

    // Check if receiving proper RPS coordinates and whether the robot is within an acceptable range
    while ((RPS.X() < x_coordinate - 0.5 || RPS.X() > x_coordinate + 0.5) && RPS.X() != -2)
    {
        if (RPS.X() > x_coordinate)
        {
            // Pulse the motors for a short duration in the correct direction
            PulseStrafe(-power, PULSEDELAY, checkColorYesNo);
        }
        else if (RPS.X() < x_coordinate)
        {
            // Pulse the motors for a short duration in the correct direction
            PulseStrafe(power, PULSEDELAY, checkColorYesNo);
        }
        Sleep(PULSEWAIT);
    }
}

/*
 * Use RPS to move to the desired y_coordinate based on the orientation of the QR code
 */
void DriveTrain::CheckY(float y_coordinate, int orientation, int checkColorYesNo)
{
    // Determine the direction of the motors based on the orientation of the QR code
    int power = PULSESPEED;
    if (orientation == -1)
    {
        power = -PULSESPEED;
    }

    // Check if receiving proper RPS coordinates and whether the robot is within an acceptable range
    while ((RPS.Y() < y_coordinate - 1 || RPS.Y() > y_coordinate + 1) && RPS.Y() != -2)
    {
        if (RPS.Y() > y_coordinate)
        {
            // Pulse the motors for a short duration in the correct direction
            PulseForward(-power, PULSEDELAY, checkColorYesNo);
        }
        else if (RPS.Y() < y_coordinate)
        {
            // Pulse the motors for a short duration in the correct direction
            PulseForward(power, PULSEDELAY, checkColorYesNo);
        }
        Sleep(PULSEWAIT);
    }
}


/*
 * Use RPS to move to the desired heading
 */
void DriveTrain::CheckHeading(float heading)
{
    int power = PULSESPEED;
    // Check if receiving proper RPS coordinates and whether the robot is within an acceptable range
    while ((RPS.Heading() < heading - 3 || RPS.Heading() > heading + 3))
    {
        if (RPS.Heading() > heading)
        {
            // Pulse the motors for a short duration in the correct direction
            PulseRotate(-power, PULSEDELAY);
        }
        else if (RPS.Heading() < heading)
        {
            // Pulse the motors for a short duration in the correct direction
            PulseRotate(power, PULSEDELAY);
        }
        Sleep(PULSEWAIT);
    }
}


// initializes the drive train when the light turns on
void DriveTrain::Initialize() {
    while (GetStartColor() == OFF || GetStartColor() == BLUE) {
        UpdateRPS();
    }

    if (abs(LEDX - (robotX - 15.87)) <= 0.5 && abs(LEDY - (robotY + 51.93)) <= 0.5) {
        LEDX = robotX - 15.87; // hard code the position of the LED based off of initial RPS position
        LEDY = robotY + 51.93;
    }
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
void Robot::Lever() {

    // Initialize on the light
    dt.Initialize();
    // position 1, wait 2 seconds, position 2, 
    // Get correct lever from the RPS
    int correctLever = RPS.GetCorrectLever();
    // Drive off launchpad
    dt.DriveForward(7, 0, 3, FORWARD, CLRCHCKNO);
    // Rotate to face ramp
    dt.DriveRotate(-30);
    Sleep(0.71);
    dt.StopDriving();
    dt.DriveForward(9, 1, 6, FORWARD, CLRCHCKNO);
     
    // Check which lever to flip and perform some action
    if(correctLever == 0) {
        dt.DriveStrafe(9, 1, 18, REVERSE, CLRCHCKNO);
    } else if(correctLever == 1) {
        dt.DriveStrafe(9, 1, 21.5, REVERSE, CLRCHCKNO);
    } else if(correctLever == 2) {
        dt.DriveStrafe(9, 1, 26, REVERSE, CLRCHCKNO);
    }
    // Flip the lever with 5 second delay
    Sleep(1.0);
    armServo.SetDegree(80);
    Sleep(0.6);
    dt.DriveForward(7, 1, 2.5, FORWARD, CLRCHCKNO);
    armServo.SetDegree(100);
    Sleep(4.0);
    dt.DriveForward(7, 1, 2, REVERSE, CLRCHCKNO);
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

void Robot::Luggage() {
    // Leave wall and rotate to face ramp
    dt.DriveForward(7, 0, 3, FORWARD, CLRCHCKNO);
    dt.DriveRotate(30); //positive turns counterclockwise
    Sleep(0.8);
    dt.StopDriving();
    dt.DriveForward(10, 0, 22, FORWARD, CLRCHCKNO);
    // Drive into wall at the top
    dt.DriveRotate(-30);
    Sleep(0.85); //0.8
    dt.StopDriving();
    dt.DriveForward(7, 0, 8, REVERSE, CLRCHCKNO);
    // drive off wall and rotate to face luggage drop

    dt.DriveForward(7, 0, 10, FORWARD, CLRCHCKNO);
    dt.DriveRotate(-30);
    Sleep(0.8);
    dt.StopDriving();

    // Drive into luggage drop
    dt.DriveForward(7, 0, 7, FORWARD, CLRCHCKNO);
    //Lift Servo Block
    luggageServo.SetDegree(100);
    Sleep(0.5);
    //Reset Servo
    luggageServo.SetDegree(10);
    Sleep(0.5);

}


void Robot::LEDButton() {
    //Strafe over to align with light
    dt.DriveStrafe(7, 0, 2, FORWARD, CLRCHCKNO);

    ///RPS MOVEMENT
    dt.CheckX(dt.LEDX, -1, CLRCHCKNO);

    //should we check heading here too?

    // Drive to light
    dt.DriveForward(7, 0, 21, REVERSE, CLRCHCKYES);
    dt.CheckX(dt.LEDX, -1, CLRCHCKYES);
    dt.CheckY(dt.LEDY, -1, CLRCHCKYES);
 

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

    dt.DriveForward(7, 0, 6, REVERSE, CLRCHCKNO);
    dt.DriveForward(7, 0, 3, FORWARD, CLRCHCKNO);
    if (thisisavar == 200) { // RED
        dt.DriveStrafe(7, 0, 6, FORWARD, CLRCHCKNO);
    }
    dt.DriveForward(7, 0, 7, FORWARD, CLRCHCKNO);

}

void Robot::Passport() {

    //diagonally move to beneath passport
    dt.DriveForward(7, 2, 15, REVERSE, CLRCHCKNO);

    //wall align
    dt.DriveRotate(-30);
    Sleep(0.8);
    dt.StopDriving();
    dt.DriveForward(7, 0, 7, REVERSE, CLRCHCKNO);
    dt.DriveForward(7, 0, 4.5, FORWARD, CLRCHCKNO);
    
    //drop arm
    armServo.SetDegree(100);
    Sleep(0.5);

    // strafe to right
    dt.DriveStrafe(7, 0, 1, FORWARD, CLRCHCKNO);

    //turns towards passport
    dt.DriveRotate(-30); 
    Sleep(0.3);
    dt.StopDriving();

    //raise passport
    armServo.SetDegree(68.6);
    Sleep(0.5);

    //push it all the way and back
    dt.DriveForward(7, 1, 4.25, REVERSE, CLRCHCKNO);
    dt.DriveForward(7, 1, 5.75, FORWARD, CLRCHCKNO);

    //raise arm
    armServo.SetDegree(15);

    //wall align
    dt.DriveRotate(30); //30 deg turn
    Sleep(0.275);
    dt.StopDriving();
    dt.DriveForward(7, 0, 6.5, REVERSE, CLRCHCKNO);
    dt.DriveForward(7, 0, 3, FORWARD, CLRCHCKNO);

    //rotate towards final button
    dt.DriveRotate(-30);
    Sleep(0.8);
    dt.StopDriving();
    
    //drive and hit the button
    dt.DriveForward(7, 0, 30, REVERSE, CLRCHCKNO);
    dt.DriveForward(15, 0, 10, REVERSE, CLRCHCKNO);

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
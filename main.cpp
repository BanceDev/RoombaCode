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

        AnalogInputPin CdS = AnalogInputPin(FEHIO::P3_0);
                

    public:
        // Delcaration for all the functions below
        DriveTrain();
        void DriveForward(float speed, int forwardMotor, int distance);
        void DriveHorizontal(float speed);
        void DriveRotate(float speed);
        void DriveCombined(float angle, int speed);
        void DriveToPoint(Vector2 currentPos, Vector2 targetPos, int speed);
        void StopDriving();
        void Initialize();
        void ShowEncoders();
        int GetColor();

};

// Default Constructor
DriveTrain::DriveTrain() {

}

// Function to stop all the motors
void DriveTrain::StopDriving() {
    motor0.SetPercent(0);
    motor1.SetPercent(0);
    motor2.SetPercent(0);
}

void DriveTrain::ShowEncoders() {
    LCD.Clear(BLACK);
    while(true) {
        LCD.WriteAt(motorOneEncoder.Counts(), 0, 0);
        LCD.WriteAt(motorTwoEncoder.Counts(), 0, 20);
        Sleep(0.2);
    }
    
}

// Drive forward in the direction of the given motor
void DriveTrain::DriveForward(float speed, int forwardMotor, int distance) {
    if (forwardMotor == 0) { // Motor 0
        motorOneEncoder.ResetCounts();
        motorTwoEncoder.ResetCounts();
        motor2.SetPercent(speed);
        motor1.SetPercent(-speed);
        LCD.Clear();
        while(((abs(motorOneEncoder.Counts()) + abs(motorTwoEncoder.Counts()))/2) < (35.07 * distance)) {
            LCD.WriteAt(motorOneEncoder.Counts(), 0, 0);
            LCD.WriteAt(motorTwoEncoder.Counts(), 0, 20);
        }
        StopDriving();
    } else if (forwardMotor == 1) { // Motor 1
        motor0.SetPercent(speed);
        motor2.SetPercent(-speed);
    } else {  // Motor 2
        motor0.SetPercent(-speed);
        motor1.SetPercent(speed);
    }
    
    
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
        void EncoderTest();
};

// Default Constructor
Robot::Robot() {

}

// Routine for the first checkpoint
/*void Robot::Checkpoint1() {
    // Initialize on the light
    dt.Initialize();
    // Drive of launchpad
    dt.DriveForward(30, 0);
    Sleep(0.7);
    dt.StopDriving();
    // Rotate to face ramp
    dt.DriveRotate(30);
    Sleep(0.4);
    dt.StopDriving();
    // Drive up ramp
    dt.DriveForward(50, 0);
    Sleep(2.7);
    dt.StopDriving();
    // Rotate to face wall
    dt.DriveRotate(-30);
    Sleep(0.4);
    dt.StopDriving();
    // Align with wall
    dt.DriveForward(-30, 1);
    Sleep(2.0);
    dt.StopDriving();
    // Drive out to face kiosk
    dt.DriveForward(30, 1);
    Sleep(2.0);
    dt.StopDriving();
    // Rotate to orient towards kiosk
    dt.DriveRotate(-30);
    Sleep(0.4);
    dt.StopDriving();
    // Drive into kiosk
    dt.DriveForward(-40, 2);
    Sleep(2.5);
    dt.StopDriving();
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
    dt.StopDriving();

}*/

void Robot::EncoderTest() {
    dt.ShowEncoders();
}

int main(void) {

    // declare robot class
    Robot robot;

    robot.EncoderTest();

    
    return 0;
}
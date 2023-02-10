#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHLCD.h>
#include <FEHIO.h>

#define LEFT 1
#define RIGHT 0
#define FRONT 0
#define BACK 1

// class for the Drive train and its functions
class DriveTrain {
    private:
        // declare all of the motors and inputs
        // must be done like this since FEHMotor has no default constructor
        // cries in lack of modular capabilities TwT
        FEHMotor rightMotor = FEHMotor(FEHMotor::Motor0,9.0);
        FEHMotor leftMotor = FEHMotor(FEHMotor::Motor1,9.0);
        DigitalInputPin backLeft = DigitalInputPin(FEHIO::P0_2);
        DigitalInputPin backRight = DigitalInputPin(FEHIO::P3_7);
        DigitalInputPin frontLeft = DigitalInputPin(FEHIO::P0_0);
        DigitalInputPin frontRight = DigitalInputPin(FEHIO::P0_1);

    public:
        DriveTrain();
        void AwaitBumper(bool bumper1, bool bumper2);
        void DriveStraight(int speed);
        void TurnUntilBumper(int dir, int side, int speed);
        void DriveUntilBumper(int side, int speed);
        void StopDriving();

};

DriveTrain::DriveTrain() {

}

// function to drive both wheels at the same speed for straight driving
void DriveTrain::DriveStraight(int speed) {
    // drive both wheels at given speed
    rightMotor.SetPercent(speed);
    leftMotor.SetPercent(speed);
}

// function to stop both wheels 
void DriveTrain::StopDriving() {
    rightMotor.Stop();
    leftMotor.Stop();
}

// function to wait for bumper contact.
void DriveTrain::AwaitBumper(bool bumper1, bool bumper2) {
    // waits for both bumpers to be pressed
    while(!bumper1 && !bumper2);
    StopDriving();
}

// Drives straight until both bumpers are pressed
// side can be front or back
// 0 for front bumpers 1 for back bumpers
void DriveTrain::DriveUntilBumper(int side, int speed) {
    DriveStraight(speed);
    
    if (side == 0) {
        AwaitBumper(frontLeft.Value(), frontRight.Value());
    } else if (side == 1) {
        AwaitBumper(backLeft.Value(), backRight.Value());
    }

}

// Turns until both bumpers are pressed
// dir can be left or right wheel and side can be front or back
// 0 for right wheel and 1 for left wheel
// 0 for front bumpers 1 for back bumpers
void DriveTrain::TurnUntilBumper(int dir, int side, int speed) {
    if (dir == 0) {
        rightMotor.SetPercent(speed);
    } else if (dir == 1) {
        leftMotor.SetPercent(speed);
    }

    if (side == 0) {
        AwaitBumper(frontLeft.Value(), frontRight.Value());
    } else if (side == 1) {
        AwaitBumper(backLeft.Value(), backRight.Value());
    }
}



int main(void) {

    //Initialize the Drive Train
    DriveTrain driveTrain;

    //Run the drive routine for the maze
    driveTrain.DriveUntilBumper(FRONT, 25);
    driveTrain.TurnUntilBumper(RIGHT, BACK, 25);
    driveTrain.DriveUntilBumper(FRONT, 25);
    driveTrain.TurnUntilBumper(LEFT, BACK, 25);
    driveTrain.DriveUntilBumper(FRONT, 25);

    
    return 0;
}
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHLCD.h>
#include <FEHIO.h>

// class for the Drive train and its functions
class DriveTrain {
    private:
        // declare all of the motors and inputs
        // must be done like this since FEHMotor has no default constructor
        // cries in lack of modular capabilities TwT
        FEHMotor rightMotor = FEHMotor(FEHMotor::Motor0,9.0);
        FEHMotor leftMotor = FEHMotor(FEHMotor::Motor1,9.0);

    public:
        DriveTrain();
        void DriveStraight(int speed);
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




int main(void) {

    //Initialize the Drive Train
    DriveTrain driveTrain;
    
    return 0;
}
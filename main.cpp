#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHLCD.h>
#include <FEHIO.h>

enum LineStates {

    MIDDLE,

    RIGHT,

    LEFT

};


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
        void FollowLine();

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

// function to follow a line
void DriveTrain::FollowLine() {


    int state = MIDDLE; // Set the initial state

    while (true) { // I will follow this line forever!
        switch(state) {
        // If I am in the middle of the line...

            case MIDDLE:
            // Set motor powers for driving straight

                /* Drive */
                if ( /* Right sensor is on line */ ) {

                    state = RIGHT; // update a new state

                }
            /* Code for if left sensor is on the line */
                break;

            // If the right sensor is on the line...
            case RIGHT:
            // Set motor powers for right turn
            /* Drive */
                if( /* I no longer need to turn right… */ ) {

                /* update a new state */

                }

                break;

            // If the left sensor is on the line...
            case LEFT:
            /* Mirror operation of RIGHT state */

                break;

            default: // Error. Something is very wrong.

                break;

        }

    // Sleep a bit

    }

    
}




int main(void) {

    //Initialize the Drive Train
    DriveTrain driveTrain;
    
    return 0;
}
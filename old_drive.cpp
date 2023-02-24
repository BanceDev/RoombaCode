// class for the Drive train and its functions
class DriveTrain {
    private:
        // declare all of the motors and inputs
        // must be done like this since FEHMotor has no default constructor
        // cries in lack of modular capabilities TwT
        FEHMotor rightMotor = FEHMotor(FEHMotor::Motor0,9.0);
        FEHMotor leftMotor = FEHMotor(FEHMotor::Motor1,9.0);
        DigitalEncoder rightEncoder = DigitalEncoder(FEHIO::P0_0);
        DigitalEncoder leftEncoder = DigitalEncoder(FEHIO::P0_1);

        AnalogInputPin rightSensor = AnalogInputPin(FEHIO::P2_0);
        AnalogInputPin middleSensor = AnalogInputPin(FEHIO::P2_1);
        AnalogInputPin leftSensor = AnalogInputPin(FEHIO::P2_2);
                

    public:
        DriveTrain();
        void DriveStraight(int speed);
        void StopDriving();
        void FollowLine();
        void DriveTurn(int speed1, int speed2);
        void DriveDistance(float distance, int speed);
        void TurnDegrees(float deg, int dir);

};

DriveTrain::DriveTrain() {

}

// function to drive both wheels at the same speed for straight driving
void DriveTrain::DriveStraight(int speed) {
    // drive both wheels at given speed
    rightMotor.SetPercent(speed);
    leftMotor.SetPercent(speed);
}

// funtion to drive a given distance
void DriveTrain::DriveDistance(float distance, int speed) {
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();
    while (leftEncoder.Counts() < (distance * 40.5) && rightEncoder.Counts() < (distance * 40.5)) {
        DriveStraight(speed);
    }
    StopDriving();
}


void DriveTrain::DriveTurn(int speed1, int speed2) {
    // drive both wheels at given speed
    rightMotor.SetPercent(speed1);
    leftMotor.SetPercent(speed2);
}

void DriveTrain::TurnDegrees(float deg, int dir) {
    // turn the specicied direction for a number of degrees
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();

    LCD.WriteAt(leftEncoder.Counts(), 0, 0); 
    if (dir == 0) { // turn to the left
        while (rightEncoder.Counts() < ((2 * M_PI * 7 * (deg/360)) * 40.5)) {
            DriveTurn(25, 0);
        }
    } else if (dir == 1) { // turn to the right
        while (leftEncoder.Counts() < ((2 * M_PI * 7 * (deg/360)) * 40.5)) {
            DriveTurn(0, 25);
            LCD.WriteAt(leftEncoder.Counts(), 0, 0); 
        }
    }
    StopDriving();

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
                DriveStraight(25);
                // Set motor powers for driving straight

                /* Drive */
                if (rightSensor.Value() > 2.3) {
                    state = RIGHT; // update a new state
                }
                /* Code for if left sensor is on the line */
                if (leftSensor.Value() > 2.3) {
                    state = LEFT; // update to left
                }
                break;

            // If the right sensor is on the line...
            case RIGHT:
                DriveTurn(0,25);
                // Set motor powers for right turn
                /* Drive */
                if(rightSensor.Value() < 1.5) {
                    state = MIDDLE;
                }

                break;

            // If the left sensor is on the line...
            case LEFT:
            /* Mirror operation of RIGHT state */
                DriveTurn(25,0);
                // Set motor powers for right turn
                /* Drive */
                if(leftSensor.Value() > 1.5) {
                    state = MIDDLE;
                }
                break;

            default: // Error. Something is very wrong.

                break;

        }

        // Sleep a bit
        Sleep(0.1);
    }
}
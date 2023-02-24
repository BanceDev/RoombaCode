#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHLCD.h>
#include <FEHIO.h>
#include <cmath>
#include <algorithm>
using namespace std;



// class for the Drive train and its functions
class DriveTrain {
    private:
        // declare all of the motors and inputs
        // must be done like this since FEHMotor has no default constructor
        // cries in lack of modular capabilities TwT
        FEHMotor motor0 = FEHMotor(FEHMotor::Motor0,9.0);
        FEHMotor motor1 = FEHMotor(FEHMotor::Motor1,9.0);
        FEHMotor motor2 = FEHMotor(FEHMotor::Motor2,9.0);
        DigitalEncoder rightEncoder = DigitalEncoder(FEHIO::P0_0);
        DigitalEncoder leftEncoder = DigitalEncoder(FEHIO::P0_1);

                

    public:
        DriveTrain();
        void DriveVertical(float speed);
        void DriveHorizontal(float speed);
        void DriveRotate(float speed);
        void DriveCombined(float speedX, float speedY, float speedRot);
        float FindAbsMax(float speed1, float speed2, float speed3);

};

DriveTrain::DriveTrain() {

}

void DriveTrain::DriveVertical(float speed) {
    motor2.SetPercent(speed);
    motor1.SetPercent(-speed);
    
}

void DriveTrian::DriveHorizontal(float speed) {
    motor2.SetPercent(speed/2);
    motor1.SetPercent(speed/2);
    motor0.SetPercent(-speed);
    
}

void DriveTrain::DriveRotate(float speed) {
    motor0.SetPercent(speed);
    motor1.SetPercent(speed);
    motor2.SetPercent(speed);
}

void DriveTrain::DriveCombined(float speedX, float speedY, float speedRot, int speedConst) {
    float motorZeroSpeed = -speedX;
    float motorOneSpeed = speedX/2;
    float motorTwoSpeed = speedX/2;

    motorOneSpeed += -(speedY * (sqrt(3)/2));
    motorTwoSpeed += (speedY * (sqrt(3)/2));

    motorZeroSpeed += speedRot;
    motorOneSpeed += speedRot;
    motorTwoSpeed += speedRot;

    if (abs(motorZeroSpeed) > 1 || abs(motorOneSpeed) > 1 || abs(motorTwoSpeed) > 1) {
        float maxSpeed = FindAbsMax(motorZeroSpeed, motorOneSpeed, motorTwoSpeed);
        motorZeroSpeed /= maxSpeed;
        motorOneSpeed /= maxSpeed;
        motorTwoSpeed /= maxSpeed;
    }
    motor0.SetPercent(motorZeroSpeed * speedConst);
    motor1.SetPercent(motorOneSpeed * speedConst);
    motor2.SetPercent(motorTwoSpeed * speedConst);
}

float DriveTrain::FindAbsMax(float speed1, float speed2, float speed3) {
    float maxSpeed;
    maxSpeed = max(abs(speed1), abs(speed2));
    maxSpeed = max(maxSpeed, abs(speed3));
    return maxSpeed;
}


int main(void) {

    //Initialize the Drive Train
    DriveTrain driveTrain;
    driveTrain.DriveDistance(14, 25);
    driveTrain.TurnDegrees(90, LEFT_TURN, 25);
    driveTrain.DriveDistance(10, 25);
    driveTrain.TurnDegrees(90, RIGHT_TURN, 25);
    driveTrain.DriveDistance(4, 25);
    
    return 0;
}
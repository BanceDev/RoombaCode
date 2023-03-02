#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHLCD.h>
#include <FEHIO.h>
#include <cmath>
#include <algorithm>
using namespace std;

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
        // cries in lack of modular capabilities TwT
        FEHMotor motor0 = FEHMotor(FEHMotor::Motor0,9.0);
        FEHMotor motor1 = FEHMotor(FEHMotor::Motor1,9.0);
        FEHMotor motor2 = FEHMotor(FEHMotor::Motor2,9.0);
        DigitalEncoder rightEncoder = DigitalEncoder(FEHIO::P0_0);
        DigitalEncoder leftEncoder = DigitalEncoder(FEHIO::P0_1);

        AnalogInputPin CdS = AnalogInputPin(FEHIO::P3_7);
                

    public:
        DriveTrain();
        void DriveVertical(float speed);
        void DriveHorizontal(float speed);
        void DriveRotate(float speed);
        void DriveCombined(Vector2 direction, float rotation, int speed);
        void DriveToPoint(Vector2 currentPos, Vector2 targetPos, float rotation, float speed);
        void StopDriving();
        float FindAbsMax(float speed1, float speed2, float speed3);
        float CdSValue();

};

DriveTrain::DriveTrain() {

}

void DriveTrain::StopDriving() {
    motor0.SetPercent(0);
    motor1.SetPercent(0);
    motor2.SetPercent(0);
}

void DriveTrain::DriveVertical(float speed) {
    motor2.SetPercent(speed);
    motor1.SetPercent(-speed);
    
}

void DriveTrain::DriveHorizontal(float speed) {
    motor2.SetPercent(speed/2);
    motor1.SetPercent(speed/2);
    motor0.SetPercent(-speed * 0.9);
    
}

void DriveTrain::DriveRotate(float speed) {
    motor0.SetPercent(speed);
    motor1.SetPercent(speed);
    motor2.SetPercent(speed);
}

void DriveTrain::DriveCombined(Vector2 direction, float rotation, int speed) {
    float motorZeroSpeed = -direction.x;
    float motorOneSpeed = direction.x/2;
    float motorTwoSpeed = direction.x/2;

    motorOneSpeed += -(direction.y * (sqrt(3)/2));
    motorTwoSpeed += (direction.y * (sqrt(3)/2));

    motorZeroSpeed += rotation;
    motorOneSpeed += rotation;
    motorTwoSpeed += rotation;

    if (abs(motorZeroSpeed) > 1 || abs(motorOneSpeed) > 1 || abs(motorTwoSpeed) > 1) {
        float maxSpeed = FindAbsMax(motorZeroSpeed, motorOneSpeed, motorTwoSpeed);
        motorZeroSpeed /= maxSpeed;
        motorOneSpeed /= maxSpeed;
        motorTwoSpeed /= maxSpeed;
    }
    motor0.SetPercent(motorZeroSpeed * speed);
    motor1.SetPercent(motorOneSpeed * speed);
    motor2.SetPercent(motorTwoSpeed * speed);
}

void DriveTrain::DriveToPoint(Vector2 currentPos, Vector2 targetPos, float rotation, float speed) {
    // get the vector between the two points
    Vector2 direction = targetPos.Subtract(currentPos);
    direction.Normalize();
    DriveCombined(direction, rotation, speed);
}

float DriveTrain::FindAbsMax(float speed1, float speed2, float speed3) {
    float maxSpeed;
    maxSpeed = max(abs(speed1), abs(speed2));
    maxSpeed = max(maxSpeed, abs(speed3));
    return maxSpeed;
}

float DriveTrain::CdSValue() {
    return CdS.Value();
}


int main(void) {

    //Initialize the Drive Train
    DriveTrain driveTrain;

    // 0 - front
    // 1 - back left
    // 2 - back right
    // positive speed turns cw.

    //deetz test code
    driveTrain.DriveVertical(50);
    Sleep(1.5);
    driveTrain.StopDriving();

    
    //og code
    /*Vector2 dir;
    dir.x = -1;
    dir.y = -1;
    driveTrain.DriveVertical(25);
    Sleep(3.0);
    driveTrain.StopDriving();
    driveTrain.DriveHorizontal(25);
    Sleep(3.0);
    driveTrain.StopDriving();
    driveTrain.DriveCombined(dir, 0, 25);
    Sleep(3.0);
    driveTrain.StopDriving();
    driveTrain.DriveRotate(25);
    Sleep(3.0);
    driveTrain.StopDriving();*/
    
    return 0;
}
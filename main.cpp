#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHLCD.h>
#include <FEHIO.h>


int main(void) {

    //TODO: swap naming
    DigitalInputPin backLeft(FEHIO::P0_0);
    DigitalInputPin backRight(FEHIO::P0_1);
    DigitalInputPin frontLeft(FEHIO::P0_2);
    DigitalInputPin frontRight(FEHIO::P3_7);

    FEHMotor rightMotor(FEHMotor::Motor0,9.0);
    FEHMotor leftMotor(FEHMotor::Motor1,9.0);

    rightMotor.SetPercent(25);
    leftMotor.SetPercent(25);

    Sleep(1.0);  

    rightMotor.Stop();
    leftMotor.Stop();



    return 0;
}
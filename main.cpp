#include <FEHIO.h>
#include <FEHLCD.h>
#include <FEHUtility.h>

int main(void)
{
    AnalogInputPin CdS_cell(FEHIO::P0_0);
    
    while(true) {
        LCD.Clear(BLACK);
        LCD.Write(CdS_cell.Value());
        Sleep(0.2);
    }
    return 0;
}

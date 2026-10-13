// Bitchshifter pad diagnostic: prints, 10 times a second, how strongly each of
// the MPR121's 12 channels reacts (baseline minus filtered reading; libDaisy
// counts a touch at 12 or more) and which channels it reports as touched.
#include "daisy_seed.h"

using namespace daisy;

DaisySeed hw;
Mpr121I2C pads;

int main(void) {
    hw.Init();
    hw.StartLog(false);
    Mpr121I2C::Config config;
    bool ok = pads.Init(config) == Mpr121I2C::OK;
    while (1) {
        if (!ok) {
            hw.PrintLine("MPR121 not found");
            System::Delay(1000);
            continue;
        }
        int delta[12];
        for (int i = 0; i < 12; i++) delta[i] = (int)pads.BaselineData(i) - (int)pads.FilteredData(i);
        uint16_t touched = pads.Touched();
        char bits[13];
        for (int i = 0; i < 12; i++) bits[i] = (touched & (1 << i)) ? "0123456789AB"[i] : '.';
        bits[12] = 0;
        hw.PrintLine("d %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d | %s", delta[0], delta[1], delta[2],
                     delta[3], delta[4], delta[5], delta[6], delta[7], delta[8], delta[9], delta[10],
                     delta[11], bits);
        System::Delay(100);
    }
}

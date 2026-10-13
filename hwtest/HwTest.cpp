// Bitchshifter hardware test for the Synthux Simple Touch: prints the 8 analog
// inputs (A0-A7, as 0-100), both 3-way switches and the 12 touch pads over USB
// serial, 5 times a second. Move one control at a time to see which is which.
#include "daisy_seed.h"

using namespace daisy;

DaisySeed hw;
Switch3 switch_a, switch_b;
Mpr121I2C pads;

int main(void) {
    hw.Init();
    hw.StartLog(false);

    AdcChannelConfig cfg[8];
    Pin pins[8] = {seed::A0, seed::A1, seed::A2, seed::A3, seed::A4, seed::A5, seed::A6, seed::A7};
    for (int i = 0; i < 8; i++) cfg[i].InitSingle(pins[i]);
    hw.adc.Init(cfg, 8);
    hw.adc.Start();

    // Pin pairs as wired on the Simple Touch (from Synthux's TouchFX source)
    switch_a.Init(seed::D7, seed::D6);
    switch_b.Init(seed::D9, seed::D8);

    Mpr121I2C::Config pad_config;
    bool pads_ok = pads.Init(pad_config) == Mpr121I2C::OK;

    const char *positions[3] = {"C", "L", "R"}; // Switch3: 0 centre, 1 left/up, 2 right/down
    while (1) {
        int a[8];
        for (int i = 0; i < 8; i++) a[i] = (int)(hw.adc.GetFloat(i) * 100.f + 0.5f);
        uint16_t touched = pads_ok ? pads.Touched() : 0;
        char pad_bits[13];
        for (int i = 0; i < 12; i++) pad_bits[i] = (touched & (1 << i)) ? "123456789ABC"[i] : '.';
        pad_bits[12] = 0;
        hw.PrintLine("A0-A7: %3d %3d %3d %3d %3d %3d %3d %3d | sw A:%s B:%s | pads:%s %s",
                     a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7],
                     positions[switch_a.Read()], positions[switch_b.Read()], pad_bits,
                     pads_ok ? "" : "(MPR121 not found)");
        hw.SetLed(touched != 0);
        System::Delay(200);
    }
}

# bit-chifter — ready to share

Make the repository understandable and usable by someone who wasn't part of building it.
No sound changes.

## README (rewritten for players)

1. **What it is:** a frequency shifter / ring mod voiced as a shortwave sideband receiver, for
   the Synthux Simple Touch (Daisy Seed); inspired by the Doepfer A-126 and others; a short tour.
2. **Hardware:** Simple Touch + Daisy Seed (tested on a Seed 1.2 / PCM3060), stereo in and out,
   the cabling note (TRS → 2×TS into balanced inputs).
3. **Controls:** faders, knobs, switch 1 (page), switch 2 (Normal / Freeze / Loop), TOU, CH
   (unused); a pad map per page laid out like the board; RECEPTION also sets the pads' amount;
   pads are on while held.
4. **Sounds to try:** a handful of starting points.
5. **Building and flashing:** libDaisy next to the repo; the Daisy bootloader (the firmware
   runs from SRAM: `APP_TYPE = BOOT_SRAM`), installed once with libDaisy's `make program-boot`;
   `make`; RESET, BOOT while the LED pulses, `make program-dfu`.
6. **The USB report:** each field (CPU, controls, stereo mode, levels, pitch, page, `fx`, capture).
7. **Developing:** Mac tests (`sim/run_tests.sh`), the render tool (`sim/render.sh` and its
   options), the hardware diagnostics in `tools/`, the design history in `docs/`.
8. **Future:** a Daisy Patch front end with MIDI in, which would also bring a musical pitch
   shifter (interval transposition) — deferred from the Simple Touch, which has no MIDI in.
9. **License:** MIT (libDaisy, MIT).

## Tidy

- Move `hwtest/`, `padtest/`, `leveltest/` into `tools/` (`git mv`), with their Makefiles'
  `LIBDAISY_DIR` updated (`../../../libDaisy`); each still builds.
- Add `sim/experiments/` to `.gitignore` (local experiments; the files are left as they are).
- Add `docs/README.md`: the specs and plans are the design history, in order; which spec is
  current for what; the early files use the project's old name.
- Check that the Makefile and the source files' top comments read well to an outsider.

## Out of scope

A prebuilt release, audio demos, the musical pitch shifter (see Future), any sound changes.

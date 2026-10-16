# Design history

bit-chifter was built step by step from a design spec and one implementation plan per step
(in `superpowers/`), each executed with tests and reviews. They're kept as a record of why
things are the way they are; the code and the main README are the current truth. The project
was first called "Bitchshifter", so the earliest files use that name.

## Specs (what it should be)

- `superpowers/specs/2026-10-12-bitchshifter-design.md`: the core: faders, knobs, signal chain, safety,
  testing (updated for the later changes).
- `superpowers/specs/2026-10-13-bit-chifter-strange-pages-design.md`: the three pages of pad effects and
  switch 2's capture modes.
- `superpowers/specs/2026-10-16-bit-chifter-sharing-design.md`: getting the repository ready to share.

## Plans (how it was built, in order)

1. `superpowers/plans/2026-10-12-bitchshifter-part1-shifter.md`: the frequency shifter, feedback, mix.
2. `superpowers/plans/2026-10-13-bitchshifter-part2-ring-filter.md`: ring mod, radio filter, true stereo.
3. `superpowers/plans/2026-10-13-bitchshifter-part3-reception.md`: RECEPTION (drift, fading, static, AGC, drive).
4. `superpowers/plans/2026-10-13-bitchshifter-part3b-ssb-dial.md`: TUNE as a sideband dial with harmonic detents.
5. `superpowers/plans/2026-10-13-bit-chifter-reverb.md`: the plate reverb on MIX's top end.
6. `superpowers/plans/2026-10-13-bit-chifter-pages.md`: the pages, the pad logic and the first effects.
7. `superpowers/plans/2026-10-15-bit-chifter-radio.md`: page 2's radio effects.
8. `superpowers/plans/2026-10-15-bit-chifter-digital.md`: page 3's damage effects and Chaos.
9. `superpowers/plans/2026-10-16-bit-chifter-time-capture.md`: page 1's echoes, spirals, chirp, warble,
   and switch 2 (Freeze / Loop).

Later changes made after playing it (momentary pads, the damage moved after the mix, Selcall and
Morse chimes) are noted in the specs where they apply.

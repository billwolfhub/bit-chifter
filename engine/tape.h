#pragma once

#include "capture_loop.h"
#include "echo_loop.h"

namespace bitch {

// The big buffers for the echoes and the capture (about 1.1 MB): the front end keeps this in
// SDRAM and passes it to Engine::Init. Plain arrays, so it stays constant-initialisable (global
// constructors run before the SDRAM is set up); it starts as garbage, which the engine never
// reads before writing (the echo clears its line first, the capture reads only what it recorded).
struct TapeMemory {
    float echo[2][EchoLoop::kMax];
    float capture[2][CaptureLoop::kMax];
};

} // namespace bitch

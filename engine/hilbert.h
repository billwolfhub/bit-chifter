#pragma once

namespace bitch {

// 90-degree phase splitter: two parallel chains of four second-order allpass
// sections (Olli Niemitalo's coefficients). The two outputs (the A chain one sample
// later (I), and the B chain (Q)) stay 90 degrees apart, within a fraction of a degree, from about 20 Hz to
// over 20 kHz at 48 kHz.
class Hilbert {
  public:
    void Reset() {
        for (State &s : a_) s = State();
        for (State &s : b_) s = State();
        a_delay_ = 0.f;
    }

    void Process(float x, float &i, float &q) {
        float a = x, b = x;
        for (int k = 0; k < 4; k++) a = Section(a, kA[k], a_[k]);
        for (int k = 0; k < 4; k++) b = Section(b, kB[k], b_[k]);
        i = a_delay_;
        a_delay_ = a;
        q = b;
    }

  private:
    struct State {
        float x1 = 0.f, x2 = 0.f, y1 = 0.f, y2 = 0.f;
    };

    // y[n] = c * (x[n] + y[n-2]) - x[n-2], c = coefficient squared
    static float Section(float x, float c, State &s) {
        float y = c * (x + s.y2) - s.x2;
        s.x2 = s.x1;
        s.x1 = x;
        s.y2 = s.y1;
        s.y1 = y;
        return y;
    }

    static constexpr float kA[4] = {0.6923878f * 0.6923878f, 0.9360654322959f * 0.9360654322959f,
                                    0.9882295226860f * 0.9882295226860f, 0.9987488452737f * 0.9987488452737f};
    static constexpr float kB[4] = {0.4021921162426f * 0.4021921162426f, 0.8561710882420f * 0.8561710882420f,
                                    0.9722909545651f * 0.9722909545651f, 0.9952884791278f * 0.9952884791278f};
    State a_[4], b_[4];
    float a_delay_ = 0.f;
};

} // namespace bitch

#ifndef PID_H
#define PID_H

extern "C" {
  #include <inttypes.h>
}

namespace {
  template<typename T> T clamp(T x, T a, T b) {
    if(x > b)
      return b;
    else if(x < a)
      return a;

    return x;
  }
}

struct PIDFrame {
  int16_t Input[3];
  int16_t Target;
  float Output;

  int16_t e;
  int16_t pTerm;
  float iTerm;
  float dTerm;

  uint8_t Enabled;

  void reset() {
    memset(this, 0, sizeof *this);    
  }

  void nextInput(int16_t Input_) {
    Input[2] = Input[1];
    Input[1] = Input[0];
    Input[0] = Input_;
  }

  PIDFrame() {
    reset();
  }
} __attribute__((packed));

// velocity pid controller (see http://www2.widener.edu/~crn0001/Engr314/Digital%20PID%20Controllers-2.pdf)
template<unsigned int ms> struct PID {
  // Constants
  constexpr static float UpdateInterval = (float) ms / 1000.f;
  constexpr static float OutputMinimum = -350.f;
  constexpr static float OutputMaximum = 350.f;

  // Tunings
  float Kc;
  float tI;
  float tD;

  void setTunings(float Kp, float Ki, float Kd) {
    Kc = Kp;
    tI = UpdateInterval / Ki;
    tD = Kd / UpdateInterval;
  }

  void compute(PIDFrame& frame) {
    if (!frame.Enabled) {
      frame.Output = frame.Target;
      return;
    }

    frame.e = frame.Target - frame.Input[0];
    frame.pTerm = frame.Input[1] - frame.Input[0];
    frame.iTerm = tI * (float) frame.e;
    frame.dTerm = -tD * (float) (frame.Input[0] - 2*frame.Input[1] + frame.Input[2]);
    frame.Output = clamp(
      frame.Output + Kc * ((float) frame.pTerm + frame.iTerm + frame.dTerm),
      OutputMinimum,
      OutputMaximum
    );
  }

  PID(float Kp, float Ki, float Kd) {
    setTunings(Kp, Ki, Kd);
  }
};

#endif /* PID_H */
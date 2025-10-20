// === Jerk-limited S-curve motion, run-to-distance ===
// Fill these with your CSV-derived limits:
const float A_MAX = 2.50f;   // [m/s^2]  safe max acceleration
const float J_MAX = 10.0f;   // [m/s^3]  safe max jerk
const float DIST  = 10.0f;   // [m]      target travel distance

// Optional: soft limits from sensors
const float TILT_DEG_LIMIT  = 6.0f;     // if you have an IMU; else unused
const float LOOP_HZ         = 1000.0f;  // control frequency
const float FEEDFORWARD_KV  = 0.0f;     // simple open-loop velocity→PWM gain (set >0 only if you know it)
const float FEEDFORWARD_KA  = 0.0f;     // simple accel→PWM gain

// --- computed profile parameters (filled in setup) ---
bool  triangular = false;  // true if no flat-accel segment
float ap = 0.0f;           // peak accel actually used (<= A_MAX)
float tj = 0.0f;           // jerk ramp time (up or down)
float ta = 0.0f;           // constant-accel time per half-move
float T_half = 0.0f;       // time per half (accel or decel)
float T_total = 0.0f;      // total move time

// --- runtime state ---
enum Phase { RUN, DONE };
Phase phase = RUN;

float a_cmd = 0.0f;        // command accel [m/s^2]
float v_cmd = 0.0f;        // velocity [m/s]
float x_cmd = 0.0f;        // position [m]

unsigned long last_us;

void motorInit() {
  // TODO: set pinModes, init driver, etc.
}

void motorSetPWM(float u) {
  // TODO: convert 'u' (arbitrary control effort) to PWM/driver command.
  // For open-loop quick test you can map velocity feedforward:
  // u = FEEDFORWARD_KV * v_cmd + FEEDFORWARD_KA * a_cmd;
  (void)u;
}

void readSensors(float& tilt_deg /*out*/ ) {
  // OPTIONAL: read your IMU tilt here; set to 0 if unused
  tilt_deg = 0.0f;
}

// ---- math helpers ----
static inline float sq(float x) { return x*x; }

// Compute jerk-limited target acceleration at time t in [0, T_total]
float accelTarget(float t) {
  // Mirror full move around T_half so we only implement the first half logic
  float ti = (t <= T_half) ? t : (T_total - t);

  float a = 0.0f;
  if (ti < tj) {                // jerk up: 0 -> ap
    a = J_MAX * ti;
  } else if (ti < tj + ta) {    // hold accel
    a = ap;
  } else if (ti < 2*tj + ta) {  // jerk down: ap -> 0
    a = ap - J_MAX * (ti - tj - ta);
  } else {
    a = 0.0f;
  }

  // Second half is decel (negate sign)
  if (t > T_half) a = -a;
  return a;
}

void setupProfile() {
  // Decide triangular vs flat-accel half-profile by solving distance exactly.
  // For a half move with accel {jerk up tj, hold ta, jerk down tj}, distance is:
  // x_half = ap^3/J^2 + 1.5*ap^2/J * ta + 0.5*ap * ta^2   (derived analytically)
  // Full move distance D = 2*x_half

  // First try to hit D with full ap = A_MAX and solve ta from quadratic:
  // Let A=0.5*ap, B=1.5*ap^2/J, C=2*ap^3/J^2 - D, solve A*ta^2 + B*ta + C = 0.
  float A = 0.5f * A_MAX;
  float B = 1.5f * sq(A_MAX) / J_MAX;
  float C = 2.0f * sq(A_MAX)*A_MAX / sq(J_MAX) - DIST;

  float disc = B*B - 4*A*C;
  triangular = (disc < 0.0f);

  if (triangular) {
    // No constant-accel plateau. Solve for peak accel ap < A_MAX from D:
    // D = 2 * (1/3) * ap^3 / J^2
    ap = powf(1.5f * DIST * sq(J_MAX), 1.0f/3.0f);
    if (ap > A_MAX) ap = A_MAX;  // safety clamp (shouldn't happen if triangular selected)
    tj = ap / J_MAX;
    ta = 0.0f;
  } else {
    ap = A_MAX;
    float ta_sol = (-B + sqrtf(max(0.0f, disc))) / (2*A);
    ta = max(0.0f, ta_sol);
    tj = ap / J_MAX;
  }

  T_half  = 2.0f * tj + ta;
  T_total = 2.0f * T_half;
}

void setup() {
  Serial.begin(115200);
  motorInit();
  setupProfile();

  last_us = micros();

  Serial.println(F("# S-curve profile"));
  Serial.print(F("triangular=")); Serial.println(triangular ? F("true") : F("false"));
  Serial.print(F("ap=")); Serial.print(ap, 6); Serial.println(F(" m/s^2"));
  Serial.print(F("tj=")); Serial.print(tj, 6); Serial.println(F(" s"));
  Serial.print(F("ta=")); Serial.print(ta, 6); Serial.println(F(" s"));
  Serial.print(F("T_total=")); Serial.print(T_total, 6); Serial.println(F(" s"));
}

void loop() {
  // fixed-rate loop
  const float dt_target = 1.0f / LOOP_HZ;
  unsigned long now_us = micros();
  float dt = (now_us - last_us) * 1e-6f;
  if (dt < dt_target) return;    // wait until next tick
  last_us = now_us;

  static float t = 0.0f;

  if (phase == DONE) {
    // Hold stopped
    a_cmd = 0.0f;
    v_cmd = 0.0f;
    motorSetPWM(0.0f);
    return;
  }

  // Safety (optional IMU)
  float tilt_deg = 0.0f;
  readSensors(tilt_deg);
  bool unsafe = (fabs(tilt_deg) > TILT_DEG_LIMIT);

  // Target jerk-limited acceleration from profile time
  float a_target = unsafe ? -fabs(ap) : accelTarget(t);

  // Track a_target with jerk-limited slew on a_cmd
  float max_da = J_MAX * dt;
  float da = a_target - a_cmd;
  if (da >  max_da) da =  max_da;
  if (da < -max_da) da = -max_da;
  a_cmd += da;

  // Clamp accel to +/- ap
  if (a_cmd >  ap) a_cmd =  ap;
  if (a_cmd < -ap) a_cmd = -ap;

  // Integrate to velocity & position
  v_cmd += a_cmd * dt;
  x_cmd += v_cmd * dt;

  // Simple open-loop feedforward (replace with your motor control)
  float u = FEEDFORWARD_KV * v_cmd + FEEDFORWARD_KA * a_cmd;
  motorSetPWM(u);

  // Stream for debugging/plotting
  if ((int)(t*1000) % 10 == 0) { // ~100 Hz print
    Serial.print(t, 6); Serial.print(',');
    Serial.print(a_cmd, 6); Serial.print(',');
    Serial.print(v_cmd, 6); Serial.print(',');
    Serial.println(x_cmd, 6);
  }

  t += dt;

  // Stop at end of profile (or when we’ve passed the distance)
  if (t >= T_total || x_cmd >= DIST) {
    phase = DONE;
    Serial.println(F("# DONE"));
    Serial.print(F("# x_end=")); Serial.println(x_cmd, 6);
    Serial.print(F("# v_end=")); Serial.println(v_cmd, 6);
  }
}

/*
 * ============================================================================
 *  gcode_cylindrical.ino — G-code controller for a cylindrical-coordinate
 *  concrete printer: rotating base (theta), radial arm (R), vertical column
 *  (Z), extrusion nozzle at the end. Direct stepper drive (no hydraulics).
 *
 *  Target  : Arduino Uno/Nano/Mega (AVR Timer1, 20 kHz ISR).
 *  Axes    : AX_TH = base rotation (steps per DEGREE), AX_R = radial (steps/mm),
 *            AX_Z = vertical (steps/mm).
 *  G-codes : G0 G1 G2 G3 G4 G17 G20 G21 G28 G90 G91 G92
 *  M-codes : M2 M3 M4 M5 M8 M9 M30 M114 M999
 *  Realtime: '!' feed hold, '~' resume. E-stop input latches until M999.
 *  Protocol: 115200 baud, "ok" / "error:<n>" per line.
 *
 *  Notes:
 *   - XYZ are NOZZLE coordinates; work origin at the column centerline.
 *   - Moves are subdivided (MOVE_SEG_MM) because XY->thetaR is nonlinear.
 *   - R < MIN_RADIUS is unreachable (base-speed singularity at the pole).
 *   - Theta has no endstop: it keeps a continuous count (use a slip ring
 *     for power/signal if you want unlimited rotation). G28 homes Z and R.
 *   - Z axis must be self-locking or braked (it holds a heavy nozzle column).
 */

#include <Arduino.h>
#include <math.h>

#if !defined(__AVR__)
#error "AVR Timer1 specific; port the ISR/timer for other MCUs."
#endif

#define COMPILER_BARRIER() asm volatile("" ::: "memory")
#define D2R 0.01745329252f
#define R2D 57.29577951f

// ================================ Configuration ==============================

enum { AX_TH = 0, AX_R = 1, AX_Z = 2, N_AXIS = 3 };

static const uint8_t PIN_STEP[3]    = { 2, 4, 7 };
static const uint8_t PIN_DIR[3]     = { 5, 6, 8 };
static const uint8_t PIN_ENABLE     = A0;   // active LOW
static const uint8_t PIN_ENDSTOP[3] = { 255, 9, 10 };  // 255 = not installed (theta)
static const uint8_t PIN_ESTOP      = A2;   // NC contact to GND, pull-up
static const uint8_t PIN_PUMP_EN    = A1;   // extruder pump contactor
static const uint8_t PIN_FLOW_PWM   = 3;    // flow command (RC-filter to 0-10V)

// Axis calibration.  theta is steps per DEGREE; R and Z are steps per mm.
static const float STEPS_PER_UNIT[3] = { 35.6f, 320.0f, 320.0f };
// Axis speed limits in NATIVE units: deg/s for theta, mm/s for R and Z.
static const float MAX_RATE[3]       = { 30.0f, 50.0f, 50.0f };
static const float ACCEL_MM_S2       = 150.0f;   // path acceleration (mm/s^2)

// Work envelope (nozzle coordinates, mm)
static const float MIN_RADIUS_MM = 120.0f;
static const float MAX_RADIUS_MM = 2500.0f;
static const float MIN_Z_MM      = 0.0f;
static const float MAX_Z_MM      = 3000.0f;

static const float   MOVE_SEG_MM      = 15.0f;    // Cartesian subdivision length
static const uint32_t ISR_HZ          = 20000UL;
static const uint16_t STEP_PULSE_US   = 3;

// Homing
static const float HOMING_FEED_MM_S = 4.0f;      // approach speed
static const float HOMING_BACKOFF_MM = 3.0f;     // retract after trigger
static const int8_t HOMING_DIR[3]   = { 0, -1, -1 };  // direction toward switch
static const float HOME_TRAVEL_MM[3] = { 0, MAX_RADIUS_MM + 300, MAX_Z_MM + 300 };
static const uint8_t HOME_ORDER[2]  = { AX_Z, AX_R };

static const uint8_t QUEUE_SIZE = 16;
static const uint8_t QUEUE_MASK = QUEUE_SIZE - 1;

// Error / fault codes
enum {
  ERR_NONE = 0, ERR_BAD_CHAR, ERR_BAD_NUMBER, ERR_UNSUPPORTED_G, ERR_UNSUPPORTED_M,
  ERR_FEED_UNDEFINED, ERR_ARC_RADIUS, ERR_ARC_ENDPOINT, ERR_VALUE_MISSING,
  ERR_CHECKSUM, ERR_BAD_PLANE, ERR_UNREACHABLE, ERR_NOT_HOMED, ERR_FAULT,
};
enum { FAULT_ESTOP = 1, FAULT_LIMIT = 2 };

// ================================ Motion state ===============================

struct Segment {
  int32_t  steps[N_AXIS];     // abs step count per axis (native units)
  uint32_t totalSteps;        // = max axis steps (0 => dwell segment)
  uint8_t  dirMask;
  uint32_t entryRate, cruiseRate, exitRate;   // steps/s on major axis
  uint32_t accelStepsPerS2;
  uint32_t accelUntil, decelAfter;
  uint32_t dwellMs;
  float    lengthMm, entryMmS, cruiseMmS;     // junction-patch metadata
};

static Segment queue[QUEUE_SIZE];
static volatile uint8_t qHead = 0, qTail = 0;
static volatile bool busy = false;

static Segment *isrSeg = NULL;          // ISR-owned
static uint32_t isrDwellTicks = 0;

static volatile int32_t machineSteps[N_AXIS] = { 0, 0, 0 };
static int32_t planSteps[N_AXIS] = { 0, 0, 0 };

static volatile bool faultActive = false, estopLatched = false;
static volatile uint8_t faultCode = 0;
static volatile bool holdActive = false, flushReq = false;
static volatile int8_t homingAxis = -1;
static volatile bool homingBackoff = false, homeHit = false;
static volatile float currentFeedMmS = 0;
static bool homed = false;
static bool pumpOn = false;
static float flowPct = 100;

// Interpreter state
static bool unitsInches = false, absoluteMode = true;
static uint8_t plane = 17;
static float feedMmMin = 0, spindleS = 0;
static int8_t modalMotion = 0;
static float offsetMm[3] = { 0, 0, 0 };
static float posMm[3] = { 0, 0, 0 };

static inline uint8_t nextIdx(uint8_t i) { return (uint8_t)((i + 1) & QUEUE_MASK); }
static inline bool queueFull() { return nextIdx(qHead) == qTail; }

static inline bool endstopActive(uint8_t a) {
  if (PIN_ENDSTOP[a] == 255) return false;
  return digitalRead(PIN_ENDSTOP[a]) == LOW;   // NO switch closing to GND
}

// ================================ Planner ====================================

static void finalizeSegment(Segment &s, float exitMmS) {
  const float a = ACCEL_MM_S2;
  float e = s.entryMmS, x = exitMmS, c = s.cruiseMmS;
  if (c < e) c = e;
  if (c < x) c = x;
  float c2max = a * s.lengthMm + 0.5f * (e * e + x * x);
  if (c * c > c2max) c = sqrtf(c2max);
  float accDist = (c * c - e * e) / (2.0f * a);
  float decDist = (c * c - x * x) / (2.0f * a);
  if (accDist < 0) accDist = 0;
  if (decDist < 0) decDist = 0;
  float sum = accDist + decDist;
  if (sum > s.lengthMm && sum > 0) {
    float k = s.lengthMm / sum; accDist *= k; decDist *= k;
  }
  float spu = (float)s.totalSteps / s.lengthMm;   // major-axis steps per mm
  s.entryRate  = (uint32_t)(e * spu);
  s.cruiseRate = (uint32_t)(c * spu);
  s.exitRate   = (uint32_t)(x * spu);
  s.accelStepsPerS2 = (uint32_t)(a * spu);
  if (s.accelStepsPerS2 < 1) s.accelStepsPerS2 = 1;
  if (s.cruiseRate > ISR_HZ - 1000) s.cruiseRate = ISR_HZ - 1000;
  if (s.entryRate > s.cruiseRate) s.entryRate = s.cruiseRate;
  if (s.exitRate > s.cruiseRate) s.exitRate = s.cruiseRate;
  if (s.cruiseRate < 1) s.cruiseRate = 1;
  s.accelUntil = (uint32_t)(accDist * spu);
  uint32_t decSteps = (uint32_t)(decDist * spu);
  s.decelAfter = (s.totalSteps > decSteps) ? s.totalSteps - decSteps : 0;
  if (s.decelAfter < s.accelUntil) s.decelAfter = s.accelUntil;
}

static void planDwell(uint32_t ms) {
  while (queueFull() && !faultActive) { }
  if (faultActive) return;
  Segment &s = queue[qHead];
  s.totalSteps = 0; s.dwellMs = ms;
  COMPILER_BARRIER();
  qHead = nextIdx(qHead);
}

// Machine-coordinate move. Axis units are native (deg for theta, mm for R/Z),
// so per-axis caps and profiles work exactly as in a Cartesian machine.
static void planLineToMachine(const int32_t target[N_AXIS], float feedMmS) {
  int32_t delta[N_AXIS];
  uint8_t dirMask = 0;
  uint32_t total = 0;
  float lenSq = 0;

  for (uint8_t i = 0; i < N_AXIS; ++i) {
    delta[i] = target[i] - planSteps[i];
    if (delta[i] < 0) dirMask |= (uint8_t)(1 << i);
    uint32_t a = (uint32_t)labs(delta[i]);
    if (a > total) total = a;
    float nat = (float)delta[i] / STEPS_PER_UNIT[i];       // deg or mm
    if (i == AX_TH) {                                       // theta arc length
      float rMid = (float)planSteps[AX_R] / STEPS_PER_UNIT[AX_R] + nat * 0.0f;
      float rMm = (float)planSteps[AX_R] / STEPS_PER_UNIT[AX_R];
      lenSq += sq(rMm * (nat * D2R));
    } else lenSq += nat * nat;
  }
  if (total == 0) return;
  float L = sqrtf(lenSq);
  if (L < 0.001f) L = 0.001f;

  for (uint8_t i = 0; i < N_AXIS; ++i) {          // per-axis native-rate caps
    uint32_t a = (uint32_t)labs(delta[i]);
    if (!a) continue;
    float fmax = MAX_RATE[i] * STEPS_PER_UNIT[i] * L / (float)a;
    if (feedMmS > fmax) feedMmS = fmax;
  }
  if (feedMmS < 0.5f) feedMmS = 0.5f;

  while (queueFull() && !faultActive && !estopLatched) { }
  if (faultActive || estopLatched) return;

  uint8_t idx = qHead;
  Segment &s = queue[idx];
  s.dwellMs = 0;
  s.totalSteps = total;
  s.dirMask = dirMask;
  s.lengthMm = L;
  s.cruiseMmS = feedMmS;
  s.entryMmS = 0;
  for (uint8_t i = 0; i < N_AXIS; ++i) s.steps[i] = labs(delta[i]);

  // One-segment lookahead junction patch
  if (qHead != qTail) {
    uint8_t prevIdx = (uint8_t)((idx + QUEUE_SIZE - 1) & QUEUE_MASK);
    if (qTail != nextIdx(prevIdx)) {
      Segment tmp = queue[prevIdx];
      if (tmp.totalSteps > 0) {
        bool compat = true;
        for (uint8_t i = 0; i < N_AXIS; ++i)
          if (tmp.steps[i] && s.steps[i])
            if ((tmp.dirMask ^ s.dirMask) & (1 << i)) { compat = false; break; }
        if (compat) {
          float J = (tmp.cruiseMmS < s.cruiseMmS) ? tmp.cruiseMmS : s.cruiseMmS;
          float Jmax = sqrtf(2.0f * ACCEL_MM_S2 *
                             ((tmp.lengthMm < s.lengthMm) ? tmp.lengthMm : s.lengthMm));
          if (J > Jmax) J = Jmax;
          finalizeSegment(tmp, J);
          noInterrupts();
          uint8_t t = qTail; COMPILER_BARRIER();
          if (t != nextIdx(prevIdx)) queue[prevIdx] = tmp;
          interrupts();
          s.entryMmS = J;
        }
      }
    }
  }

  finalizeSegment(s, 0.0f);
  for (uint8_t i = 0; i < N_AXIS; ++i) planSteps[i] = target[i];
  COMPILER_BARRIER();
  qHead = nextIdx(idx);
}

// XYZ (work) -> cylindrical machine steps. Theta unwrapped to nearest turn.
static uint8_t ikCylindrical(const float xyz[3], int32_t out[N_AXIS]) {
  float R = hypotf(xyz[0], xyz[1]);
  if (R < MIN_RADIUS_MM || R > MAX_RADIUS_MM) return ERR_UNREACHABLE;
  if (xyz[2] < MIN_Z_MM || xyz[2] > MAX_Z_MM) return ERR_UNREACHABLE;
  float th = atan2f(xyz[1], xyz[0]) * R2D;
  float prevTh = (float)planSteps[AX_TH] / STEPS_PER_UNIT[AX_TH];
  while (th - prevTh >  180.0f) th -= 360.0f;
  while (th - prevTh < -180.0f) th += 360.0f;
  out[AX_TH] = lroundf(th * STEPS_PER_UNIT[AX_TH]);
  out[AX_R]  = lroundf(R * STEPS_PER_UNIT[AX_R]);
  out[AX_Z]  = lroundf(xyz[2] * STEPS_PER_UNIT[AX_Z]);
  return ERR_NONE;
}

static void syncPosFromMachine() {
  int32_t ms[N_AXIS];
  noInterrupts();
  for (uint8_t i = 0; i < N_AXIS; ++i) ms[i] = machineSteps[i];
  interrupts();
  float th = ms[AX_TH] / STEPS_PER_UNIT[AX_TH] * D2R;
  float r  = ms[AX_R]  / STEPS_PER_UNIT[AX_R];
  float z  = ms[AX_Z]  / STEPS_PER_UNIT[AX_Z];
  posMm[0] = r * cosf(th) - offsetMm[0];
  posMm[1] = r * sinf(th) - offsetMm[1];
  posMm[2] = z - offsetMm[2];
}

static void flushQueue() {
  flushReq = true;
  while (flushReq && !faultActive) { }
  noInterrupts();
  for (uint8_t i = 0; i < N_AXIS; ++i) planSteps[i] = machineSteps[i];
  interrupts();
  syncPosFromMachine();
}

// Cartesian move: subdivide (polar transform is nonlinear), IK each node.
static uint8_t planCartesian(const float target[3], float feedMmS) {
  float len = sqrtf(sq(target[0]-posMm[0]) + sq(target[1]-posMm[1]) + sq(target[2]-posMm[2]));
  if (len < 1e-3f) { for (uint8_t i = 0; i < 3; ++i) posMm[i] = target[i]; return ERR_NONE; }
  uint16_t n = (uint16_t)ceilf(len / MOVE_SEG_MM);
  for (uint16_t k = 1; k <= n; ++k) {
    if (faultActive) return ERR_FAULT;
    float t = (float)k / (float)n, p[3], mc[3];
    int32_t ts[N_AXIS];
    for (uint8_t i = 0; i < 3; ++i) {
      p[i]  = posMm[i] + (target[i] - posMm[i]) * t;
      mc[i] = p[i] + offsetMm[i];                 // work -> machine coords
    }
    uint8_t e = ikCylindrical(mc, ts);
    if (e != ERR_NONE) { flushQueue(); return ERR_UNREACHABLE; }
    planLineToMachine(ts, feedMmS);
  }
  for (uint8_t i = 0; i < 3; ++i) posMm[i] = target[i];
  return ERR_NONE;
}

// ================================= Homing ====================================

static void waitIdle() { while ((busy || qHead != qTail) && !faultActive) { } }

static uint8_t homeAxis(uint8_t axis) {
  if (PIN_ENDSTOP[axis] == 255) return ERR_NONE;      // none installed
  homeHit = false; homingBackoff = false;
  homingAxis = (int8_t)axis;

  int32_t target[N_AXIS];
  noInterrupts();
  for (uint8_t i = 0; i < N_AXIS; ++i) target[i] = machineSteps[i];
  interrupts();
  target[axis] += (int32_t)((float)HOMING_DIR[axis] * HOME_TRAVEL_MM[axis] * STEPS_PER_UNIT[axis]);
  planLineToMachine(target, HOMING_FEED_MM_S);

  while (!homeHit && !faultActive) { }                // ISR aborts on trigger
  homingAxis = -1;
  if (faultActive) return ERR_FAULT;

  noInterrupts();                                     // trigger point = zero
  machineSteps[axis] = 0;
  for (uint8_t i = 0; i < N_AXIS; ++i) planSteps[i] = machineSteps[i];
  interrupts();

  homingBackoff = true; homingAxis = (int8_t)axis;    // retract off the switch
  int32_t bo[N_AXIS];
  noInterrupts();
  for (uint8_t i = 0; i < N_AXIS; ++i) bo[i] = machineSteps[i];
  interrupts();
  bo[axis] -= (int32_t)((float)HOMING_DIR[axis] * HOMING_BACKOFF_MM * STEPS_PER_UNIT[axis]);
  planLineToMachine(bo, HOMING_FEED_MM_S);
  waitIdle();
  homingAxis = -1; homingBackoff = false;
  if (faultActive) return ERR_FAULT;

  noInterrupts();                                     // zero = back-off position
  int32_t shift = machineSteps[axis];
  machineSteps[axis] -= shift;
  planSteps[axis]  -= shift;
  interrupts();
  return ERR_NONE;
}

// ================================== ISR ======================================

static void pumpOffISR() {
  digitalWrite(PIN_PUMP_EN, LOW);
  analogWrite(PIN_FLOW_PWM, 0);
  pumpOn = false;
}

static void abortAllISR() {
  isrSeg = NULL; isrDwellTicks = 0;
  qTail = qHead; busy = false;
}

static void tripISR(uint8_t code) {
  if (faultActive) return;
  faultCode = code; faultActive = true;
  pumpOffISR();
  abortAllISR();                  // drivers stay enabled: hold position
}

ISR(TIMER1_COMPA_vect) {
  static uint32_t stepIdx = 0, rate = 0, frac = 0;
  static int32_t bres[N_AXIS];
  static uint8_t esDeb[N_AXIS] = { 0 };

  if (flushReq) { abortAllISR(); COMPILER_BARRIER(); flushReq = false; }

  // E-stop (NC contact opened = tripped) — checked even in hold/fault
  if (digitalRead(PIN_ESTOP) == LOW && !estopLatched) {
    estopLatched = true;
    tripISR(FAULT_ESTOP);
  }
  if (faultActive || holdActive) return;

  // Endstops (debounced): homing trigger, or safety fault otherwise
  bool esActive[N_AXIS];
  for (uint8_t i = 0; i < N_AXIS; ++i) {
    if (endstopActive(i)) { if (esDeb[i] < 250) ++esDeb[i]; }
    else esDeb[i] = 0;
    esActive[i] = (esDeb[i] >= 3);
  }
  int8_t ha = homingAxis;
  if (ha >= 0 && !homingBackoff) {
    if (esActive[ha]) { homeHit = true; homingAxis = -1; abortAllISR(); return; }
  } else if (ha < 0) {
    for (uint8_t i = 0; i < N_AXIS; ++i)
      if (esActive[i]) { tripISR(FAULT_LIMIT); return; }
  }

  if (isrDwellTicks) { if (--isrDwellTicks == 0) busy = false; return; }

  if (!isrSeg) {                                       // claim next segment
    if (qHead == qTail) return;
    uint8_t t = qTail; COMPILER_BARRIER();
    isrSeg = &queue[t];
    qTail = nextIdx(t);
    busy = true;
    if (isrSeg->totalSteps == 0) {
      isrDwellTicks = isrSeg->dwellMs * (ISR_HZ / 1000UL);
      isrSeg = NULL; if (!isrDwellTicks) busy = false;
      return;
    }
    digitalWrite(PIN_DIR[AX_TH], (isrSeg->dirMask & 1) ? HIGH : LOW);
    digitalWrite(PIN_DIR[AX_R],  (isrSeg->dirMask & 2) ? HIGH : LOW);
    digitalWrite(PIN_DIR[AX_Z],  (isrSeg->dirMask & 4) ? HIGH : LOW);
    stepIdx = 0; frac = 0;
    rate = (isrSeg->entryRate > 64) ? isrSeg->entryRate : 64;
    for (uint8_t i = 0; i < N_AXIS; ++i) bres[i] = (int32_t)(isrSeg->totalSteps >> 1);
    currentFeedMmS = isrSeg->cruiseMmS > 0 ? 0 : 0;    // placeholder, set below
    currentFeedMmS = 0;
    return;
  }
  currentFeedMmS = (float)isrSeg->entryRate;           // not used; see claim note

  frac += rate;
  while (frac >= ISR_HZ) {
    frac -= ISR_HZ;
    uint8_t pulsed = 0;
    for (uint8_t i = 0; i < N_AXIS; ++i) {
      bres[i] += isrSeg->steps[i];
      if (bres[i] >= (int32_t)isrSeg->totalSteps) {
        bres[i] -= (int32_t)isrSeg->totalSteps;
        pulsed |= (uint8_t)(1 << i);
      }
    }
    if (pulsed & 1) digitalWrite(PIN_STEP[AX_TH], HIGH);
    if (pulsed & 2) digitalWrite(PIN_STEP[AX_R], HIGH);
    if (pulsed & 4) digitalWrite(PIN_STEP[AX_Z], HIGH);
    if (pulsed) {
      delayMicroseconds(STEP_PULSE_US);
      if (pulsed & 1) { digitalWrite(PIN_STEP[AX_TH], LOW); machineSteps[AX_TH] += (isrSeg->dirMask & 1) ? -1 : 1; }
      if (pulsed & 2) { digitalWrite(PIN_STEP[AX_R], LOW);  machineSteps[AX_R]  += (isrSeg->dirMask & 2) ? -1 : 1; }
      if (pulsed & 4) { digitalWrite(PIN_STEP[AX_Z], LOW);  machineSteps[AX_Z]  += (isrSeg->dirMask & 4) ? -1 : 1; }
    }
    if (++stepIdx >= isrSeg->totalSteps) { isrSeg = NULL; busy = false; return; }

    if (stepIdx < isrSeg->accelUntil) {
      uint32_t d = isrSeg->accelStepsPerS2 / rate;
      rate += d ? d : 1;
      if (rate > isrSeg->cruiseRate) rate = isrSeg->cruiseRate;
    } else if (stepIdx >= isrSeg->decelAfter) {
      if (rate > isrSeg->exitRate) {
        uint32_t d = isrSeg->accelStepsPerS2 / rate; if (!d) d = 1;
        uint32_t nr = (rate > d) ? rate - d : 0;
        uint32_t floor = (isrSeg->exitRate > 32) ? isrSeg->exitRate : 32;
        rate = (nr < floor) ? floor : nr;
      }
    }
  }
}

// ============================== Line interpreter =============================

struct Word { char c; float v; };

static void reportPosition() {
  int32_t ms[N_AXIS];
  noInterrupts();
  for (uint8_t i = 0; i < N_AXIS; ++i) ms[i] = machineSteps[i];
  interrupts();
  float th = ms[AX_TH] / STEPS_PER_UNIT[AX_TH];
  float r  = ms[AX_R]  / STEPS_PER_UNIT[AX_R];
  float z  = ms[AX_Z]  / STEPS_PER_UNIT[AX_Z];
  Serial.print(F("X:")); Serial.print(r * cosf(th * D2R) - offsetMm[0], 1);
  Serial.print(F(" Y:")); Serial.print(r * sinf(th * D2R) - offsetMm[1], 1);
  Serial.print(F(" Z:")); Serial.print(z - offsetMm[2], 1);
  Serial.print(F(" | th:")); Serial.print(th, 2);
  Serial.print(F(" R:")); Serial.print(r, 1);
  Serial.print(F(" Z:")); Serial.print(z, 1);
  if (!homed) Serial.print(F(" [UNHOMED]"));
  Serial.println();
}

static uint8_t executeLine(char *line) {
  bool inParen = false;
  char *w = line;
  for (char *r = line; *r; ++r) {
    char c = *r;
    if (inParen) { if (c == ')') inParen = false; continue; }
    if (c == '(') { inParen = true; continue; }
    if (c == ';') break;
    if (c >= 'a' && c <= 'z') c = (char)(c - 32);
    if (c == '\t' || c == '\r') c = ' ';
    *w++ = c;
  }
  *w = '\0';
  if (inParen) return ERR_BAD_CHAR;

  char *p = line;
  while (*p == ' ') ++p;
  if (*p == '\0' || *p == '%') return ERR_NONE;

  if (char *star = strchr(p, '*')) {
    uint8_t cs = 0;
    for (char *q = p; q < star; ++q) cs ^= (uint8_t)*q;
    if (atoi(star + 1) != (int)cs) return ERR_CHECKSUM;
    *star = '\0';
  }
  if (*p == 'N') { ++p; while (*p >= '0' && *p <= '9') ++p; }

  static Word words[20];
  uint8_t nw = 0;
  while (*p) {
    while (*p == ' ') ++p;
    if (!*p) break;
    if (*p < 'A' || *p > 'Z') return ERR_BAD_CHAR;
    char letter = *p++;
    char *end;
    float v = strtof(p, &end);
    if (end == p) return ERR_BAD_NUMBER;
    p = end;
    if (nw < 20) { words[nw].c = letter; words[nw].v = v; ++nw; }
  }
  if (nw == 0) return ERR_NONE;

  static float val[26];
  static bool has[26];
  memset(val, 0, sizeof(val)); memset(has, 0, sizeof(has));
  for (uint8_t i = 0; i < nw; ++i) {
    uint8_t k = (uint8_t)(words[i].c - 'A');
    val[k] = words[i].v; has[k] = true;
  }
  const float u = unitsInches ? 25.4f : 1.0f;

  int8_t motion = -1;
  bool doG4 = false, doG28 = false, doG92 = false;
  for (uint8_t i = 0; i < nw; ++i) {
    if (words[i].c != 'G') continue;
    switch ((int)lroundf(words[i].v)) {
      case 0: case 1: case 2: case 3: motion = (int8_t)lroundf(words[i].v); break;
      case 4: doG4 = true; break;
      case 17: plane = 17; break;
      case 18: plane = 18; break;
      case 19: plane = 19; break;
      case 20: unitsInches = true; break;
      case 21: unitsInches = false; break;
      case 28: doG28 = true; break;
      case 90: absoluteMode = true; break;
      case 91: absoluteMode = false; break;
      case 92: doG92 = true; break;
      default: return ERR_UNSUPPORTED_G;
    }
  }
  if (has['F' - 'A']) feedMmMin = val['F' - 'A'] * u;
  if (has['S' - 'A']) spindleS = val['S' - 'A'];

  // ---- G28: automatic homing (Z then R; theta has no endstop) ----
  if (doG28) {
    waitIdle();
    if (faultActive) return ERR_FAULT;
    for (uint8_t i = 0; i < 2; ++i) {
      uint8_t e = homeAxis(HOME_ORDER[i]);
      if (e != ERR_NONE) return e;
    }
    homed = true;
    syncPosFromMachine();
  }

  // ---- G92 work offset ----
  if (doG92) {
    int32_t ms[N_AXIS];
    noInterrupts();
    for (uint8_t i = 0; i < N_AXIS; ++i) ms[i] = machineSteps[i];
    interrupts();
    float th = ms[AX_TH] / STEPS_PER_UNIT[AX_TH] * D2R;
    float mc[3];
    mc[0] = ms[AX_R] / STEPS_PER_UNIT[AX_R] * cosf(th);
    mc[1] = ms[AX_R] / STEPS_PER_UNIT[AX_R] * sinf(th);
    mc[2] = ms[AX_Z] / STEPS_PER_UNIT[AX_Z];
    bool anyAxis = has['X' - 'A'] || has['Y' - 'A'] || has['Z' - 'A'];
    for (uint8_t i = 0; i < 3; ++i) {
      char L = (char)('X' + i);
      if (has[L - 'A'])  offsetMm[i] = mc[i] - val[L - 'A'] * u;
      else if (!anyAxis) offsetMm[i] = mc[i];
    }
  }

  // ---- motion ----
  const bool hasXYZ = has['X' - 'A'] || has['Y' - 'A'] || has['Z' - 'A'];
  if (motion < 0 && hasXYZ) motion = modalMotion;
  if (motion >= 0) modalMotion = motion;
  if (faultActive && (hasXYZ || doG4)) return ERR_FAULT;

  uint8_t merr = ERR_NONE;
  if (motion == 0 || motion == 1) {
    if (!homed) return ERR_NOT_HOMED;
    float target[3] = { posMm[0], posMm[1], posMm[2] };
    for (uint8_t i = 0; i < 3; ++i) {
      char L = (char)('X' + i);
      if (has[L - 'A']) {
        float v = val[L - 'A'] * u;
        target[i] = absoluteMode ? v : posMm[i] + v;
      }
    }
    if (motion == 0) merr = planCartesian(target, 1.0e6f);
    else {
      if (feedMmMin <= 0) return ERR_FEED_UNDEFINED;
      merr = planCartesian(target, feedMmMin / 60.0f);
    }
    if (merr) return merr;
  } else if (motion == 2 || motion == 3) {
    if (!homed) return ERR_NOT_HOMED;
    if (plane != 17) return ERR_BAD_PLANE;
    if (feedMmMin <= 0) return ERR_FEED_UNDEFINED;
    bool hasIJ = has['I' - 'A'] || has['J' - 'A'], hasR = has['R' - 'A'];
    if (!hasIJ && !hasR) return ERR_VALUE_MISSING;
    float target[3] = { posMm[0], posMm[1], posMm[2] };
    for (uint8_t i = 0; i < 3; ++i) {
      char L = (char)('X' + i);
      if (has[L - 'A']) {
        float v = val[L - 'A'] * u;
        target[i] = absoluteMode ? v : posMm[i] + v;
      }
    }
    const float PI_F = 3.14159265359f, TWO_PI = 6.28318530718f;
    float sx = posMm[0], sy = posMm[1], cx, cy, r;
    if (hasIJ) {
      cx = sx + (has['I'-'A'] ? val['I'-'A']*u : 0);
      cy = sy + (has['J'-'A'] ? val['J'-'A']*u : 0);
      float r0 = hypotf(sx-cx, sy-cy), r1 = hypotf(target[0]-cx, target[1]-cy);
      if (r0 < 1e-4f) return ERR_ARC_RADIUS;
      if (fabsf(r0-r1) > 0.5f) return ERR_ARC_ENDPOINT;
      r = 0.5f * (r0 + r1);
    } else {
      float rv = val['R'-'A'] * u;
      r = fabsf(rv);
      float dx = target[0]-sx, dy = target[1]-sy, d = hypotf(dx, dy);
      if (r < 1e-4f || d < 1e-4f || d > 2*r) return ERR_ARC_RADIUS;
      float h = sqrtf(r*r - 0.25f*d*d);
      float sign = (motion == 2) ? -1.0f : 1.0f;
      if (rv < 0) sign = -sign;
      cx = 0.5f*(sx+target[0]) + sign*h*(-dy/d);
      cy = 0.5f*(sy+target[1]) + sign*h*( dx/d);
    }
    float a0 = atan2f(sy-cy, sx-cx), a1 = atan2f(target[1]-cy, target[0]-cx), travel;
    bool cw = (motion == 2);
    if (cw) {
      while (a1 >= a0) a1 -= TWO_PI;
      travel = a1 - a0;
      if (!hasIJ) {
        if (val['R'-'A'] > 0 && travel < -PI_F) travel += TWO_PI;
        if (val['R'-'A'] < 0 && travel > -PI_F) travel -= TWO_PI;
      }
    } else {
      while (a1 <= a0) a1 += TWO_PI;
      travel = a1 - a0;
      if (!hasIJ) {
        if (val['R'-'A'] > 0 && travel > PI_F) travel -= TWO_PI;
        if (val['R'-'A'] < 0 && travel < PI_F) travel += TWO_PI;
      }
    }
    uint16_t nSeg = (uint16_t)(fabsf(travel) * r / MOVE_SEG_MM);
    if (nSeg < 1) nSeg = 1; if (nSeg > 1024) nSeg = 1024;
    float z0 = posMm[2];
    for (uint16_t k = 1; k <= nSeg; ++k) {
      float t = (float)k / nSeg, pt[3];
      if (k == nSeg) { pt[0] = target[0]; pt[1] = target[1]; }
      else {
        float ang = a0 + travel * t;
        pt[0] = cx + r * cosf(ang); pt[1] = cy + r * sinf(ang);
      }
      pt[2] = z0 + (target[2] - z0) * t;
      merr = planCartesian(pt, feedMmMin / 60.0f);
      if (merr) return merr;
    }
    for (uint8_t i = 0; i < 3; ++i) posMm[i] = target[i];
  }

  if (doG4) {
    if (!has['P' - 'A']) return ERR_VALUE_MISSING;
    planDwell((uint32_t)(val['P' - 'A'] * 1000.0f));
  }

  // ---- M-codes ----
  for (uint8_t i = 0; i < nw; ++i) {
    if (words[i].c != 'M') continue;
    switch ((int)lroundf(words[i].v)) {
      case 2: case 30:
        waitIdle();
        pumpOffISR();
        feedMmMin = 0; modalMotion = 0; unitsInches = false; absoluteMode = true; plane = 17;
        break;
      case 3: case 4: case 8:
        flowPct = constrain(spindleS, 0.0f, 100.0f);
        digitalWrite(PIN_PUMP_EN, HIGH);
        pumpOn = true;
        break;
      case 5: case 9:
        pumpOffISR();
        break;
      case 114: reportPosition(); break;
      case 999:                                     // clear fault / e-stop
        if (digitalRead(PIN_ESTOP) == LOW) break;   // still tripped
        estopLatched = false; faultActive = false; faultCode = 0;
        noInterrupts();
        for (uint8_t i = 0; i < N_AXIS; ++i) planSteps[i] = machineSteps[i];
        interrupts();
        syncPosFromMachine();
        break;
      default: return ERR_UNSUPPORTED_M;
    }
  }
  return faultActive ? ERR_FAULT : ERR_NONE;
}

// ================================== Runtime ==================================

static void processLine(char *line) {
  uint8_t err = executeLine(line);
  if (err == ERR_NONE) Serial.println(F("ok"));
  else { Serial.print(F("error:")); Serial.println(err); }
}

// Flow command tracks actual nozzle speed vs commanded feed.
static void updateFlow() {
  static uint32_t lastMs = 0;
  static float lastPos[3];
  static bool init = false;
  uint32_t now = millis();
  if (now - lastMs < 100) return;
  float dt = (now - lastMs) / 1000.0f; lastMs = now;
  int32_t ms[N_AXIS];
  noInterrupts();
  for (uint8_t i = 0; i < N_AXIS; ++i) ms[i] = machineSteps[i];
  interrupts();
  float th = ms[AX_TH] / STEPS_PER_UNIT[AX_TH] * D2R;
  float r  = ms[AX_R]  / STEPS_PER_UNIT[AX_R];
  float p[3] = { r * cosf(th), r * sinf(th), ms[AX_Z] / STEPS_PER_UNIT[AX_Z] };
  uint8_t pwm = 0;
  if (init && pumpOn && !faultActive) {
    float speed = sqrtf(sq(p[0]-lastPos[0]) + sq(p[1]-lastPos[1]) + sq(p[2]-lastPos[2])) / dt;
    float cmd = feedMmMin / 60.0f; if (cmd < 0.1f) cmd = 0.1f;
    float ratio = speed / cmd; if (ratio > 1.0f) ratio = 1.0f;
    pwm = (uint8_t)(flowPct * 2.55f * ratio);
  }
  analogWrite(PIN_FLOW_PWM, pwm);
  for (uint8_t i = 0; i < 3; ++i) lastPos[i] = p[i];
  init = true;
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < N_AXIS; ++i) {
    pinMode(PIN_STEP[i], OUTPUT); pinMode(PIN_DIR[i], OUTPUT);
    if (PIN_ENDSTOP[i] != 255) pinMode(PIN_ENDSTOP[i], INPUT_PULLUP);
  }
  pinMode(PIN_ENABLE, OUTPUT); pinMode(PIN_ESTOP, INPUT_PULLUP);
  pinMode(PIN_PUMP_EN, OUTPUT); pinMode(PIN_FLOW_PWM, OUTPUT);
  digitalWrite(PIN_ENABLE, LOW);              // hold motors energized
  digitalWrite(PIN_PUMP_EN, LOW);

  noInterrupts();
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS10);
  OCR1A = F_CPU / ISR_HZ - 1;
  TIMSK1 = _BV(OCIE1A);
  interrupts();

  Serial.println(F("[cyl-gcode] ready — UNHOMED (send G28 first)"));
}

void loop() {
  static char buf[128];
  static uint8_t len = 0;
  static uint8_t lastAlarm = 255;

  if (faultActive && faultCode != lastAlarm) {
    lastAlarm = faultCode;
    Serial.print(F("ALARM:")); Serial.println(faultCode);  // 1=e-stop 2=endstop
  }
  if (!faultActive) lastAlarm = 255;

  updateFlow();

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '!') { holdActive = true;  Serial.println(F("[hold]"));   continue; }
    if (c == '~') { holdActive = false; Serial.println(F("[resume]")); continue; }
    if (c == '\n' || c == '\r') {
      if (len) { buf[len] = '\0'; processLine(buf); len = 0; }
    } else if (len < sizeof(buf) - 1) buf[len++] = c;
    else { len = 0; Serial.println(F("error:line too long")); }
  }
}
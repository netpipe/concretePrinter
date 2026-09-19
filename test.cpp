/*
 * ============================================================================
 *  gcode.ino — single-file G-code interpreter + motion controller
 *  Target: Arduino Uno / Nano (ATmega328P, 16 MHz)
 * ============================================================================
 *
 *  G-codes : G0 G1 G2 G3 G4 G17 G20 G21 G28 G90 G91 G92
 *  M-codes : M2 M3 M4 M5 M8 M9 M30 M114
 *
 *  Protocol: 115200 baud, line-oriented. Replies "ok" per line,
 *            or "error:<n>". Works with most line-by-line G-code senders.
 *
 *  Motion  : Timer1 @ 20 kHz ISR, integer Bresenham stepping,
 *            trapezoidal velocity profile per segment, 1-segment lookahead.
 *            NOTE: uses Timer1 — don't combine with the Servo library.
 *
 *  Wiring (3x STEP/DIR drivers: A4988 / DRV8825 / TMC2209 ...):
 *    D2,D3,D4 -> X,Y,Z STEP        D9  -> spindle (PWM, M3/M4/M5, S0..1000)
 *    D5,D6,D7 -> X,Y,Z DIR         D10 -> coolant (M8/M9)
 *    D8       -> all EN (active LOW)
 */

#include <Arduino.h>
#include <math.h>

#if !defined(__AVR__)
#error "This sketch uses AVR Timer1 registers; port the ISR/timer for other MCUs."
#endif

// ================================ Configuration ==============================

static const uint8_t PIN_X_STEP  = 2;
static const uint8_t PIN_X_DIR   = 5;
static const uint8_t PIN_Y_STEP  = 3;
static const uint8_t PIN_Y_DIR   = 6;
static const uint8_t PIN_Z_STEP  = 4;
static const uint8_t PIN_Z_DIR   = 7;
static const uint8_t PIN_ENABLE  = 8;    // active LOW
static const uint8_t PIN_SPINDLE = 9;    // PWM capable
static const uint8_t PIN_COOLANT = 10;

static const float STEPS_PER_MM[3]  = { 80.0f, 80.0f, 400.0f };  // X Y Z
static const float MAX_RATE_MM_S[3] = { 60.0f, 60.0f,  12.0f };  // X Y Z
static const float ACCEL_MM_S2      = 150.0f;                    // global accel

static const float    ARC_MM_PER_SEGMENT = 0.5f;   // arc subdivision chord (mm)
static const uint32_t ISR_HZ             = 20000UL; // stepper tick rate
static const uint16_t STEP_PULSE_US      = 3;       // pulse width

enum { X = 0, Y = 1, Z = 2, N_AXIS = 3 };

// Move queue (size MUST be a power of two)
static const uint8_t QUEUE_SIZE = 16;
static const uint8_t QUEUE_MASK = QUEUE_SIZE - 1;

// Error codes
enum {
  ERR_NONE = 0,
  ERR_BAD_CHAR, ERR_BAD_NUMBER, ERR_UNSUPPORTED_G, ERR_UNSUPPORTED_M,
  ERR_FEED_UNDEFINED, ERR_ARC_RADIUS, ERR_ARC_ENDPOINT, ERR_VALUE_MISSING,
  ERR_CHECKSUM, ERR_BAD_PLANE,
};

// ================================ Motion state ===============================

struct Segment {
  int32_t  steps[N_AXIS];     // abs step count per axis
  uint32_t totalSteps;        // = max axis steps (0 => dwell segment)
  uint8_t  dirMask;           // bit i set => axis i moves negative
  uint32_t entryRate;         // steps/s on major axis at segment start
  uint32_t cruiseRate;
  uint32_t exitRate;
  uint32_t accelStepsPerS2;
  uint32_t accelUntil;        // step index where accel phase ends
  uint32_t decelAfter;        // step index where decel phase begins
  uint32_t dwellMs;
  // mm-domain planning data (for junction patching)
  float    lengthMm, entryMmS, cruiseMmS;
};

static Segment queue[QUEUE_SIZE];
static volatile uint8_t qHead = 0;        // next slot to write (planner)
static volatile uint8_t qTail = 0;        // next slot to claim (ISR)
static volatile bool    busy  = false;    // ISR holds an active segment/dwell

static volatile int32_t machineSteps[N_AXIS] = { 0, 0, 0 }; // executed position
static int32_t          planSteps[N_AXIS]    = { 0, 0, 0 }; // end of queue

// Interpreter (modal) state
static bool    unitsInches = false;
static bool    absoluteMode = true;
static uint8_t plane = 17;
static float   feedMmMin = 0.0f;
static float   spindleS = 0.0f;
static int8_t  modalMotion = 0;           // 0=G0 1=G1 2=G2 3=G3
static float   offsetMm[N_AXIS] = { 0, 0, 0 };   // G92 work offset
static float   posMm[N_AXIS]    = { 0, 0, 0 };   // last commanded work pos

#define COMPILER_BARRIER() asm volatile("" ::: "memory")

static inline uint8_t nextIdx(uint8_t i) { return (uint8_t)((i + 1) & QUEUE_MASK); }
static inline bool queueFull()  { return nextIdx(qHead) == qTail; }

// Compute step-domain velocity profile from mm-domain values.
// Entry speed is s.entryMmS, exit speed is exitMmS, requested cruise s.cruiseMmS.
static void finalizeSegment(Segment &s, float exitMmS) {
  const float a = ACCEL_MM_S2;
  float e = s.entryMmS, x = exitMmS, c = s.cruiseMmS;
  if (c < e) c = e;
  if (c < x) c = x;
  // Peak speed limited by available distance (triangular profile if needed)
  float c2max = a * s.lengthMm + 0.5f * (e * e + x * x);
  if (c * c > c2max) c = sqrtf(c2max);

  float accDist = (c * c - e * e) / (2.0f * a);
  float decDist = (c * c - x * x) / (2.0f * a);
  if (accDist < 0) accDist = 0;
  if (decDist < 0) decDist = 0;
  float sum = accDist + decDist;
  if (sum > s.lengthMm && sum > 0) {           // rounding guard
    float k = s.lengthMm / sum;
    accDist *= k; decDist *= k;
  }

  float spm = (float)s.totalSteps / s.lengthMm; // major-axis steps per mm
  s.entryRate       = (uint32_t)(e * spm);
  s.cruiseRate      = (uint32_t)(c * spm);
  s.exitRate        = (uint32_t)(x * spm);
  s.accelStepsPerS2 = (uint32_t)(a * spm);
  if (s.accelStepsPerS2 < 1) s.accelStepsPerS2 = 1;
  if (s.cruiseRate > ISR_HZ - 1000) s.cruiseRate = ISR_HZ - 1000;
  if (s.entryRate > s.cruiseRate) s.entryRate = s.cruiseRate;
  if (s.exitRate  > s.cruiseRate) s.exitRate  = s.cruiseRate;
  if (s.cruiseRate < 1) s.cruiseRate = 1;

  s.accelUntil = (uint32_t)(accDist * spm);
  uint32_t decSteps = (uint32_t)(decDist * spm);
  s.decelAfter = (s.totalSteps > decSteps) ? s.totalSteps - decSteps : 0;
  if (s.decelAfter < s.accelUntil) s.decelAfter = s.accelUntil;
}

static void planDwell(uint32_t ms) {
  while (queueFull()) { }
  Segment &s = queue[qHead];
  s.totalSteps = 0;
  s.dwellMs = ms;
  COMPILER_BARRIER();
  qHead = nextIdx(qHead);
}

// Core planner: machine-coordinate linear move.
static void planLineToMachine(const int32_t target[N_AXIS], float feedMmS) {
  int32_t  delta[N_AXIS];
  uint8_t  dirMask = 0;
  uint32_t total = 0;
  float lenSq = 0;

  for (uint8_t i = 0; i < N_AXIS; ++i) {
    delta[i] = target[i] - planSteps[i];
    if (delta[i] < 0) dirMask |= (uint8_t)(1 << i);
    uint32_t a = (uint32_t)labs(delta[i]);
    if (a > total) total = a;
    float mm = (float)delta[i] / STEPS_PER_MM[i];
    lenSq += mm * mm;
  }
  if (total == 0) return;
  float L = sqrtf(lenSq);

  // Limit feed by each axis's max rate
  for (uint8_t i = 0; i < N_AXIS; ++i) {
    uint32_t a = (uint32_t)labs(delta[i]);
    if (!a) continue;
    float fmax = MAX_RATE_MM_S[i] * STEPS_PER_MM[i] * L / (float)a;
    if (feedMmS > fmax) feedMmS = fmax;
  }
  if (feedMmS < 0.5f) feedMmS = 0.5f;

  while (queueFull()) { }                    // ISR drains the queue

  uint8_t idx = qHead;
  Segment &s = queue[idx];
  s.dwellMs = 0;
  s.totalSteps = total;
  s.dirMask = dirMask;
  s.lengthMm = L;
  s.cruiseMmS = feedMmS;
  s.entryMmS = 0;
  for (uint8_t i = 0; i < N_AXIS; ++i) s.steps[i] = labs(delta[i]);

  // One-segment lookahead: patch junction speed with previous pending move
  if (qHead != qTail) {
    uint8_t prevIdx = (uint8_t)((idx + QUEUE_SIZE - 1) & QUEUE_MASK);
    if (qTail != nextIdx(prevIdx)) {         // previous not yet claimed by ISR
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
          uint8_t t = qTail;
          COMPILER_BARRIER();
          if (t != nextIdx(prevIdx)) queue[prevIdx] = tmp;
          interrupts();
          s.entryMmS = J;
        }
      }
    }
  }

  finalizeSegment(s, 0.0f);                  // queue currently ends at rest
  for (uint8_t i = 0; i < N_AXIS; ++i) planSteps[i] = target[i];
  COMPILER_BARRIER();
  qHead = nextIdx(idx);
}

static void planLineToWork(const float target[N_AXIS], float feedMmS) {
  int32_t ts[N_AXIS];
  for (uint8_t i = 0; i < N_AXIS; ++i)
    ts[i] = lroundf((target[i] + offsetMm[i]) * STEPS_PER_MM[i]);
  planLineToMachine(ts, feedMmS);
}

// G2/G3 arc in XY plane, subdivided into short linear segments (helix for Z).
static uint8_t planArc(bool cw, bool hasIJ, float iOff, float jOff, float rVal,
                       const float target[N_AXIS], float feedMmS) {
  const float PI_F = 3.14159265359f, TWO_PI = 6.28318530718f;
  float sx = posMm[X], sy = posMm[Y];
  float cx, cy, r;

  if (hasIJ) {                               // I/J are always incremental
    cx = sx + iOff; cy = sy + jOff;
    float r0 = hypotf(sx - cx, sy - cy);
    float r1 = hypotf(target[X] - cx, target[Y] - cy);
    if (r0 < 1e-4f) return ERR_ARC_RADIUS;
    if (fabsf(r0 - r1) > 0.5f) return ERR_ARC_ENDPOINT;
    r = 0.5f * (r0 + r1);
  } else {                                   // R mode
    r = fabsf(rVal);
    if (r < 1e-4f) return ERR_ARC_RADIUS;
    float dx = target[X] - sx, dy = target[Y] - sy;
    float d = hypotf(dx, dy);
    if (d < 1e-4f || d > 2.0f * r) return ERR_ARC_RADIUS;
    float h = sqrtf(r * r - 0.25f * d * d);
    float sign = cw ? -1.0f : 1.0f;
    if (rVal < 0) sign = -sign;              // negative R => major arc
    cx = 0.5f * (sx + target[X]) + sign * h * (-dy / d);
    cy = 0.5f * (sy + target[Y]) + sign * h * ( dx / d);
  }

  float a0 = atan2f(sy - cy, sx - cx);
  float a1 = atan2f(target[Y] - cy, target[X] - cx);
  float travel;
  if (cw) {
    while (a1 >= a0) a1 -= TWO_PI;
    travel = a1 - a0;                        // (-2pi, 0)
    if (!hasIJ) {
      if (rVal > 0 && travel < -PI_F) travel += TWO_PI;
      if (rVal < 0 && travel > -PI_F) travel -= TWO_PI;
    }
  } else {
    while (a1 <= a0) a1 += TWO_PI;
    travel = a1 - a0;                        // (0, 2pi)
    if (!hasIJ) {
      if (rVal > 0 && travel > PI_F) travel -= TWO_PI;
      if (rVal < 0 && travel < PI_F) travel += TWO_PI;
    }
  }

  float arcLen = fabsf(travel) * r;
  uint16_t nSeg = (uint16_t)(arcLen / ARC_MM_PER_SEGMENT);
  if (nSeg < 1) nSeg = 1;
  if (nSeg > 512) nSeg = 512;

  float z0 = posMm[Z];
  for (uint16_t k = 1; k <= nSeg; ++k) {
    float t = (float)k / (float)nSeg;
    float pt[N_AXIS];
    if (k == nSeg) { pt[X] = target[X]; pt[Y] = target[Y]; }
    else {
      float ang = a0 + travel * t;
      pt[X] = cx + r * cosf(ang);
      pt[Y] = cy + r * sinf(ang);
    }
    pt[Z] = z0 + (target[Z] - z0) * t;
    planLineToWork(pt, feedMmS);
  }
  for (uint8_t i = 0; i < N_AXIS; ++i) posMm[i] = target[i];
  return ERR_NONE;
}

// ============================ Utilities / reports ============================

static void waitIdle() { while (busy || qHead != qTail) { } }

static void reportPosition() {
  int32_t m[N_AXIS];
  noInterrupts();
  for (uint8_t i = 0; i < N_AXIS; ++i) m[i] = machineSteps[i];
  interrupts();
  Serial.print(F("X:")); Serial.print(m[X] / STEPS_PER_MM[X] - offsetMm[X], 3);
  Serial.print(F(" Y:")); Serial.print(m[Y] / STEPS_PER_MM[Y] - offsetMm[Y], 3);
  Serial.print(F(" Z:")); Serial.println(m[Z] / STEPS_PER_MM[Z] - offsetMm[Z], 3);
}

static void resetModal() {
  unitsInches = false; absoluteMode = true; plane = 17;
  feedMmMin = 0; modalMotion = 0; spindleS = 0;
}

// ================================ Stepper ISR ================================

ISR(TIMER1_COMPA_vect) {
  static Segment *seg = NULL;
  static uint32_t stepIdx = 0, rate = 0, frac = 0, dwellTicks = 0;
  static int32_t  bres[N_AXIS];

  if (dwellTicks) {                          // dwell in progress
    if (--dwellTicks == 0) busy = false;
    return;
  }

  if (!seg) {                                // claim next segment
    if (qHead == qTail) return;              // idle
    uint8_t t = qTail;
    COMPILER_BARRIER();
    seg = &queue[t];
    qTail = nextIdx(t);
    busy = true;
    if (seg->totalSteps == 0) {              // dwell segment
      dwellTicks = seg->dwellMs * (ISR_HZ / 1000UL);
      seg = NULL;
      if (!dwellTicks) busy = false;
      return;
    }
    digitalWrite(PIN_X_DIR, (seg->dirMask & 1) ? HIGH : LOW);
    digitalWrite(PIN_Y_DIR, (seg->dirMask & 2) ? HIGH : LOW);
    digitalWrite(PIN_Z_DIR, (seg->dirMask & 4) ? HIGH : LOW);
    stepIdx = 0; frac = 0;
    rate = (seg->entryRate > 64) ? seg->entryRate : 64;
    for (uint8_t i = 0; i < N_AXIS; ++i) bres[i] = (int32_t)(seg->totalSteps >> 1);
    return;                                  // first pulses on the next tick
  }

  // Integrate current rate (steps/s) into a fractional step accumulator.
  frac += rate;
  while (frac >= ISR_HZ) {
    frac -= ISR_HZ;

    // Bresenham across all axes
    uint8_t pulsed = 0;
    for (uint8_t i = 0; i < N_AXIS; ++i) {
      bres[i] += seg->steps[i];
      if (bres[i] >= (int32_t)seg->totalSteps) {
        bres[i] -= (int32_t)seg->totalSteps;
        pulsed |= (uint8_t)(1 << i);
      }
    }
    if (pulsed & 1) digitalWrite(PIN_X_STEP, HIGH);
    if (pulsed & 2) digitalWrite(PIN_Y_STEP, HIGH);
    if (pulsed & 4) digitalWrite(PIN_Z_STEP, HIGH);
    if (pulsed) {
      delayMicroseconds(STEP_PULSE_US);
      if (pulsed & 1) { digitalWrite(PIN_X_STEP, LOW); machineSteps[X] += (seg->dirMask & 1) ? -1 : 1; }
      if (pulsed & 2) { digitalWrite(PIN_Y_STEP, LOW); machineSteps[Y] += (seg->dirMask & 2) ? -1 : 1; }
      if (pulsed & 4) { digitalWrite(PIN_Z_STEP, LOW); machineSteps[Z] += (seg->dirMask & 4) ? -1 : 1; }
    }

    if (++stepIdx >= seg->totalSteps) { seg = NULL; busy = false; return; }

    // Trapezoidal profile: v*dv = a*ds  =>  dv ~= a/v per step
    if (stepIdx < seg->accelUntil) {
      uint32_t d = seg->accelStepsPerS2 / rate;
      rate += d ? d : 1;
      if (rate > seg->cruiseRate) rate = seg->cruiseRate;
    } else if (stepIdx >= seg->decelAfter) {
      if (rate > seg->exitRate) {
        uint32_t d = seg->accelStepsPerS2 / rate;
        if (!d) d = 1;
        uint32_t nr = (rate > d) ? rate - d : 0;
        uint32_t floor = (seg->exitRate > 32) ? seg->exitRate : 32; // never stall
        rate = (nr < floor) ? floor : nr;
      }
    }
  }
}

// ============================== Line interpreter =============================

struct Word { char c; float v; };

static uint8_t executeLine(char *line) {
  // --- strip comments, uppercase, compact ---
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

  // --- optional checksum (RepRap-style XOR) ---
  if (char *star = strchr(p, '*')) {
    uint8_t cs = 0;
    for (char *q = p; q < star; ++q) cs ^= (uint8_t)*q;
    if (atoi(star + 1) != (int)cs) return ERR_CHECKSUM;
    *star = '\0';
  }

  // --- optional line number ---
  if (*p == 'N') { ++p; while (*p >= '0' && *p <= '9') ++p; }

  // --- tokenize words ---
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
  static bool  has[26];
  memset(val, 0, sizeof(val));
  memset(has, 0, sizeof(has));
  for (uint8_t i = 0; i < nw; ++i) {
    uint8_t k = (uint8_t)(words[i].c - 'A');
    val[k] = words[i].v;
    has[k] = true;
  }

  const float u = unitsInches ? 25.4f : 1.0f;

  // --- non-motion G-codes, in order of appearance ---
  int8_t motion = -1;
  bool doG4 = false, doG28 = false, doG92 = false;
  for (uint8_t i = 0; i < nw; ++i) {
    if (words[i].c != 'G') continue;
    switch ((int)lroundf(words[i].v)) {
      case 0: case 1: case 2: case 3: motion = (int8_t)lroundf(words[i].v); break;
      case 4:  doG4 = true; break;
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
  if (has['S' - 'A']) spindleS  = val['S' - 'A'];

  // --- G92 work offset ---
  if (doG92) {
    bool anyAxis = has['X' - 'A'] || has['Y' - 'A'] || has['Z' - 'A'];
    for (uint8_t i = 0; i < N_AXIS; ++i) {
      char L = (char)('X' + i);
      float machineMm = (float)planSteps[i] / STEPS_PER_MM[i];
      if (has[L - 'A'])      offsetMm[i] = machineMm - val[L - 'A'] * u;
      else if (!anyAxis)     offsetMm[i] = machineMm;   // G92 with no words -> zero here
    }
  }

  // --- motion (explicit G or modal) ---
  const bool hasXYZ = has['X' - 'A'] || has['Y' - 'A'] || has['Z' - 'A'];
  if (motion < 0 && hasXYZ) motion = modalMotion;

  float target[N_AXIS];
  for (uint8_t i = 0; i < N_AXIS; ++i) target[i] = posMm[i];
  for (uint8_t i = 0; i < N_AXIS; ++i) {
    char L = (char)('X' + i);
    if (has[L - 'A']) {
      float v = val[L - 'A'] * u;
      target[i] = absoluteMode ? v : posMm[i] + v;
    }
  }
  if (motion >= 0) modalMotion = motion;

  if (doG28) {                                       // rapid to machine origin
    const int32_t home[N_AXIS] = { 0, 0, 0 };
    planLineToMachine(home, 1.0e6f);
    for (uint8_t i = 0; i < N_AXIS; ++i) posMm[i] = -offsetMm[i];
  } else if (motion == 0) {
    planLineToWork(target, 1.0e6f);
    for (uint8_t i = 0; i < N_AXIS; ++i) posMm[i] = target[i];
  } else if (motion == 1) {
    if (feedMmMin <= 0) return ERR_FEED_UNDEFINED;
    planLineToWork(target, feedMmMin / 60.0f);
    for (uint8_t i = 0; i < N_AXIS; ++i) posMm[i] = target[i];
  } else if (motion == 2 || motion == 3) {
    if (plane != 17) return ERR_BAD_PLANE;
    if (feedMmMin <= 0) return ERR_FEED_UNDEFINED;
    bool hasIJ = has['I' - 'A'] || has['J' - 'A'];
    bool hasR  = has['R' - 'A'];
    if (!hasIJ && !hasR) return ERR_VALUE_MISSING;
    uint8_t err = planArc(motion == 2, hasIJ,
                          has['I' - 'A'] ? val['I' - 'A'] * u : 0.0f,
                          has['J' - 'A'] ? val['J' - 'A'] * u : 0.0f,
                          hasR ? val['R' - 'A'] * u : 0.0f,
                          target, feedMmMin / 60.0f);
    if (err != ERR_NONE) return err;
  }

  if (doG4) {
    if (!has['P' - 'A']) return ERR_VALUE_MISSING;
    planDwell((uint32_t)(val['P' - 'A'] * 1000.0f));
  }

  // --- M-codes ---
  for (uint8_t i = 0; i < nw; ++i) {
    if (words[i].c != 'M') continue;
    switch ((int)lroundf(words[i].v)) {
      case 2: case 30:
        waitIdle();
        digitalWrite(PIN_SPINDLE, LOW);
        digitalWrite(PIN_COOLANT, LOW);
        resetModal();
        break;
      case 3: case 4: {
        int pwm = (spindleS > 0)
                ? constrain((int)(spindleS * 255.0f / 1000.0f), 1, 255) : 255;
        analogWrite(PIN_SPINDLE, (uint8_t)pwm);     // ~490 Hz PWM on D9
        break;
      }
      case 5:  digitalWrite(PIN_SPINDLE, LOW); break;
      case 8:  digitalWrite(PIN_COOLANT, HIGH); break;
      case 9:  digitalWrite(PIN_COOLANT, LOW); break;
      case 114: reportPosition(); break;
      default: return ERR_UNSUPPORTED_M;
    }
  }
  return ERR_NONE;
}

// ================================== Runtime ==================================

static void processLine(char *line) {
  uint8_t err = executeLine(line);
  if (err == ERR_NONE) Serial.println(F("ok"));
  else { Serial.print(F("error:")); Serial.println(err); }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_X_STEP, OUTPUT); pinMode(PIN_X_DIR, OUTPUT);
  pinMode(PIN_Y_STEP, OUTPUT); pinMode(PIN_Y_DIR, OUTPUT);
  pinMode(PIN_Z_STEP, OUTPUT); pinMode(PIN_Z_DIR, OUTPUT);
  pinMode(PIN_ENABLE, OUTPUT); pinMode(PIN_SPINDLE, OUTPUT); pinMode(PIN_COOLANT, OUTPUT);
  digitalWrite(PIN_ENABLE, LOW);            // enable drivers (active LOW)

  // Timer1: CTC, no prescaler -> ISR at ISR_HZ
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS10);
  OCR1A  = F_CPU / ISR_HZ - 1;
  TIMSK1 = _BV(OCIE1A);
  interrupts();

  Serial.println(F("[gcode] ready — G0 G1 G2 G3 G4 G17 G20 G21 G28 G90 G91 G92"));
  Serial.println(F("                 M2 M3 M4 M5 M8 M9 M30 M114"));
}

void loop() {
  static char buf[128];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (len) { buf[len] = '\0'; processLine(buf); len = 0; }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      len = 0;
      Serial.println(F("error:1 (line too long, flushed)"));
    }
  }
}
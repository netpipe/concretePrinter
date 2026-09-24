/*
 * ============================================================================
 *  gcode_boom.ino — G-code controller for a knuckle-boom picker truck
 *  retrofitted with clamp-on stepper lever actuators + joint encoders,
 *  prepared for a concrete extrusion end at the hook point.
 *
 *  Board   : Arduino Mega recommended (pin count). AVR Timer1, 5 kHz ISR.
 *  Inputs  : Serial 115200 (G-code), 4x quadrature joint encoders (hall pairs ok)
 *  Outputs : 4x STEP/DIR lever actuators, pump contactor, flow PWM
 *
 *  G-codes : G0 G1 G2 G3 G4 G17 G20 G21 G28 G90 G91 G92
 *            + joint words A B C D (degrees) for direct joint moves/jogging
 *  M-codes : M2 M3 M4 M5 M8 M9 M17 M18 M30 M114 M999
 *  Realtime: '!' feed hold, '~' resume. E-stop pin latches until M999.
 *
 *  Pipeline:  XYZ line -> subdivided -> inverse kinematics -> joint segments
 *             (trapezoid velocity profile) -> ISR interpolates joint targets
 *             -> closed-loop stepping pulls the hydraulic levers.
 *
 *  HOMING: jog each joint onto its mechanical reference with G91 A/B/C/D
 *  moves, then send G28 to record that position as HOME_ANGLE[].
 *  Cartesian moves are refused until homed.
 
 * for automatically bringing the boom in and out while printing
 */

#include <Arduino.h>
#include <math.h>
#define strtof(A, B) strtod(A, B)
#include <stdlib.h>
#include <stdio.h>

#if !defined(__AVR__)
#error "AVR Timer1 specific; port the ISR for other MCUs."
#endif

#define COMPILER_BARRIER() asm volatile("" ::: "memory")
#define D2R 0.01745329252f
#define R2D 57.29577951f

// ============================ MACHINE GEOMETRY ===============================
//  *** MEASURE YOUR MACHINE. Every value below is a placeholder. ***
//  Work origin: slew axis, truck frame deck level. Units: mm.

// ============================ MACHINE GEOMETRY ===============================
static const float PIVOT_HEIGHT_MM = 1600;   // boom pivot height above origin
static const float L1_MM = 9100;             // main boom pin-to-pin
static const float L2_BASE_MM = 7300;        // outer boom base pin-to-pin (before extension)
static const float L3_MM = 1100;             // knuckle pin -> nozzle tip (hangs straight down)
static const float ELBOW_SIGN = +1;          // +1/-1 selects elbow branch

// Changed JT_WRIST to JT_EXT
enum { JT_SLEW = 0, JT_BOOM, JT_KNUCKLE, JT_EXT, N_JOINTS = 4 };

// Limits: 0-2 are degrees, Joint 3 is mm (set these to your actual cylinder stroke)
static const float JMIN[4] = { -175, -10, -170,    300 };  // 0mm extension
static const float JMAX[4] = {  175,  82,  165, 5000 };  // 5000mm max extension
static const float HOME_POS[4] = { 0, 0, 0, 0 }; 

static const float HOME_ANGLE[4] = { 0, 0, 0, 0 };       // angles at home position

static const float MAX_RADIUS_MM = 16000;    // reach envelope
static const float MIN_RADIUS_MM = 1500;
static const float MAX_HEIGHT_MM = 19000;
static const float MIN_HEIGHT_MM = 200;

// ============================== MOTION LIMITS ================================

static const float JOINT_RATE_DPS[4] = { 6, 5, 8, 10 };  // max joint speed deg/s
static const float JOINT_ACCEL_DPS2  = 4;                // path accel deg/s^2
static const float FOLLOW_ERR_DEG    = 3.0;  // fault if |target-measured| > this
static const float FOLLOW_HOLD_DEG   = 1.0;  // pause trajectory above this lag
static const float DEADBAND_DEG      = 0.15; // no stepping inside this band
static const float MOVE_SEG_MM       = 20;   // Cartesian subdivision length
static const float HARD_MARGIN_DEG   = 2.0;  // hard-limit margin past soft limit
static const uint32_t ISR_HZ         = 5000UL;

// =============================== PINS (Mega) =================================

static const uint8_t PIN_STEP[4]   = { 2, 3, 4, 5 };
static const uint8_t PIN_DIR[4]    = { 6, 7, 8, 9 };
static const uint8_t PIN_ENC_A[4]  = { 18, 20, 22, 24 };
static const uint8_t PIN_ENC_B[4]  = { 19, 21, 23, 25 };
static const uint8_t PIN_ENABLE    = 40;   // active LOW, all actuators
static const uint8_t PIN_ESTOP     = 41;   // NC contact to GND (pull-up)
static const uint8_t PIN_PUMP_EN   = 42;   // pump contactor
static const uint8_t PIN_FLOW_PWM  = 44;   // flow command (via RC/0-10V stage)

// Per-joint actuation/sensing calibration
static const float COUNTS_PER_DEG[4]   = { 25, 40, 40, 20 }; // joint encoder resolution
//static const int8_t ENC_SIGN[4]        = { 1, 1, 1, 1 };      // flip to match direction
//static const uint16_t STEP_RATE_LIMIT[4] = { 2000, 2000, 2000, 1200 }; // steps/s




// Per-joint resolution. 0-2 are counts/deg, Joint 3 is counts/mm
static const float COUNTS_PER_UNIT[4] = { 25, 40, 40, 100 }; // UPDATE: steps per mm for extension!
static const int8_t ENC_SIGN[4]        = { 1, 1, 1, 1 };
static const uint16_t STEP_RATE_LIMIT[4] = { 2000, 2000, 2000, 5000 }; // max steps/s

// Motion limits: 0-2 are deg/s, Joint 3 is mm/s
static const float JOINT_RATE_UNITS[4]   = { 30.0f, 30.0f, 30.0f, 100.0f }; 
static const float JOINT_ACCEL_UNITS2[4] = { 60.0f, 60.0f, 60.0f, 200.0f }; 




// ================================== STATE ====================================

struct Segment {
  int32_t  startMd[N_JOINTS];      // joint millidegrees at start
  int32_t  deltaMd[N_JOINTS];
  uint32_t majorMd;                // max |deltaMd| (path units)
  uint32_t dwellMs;                // majorMd==0 && dwellMs>0 => dwell
  float    nozzleLenMm, feedMmS;
  uint32_t entryRate, cruiseRate, exitRate;  // mdeg/s
  uint32_t accelMdS2;
  float    accelUntilU, decelAfterU;         // in u (0..65536)
  float    lengthMd, entryMdS, cruiseMdS;    // junction-patch metadata
  uint8_t  dirMask;
};

static const uint8_t QUEUE_SIZE = 16, QUEUE_MASK = QUEUE_SIZE - 1;
static Segment queue[QUEUE_SIZE];
static volatile uint8_t qHead = 0, qTail = 0;
static volatile bool busy = false;

static volatile int32_t encCounts[N_JOINTS];
static volatile int32_t measMd[N_JOINTS];     // measured joint angles (mdeg)
static volatile int32_t targetMd[N_JOINTS];
static volatile int32_t homeCounts[N_JOINTS];
static volatile int32_t homeMd[N_JOINTS];
static int32_t chainMd[N_JOINTS];             // planner chain end

static volatile bool faultActive = false, estopLatched = false;
static volatile uint8_t faultCode = 0;
static volatile bool holdActive = false, flushReq = false, resyncTargets = false;
static volatile bool drivesActive = false;
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

// Precomputed in setup()
static float mdPerCount[N_JOINTS];
static int32_t deadbandCounts[N_JOINTS];
static int32_t jminMdHard[N_JOINTS], jmaxMdHard[N_JOINTS];

enum { FAULT_ESTOP = 1, FAULT_FOLLOW = 2, FAULT_LIMIT = 3 };
enum {
  ERR_NONE = 0, ERR_BAD_CHAR, ERR_BAD_NUMBER, ERR_UNSUPPORTED_G, ERR_UNSUPPORTED_M,
  ERR_FEED_UNDEFINED, ERR_ARC_RADIUS, ERR_ARC_ENDPOINT, ERR_VALUE_MISSING,
  ERR_CHECKSUM, ERR_BAD_PLANE, ERR_UNREACHABLE, ERR_NOT_HOMED, ERR_FAULT,
  ERR_MIXED_WORDS, ERR_JOINT_LIMIT,
};


float startExt = chainMd[JT_EXT] / 1000.0f;
// ============================== KINEMATICS ===================================

// Inverse kinematics: nozzle XYZ (work origin at slew axis) -> joint degrees.
// Wrist solves so the nozzle absolute angle equals NOZZLE_PITCH_DEG.
// Inverse kinematics: nozzle XYZ -> joint angles (deg) + extension (mm)
// ext_mm holds the boom extension for this calculation
static bool ikSolve(const float p[3], float out[4], float ext_mm = 0.0f) {
  float X = p[0], Y = p[1], Z = p[2];
  float R = hypotf(X, Y);
  
  float slew = atan2f(Y, X) * R2D;
  float L2_curr = L2_BASE_MM + ext_mm; // Outer boom length + extension
  
  // Because the nozzle hangs straight down:
  // 1. The knuckle pin's horizontal distance (wx) is exactly R.
  // 2. The knuckle pin's vertical position (wz) is exactly L3_MM higher than the nozzle.
  float wx = R;                                   
  float wz = (Z - PIVOT_HEIGHT_MM) + L3_MM;       
  
  float D2 = wx * wx + wz * wz;
  float c2 = (D2 - L1_MM * L1_MM - L2_curr * L2_curr) / (2.0f * L1_MM * L2_curr);
  if (c2 < -1.0f || c2 > 1.0f) return false;              // out of reach
  
  float a2 = ELBOW_SIGN * acosf(c2);
  float a1 = atan2f(wz, wx) - atan2f(L2_curr * sinf(a2), L1_MM + L2_curr * cosf(a2));
  
  out[0] = slew; 
  out[1] = a1 * R2D; 
  out[2] = a2 * R2D; 
  out[3] = ext_mm; // Pass extension through
  return true;
}

static void fkSolve(const float jUnits[4], float p[3]) {
  float a1 = jUnits[1] * D2R;
  float a12 = a1 + jUnits[2] * D2R;
  float ext = jUnits[3];
  float L2_curr = L2_BASE_MM + ext;
  
  float s0 = jUnits[0] * D2R;
  
  // Knuckle pin position relative to pivot
  float R_knuckle = L1_MM * cosf(a1) + L2_curr * cosf(a12);
  float Z_knuckle = PIVOT_HEIGHT_MM + L1_MM * sinf(a1) + L2_curr * sinf(a12);
  
  // Nozzle hangs straight down from knuckle pin
  p[0] = R_knuckle * cosf(s0); 
  p[1] = R_knuckle * sinf(s0); 
  p[2] = Z_knuckle - L3_MM; 
}

static bool jointsInLimits(const float jd[4]) {
  for (uint8_t j = 0; j < N_JOINTS; ++j)
    if (jd[j] < JMIN[j] - 0.01f || jd[j] > JMAX[j] + 0.01f) return false;
  return true;
}
static bool ikSolveLimited(const float p[3], float ext_mm, float out[4]) {
  if (!ikSolve(p, out, ext_mm)) return false;
  return jointsInLimits(out);
}

static bool findNearestExtension(
  const float target[3],
  float currentExt,
  float &bestExt,
  float outJoints[4]
) {
  const float STEP_MM = 10.0f;
  const float MIN_EXT = JMIN[JT_EXT];
  const float MAX_EXT = JMAX[JT_EXT];

  // First try the current extension.
  if (ikSolveLimited(target, currentExt, outJoints)) {
    bestExt = currentExt;
    return true;
  }

  float bestDelta = 1e30f;
  bool found = false;

  // Search outward and inward from the current extension.
  for (float d = STEP_MM; d <= (MAX_EXT - MIN_EXT) + STEP_MM; d += STEP_MM) {
    float candidates[2];
    candidates[0] = currentExt + d;   // extend
    candidates[1] = currentExt - d;   // retract

    for (uint8_t i = 0; i < 2; ++i) {
      float e = candidates[i];

      if (e < MIN_EXT || e > MAX_EXT) continue;

      float jd[4];
      if (ikSolveLimited(target, e, jd)) {
        float delta = fabsf(e - currentExt);

        if (delta < bestDelta) {
          bestDelta = delta;
          bestExt = e;
          memcpy(outJoints, jd, sizeof(jd));
          found = true;
        }
      }
    }
  }

  return found;
}
// ================================ PLANNER ====================================

static inline uint8_t nextIdx(uint8_t i) { return (uint8_t)((i + 1) & QUEUE_MASK); }
static inline bool queueFull() { return nextIdx(qHead) == qTail; }

static void finalizeSegment(Segment &s, float exitMdS) {
  float a = (float)s.accelMdS2; if (a < 1) a = 1;
  float e = s.entryMdS, x = exitMdS, c = s.cruiseMdS;
  if (c < e) c = e;
  if (c < x) c = x;
  float c2max = a * s.lengthMd + 0.5f * (e * e + x * x);
  if (c * c > c2max) c = sqrtf(c2max);
  float accDist = (c * c - e * e) / (2.0f * a);
  float decDist = (c * c - x * x) / (2.0f * a);
  if (accDist < 0) accDist = 0;
  if (decDist < 0) decDist = 0;
  float sum = accDist + decDist;
  if (sum > s.lengthMd && sum > 0) {
    float k = s.lengthMd / sum; accDist *= k; decDist *= k;
  }
  s.entryRate = (uint32_t)e; s.cruiseRate = (uint32_t)c; s.exitRate = (uint32_t)x;
  if (s.cruiseRate < 1) s.cruiseRate = 1;
  if (s.entryRate > s.cruiseRate) s.entryRate = s.cruiseRate;
  if (s.exitRate > s.cruiseRate) s.exitRate = s.cruiseRate;
  float uPerMd = 65536.0f / s.lengthMd;
  s.accelUntilU = accDist * uPerMd;
  float decU = decDist * uPerMd;
  s.decelAfterU = 65536.0f - decU;
  if (s.decelAfterU < s.accelUntilU) s.decelAfterU = s.accelUntilU;
}

// Enqueue one joint-space segment ending at endMd.
static void planJointStep(const int32_t endMd[N_JOINTS], float nozzleLenMm, float feedMmS) {
  int32_t delta[N_JOINTS];
  uint32_t major = 0; uint8_t dirMask = 0;
  for (uint8_t j = 0; j < N_JOINTS; ++j) {
    delta[j] = endMd[j] - chainMd[j];
    if (delta[j] < 0) dirMask |= (uint8_t)(1 << j);
    uint32_t a = (uint32_t)labs(delta[j]);
    if (a > major) major = a;
  }
  if (major == 0) return;
  if (nozzleLenMm < 0.001f) nozzleLenMm = 0.001f;

  for (uint8_t j = 0; j < N_JOINTS; ++j) {           // per-joint speed cap
    uint32_t a = (uint32_t)labs(delta[j]);
    if (!a) continue;
    float fmax = JOINT_RATE_DPS[j] * 1000.0f * nozzleLenMm / (float)a;
    if (feedMmS > fmax) feedMmS = fmax;
  }
  if (feedMmS < 0.1f) feedMmS = 0.1f;

  while (queueFull() && !faultActive && !estopLatched) { }
  if (faultActive || estopLatched) return;

  uint8_t idx = qHead;
  Segment &s = queue[idx];
  s.dwellMs = 0; s.majorMd = major; s.dirMask = dirMask;
  s.nozzleLenMm = nozzleLenMm; s.feedMmS = feedMmS;
  s.lengthMd = (float)major;
  s.cruiseMdS = feedMmS * (float)major / nozzleLenMm;
  s.entryMdS = 0;
  s.accelMdS2 = (uint32_t)(JOINT_ACCEL_DPS2 * 1000.0f * (float)major / nozzleLenMm);
  if (s.accelMdS2 < 1) s.accelMdS2 = 1;
  for (uint8_t j = 0; j < N_JOINTS; ++j) { s.startMd[j] = chainMd[j]; s.deltaMd[j] = delta[j]; }

  // One-segment lookahead junction patch (same idea as Cartesian version)
  if (qHead != qTail) {
    uint8_t prevIdx = (uint8_t)((idx + QUEUE_SIZE - 1) & QUEUE_MASK);
    if (qTail != nextIdx(prevIdx)) {
      Segment tmp = queue[prevIdx];
      if (tmp.majorMd > 0) {
        bool compat = true;
        for (uint8_t j = 0; j < N_JOINTS; ++j)
          if (tmp.deltaMd[j] && s.deltaMd[j])
            if ((tmp.dirMask ^ s.dirMask) & (1 << j)) { compat = false; break; }
        if (compat) {
          float J = (tmp.cruiseMdS < s.cruiseMdS) ? tmp.cruiseMdS : s.cruiseMdS;
          float aMin = (tmp.accelMdS2 < s.accelMdS2) ? (float)tmp.accelMdS2 : (float)s.accelMdS2;
          float Lmin = (tmp.lengthMd < s.lengthMd) ? tmp.lengthMd : s.lengthMd;
          float Jmax = sqrtf(2.0f * aMin * Lmin);
          if (J > Jmax) J = Jmax;
          finalizeSegment(tmp, J);
          noInterrupts();
          uint8_t t = qTail; COMPILER_BARRIER();
          if (t != nextIdx(prevIdx)) queue[prevIdx] = tmp;
          interrupts();
          s.entryMdS = J;
        }
      }
    }
  }

  finalizeSegment(s, 0.0f);
  for (uint8_t j = 0; j < N_JOINTS; ++j) chainMd[j] = endMd[j];
  COMPILER_BARRIER();
  qHead = nextIdx(idx);
}

// Cartesian move: subdivide, IK each node, enqueue joint segments.
static uint8_t planCartesian(const float target[3], float feedMmS) {
  float len = sqrtf(
    sq(target[0] - posMm[0]) +
    sq(target[1] - posMm[1]) +
    sq(target[2] - posMm[2])
  );

  if (len < 1e-3f) {
    for (uint8_t i = 0; i < 3; ++i) posMm[i] = target[i];
    return ERR_NONE;
  }

  // Declare current extension here
  float startExt = chainMd[JT_EXT] / 1000.0f;

  uint16_t n = (uint16_t)ceilf(len / MOVE_SEG_MM);
  float prevP[3] = { posMm[0], posMm[1], posMm[2] };

  for (uint16_t k = 1; k <= n; ++k) {
    if (faultActive) return ERR_FAULT;

    float t = (float)k / (float)n;
    float p[3], jd[4];

    for (uint8_t i = 0; i < 3; ++i) {
      p[i] = posMm[i] + (target[i] - posMm[i]) * t;
    }

    // Use startExt here
    if (!ikSolve(p, jd, startExt)) goto fail;

    while (jd[0] * 1000.0f - chainMd[JT_SLEW] >  180000) jd[0] -= 360;
    while (jd[0] * 1000.0f - chainMd[JT_SLEW] < -180000) jd[0] += 360;

    if (!jointsInLimits(jd)) goto fail;

    {
      int32_t endMd[N_JOINTS];
      for (uint8_t j = 0; j < N_JOINTS; ++j) {
        endMd[j] = lroundf(jd[j] * 1000.0f);
      }

      float segLen = sqrtf(
        sq(p[0] - prevP[0]) +
        sq(p[1] - prevP[1]) +
        sq(p[2] - prevP[2])
      );

      planJointStep(endMd, segLen, feedMmS);
    }

    for (uint8_t i = 0; i < 3; ++i) prevP[i] = p[i];
  }

  for (uint8_t i = 0; i < 3; ++i) posMm[i] = target[i];
  return ERR_NONE;

fail:
  flushReq = true;
  while (flushReq && !faultActive) { }

  for (uint8_t j = 0; j < N_JOINTS; ++j) chainMd[j] = measMd[j];

  {
    float jd[4], p[3];
    for (uint8_t j = 0; j < N_JOINTS; ++j) jd[j] = measMd[j] / 1000.0f;
    fkSolve(jd, p);
    for (uint8_t i = 0; i < 3; ++i) posMm[i] = p[i] - offsetMm[i];
  }

  return ERR_UNREACHABLE;
}

static void planDwell(uint32_t ms) {
  while (queueFull() && !faultActive) { }
  if (faultActive) return;
  Segment &s = queue[qHead];
  s.majorMd = 0; s.dwellMs = ms;
  COMPILER_BARRIER();
  qHead = nextIdx(qHead);
}

static void waitIdle() { while ((busy || qHead != qTail) && !faultActive) { } }

// ================================== ISR ======================================

static void pumpOffISR() {
  digitalWrite(PIN_PUMP_EN, LOW);
  analogWrite(PIN_FLOW_PWM, 0);
  pumpOn = false;
}

static void tripISR(uint8_t code) {
  if (faultActive) return;
  faultCode = code; faultActive = true;
  pumpOffISR();
  digitalWrite(PIN_ENABLE, HIGH);         // release lever actuators
  drivesActive = false;
  for (uint8_t j = 0; j < N_JOINTS; ++j) targetMd[j] = measMd[j];
}

ISR(TIMER1_COMPA_vect) {
  static Segment *seg = NULL;
  static float u = 0, invU = 0;
  static uint32_t rate = 0, dwellTicks = 0;
  static uint32_t stepAcc[N_JOINTS] = { 0 };
  static uint8_t encState[N_JOINTS] = { 0 };

  // ---- E-stop (NC contact opened = tripped) ----
  if (digitalRead(PIN_ESTOP) == LOW && !estopLatched) {
    estopLatched = true;
    for (uint8_t j = 0; j < N_JOINTS; ++j) { /* keep meas fresh below */ }
    faultCode = FAULT_ESTOP; faultActive = true;
    pumpOffISR();
    digitalWrite(PIN_ENABLE, HIGH);
    drivesActive = false;
    seg = NULL; dwellTicks = 0; busy = false;
  }

  // ---- quadrature joint encoders (hall pairs work the same) ----
  static const int8_t QD[16] = { 0,1,-1,0, -1,0,0,1, 1,0,0,-1, 0,-1,1,0 };
  for (uint8_t j = 0; j < N_JOINTS; ++j) {
    uint8_t s = (uint8_t)((digitalRead(PIN_ENC_A[j]) ? 1 : 0) |
                          (digitalRead(PIN_ENC_B[j]) ? 2 : 0));
    if (s != encState[j]) {
      encCounts[j] += ENC_SIGN[j] * QD[(encState[j] << 2) | s];
      encState[j] = s;
    }
    measMd[j] = homeMd[j] + (int32_t)((encCounts[j] - homeCounts[j]) * mdPerCount[j]);
  }

  // ---- queue flush / target resync requests from the foreground ----
  if (flushReq) {
    seg = NULL; dwellTicks = 0; busy = false;
    qTail = qHead;
    resyncTargets = true;
    COMPILER_BARRIER(); flushReq = false;
  }
  if (resyncTargets) {
    for (uint8_t j = 0; j < N_JOINTS; ++j) targetMd[j] = measMd[j];
    resyncTargets = false;
  }
  if (faultActive) return;

  // ---- trajectory: claim, dwell, profile, interpolation ----
  bool catchup = false;
  {
    int32_t maxErr = 0;
    for (uint8_t j = 0; j < N_JOINTS; ++j) {
      int32_t e = targetMd[j] - measMd[j]; if (e < 0) e = -e;
      if (e > maxErr) maxErr = e;
    }
    catchup = (maxErr > (int32_t)(FOLLOW_HOLD_DEG * 1000));  // hydraulics lagging: wait
  }

  if (!seg && !holdActive && dwellTicks == 0 && !catchup && qHead != qTail) {
    uint8_t t = qTail; COMPILER_BARRIER();
    seg = &queue[t]; qTail = nextIdx(t); busy = true;
    if (seg->majorMd == 0) {
      dwellTicks = seg->dwellMs * (ISR_HZ / 1000UL);
      seg = NULL; if (!dwellTicks) busy = false;
    } else {
      u = 0;
      rate = (seg->entryRate > 200) ? seg->entryRate : 200;
      invU = 65536.0f / ((float)seg->majorMd * ISR_HZ);
      currentFeedMmS = seg->feedMmS;
      for (uint8_t j = 0; j < N_JOINTS; ++j) targetMd[j] = seg->startMd[j];
      return;
    }
  }

  if (seg && !holdActive && !catchup) {
    if (dwellTicks) { --dwellTicks; if (!dwellTicks) busy = false; }
    else {
      if (u < seg->accelUntilU) {
        uint32_t d = seg->accelMdS2 / rate;
        rate += d ? d : 1;
        if (rate > seg->cruiseRate) rate = seg->cruiseRate;
      } else if (u >= seg->decelAfterU && rate > seg->exitRate) {
        uint32_t d = seg->accelMdS2 / rate; if (!d) d = 1;
        uint32_t nr = (rate > d) ? rate - d : 0;
        uint32_t floor = (seg->exitRate > 200) ? seg->exitRate : 200;
        rate = (nr < floor) ? floor : nr;
      }
      u += (float)rate * invU;
      if (u > 65536.0f) u = 65536.0f;
      uint32_t ui = (uint32_t)u;
      for (uint8_t j = 0; j < N_JOINTS; ++j)
        targetMd[j] = seg->startMd[j] + (int32_t)(((int64_t)seg->deltaMd[j] * ui) >> 16);
      if (u >= 65536.0f) { seg = NULL; busy = false; }
    }
  } else if (dwellTicks && !holdActive) {
    --dwellTicks; if (!dwellTicks) busy = false;
  }

  // ---- closed-loop joint following: steppers pull the levers ----
  if (!drivesActive) return;
  for (uint8_t j = 0; j < N_JOINTS; ++j) {
    int32_t err = targetMd[j] - measMd[j];
    if (labs(err) > (int32_t)(FOLLOW_ERR_DEG * 1000)) { tripISR(FAULT_FOLLOW); return; }
    if (homed && (measMd[j] < jminMdHard[j] || measMd[j] > jmaxMdHard[j])) {
      tripISR(FAULT_LIMIT); return;
    }
    stepAcc[j] += STEP_RATE_LIMIT[j];
    if (stepAcc[j] >= ISR_HZ) {
      stepAcc[j] -= ISR_HZ;
      if (err > deadbandCounts[j]) {
        digitalWrite(PIN_DIR[j], HIGH);
        digitalWrite(PIN_STEP[j], HIGH); delayMicroseconds(3);
        digitalWrite(PIN_STEP[j], LOW);
      } else if (err < -deadbandCounts[j]) {
        digitalWrite(PIN_DIR[j], LOW);
        digitalWrite(PIN_STEP[j], HIGH); delayMicroseconds(3);
        digitalWrite(PIN_STEP[j], LOW);
      }
    }
  }
}

// ============================== LINE INTERPRETER =============================

struct Word { char c; float v; };

static void reportPosition() {
  float jd[N_JOINTS], p[3];
  noInterrupts();
  for (uint8_t j = 0; j < N_JOINTS; ++j) jd[j] = measMd[j] / 1000.0f;
  interrupts();
  fkSolve(jd, p);
  Serial.print(F("X:")); Serial.print(p[0] - offsetMm[0], 1);
  Serial.print(F(" Y:")); Serial.print(p[1] - offsetMm[1], 1);
  Serial.print(F(" Z:")); Serial.print(p[2] - offsetMm[2], 1);
  Serial.print(F(" | A:")); Serial.print(jd[0], 2);
  Serial.print(F(" B:")); Serial.print(jd[1], 2);
  Serial.print(F(" C:")); Serial.print(jd[2], 2);
  Serial.print(F(" D:")); Serial.print(jd[3], 2);
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

  // ---- G28: record current pose as home reference ----
  if (doG28) {
    waitIdle();
    if (faultActive) return ERR_FAULT;
    noInterrupts();
    for (uint8_t j = 0; j < N_JOINTS; ++j) {
      homeCounts[j] = encCounts[j];
      homeMd[j] = lroundf(HOME_ANGLE[j] * 1000.0f);
      chainMd[j] = homeMd[j];
    }
    interrupts();
    homed = true;
    resyncTargets = true;
    float p[3]; fkSolve(const_cast<float*>(HOME_ANGLE), p);
    for (uint8_t i = 0; i < 3; ++i) posMm[i] = p[i] - offsetMm[i];
  }

  // ---- G92 work offset (machine pos from FK of planner chain) ----
  if (doG92) {
    float jd[4], m[3];
    for (uint8_t j = 0; j < N_JOINTS; ++j) jd[j] = chainMd[j] / 1000.0f;
    fkSolve(jd, m);
    bool anyAxis = has['X' - 'A'] || has['Y' - 'A'] || has['Z' - 'A'];
    for (uint8_t i = 0; i < 3; ++i) {
      char L = (char)('X' + i);
      if (has[L - 'A'])  offsetMm[i] = m[i] - val[L - 'A'] * u;
      else if (!anyAxis) offsetMm[i] = m[i];
    }
  }

  // ---- motion ----
  bool hasXYZ = has['X' - 'A'] || has['Y' - 'A'] || has['Z' - 'A'];
  bool hasABCD = has['A' - 'A'] || has['B' - 'A'] || has['C' - 'A'] || has['D' - 'A'];
  if (motion < 0 && (hasXYZ || hasABCD)) motion = modalMotion;
  if (motion >= 0) modalMotion = motion;
  if (faultActive && (hasXYZ || hasABCD || doG4)) return ERR_FAULT;
  if (hasXYZ && hasABCD) return ERR_MIXED_WORDS;

  uint8_t merr = ERR_NONE;
  if (hasABCD) {                                    // ---- joint-space move ----
    if (motion != 0 && motion != 1) return ERR_UNSUPPORTED_G;
    if (absoluteMode && !homed) return ERR_NOT_HOMED;
    int32_t endMd[N_JOINTS];
    float endDeg[N_JOINTS], majorDeg = 0;
    for (uint8_t j = 0; j < N_JOINTS; ++j) {
      char L = (char)('A' + j);
      if (has[L - 'A']) {
        float v = val[L - 'A'];
        endDeg[j] = absoluteMode ? v : chainMd[j] / 1000.0f + v;
      } else endDeg[j] = chainMd[j] / 1000.0f;
      endMd[j] = lroundf(endDeg[j] * 1000.0f);
      float d = fabsf(endDeg[j] - chainMd[j] / 1000.0f);
      if (d > majorDeg) majorDeg = d;
    }
    if (homed && !jointsInLimits(endDeg)) return ERR_JOINT_LIMIT;
    float jd0[N_JOINTS], p0[3], p1[3];
    for (uint8_t j = 0; j < N_JOINTS; ++j) jd0[j] = chainMd[j] / 1000.0f;
    fkSolve(jd0, p0); fkSolve(endDeg, p1);
    float nozzleLen = sqrtf(sq(p1[0]-p0[0]) + sq(p1[1]-p0[1]) + sq(p1[2]-p0[2]));
    if (nozzleLen < 1.0f) nozzleLen = 1.0f;
    float feedMmS;
    if (motion == 0) feedMmS = 1.0e6f;              // joint rapid: capped by JOINT_RATE
    else {
      if (!has['F' - 'A']) return ERR_FEED_UNDEFINED;   // F = deg/min of largest joint move
      if (majorDeg < 1e-3f) feedMmS = 1.0f;
      else feedMmS = val['F' - 'A'] * nozzleLen / (60.0f * majorDeg);
    }
    planJointStep(endMd, nozzleLen, feedMmS);
    for (uint8_t i = 0; i < 3; ++i) posMm[i] = p1[i] - offsetMm[i];
  } else if (motion == 0 || motion == 1) {          // ---- Cartesian linear ----
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
  } else if (motion == 2 || motion == 3) {          // ---- arcs (subdivided) ----
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
    const float PI_F = 3.14159265359f;
   // float TWO_PI = 6.28318530718f;
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
    if (nSeg < 1) nSeg = 1; if (nSeg > 512) nSeg = 512;
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
      case 3: case 4: {
        flowPct = constrain(spindleS, 0.0f, 100.0f);
        digitalWrite(PIN_PUMP_EN, HIGH);
        pumpOn = true;
        break;
      }
      case 5: pumpOffISR(); break;
      case 8: digitalWrite(PIN_PUMP_EN, HIGH); pumpOn = true; break;
      case 9: pumpOffISR(); break;
      case 17:                                     // enable lever actuators
        if (!faultActive && !estopLatched) {
          drivesActive = true;
          digitalWrite(PIN_ENABLE, LOW);
          resyncTargets = true;
        }
        break;
      case 18:                                     // release lever actuators
        drivesActive = false;
        digitalWrite(PIN_ENABLE, HIGH);
        resyncTargets = true;
        break;
      case 114: reportPosition(); break;
      case 999:                                    // clear fault / e-stop latch
        if (digitalRead(PIN_ESTOP) == LOW) break;  // still pressed: can't clear
        estopLatched = false; faultActive = false; faultCode = 0;
        drivesActive = true;
        digitalWrite(PIN_ENABLE, LOW);
        resyncTargets = true;
        break;
      default: return ERR_UNSUPPORTED_M;
    }
  }
  return faultActive ? ERR_FAULT : ERR_NONE;
}

// ================================== RUNTIME ==================================

static void processLine(char *line) {
  uint8_t err = executeLine(line);
  if (err == ERR_NONE) Serial.println(F("ok"));
  else { Serial.print(F("error:")); Serial.println(err); }
}

// Pump flow command tracks actual nozzle speed vs commanded feed.
static void updateFlow() {
  static uint32_t lastMs = 0;
  static float lastPos[3];
  static bool init = false;
  uint32_t now = millis();
  if (now - lastMs < 100) return;
  float dt = (now - lastMs) / 1000.0f; lastMs = now;
  float jd[N_JOINTS], p[3];
  noInterrupts();
  for (uint8_t j = 0; j < N_JOINTS; ++j) jd[j] = measMd[j] / 1000.0f;
  interrupts();
  fkSolve(jd, p);
  float pwm = 0;
  if (init && pumpOn && !faultActive) {
    float speed = sqrtf(sq(p[0]-lastPos[0]) + sq(p[1]-lastPos[1]) + sq(p[2]-lastPos[2])) / dt;
    float cmd = currentFeedMmS; if (cmd < 0.1f) cmd = 0.1f;
    float ratio = speed / cmd; if (ratio > 1.0f) ratio = 1.0f;
    pwm = (int)(flowPct * 2.55f * ratio);
  }
  analogWrite(PIN_FLOW_PWM, (uint8_t)pwm);
  for (uint8_t i = 0; i < 3; ++i) lastPos[i] = p[i];
  init = true;
}

void setup() {
  Serial.begin(115200);
  for (uint8_t j = 0; j < N_JOINTS; ++j) {
    pinMode(PIN_STEP[j], OUTPUT); pinMode(PIN_DIR[j], OUTPUT);
    pinMode(PIN_ENC_A[j], INPUT_PULLUP); pinMode(PIN_ENC_B[j], INPUT_PULLUP);
    mdPerCount[j] = 1000.0f / COUNTS_PER_DEG[j];
    deadbandCounts[j] = (int32_t)(DEADBAND_DEG * COUNTS_PER_DEG[j]);
    if (deadbandCounts[j] < 1) deadbandCounts[j] = 1;
    jminMdHard[j] = (int32_t)((JMIN[j] - HARD_MARGIN_DEG) * 1000);
    jmaxMdHard[j] = (int32_t)((JMAX[j] + HARD_MARGIN_DEG) * 1000);
    homeMd[j] = lroundf(HOME_ANGLE[j] * 1000.0f);
    targetMd[j] = homeMd[j];
    chainMd[j] = homeMd[j];
  }
  pinMode(PIN_ENABLE, OUTPUT); pinMode(PIN_ESTOP, INPUT_PULLUP);
  pinMode(PIN_PUMP_EN, OUTPUT); pinMode(PIN_FLOW_PWM, OUTPUT);
  digitalWrite(PIN_ENABLE, HIGH);              // actuators released until M17
  digitalWrite(PIN_PUMP_EN, LOW);
  float p[3]; fkSolve(const_cast<float*>(HOME_ANGLE), p);
  for (uint8_t i = 0; i < 3; ++i) posMm[i] = p[i];

  noInterrupts();
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS10);
  OCR1A = F_CPU / ISR_HZ - 1;
  TIMSK1 = _BV(OCIE1A);
  interrupts();

  Serial.println(F("[boom-gcode] ready — UNHOMED"));
  Serial.println(F("home procedure: M17 -> jog joints with G91 A/B/C/D -> G28"));
}

void loop() {
  static char buf[128];
  static uint8_t len = 0;
  static uint8_t lastAlarm = 255;

  if (faultActive && faultCode != lastAlarm) {
    lastAlarm = faultCode;
    Serial.print(F("ALARM:"));
    Serial.println(faultCode);   // 1=e-stop 2=following error 3=joint hard limit
  }
  if (!faultActive) lastAlarm = 255;

  updateFlow();

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '!') { holdActive = true;  Serial.println(F("[hold]"));  continue; }
    if (c == '~') { holdActive = false; Serial.println(F("[resume]")); continue; }
    if (c == '\n' || c == '\r') {
      if (len) { buf[len] = '\0'; processLine(buf); len = 0; }
    } else if (len < sizeof(buf) - 1) buf[len++] = c;
    else { len = 0; Serial.println(F("error:line too long")); }
  }
}
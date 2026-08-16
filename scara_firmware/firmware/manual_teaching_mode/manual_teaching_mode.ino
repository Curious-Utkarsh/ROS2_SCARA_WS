/*
 * manual_teaching_mode.ino  --  SCARA manual teaching (lead-through) mode
 *
 * Isolated experimental sketch.  It does NOT share code with, and does not
 * replace, Scara_Stepper_Test.ino -- it lives in its own sketch folder so the
 * Arduino build never concatenates the two.
 *
 * What it does
 *   1. Holds the CNC-Shield driver ENABLE line de-asserted, so all stepper
 *      drivers are OFF and the arm can be pushed around by hand.
 *   2. Reads a joint-position source (see TEACH_SOURCE below).
 *   3. Streams the five joint values to the host at TX_HZ as plain ASCII.
 *
 * It never generates STEP pulses.  There is no closed loop, no homing and no
 * trajectory execution in this sketch.
 *
 * ---------------------------------------------------------------------------
 * WHY STEP/DIR CANNOT BE USED AS FEEDBACK
 *
 * STEP and DIR are *inputs* to the A4988/DRV8825.  Nothing on the shield ever
 * drives them back towards the Arduino, so turning the shaft by hand produces
 * no edges to count.  A disabled driver leaves the coils floating; the only
 * electrical trace of manual motion is back-EMF on the motor leads, which is
 * (a) not routed to any Arduino pin, (b) proportional to speed so it vanishes
 * when you move slowly, and (c) several volts peak.  Manual teaching therefore
 * needs a real position sensor.  See TEACH_SOURCE.
 * ---------------------------------------------------------------------------
 *
 * Serial protocol (115200 8N1)
 *   OUT  "J <column> <shoulder> <forearm> <wrist> <gripper>\n"   SI: rad / m
 *   OUT  "# ..."                                                 human comment
 *   IN   'Z'  zero all joints at the current pose (defines the teach origin)
 *   IN   'D'  disable drivers  (free / teaching -- the power-on default)
 *   IN   'E'  enable  drivers  (holding torque back on; motion is NOT free)
 *   IN   '?'  print a banner
 */

/* ── position source ─────────────────────────────────────────────────────── */
#define SRC_SIM   0   /* no extra hardware: synthetic motion, pipeline test   */
#define SRC_POT   1   /* 10k pot per joint on A0..A3, absolute, ~1 deg noise  */
#define SRC_QUAD  2   /* incremental quadrature encoder per joint (preferred) */

#ifndef TEACH_SOURCE
#define TEACH_SOURCE SRC_SIM
#endif

#define NUM_JOINTS 5      /* column, shoulder, forearm, wrist, gripper */
#define TX_HZ      50     /* host resamples/publishes this at ~30 Hz   */

/* ── CNC Shield V3 ───────────────────────────────────────────────────────── */
#define ENABLE_PIN 8      /* active LOW on A4988/DRV8825 -> HIGH = free shaft */

/* STEP/DIR pins are parked as inputs so this sketch physically cannot drive a
 * motor even if something else on the bench pulls on them. */
static const uint8_t STEP_PINS[4] = {2, 3, 4, 12};
static const uint8_t DIR_PINS[4]  = {5, 6, 7, 13};

/* ── calibration ─────────────────────────────────────────────────────────────
 * Joint order everywhere: column, shoulder, forearm, wrist, gripper.
 * Units are the URDF's: rad for the revolute joints, m for shoulder/gripper.
 *
 * SRC_QUAD: counts_per_joint_rev = encoder_PPR * 4 * gear_ratio.
 *   e.g. 600-PPR encoder on the motor, 5:1 belt  ->  600*4*5 = 12000 counts/rev
 * Measure yours: send 'Z', turn the joint exactly one revolution by hand, read
 * the streamed value back -- the ratio of what you get to 2*PI is your error.
 *
 * The gear ratios below are PLACEHOLDERS -- this robot has no encoders fitted.
 * They are NOT the same numbers as COLUMN_GEARING / FOREARM_GEARING in
 * scara_moveit_control.ino (19 and 16): those are motor-shaft-to-joint, while
 * these depend on where the encoder ends up mounted. Only if you put the
 * encoder on the motor shaft do they become 19 and 16.
 */
static const float UNITS_PER_COUNT[NUM_JOINTS] = {
  (2.0f * PI) / (2400.0f * 5.0f),   /* column   600 PPR x4, 5:1  [rad/count] */
  0.008f      / (2400.0f * 1.0f),   /* shoulder 8 mm/rev leadscrew  [m/count] */
  (2.0f * PI) / (2400.0f * 3.0f),   /* forearm  600 PPR x4, 3:1  [rad/count] */
  (2.0f * PI) / (2400.0f * 1.0f),   /* wrist    600 PPR x4, 1:1  [rad/count] */
  0.0f                              /* gripper  no sensor -> stays at 0       */
};

/* URDF joint limits -- clamped here as well as in the ROS node, so a miscount
 * can never push RViz outside the model. */
static const float JOINT_MIN[NUM_JOINTS] = {-2.0f, -0.15f, -2.0f, -2.0f, -0.06f};
static const float JOINT_MAX[NUM_JOINTS] = { 2.0f,  0.02f,  2.0f,  2.0f,  0.0f };

/* ── state ───────────────────────────────────────────────────────────────── */
#if TEACH_SOURCE != SRC_SIM
static volatile long g_counts[4];        /* raw counts, ISR-owned in SRC_QUAD */
static long          g_zero[4];          /* count value that means "joint 0"  */
#endif
static float         g_joints[NUM_JOINTS];
static uint32_t      g_next_tx_ms;

/* ═══════════════════════════════ SRC_QUAD ═══════════════════════════════════
 * Encoder A/B pairs, chosen to avoid every STEP/DIR pin and to sit inside two
 * pin-change-interrupt banks:
 *      joint 0  A0/A1     joint 1  A2/A3      (PORTC, PCINT1)
 *      joint 2  D9/D10    joint 3  D11/D12    (PORTB, PCINT0)
 * The CNC shield's limit-switch and spindle headers use D9..D13, so leave those
 * unpopulated while teaching.
 */
#if TEACH_SOURCE == SRC_QUAD

/* Gray-code transition table, indexed by (previous << 2) | current. */
static const int8_t QTAB[16] = { 0, -1,  1,  0,
                                 1,  0,  0, -1,
                                -1,  0,  0,  1,
                                 0,  1, -1,  0 };

static volatile uint8_t g_prev[4];

static inline void decodePair(uint8_t joint, uint8_t ab)
{
  const uint8_t idx = (uint8_t)((g_prev[joint] << 2) | ab);
  g_counts[joint] += QTAB[idx];
  g_prev[joint]    = ab;
}

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
#define QUAD_USE_PCINT 1

ISR(PCINT1_vect)            /* PORTC: A0..A3 */
{
  const uint8_t p = PINC;
  decodePair(0, (uint8_t)( p       & 0x03));
  decodePair(1, (uint8_t)((p >> 2) & 0x03));
}

ISR(PCINT0_vect)            /* PORTB: D9..D12 */
{
  const uint8_t p = PINB;
  decodePair(2, (uint8_t)((p >> 1) & 0x03));
  decodePair(3, (uint8_t)((p >> 3) & 0x03));
}
#else
#define QUAD_USE_PCINT 0
static const uint8_t QUAD_PINS[4][2] = {{A0, A1}, {A2, A3}, {9, 10}, {11, 12}};
#endif

static void quadBegin(void)
{
#if QUAD_USE_PCINT
  DDRC  &= (uint8_t)~0x0F;   /* A0..A3 inputs  */
  PORTC |=            0x0F;  /* with pull-ups  */
  DDRB  &= (uint8_t)~0x1E;   /* D9..D12 inputs */
  PORTB |=            0x1E;

  g_prev[0] = (uint8_t)( PINC       & 0x03);
  g_prev[1] = (uint8_t)((PINC >> 2) & 0x03);
  g_prev[2] = (uint8_t)((PINB >> 1) & 0x03);
  g_prev[3] = (uint8_t)((PINB >> 3) & 0x03);

  PCMSK1 |= 0x0F;            /* PCINT8..11  */
  PCMSK0 |= 0x1E;            /* PCINT1..4   */
  PCIFR   = (uint8_t)(bit(PCIF0) | bit(PCIF1));
  PCICR  |= (uint8_t)(bit(PCIE0) | bit(PCIE1));
#else
  for (uint8_t j = 0; j < 4; ++j) {
    pinMode(QUAD_PINS[j][0], INPUT_PULLUP);
    pinMode(QUAD_PINS[j][1], INPUT_PULLUP);
    g_prev[j] = (uint8_t)((digitalRead(QUAD_PINS[j][0]) << 1) |
                           digitalRead(QUAD_PINS[j][1]));
  }
#endif
}

static void quadPoll(void)
{
#if !QUAD_USE_PCINT
  /* Fallback for non-328P boards: fine for hand speeds, but a fast flick can
   * drop counts while Serial is transmitting.  Prefer a board with PCINT. */
  for (uint8_t j = 0; j < 4; ++j) {
    decodePair(j, (uint8_t)((digitalRead(QUAD_PINS[j][0]) << 1) |
                             digitalRead(QUAD_PINS[j][1])));
  }
#endif
}

static void readSource(void)
{
  quadPoll();
  for (uint8_t j = 0; j < 4; ++j) {
    noInterrupts();
    const long c = g_counts[j];
    interrupts();
    g_joints[j] = (float)(c - g_zero[j]) * UNITS_PER_COUNT[j];
  }
  g_joints[4] = 0.0f;   /* gripper has no teaching sensor */
}

/* ═══════════════════════════════ SRC_POT ════════════════════════════════════
 * One 10k potentiometer per joint on A0..A3, wiper to the analog pin.  Cheap,
 * absolute (survives a reset), no interrupts -- but ~10 bits over the full pot
 * sweep, so expect a degree or so of jitter.  ADC_SPAN maps the usable sweep
 * onto UNITS_PER_ADC below.
 */
#elif TEACH_SOURCE == SRC_POT

static const uint8_t POT_PINS[4] = {A0, A1, A2, A3};

/* Joint units per ADC count.  Default assumes the pot's usable ~300 deg sweep
 * is geared 1:1 to the joint: (300 deg in rad) / 1023 counts. */
static const float UNITS_PER_ADC[4] = {
  5.236f / 1023.0f,   /* column   [rad/count] */
  0.150f / 1023.0f,   /* shoulder [m/count]   */
  5.236f / 1023.0f,   /* forearm  [rad/count] */
  5.236f / 1023.0f    /* wrist    [rad/count] */
};

static float g_filt[4];
static bool  g_filt_primed;

static void potBegin(void)
{
  for (uint8_t j = 0; j < 4; ++j) pinMode(POT_PINS[j], INPUT);
}

static void readSource(void)
{
  for (uint8_t j = 0; j < 4; ++j) {
    const float raw = (float)analogRead(POT_PINS[j]);
    /* light EMA -- the ADC's last bit or two dance around constantly */
    g_filt[j] = g_filt_primed ? (0.8f * g_filt[j] + 0.2f * raw) : raw;
    g_counts[j] = (long)g_filt[j];
    g_joints[j] = (float)(g_counts[j] - g_zero[j]) * UNITS_PER_ADC[j];
  }
  g_filt_primed = true;
  g_joints[4] = 0.0f;
}

/* ═══════════════════════════════ SRC_SIM ════════════════════════════════════
 * No sensor at all.  Sweeps each joint slowly so the whole chain -- serial,
 * bridge node, robot_state_publisher, RViz -- can be brought up and verified
 * before any encoder exists.  'Z' freezes the sweep at the current pose.
 */
#else

static uint32_t g_sim_t0;
static bool     g_sim_frozen;
static float    g_sim_hold[NUM_JOINTS];

static void readSource(void)
{
  if (g_sim_frozen) {
    for (uint8_t j = 0; j < NUM_JOINTS; ++j) g_joints[j] = g_sim_hold[j];
    return;
  }
  const float t = (float)(millis() - g_sim_t0) * 0.001f;
  g_joints[0] =  1.20f * sinf(0.35f * t);
  g_joints[1] = -0.075f + 0.070f * sinf(0.55f * t);
  g_joints[2] =  1.20f * sinf(0.45f * t + 1.0f);
  g_joints[3] =  1.50f * sinf(0.60f * t + 2.0f);
  g_joints[4] = -0.030f + 0.030f * sinf(0.80f * t);
}

#endif

/* ═════════════════════════════════ common ═══════════════════════════════════ */

static void setDriversEnabled(bool en)
{
  digitalWrite(ENABLE_PIN, en ? LOW : HIGH);   /* active LOW */
}

static void zeroHere(void)
{
#if TEACH_SOURCE == SRC_SIM
  for (uint8_t j = 0; j < NUM_JOINTS; ++j) g_sim_hold[j] = g_joints[j];
  g_sim_frozen = !g_sim_frozen;
  Serial.println(g_sim_frozen ? F("# sim frozen") : F("# sim running"));
  if (!g_sim_frozen) g_sim_t0 = millis();
#else
  for (uint8_t j = 0; j < 4; ++j) {
    noInterrupts();
    g_zero[j] = g_counts[j];
    interrupts();
  }
  Serial.println(F("# zeroed"));
#endif
}

static void banner(void)
{
  Serial.println(F("# scara manual teaching mode"));
  Serial.print(F("# source="));
#if   TEACH_SOURCE == SRC_QUAD
  Serial.println(F("quad"));
#elif TEACH_SOURCE == SRC_POT
  Serial.println(F("pot"));
#else
  Serial.println(F("sim"));
#endif
  Serial.println(F("# joints=column,shoulder,forearm,wrist,gripper units=rad,m"));
  Serial.println(F("# cmds: Z=zero D=free E=hold ?=banner"));
}

static void handleSerial(void)
{
  while (Serial.available() > 0) {
    switch (Serial.read()) {
      case 'Z': case 'z': zeroHere(); break;
      case 'D': case 'd': setDriversEnabled(false); Serial.println(F("# drivers free")); break;
      case 'E': case 'e': setDriversEnabled(true);  Serial.println(F("# drivers holding")); break;
      case '?':           banner(); break;
      default:            break;   /* ignore CR/LF and anything else */
    }
  }
}

static void transmit(void)
{
  Serial.print('J');
  for (uint8_t j = 0; j < NUM_JOINTS; ++j) {
    float v = g_joints[j];
    if (v < JOINT_MIN[j]) v = JOINT_MIN[j];
    if (v > JOINT_MAX[j]) v = JOINT_MAX[j];
    Serial.print(' ');
    Serial.print(v, 5);
  }
  Serial.print('\n');
}

void setup()
{
  Serial.begin(115200);

  /* Free the arm before anything else touches the shield. */
  pinMode(ENABLE_PIN, OUTPUT);
  setDriversEnabled(false);

  /* Park STEP/DIR as high-Z inputs: no pulses can originate here. */
  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(STEP_PINS[i], INPUT);
    pinMode(DIR_PINS[i],  INPUT);
  }

#if   TEACH_SOURCE == SRC_QUAD
  quadBegin();
#elif TEACH_SOURCE == SRC_POT
  potBegin();
  readSource();                       /* prime the filter ... */
  for (uint8_t j = 0; j < 4; ++j) g_zero[j] = g_counts[j];   /* ... then zero */
#else
  g_sim_t0 = millis();
#endif

  banner();
  g_next_tx_ms = millis();
}

void loop()
{
  handleSerial();
  readSource();

  const uint32_t now = millis();
  if ((int32_t)(now - g_next_tx_ms) >= 0) {
    g_next_tx_ms += (1000UL / TX_HZ);
    if ((int32_t)(now - g_next_tx_ms) > 0) g_next_tx_ms = now;   /* catch up */
    transmit();
  }
}

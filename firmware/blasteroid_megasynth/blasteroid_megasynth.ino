/*
  BLASTEROID MEGASYNTH - experimental industrial ambient for keypad_synth (revA / revB)
  ===========================================================================================================
  A slow machine drone you play with layers, two robot mouths (SAM and the Talkie LPC voice) that can freeze into
  drones, scratch on the knob and talk to each other, a box of industrial noises, and a big reverb "space".
  Grown out of metal_hammer / speakbeat_hard (the voices, vocoder, sequencer and the splash renderer come from there).

  SAM (Software Automatic Mouth, C core from earlephilhower/ESP8266SAM) and the Talkie LPC vocabularies
  (Peter Knight / Armin Joachimsmeyer) are bundled in this folder; both GPL v3, so this sketch is too.

  KEYPAD (keypad hanging below the board, S1 top-left)
     [MODE ] [ pad 0 ] [ pad 1 ] [ pad 2 ]
     [FX   ] [ pad 3 ] [ pad 4 ] [ pad 5 ]
     [SEQ  ] [ pad 6 ] [ pad 7 ] [ pad 8 ]
     [SHIFT] [ pad 9 ] [ pad 10] [ pad 11]
  Left column: tap for the quick action, HOLD it and the screen shows what the 12 pads do.
     MODE  tap = next mode (HUM > SAM > TALK > MACH)     hold: modes, volume, talk bank, replay film, SAM voice
     FX    tap = next voice FX                            hold: pick FX, STUTTER/REVERSE/TAPE/UNDER, INFINITE (hold)
     SEQ   tap = REC on/off                               hold: play/stop, CROSSTALK, auto, BPM, random machines, clears
     SHIFT tap = PLAY/STOP                                hold: per-mode extras
  KNOB (the rotary pot)
     alone        = the current voice FX's main control (pitch, echo feedback, radio tuning, grain density...)
     SHIFT + knob = SCRATCH: scrub through the current SAM / Talkie word like a record under your hand
     FX + knob    = SPACE: from dry and close to a huge, long reverb
  Pads by mode:
     HUM   layers on/off: MAINS SUB PAD CHOIR / METAL WIND RADIO PULSE / DRIFT KEY- KEY+ CHORD
     SAM   12 words. tap = say it, hold = FREEZE it into a drone (granular). SHIFT+pad = say it backwards
     TALK  12 Speak & Spell words (4 banks). tap = say it, hold = freeze the mouth on one frame, humming in key
     MACH  12 machines (KLANK PIPE HAMMER / HISS GEIGER RELAY / SERVO SONAR CHAIN / DRIP HORN THUD). hold = repeat
  With REC on, voices and machines are quantized into the 16 steps.

  Board: "ESP32S3 Dev Module", USB CDC On Boot: Enabled, PSRAM: enabled
  (arduino-cli FQBN esp32:esp32:esp32s3:CDCOnBoot=cdc,PSRAM=enabled). Libraries: U8g2. ESP32 core 3.x.
*/

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <ESP_I2S.h>
#include "esp_dsp.h"
#include "SamData.h"
#include "sam.h"
#include "reciter.h"
#include "Vocab_US_Large.h"
#include "Vocab_AstroBlaster.h"
#include "Vocab_Soundbites.h"

// ---------------- pins (keypad_synth netlist, revA and revB) ----------------
#define PIN_SDA    9
#define PIN_SCL    8
#define AMP_LRC    1
#define AMP_BCLK   44
#define AMP_DIN    43
#define PIN_RGB    48
#define PIN_POT    10     // wiper
#define PIN_POT_HI 11     // driven HIGH: powers the pot's high leg
const int ROW_PINS[4] = {2, 3, 4, 5};
const int COL_PINS[4] = {6, 7, 13, 12};

#define STATUS_LOG 0      // 1 = print a status line every 2 s (only needed for debugging)
#define FS     22050
#define INV_FS (1.0f / FS)
#define BLOCK  128

const float VOLS[6] = {0.05f, 0.09f, 0.14f, 0.2f, 0.28f, 0.38f};
volatile int volIdx = 2;

// ---------------- types first (Arduino auto-prototypes) ----------------
struct Word { const char* text; uint32_t off, len; };
struct Voice { const char* name; uint8_t speed, pitch, throat, mouth; };
struct Step { int8_t word; uint8_t src, flags; uint16_t mach; bool skip; };
struct LpcFrame { uint8_t energy, period; int16_t k1, k2; int8_t k[8]; };
struct TalkWord { const char* name; const uint8_t* data; uint32_t first, n; };
struct PercDef { const char* name; uint8_t wave; float f0, f1, ptime, dec, gain, noise, drive; };
struct ModKey { bool down, used; uint32_t tDown; };
struct BQ {                                          // biquad, transposed direct form II
  float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
  inline float p(float x) { float y = b0 * x + z1; z1 = b1 * x - a1 * y + z2; z2 = b2 * x - a2 * y; return y; }
};
struct SVF { float lo = 0, bp = 0; };                 // Chamberlin state-variable filter state
struct Rng {                                         // small deterministic RNG (same seed -> same bolt)
  uint32_t s;
  explicit Rng(uint32_t seed) : s(seed * 2654435761u + 12345u) {}
  uint32_t next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
  float uni(float a, float b) { return a + (b - a) * (next() & 0xFFFFFF) / 16777216.0f; }
  int range(int a, int b) { return a + (int)(next() % (uint32_t)(b - a)); }
};
struct Grain { int word; float pos, inc, age, len, amp; bool on; };

enum { M_HUM, M_SAM, M_TALK, M_MACH, M_COUNT };
const char* MODE_NAMES[] = {"HUM", "SAM", "TALK", "MACH"};
enum { K_MODE, K_FX, K_SEQ, K_SHIFT };
const char* MOD_NAMES[] = {"MODE", "FX", "SEQ", "SHIFT"};
enum { SRC_SAM, SRC_TALK };
enum { SF_REV = 2 };
enum { FX_CLEAN, FX_ECHO, FX_RADIO, FX_GRAIN, FX_DEEP, FX_VOX, FX_CHOIR, FX_COUNT };
const char* FX_NAMES[] = {"CLEAN", "ECHO", "RADIO", "GRAIN", "DEEP", "VOX", "CHOIR"};
const char* FX_KNOB[] = {"PITCH", "FEEDBACK", "TUNING", "DENSITY", "DEPTH", "WET", "BREATH"};

const Voice VOICES[] = {
  {"ROBOT", 92, 60, 190, 190}, {"SAM", 72, 64, 128, 128}, {"GHOST", 110, 50, 170, 90}, {"DEEP", 96, 118, 120, 150},
};
#define NVOICES 4
// slow, cold, wide (the last pad word is the long line; bank[12] is the hidden name, said at boot)
Word bank[13] = {
  {"OHM"}, {"SILENCE"}, {"SIGNAL"}, {"MACHINE"}, {"BREATHE"}, {"ECHO"},
  {"VOID"}, {"DRIFT"}, {"STATIC"}, {"RESONANCE"}, {"FREQUENCY"}, {"WE ARE LISTENING"},
  {"BLASTEROID MEGA SYNTH"},
};

#define NBANKS 4
TalkWord talk[NBANKS][12] = {
  {{"CONTROL", sp4_CONTROL}, {"CONTACT", sp4_CONTACT}, {"RADAR", sp4_RADAR}, {"BASE", sp4_BASE},
   {"ZONE", sp4_ZONE}, {"SECURITY", sp4_SECURITY}, {"FREQUENCY", sp2_FREQUENCY}, {"HERTZ", sp2_HERTZ},
   {"PHASE", sp2_PHASE}, {"CIRCUIT", sp2_CIRCUIT}, {"DEVICE", sp2_DEVICE}, {"TERMINAL", sp4_TERMINAL}},
  {{"ZERO", sp3_ZERO}, {"ONE", sp3_ONE}, {"TWO", sp3_TWO}, {"THREE", sp3_THREE},
   {"FOUR", sp3_FOUR}, {"FIVE", sp3_FIVE}, {"SIX", sp3_SIX}, {"SEVEN", sp3_SEVEN},
   {"EIGHT", sp3_EIGHT}, {"NINE", sp3_NINE}, {"POINT", sp3_POINT}, {"MINUS", sp3_MINUS}},
  {{"WIND", sp3_WIND}, {"RAIN", sp3_RAIN}, {"FOG", sp3_FOG}, {"MIST", sp3_MIST},
   {"ICE", sp3_ICE}, {"SNOW", sp3_SNOW}, {"STORM", sp3_STORM}, {"HAZE", sp3_HAZE},
   {"SMOKE", sp3_SMOKE}, {"SAND", sp3_SAND}, {"HEAT", sp3_HEAT}, {"THUNDERSTORM", sp3_THUNDERSTORM}},
  {{"MACHINE", sp2_MACHINE}, {"MOTOR", sp2_MOTOR}, {"ENGINE", sp5_ENGINE}, {"VALVE", sp2_VALVE},
   {"PRESSURE", sp2_PRESSURE}, {"TEMPERATURE", sp2_TEMPERATURE}, {"POWER", sp2_POWER}, {"VOLTS", sp2_VOLTS},
   {"OHMS", sp2_OHMS}, {"CYCLE", sp2_CYCLE}, {"OPERATIONAL", operational}, {"IDLE", sp5_IDLE}},
};
const char* BANK_NAMES[NBANKS] = {"SIGNAL", "NUMBERS", "ELEMENTS", "MACHINE"};

// machines. drive > 0 = through softclip after the envelope. W_FM: f0 carrier, f1 ratio, noise = index.
// W_SINE sweeps f0 -> f1 with ptime. W_SNR with f0 = f1 = 0 is plain bright noise (hiss / clicks).
enum { W_SINE, W_KICK, W_SNR, W_CLAP, W_METAL, W_FM, W_SLAM };
const PercDef PERC[12] = {
  // name     wave     f0     f1     ptime   dec    gain   noise  drive
  {"KLANK", W_FM,    260,   2.41f, 0.08f,  0.90f, 0.42f, 2.2f,  0},
  {"PIPE",  W_FM,    610,   1.41f, 0.15f,  1.40f, 0.32f, 0.6f,  0},
  {"HAMMR", W_SLAM,  70,    40,    0.06f,  0.50f, 0.75f, 0,     2.2f},
  {"HISS",  W_SNR,   0,     0,     0.01f,  1.20f, 0.22f, 0.8f,  0},
  {"GEIGR", W_SNR,   0,     0,     0.001f, 0.004f,0.70f, 1.0f,  0},
  {"RELAY", W_KICK,  3200,  1900,  0.002f, 0.012f,0.55f, 0.6f,  1.5f},
  {"SERVO", W_SINE,  300,   1300,  0.40f,  0.55f, 0.30f, 0,     0},
  {"SONAR", W_SINE,  1250,  1250,  0.01f,  1.60f, 0.30f, 0,     0},
  {"CHAIN", W_METAL, 2.2f,  0,     0.01f,  0.05f, 0.50f, 0,     0},
  {"DRIP",  W_SINE,  1800,  500,   0.02f,  0.09f, 0.45f, 0,     0},
  {"HORN",  W_FM,    110,   0.5f,  0.30f,  1.30f, 0.40f, 1.5f,  1.4f},
  {"THUD",  W_KICK,  90,    38,    0.08f,  0.90f, 0.90f, 0.1f,  1.5f},
};
enum { D_KLANK, D_PIPE, D_HAMMR, D_HISS, D_GEIGR, D_RELAY, D_SERVO, D_SONAR, D_CHAIN, D_DRIP, D_HORN, D_THUD };
const uint16_t DUCK_MASK = (1 << D_THUD) | (1 << D_HAMMR);

// ---------------- the drone ----------------
enum { L_MAINS, L_SUB, L_PAD, L_CHOIR, L_METAL, L_WIND, L_RADIO, L_PULSE, L_COUNT };
const char* LAYER_NAMES[L_COUNT] = {"MAINS", "SUB", "PAD", "CHOIR", "METAL", "WIND", "RADIO", "PULSE"};
const char* LAYER_SHORT[L_COUNT] = {"MNS", "SUB", "PAD", "CHR", "MTL", "WND", "RAD", "PLS"};
const float LAYER_GAIN[L_COUNT] = {0.22f, 0.35f, 0.30f, 0.34f, 0.16f, 0.30f, 0.20f, 1.0f};
const char* CHORD_NAMES[5] = {"FIFTH", "MINOR7", "SUS2", "MINOR9", "VOID"};
const char* CHORD_SUF[5] = {"5", "m7", "sus2", "m9", "x"};
const int8_t CHORD_IV[5][4] = {{0, 7, 12, 19}, {0, 3, 7, 10}, {0, 2, 7, 14}, {0, 3, 10, 14}, {0, 6, 13, 18}};
const int8_t DRIFT_STEPS[4] = {0, 8, 3, 10};        // i - bVI - bIII - bVII, one move every 2 bars
const char* NOTE[12] = {"C", "C#", "D", "Eb", "E", "F", "F#", "G", "Ab", "A", "Bb", "B"};

// Serial log that never blocks: with USB plugged in but no serial monitor open, the USB-CDC TX buffer fills
// and a plain Serial.printf hangs (screen froze ~3 s after boot, audio task stalled). Lines that don't fit are dropped.
void logf(const char* fmt, ...) {
  char b[160];
  va_list ap; va_start(ap, fmt);
  int n = vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  n = min(n, (int)sizeof b - 1);
  if (Serial.availableForWrite() >= n) Serial.write((const uint8_t*)b, n);
}

U8G2_SH1107_PIMORONI_128X128_F_HW_I2C oled(U8G2_R1, U8X8_PIN_NONE, PIN_SCL, PIN_SDA);   // R1 = 90 deg clockwise
I2SClass amp;

void drawFrame();

// =====================================================================
// SAM -> sample pool
// =====================================================================
SamData* samdata;
int8_t* pool = nullptr;
uint32_t poolSize = 0, poolPos = 0;
volatile bool bankBusy = true;
volatile bool nameReady = false;                    // bank[12] (the name) renders first
volatile int voiceIdx = 0;
int renderProgress = 0;

static void samByte(void*, unsigned char b) { if (poolPos < poolSize) pool[poolPos++] = (int8_t)((int)b - 128); }

bool sayToPool(const char* text, const Voice& v) {
  char input[256];
  int n = 0;
  for (; text[n] && n < 200; n++) input[n] = toupper((int)text[n]);
  input[n] = 0;
  strcat(input, "[");
  samdata = new SamData;
  if (!samdata) return false;
  EnableSingmode(0);
  SetSpeed(v.speed); SetPitch(v.pitch); SetThroat(v.throat); SetMouth(v.mouth);
  bool ok = TextToPhonemes(input);
  if (ok) { SetInput(input); SAMMain(samByte, nullptr); }
  delete samdata;
  return ok;
}

void renderBank(int vi, bool draw) {
  bankBusy = true; nameReady = false;
  delay(30);
  poolPos = 0;
  for (int step = 0; step < 13; step++) {
    int i = step == 0 ? 12 : step - 1;               // the name first, then the 12 pad words
    renderProgress = step * 12 / 13;
    if (draw) drawFrame();
    uint32_t start = poolPos;
    bank[i].off = start; bank[i].len = 0;
    if (poolSize - poolPos < FS / 2) continue;
    if (!sayToPool(bank[i].text, VOICES[vi])) { poolPos = start; continue; }
    uint32_t a = start, b = poolPos;
    while (a < b && abs(pool[a]) < 4) a++;
    while (b > a && abs(pool[b - 1]) < 4) b--;
    bank[i].off = a; bank[i].len = b > a + 8 ? b - a : 0;
    if (i == 12) nameReady = true;
  }
  renderProgress = 12;
  logf("SAM voice %s: %u bytes\n", VOICES[vi].name, poolPos);
  bankBusy = false;
}

static inline float wordAt(int w, float p) {
  const Word& wd = bank[w];
  if (p < 0 || p >= (float)wd.len - 1) return 0;          // float compare: len 0 must not wrap to 4 billion
  int i = (int)p; float f = p - i;
  float a = pool[wd.off + i], b = pool[wd.off + i + 1];
  return (a + (b - a) * f) * (1.0f / 128.0f);
}

// =====================================================================
// Talkie LPC: words decoded to frames at boot, re-synthesized live (so they can be bent)
// Tables and lattice filter from Talkie (Peter Knight) via ESP8266Audio's AudioGeneratorTalkie.
// =====================================================================
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnarrowing"
static const uint8_t tmsEnergy[0x10] = {0x00,0x02,0x03,0x04,0x05,0x07,0x0a,0x0f,0x14,0x20,0x29,0x39,0x51,0x72,0xa1,0xff};
static const uint8_t tmsPeriod[0x40] = {0x00,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x2A,0x2B,0x2D,0x2F,0x31,0x33,0x35,0x36,0x39,0x3B,0x3D,0x3F,0x42,0x45,0x47,0x49,0x4D,0x4F,0x51,0x55,0x57,0x5C,0x5F,0x63,0x66,0x6A,0x6E,0x73,0x77,0x7B,0x80,0x85,0x8A,0x8F,0x95,0x9A,0xA0};
static const int16_t tmsK1[0x20] = {0x82C0,0x8380,0x83C0,0x8440,0x84C0,0x8540,0x8600,0x8780,0x8880,0x8980,0x8AC0,0x8C00,0x8D40,0x8F00,0x90C0,0x92C0,0x9900,0xA140,0xAB80,0xB840,0xC740,0xD8C0,0xEBC0,0x0000,0x1440,0x2740,0x38C0,0x47C0,0x5480,0x5EC0,0x6700,0x6D40};
static const int16_t tmsK2[0x20] = {0xAE00,0xB480,0xBB80,0xC340,0xCB80,0xD440,0xDDC0,0xE780,0xF180,0xFBC0,0x0600,0x1040,0x1A40,0x2400,0x2D40,0x3600,0x3E40,0x45C0,0x4CC0,0x5300,0x5880,0x5DC0,0x6240,0x6640,0x69C0,0x6CC0,0x6F80,0x71C0,0x73C0,0x7580,0x7700,0x7E80};
static const int8_t tmsK3[0x10] = {0x92,0x9F,0xAD,0xBA,0xC8,0xD5,0xE3,0xF0,0xFE,0x0B,0x19,0x26,0x34,0x41,0x4F,0x5C};
static const int8_t tmsK4[0x10] = {0xAE,0xBC,0xCA,0xD8,0xE6,0xF4,0x01,0x0F,0x1D,0x2B,0x39,0x47,0x55,0x63,0x71,0x7E};
static const int8_t tmsK5[0x10] = {0xAE,0xBA,0xC5,0xD1,0xDD,0xE8,0xF4,0xFF,0x0B,0x17,0x22,0x2E,0x39,0x45,0x51,0x5C};
static const int8_t tmsK6[0x10] = {0xC0,0xCB,0xD6,0xE1,0xEC,0xF7,0x03,0x0E,0x19,0x24,0x2F,0x3A,0x45,0x50,0x5B,0x66};
static const int8_t tmsK7[0x10] = {0xB3,0xBF,0xCB,0xD7,0xE3,0xEF,0xFB,0x07,0x13,0x1F,0x2B,0x37,0x43,0x4F,0x5A,0x66};
static const int8_t tmsK8[0x08] = {0xC0,0xD8,0xF0,0x07,0x1F,0x37,0x4F,0x66};
static const int8_t tmsK9[0x08] = {0xC0,0xD4,0xE8,0xFC,0x10,0x25,0x39,0x4D};
static const int8_t tmsK10[0x08] = {0xCD,0xDF,0xF1,0x04,0x16,0x20,0x3B,0x4D};
static const int8_t chirp[] = {0x00,0x2a,0xd4,0x32,0xb2,0x12,0x25,0x14,0x02,0xe1,0xc5,0x02,0x5f,0x5a,0x05,0x0f,0x26,0xfc,0xa5,0xa5,0xd6,0xdd,0xdc,0xfc,0x25,0x2b,0x22,0x21,0x0f,0xff,0xf8,0xee,0xed,0xef,0xf7,0xf6,0xfa,0x00,0x03,0x02,0x01};
#pragma GCC diagnostic pop

LpcFrame* lpcFrames = nullptr;
uint32_t lpcCap = 0, lpcUsed = 0;
volatile int talkBank = 0;

struct BitReader {
  const uint8_t* p; uint8_t bit = 0;
  static uint8_t rev(uint8_t a) {
    a = (a >> 4) | (a << 4); a = ((a & 0xcc) >> 2) | ((a & 0x33) << 2); a = ((a & 0xaa) >> 1) | ((a & 0x55) << 1); return a;
  }
  uint8_t get(uint8_t bits) {
    uint16_t data = rev(p[0]) << 8;
    if (bit + bits > 8) data |= rev(p[1]);
    data <<= bit;
    uint8_t v = data >> (16 - bits);
    bit += bits;
    if (bit >= 8) { bit -= 8; p++; }
    return v;
  }
};

// decode one LPC word; out == nullptr just counts frames
uint32_t decodeLpc(const uint8_t* data, LpcFrame* out, uint32_t maxN) {
  BitReader br{data};
  LpcFrame cur = {};
  uint32_t n = 0;
  while (n < maxN) {
    uint8_t e = br.get(4);
    if (e == 0xF) break;
    if (e == 0) cur.energy = 0;
    else {
      cur.energy = tmsEnergy[e];
      uint8_t rep = br.get(1);
      cur.period = tmsPeriod[br.get(6)];
      if (!rep) {
        cur.k1 = tmsK1[br.get(5)]; cur.k2 = tmsK2[br.get(5)];
        cur.k[0] = tmsK3[br.get(4)]; cur.k[1] = tmsK4[br.get(4)];
        if (cur.period) {
          cur.k[2] = tmsK5[br.get(4)]; cur.k[3] = tmsK6[br.get(4)]; cur.k[4] = tmsK7[br.get(4)];
          cur.k[5] = tmsK8[br.get(3)]; cur.k[6] = tmsK9[br.get(3)]; cur.k[7] = tmsK10[br.get(3)];
        }
      }
    }
    if (out) out[n] = cur;
    n++;
  }
  return n;
}

void decodeTalk() {
  const uint32_t MAXW = 8000;
  uint32_t total = 0;
  for (auto& b : talk) for (auto& w : b) total += decodeLpc(w.data, nullptr, MAXW);
  lpcCap = total;
  lpcFrames = psramFound() ? (LpcFrame*)ps_malloc(total * sizeof(LpcFrame)) : nullptr;
  if (!lpcFrames) lpcFrames = (LpcFrame*)malloc(total * sizeof(LpcFrame));
  if (!lpcFrames) { logf("!! no memory for LPC frames\n"); return; }
  for (auto& b : talk) for (auto& w : b) {
    uint32_t n = decodeLpc(w.data, lpcFrames + lpcUsed, MAXW);
    uint32_t a = 0;
    while (a < n && lpcFrames[lpcUsed + a].energy == 0) a++;      // trim silent frames at both ends
    while (n > a && lpcFrames[lpcUsed + n - 1].energy == 0) n--;
    w.first = lpcUsed + a; w.n = n - a;
    lpcUsed += n;
  }
  logf("LPC: %u frames decoded (%u KB)\n", lpcUsed, lpcUsed * sizeof(LpcFrame) / 1024);
}

// live LPC voice with circuit bends
volatile bool lpWhisper = false, lpMono = false, lpFreeze = false, lpStretch = false, lpGlitch = false, lpRev = false;
volatile float lpPitch = 1, lpFormant = 1;
volatile float monoFreq = 130;

struct Lpc {
  int bank = 0, word = -1; float fpos = 0; bool on = false, scratch = false, rev = false; float base = 0;
  int loaded = -1;
  uint8_t period = 0; int32_t energy = 0, k1 = 0, k2 = 0; int32_t k[8] = {0};
  uint8_t pc = 0; int32_t x[10] = {0}; uint16_t rnd = 1;
  float ph = 0, y0 = 0, y1 = 0;
} lpc;
volatile int lpcVisFrame = -1;

void talkPlay(int w, bool rev) {
  const TalkWord& t = talk[talkBank][w];
  if (!t.n) return;
  lpc.bank = talkBank; lpc.word = w; lpc.rev = rev ^ lpRev; lpc.scratch = false;
  lpc.fpos = lpc.rev ? t.n - 0.01f : 0; lpc.loaded = -1; lpc.on = true;
}
void talkScratch(int w) {
  const TalkWord& t = talk[talkBank][w];
  if (!t.n) return;
  lpc.bank = talkBank; lpc.word = w; lpc.scratch = true; lpc.base = 0; lpc.fpos = 0; lpc.loaded = -1; lpc.on = true;
}

void lpcLoad(int fi) {
  const TalkWord& t = talk[lpc.bank][lpc.word];
  const LpcFrame& f = lpcFrames[t.first + fi];
  lpc.loaded = fi;
  lpc.energy = f.energy;
  lpc.k1 = f.k1; lpc.k2 = f.k2;
  for (int i = 0; i < 8; i++) lpc.k[i] = f.k[i];
  if (lpGlitch) {                                   // circuit bend: corrupt some coefficients
    if (random(3) == 0) lpc.k1 = tmsK1[random(32)];
    if (random(3) == 0) lpc.k2 = tmsK2[random(32)];
    if (random(2) == 0) lpc.k[random(8)] = -lpc.k[random(8)];
  }
  int per = f.period;
  if (lpWhisper) per = 0;
  else if (per) {
    if (lpMono) per = (int)(8000.0f / monoFreq);
    else per = (int)(per / lpPitch);
    per = constrain(per, 10, 255);
  }
  lpc.period = per;
}

// one 8 kHz sample
static inline float lpcTick() {
  const TalkWord& t = talk[lpc.bank][lpc.word];
  if (!lpc.on || !t.n) return 0;
  int fi = (int)lpc.fpos;
  if (fi < 0 || fi >= (int)t.n) { if (!lpc.scratch) { lpc.on = false; return 0; } fi = constrain(fi, 0, (int)t.n - 1); }
  if (fi != lpc.loaded) lpcLoad(fi);
  if (!lpc.scratch && !lpFreeze) lpc.fpos += (lpc.rev ? -1.0f : 1.0f) * (lpStretch ? 0.4f : 1.0f) / 200.0f;

  int32_t u10;
  if (lpc.period) {
    if (lpc.pc < lpc.period) lpc.pc++; else lpc.pc = 0;
    u10 = lpc.pc < sizeof(chirp) ? (chirp[lpc.pc] * lpc.energy) >> 8 : 0;
  } else {
    lpc.rnd = (lpc.rnd >> 1) ^ ((lpc.rnd & 1) ? 0xB800 : 0);
    u10 = (lpc.rnd & 1) ? lpc.energy : -lpc.energy;
  }
  int32_t* x = lpc.x; int32_t* k = lpc.k;
  int32_t u9 = u10 - ((k[7] * x[9]) >> 7);
  int32_t u8 = u9 - ((k[6] * x[8]) >> 7);
  int32_t u7 = u8 - ((k[5] * x[7]) >> 7);
  int32_t u6 = u7 - ((k[4] * x[6]) >> 7);
  int32_t u5 = u6 - ((k[3] * x[5]) >> 7);
  int32_t u4 = u5 - ((k[2] * x[4]) >> 7);
  int32_t u3 = u4 - ((k[1] * x[3]) >> 7);
  int32_t u2 = u3 - ((k[0] * x[2]) >> 7);
  int32_t u1 = u2 - ((lpc.k2 * x[1]) >> 15);
  int32_t u0 = u1 - ((lpc.k1 * x[0]) >> 15);
  u0 = constrain(u0, -512, 511);
  x[9] = x[8] + ((k[6] * u8) >> 7);
  x[8] = x[7] + ((k[5] * u7) >> 7);
  x[7] = x[6] + ((k[4] * u6) >> 7);
  x[6] = x[5] + ((k[3] * u5) >> 7);
  x[5] = x[4] + ((k[2] * u4) >> 7);
  x[4] = x[3] + ((k[1] * u3) >> 7);
  x[3] = x[2] + ((k[0] * u2) >> 7);
  x[2] = x[1] + ((lpc.k2 * u1) >> 15);
  x[1] = x[0] + ((lpc.k1 * u0) >> 15);
  x[0] = u0;
  for (int i = 0; i < 10; i++) x[i] = constrain(x[i], -32768, 32767);
  return u0 * (1.0f / 512.0f);
}

// =====================================================================
// DSP helpers + drums
// =====================================================================
float sineTab[1025];
static inline float wrap1(float p) { return p - floorf(p); }
static inline float fsin(float ph) {
  float x = ph * 1024.0f; int i = (int)x; float f = x - i; i &= 1023;
  return sineTab[i] + (sineTab[i + 1] - sineTab[i]) * f;
}
static uint32_t nstate = 22222;
static inline float nz() { nstate ^= nstate << 13; nstate ^= nstate >> 17; nstate ^= nstate << 5; return (int32_t)nstate * (1.0f / 2147483648.0f); }
static inline float dcoef(float sec) { return expf(-1.0f / (sec * FS)); }
static inline float softclip(float x) { if (x > 3) return 1; if (x < -3) return -1; return x * (27 + x * x) / (27 + 9 * x * x); }
static inline float midiHz(float m) { return 440.0f * exp2f((m - 69) * (1.0f / 12.0f)); }

enum { BQ_LP, BQ_HP, BQ_PEAK };
void bqSet(BQ& q, int type, float fc, float Q, float dB) {   // RBJ cookbook
  float w = 2 * PI * fc / FS, cw = cosf(w), al = sinf(w) / (2 * Q), A = powf(10, dB / 40);
  float b0, b1, b2, a0, a1, a2;
  if (type == BQ_LP) { b0 = (1 - cw) / 2; b1 = 1 - cw; b2 = b0; a0 = 1 + al; a1 = -2 * cw; a2 = 1 - al; }
  else if (type == BQ_HP) { b0 = (1 + cw) / 2; b1 = -(1 + cw); b2 = b0; a0 = 1 + al; a1 = -2 * cw; a2 = 1 - al; }
  else { b0 = 1 + al * A; b1 = -2 * cw; b2 = 1 - al * A; a0 = 1 + al / A; a1 = -2 * cw; a2 = 1 - al / A; }
  q.b0 = b0 / a0; q.b1 = b1 / a0; q.b2 = b2 / a0; q.a1 = a1 / a0; q.a2 = a2 / a0;
}

struct Perc {
  float env = 0, d = 0.99f, penv = 0, pd = 0.99f, ph = 0, ph2 = 0, prev = 0, lp = 0, bp = 0;
  float mph[6] = {0};
  uint32_t age = 0;
  void trig(const PercDef& p, float a) { env = a; d = dcoef(p.dec); penv = 1; pd = dcoef(p.ptime); ph = 0; age = 0; }
  inline float s(const PercDef& p, float rate) {
    if (env < 0.0001f) return 0;
    float f = (p.f1 + (p.f0 - p.f1) * penv) * rate, o = 0;
    penv *= pd;
    switch (p.wave) {
      case W_SINE: ph = wrap1(ph + f * INV_FS); o = fsin(ph) + nz() * p.noise * penv; break;
      case W_KICK: {                                   // sine sweep + a noise click on the front
        ph = wrap1(ph + f * INV_FS);
        o = fsin(ph) + (age < 110 ? nz() * p.noise * (1 - age / 110.0f) : 0);
        break;
      }
      case W_SNR: {                                    // body that thins out + bright noise that stays
        ph = wrap1(ph + f * INV_FS);
        float n = nz(), h = n - prev; prev = n;
        o = fsin(ph) * (0.2f + 0.8f * penv) + h * p.noise;
        break;
      }
      case W_CLAP: {                                   // three quick bursts, then the tail
        float n = nz(), q = 0.42f; lp += q * bp; bp += q * (n - lp - 0.6f * bp);
        float g = age < 660 ? 1.0f - (age % 220) / 260.0f : 1.0f;
        o = bp * 1.6f * g;
        break;
      }
      case W_METAL: {                                  // 808 cymbal: six detuned squares, high-passed
        static const float MF[6] = {205.3f, 304.4f, 369.6f, 522.7f, 540.0f, 800.0f};
        float sum = 0, sc = p.f0 * rate * INV_FS;
        for (int i = 0; i < 6; i++) { mph[i] = wrap1(mph[i] + MF[i] * sc); sum += mph[i] < 0.5f ? 1 : -1; }
        lp += 0.55f * (sum - lp);
        float h = sum - lp;
        bp += 0.55f * (h - bp);
        o = h - bp;
        break;
      }
      case W_FM: {                                     // inharmonic clang, index falls with penv
        float fc = p.f0 * rate;
        ph = wrap1(ph + fc * INV_FS); ph2 = wrap1(ph2 + fc * p.f1 * INV_FS);
        o = fsin(wrap1(ph + p.noise * (0.25f + penv) * fsin(ph2)));
        break;
      }
      case W_SLAM: {                                   // dark noise + thump: a big steel door
        ph = wrap1(ph + f * INV_FS);
        lp += 0.12f * (nz() - lp);
        o = lp * 2.6f * (0.35f + 0.65f * penv) + fsin(ph) * penv;
        break;
      }
    }
    age++;
    o *= env; env *= d;
    if (p.drive > 0) o = softclip(o * p.drive) * (1.0f / 1.15f);
    return o * p.gain;
  }
};
Perc perc[12];
volatile uint16_t machMute = 0;
volatile float duckEnv = 0;                         // THUD / HAMMER push everything else down a little
void hitMach(uint16_t bits) {
  for (int i = 0; i < 12; i++) if (bits & (1 << i)) perc[i].trig(PERC[i], 1);
  if (bits & DUCK_MASK) duckEnv = 1;
}

#define DLY_N 16384
float* dlyBuf = nullptr;
int dlyW = 0;

// big reverb: 4-line feedback delay network, Hadamard mix, damped (from Nightlight), lines in RAM
const int RL[4] = {3001, 3769, 4513, 5333};
float* rv[4]; int ri[4] = {0};
float rlp[4] = {0};

// =====================================================================
// Knob + instrument state
// =====================================================================
volatile float knobVal = 0.5f, space = 0.45f;
volatile float fxAmt[FX_COUNT] = {0.5f, 0.55f, 0.45f, 0.5f, 0.5f, 0.6f, 0.4f};
volatile bool knobScratch = false;                  // SHIFT + knob is scrubbing a word
volatile float deckTarget = 0, lpcTarget = 0;

Step seq[16];
volatile int mode = M_HUM, curStep = 0, curPad = 0, fx = FX_ECHO, bpm = 72, machDens = 0;
volatile bool rec = false, playing = false, stutterOn = false, reverseOn = false, tapeOn = false, underOn = false;
volatile bool xtalk = false, revFreeze = false, freezeHeld = false, talkFreezeHeld = false;
volatile bool eggReq = false, replayReq = false;    // MODE + SHIFT ~1 s = the film with credits; MODE menu REPLAY = without
volatile int lastSrc = SRC_SAM;
volatile float tapeRate = 1, stepFrac = 0;
volatile uint16_t rollBits = 0;
char msg[28] = ""; volatile uint32_t msgAt = 0;
void flash(const char* m) { strncpy(msg, m, sizeof(msg) - 1); msgAt = millis(); }

// drone state
volatile uint16_t humOn = (1 << L_SUB) | (1 << L_PAD);
volatile int humKey = 2, humOct = 0, humChord = 3, driftPos = 0;
volatile bool driftOn = true;
volatile float humBright = 0.5f, humDetune = 0.004f, lfoRate = 1;
volatile float humLvl[L_COUNT];                      // per-layer level, for the screen
volatile float masterFade = 0;                       // boot fade-in

struct Deck {
  int word = -1;
  float pos = 0, rate = 1;
  bool on = false, scratch = false, rev = false, freeze = false;
  float freezePos = 0;
} deck;
volatile float deckVis = 0, outPeak = 0;
volatile float scopeBuf[128];
Grain grains[12];

void deckPlay(int w, bool rev) {
  if (bankBusy || w < 0 || bank[w].len == 0) return;
  deck.word = w; deck.rev = rev; deck.scratch = false; deck.freeze = false;
  deck.pos = rev ? bank[w].len - 2 : 0;
  deck.on = true; lastSrc = SRC_SAM;
}
void deckFreeze(int w, float at) {                   // hold a word: grains keep reading around one spot
  if (bankBusy || w < 0 || bank[w].len == 0) return;
  deck.word = w; deck.freeze = true; deck.scratch = false; deck.on = true;
  deck.freezePos = constrain(at, 0.0f, (float)bank[w].len - 2); lastSrc = SRC_SAM;
}
void deckScratchStart(int w) {
  if (bankBusy || w < 0 || bank[w].len == 0) return;
  deck.word = w; deck.scratch = true; deck.freeze = false; deck.on = true; deck.pos = deckTarget; lastSrc = SRC_SAM;
}

int quantStep() { return (curStep + (stepFrac > 0.5f ? 1 : 0)) & 15; }
void recVoice(uint8_t src, int w, uint8_t flags) {
  if (!rec || !playing) return;
  int q = quantStep();
  seq[q].word = w; seq[q].src = src; seq[q].flags = flags;
  if (q != curStep) seq[q].skip = true;
}
void recMach(int d) {
  if (!rec || !playing) return;
  int q = quantStep();
  seq[q].mach |= 1 << d;
  if (q != curStep) seq[q].skip = true;
}
void clearVoices() { for (auto& s : seq) { s.word = -1; s.flags = 0; } }
void clearMach() { for (auto& s : seq) s.mach = 0; }
void clearAll() { for (auto& s : seq) s = {-1, 0, 0, 0, false}; }
void mutate() { for (auto& s : seq) if (s.word >= 0) s.word = random(12); }
void autoPattern() {                                 // sparse: a heartbeat, a few machines, a word or two
  clearAll();
  seq[0].mach |= 1 << D_THUD;
  if (random(2)) seq[8].mach |= 1 << D_THUD;
  static const uint8_t POOL[9] = {D_KLANK, D_PIPE, D_HAMMR, D_GEIGR, D_RELAY, D_SONAR, D_CHAIN, D_DRIP, D_HISS};
  int hits = 3 + random(4);
  for (int h = 0; h < hits; h++) seq[random(16)].mach |= 1 << POOL[random(9)];
  uint8_t src = mode == M_TALK ? SRC_TALK : SRC_SAM;
  int words = 1 + random(2);
  for (int h = 0; h < words; h++) { int st = random(4) * 4 + 2; seq[st].word = random(12); seq[st].src = src; seq[st].flags = 0; }
  flash("AUTO");
}

// =====================================================================
// Keypad (scanned inside the audio task every 5.8 ms: low latency) + knob
// =====================================================================
ModKey mods[4];
bool padDown[12], padLong[12];
int8_t padMenu[12];       // which modifier was held when this pad went down (-1 = none)
int8_t padMode[12];       // mode when the pad went down (holds release by the mode they started in)
uint32_t padT[12];
uint16_t keysStable = 0, keysRaw = 0;
volatile int heldModVis = -1;
volatile uint16_t keysVis = 0;

uint16_t scanKeys() {
  uint16_t bits = 0;
  for (int r = 0; r < 4; r++) {
    pinMode(ROW_PINS[r], OUTPUT);
    digitalWrite(ROW_PINS[r], LOW);
    delayMicroseconds(20);
    for (int c = 0; c < 4; c++) if (!digitalRead(COL_PINS[c])) bits |= 1 << (r * 4 + c);
    pinMode(ROW_PINS[r], INPUT);
  }
  return bits;
}

int heldMod() {
  int best = -1; uint32_t t = 0;
  for (int m = 0; m < 4; m++) if (mods[m].down && (best < 0 || mods[m].tDown > t)) { best = m; t = mods[m].tDown; }
  return best;
}
void setMode(int m) { mode = m; flash(MODE_NAMES[m]); }

int padNote() { return 36 + humKey + humOct * 12; }

void menuAction(int m, int p, bool down) {
  char t[28];
  switch (m) {
    case K_MODE:
      if (!down) return;
      if (p < 4) setMode(p);
      else if (p == 4) { volIdx = max(0, volIdx - 1); snprintf(t, sizeof t, "VOLUME %d/6", volIdx + 1); flash(t); }
      else if (p == 5) { volIdx = min(5, volIdx + 1); snprintf(t, sizeof t, "VOLUME %d/6", volIdx + 1); flash(t); }
      else if (p == 6) { talkBank = (talkBank + 1) % NBANKS; snprintf(t, sizeof t, "BANK %s", BANK_NAMES[talkBank]); flash(t); }
      else if (p == 7) replayReq = true;
      else voiceIdx = p - 8;                       // loop() sees the change and re-renders the SAM words
      break;
    case K_FX:
      if (p < 7) { if (down) { fx = p; flash(FX_NAMES[p]); } }
      else if (p == 7) stutterOn = down;
      else if (p == 8) reverseOn = down;
      else if (p == 9) tapeOn = down;
      else if (p == 10) underOn = down;
      else { revFreeze = down; if (down) flash("INFINITE"); }
      break;
    case K_SEQ:
      if (!down) return;
      switch (p) {
        case 0: rec = !rec; flash(rec ? "REC" : "rec off"); break;
        case 1: playing = !playing; flash(playing ? "PLAY" : "STOP"); break;
        case 2: xtalk = !xtalk; flash(xtalk ? "CROSSTALK" : "crosstalk off"); if (xtalk && !playing) playing = true; break;
        case 3: autoPattern(); if (!playing) playing = true; break;
        case 4: bpm = max(40, bpm - 4); snprintf(t, sizeof t, "%d BPM", bpm); flash(t); break;
        case 5: bpm = min(140, bpm + 4); snprintf(t, sizeof t, "%d BPM", bpm); flash(t); break;
        case 6: mutate(); flash("MUTATE"); break;
        case 7: { static const char* DN[4] = {"off", "SPARSE", "BUSY", "FACTORY"}; machDens = (machDens + 1) % 4;
                  snprintf(t, sizeof t, "RANDOM %s", DN[machDens]); flash(t); if (machDens && !playing) playing = true; break; }
        case 8: clearVoices(); flash("voices cleared"); break;
        case 9: clearMach(); flash("machines cleared"); break;
        case 10: clearAll(); machDens = 0; xtalk = false; flash("all cleared"); break;
        default: humOn = 0; deck.on = false; lpc.on = false; for (auto& g : grains) g.on = false; rollBits = 0; flash("SILENCE"); break;
      }
      break;
    case K_SHIFT:
      if (mode == M_HUM) {
        if (!down) return;
        switch (p) {
          case 0: humOct = max(-1, humOct - 1); snprintf(t, sizeof t, "OCTAVE %+d", humOct); flash(t); break;
          case 1: humOct = min(1, humOct + 1); snprintf(t, sizeof t, "OCTAVE %+d", humOct); flash(t); break;
          case 2: humBright = max(0.0f, humBright - 0.2f); flash("DARKER"); break;
          case 3: humBright = min(1.0f, humBright + 0.2f); flash("BRIGHTER"); break;
          case 4: lfoRate = max(0.25f, lfoRate * 0.5f); flash("SLOWER"); break;
          case 5: lfoRate = min(4.0f, lfoRate * 2); flash("FASTER"); break;
          case 6: humDetune = humDetune > 0.01f ? 0.0015f : humDetune * 2; snprintf(t, sizeof t, "DETUNE %d", (int)(humDetune * 1000)); flash(t); break;
          case 7: humOn = 0; flash("DRONE OFF"); break;
          case 8: humOn = (1 << L_COUNT) - 1; flash("ALL LAYERS"); break;
          default: humOn = (1 << L_SUB) | (1 << L_PAD); flash("DRONE RESET"); break;
        }
      } else if (mode == M_SAM) {
        if (down) { curPad = p; deckPlay(p, true); recVoice(SRC_SAM, p, SF_REV); }
      } else if (mode == M_TALK) {
        if (p == 2) { lpFreeze = down; return; }
        if (!down) return;
        switch (p) {
          case 0: lpWhisper = !lpWhisper; flash(lpWhisper ? "WHISPER" : "whisper off"); break;
          case 1: lpMono = !lpMono; flash(lpMono ? "ROBOT MONO" : "mono off"); break;
          case 3: lpStretch = !lpStretch; flash(lpStretch ? "STRETCH" : "stretch off"); break;
          case 4: lpPitch = max(0.25f, lpPitch * 0.84f); flash("PITCH -"); break;
          case 5: lpPitch = min(4.0f, lpPitch * 1.19f); flash("PITCH +"); break;
          case 6: lpPitch = 1; flash("PITCH 0"); break;
          case 7: lpGlitch = !lpGlitch; flash(lpGlitch ? "GLITCH" : "glitch off"); break;
          case 8: lpRev = !lpRev; flash(lpRev ? "BACKWARDS" : "forwards"); break;
          case 9: lpFormant = max(0.5f, lpFormant * 0.89f); flash("FORMANT -"); break;
          case 10: lpFormant = min(2.0f, lpFormant * 1.12f); flash("FORMANT +"); break;
          default: lpWhisper = false; lpMono = false; lpStretch = false; lpGlitch = false; lpRev = false; lpPitch = 1; lpFormant = 1; flash("BENDS RESET"); break;
        }
      } else {
        if (!down) return;
        machMute ^= 1 << p;
        snprintf(t, sizeof t, "%s %s", PERC[p].name, machMute & (1 << p) ? "MUTED" : "on"); flash(t);
      }
      break;
  }
}

void tapAction(int m) {
  if (m == K_MODE) setMode((mode + 1) % M_COUNT);
  else if (m == K_FX) { fx = (fx + 1) % FX_COUNT; flash(FX_NAMES[fx]); }
  else if (m == K_SEQ) { rec = !rec; flash(rec ? "REC" : "rec off"); }
  else { playing = !playing; flash(playing ? "PLAY" : "STOP"); }
}

void humPad(int p) {
  char t[28];
  if (p < 8) { humOn ^= 1 << p; snprintf(t, sizeof t, "%s %s", LAYER_NAMES[p], humOn & (1 << p) ? "ON" : "off"); flash(t); }
  else if (p == 8) { driftOn = !driftOn; flash(driftOn ? "DRIFT" : "drift off"); }
  else if (p == 9) { humKey = (humKey + 11) % 12; snprintf(t, sizeof t, "KEY %s", NOTE[humKey]); flash(t); }
  else if (p == 10) { humKey = (humKey + 1) % 12; snprintf(t, sizeof t, "KEY %s", NOTE[humKey]); flash(t); }
  else { humChord = (humChord + 1) % 5; flash(CHORD_NAMES[humChord]); }
}

void padPress(int p, uint32_t now) {
  padDown[p] = true; padLong[p] = false; padT[p] = now;
  int m = heldMod();
  padMenu[p] = m;
  if (m >= 0) { mods[m].used = true; menuAction(m, p, true); return; }
  curPad = p; padMode[p] = mode;
  switch (mode) {
    case M_HUM: humPad(p); break;
    case M_SAM: deckPlay(p, reverseOn); recVoice(SRC_SAM, p, reverseOn ? SF_REV : 0); break;
    case M_TALK: talkPlay(p, reverseOn); recVoice(SRC_TALK, p, reverseOn ? SF_REV : 0); lastSrc = SRC_TALK; break;
    default: hitMach(1 << p); recMach(p); break;
  }
}

void padHold(int p) {
  padLong[p] = true;
  if (padMenu[p] >= 0) return;
  switch (padMode[p]) {
    case M_SAM:                                      // FREEZE where the word is right now (or its middle)
      freezeHeld = true;
      deckFreeze(p, deck.on && deck.word == p && !deck.freeze ? deck.pos : bank[p].len * 0.4f);
      break;
    case M_TALK:
      talkFreezeHeld = true; lpFreeze = true; lpMono = true; monoFreq = midiHz(padNote() + 12);
      if (!(lpc.on && lpc.word == p)) talkPlay(p, false);
      break;
    case M_MACH: rollBits |= 1 << p; break;
    default: break;
  }
}

void padRelease(int p, uint32_t now) {
  padDown[p] = false;
  if (padMenu[p] >= 0) { menuAction(padMenu[p], p, false); return; }
  if (!padLong[p]) return;
  switch (padMode[p]) {
    case M_SAM: if (freezeHeld) { freezeHeld = false; deck.freeze = false; deck.on = false; } break;
    case M_TALK: if (talkFreezeHeld) { talkFreezeHeld = false; lpFreeze = false; lpMono = false; lpc.on = false; } break;
    case M_MACH: rollBits &= ~(1 << p); break;
    default: break;
  }
}

// the knob: alone = FX amount, SHIFT = scratch, FX = space
void knobTick(uint32_t now) {
  static float potS = -1, potMax = 2700, ref = -1;
  int raw = 0;                                       // 8 reads averaged: knocks the ADC noise down ~3x
  for (int i = 0; i < 8; i++) raw += analogRead(PIN_POT);
  raw >>= 3;
  if (potS < 0) potS = raw;
  potS += (raw - potS) * 0.15f;
  if (potS > potMax) potMax = min(4095.0f, potS);
  float pv = constrain(potS / potMax, 0.0f, 1.0f);
  knobVal = pv;
  if (ref < 0) { ref = pv; return; }
  if (fabsf(pv - ref) < 0.025f) return;              // deadband: small wobble never counts as a turn
  ref = pv;
  char t[28];
  if (mods[K_SHIFT].down) {
    mods[K_SHIFT].used = true;
    if (lastSrc == SRC_TALK || mode == M_TALK) {
      const TalkWord& w = talk[talkBank][curPad];
      lpcTarget = pv * (w.n > 1 ? w.n - 1 : 0);
      if (!knobScratch || !lpc.scratch) { lpc.bank = talkBank; lpc.word = curPad; lpc.scratch = true; lpc.fpos = lpcTarget; lpc.loaded = -1; lpc.on = true; lastSrc = SRC_TALK; }
    } else {
      deckTarget = pv * max(0.0f, (float)bank[curPad].len - 2);
      if (!knobScratch || !deck.scratch) deckScratchStart(curPad);
    }
    knobScratch = true;
  } else if (mods[K_FX].down) {
    mods[K_FX].used = true;
    space = pv; snprintf(t, sizeof t, "SPACE %d%%", (int)(pv * 100)); flash(t);
  } else {
    fxAmt[fx] = pv; snprintf(t, sizeof t, "%s %s %d%%", FX_NAMES[fx], FX_KNOB[fx], (int)(pv * 100)); flash(t);
  }
}

void inputTick() {
  uint32_t now = millis();
  uint16_t raw = scanKeys();
  // asymmetric debounce: down after 2 scans (~12 ms), up only after 8 scans (~46 ms) - rides over flaky contacts
  static uint8_t upCnt[16], dnCnt[16];
  uint16_t stable = keysStable;
  for (int k = 0; k < 16; k++) {
    bool r = raw & (1 << k);
    if (r) { upCnt[k] = 0; if (dnCnt[k] < 255) dnCnt[k]++; if (dnCnt[k] >= 2) stable |= 1 << k; }
    else { dnCnt[k] = 0; if (upCnt[k] < 255) upCnt[k]++; if (upCnt[k] >= 8) stable &= ~(1 << k); }
  }
  keysRaw = raw;
  uint16_t pressed = stable & ~keysStable, released = keysStable & ~stable;
  keysStable = stable;
  keysVis = stable;
  for (int r = 0; r < 4; r++) {
    int bit = 1 << (r * 4);
    if (pressed & bit) { mods[r].down = true; mods[r].used = false; mods[r].tDown = now; }
    if (released & bit) {
      mods[r].down = false;
      if (r == K_SHIFT && knobScratch) {             // let go of the record
        knobScratch = false; deck.scratch = false; deck.on = false; lpc.scratch = false; lpc.on = false;
      }
      if (!mods[r].used && now - mods[r].tDown < 500) tapAction(r);
    }
    for (int c = 1; c < 4; c++) {
      int b = 1 << (r * 4 + c), p = r * 3 + c - 1;
      if (pressed & b) padPress(p, now);
      if (released & b) padRelease(p, now);
    }
  }
  for (int p = 0; p < 12; p++) if (padDown[p] && !padLong[p] && now - padT[p] > 300) padHold(p);
  static bool eggFired = false;
  if (mods[K_MODE].down && mods[K_SHIFT].down) {
    if (!eggFired && now - max(mods[K_MODE].tDown, mods[K_SHIFT].tDown) > 900) {
      eggFired = true; eggReq = true; mods[K_MODE].used = mods[K_SHIFT].used = true;
    }
  } else eggFired = false;
  int m = heldMod();
  heldModVis = (m >= 0 && now - mods[m].tDown > 180 && !knobScratch) ? m : -1;
  knobTick(now);
}

// =====================================================================
// Audio
// =====================================================================
// ---- vocoder (from Robo Choir) ----
#define NB 16
struct VOsc { float ph = 0, ph2 = 0, f = 110, target = 110; bool on = false; };
VOsc vosc[5];
float coefM[NB][5], coefC[NB][5], wM[NB][2], wC[NB][2], bandEnv[NB];
float vBuf[BLOCK], dBuf[BLOCK], gBuf[BLOCK], carBuf[BLOCK], noiseBuf[BLOCK], tmpM[BLOCK], tmpC[BLOCK], tmpC2[BLOCK], vocOut[BLOCK];
float vocAgc = 6;

void setupVocoder() {
  for (int b = 0; b < NB; b++) {
    float f = 150.0f * powf(5500.0f / 150.0f, b / (float)(NB - 1));
    dsps_biquad_gen_bpf_f32(coefM[b], f / FS, 5.0f);
    dsps_biquad_gen_bpf_f32(coefC[b], f / FS, 5.0f);
  }
}



static inline float blep(float t, float dt) {
  if (t < dt) { t /= dt; return t + t - t * t - 1; }
  if (t > 1 - dt) { t = (t - 1) / dt; return t * t + t + t + 1; }
  return 0;
}

// tbox = TALKBOX: the guitar (gBuf) is the carrier instead of the synth chord
void vocodeBlock(bool tbox) {
  for (int n = 0; n < BLOCK; n++) {
    if (tbox) carBuf[n] = gBuf[n] * 2.0f;
    else {
      float c = 0;
      for (int k = 0; k < 5; k++) {
        VOsc& o = vosc[k];
        if (!o.on) continue;
        o.f += (o.target - o.f) * 0.0015f;
        float dt = o.f * INV_FS, dt2 = dt * 1.007f;
        o.ph += dt; if (o.ph >= 1) o.ph -= 1;
        o.ph2 += dt2; if (o.ph2 >= 1) o.ph2 -= 1;
        c += (2 * o.ph - 1 - blep(o.ph, dt) + 2 * o.ph2 - 1 - blep(o.ph2, dt2)) * (k == 4 ? 0.8f : 0.5f);
      }
      carBuf[n] = softclip(c * 0.7f) * 1.6f;          // driven carrier: grittier robot
    }
    noiseBuf[n] = nz();
  }
  memset(vocOut, 0, sizeof vocOut);
  for (int b = 0; b < NB; b++) {
    dsps_biquad_f32(vBuf, tmpM, BLOCK, coefM[b], wM[b]);
    if (b >= NB - 3) for (int n = 0; n < BLOCK; n++) tmpC[n] = carBuf[n] * 0.4f + noiseBuf[n] * 1.2f;
    else memcpy(tmpC, carBuf, sizeof tmpC);
    dsps_biquad_f32(tmpC, tmpC2, BLOCK, coefC[b], wC[b]);
    float env = bandEnv[b];
    for (int n = 0; n < BLOCK; n++) {
      float a = fabsf(tmpM[n]);
      env += (a - env) * (a > env ? 0.02f : 0.0025f);
      vocOut[n] += tmpC2[n] * env;
    }
    bandEnv[b] = env;
  }
  float pk = 0.0001f;
  for (int n = 0; n < BLOCK; n++) pk = max(pk, fabsf(vocOut[n]));
  float want = min(40.0f, 0.55f / pk);
  if (want < vocAgc) vocAgc += (want - vocAgc) * 0.3f;
  else if (pk > 0.002f) vocAgc += (want - vocAgc) * 0.01f;
  for (int n = 0; n < BLOCK; n++) vBuf[n] = vocOut[n] * vocAgc + (tbox ? 0 : vBuf[n] * 0.45f);
}

// the vocoder sings the drone chord
void vocChord() {
  int root = padNote();
  for (int k = 0; k < 4; k++) {
    int n = root + 12 + CHORD_IV[humChord][k] + (driftOn ? DRIFT_STEPS[driftPos] : 0);
    while (n < 50) n += 12;
    while (n > 70) n -= 12;
    vosc[k].target = midiHz(n);
    if (!vosc[k].on) vosc[k].f = vosc[k].target;
    vosc[k].on = true;
  }
  vosc[4].target = midiHz(root + (driftOn ? DRIFT_STEPS[driftPos] : 0));
  if (!vosc[4].on) vosc[4].f = vosc[4].target;
  vosc[4].on = true;
}

void spawnGrain(int word, float at, float spray, bool mixed) {
  if (word < 0 || bank[word].len == 0) return;
  for (auto& g : grains) if (!g.on) {
    static const float GP[6] = {1.0f, 1.0f, 0.5f, 1.5f, 2.0f, 0.75f};
    g.word = word;
    g.len = FS * (0.06f + 0.06f * random(100) / 100.0f);
    g.pos = constrain(at + (random(200) - 100) / 100.0f * FS * spray, 0.0f, (float)bank[word].len - 2);
    g.inc = mixed ? GP[random(6)] : 1.0f + (random(200) - 100) / 100.0f * 0.006f;
    g.amp = mixed ? 0.7f : 0.55f;
    g.age = 0; g.on = true;
    return;
  }
}
static inline float grainSum() {
  float v = 0;
  for (auto& g : grains) {
    if (!g.on) continue;
    float w = fsin(g.age / g.len * 0.5f);
    v += wordAt(g.word, g.pos) * w * w * g.amp;
    g.pos += g.inc * tapeRate;
    if (++g.age >= g.len) g.on = false;
  }
  return v;
}

static inline float svfBP(SVF& s, float x, float f, float q) { s.lo += f * s.bp; float hi = x - s.lo - q * s.bp; s.bp += f * hi; return s.bp; }
static inline float svfLP(SVF& s, float x, float f, float q) { s.lo += f * s.bp; float hi = x - s.lo - q * s.bp; s.bp += f * hi; return s.lo; }
static inline float blepSaw(float& ph, float dt) { ph += dt; if (ph >= 1) ph -= 1; return 2 * ph - 1 - blep(ph, dt); }

void setupAmp() {
  amp.setPins(AMP_BCLK, AMP_LRC, AMP_DIN);
  bool ok = amp.begin(I2S_MODE_STD, FS, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
  logf("I2S %s\n", ok ? "OK" : "FAILED");
}
volatile uint32_t audioBlocks = 0;
TaskHandle_t audioHandle = nullptr;
volatile bool bootVoiceReq = false, bootTalkReq = false;
extern volatile bool bootFreezeStopReq;

void audioTask(void*) {
  static int16_t out[BLOCK * 2];
  float stepPos = 0, underLP = 0, under = 0, deckLP = 0, gateSm = 1, compEnv = 0, subLP = 0, scrGate = 0, lastPos = 0;
  float radLP = 0, deepLP = 0, lfo1 = 0, lfo2 = 0, mainsPh = 0, subPh = 0, metPh = 0, metPh2 = 0, metPh3 = 0;
  float sawPh[8] = {0}, chF[4] = {65, 98, 131, 196}, windF = 600, windT = 600, radF = 1200, radT = 1200, crack = 0, xtTurn = 0;
  SVF padF, chF1, chF2, windS, radS, radS2;
  float lvl[L_COUNT] = {0}, gain[L_COUNT] = {0};
  Perc pulse;
  const PercDef PULSE_DEF = {"PULSE", W_KICK, 75, 42, 0.07f, 0.7f, 0.55f, 0.05f, 1.2f};
  const float duckD = dcoef(0.2f);
  float stutterBase = -1, lpcStutter = -1;
  int grainTimer = 0, lastBeat = -1;
  for (;;) {
    inputTick();
    if (bootVoiceReq && nameReady && bank[12].len) {   // SAM says the name
      bootVoiceReq = false; deck.word = 12; deck.pos = 0; deck.rev = false; deck.freeze = false; deck.scratch = false; deck.on = true;
    }
    if (bootFreezeStopReq) { bootFreezeStopReq = false; if (!freezeHeld && deck.freeze) { deck.freeze = false; deck.on = false; } }
    if (bootTalkReq) { bootTalkReq = false; lpc.bank = 3; lpc.word = 10; lpc.rev = false; lpc.scratch = false; lpc.fpos = 0; lpc.loaded = -1; lpc.on = talk[3][10].n > 0; }   // Talkie: "OPERATIONAL"
    tapeRate += ((tapeOn ? 0.0f : 1.0f) - tapeRate) * (tapeOn ? 0.03f : 0.08f);
    under += ((underOn ? 1.0f : 0.0f) - under) * 0.05f;
    masterFade = min(1.0f, masterFade + BLOCK * INV_FS / 4.0f);
    float stepLen = FS * 60.0f / bpm / 4.0f;
    float master = VOLS[volIdx] * masterFade;
    bool busy = bankBusy;
    if (busy && !(deck.word == 12 && nameReady)) { deck.on = false; for (auto& g : grains) g.on = false; }
    if (!stutterOn) { stutterBase = -1; lpcStutter = -1; }
    float pitchK = fx == FX_CLEAN ? 0.5f + fxAmt[FX_CLEAN] : fx == FX_DEEP ? 0.75f - 0.4f * fxAmt[FX_DEEP] : 1.0f;
    float lpcInc = 8000.0f * lpFormant * tapeRate * INV_FS * (fx == FX_DEEP ? pitchK : 1.0f);

    // ---- drone control rate ----
    float sp = space;
    float lfoInc = BLOCK * INV_FS * lfoRate;
    lfo1 = wrap1(lfo1 + 0.07f * lfoInc); lfo2 = wrap1(lfo2 + 0.031f * lfoInc);
    float l1 = fsin(lfo1), l2 = fsin(lfo2);
    int root = padNote() + (driftOn ? DRIFT_STEPS[driftPos] : 0);
    float target[4];
    for (int k = 0; k < 4; k++) target[k] = midiHz(root + CHORD_IV[humChord][k]);
    uint16_t on = humOn;
    for (int k = 0; k < L_COUNT; k++) gain[k] += (((on >> k) & 1 ? LAYER_GAIN[k] : 0) - gain[k]) * 0.02f;   // ~0.3 s fades
    float padCut = 2 * sinf(PI * min(4000.0f, (260 + 1500 * humBright + 900 * sp) * (1 + 0.45f * l1)) * INV_FS);
    float vowel = 0.5f + 0.5f * l2;                  // "ah" <-> "oo"
    float f1c = 2 * sinf(PI * (720 - 380 * vowel) * INV_FS), f2c = 2 * sinf(PI * (1150 - 350 * vowel) * INV_FS);
    if (random(1000) < 6) windT = 250 + random(1900);
    windF += (windT - windF) * 0.01f;
    float wF = 2 * sinf(PI * windF * INV_FS);
    if (random(1000) < 4) radT = 400 + random(2600);
    radF += (radT - radF) * 0.004f;
    float radInc = radF * INV_FS, rF = 2 * sinf(PI * 1800 * INV_FS);
    float metIdx = 1.4f + 1.2f * l2, metF = midiHz(root + 24 + humOct * 0);
    // reverb from the SPACE knob
    float revFb = revFreeze ? 0.999f : 0.78f + 0.19f * sp, revDamp = 0.55f - 0.3f * sp, revIn = revFreeze ? 0.0f : 1.0f;
    float sendHum = 0.35f + 0.5f * sp, sendVox = 0.25f + 0.6f * sp, sendMach = 0.3f + 0.5f * sp;
    float wet = 0.35f + 0.5f * sp, dry = 1.0f - 0.3f * sp;
    float echoFb = 0.25f + 0.65f * fxAmt[FX_ECHO];
    float radCtr = 2 * sinf(PI * (300 + 3200 * fxAmt[FX_RADIO]) * INV_FS);
    float blockLvl[L_COUNT] = {0};

    for (int n = 0; n < BLOCK; n++) {
      // ---- 16th-note clock (always runs; the sequencer only fires while playing) ----
      stepPos += tapeRate;
      if (stepPos >= stepLen) {
        stepPos -= stepLen;
        int s = (curStep + 1) & 15;
        curStep = s;
        Step& st = seq[s];
        uint16_t d = 0;
        if (s == 0) {
          static int bar = 0;
          bar++;
          if (driftOn && (bar & 1) == 0) driftPos = (driftPos + 1) & 3;
          vocChord();
          if (playing && xtalk && random(10) < 7 && !busy) {      // the two mouths take turns
            xtTurn = 1 - xtTurn;
            if (xtTurn > 0.5f) { int w = random(12); deckPlay(w, false); curPad = w; }
            else { talkPlay(random(12), false); lastSrc = SRC_TALK; }
          }
        }
        if ((s & 3) == 0 && (humOn & (1 << L_PULSE))) pulse.trig(PULSE_DEF, 1);
        if (playing) {
          d = st.skip ? 0 : st.mach;
          static const uint8_t DENS[4] = {0, 6, 14, 30};
          if (machDens && random(100) < DENS[machDens]) d |= 1 << random(11);   // random machines (no THUD)
          d &= ~machMute;
          if (!st.skip && !busy && st.word >= 0) {
            if (st.src == SRC_SAM && !freezeHeld && !knobScratch) deckPlay(st.word, st.flags & SF_REV);
            else if (st.src == SRC_TALK && !talkFreezeHeld && !knobScratch) { talkPlay(st.word, st.flags & SF_REV); lastSrc = SRC_TALK; }
          }
        }
        d |= rollBits;
        if (rollBits && rec && playing) seq[s].mach |= rollBits;
        hitMach(d);
        st.skip = false;
        if (stutterOn && !busy) {
          if (deck.word >= 0 && !deck.freeze) {
            if (stutterBase < 0) stutterBase = deck.on ? deck.pos : 0;
            deck.pos = stutterBase; deck.on = true; deck.scratch = false;
          }
          if (lpc.word >= 0) {
            if (lpcStutter < 0) lpcStutter = lpc.on ? lpc.fpos : 0;
            lpc.fpos = lpcStutter; lpc.loaded = -1; lpc.on = true;
          }
        }
      }
      stepFrac = stepPos / stepLen;

      // ---- SAM deck ----
      float v = 0, gTarget = 1;
      if (deck.on && deck.word >= 0) {
        const Word& w = bank[deck.word];
        if (deck.freeze) {
          if (--grainTimer <= 0) { spawnGrain(deck.word, deck.freezePos, 0.02f, false); grainTimer = FS / 70; }
        } else if (deck.scratch) {
          lastPos = deck.pos;
          deck.pos += (deckTarget - deck.pos) * 0.004f;
          float vel = fabsf(deck.pos - lastPos);
          scrGate += (min(1.0f, vel * 3) - scrGate) * 0.01f;
          gTarget = scrGate;
          v = wordAt(deck.word, deck.pos) * 1.3f;
        } else {
          deck.pos += (deck.rev ? -1 : 1) * pitchK * tapeRate;
          if (deck.pos < 0 || deck.pos >= w.len - 1) deck.on = false;
          if (stutterOn && stutterBase >= 0 && deck.pos > stutterBase + stepLen * 0.5f) gTarget = 0;
          if (fx == FX_GRAIN) {
            if (--grainTimer <= 0) { spawnGrain(deck.word, deck.pos, 0.03f, true); grainTimer = (int)(FS / (20 + 120 * fxAmt[FX_GRAIN])); }
          } else v = wordAt(deck.word, deck.pos) * 1.3f;
        }
      }
      v += grainSum();
      gateSm += (gTarget - gateSm) * 0.25f;
      v *= gateSm;
      deckLP += 0.6f * (v - deckLP);
      v = deckLP;

      // ---- Talkie LPC (8 kHz engine, interpolated up) ----
      if (lpc.on) {
        if (lpc.scratch) {
          float prev = lpc.fpos;
          lpc.fpos += (lpcTarget - lpc.fpos) * 0.003f;
          scrGate += (min(1.0f, fabsf(lpc.fpos - prev) * 600) - scrGate) * 0.01f;
        }
        lpc.ph += lpcInc;
        while (lpc.ph >= 1) { lpc.ph -= 1; lpc.y0 = lpc.y1; lpc.y1 = lpcTick(); }
        float lv = lpc.y0 + (lpc.y1 - lpc.y0) * lpc.ph;
        v += lv * (lpc.scratch ? scrGate : 1.0f) * 2.5f;
      }

      // ---- voice FX ----
      if (fx == FX_RADIO) {                            // bandpass on the knob, driven, with crackle
        float b = svfBP(radS, v + nz() * 0.03f, radCtr, 0.5f);
        v = softclip(b * 3.0f) * 0.6f;
      } else if (fx == FX_DEEP) {
        deepLP += 0.25f * (v - deepLP); v = deepLP * 1.3f;
      }
      vBuf[n] = v;

      // ---- the drone ----
      float hum = 0;
      for (int k = 0; k < 4; k++) chF[k] += (target[k] - chF[k]) * 0.0004f;
      if (gain[L_PAD] > 0.001f || gain[L_CHOIR] > 0.001f) {
        float saws = 0;
        for (int k = 0; k < 4; k++) {
          float dt = chF[k] * INV_FS;
          saws += blepSaw(sawPh[2 * k], dt * (1 + humDetune)) + blepSaw(sawPh[2 * k + 1], dt * (1 - humDetune));
        }
        saws *= 0.18f;
        float pd = svfLP(padF, saws, padCut, 0.9f) * gain[L_PAD];
        float ch = (svfBP(chF1, saws, f1c, 0.18f) + 0.6f * svfBP(chF2, saws, f2c, 0.2f)) * gain[L_CHOIR] * 1.4f;
        hum += pd + ch; blockLvl[L_PAD] = max(blockLvl[L_PAD], fabsf(pd)); blockLvl[L_CHOIR] = max(blockLvl[L_CHOIR], fabsf(ch));
      }
      if (gain[L_MAINS] > 0.001f) {                    // transformer hum + buzz, tuned to the root
        mainsPh = wrap1(mainsPh + chF[0] * INV_FS);
        float s = fsin(mainsPh) + 0.5f * fsin(wrap1(mainsPh * 2)) + 0.3f * fsin(wrap1(mainsPh * 3));
        float m = (s * 0.5f + softclip(s * 2.5f) * 0.4f) * gain[L_MAINS];
        hum += m; blockLvl[L_MAINS] = max(blockLvl[L_MAINS], fabsf(m));
      }
      if (gain[L_SUB] > 0.001f) {
        subPh = wrap1(subPh + chF[0] * 0.5f * INV_FS);
        float s = (fsin(subPh) * (0.8f + 0.2f * l1) + 0.35f * fsin(wrap1(subPh * 2))) * gain[L_SUB];
        hum += s; blockLvl[L_SUB] = max(blockLvl[L_SUB], fabsf(s));
      }
      if (gain[L_METAL] > 0.001f) {                    // inharmonic FM: a struck girder that never stops
        metPh = wrap1(metPh + metF * INV_FS); metPh2 = wrap1(metPh2 + metF * 1.414f * INV_FS); metPh3 = wrap1(metPh3 + metF * 2.76f * INV_FS);
        float m = (fsin(wrap1(metPh + metIdx * fsin(metPh2) * 0.3f)) + 0.35f * fsin(metPh3)) * gain[L_METAL];
        hum += m; blockLvl[L_METAL] = max(blockLvl[L_METAL], fabsf(m));
      }
      if (gain[L_WIND] > 0.001f) {
        float w = svfBP(windS, nz(), wF, 0.12f) * gain[L_WIND] * 0.6f;
        hum += w; blockLvl[L_WIND] = max(blockLvl[L_WIND], fabsf(w));
      }
      if (gain[L_RADIO] > 0.001f) {                    // shortwave: band noise, a drifting heterodyne, crackle
        if (random(4000) == 0) crack = 1;
        crack *= 0.9993f;
        radLP = wrap1(radLP + radInc);
        float r = (svfBP(radS2, nz(), rF, 0.4f) * (0.25f + 1.5f * crack * (nz() > 0.6f)) + 0.25f * fsin(radLP)) * gain[L_RADIO];
        hum += r; blockLvl[L_RADIO] = max(blockLvl[L_RADIO], fabsf(r));
      }
      float pl = pulse.s(PULSE_DEF, 1) * gain[L_PULSE];
      hum += pl; blockLvl[L_PULSE] = max(blockLvl[L_PULSE], fabsf(pl));
      gBuf[n] = hum;

      float dr = 0;
      for (int i = 0; i < 12; i++) dr += perc[i].s(PERC[i], tapeRate);
      dBuf[n] = dr * 0.7f;
    }

    if (fx == FX_VOX || fx == FX_CHOIR) {
      float keep[BLOCK];
      memcpy(keep, vBuf, sizeof keep);
      vocodeBlock(false);
      float wetV = fx == FX_VOX ? fxAmt[FX_VOX] : 1.0f;
      for (int n = 0; n < BLOCK; n++) {
        float vv = vBuf[n] * wetV + keep[n] * (fx == FX_VOX ? (1 - wetV) * 1.4f : 0);
        if (fx == FX_CHOIR) vv += nz() * fxAmt[FX_CHOIR] * 0.05f * fabsf(vBuf[n]) * 6;
        vBuf[n] = vv;
      }
    }
    for (int k = 0; k < L_COUNT; k++) { lvl[k] = max(blockLvl[k], lvl[k] * 0.92f); humLvl[k] = lvl[k]; }

    float blockPeak = 0;
    for (int n = 0; n < BLOCK; n++) {
      float v = vBuf[n], hum = gBuf[n], dr = dBuf[n];
      float echo = 0;
      if (dlyBuf) {
        int dl = (int)(stepLen * 3);
        if (dl >= DLY_N) dl = DLY_N - 1;
        int r = dlyW - dl; if (r < 0) r += DLY_N;
        echo = dlyBuf[r];
        dlyBuf[dlyW] = (fx == FX_ECHO ? v : 0) + echo * echoFb;
        if (++dlyW >= DLY_N) dlyW = 0;
      }
      float duck = 1.0f - 0.35f * duckEnv;
      duckEnv *= duckD;
      float vox = v * 0.9f + echo * 0.6f;
      // reverb
      float in = (hum * sendHum + vox * sendVox + dr * sendMach) * revIn;
      for (int k = 0; k < 4; k++) rlp[k] += revDamp * (rv[k][ri[k]] - rlp[k]);
      float h0 = (rlp[0] + rlp[1] + rlp[2] + rlp[3]) * 0.5f, h1 = (rlp[0] - rlp[1] + rlp[2] - rlp[3]) * 0.5f;
      float h2 = (rlp[0] + rlp[1] - rlp[2] - rlp[3]) * 0.5f, h3 = (rlp[0] - rlp[1] - rlp[2] + rlp[3]) * 0.5f;
      rv[0][ri[0]] = in + revFb * h0; rv[1][ri[1]] = in + revFb * h1; rv[2][ri[2]] = in + revFb * h2; rv[3][ri[3]] = in + revFb * h3;
      for (int k = 0; k < 4; k++) if (++ri[k] >= RL[k]) ri[k] = 0;
      float rev = (rlp[0] + rlp[1] + rlp[2] + rlp[3]) * 0.25f;
      float x = (hum * duck + vox + dr) * dry + rev * wet * 1.4f;
      subLP += 0.0113f * (x - subLP);                 // ~40 Hz highpass: sub the speaker can't play
      x -= subLP;
      underLP += (1.0f - under * 0.95f) * (x - underLP);
      float a = fabsf(underLP);
      compEnv += (a - compEnv) * (a > compEnv ? 0.015f : 0.00025f);
      float gr = compEnv > 0.5f ? powf(0.5f / compEnv, 0.667f) : 1.0f;
      float y = softclip(underLP * gr * 1.35f);
      scopeBuf[n] = y;
      if (fabsf(y) > blockPeak) blockPeak = fabsf(y);
      int16_t s = (int16_t)(y * master * 32000);
      out[2 * n] = s; out[2 * n + 1] = s;
    }
    outPeak = blockPeak;
    deckVis = deck.on ? (deck.freeze ? deck.freezePos : deck.pos) : -1;
    lpcVisFrame = lpc.on ? lpc.loaded : -1;
    amp.write((uint8_t*)out, sizeof(out));
    audioBlocks++;
  }
}

// =====================================================================
// Visuals
// =====================================================================
void brackets(int x, int y, int w, int h) {
  const int l = 5;
  oled.drawHLine(x, y, l);             oled.drawVLine(x, y, l);
  oled.drawHLine(x + w - l, y, l);     oled.drawVLine(x + w - 1, y, l);
  oled.drawHLine(x, y + h - 1, l);     oled.drawVLine(x, y + h - l, l);
  oled.drawHLine(x + w - l, y + h - 1, l); oled.drawVLine(x + w - 1, y + h - l, l);
}
int tab(int x, int y, const char* t) {
  oled.setFont(u8g2_font_4x6_tf);
  int w = oled.getStrWidth(t) + 4;
  oled.drawBox(x, y, w, 7);
  oled.setDrawColor(0); oled.drawStr(x + 2, y + 6, t); oled.setDrawColor(1);
  return w;
}
void dottedH(int x, int y, int w) { for (int i = 0; i < w; i += 2) oled.drawPixel(x + i, y); }
const char* padName(int m, int p) {
  static char nb[4][16];
  static int k = 0;
  char* b = nb[k = (k + 1) & 3];
  switch (m) {
    case M_HUM:
      if (p < 8) return LAYER_NAMES[p];
      if (p == 8) return "DRIFT";
      if (p == 9) return "KEY-";
      if (p == 10) return "KEY+";
      snprintf(b, 16, "%s", CHORD_SUF[humChord]); return b;
    case M_SAM: return bank[p].text;
    case M_TALK: return talk[talkBank][p].name;
    default: return PERC[p].name;
  }
}
void shortName(const char* s, char* out, int n) {
  int j = 0;
  for (int i = 0; s[i] && j < n; i++) if (s[i] != ' ' && s[i] != '\'') out[j++] = s[i];
  out[j] = 0;
}
void chordName(char* out, size_t n) {
  int r = (humKey + (driftOn ? DRIFT_STEPS[driftPos] : 0)) % 12;
  snprintf(out, n, "%s%s", NOTE[r], CHORD_SUF[humChord]);
}

void drawTopBar() {
  int w = tab(0, 0, "BLASTEROID");
  oled.setFont(u8g2_font_4x6_tf);
  char t[12]; snprintf(t, sizeof t, "%d", bpm);
  oled.drawStr(w + 3, 6, t);
  int bx = w + 5 + oled.getStrWidth(t);
  int beat = curStep / 4;
  for (int i = 0; i < 4; i++) { if (playing && i == beat) oled.drawBox(bx + i * 5, 1, 4, 4); else oled.drawFrame(bx + i * 5, 1, 4, 4); }
  const char* mn = MODE_NAMES[mode];
  int mw = oled.getStrWidth(mn) + 4;
  oled.drawFrame(128 - mw, 0, mw, 7);
  oled.drawStr(128 - mw + 2, 6, mn);
  dottedH(0, 8, 128);
}

void drawSigPane() {
  const int x = 0, y = 11, w = 54, h = 40, mid = y + 22;
  brackets(x, y, w, h);
  tab(x + 2, y + 2, "SIG");
  int py = mid;
  for (int i = 0; i < 44; i++) {
    float v = scopeBuf[i * 128 / 44];
    int yy = constrain(mid - (int)(v * 14), y + 9, y + h - 4);
    if (i) oled.drawLine(x + 3 + i, py, x + 4 + i, yy);
    py = yy;
  }
  static float lvl = 0;
  lvl = max((float)outPeak, lvl * 0.9f);
  int segs = (int)(lvl * 8 + 0.5f);
  for (int i = 0; i < 8; i++) {
    int sy = y + h - 5 - i * 4;
    if (i < segs) oled.drawBox(x + w - 5, sy, 3, 3); else oled.drawPixel(x + w - 4, sy + 1);
  }
}
void drawPadPane() {                                 // the 12 pads as they sit on the keypad
  const int x0 = 56, y0 = 11, cw = 24, ch = 10;
  uint16_t k = keysVis;
  oled.setFont(u8g2_font_4x6_tf);
  for (int p = 0; p < 12; p++) {
    int r = p / 3, c = p % 3, x = x0 + c * cw, y = y0 + r * ch;
    bool down = k & (1 << (r * 4 + c + 1));
    bool active = (mode == M_HUM && ((p < 8 && (humOn & (1 << p))) || (p == 8 && driftOn))) ||
                  (mode == M_MACH && perc[p].env > 0.05f) ||
                  (mode == M_SAM && deck.on && deck.word == p) || (mode == M_TALK && lpc.on && lpc.word == p && lpc.bank == talkBank);
    char nm[8]; shortName(padName(mode, p), nm, 5);
    if (down || active) { oled.drawBox(x, y, cw - 1, ch - 1); oled.setDrawColor(0); }
    else oled.drawFrame(x, y, cw - 1, ch - 1);
    if (mode == M_MACH && (machMute & (1 << p))) oled.drawLine(x + 1, y + ch - 3, x + cw - 3, y + 1);
    oled.drawStr(x + 2, y + 7, nm);
    oled.setDrawColor(1);
  }
}

float envStrip[120];
int envWord = -2;
void buildEnvStrip(int w) {
  envWord = w;
  for (int k = 0; k < 120; k++) envStrip[k] = 0;
  if (w < 0 || bank[w].len == 0) return;
  uint32_t len = bank[w].len;
  for (int k = 0; k < 120; k++) {
    uint32_t a = len * k / 120, b = len * (k + 1) / 120;
    int pk = 0;
    for (uint32_t i = a; i < b; i += 4) pk = max(pk, abs((int)pool[bank[w].off + i]));
    envStrip[k] = pk / 128.0f;
  }
}
void drawVizStrip() {                                // y 53..72: mode-specific view
  const int y = 53, h = 20, mid = y + 12;
  char lab[24];
  oled.setFont(u8g2_font_4x6_tf);
  if (mode == M_HUM) {                               // layer meters
    for (int i = 0; i < L_COUNT; i++) {
      int x = 3 + i * 12, e = constrain((int)(humLvl[i] * 40), 0, 12);
      bool on = humOn & (1 << i);
      oled.drawFrame(x, y + 2, 9, 14);
      if (e) oled.drawBox(x + 2, y + 15 - e, 5, e); else if (on) oled.drawHLine(x + 2, y + 14, 5);
      if (!on) oled.drawLine(x, y + 15, x + 8, y + 2);
    }
    chordName(lab, sizeof lab);
  } else if (mode == M_SAM) {
    int wd = deck.on ? deck.word : curPad;
    if (wd != envWord) buildEnvStrip(wd);
    for (int k = 0; k < 120; k++) { int a = (int)(envStrip[k] * 7); if (a) oled.drawVLine(4 + k, mid - a, a * 2 + 1); else oled.drawPixel(4 + k, mid); }
    float pos = deckVis;
    if (pos >= 0 && wd >= 0 && bank[wd].len) {
      int px = 4 + (int)(pos / bank[wd].len * 119);
      oled.setDrawColor(2); oled.drawVLine(px, y + 2, h - 2); oled.setDrawColor(1);
    }
    snprintf(lab, sizeof lab, "%s", knobScratch ? "SCRATCH" : deck.freeze ? "FREEZE" : VOICES[voiceIdx].name);
  } else if (mode == M_TALK) {
    int fi = lpcVisFrame;
    if (fi >= 0) {
      int32_t kk[10] = {lpc.k1 >> 8, lpc.k2 >> 8, lpc.k[0], lpc.k[1], lpc.k[2], lpc.k[3], lpc.k[4], lpc.k[5], lpc.k[6], lpc.k[7]};
      for (int i = 0; i < 10; i++) {
        int a = constrain((int)(kk[i] * 8 / 128), -8, 8), bx = 30 + i * 8;
        if (a > 0) oled.drawBox(bx, mid - a, 6, a); else if (a < 0) oled.drawBox(bx, mid, 6, -a); else oled.drawHLine(bx, mid, 6);
      }
      int e = constrain((int)(lpc.energy * 18 / 255), 0, 18);
      oled.drawFrame(116, y + 1, 8, 19); oled.drawBox(118, y + 19 - e, 4, e);
      const TalkWord& t = talk[lpc.bank][lpc.word];
      if (t.n) oled.drawHLine(30, y + 19, (int)(fi * 78 / t.n) + 1);
    } else for (int i = 0; i < 10; i++) oled.drawHLine(30 + i * 8, mid, 6);
    oled.drawStr(2, mid + 3, lpFreeze ? "FRZ" : lpMono ? "MONO" : (lpWhisper ? "WHSP" : "LPC"));
    snprintf(lab, sizeof lab, "%s", BANK_NAMES[talkBank]);
  } else {
    for (int i = 0; i < 12; i++) {
      int e = constrain((int)(perc[i].env * 16), 0, 16);
      oled.drawFrame(4 + i * 10, y + 2, 8, 18);
      if (e) oled.drawBox(5 + i * 10, y + 19 - e, 6, e);
    }
    snprintf(lab, sizeof lab, "%s", machMute ? "MUTES ON" : "");
  }
  if (lab[0]) tab(128 - oled.getStrWidth(lab) - 4, y, lab);
}

void drawReadout() {
  char wq[28], ch[12];
  if (mode == M_HUM) { chordName(ch, sizeof ch); snprintf(wq, sizeof wq, "[%s %s]", ch, driftOn ? "DRIFT" : "HOLD"); }
  else snprintf(wq, sizeof wq, "[%s]", padName(mode, curPad));
  oled.setFont(u8g2_font_5x8_tf);
  oled.drawStr(64 - oled.getStrWidth(wq) / 2, 81, wq);
  oled.setFont(u8g2_font_4x6_tf);
  char info[44];
  snprintf(info, sizeof info, "FX:%s %s %d%%", FX_NAMES[fx], FX_KNOB[fx], (int)(fxAmt[fx] * 100));
  oled.drawStr(64 - oled.getStrWidth(info) / 2, 88, info);
}

void drawSeqPane() {                                 // 16 steps: VOX / MACH lanes + knob meters
  const int y = 90, h = 29;
  brackets(0, y, 128, h);
  for (int s = 0; s < 16; s++) {
    int x = 4 + s * 7 + (s / 4);
    const Step& st = seq[s];
    int lv = y + 3, ld = y + 11;
    if (st.word >= 0) { if (st.src == SRC_TALK) oled.drawFrame(x, lv, 6, 6); else oled.drawBox(x, lv, 6, 6); }
    else oled.drawPixel(x + 2, lv + 3);
    uint16_t d = st.mach & ~machMute;
    if (d & (1 << D_THUD)) oled.drawBox(x, ld, 6, 6);
    else if (d) oled.drawFrame(x, ld, 6, 6);
    else oled.drawPixel(x + 2, ld + 3);
    if (s == curStep && playing) { oled.setDrawColor(2); oled.drawBox(x - 1, y + 2, 8, 16); oled.setDrawColor(1); }
  }
  oled.setFont(u8g2_font_4x6_tf);                    // knob meters: FX amount and SPACE
  oled.drawStr(3, y + 25, "FX"); oled.drawFrame(13, y + 20, 48, 5); oled.drawBox(14, y + 21, (int)(46 * fxAmt[fx]), 3);
  oled.drawStr(66, y + 25, "SP"); oled.drawFrame(76, y + 20, 48, 5); oled.drawBox(77, y + 21, (int)(46 * space), 3);
}

void drawChips() {
  struct { const char* t; bool on; } chips[6] = {
    {"REC", rec}, {"PLAY", playing}, {"XTLK", xtalk}, {"RND", machDens > 0}, {"FRZ", freezeHeld || lpFreeze}, {"INF", revFreeze}};
  oled.setFont(u8g2_font_4x6_tf);
  for (int i = 0; i < 6; i++) {
    int x = i * 21 + 1, w = 20;
    if (chips[i].on && !(i == 0 && ((millis() / 300) & 1))) {
      oled.drawBox(x, 120, w, 8);
      oled.setDrawColor(0); oled.drawStr(x + (w - oled.getStrWidth(chips[i].t)) / 2, 126, chips[i].t); oled.setDrawColor(1);
    } else {
      dottedH(x, 120, w); dottedH(x, 127, w);
      oled.drawStr(x + (w - oled.getStrWidth(chips[i].t)) / 2, 126, chips[i].t);
    }
  }
}

void menuLabel(int m, int p, char* nm, bool& on) {
  static const char* MODE_L[12] = {"HUM", "SAM", "TALK", "MACH", "VOL-", "VOL+", "BANK", "REPLAY", "ROBOT", "SAM", "GHOST", "DEEP"};
  static const char* FX_L[12] = {"CLEAN", "ECHO", "RADIO", "GRAIN", "DEEP", "VOX", "CHOIR", "STUTTR", "REVRS", "TAPE", "UNDER", "INFINIT"};
  static const char* SEQ_L[12] = {"REC", "PLAY", "XTALK", "AUTO", "BPM-", "BPM+", "MUTATE", "RANDOM", "CLR VOX", "CLR MCH", "CLR ALL", "SILENCE"};
  static const char* HUM_L[12] = {"OCT-", "OCT+", "DARK", "BRIGHT", "SLOWER", "FASTER", "DETUNE", "ALL OFF", "ALL ON", "RESET", "", ""};
  static const char* TALK_L[12] = {"WHISPER", "MONO", "FREEZE", "STRETCH", "PITCH-", "PITCH+", "PITCH 0", "GLITCH", "BACKWRD", "FORMNT-", "FORMNT+", "RESET"};
  const char* l = "";
  on = false;
  switch (m) {
    case K_MODE: l = MODE_L[p]; on = (p < 4 && p == mode) || (p >= 8 && p - 8 == voiceIdx); break;
    case K_FX: l = FX_L[p]; on = (p < 7 && p == fx) || (p == 7 && stutterOn) || (p == 8 && reverseOn) || (p == 9 && tapeOn) || (p == 10 && underOn) || (p == 11 && revFreeze); break;
    case K_SEQ: l = SEQ_L[p]; on = (p == 0 && rec) || (p == 1 && playing) || (p == 2 && xtalk) || (p == 7 && machDens); break;
    default:
      if (mode == M_HUM) l = HUM_L[p];
      else if (mode == M_TALK) { l = TALK_L[p]; on = (p == 0 && lpWhisper) || (p == 1 && lpMono) || (p == 2 && lpFreeze) || (p == 3 && lpStretch) || (p == 7 && lpGlitch) || (p == 8 && lpRev); }
      else if (mode == M_MACH) { l = PERC[p].name; on = machMute & (1 << p); }
      else l = bank[p].text;
      break;
  }
  strncpy(nm, l, 8); nm[8] = 0;
}

void drawMenu(int m) {
  char title[32];
  snprintf(title, sizeof title, "%s", MOD_NAMES[m]);
  if (m == K_SHIFT) snprintf(title, sizeof title, "SHIFT: %s  knob=SCRATCH", mode == M_HUM ? "DRONE" : mode == M_SAM ? "BACKWARDS" : mode == M_TALK ? "BENDS" : "MUTE");
  if (m == K_MODE) snprintf(title, sizeof title, "MODE  vol %d  %s", volIdx + 1, BANK_NAMES[talkBank]);
  if (m == K_SEQ) snprintf(title, sizeof title, "SEQ  %d BPM", bpm);
  if (m == K_FX) snprintf(title, sizeof title, "FX  knob=SPACE %d%%", (int)(space * 100));
  tab(0, 0, title);
  oled.setFont(u8g2_font_5x8_tf);
  const int cw = 42, ch = 28, y0 = 10;
  for (int p = 0; p < 12; p++) {
    int r = p / 3, c = p % 3, x = c * cw + 1, y = y0 + r * ch;
    char nm[10]; bool on;
    menuLabel(m, p, nm, on);
    if (on) { oled.drawBox(x, y, cw - 2, ch - 2); oled.setDrawColor(0); } else oled.drawFrame(x, y, cw - 2, ch - 2);
    oled.drawStr(x + (cw - 2 - oled.getStrWidth(nm)) / 2, y + 16, nm);
    oled.setDrawColor(1);
  }
}

void drawAlert(const char* m) {
  oled.setFont(u8g2_font_6x10_tf);
  char t[32]; snprintf(t, sizeof t, ">> %s", m);
  int w = oled.getStrWidth(t) + 10, x = max(2, 64 - w / 2);
  oled.setDrawColor(0); oled.drawBox(x - 2, 40, w + 4, 20); oled.setDrawColor(1);
  oled.drawFrame(x - 2, 40, w + 4, 20); oled.drawFrame(x, 42, w, 16);
  oled.drawStr(x + 5, 53, t);
}
// ---- boot splash: generative geometry, different every boot. Two random scenes (times-table mandala, polygon
// rings, flow-field particles, a rotating tunnel, a 3D Lissajous knot; particles about half the time), glitch cuts,
// then the particles converge into the BLASTEROID: a cyborg cyclops in wraparound shades. His eye charges and two
// lasers burn BLASTEROID / MEGASYNTH in from opposite ends while SAM says the name; Talkie answers "OPERATIONAL".
// Painted in grey with persistence (trails), then Bayer-dithered with film grain. splash_preview.py mirrors this.
volatile bool splashOn = true;
uint32_t splashT0 = 0, splashEnd = 0, splashLast = 0;
bool splashVoice = false, splashTalk = false, splashStopped = false, showCredit = false, splashKeysClear = false;
volatile bool bootFreezeStopReq = false;
float camS = 1, camX = 64, camY = 64;
int shakeX = 0, shakeY = 0;

static uint8_t gbuf[128 * 128];
float pB = 1, pGX = 0, pGY = 0, pOX = 64, pOY = 64;      // paint = base + linear gradient (per world px) around (pOX, pOY)
bool pChrome = false; float pTop = 0, pH = 1;           // chrome paint: banded by height (logo letters)
float fxLift = 0, fxFade = 1;                           // lift: lightning flash on everything lit; fade: logo fade-in
const float LGX = 0.010f, LGY = -0.008f;                // key light from the upper right
void paint(float b, float gx = 0, float gy = 0, float ox = 64, float oy = 64) { pB = b; pGX = gx; pGY = gy; pOX = ox; pOY = oy; pChrome = false; }
void lit(float b, float k = 1) { paint(b, LGX * k, LGY * k, 60, 60); }
void chrome(float top, float h) { pChrome = true; pTop = top; pH = h; }
static inline float chromeFn(float v) { return v < 0.5f ? 1.1f - v * 0.9f : 0.5f + (v - 0.5f) * 1.2f; }

static inline float camEX(float x) { return 64 + (x - camX) * camS + shakeX; }   // world edge -> screen
static inline float camEY(float y) { return 64 + (y - camY) * camS + shakeY; }
static inline float camTX(float x) { return camEX(x + 0.5f); }                    // world pixel centre -> screen
static inline float camTY(float y) { return camEY(y + 0.5f); }
static inline float camWX(float sx) { return camX + (sx + 0.5f - 64 - shakeX) / camS - 0.5f; }   // screen -> world
static inline float camWY(float sy) { return camY + (sy + 0.5f - 64 - shakeY) / camS - 0.5f; }
static inline uint8_t g8(float g) { return (uint8_t)constrain((int)(g * 255), 0, 255); }

void hspan(float xa, float xb, int y) {              // paint pixels whose centres lie in [xa, xb]
  if (y < 0 || y > 127) return;
  if (xa > xb) { float t = xa; xa = xb; xb = t; }
  int x0 = (int)ceilf(xa - 0.5f), x1 = (int)floorf(xb - 0.5f);
  if (x1 < x0) x1 = x0 = (int)floorf((xa + xb) * 0.5f);   // thinner than a pixel: still paint one
  if (x1 < 0 || x0 > 127) return;
  x0 = max(x0, 0); x1 = min(x1, 127);
  uint8_t* row = gbuf + y * 128;
  if (pChrome) { uint8_t v = g8(chromeFn((camWY(y) - pTop) / pH) * fxFade); for (int x = x0; x <= x1; x++) row[x] = v; return; }
  float g = pB + pGX * (camWX(x0) - pOX) + pGY * (camWY(y) - pOY), dg = pGX / camS;
  if (pB > 0) g += fxLift;
  for (int x = x0; x <= x1; x++, g += dg) row[x] = g8(g);
}
void rTri(float ax, float ay, float bx, float by, float cx, float cy) {
  float t;
  if (ay > by) { t = ax; ax = bx; bx = t; t = ay; ay = by; by = t; }
  if (by > cy) { t = bx; bx = cx; cx = t; t = by; by = cy; cy = t; }
  if (ay > by) { t = ax; ax = bx; bx = t; t = ay; ay = by; by = t; }
  int y0 = max(0, (int)ceilf(ay - 0.5f)), y1 = min(127, (int)floorf(cy - 0.5f));
  for (int y = y0; y <= y1; y++) {
    float yc = y + 0.5f;
    float xa = ax + (cx - ax) * (yc - ay) / max(cy - ay, 1e-4f);
    float xb = yc < by ? ax + (bx - ax) * (yc - ay) / max(by - ay, 1e-4f) : bx + (cx - bx) * (yc - by) / max(cy - by, 1e-4f);
    hspan(xa, xb, y);
  }
}
void rDisc(float cx, float cy, float r) {
  float R = r + 0.5f;
  int y0 = max(0, (int)floorf(cy - R)), y1 = min(127, (int)ceilf(cy + R));
  for (int y = y0; y <= y1; y++) {
    float dy = y + 0.5f - cy;
    if (fabsf(dy) > R) continue;
    float dx = sqrtf(R * R - dy * dy);
    hspan(cx - dx, cx + dx, y);
  }
}
void rPoly(const float* P, int n) {                  // scanline fill, even-odd; P = x0,y0,x1,y1... screen coords
  float ymin = 1e9f, ymax = -1e9f;
  for (int i = 0; i < n; i++) { ymin = min(ymin, P[2 * i + 1]); ymax = max(ymax, P[2 * i + 1]); }
  int y0 = max(0, (int)ceilf(ymin - 0.5f)), y1 = min(127, (int)floorf(ymax - 0.5f));
  float xs[24];
  for (int y = y0; y <= y1; y++) {
    float yc = y + 0.5f; int m = 0;
    for (int i = 0; i < n && m < 24; i++) {
      float ax = P[2 * i], ay = P[2 * i + 1], bx = P[2 * ((i + 1) % n)], by = P[2 * ((i + 1) % n) + 1];
      if ((ay <= yc && yc < by) || (by <= yc && yc < ay)) xs[m++] = ax + (bx - ax) * (yc - ay) / (by - ay);
    }
    for (int i = 1; i < m; i++) { float v = xs[i]; int j = i - 1; while (j >= 0 && xs[j] > v) { xs[j + 1] = xs[j]; j--; } xs[j + 1] = v; }
    for (int k = 0; k + 1 < m; k += 2) hspan(xs[k], xs[k + 1], y);
  }
}
// world-space painting (t = line width in world pixels)
void cLine(float x0, float y0, float x1, float y1, float t = 1) {
  float a = camTX(x0), b = camTY(y0), c = camTX(x1), d = camTY(y1), w = max(t * camS, 1.0f);
  float len = sqrtf((c - a) * (c - a) + (d - b) * (d - b)) + 1e-4f, nx = -(d - b) / len * w * 0.5f, ny = (c - a) / len * w * 0.5f;
  rTri(a + nx, b + ny, a - nx, b - ny, c + nx, d + ny);
  rTri(c + nx, d + ny, c - nx, d - ny, a - nx, b - ny);
  if (w >= 2) { rDisc(a, b, w * 0.5f - 0.5f); rDisc(c, d, w * 0.5f - 0.5f); }
}
void cBox(float x, float y, float w, float h) {
  float x0 = camEX(x), x1 = camEX(x + w), y0 = camEY(y), y1 = camEY(y + h);
  int ya = max(0, (int)ceilf(y0 - 0.5f)), yb = min(127, (int)floorf(y1 - 0.5f));
  if (yb < ya) ya = yb = (int)floorf((y0 + y1) * 0.5f);
  for (int yy = ya; yy <= yb; yy++) hspan(x0, x1 - 0.001f, yy);
}
void cPix(float x, float y) { cBox(x, y, 1, 1); }
void cDisc(float x, float y, float r) { rDisc(camTX(x), camTY(y), r * camS); }
void cTri(float ax, float ay, float bx, float by, float cx, float cy) { rTri(camTX(ax), camTY(ay), camTX(bx), camTY(by), camTX(cx), camTY(cy)); }
void cPoly(const float* P, int n, float dx = 0, float dy = 0) {
  float S[48];
  n = min(n, 24);
  for (int i = 0; i < n; i++) { S[2 * i] = camTX(P[2 * i] + dx); S[2 * i + 1] = camTY(P[2 * i + 1] + dy); }
  rPoly(S, n);
}
void rimPoly(const float* P, int n, float dark, float rim) {   // bright copy, dark copy nudged away from the light = rim light
  paint(rim); cPoly(P, n);
  paint(dark); cPoly(P, n, -0.8f, 0.6f);
}


float smokeNz[32][32];
static inline float vnoise(float x, float y) {
  int xi = (int)floorf(x), yi = (int)floorf(y);
  float fx = x - xi, fy = y - yi;
  fx = fx * fx * (3 - 2 * fx); fy = fy * fy * (3 - 2 * fy);
  float a = smokeNz[yi & 31][xi & 31], b = smokeNz[yi & 31][(xi + 1) & 31];
  float c = smokeNz[(yi + 1) & 31][xi & 31], d = smokeNz[(yi + 1) & 31][(xi + 1) & 31];
  return a + (b - a) * fx + (c - a) * fy + (a - b - c + d) * fx * fy;
}
static inline float ridgeFar(float x) { return 97 - 9 * fabsf(sinf(x * 0.045f)) - 6 * sinf(x * 0.13f + 1) - 3 * fabsf(sinf(x * 0.37f)); }
static inline float ridgeMid(float x) { return 110 - 6 * fabsf(sinf(x * 0.06f + 2)) - 4 * sinf(x * 0.19f) - 2 * fabsf(sinf(x * 0.5f)); }
// hard contrast + Bayer 4x4 + grain; true black stays black
void ditherOut(uint32_t t) {
  static const uint8_t B4[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};
  uint32_t r = t * 2654435761u | 1;
  for (int y = 0; y < 128; y++) {
    const uint8_t* row = gbuf + y * 128;
    for (int x = 0; x < 128; x++) {
      int g = ((int)row[x] - 115) * 3 / 2 + 115;                 // (g - 0.45) * 1.5 + 0.45
      r ^= r << 13; r ^= r >> 17; r ^= r << 5;
      int thr = B4[y & 3][x & 3] * 16 + 8 + (int)(r % 37) - 18;   // +-0.07 grain
      if (g > max(thr, 8)) oled.drawPixel(x, y);
    }
  }
}
enum { SC_TIMES, SC_RINGS, SC_PART, SC_TUNNEL, SC_LISSA };
struct Film {
  int scenes[2]; float k0, kv; int sides[5]; float rsp[5]; int la, lb, lc;
  float P[150][2]; float t; float face[150][2];
} film;
// the cyclops (head coordinates; drawn 1.2x around (64, 27) -> screen y 30)
const float HEAD[24] = {44, 11, 52, 5, 76, 5, 84, 11, 87, 22, 86, 36, 80, 46, 72, 51, 56, 51, 48, 46, 42, 36, 41, 22};
const float SHADES[20] = {35, 22, 46, 18.5f, 64, 17.5f, 82, 18.5f, 93, 22, 91, 30, 80, 33.5f, 64, 32, 48, 33.5f, 37, 30};
const float EYE_X = 64, EYE_Y = 25.5f, HS = 1.2f, HCY = 27, HSY = 30;
const int L1_BASE = 82, L2_BASE = 102;              // text baselines; the lasers hit the middle of the letters
static inline float headSX(float x) { return 64 + (x - 64) * HS; }
static inline float headSY(float y) { return HSY + (y - HCY) * HS; }

void filmInit() {
  static bool nzInit = false;
  if (!nzInit) { for (auto& r : smokeNz) for (auto& v : r) v = (esp_random() & 0xFFFF) / 65535.0f; nzInit = true; }
  Rng r(esp_random());
  static const int GEO[4] = {SC_TIMES, SC_RINGS, SC_TUNNEL, SC_LISSA};
  int a = GEO[r.range(0, 4)], b;
  if (r.uni(0, 1) < 0.5f) b = SC_PART;
  else { do b = GEO[r.range(0, 4)]; while (b == a); }
  film.scenes[0] = a; film.scenes[1] = b;
  film.k0 = r.uni(2, 4); film.kv = r.uni(0.25f, 0.6f);
  for (int j = 0; j < 5; j++) { film.sides[j] = r.range(3, 9); film.rsp[j] = r.uni(-1.2f, 1.2f); }
  static const int LA[3] = {1, 2, 3}, LB[4] = {2, 3, 4, 5}, LC[3] = {1, 3, 5};
  film.la = LA[r.range(0, 3)]; film.lb = LB[r.range(0, 4)]; film.lc = LC[r.range(0, 3)];
  for (auto& p : film.P) { p[0] = r.uni(0, 128); p[1] = r.uni(0, 128); }
  film.t = 0;
  // target points along the head outline (60 %) and the shades (40 %)
  int k = 0;
  for (int pass = 0; pass < 2; pass++) {
    const float* P = pass ? SHADES : HEAD; int n = pass ? 10 : 12, m = pass ? 150 - 90 : 90;
    float L[12], tot = 0;
    for (int e = 0; e < n; e++) { int q = (e + 1) % n; L[e] = sqrtf(sq(P[2 * q] - P[2 * e]) + sq(P[2 * q + 1] - P[2 * e + 1])); tot += L[e]; }
    for (int c = 0; c < m && k < 150; c++) {
      float d = tot * c / m; int e = 0;
      while (e < n - 1 && d > L[e]) { d -= L[e]; e++; }
      int q = (e + 1) % n; float u = d / L[e];
      film.face[k][0] = headSX(P[2 * e] + (P[2 * q] - P[2 * e]) * u); film.face[k][1] = headSY(P[2 * e + 1] + (P[2 * q + 1] - P[2 * e + 1]) * u); k++;
    }
  }
  memset(gbuf, 0, sizeof gbuf);
}

void sceneTimes(float t, float fade) {
  const int N = 72; float k = film.k0 + film.kv * t, rot = t * 0.2f;
  paint(0.62f * fade);
  for (int i = 0; i < N; i++) {
    float a1 = rot + i * 2 * PI / N, a2 = rot + i * k * 2 * PI / N;
    cLine(64 + 54 * cosf(a1), 64 + 54 * sinf(a1), 64 + 54 * cosf(a2), 64 + 54 * sinf(a2), 0.5f);
  }
  paint(fade);
  for (int i = 0; i < N; i += 6) { float a1 = rot + i * 2 * PI / N; cDisc(64 + 54 * cosf(a1), 64 + 54 * sinf(a1), 0.8f); }
}
void sceneRings(float t, float fade) {
  for (int j = 0; j < 5; j++) {
    float rad = 12 + j * 11, rot = t * film.rsp[j] + j; int n = film.sides[j];
    float pts[18];
    for (int i = 0; i < n; i++) { pts[2 * i] = 64 + rad * cosf(rot + 2 * PI * i / n); pts[2 * i + 1] = 64 + rad * sinf(rot + 2 * PI * i / n); }
    if (j == 0) { paint(0.18f * fade); cPoly(pts, n); }
    paint(((j & 1) ? 0.5f : 1.0f) * fade);
    for (int i = 0; i < n; i++) { int q = (i + 1) % n; cLine(pts[2 * i], pts[2 * i + 1], pts[2 * q], pts[2 * q + 1], (j & 1) ? 0.6f : 1.0f); }
    float ang = -t * film.rsp[j] * 2 + j * 1.7f;
    paint(fade); cDisc(64 + rad * cosf(ang), 64 + rad * sinf(ang), 1.3f);
  }
}
void sceneParticles(float t, float fade, float dt) {
  static Rng r(77);
  paint(0.95f * fade);
  for (auto& p : film.P) {
    float a = vnoise(p[0] * 0.025f, p[1] * 0.025f + t * 0.15f) * 4 * PI;
    p[0] += cosf(a) * 40 * dt; p[1] += sinf(a) * 40 * dt;
    if (p[0] < 0 || p[0] >= 128 || p[1] < 0 || p[1] >= 128) { p[0] = r.uni(0, 128); p[1] = r.uni(0, 128); }
    cDisc(p[0], p[1], 0.6f);
  }
}
void sceneTunnel(float t, float fade) {
  for (int i = 0; i < 12; i++) {
    float z = fmodf(i - t * 2.2f + 120, 12) + 0.6f, size = 70 / z, rot = z * 0.35f + t * 0.6f;
    float pts[8];
    for (int q = 0; q < 4; q++) { pts[2 * q] = 64 + size * cosf(rot + PI / 4 + q * PI / 2); pts[2 * q + 1] = 64 + size * sinf(rot + PI / 4 + q * PI / 2); }
    paint(min(1.0f, 1.6f / z) * fade);
    for (int q = 0; q < 4; q++) { int e = (q + 1) & 3; cLine(pts[2 * q], pts[2 * q + 1], pts[2 * e], pts[2 * e + 1], max(0.5f, 2.5f / z)); }
  }
}
void sceneLissa(float t, float fade) {
  float px = 0, py = 0, ry = t * 0.7f, rx = 0.4f + 0.3f * sinf(t * 0.5f);
  for (int i = 0; i <= 180; i++) {
    float q = i / 180.0f * 2 * PI;
    float x = sinf(film.la * q + t * 0.3f), y = sinf(film.lb * q), z = sinf(film.lc * q + 1.0f);
    float x2 = x * cosf(ry) + z * sinf(ry), z2 = -x * sinf(ry) + z * cosf(ry);
    float y2 = y * cosf(rx) - z2 * sinf(rx), z3 = y * sinf(rx) + z2 * cosf(rx);
    float sx = 64 + x2 * 46, sy = 64 + y2 * 46, d = (z3 + 1) / 2;
    if (i) { paint((0.45f + 0.5f * d) * fade); cLine(px, py, sx, sy, 0.6f + 0.8f * d); }
    px = sx; py = sy;
  }
}

void drawHead(float fade, float eye, float t) {
  float cs = camS, cx = camX, cy = camY;
  camS = HS; camX = 64; camY = HCY + (64 - HSY) / HS;
  float tmp[24];
  // shoulders / collar: dark, rim-lit
  static const float SH0[8] = {46, 49, 82, 49, 100, 58, 28, 58}, SH1[8] = {48, 50, 80, 50, 97, 57.5f, 31, 57.5f};
  paint(0); cPoly(SH0, 4);
  paint(0.3f * fade); cPoly(SH1, 4);
  paint(0.9f * fade); cLine(80, 50, 97, 57.5f, 0.7f); cLine(31, 57.5f, 48, 50, 0.5f);
  // head: bold and bright, a shadow side
  for (int i = 0; i < 12; i++) { tmp[2 * i] = HEAD[2 * i] + (HEAD[2 * i] - 64) * 0.07f; tmp[2 * i + 1] = HEAD[2 * i + 1] + (HEAD[2 * i + 1] - 28) * 0.07f; }
  paint(0); cPoly(tmp, 12);
  paint(0.95f * fade); cPoly(HEAD, 12);
  static const float SHADOW[12] = {42, 36, 41, 22, 46.5f, 22, 48, 37, 52, 48, 48, 46};
  paint(0.55f * fade); cPoly(SHADOW, 6);
  paint(0); cLine(52, 10.5f, 76, 10.5f, 0.5f);                                // forehead seam
  cLine(64.5f, 33.5f, 62, 38.8f, 0.7f); cLine(62, 38.8f, 65.5f, 39.2f, 0.7f); // nose
  cLine(56, 43, 72, 43, 1.6f); cLine(56, 43, 54, 44.8f, 1.2f); cLine(72, 43, 74, 44.8f, 1.2f);   // grim mouth
  cLine(61, 48.6f, 67, 48.6f, 0.7f);                                          // chin
  // wraparound shades: solid black, hard white rim, one reflection line, a glint
  for (int i = 0; i < 10; i++) { tmp[2 * i] = SHADES[2 * i] + (SHADES[2 * i] - 64) * 0.05f; tmp[2 * i + 1] = SHADES[2 * i + 1] + (SHADES[2 * i + 1] > 25 ? 1.4f : -1.4f); }
  paint(0); cPoly(tmp, 10);
  paint(0.85f * fade);
  for (int i = 0; i < 10; i++) { int q = (i + 1) % 10; cLine(tmp[2 * i], tmp[2 * i + 1], tmp[2 * q], tmp[2 * q + 1], 0.55f); }
  paint(fade); cLine(38, 28.5f, 90, 28.5f, 0.6f);
  float g = fmodf(t * 0.6f, 1.6f);
  if (g < 1) { paint(1); cLine(40 + g * 46, 19.5f, 37 + g * 46, 30, 0.9f); }
  if (eye > 0) {                                     // the single eye blazing through the lens
    paint(0.3f + 0.35f * eye); cDisc(EYE_X, EYE_Y, 2 + 3.5f * eye);
    paint(1); cDisc(EYE_X, EYE_Y, 0.8f + 1.4f * eye);
  }
  camS = cs; camX = cx; camY = cy;
}

void glitch(Rng& r) {                                // horizontal tearing bands
  uint8_t tmp[128];
  for (int k = 0; k < 4; k++) {
    int y0 = r.range(0, 120), h = r.range(2, 8), sh = r.range(-14, 15);
    for (int y = y0; y < min(128, y0 + h); y++) {
      uint8_t* row = gbuf + y * 128;
      for (int x = 0; x < 128; x++) tmp[x] = row[(x - sh + 128) & 127];
      memcpy(row, tmp, 128);
    }
    if (r.uni(0, 1) < 0.4f) for (int x = 0; x < 128; x++) gbuf[y0 * 128 + x] = max(gbuf[y0 * 128 + x], (uint8_t)153);
  }
}

extern volatile bool bootVoiceReq, bootTalkReq;
float laserX1 = 999, laserX2 = -999;               // how far the lasers have burned each line
void drawSplash() {
  uint32_t now = millis();
  if (!splashT0) { splashT0 = now; splashLast = now; splashEnd = now + 13500; filmInit(); }
  float dt = min(0.1f, (now - splashLast) * 0.001f);
  splashLast = now;
  film.t += dt;
  float u = film.t;
  static Rng gr(5);
  camS = 1; camX = 64; camY = 64; shakeX = shakeY = 0; fxLift = 0; fxFade = 1;
  int sc = u < 3 ? film.scenes[0] : film.scenes[1];
  float decay = u < 6 ? (sc == SC_PART ? 0.86f : 0.55f) : u < 7.2f ? 0.8f : 0.55f;
  int dk = (int)(decay * 256);
  for (auto& v : gbuf) v = (uint8_t)((v * dk) >> 8);

  if (u < 6) {
    float lt = u < 3 ? u : u - 3, fade = min(1.0f, lt / 0.5f);
    switch (sc) {
      case SC_TIMES: sceneTimes(lt, fade); break;
      case SC_RINGS: sceneRings(lt, fade); break;
      case SC_PART: sceneParticles(lt, fade, dt); break;
      case SC_TUNNEL: sceneTunnel(lt, fade); break;
      default: sceneLissa(lt, fade); break;
    }
    if (u > 2.85f && u < 3.15f) glitch(gr);
  } else if (u < 7.2f) {                             // everything flies into the cyclops
    float k = (u - 6) / 1.2f, e = 1 - powf(1 - min(1.0f, k), 3);
    if (k > 0.6f) drawHead((k - 0.6f) / 0.4f * 0.7f, 0, u);
    paint(1);
    for (int i = 0; i < 150; i++) cDisc(film.P[i][0] + (film.face[i][0] - film.P[i][0]) * e, film.P[i][1] + (film.face[i][1] - film.P[i][1]) * e, 0.7f);
  } else {                                           // eye charges, lasers burn the name in
    float k = u - 7.2f;
    float eye = k < 2.4f ? min(1.0f, k / 0.6f) : 0.55f + 0.25f * sinf(k * 5);
    if (k > 0.5f && k < 0.7f) shakeX = gr.range(-2, 3);
    drawHead(min(1.0f, 0.7f + k), eye, u);
    laserX1 = 999; laserX2 = -999;
    if (k > 0.6f && k < 2.4f) {
      float q = (k - 0.6f) / 1.8f, ex = headSX(EYE_X), ey = headSY(EYE_Y);
      laserX1 = 8 + q * 112; laserX2 = 120 - q * 112;
      float hit[2][2] = {{laserX1, L1_BASE - 6.0f}, {laserX2, L2_BASE - 6.0f}};
      for (auto& h : hit) {
        paint(0.55f); cLine(ex, ey, h[0], h[1], 3.2f);
        paint(1); cLine(ex, ey, h[0], h[1], 1.3f);
        paint(0.6f); cDisc(h[0], h[1], 4); paint(1); cDisc(h[0], h[1], 1.6f);
        paint(0.9f);
        for (int j = 0; j < 6; j++) { float a = gr.uni(0, 2 * PI), rr = gr.uni(2, 9); cDisc(h[0] + cosf(a) * rr, h[1] + sinf(a) * rr * 0.6f, 0.5f); }
      }
    }
    for (int line = 0; line < 2; line++) {           // scorch glow behind the burned letters
      int c = (line ? L2_BASE : L1_BASE) - 6;
      for (int y = max(0, c - 12); y < min(128, c + 13); y++) {
        uint8_t v = g8(0.16f * expf(-sq((y - c) / 7.0f)));
        for (int x = 0; x < 128; x++) {
          bool lit = line ? x > laserX2 : x < laserX1;
          if (lit && v > gbuf[y * 128 + x]) gbuf[y * 128 + x] = v;
        }
      }
    }
  }
  if (u >= 7.9f && !splashVoice) { splashVoice = true; bootVoiceReq = true; }   // SAM: "BLASTEROID MEGASYNTH"
  if (u >= 10.4f && !splashTalk) { splashTalk = true; bootTalkReq = true; }     // Talkie: "OPERATIONAL"
  splashStopped = true;
  ditherOut(now);

  if (u > 7.8f) {                                    // the name, burned in up to where the lasers are
    oled.setFont(u8g2_font_helvB12_tr);
    const char* L[2] = {"BLASTEROID", "MEGASYNTH"};
    const int B[2] = {L1_BASE, L2_BASE};
    for (int line = 0; line < 2; line++) {
      int w = oled.getStrWidth(L[line]), x0 = 64 - w / 2;
      oled.setDrawColor(0);
      for (int dx = -1; dx <= 1; dx++) for (int dy = -1; dy <= 1; dy++) if (dx || dy) oled.drawStr(x0 + dx, B[line] + dy, L[line]);
      oled.setDrawColor(1);
      oled.drawStr(x0, B[line], L[line]);
      oled.setDrawColor(0);                          // not burned yet: hide it
      if (line == 0 && laserX1 < 128) oled.drawBox(max(0, (int)laserX1), B[0] - 14, 128 - max(0, (int)laserX1), 17);
      if (line == 1 && laserX2 > 0) oled.drawBox(0, B[1] - 14, min(128, (int)laserX2 + 1), 17);
      oled.setDrawColor(1);
    }
    oled.setFont(u8g2_font_4x6_tf);
    if (showCredit && u > 9.8f) {
      const char* cr = "MARK HELLAR + CLAUDE";
      int cw = oled.getStrWidth(cr);
      oled.setDrawColor(0); oled.drawBox(64 - cw / 2 - 2, 106, cw + 4, 8); oled.setDrawColor(1);
      oled.drawStr(64 - cw / 2, 112, cr);
    }
    if (u > 10.8f && ((now / 500) & 1)) {
      const char* pr = "PRESS ANY KEY";
      int pw = oled.getStrWidth(pr);
      oled.setDrawColor(0); oled.drawBox(64 - pw / 2 - 2, 119, pw + 4, 8); oled.setDrawColor(1);
      oled.drawStr(64 - pw / 2, 125, pr);
    }
  }
  if (bankBusy) oled.drawHLine(34, 127, 60 * renderProgress / 12);   // voices still loading
}

// SAM renders its words on a background task during the film (the splash keeps animating)
void bankTask(void*) { renderBank(voiceIdx, false); vTaskDelete(nullptr); }

void drawFrame() {
  oled.clearBuffer();
  if (eggReq || replayReq) {                         // replay the film (MODE+SHIFT = with credits)
    showCredit = eggReq; eggReq = false; replayReq = false;
    splashOn = true; splashT0 = splashEnd = 0; splashKeysClear = false; splashVoice = splashTalk = false;
  }
  if (splashOn) {
    if (!keysVis) splashKeysClear = true;
    if (splashEnd && (millis() > splashEnd || (keysVis && splashKeysClear))) {
      splashOn = false;
    }
    else { drawSplash(); oled.sendBuffer(); return; }
  }
  if (bankBusy) {
    tab(0, 0, "BLASTEROID MEGASYNTH");
    oled.setFont(u8g2_font_4x6_tf);
    oled.drawStr(0, 30, "> TUNING THE MOUTHS");
    char v[24]; snprintf(v, sizeof v, "> VOICE %s", VOICES[voiceIdx].name);
    oled.drawStr(0, 40, v);
    char wtxt[32]; snprintf(wtxt, sizeof wtxt, "> %s", bank[min(renderProgress, 11)].text);
    oled.drawStr(0, 50, wtxt);
    brackets(10, 60, 108, 14);
    int segs = 20 * renderProgress / 12;
    for (int i = 0; i < 20; i++) { if (i < segs) oled.drawBox(13 + i * 5, 63, 4, 8); else oled.drawPixel(15 + i * 5, 67); }
    oled.sendBuffer();
    return;
  }
  int hm = heldModVis;
  if (hm >= 0) { drawMenu(hm); if (millis() - msgAt < 700) drawAlert(msg); oled.sendBuffer(); return; }
  drawTopBar();
  drawSigPane();
  drawPadPane();
  drawVizStrip();
  drawReadout();
  drawSeqPane();
  drawChips();
  if (millis() - msgAt < 900) drawAlert(msg);
  oled.sendBuffer();
}

// =====================================================================
void setup() {
  Serial.setTxTimeoutMs(0);
  Serial.begin(115200);
  delay(300);
  for (int c = 0; c < 4; c++) pinMode(COL_PINS[c], INPUT_PULLUP);
  for (int r = 0; r < 4; r++) pinMode(ROW_PINS[r], INPUT);
  pinMode(PIN_POT_HI, OUTPUT); digitalWrite(PIN_POT_HI, HIGH);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_POT, ADC_11db);
  for (int i = 0; i <= 1024; i++) sineTab[i] = sinf(2.0f * PI * i / 1024.0f);
  for (int p = 0; p < 12; p++) { padMenu[p] = -1; padMode[p] = M_HUM; }
  rgbLedWrite(PIN_RGB, 0, 0, 0);

  oled.setBusClock(400000);
  oled.begin();
  oled.setContrast(140);

  if (psramFound()) { poolSize = 768 * 1024; pool = (int8_t*)ps_malloc(poolSize); dlyBuf = (float*)ps_calloc(DLY_N, sizeof(float)); }
  if (!pool) { poolSize = 120 * 1024; pool = (int8_t*)malloc(poolSize); }
  if (!dlyBuf) dlyBuf = (float*)calloc(DLY_N, sizeof(float));
  for (int k = 0; k < 4; k++) {                      // reverb lines: internal RAM first (faster), PSRAM if not
    rv[k] = (float*)heap_caps_calloc(RL[k], sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!rv[k]) rv[k] = (float*)ps_calloc(RL[k], sizeof(float));
  }
  logf("ohm: PSRAM %s, pool %u KB, free heap %u\n", psramFound() ? "yes" : "no", poolSize / 1024, ESP.getFreeHeap());
  decodeTalk();

  nstate = esp_random() | 1;
  randomSeed(esp_random());
  clearAll();

  setupVocoder();
  vocChord();
  setupAmp();
  xTaskCreatePinnedToCore(audioTask, "audio", 16384, nullptr, 10, &audioHandle, 0);
  xTaskCreatePinnedToCore(bankTask, "bank", 12288, nullptr, 1, nullptr, 1);   // voices load while the film plays
}

void loop() {
  static int renderedVoice = 0;
  if (voiceIdx != renderedVoice) {
    renderedVoice = voiceIdx;
    renderBank(renderedVoice, true);
    flash(VOICES[renderedVoice].name);
    if (mode == M_SAM) deckPlay(curPad, false);
  }
  drawFrame();
  static uint32_t lastLog = 0;
  if (STATUS_LOG && millis() - lastLog > 2000) {
    lastLog = millis();
    logf("t=%lus mode %s fx %s step %d blocks %lu heap %u stackfree %u knob %.2f\n", millis() / 1000,
         MODE_NAMES[mode], FX_NAMES[fx], curStep, (unsigned long)audioBlocks, (unsigned)ESP.getFreeHeap(),
         audioHandle ? (unsigned)uxTaskGetStackHighWaterMark(audioHandle) : 0u, knobVal);
  }
  delay(5);
}

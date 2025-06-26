#include <avr/io.h>
#include <avr/interrupt.h>

constexpr uint32_t kBaud = 250000;   // must match kArduinoBaud in host
constexpr uint32_t kTimerHz = 40000;    // ISR tick rate (25 µs)
constexpr uint8_t  kAxes = 3;
constexpr uint32_t kUsPerTick = 1000000UL / kTimerHz;

volatile uint32_t period_ticks[kAxes] = {0,0,0};  // 0 ⇒ axis disabled
volatile uint32_t tick_accum [kAxes] = {0,0,0};

// -------- Fast pin macros (UNO R3) -----------------------------------------
#define STEP_PORT PORTB
#define STEP_DDR  DDRB
constexpr uint8_t STEP_MASK[kAxes] = {_BV(1), _BV(2), _BV(3)}; // pins 9,10,11

#define DIR_PORT PORTD
#define DIR_DDR  DDRD
constexpr uint8_t DIR_MASK[kAxes]  = {_BV(6), _BV(7), _BV(0)}; // pins 6,7,8

inline void stepHigh(uint8_t a){ STEP_PORT |=  STEP_MASK[a]; }
inline void stepLow (uint8_t a){ STEP_PORT &= ~STEP_MASK[a]; }
inline void setDir  (uint8_t a,bool dir){ if(dir) DIR_PORT |= DIR_MASK[a]; else DIR_PORT &= ~DIR_MASK[a]; }

// -------- Timer‑1 setup (CTC) ---------------------------------------------
void setupTimer1()
{
    cli();
    TCCR1A = 0;                               // CTC, OC1A disconnected
    TCCR1B = _BV(WGM12) | _BV(CS10);          // clk/1 prescaler
    OCR1A  = (F_CPU / kTimerHz) - 1;          // compare at 25 µs intervals
    TIMSK1 = _BV(OCIE1A);                     // enable compare‑match IRQ
    sei();
}

ISR(TIMER1_COMPA_vect)
{
    for(uint8_t a=0; a<kAxes; ++a)
    {
        uint32_t p = period_ticks[a];
        if(!p) continue;                      // axis stopped

        if(++tick_accum[a] >= p)              // time for next pulse?
        {
            tick_accum[a] = 0;
            stepHigh(a);                      // rising edge
        }
        else if(tick_accum[a] == 1)
        {
            stepLow(a);                       // ensure ≥25 µs low‑time
        }
    }
}

// ---------------------------------------------------------------------------
void setup()
{
    STEP_DDR |= STEP_MASK[0] | STEP_MASK[1] | STEP_MASK[2];
    DIR_DDR  |= DIR_MASK[0]  | DIR_MASK[1]  | DIR_MASK[2];

    STEP_PORT &= ~(STEP_MASK[0] | STEP_MASK[1] | STEP_MASK[2]);
    DIR_PORT  &= ~(DIR_MASK[0]  | DIR_MASK[1]  | DIR_MASK[2]);

    Serial.begin(kBaud);
    while(!Serial) ;                          // wait for USB‑CDC
    setupTimer1();
}

// Expected line: "F,fx,fy,fz,dirBits"
void parseLine(const char* line)
{
    if(line[0] != 'F') return;
    double fx=0, fy=0, fz=0; uint8_t dir=0;
    if(sscanf(line, "F,%lf,%lf,%lf,%hhu", &fx, &fy, &fz, &dir) != 4) return;

    const double freq[kAxes] = {fx, fy, fz};
    for(uint8_t a=0; a<kAxes; ++a)
    {
        setDir(a, dir & (1u << a));

        if(freq[a] < 1.0)                     // below 1 Hz ⇒ stop axis
        {
            period_ticks[a] = 0;
            continue;
        }
        double T_us = 1e6 / freq[a];          // period in µs/pulse
        uint32_t ticks = T_us / kUsPerTick;
        if(ticks < 2) ticks = 2;              // need ≥1 tick high & low
        period_ticks[a] = ticks;
    }
}

void loop()
{
    static char buf[64];
    static uint8_t pos = 0;

    while(Serial.available())
    {
        char c = Serial.read();
        if(c == '\n')
        {
            buf[pos] = '\0';
            parseLine(buf);
            pos = 0;
        }
        else if(pos < sizeof(buf)-1)
        {
            buf[pos++] = c;
        }
    }
}


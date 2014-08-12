#define USE_IRQ_TIMER1 false

#define LED     13      // PORTC7
#define SR_CLK  2       // PORTD1
#define SR_SIN  3       // PORTD0

#define SR_PORT     PORTD
#define SR_CLK_MASK _BV(1)
#define SW_CLK_SIN  _BV(0)

#define LAYER_PORT      PORTF       // Port on which the layer control bits are
#define LAYER_PORT_MASK B00001111   // Mask to turn off every layer control bits

#define FX_MAX          1       // Max number of effect
#define FX_DURATION_MS  5000    // How long an effect is being played before switching to a new one

#define WAIT_MS 500 // Delay between animation frame

#define X_AXIS 1
#define Y_AXIS 2
#define Z_AXIS 3

const byte SWITCH_LAYER_PIN[4] = {A0, A1, A2, A3};
const byte SWITCH_LAYER_MASK[4] = {_BV(7), _BV(6), _BV(5), _BV(4)};

boolean isInit = false;
boolean alwaysOn = false;

#if USE_IRQ_TIMER1
byte currentlayer = 0;
#endif

volatile uint16_t buffer[4] = {0x0000, 0x0000, 0x0000, 0x0000};


void setup()
{
    // initialize the digital pin as an output.
    pinMode(LED, OUTPUT);     
    pinMode(SR_CLK, OUTPUT);     
    pinMode(SR_SIN, OUTPUT);     

    for (byte p=0; p<4; p++)
    {
        pinMode(SWITCH_LAYER_PIN[p], OUTPUT);
        digitalWrite(SWITCH_LAYER_PIN[p], LOW);
    }

#if USE_IRQ_TIMER1 == true    
    cli();//stop interrupts

    //set timer1 interrupt at 735Hz
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0
    // set compare match register for 735hz increments
    OCR1A = 2720;// = (16*10^6) / (735*8) - 1 (must be <65536)
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS11 bit for 8 prescaler
    TCCR1B |= (1 << CS11);
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);

    sei();//allow interrupts
#endif

    Serial.begin(115200);

    delay(1);

    while (!Serial)
    {
        message(3);
        delay(100);
    }
}

// Notify user via high-speed LED flash
void message(int nb)
{
    for (int i=0; i<nb; i++)
    {
        digitalWrite(LED, HIGH);
        delay(10);
        digitalWrite(LED, LOW);
        delay(90);
    }
}

void shift(byte axis, byte direction)
{
    if (axis == Z_AXIS)
    {
        if (direction >= 0)
        {
            for(int p=0; p<3; p++)
            {
                buffer[p] = buffer[p+1];
            }
            buffer[3] = 0x00;
        }
        else
        {
            for(int p=3; p>1; p--)
            {
                buffer[p] = buffer[p-1];
            }
            buffer[0] = 0x00;
        }
    }
}

// Turn 0-16 LEDs of each layer on at random.
void effectRandom(unsigned long elapsed)
{
    if (!isInit)
    {
        randomSeed(analogRead(A4));
        isInit = true;
    }
    for(byte p=0; p<4; p++)
    {
        buffer[p] = random(0xFFFF + 1);
    }
}

// Turn cube full on.
void effectOn(unsigned long elapsed)
{
    if (!isInit)
    {
        for(byte p=0; p<4; p++)
        {
            buffer[p] = 0xFFFF;
        }
        isInit = true;
    }
}

void effectRainDown(unsigned long elapsed)
{
    if (!isInit)
    {
        randomSeed(analogRead(A4));
        isInit = true;
    }
}

// Process next frame of cube animation
void updateAnimation()
{
    static byte currentfx = 1;
    static unsigned long elapsed = millis();

    if (alwaysOn) currentfx = 2;

    switch(currentfx) {
    case 1: 
        effectRandom(millis());
        break;
    case 2: 
        effectOn(millis());
        break;
    }

    if (millis() - elapsed >= FX_DURATION_MS)
    {
        // Reset effect
        isInit = false;

        // Switch to new effect
        if (currentfx >= FX_MAX) 
            currentfx = 1;
        else
            currentfx = currentfx + 1;

        // Notify user of effect change
        message(3);

        elapsed = millis();
    }
}

#if USE_IRQ_TIMER1 == true

inline void inttimer1(void)
{
    // Turn off all layers
    LAYER_PORT &= LAYER_PORT_MASK; 

    // Set current layer anodes
    for(int i=0; i<16; i++)
    {
        SR_PORT &= ~SR_CLK_MASK; // CLR
        SR_PORT = (SR_PORT & ~(1 << PORTD0)) | (((buffer[currentlayer] >> i) & 1) << PORTD0);
        SR_PORT |= SR_CLK_MASK; // SET
    } 

    // Turn on current layer
    LAYER_PORT |= SWITCH_LAYER_MASK[currentlayer];

    // Next layer
    if (currentlayer >= 3) 
        currentlayer = 0;
    else
        currentlayer++;
}

ISR(TIMER1_COMPA_vect) { inttimer1(); }

void loop()
{
    updateAnimation();
    delay(WAIT_MS);
}

#else

void loop()
{
    static int i = 0;
    int timeon[4] = {76, 450, 1000, 2500};

    static unsigned long t = 0;
    static unsigned long n = 0;

    static unsigned long ton = 0;
    static unsigned long ton_n = 0;

    static unsigned long toff = 0;
    static unsigned long toff_n = 0;

    if (t >= 3000)
    {
        unsigned long t1 = micros();
        unsigned long t1off;
        unsigned long t1on;

        static unsigned long thistime = millis();

        for(int currentlayer=0; currentlayer<4; currentlayer++)
        {
            LAYER_PORT &= LAYER_PORT_MASK; 
            
            t1off = t1;
            for(int i=0; i<16; i++)
            {
                SR_PORT &= ~SR_CLK_MASK;
                SR_PORT = (SR_PORT & ~(1 << PORTD0)) | (((buffer[currentlayer] >> i) & 1) << PORTD0);
                SR_PORT |= SR_CLK_MASK;
            }
            LAYER_PORT |= SWITCH_LAYER_MASK[currentlayer];

            t1on = micros();
            toff += t1on - t1off;

            delayMicroseconds(timeon[i]);
        }

        /*for(int currentlayer=0; currentlayer<4; currentlayer++)
        {
            LAYER_PORT &= LAYER_PORT_MASK; 

            t1off = t1;
            for(int i=0; i<16; i++)
            {
                bitClear(SR_PORT, 1);
                bitWrite(SR_PORT, 0, bitRead(buffer[currentlayer], i));
                bitSet(SR_PORT, 1);
            }
            LAYER_PORT |= SWITCH_LAYER_MASK[currentlayer];

            t1on = micros();
            toff += t1on - t1off;

            delayMicroseconds(timeon[i]);
        }*/

        LAYER_PORT &= LAYER_PORT_MASK; 

        unsigned long t2 = micros();
        ton += t2 - t1on;
        t += t2 - t1;
        n++;

        if ((millis() - thistime) >= WAIT_MS)
        {
            // loop
            updateAnimation();
            thistime = millis();
        }
    }
    else
    {
        static boolean done = false;
        if (!done)
        {
            Serial.print("\n");
            Serial.print("\n");
            Serial.print("LED CUBE STATISTICS\n");
            Serial.print("\n");
            Serial.print("time on delay="); Serial.print(timeon[i]); Serial.print("us\n");
            Serial.print("loop="); Serial.print((double)t / (double)n); Serial.print("us\n");
            Serial.print("toff/loop="); Serial.print((double)toff / (double)n); Serial.print("us\n");
            Serial.print("ton/loop="); Serial.print((double)ton / (double)n); Serial.print("us\n");
            Serial.print("duty cycle="); Serial.print(100.0 * ((double)ton / (double)n) / ((double)t / (double)n)); Serial.print("%\n");
            Serial.print("refresh rate="); Serial.print(1000000.0 / ((double)t / (double)n)); Serial.print("Hz\n");
            Serial.print("\n");
        }
        done = true;
    }
}

#endif

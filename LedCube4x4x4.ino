/// 1. Compare time spent between effectRandom1 and effectRandom2 (setVoxel vs brute force)
/// 2. Compare performance between 2D buffer vs 1D buffer
/// 3. Compare performance between port manipulation and bit functions (set anodes loops)
/// 4. Decide if animation functions use setVoxel or brute force when possible
/// 5. Decide whether or not to use a fixed frame rate for the animation (currently, yes)
///    i.e.
///      with a fixed delay of 3 units (...)
///        random effect (low-cost)  ::...::...::...       (2 units to compute animation, period of 5 units - fast rate)
///        raindrop fx (expensive)   ::::...::::...::::... (4 units to compute animation, period of 7 units - slow rate)
/// 6. Check if animation routine takes longer than WAIT_MS (i.e. message(10))
/// 7. Test animation effects!

#define USE_IRQ_TIMER1 false
#define USE_BIT_BUFFER 16

#define LED     13      // PORTC7
#define SR_CLK  2       // PORTD1
#define SR_SIN  3       // PORTD0

#define SR_PORT     PORTD
#define SR_CLK_MASK _BV(1)
#define SW_CLK_SIN  _BV(0)

#define RANDOM_PIN A4

#define LAYER_PORT      PORTF       // Port on which the layer control bits are
#define LAYER_PORT_MASK B00001111   // Mask to turn off every layer control bits

#define FX_MAX          4       // Max number of effects
#define FX_LOOPING      255     // For looping animations
#define FX_DURATION_MS  5000    // How long an effect is being played before switching to a new one

#define WAIT_MS 500 // Delay between animation frame

#define X_AXIS 1
#define Y_AXIS 2
#define Z_AXIS 3

const byte SWITCH_LAYER_PIN[4] = {A0, A1, A2, A3};
const byte SWITCH_LAYER_MASK[4] = {_BV(7), _BV(6), _BV(5), _BV(4)};

boolean isInit = false;

#if USE_IRQ_TIMER1
byte currentLayer = 0;
#endif

#if USE_BIT_BUFFER == 8
#elif USE_BIT_BUFFER == 16
#elif USE_BIT_BUFFER == 64
#endif

#if USE_BIT_BUFFER == 8
volatile uint8_t buffer[4][4] = {{0x0, 0x0, 0x0, 0x0},
                                 {0x0, 0x0, 0x0, 0x0},
                                 {0x0, 0x0, 0x0, 0x0},
                                 {0x0, 0x0, 0x0, 0x0}};
#elif USE_BIT_BUFFER == 16
volatile uint16_t buffer[4] = {0x0000, 0x0000, 0x0000, 0x0000};
#elif USE_BIT_BUFFER == 64
volatile uint64_t buffer[4] = {0x0000000000000000,
                               0x0000000000000000, 
                               0x0000000000000000, 
                               0x0000000000000000};
#endif

void setup()
{
    // Initialize the digital pin as an output.
    pinMode(LED, OUTPUT);
    pinMode(SR_CLK, OUTPUT);
    pinMode(SR_SIN, OUTPUT);

    pinMode(RANDOM_PIN, INPUT);

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


// Returns a byte with a row of 1's drawn in it
// byteline(2,5) gives B00111100
byte byteline(int start, int end)
{
	return ((0xff<<start) & ~(0xff<<(end+1)));
}

// Normalize arguments (i.e. ensure x1 < x2)
void normArgs(byte ix1, byte ix2, byte *ox1, byte *ox2)
{
	if (ix1>ix2)
	{
		byte tmp;
		tmp = ix1;
		ix1= ix2;
		ix2 = tmp;
	}
	*ox1 = ix1;
	*ox2 = ix2;
}

// Low-Level Graphic Func
// Set a voxel on or off
void setVoxel(byte x, byte y, byte z, bool state)
{
#if USE_BIT_BUFFER == 8
    if (state == true)
        bitSet(buffer[z][y], x);
    else
        bitClear(buffer[z][y], x);
#else
    if (state == true)
        bitSet(buffer[z], (y * 4) + x);
    else
        bitClear(buffer[z], (y * 4) + x);
#endif
}

// Low-Level Graphic Func
// Set a plane on or off
void setPlane(byte axis, byte n, bool state)
{
#if USE_BIT_BUFFER == 8
    if (axis == Z_AXIS)
    {
        for (byte y=0; y<4; y++)
            buffer[n][y] = state ? B1111 : B0000;
    } 
    else if (axis == Y_AXIS)
    {
        for (byte z=0; z<4; z++)
            buffer[z][n] = state ? B1111 : B0000;
    }
    else // X_AXIS
    {
        for (byte z=0; z<4; z++)
            for (byte y=0; y<4; y++)
                bitWrite(buffer[z][y], n, state ? 1 : 0);
    }
#else
    if (axis == Z_AXIS)
    {
        buffer[n] = state ? 0xFF : 0x00;
    } 
    else if (axis == Y_AXIS)
    {
        for (byte z=0; z<4; z++)
        {
            if (state == true)
                buffer[z] |= B1111 << n;
            else
                buffer[z] &= ~(B1111 << n);
        }
    }
    else // X_AXIS
    {
        for (byte z=0; z<4; z++)
            for (byte y=0; y<4; y++)
            {
                if (state == true)
                    buffer[z] |= B1 << ((y * 4) + n);
                else
                    buffer[z] &= ~(B1 << ((y * 4) + n));
            }
    }
#endif
}

// Low-Level Graphic Func
// Draw wireframe box
void boxWireframe(byte x1, byte y1, byte z1, byte x2, byte y2, byte z2)
{
	byte ix;
	byte iy;
	byte iz;

	normArgs(x1, x2, &x1, &x2);
	normArgs(y1, y2, &y1, &y2);
	normArgs(z1, z2, &z1, &z2);

	// Lines along X axis
	for (ix=x1;ix<=x2;ix++)
	{
		setVoxel(ix,y1,z1, true);
		setVoxel(ix,y1,z2, true);
		setVoxel(ix,y2,z1, true);
		setVoxel(ix,y2,z2, true);
    }

	// Lines along Y axis
	for (iy=y1;iy<=y2;iy++)
	{
		setVoxel(x1,iy,z1, true);
		setVoxel(x1,iy,z2, true);
		setVoxel(x2,iy,z1, true);
		setVoxel(x2,iy,z2, true);
	}

	// Lines along Z axis
	for (iz=z1;iz<=z2;iz++)
	{
		setVoxel(x1,y1,iz, true);
		setVoxel(x1,y2,iz, true);
		setVoxel(x2,y1,iz, true);
		setVoxel(x2,y2,iz, true);
	}
}

// Low-Level Graphic Func
// Shift entire buffer in the specified direction
void shift(byte axis, byte direction)
{
#if USE_BIT_BUFFER == 8
    if (axis == Z_AXIS)
    {
        if (direction >= 0)
        {
            for(int z=0; z<3; z++)
                for(int y=0; y<3; y++)
                    buffer[z][y] = buffer[z+1][y];
            for(int y=0; y<3; y++)
                buffer[3][y] = 0x00;
        }
        else
        {
            for(int z=3; z>1; z--)
                for(int y=0; y<3; y++)
                    buffer[z][y] = buffer[z-1][y];
            for(int y=0; y<3; y++)
                buffer[0][y] = 0x00;
        }
    }
#else
    if (axis == Z_AXIS)
    {
        if (direction >= 0)
        {
            for(int z=0; z<3; z++)
                buffer[z] = buffer[z+1];
            buffer[3] = 0x00;
        }
        else
        {
            for(int z=3; z>1; z--)
                buffer[z] = buffer[z-1];
            buffer[0] = 0x00;
        }
    }
#endif
}


// Turn 0-16 LEDs of each layer on at random.
// Use setVoxel
void effectRandom1(unsigned long elapsed)
{
    if (!isInit)
    {
        randomSeed(analogRead(RANDOM_PIN));
        isInit = true;
    }

    for(byte z=0; z<4; z++)
        for(int y=0; y<4; y++)
            for(int x=0; x<4; x++)
            {
                byte r = random(2); // 0 to 1
                setVoxel(x, y, z, r == 1);
            }
}

// Turn 0-16 LEDs of each layer on at random.
// Use buffer brute force
void effectRandom2(unsigned long elapsed)
{
    if (!isInit)
    {
        randomSeed(analogRead(RANDOM_PIN));
        isInit = true;
    }

#if USE_BIT_BUFFER == 8
    for(byte z=0; z<4; z++)
        for(int y=0; y<4; y++)
        {
            byte r = random(B1111 + 1);
            buffer[z][y] = r;
        }
#else
    for(byte z=0; z<4; z++)
    {
        byte r = random(0xFF + 1);
        buffer[z] = r;
    }
#endif
}

// Turn cube full on.
void effectOn(unsigned long elapsed)
{
    if (!isInit)
    {
    for(byte z=0; z<4; z++)
        for(int y=0; y<4; y++)
            for(int x=0; x<4; x++)
                setVoxel(x, y, z, true);
        isInit = true;
    }
}

// Turn 1 to 4 LEDs on at the top of the cube
// and shift cube down one layer to simulate
// rain drops.
void effectRainDown(unsigned long elapsed)
{
    if (!isInit)
    {
        randomSeed(analogRead(RANDOM_PIN));
        isInit = true;
    }

    shift(Z_AXIS, -1);

    byte n = random(4) + 1; // 1 to 4
    for (byte i=0; i<n; i++)
    {
        byte x = random(4); // 0 to 3
        byte y = random(4); // 0 to 3
        setVoxel(x, y, 3, true); 
    }
}

// Used to debug drawing functions
void effectDebug(unsigned long elapsed)
{
    if (!isInit)
    {
        isInit = true;
    }
}

// Process next frame of cube animation
// If fx is FX_LOOPING, animations are being looped
// If fx is not FX_LOOPING, animation is fixed to fx
void updateAnimation(byte fx)
{
    static byte currentfx = 1;
    static unsigned long elapsed = millis();

    if (fx != FX_LOOPING) currentfx = fx;

    switch(currentfx) {
    case 1: 
        effectRandom1(millis());
        break;
    case 2: 
        effectRandom2(millis());
        break;
    case 3: // Use updateAnimation(3) for always on cube
        effectOn(millis());
        break;
    case 4: 
        effectRainDown(millis());
        break;
    case 128: // Don't include in loop (128 > FX_MAX) use updateAnimation(128)
        effectDebug(millis());
        break;
    }

    if (fx == FX_LOOPING)
    {
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
        SR_PORT = (SR_PORT & ~(1 << PORTD0)) | (((buffer[currentLayer] >> i) & 1) << PORTD0);
        SR_PORT |= SR_CLK_MASK; // SET
    } 

    // Turn on current layer
    LAYER_PORT |= SWITCH_LAYER_MASK[currentLayer];

    // Next layer
    if (currentLayer >= 3) 
        currentLayer = 0;
    else
        currentLayer++;
}

ISR(TIMER1_COMPA_vect) { inttimer1(); }

void loop()
{
    unsigned long t1 = millis();
    updateAnimation(FX_LOOPING);
    unsigned long t2 = millis();

    if ((WAIT_MS - (t2 - t1)) > 0)
        delay(WAIT_MS - (t2 - t1));
    else
        message(10);
}

#else

void loop()
{
    static int toindex = 0;
    int timeon[4] = {76, 450, 1000, 2500};

    static unsigned long t = 0;
    static unsigned long n = 0;

    static unsigned long ton = 0;
    static unsigned long ton_n = 0;

    static unsigned long toff = 0;
    static unsigned long toff_n = 0;

    static unsigned long tanim = 0;
    static unsigned long nanim = 0;

    if (t >= 3000)
    {
        unsigned long t1 = micros();
        unsigned long t1off;
        unsigned long t1on;

        static unsigned long thistime = millis();

        for(int currentLayer=0; currentLayer<4; currentLayer++)
        {
            LAYER_PORT &= LAYER_PORT_MASK; 
            
            t1off = t1;
#if USE_BIT_BUFFER == 8
            for(int y=0; y<4; y++)
            {
                for(int x=0; x<4; x++)
                {
                    SR_PORT &= ~SR_CLK_MASK;
                    SR_PORT = (SR_PORT & ~(1 << PORTD0)) | (((buffer[currentLayer][y] >> x) & 1) << PORTD0);
                    SR_PORT |= SR_CLK_MASK;
                }
            }
            LAYER_PORT |= SWITCH_LAYER_MASK[currentLayer];
#else
            for(int i=0; i<16; i++)
            {
                SR_PORT &= ~SR_CLK_MASK;
                SR_PORT = (SR_PORT & ~(1 << PORTD0)) | (((buffer[currentLayer] >> i) & 1) << PORTD0);
                SR_PORT |= SR_CLK_MASK;
            }
            LAYER_PORT |= SWITCH_LAYER_MASK[currentLayer];
#endif
            t1on = micros();
            toff += t1on - t1off;

            delayMicroseconds(timeon[toindex]);
        }

//        for(int currentLayer=0; currentLayer<4; currentLayer++)
//        {
//            LAYER_PORT &= LAYER_PORT_MASK; 
//
//            t1off = t1;
//#if USE_BIT_BUFFER == 8
//            for(int y=0; y<4; y++)
//            {
//                for(int x=0; x<4; x++)
//                {
//                    bitClear(SR_PORT, 1);
//                    bitWrite(SR_PORT, 0, bitRead(buffer[currentLayer][y], x));
//                    bitSet(SR_PORT, 1);
//                }
//            }
//            LAYER_PORT |= SWITCH_LAYER_MASK[currentLayer];
//#else
//            for(int i=0; i<16; i++)
//            {
//                bitClear(SR_PORT, 1);
//                bitWrite(SR_PORT, 0, bitRead(buffer[currentLayer], i));
//                bitSet(SR_PORT, 1);
//            }
//            LAYER_PORT |= SWITCH_LAYER_MASK[currentLayer];
//#endif
//
//            t1on = micros();
//            toff += t1on - t1off;
//
//            delayMicroseconds(timeon[toindex]);
//        }

        LAYER_PORT &= LAYER_PORT_MASK; 

        unsigned long t2 = micros();
        ton += t2 - t1on;
        t += t2 - t1;
        n++;

        // Warning: time spent here is not computed
        // but it will affect overall brightness (duty cycle)
        // since it will lengthen toff after last layer
        if ((millis() - thistime) >= WAIT_MS)
        {
            // loop
            thistime = millis();

            unsigned long t1anim = micros();
            updateAnimation(FX_LOOPING);
            unsigned long t2anim = micros();

            tanim += t2anim - t1anim;
            nanim++;
        }
    }
    else
    {
        static boolean done = false;
        if (!done)
        {
            Serial.begin(115200);

            delay(1);

            while (!Serial)
            {
                message(3);
                delay(100);
            }

            Serial.print("\n");
            Serial.print("\n");
            Serial.print("LED CUBE STATISTICS\n");
            Serial.print("\n");
            Serial.print("time on delay="); Serial.print(timeon[toindex]); Serial.print("us\n");
            Serial.print("loop="); Serial.print((double)t / (double)n); Serial.print("us\n");
            Serial.print("toff/loop="); Serial.print((double)toff / (double)n); Serial.print("us\n");
            Serial.print("ton/loop="); Serial.print((double)ton / (double)n); Serial.print("us\n");
            Serial.print("duty cycle="); Serial.print(100.0 * ((double)ton / (double)n) / ((double)t / (double)n)); Serial.print("%\n");
            Serial.print("refresh rate="); Serial.print(1000000.0 / ((double)t / (double)n)); Serial.print("Hz\n");
            Serial.print("tanim="); Serial.print((double)tanim / (double)nanim); Serial.print("us\n");
            Serial.print("\n");
        }
        done = true;
    }
}

#endif

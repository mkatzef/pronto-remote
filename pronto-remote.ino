/* 
 * pronto-remote.ino
 * 
 * A sketch to turn an Arduino Nano into a universal remote. Records and transmits
 * infrared codes using the Pronto hex format.
 * 
 * The Pronto hex format consists of 16-bit unsigned integers as follows:
 * Header:
 * 1 uint 0
 * 1 uint frequency specifier
 * 1 uint data length in burst pairs
 * 1 uint trailer length in burst pairs, frequently 0
 * 
 * Data, of length described by last two header fields:
 * Pairs of uints - LED on cycle count, LED off cycle count
 * Where cycle counts are the number of IR carrier signal pulses for that period.
 * 
 * Written by Marc Katzef
 */

#define CPU_FREQUENCY 16000000

#define OUTPUT_SIGNAL_PIN 3
#define INPUT_SIGNAL_PIN 2

#define MAX_BURST_PAIRS 110
#define PRONTO_HEADER_LENGTH 4

#define TIMER1_PRESCALE 64
#define TIMER1_DEFAULT_PERIOD_MICROS 50000
#define CARRIER_PWM_PRESCALE 2

#define DEFAULT_CARRIER_FREQUENCY 38000
#define DEFAULT_CARRIER_DUTY_CYCLE 0.333
#define CARRIER_FACTOR 4145146 // 1000000 / 0.241246 from "http://www.remotecentral.com/features/irdisp2.htm"
#define RECORDER_TIMEOUT_MICROS 175000

// To reduce the effect of noise
#define MIN_RECORDER_COUNT 6 // Minimum number of entries in the burst pair buffer to consider a recording valid
#define MIN_CARRIER_COUNT 3 // Minimum duration of a cycle to consider it valid
#define BRIDGE_THRESHOLD 0x170 // The minimum length of a cycle for it to be considered a separator between codes
#define TIMER_ACCURACY_IMPROVEMENT 1000 // 3 decimal places

// General functionality
bool g_intsEnabled = true;
const int32_t g_prontoBufferLength = PRONTO_HEADER_LENGTH + 2 * MAX_BURST_PAIRS;
volatile uint16_t g_prontoBuffer[g_prontoBufferLength];
volatile uint16_t g_BPPointer; // The pronto buffer index to write to or read from
volatile uint16_t g_currentProntoLength; // Current number of items in pronto buffer

// Tx
typedef enum CARRIER_STATES {STATE_OFF=0, STATE_ON};
uint32_t g_carrierFrequency = DEFAULT_CARRIER_FREQUENCY;
volatile bool g_busySending;
uint8_t g_carrierWidth; // OCR2B's value when the carrier is enabled (0 otherwise)
uint16_t g_ticksPerCarrierTx;

// Rx
volatile bool g_busyRecording;
volatile bool g_recordTimedOut;
uint16_t g_ticksPerCarrierRx;
uint8_t g_inputSignalBitMask;
volatile uint8_t *g_inputSignalPort;
volatile bool g_allowedBridge; // Whether or not a long period of inactivity was recorded
volatile uint8_t g_wrapCountRx; // Number of times the timer elapsed since last sample
volatile uint32_t g_pinChangeCount; // To keep track of activity before incrementing wrap count

/* 
 * Sets the on period of the output PWM signal depending on the given carrier
 * state.
 */
void setCarrierState(CARRIER_STATES state) {
    switch (state) {
        case STATE_ON:
            OCR2B = g_carrierWidth;
            break;
        case STATE_OFF:
            OCR2B = 0;
            break;
        default:
            OCR2B = 0;
    }
}


/* 
 * Sets the frequency and duty cycle of the output LED PWM signal to match the
 * given values.
 */
void setCarrierPWM(uint32_t frequency, float dutyCycle) {
    bool prevIntState = g_intsEnabled;
    noInterrupts();
    g_intsEnabled = false;

    g_carrierFrequency = frequency;
    uint8_t periodCount = CPU_FREQUENCY / CARRIER_PWM_PRESCALE / frequency;
    OCR2A = periodCount;
    g_carrierWidth = (uint32_t)(periodCount * dutyCycle);
    TCNT2 = 0;
    
    if (prevIntState) {
        interrupts();
        g_intsEnabled = true;
    }
}


/* 
 * Loads timer 1 with the value which would trigger the timer ISR after the given
 * number of microseconds.
 * Maximum time between interrupts is
 * 2^32 * TIMER1_PRESCALE / CPU_FREQUENCY
 * For a prescale of 64, and CPU frequency of 16 MHz, this is about 5 hours
 */
void setNextInterrupt(uint32_t microseconds) {
    TCNT1 = 0;
  
    uint64_t roomyTemp = CPU_FREQUENCY;
    roomyTemp /= TIMER1_PRESCALE;
    roomyTemp *= microseconds;
    roomyTemp /= 1000000;

    bool prevIntState = g_intsEnabled;
    noInterrupts();
    g_intsEnabled = false;
    
    OCR1A = roomyTemp;

    if (prevIntState) {
        interrupts();
        g_intsEnabled = true;
    }
}


/* 
 * The interrupt service routine for timer 1. Used for toggling carrier state at
 * the right times, and as a watchdog timer while recording.
 */
ISR(TIMER1_COMPA_vect)
{
    TCNT1 = 0;
    noInterrupts();
    
    if (g_busySending) {
        if (g_BPPointer < g_currentProntoLength) {            
            uint32_t focusInt = g_prontoBuffer[g_BPPointer];
            uint8_t focusState = (g_BPPointer - PRONTO_HEADER_LENGTH) % 2;

            if (focusState == 0) {
                setCarrierState(STATE_ON);
            } else {
                setCarrierState(STATE_OFF);
            }
            
            uint32_t ticksForNextTimeout = (2*(focusInt * g_ticksPerCarrierTx) + TIMER_ACCURACY_IMPROVEMENT) / (2*TIMER_ACCURACY_IMPROVEMENT);
            OCR1A = ticksForNextTimeout;
            
            if (TCNT1 > ticksForNextTimeout) { // Number of carriers transmitted before leaving the function.
                OCR1A = 4; // Trigger timeout soon (avoid wrap around). Say 2us -> (16MHz / prescaler) * 2u = 4 ticks
                TCNT1 = 0;
            }
            
            g_BPPointer++;
            
        } else {
            g_busySending = false;
            setCarrierState(STATE_OFF);
            setNextInterrupt(TIMER1_DEFAULT_PERIOD_MICROS);
        }
        
    } else if (g_busyRecording) {
        if (g_pinChangeCount != 0) {
            g_pinChangeCount = 0;
            g_wrapCountRx++;
        } else if (g_BPPointer > PRONTO_HEADER_LENGTH) {
            g_recordTimedOut = true;
            g_busyRecording = false; // Prevent reaching this point again before finalizing the recording
        }
    } else {
        setNextInterrupt(TIMER1_DEFAULT_PERIOD_MICROS);
    }

    interrupts();
}


/* 
 * The interrupt service routing to track changes from the IR receiver. Calculates
 * the number of carrier pulses that should have occured since the last pin change
 * and stores the value in the pronto buffer.
 */
void pinChangeIntHandler(void) {
    uint32_t sampleTime = TCNT1;
    TCNT1 = 0;
    uint8_t newPinState = (*g_inputSignalPort & g_inputSignalBitMask) != 0;
    
    noInterrupts();
    uint8_t expectedPinState = 1 - (g_BPPointer % 2);

    bool shifted = false;
    if ((newPinState != expectedPinState) && (g_BPPointer < g_prontoBufferLength)) {
        g_prontoBuffer[g_BPPointer] = 0;
        g_BPPointer++;
        shifted = true;
    }
    
    g_busyRecording = true;
    if (g_BPPointer >= PRONTO_HEADER_LENGTH) {
        if (g_BPPointer >= g_prontoBufferLength) {
            g_recordTimedOut = true;
            g_busyRecording = false;
        } else {
            int32_t difference = sampleTime;
            if (g_wrapCountRx) {
                difference += g_wrapCountRx * OCR1A;
            }
            
            // Rounded integer division
            uint16_t ticks = (2*(TIMER_ACCURACY_IMPROVEMENT * difference) + g_ticksPerCarrierRx) / (2*g_ticksPerCarrierRx);
            
            if (ticks > BRIDGE_THRESHOLD) {
                if (g_allowedBridge) {
                    g_recordTimedOut = true;
                    g_busyRecording = false;
                } else {
                    g_allowedBridge = true;
                    g_prontoBuffer[g_BPPointer] = ticks;
                
                    // Update group
                    g_BPPointer++;
                    g_wrapCountRx = 0;
                    g_pinChangeCount++;
                }
            } else if (ticks >= MIN_CARRIER_COUNT) {
                g_prontoBuffer[g_BPPointer] = ticks;
                
                // Update group
                g_BPPointer++;
                g_wrapCountRx = 0;
                g_pinChangeCount++;
            } else {
                TCNT1 = sampleTime;
                if (shifted) {
                    g_BPPointer--;
                }
            }
        }
    } else {
        // Partial update group
        g_BPPointer++;
        g_pinChangeCount++;
    }
    interrupts();
}


/* 
 * Reads pronto hex (as ASCII) from serial. Writes it to the pronto buffer.
 * Returns the number of values added to the buffer.
 */
uint16_t fillBurstPairBuffer(char firstChar, uint16_t* burstPairsBuffer) {
    int16_t count = -1;
    uint16_t currentInt = 0;
    uint8_t currentPosition = 0;
    String packet = (String)firstChar;
    
    while ((Serial.available() || count == -1) && count < g_prontoBufferLength) {
        if (count == -1) {
            count = 0;
        } else {
            packet = Serial.readString();
        }
        for (uint16_t i = 0; i < packet.length(); i++) {
            char digit = packet[i];
            int8_t value = -1;
            if (digit >= '0' && digit <= '9') {
                value = digit - '0';
            } else if (digit >= 'A' && digit <= 'F') {
                value = digit - 'A' + 10;
            } else if (digit >= 'a' && digit <= 'f') {
                value = digit - 'a' + 10;
            }
            
            if (value >= 0) {                
                currentInt |= value << (4 * (3 - currentPosition));
    
                currentPosition++;
                if (currentPosition == 4) {
                    burstPairsBuffer[count] = currentInt;
                    count++;
                    currentPosition = 0;
                    currentInt = 0;
                }
            }
        }
    }
    g_currentProntoLength = count;
    return count;
}


/* 
 * Reads pronto hex (as bytes) from serial. Writes it to the pronto buffer.
 * Returns the number of values added to the buffer.
 * 
 * Intended to be a faster equivalent to "fillBurstPairBuffer" for inter-device
 * communication.
 */
uint16_t fillBurstPairBufferBytes(char firstChar, uint16_t* burstPairsBuffer) {
    int16_t count = 0;
    uint16_t currentInt = 0;
    uint8_t currentPosition = 0;
    
    String packet;
    while (Serial.available() && count < g_prontoBufferLength) {    
        packet = Serial.readString();
        for (uint16_t i = 0; i < packet.length(); i++) {
            int16_t value = packet[i];
            
            if ((value >= 0) && (value <= 255)) {                
                currentInt |= value << (8 * (1 - currentPosition));
                currentPosition++;
                if (currentPosition == 2) {
                    burstPairsBuffer[count] = currentInt;
                    count++;
                    currentPosition = 0;
                    currentInt = 0;
                }
            }
        }
    }
    g_currentProntoLength = count;
    return count;
}


/* 
 * Sets global variables for pinChangeInterrupt to begin filling the pronto buffer.
 * Once the buffer has filled or text is available through serial (to cancel),
 * returns the number of values written to the buffer.
 * 
 * May corrupt current pronto buffer contents if recording is cancelled.
 */
uint16_t recordPronto(uint16_t *burstPairsBuffer) {
    int32_t count = 0;
    double roomyTicksPerCarrier = CPU_FREQUENCY;
    roomyTicksPerCarrier *= TIMER_ACCURACY_IMPROVEMENT;
    roomyTicksPerCarrier /= DEFAULT_CARRIER_FREQUENCY;
    roomyTicksPerCarrier /= TIMER1_PRESCALE;

    g_ticksPerCarrierRx = round(roomyTicksPerCarrier); // (!) simpler rounding
    
    while (count >= 0 && count <= MIN_RECORDER_COUNT) {
        noInterrupts();
        g_intsEnabled = false;
    
        attachInterrupt(digitalPinToInterrupt(INPUT_SIGNAL_PIN), pinChangeIntHandler, CHANGE);
        g_BPPointer = PRONTO_HEADER_LENGTH - 1; // pointing out of bounds, for first time sample
        
        g_pinChangeCount = 0;
        g_recordTimedOut = false;
        g_busyRecording = false; // Set to true in first pin change interrupt 
        g_allowedBridge = false;
        g_wrapCountRx = 0;
        
        setNextInterrupt(RECORDER_TIMEOUT_MICROS);
            
        interrupts();
        g_intsEnabled = true;
    
        while (!g_recordTimedOut) { // Wait for timeout, buffer fill, or cancel
            if (Serial.available()) {
                // No longer clears the input text - assumes interrupted by another command to process.
                count = -1;
                g_BPPointer = 0;
                break;
            }
        }
        
        detachInterrupt(digitalPinToInterrupt(INPUT_SIGNAL_PIN));
        g_busyRecording = false;

        if (count >= 0) {
            if ((g_BPPointer - PRONTO_HEADER_LENGTH) % 2) {
                if (g_BPPointer >= g_prontoBufferLength) {
                    g_BPPointer--;
                } else { 
                    g_prontoBuffer[g_BPPointer++] = 0;
                }
            }
            
            g_prontoBuffer[0] = 0;
            g_prontoBuffer[1] = (float)CARRIER_FACTOR / DEFAULT_CARRIER_FREQUENCY;
            g_prontoBuffer[2] = (g_BPPointer - PRONTO_HEADER_LENGTH) / 2;
            g_prontoBuffer[3] = 0;
        }
    
        g_currentProntoLength = g_BPPointer;
        count = g_BPPointer - PRONTO_HEADER_LENGTH;
    }
    
    return g_BPPointer;
}


/* 
 * Sets the global variables for the timer ISR to send the IR code defined by the
 * first "count" entries in the global pronto buffer.
 */
void transmitPronto(uint16_t count) {
    bool isValid = true;
    uint16_t bpCount = 0;
    uint32_t carrierFrequency;
    
    if (count < PRONTO_HEADER_LENGTH) {
        isValid = false;
    } else if (g_prontoBuffer[0] != 0) {
        isValid = false;
    } else {
        carrierFrequency = CARRIER_FACTOR / g_prontoBuffer[1];
        setCarrierPWM(carrierFrequency, DEFAULT_CARRIER_DUTY_CYCLE);

        bpCount = g_prontoBuffer[2] + g_prontoBuffer[3];
    }

    if ((count - PRONTO_HEADER_LENGTH)/2 != bpCount) {
        isValid = false;
    }
                        
    if (isValid) {
        bool prevIntState = g_intsEnabled;
        noInterrupts();
        g_intsEnabled = false;

        double roomyTicksPerCarrier = CARRIER_PWM_PRESCALE;
        roomyTicksPerCarrier *= TIMER_ACCURACY_IMPROVEMENT;
        roomyTicksPerCarrier *= CPU_FREQUENCY;
        roomyTicksPerCarrier /= g_carrierFrequency;
        roomyTicksPerCarrier /= TIMER1_PRESCALE;
        roomyTicksPerCarrier /= CARRIER_PWM_PRESCALE;
        g_ticksPerCarrierTx = (uint16_t)roomyTicksPerCarrier;
    
        if ((roomyTicksPerCarrier - g_ticksPerCarrierTx) >= 0.5) {
            g_ticksPerCarrierTx++;
        }
        
        g_BPPointer = PRONTO_HEADER_LENGTH;
        g_currentProntoLength = count;
        g_busySending = true;
    
        if (prevIntState) {
            interrupts();
            g_intsEnabled = true;
        }

        while (g_busySending);
    }
}


/* 
 * Reads the first character in the serial buffer to identify which function to
 * perform. The accepted letters and their commands are as follows:
 * R - record pronto
 * S - send pronto
 * I - print stored information
 * T - fill pronto buffer using the more direct (bytes) method
 * Any other symbol is interpreted as the beginning of an ASCII pronto buffer string.
 * 
 * Returns the length of the currently stored pronto code. Returns this value * -1
 * if the pronto code must be transmitted.
 */
int16_t parseInput(uint16_t* burstPairsBuffer) {
    int16_t count = 0;
    
    int16_t firstChar = Serial.read(); // Should refactor to use Serial.peek()
    delay(5);
    if (firstChar == 'R' || firstChar == 'r') { // Asked to record pronto
        count = recordPronto(burstPairsBuffer);
    } else if (firstChar == 'S' || firstChar == 's') { // Asked to send current pronto
        count = -g_currentProntoLength;
    } else if (firstChar == 'I' || firstChar == 'i') {
        count = g_currentProntoLength;
    } else if (firstChar == 'T' || firstChar == 't') { // TURBO
        count = -fillBurstPairBufferBytes(firstChar, burstPairsBuffer);
    } else { // Given pronto
        count = -fillBurstPairBuffer(firstChar, burstPairsBuffer);
    }
    return count;
}


/* 
 * Initializes pins, the carrier PWM signal, and timer.
 */
void setup() {
    Serial.begin(115200);
    Serial.setTimeout(50);
    pinMode(OUTPUT_SIGNAL_PIN, OUTPUT);
    pinMode(INPUT_SIGNAL_PIN, INPUT);

    g_inputSignalBitMask = digitalPinToBitMask(INPUT_SIGNAL_PIN);
    g_inputSignalPort = portInputRegister(digitalPinToPort(INPUT_SIGNAL_PIN));
    
    bool prevIntState = g_intsEnabled;
    noInterrupts();
    g_intsEnabled = false;
    
    // Carrier signal PWM
    TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM20);
    TCCR2B = _BV(WGM22) | _BV(CS20);

    setCarrierPWM(DEFAULT_CARRIER_FREQUENCY, DEFAULT_CARRIER_DUTY_CYCLE);
    setCarrierState(STATE_OFF);
    
    TCCR1A = 0;
    TCCR1B = 0;
    
    setNextInterrupt(TIMER1_DEFAULT_PERIOD_MICROS);
    
    TCCR1B |= 1 << WGM12;
    TCCR1B |= (1 << CS11) | (1 << CS10); // Clock/64 prescaler (!) try clock/8 (remove CS10)
    TIMSK1 |= 1 << OCIE1A; // Enable timer interrupt
    
    if (prevIntState) {
        interrupts();
        g_intsEnabled = true;
    }
}


/*
 * Waits for the last command to complete and for data to become available through
 * serial before filling the buffer, printing the current contents, or transmitting
 * the stored code.
 */
void loop() {
    while (g_busySending || g_busyRecording);
    while (!Serial.available());
    
    int16_t count = parseInput(g_prontoBuffer);
    Serial.println(count);
    if (count > 0) {
        Serial.println(count, DEC);
        char message[5] = {0};
        for (uint16_t i = 0; i < count; i++) {
            sprintf(message, "%.4X", g_prontoBuffer[i]);
            Serial.println(message);
        }
    } else if (count < 0) {
        transmitPronto(-count);
    }
}


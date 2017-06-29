#define CPU_FREQUENCY 16000000
#define CARRIER_PWM_PRESCALE 2

#define OUTPUT_SIGNAL_PIN 3
#define INPUT_SIGNAL_PIN 2

#define MAX_BURST_PAIRS 220
#define PRONTO_HEADER_LENGTH 4
#define TIMER1_PRESCALE 64

#define DEFAULT_CARRIER_FREQUENCY 38000
#define DEFAULT_CARRIER_DUTY_CYCLE 0.333

#define TIMER_ACCURACY_IMPROVEMENT 1000 // 3 decimal places

#define TIMER1_DEFAULT_PERIOD_MICROS 50000

#define CARRIER_FACTOR 4145146 // 1000000 / 0.241246 from "http://www.remotecentral.com/features/irdisp2.htm"
#define RECORDER_TIMEOUT_MICROS 175000

// Reduce the effect of noise
#define MIN_RECORDER_COUNT 6
#define MIN_CARRIER_COUNT 3
#define BRIDGE_THRESHOLD 0x170 

// All functionality
static _Bool g_intsOn = true;
static volatile uint16_t g_prontoBuffer[PRONTO_HEADER_LENGTH + MAX_BURST_PAIRS] = {0};
static volatile uint16_t g_BPPointer = 0;
static volatile uint16_t g_maxBPPointer = 0; // number of items in pronto buffer.

// Tx
typedef enum {STATE_OFF = 0, STATE_ON} CARRIER_STATES;
static uint32_t g_carrierFrequency = DEFAULT_CARRIER_FREQUENCY;
static volatile _Bool g_busySending = false;
static uint8_t g_carrierWidth = 0; // OCR2B's value when the carrier is in use (0 otherwise)
static uint16_t g_ticksPerCarrierTx = 0;

// Rx
static volatile uint32_t g_pinChangeCount = 0; // Reset by timeout timer.
static volatile _Bool g_busyRecording = false;
static volatile _Bool g_recordTimedOut = false;
static volatile uint16_t g_lastSampleTime = 0;
static uint16_t g_ticksPerCarrierRx = 0;
static uint16_t g_recordingCarrierFrequency = DEFAULT_CARRIER_FREQUENCY;
static uint8_t inputSignalBitMask;
static volatile uint8_t *inputSignalPort;
static volatile _Bool g_allowedBridge = false;
static volatile uint8_t g_wrapCountRx = 0;


// --------------------------------------------------
// Setup stuff

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


void setCarrierPWM(uint32_t frequency, float dutyCycle) {
    _Bool prevIntState = g_intsOn;
    noInterrupts();
    g_intsOn = false;

    g_carrierFrequency = frequency;
    uint8_t periodCount = CPU_FREQUENCY / CARRIER_PWM_PRESCALE / frequency;
    OCR2A = periodCount;
    g_carrierWidth = (uint32_t)(periodCount * dutyCycle);
    TCNT2 = 0;
    
    if (prevIntState) {
        interrupts();
        g_intsOn = true;
    }
}

// Assumes given time is larger than that taken to set up interrupt.
void setNextInterrupt(uint32_t microSeconds) {
    TCNT1 = 0; //Moved to caller side (earlier).
  
    uint64_t roomyTemp = CPU_FREQUENCY;
    roomyTemp /= TIMER1_PRESCALE;
    roomyTemp *= microSeconds;
    roomyTemp /= 1000000;

    _Bool prevIntState = g_intsOn;
    noInterrupts();
    g_intsOn = false;
    
    OCR1A = (uint32_t)roomyTemp;

    if (prevIntState) {
        interrupts();
        g_intsOn = true;
    }
}

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(50);
    pinMode(OUTPUT_SIGNAL_PIN, OUTPUT);
    pinMode(INPUT_SIGNAL_PIN, INPUT);

    inputSignalBitMask = digitalPinToBitMask(INPUT_SIGNAL_PIN);
    inputSignalPort = portInputRegister(digitalPinToPort(INPUT_SIGNAL_PIN));
    
    _Bool prevIntState = g_intsOn;
    noInterrupts();
    g_intsOn = false;
    
    // Carrier Signal
    TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM20);
    TCCR2B = _BV(WGM22) | _BV(CS20);

    setCarrierPWM(DEFAULT_CARRIER_FREQUENCY, DEFAULT_CARRIER_DUTY_CYCLE);
    setCarrierState(STATE_OFF);
    
    TCCR1A = 0;
    TCCR1B = 0;
    
    setNextInterrupt(TIMER1_DEFAULT_PERIOD_MICROS);
    
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS11) | (1 << CS10); // clk/8 prescaler
    TIMSK1 |= 1 << OCIE1A; // enable timer interrupt
    
    if (prevIntState) {
        interrupts();
        g_intsOn = true;
    }
}
// --------------------------------------------------


// --------------------------------------------------
// ISRs

ISR(TIMER1_COMPA_vect)
{
    TCNT1 = 0;
    noInterrupts(); // Assuming interrupts may be preemptive
    
    if (g_busySending) {
        // find burst pairs from pointer (or set g_busySending to false)
        // figure out if high or low
        // calculate time to wait for burst pair defined pulses
        // set the timer to that time
        // increment pointer
        
        if (g_BPPointer < g_maxBPPointer) {            
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
            g_busyRecording = false; // move outside
        }
    } else {
        setNextInterrupt(TIMER1_DEFAULT_PERIOD_MICROS);
    }

    interrupts();
}


void pinChangeIntHandler() {
    uint32_t sampleTime = TCNT1;
    TCNT1 = 0;
    uint8_t newPinState = (*inputSignalPort & inputSignalBitMask) != 0;
    
    noInterrupts();
    uint8_t expectedPinState = 1 - (g_BPPointer % 2);

    _Bool shifted = false;
    if ((newPinState != expectedPinState) && (g_BPPointer < (MAX_BURST_PAIRS + PRONTO_HEADER_LENGTH))) {
        g_prontoBuffer[g_BPPointer] = 0;
        g_BPPointer++;
        shifted = true;
    }
    
    g_busyRecording = true;
    if (g_BPPointer >= PRONTO_HEADER_LENGTH) {
        if (g_BPPointer >= (MAX_BURST_PAIRS + PRONTO_HEADER_LENGTH)) {
            g_recordTimedOut = true;
            g_busyRecording = false;
        } else {
            int32_t difference = ((int32_t)sampleTime);// - g_lastSampleTime;
            if (g_wrapCountRx) {
                difference += g_wrapCountRx * OCR1A;
            }
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
// --------------------------------------------------


// --------------------------------------------------
// Serin, Rx, Tx

// Returns number of ints put in buffer
uint16_t fillBurstPairBuffer(char firstChar, uint16_t* burstPairsBuffer) {
    int16_t count = -1;
    uint16_t currentInt = 0;
    uint8_t currentPosition = 0;
    
    String packet = (String)firstChar;
    while ((Serial.available() || count == -1) && count <= (MAX_BURST_PAIRS + PRONTO_HEADER_LENGTH)) {
        if (count == -1) {;
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
    g_maxBPPointer = count;
    return count;
}

uint16_t fillBurstPairBufferBytes(char firstChar, uint16_t* burstPairsBuffer) {
    int16_t count = 0;
    uint16_t currentInt = 0;
    uint8_t currentPosition = 0;
    
    String packet;
    while (Serial.available() && count <= (MAX_BURST_PAIRS + PRONTO_HEADER_LENGTH)) {    
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
    g_maxBPPointer = count;
    return count;
}

uint16_t recordPronto(uint16_t * burstPairsBuffer) {
    int32_t count = 0;
    double roomyTicksPerCarrier = CPU_FREQUENCY;
    roomyTicksPerCarrier *= TIMER_ACCURACY_IMPROVEMENT;
    roomyTicksPerCarrier /= g_recordingCarrierFrequency;
    roomyTicksPerCarrier /= TIMER1_PRESCALE;

    g_ticksPerCarrierRx = (uint16_t)roomyTicksPerCarrier;
    if ((roomyTicksPerCarrier - g_ticksPerCarrierRx) >= 0.5) {
        g_ticksPerCarrierRx++;
    }
    
    while (count >= 0 && count <= MIN_RECORDER_COUNT) {
        noInterrupts();
        g_intsOn = false;
        
        attachInterrupt(digitalPinToInterrupt(INPUT_SIGNAL_PIN), pinChangeIntHandler, CHANGE);
        g_BPPointer = PRONTO_HEADER_LENGTH - 1; // pointing out of bounds, for first time sample
        
        g_pinChangeCount = 0;
        g_recordTimedOut = false;
        g_busyRecording = false; // Set to true in first pin change interrupt 
        g_allowedBridge = false;
        g_wrapCountRx = 0;
        
        setNextInterrupt(RECORDER_TIMEOUT_MICROS);
            
        interrupts();
        g_intsOn = true;
    
        while (!g_recordTimedOut) { // Wait for timeout, buffer fill, or cancel
            //delay(15);
            if (Serial.available()) {
                // No longer clears the input text - assumes interrupted by another command to process.
                count = -1;
                g_BPPointer = 0;
            }
        }
        
        detachInterrupt(digitalPinToInterrupt(INPUT_SIGNAL_PIN));
        g_busyRecording = false;

        if (count >= 0) {
            if ((g_BPPointer - PRONTO_HEADER_LENGTH) % 2) {
                if (g_BPPointer >= MAX_BURST_PAIRS + PRONTO_HEADER_LENGTH) {
                    g_BPPointer--;
                } else { 
                    g_prontoBuffer[g_BPPointer++] = 0;
                }
            }
            
            g_prontoBuffer[0] = 0;
            g_prontoBuffer[1] = (uint16_t)((float)CARRIER_FACTOR / g_recordingCarrierFrequency);
            g_prontoBuffer[2] = (g_BPPointer - PRONTO_HEADER_LENGTH) / 2;
            g_prontoBuffer[3] = 0;
        }
    
        g_maxBPPointer = g_BPPointer;
        count = g_BPPointer - PRONTO_HEADER_LENGTH;
    }
    return g_BPPointer;
}


// Use header info, set g_BPPointer such that the timer ISR will send.
// Count is the length of the pronto buffer (including header)
void transmitPronto(uint16_t count) {
    _Bool isValid = true;
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
        _Bool prevIntState = g_intsOn;
        noInterrupts();
        g_intsOn = false;

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
        g_maxBPPointer = count;
        g_busySending = true;
    
        if (prevIntState) {
            interrupts();
            g_intsOn = true;
        }

        while (g_busySending);
    }
}
// --------------------------------------------------


int16_t parseInput(uint16_t* burstPairsBuffer) {
    int16_t count = 0;
    
    int16_t firstChar = Serial.read();
    delay(5);
    if (firstChar == 'R' || firstChar == 'r') { // Asked to record pronto
        count = recordPronto(burstPairsBuffer);
    } else if (firstChar == 'S' || firstChar == 's') { // Asked to send current pronto
        count = -g_maxBPPointer;
    } else if (firstChar == 'I' || firstChar == 'i') {
        count = g_maxBPPointer;
    } else if (firstChar == 'T' || firstChar == 't') { // TURBO
        count = -fillBurstPairBufferBytes(firstChar, burstPairsBuffer);
    } else { // Given pronto
        count = -fillBurstPairBuffer(firstChar, burstPairsBuffer);
    }
    return count;
}

void loop() {
    while (g_busySending || g_busyRecording);
    while (!Serial.available());
    
    int16_t count = parseInput(g_prontoBuffer);
    
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

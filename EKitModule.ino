#include <USBHost_t36.h>
#include <ADC.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SerialFlash.h>
#include <EEPROM.h>
#include "AudioSample.h" // User must generate this with wav2sketch.py!

// GUItool: begin automatically generated code
AudioPlayMemory          playMem1;       //xy=100,100
AudioPlayMemory          playMem2;       //xy=100,150
AudioPlayMemory          playMem3;       //xy=100,200
AudioPlayMemory          playMem4;       //xy=100,250
AudioPlayMemory          playMem5;       //xy=100,300
AudioPlayMemory          playMem6;       //xy=100,350
AudioPlayMemory          playMem7;       //xy=100,400
AudioPlayMemory          playMem8;       //xy=100,450
AudioMixer4              mixer1;         //xy=300,125
AudioMixer4              mixer2;         //xy=300,325
AudioMixer4              mixerFinal;     //xy=500,225
AudioOutputI2S           i2s1;           //xy=700,225

AudioConnection          patchCord1(playMem1, 0, mixer1, 0);
AudioConnection          patchCord2(playMem2, 0, mixer1, 1);
AudioConnection          patchCord3(playMem3, 0, mixer1, 2);
AudioConnection          patchCord4(playMem4, 0, mixer1, 3);
AudioConnection          patchCord5(playMem5, 0, mixer2, 0);
AudioConnection          patchCord6(playMem6, 0, mixer2, 1);
AudioConnection          patchCord7(playMem7, 0, mixer2, 2);
AudioConnection          patchCord8(playMem8, 0, mixer2, 3);
AudioConnection          patchCord9(mixer1, 0, mixerFinal, 0);
AudioConnection          patchCord10(mixer2, 0, mixerFinal, 1);
AudioConnection          patchCord11(mixerFinal, 0, i2s1, 0);
AudioConnection          patchCord12(mixerFinal, 0, i2s1, 1);
// GUItool: end automatically generated code

ADC *adc = new ADC();

// --- Configuration Constants ---
#define NUM_VOICES       8

// Global Calibration State
bool monitorMode = false;
int globalThreshold = 9; // Fallback legacy threshold

// --- Voice Management ---
struct VoiceManager {
  AudioPlayMemory* voices[NUM_VOICES];

  VoiceManager() {
    voices[0] = &playMem1; voices[1] = &playMem2; voices[2] = &playMem3; voices[3] = &playMem4;
    voices[4] = &playMem5; voices[5] = &playMem6; voices[6] = &playMem7; voices[7] = &playMem8;
  }

  void play(const unsigned int *sampleData) {
    if (sampleData == nullptr) return;

    // 1. Find a voice that is not playing
    for (int i = 0; i < NUM_VOICES; i++) {
        if (!voices[i]->isPlaying()) {
            voices[i]->play(sampleData);
            return;
        }
    }
  }
};

VoiceManager* voiceMgr;

// --- Enhanced Trigger Class ---
class DrumTrigger {
public:
  int analogPin;
  int midiNote;
  const unsigned int *sampleData; // Pointer to flash memory audio array
  int threshold;
  int id; // For EEPROM mapping

  // State
  int peakValue = 0;
  enum channelState {ch_idle, ch_triggered};
  channelState state = ch_idle;
  unsigned long lastStrikeTime = 0;
  bool noteActive = false;
  
  // Calibration
  int eepromAddr;

  DrumTrigger(int _id, int _pin, int _note, const unsigned int *_sample) : id(_id), analogPin(_pin), midiNote(_note), sampleData(_sample) {
    // EEPROM Memory Map: ID * 4 (int size)
    eepromAddr = id * sizeof(int);
    
    // Load threshold from EEPROM, default to global if invalid
    int savedThresh;
    EEPROM.get(eepromAddr, savedThresh);
    if (savedThresh <= 0 || savedThresh > 1023) {
        threshold = globalThreshold;
    } else {
        threshold = savedThresh;
    }
  }

  void setThreshold(int newThresh) {
    threshold = newThresh;
    EEPROM.put(eepromAddr, threshold);
  }

  void checkAndTrigger() {
    int sensorValue = adc->analogRead(analogPin);
    unsigned long currentTime = millis();  // Get the current time
    unsigned long debounceTime = 15; // Increased slightly for stability

    // Auto-calibration / Monitor
    if (monitorMode && sensorValue > threshold) {
         Serial.print("ID:"); Serial.print(id);
         Serial.print(" Val:"); Serial.println(sensorValue);
    }

    if (state == ch_idle) {
      if (sensorValue > (peakValue + threshold)) {
        state = ch_triggered;
        peakValue = sensorValue;
        lastStrikeTime = currentTime; 
      } else if (sensorValue <= (peakValue - threshold)) {
        peakValue = sensorValue; // Follow signal down
      }
    }

    if (state == ch_triggered) {
      if (sensorValue > peakValue) {
        peakValue = sensorValue;  // Track rising peak
      } else if (sensorValue <= (peakValue - threshold) && (currentTime - lastStrikeTime) > debounceTime) {
        // Peak detected!
        
        // 1. Calculate Velocity
        int effectivePeak = (peakValue > 1023) ? 1023 : peakValue;
        int velocity = map(effectivePeak, threshold, 800, 1, 127); 
        velocity = constrain(velocity, 1, 127);

        // 2. Send MIDI
        usbMIDI.sendNoteOn(midiNote, velocity, 1);
        usbMIDI.sendNoteOff(midiNote, 0, 1); // Immediate NoteOff for triggers

        // 3. Play Audio
        voiceMgr->play(sampleData);

        // 4. Reset
        noteActive = false;
        state = ch_idle;           
        peakValue = sensorValue;   
        lastStrikeTime = currentTime;
      }
    }
  }
};

const byte numTriggers = 8;
DrumTrigger* triggers[numTriggers];

void initTriggers() {
    // Initialize pointers. ID is index.
    // Ensure these variable names match what wav2sketch.py produces!
    // Format: AudioSample + TitleCase(Filename)
    
    #ifdef AUDIOSAMPLE_H 
      // Only compile this if the header exists, otherwise use nulls so code compiles (but no sound)
      triggers[0] = new DrumTrigger(0, A0, 36, AudioSampleKick);   
      triggers[1] = new DrumTrigger(1, A1, 38, AudioSampleSnare);  
      triggers[2] = new DrumTrigger(2, A2, 42, AudioSampleHh_cl);  
      triggers[3] = new DrumTrigger(3, A3, 46, AudioSampleHh_op);  
      triggers[4] = new DrumTrigger(4, A4, 43, AudioSampleTom1);   
      triggers[5] = new DrumTrigger(5, A5, 47, AudioSampleTom2);   
      triggers[6] = new DrumTrigger(6, A6, 49, AudioSampleCrash);  
      triggers[7] = new DrumTrigger(7, A7, 51, AudioSampleRide);   
    #else
       // Fallback for compilation before running script
       const unsigned int* dummy = nullptr;
       for(int i=0; i<8; i++) triggers[i] = new DrumTrigger(i, A0+i, 40+i, dummy);
    #endif
}

// Adding the CC control for the potentiometer on pin A9
const byte numCCs = 1;
ccControl ccControls[] = {
  {A8, 4},  // CC #4 for hi-hat position control in Studio Drummer
};

void setup() {
  Serial.begin(9600); 
  AudioMemory(16); // Allocate memory for audio

  // No SD init needed anymore!
  
  usbMIDI.begin();

  // ADC settings
  adc->adc0->setResolution(10);
  adc->adc0->setAveraging(8);  // Slight increase for smoother signal
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); 

  voiceMgr = new VoiceManager();
  initTriggers();
  
  Serial.println("E-Kit Module Ready (Flash Audio).");
  Serial.println("Commands: 'm' (monitor), 's' (save), 'l' (load), 't <id> <val>' (set threshold)");
}

void processSerialCommands() {
    if (Serial.available()) {
        char cmd = Serial.read();
        
        switch (cmd) {
            case 'm':
                monitorMode = !monitorMode;
                Serial.print("Monitor Mode: "); Serial.println(monitorMode ? "ON" : "OFF");
                break;
            case 's':
                Serial.println("Saving configuration...");
                // Saving is implicit in setThreshold, but we could add bulk save here if needed.
                // For now, confirm.
                break;
             case 'l':
                Serial.println("Reloading configuration..."); 
                // Re-init limits?
                for (int i=0; i<numTriggers; i++) {
                   int val; EEPROM.get(triggers[i]->eepromAddr, val);
                   if (val > 0 && val < 1024) triggers[i]->threshold = val;
                   Serial.print("ID "); Serial.print(i); Serial.print(": "); Serial.println(triggers[i]->threshold);
                }
                break;
             case 't': {
                int id = Serial.parseInt();
                int val = Serial.parseInt();
                if (id >= 0 && id < numTriggers) {
                    triggers[id]->setThreshold(val);
                    Serial.print("Set ID "); Serial.print(id); Serial.print(" to "); Serial.println(val);
                } else {
                    Serial.println("Invalid ID");
                }
                break;
             }
        }
    }
}

void loop() {
  processSerialCommands();
  checkNotes();
  checkCC();
}

void checkNotes() {
  for (int i = 0; i < numTriggers; i++) {
    triggers[i]->checkAndTrigger();
  }
}

void checkCC() {
  for (int i = 0; i < numCCs; i++) {
    ccControls[i].checkAndSend();
  }
}
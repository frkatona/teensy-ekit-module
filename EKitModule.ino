#include <USBHost_t36.h>
#include <ADC.h>

ADC *adc = new ADC();

// Both the INCREASE from baseline to trigger rise detection as well as the DECREASE from peak to trigger peak detection
int detectionThreshold = 7;  // Balance between hit detection and noise rejection
// int MIDIChannel = 1;

struct noteTrigger {
  int analogPin;
  int midiNote;

  int peakValue = 0;
  bool noteActive = false;
  unsigned long lastStrikeTime = 0;

  enum channelState {ch_idle, ch_triggered};
  channelState state = ch_idle;

  void checkAndTrigger() {
    int sensorValue = adc->analogRead(analogPin);

    if (state == ch_idle) {
      if (sensorValue > (peakValue + detectionThreshold)) {
        state = ch_triggered;
        peakValue = sensorValue;
      } else if (sensorValue <= (peakValue - detectionThreshold)) {
        peakValue = sensorValue;
      }
    }

    if (state == ch_triggered) {
      if (sensorValue > peakValue) {
        peakValue = sensorValue;  // Update peak value
      } else if (sensorValue <= (peakValue - detectionThreshold)) {
        // Signal has settled; trigger the MIDI note
        float velocity = map(peakValue + 100, 1, 1024, 1, 127);
        usbMIDI.sendNoteOn(midiNote, velocity, 1);
        usbMIDI.sendNoteOff(midiNote, 0, 1);
        noteActive = false;
        state = ch_idle;           // Reset to idle state
        peakValue = sensorValue;   // Trail the peak lower now that it has settled
      }
    }
  }
};

// Struct for managing MIDI CC from a potentiometer
struct ccControl {
  int analogPin;
  int ccNumber;
  int lastValue = -1;  // Initialize with an invalid value to force the first send

  void checkAndSend() {
    int sensorValue = adc->analogRead(analogPin);
    // Serial.println(sensorValue);
    delay(50);
    int ccValue = map(sensorValue, 840, 0, 0, 127);  // Map to MIDI CC range
    if (ccValue - lastValue > 2 || ccValue - lastValue < -2) {  // Only send if value has changed
      usbMIDI.sendControlChange(ccNumber, ccValue, 1);
      lastValue = ccValue;
      Serial.println(ccValue);
    }
  }
};

const byte numTriggers = 8;
noteTrigger triggers[] = {
  {A0, 46},
  {A1, 61},
  {A2, 62},
  {A3, 63},
  {A4, 64},
  {A5, 65},
  {A6, 66},
  {A7, 67},
};

// Adding the CC control for the potentiometer on pin A9
const byte numCCs = 1;
ccControl ccControls[] = {
  {A8, 4},  // CC #4 for hi-hat position control in Studio Drummer
};

void setup() {
  Serial.begin(9600); // Uncomment if needed for debugging
  usbMIDI.begin();

  // ADC settings
  adc->adc0->setResolution(10);
  adc->adc0->setAveraging(6);  // Number of samples to average over (adjust for noise rejection vs peak accuracy)
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);  // Highest sample conversion time
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);  // Highest sampling speed
}

void loop() {
  // checkNotes();
  checkCC();
}

void checkNotes() {
  for (int i = 0; i < numTriggers; i++) {
    triggers[i].checkAndTrigger();
  }
}

void checkCC() {
  for (int i = 0; i < numCCs; i++) {
    ccControls[i].checkAndSend();
  }
}
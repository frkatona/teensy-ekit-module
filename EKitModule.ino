#include <USBHost_t36.h>
#include <ADC.h>

ADC *adc = new ADC();

int detectionThreshold = 0.005 * 1024;

struct MidiTrigger {
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
        float velocity = map(peakValue, 1, 1024, 30, 127);
        usbMIDI.sendNoteOn(midiNote, velocity, 1);
        usbMIDI.sendNoteOff(midiNote, 0, 1);
        noteActive = false;
        state = ch_idle;           // Reset to idle state
        peakValue = sensorValue;   // Trail the peak lower now that it has settled
      }
    }
  }
};

// Number of triggers
const byte numTriggers = 4;

// Array of MidiTrigger objects
MidiTrigger triggers[] = {
  {A0, 60},
  {A1, 61},
  {A2, 62},
  {A8, 63},
};

void setup() {
  // Serial.begin(9600); // commenting out when not debugging with serial writes to isolate necessary expenses for latency reduction testing
  usbMIDI.begin();

  // ADC settings (still need to dial in)
  adc->adc0->setResolution(10);
  adc->adc0->setAveraging(4);  // number of samples to average over (play with noise rejection vs peak accuracy + latency)
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);  // highest sample conversion time
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);  // highest sampling speed
}

void loop() {
  for (int i = 0; i < numTriggers; i++) {
    triggers[i].checkAndTrigger();
  }
}
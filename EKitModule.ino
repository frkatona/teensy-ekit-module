#include <USBHost_t36.h>
#include <ADC.h>

ADC *adc = new ADC();

int sensorValue1, sensorValue2, sensorValue3, sensorValue8;
int threshold = 20;
unsigned long debounceDelay = 30;

struct MidiTrigger {
  int analogPin;
  int threshold;
  int midiNote;
  int &peakValue;
  bool &noteActive;
  unsigned long &lastStrikeTime;
  
  enum channelState {ch_idle, ch_triggered};
  channelState state = ch_idle;     // Start in idle state

  void checkAndTrigger() {
    int sensorValue = adc->analogRead(analogPin);
    int somePercentOfMax = 0.2 * 1024;  // Example value, adjust as needed

    if (state == ch_idle) {
      if (sensorValue > (peakValue + somePercentOfMax)) {
        state = ch_triggered;
        peakValue = sensorValue;
      } else if (sensorValue <= (peakValue - somePercentOfMax)) {
        peakValue = sensorValue;
      }
    }

    if (state == ch_triggered) {
      if (sensorValue > peakValue) {
        peakValue = sensorValue;  // Update peak value
      } else if (sensorValue <= (peakValue - somePercentOfMax)) {
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

// Global variables to manage states
const byte numTriggers = 4;
int peakValue1 = 0, peakValue2 = 0, peakValue3 = 0, peakValue8 = 0;
bool noteActive1 = false, noteActive2 = false, noteActive3 = false, noteActive8 = false;
unsigned long lastStrikeTime1 = 0, lastStrikeTime2 = 0, lastStrikeTime3 = 0, lastStrikeTime8 = 0;

MidiTrigger triggers[] = {
  {A0, 20, 60, peakValue1, noteActive1, lastStrikeTime1},
  {A1, 20, 61, peakValue2, noteActive2, lastStrikeTime2},
  {A2, 20, 62, peakValue3, noteActive3, lastStrikeTime3},
  {A8, 20, 63, peakValue8, noteActive8, lastStrikeTime8},
};

void setup() {
  Serial.begin(9600);
  usbMIDI.begin();

  adc->adc0->setResolution(10);  // Set ADC resolution to 10 bits
  adc->adc0->setAveraging(2);  // Set averaging to 4 samples
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);  // Set conversion speed to very high
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);  // Set sampling speed to very high
}

void loop() {
  for (int i = 0; i < numTriggers; i++) {
    triggers[i].checkAndTrigger();
  }
}
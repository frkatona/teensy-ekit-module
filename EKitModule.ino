#include <USBHost_t36.h>
#include <ADC.h>

ADC *adc = new ADC();

//  both the INCREASE from baseline to trigger rise detection as well as the DECREASE from peak to trigger peak detection
int detectionThreshold = 7;  // balance between hit detection and noise rejection...(<10 will let some noise through, >10 won't detect softest hits)

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

const byte numTriggers = 8;
noteTrigger triggers[] = {
  {A0, 60},
  {A1, 61},
  {A2, 62},
  {A3, 63},
  {A4, 64},
  {A5, 65},
  {A6, 66},
  {A7, 67},
};

const byte numCC = 1;
noteTrigger triggers[] = {
  {A9, 60},
};

void setup() {
  // Serial.begin(9600); // commenting out when not debugging with serial writes to isolate necessary expenses for latency reduction testing
  usbMIDI.begin();

  // ADC settings (still need to dial in)
  adc->adc0->setResolution(10);
  adc->adc0->setAveraging(6);  // number of samples to average over (play with noise rejection vs peak accuracy + latency)
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);  // highest sample conversion time
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);  // highest sampling speed
}

void loop() {
  checkNotes();
  checkCC();
}

void checkNotes(){
  for (int i = 0; i < numTriggers; i++) {
    triggers[i].checkAndTrigger();
  }

void checkCC(){

}
}
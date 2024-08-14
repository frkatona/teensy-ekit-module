#include <USBHost_t36.h>
#include <ADC.h>

ADC *adc = new ADC();

// Both the INCREASE from baseline to trigger rise detection as well as the DECREASE from peak to trigger peak detection
int detectionThreshold = 9;  // Balance between hit detection and noise rejection
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
  unsigned long currentTime = millis();  // Get the current time

  // Define debounce time in milliseconds
  unsigned long debounceTime = 10;  // Adjust this value based on your needs

  if (state == ch_idle) {
    if (sensorValue > (peakValue + detectionThreshold)) {
      state = ch_triggered;
      peakValue = sensorValue;
      lastStrikeTime = currentTime;  // Update last strike time when transitioning to triggered state
    } else if (sensorValue <= (peakValue - detectionThreshold)) {
      peakValue = sensorValue;
    }
  }

  if (state == ch_triggered) {
    if (sensorValue > peakValue) {
      peakValue = sensorValue;  // Update peak value
    } else if (sensorValue <= (peakValue - detectionThreshold) && (currentTime - lastStrikeTime) > debounceTime) {
      // Signal has settled and debounce time has passed; trigger the MIDI note
      float velocity = map(peakValue, 1, 650, 1, 127); // Adjusted ceiling from 1024
      // Serial.print("velocity:");
      // Serial.println(velocity);
      usbMIDI.sendNoteOn(midiNote, velocity, 1);
      usbMIDI.sendNoteOff(midiNote, 0, 1);
      noteActive = false;
      state = ch_idle;           // Reset to idle state
      peakValue = sensorValue;   // Trail the peak lower now that it has settled
      lastStrikeTime = currentTime;  // Update last strike time when note is sent
    }
  }
}

};

// Struct for managing MIDI CC from a potentiometer
struct ccControl {
  int analogPin;
  int ccNumber;
  int lastValue = -1;  // Initialize with an invalid value to force the first send
  bool isPedalDown = false;

  void checkAndSend() {
    int sensorValue = adc->analogRead(analogPin);
    int ccValue = map(sensorValue, 840, 5, 0, 127);  // Map to MIDI CC range
    if (ccValue - lastValue > 2 || ccValue - lastValue < -2) {  // Only send if value has changed
      usbMIDI.sendControlChange(ccNumber, ccValue, 1);
      lastValue = ccValue;

      // Check if the pedal has moved to the near-closed position rapidly
      if (ccValue > 120 && !isPedalDown) {  // Adjust threshold as needed
        // int pedalVelocity = map(lastValue - ccValue, 0, 7, 60, 127);
        usbMIDI.sendNoteOn(44, 110, 1); 
        usbMIDI.sendNoteOff(44, 0, 1);
        isPedalDown = true;
        // Serial.print("Pedal Down");
      }

      if (isPedalDown && ccValue <= 120){
        isPedalDown = false;
      }
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
  // Serial.begin(9600); // Uncomment if needed for debugging
  usbMIDI.begin();

  // ADC settings
  adc->adc0->setResolution(10);
  adc->adc0->setAveraging(6);  // Number of samples to average over (adjust for noise rejection vs peak accuracy)
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);  // Highest sample conversion time
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);  // Highest sampling speed
}

void loop() {
  checkNotes();
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
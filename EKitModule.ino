#include <USBHost_t36.h>
#include <ADC.h>


ADC *adc = new ADC();  // Create an ADC object

int sensorValue1, sensorValue2, sensorValue3, sensorValue8;
int threshold = 20;
unsigned long debounceDelay = 30;
int MIDINote = 51;

// ****** stateful implementation ******

void signalToMIDI(float signalAmplitude, int midiNote);

struct MidiTrigger {
  int analogPin;
  int threshold;
  int midiNote;
  int &peakValue;
  bool &noteActive;
  unsigned long &lastStrikeTime;

  void checkAndTrigger() {
    int sensorValue = adc->analogRead(analogPin);
    if (sensorValue > threshold) {
      if (!noteActive) {
        noteActive = true;
        peakValue = sensorValue;
        lastStrikeTime = millis();
      } else if (sensorValue > peakValue) {
        peakValue = sensorValue;  // Update peak value
      }
    } else if (noteActive && millis() - lastStrikeTime > debounceDelay) {
      float velocity = map(peakValue, 1, 1024, 30, 127);
      usbMIDI.sendNoteOn(midiNote, velocity, 1);
      usbMIDI.sendNoteOff(midiNote, 0, 1);
      noteActive = false;
      peakValue = 0;
    }
  }
};

// Global variables to manage states
int peakValue1 = 0, peakValue2 = 0, peakValue3 = 0, peakValue8 = 0;
bool noteActive1 = false, noteActive2 = false, noteActive3 = false, noteActive8 = false;
unsigned long lastStrikeTime1 = 0, lastStrikeTime2 = 0, lastStrikeTime3 = 0, lastStrikeTime8 = 0;

// Array of MidiTrigger structs for each analog input
const byte numTriggers = 4;
MidiTrigger triggers[] = {
  {A0, 20, 60, peakValue1, noteActive1, lastStrikeTime1},
  {A1, 20, 61, peakValue2, noteActive2, lastStrikeTime2},
  {A2, 20, 62, peakValue3, noteActive3, lastStrikeTime3},
  {A8, 20, 63, peakValue8, noteActive8, lastStrikeTime8},
};

// ****** /stateful implementation ******

void setup() {
  Serial.begin(9600);
  usbMIDI.begin();

  adc->adc0->setResolution(10);  // Set ADC resolution to 10 bits
  adc->adc0->setAveraging(2);  // Set averaging to 4 samples
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);  // Set conversion speed to very high
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);  // Set sampling speed to very high
}

void loop() {
  // sensorValue1 = adc->analogRead(A0);
  // sensorValue2 = adc->analogRead(A1);
  // sensorValue3 = adc->analogRead(A2);
  // sensorValue8 = adc->analogRead(A8);
  // scanChannels(sensorValue8, peakValue8, noteActive8, lastStrikeTime8);

  for (int i = 0; i < numTriggers; i++) {
    triggers[i].checkAndTrigger();
  }

  // triggers[3].checkAndTrigger();
}

void scanChannels(int sensorValue, int &peakValue, bool &noteActive, unsigned long &lastStrikeTime) {
  static int previousValue = 0;
  static bool isRising = true;

  if (sensorValue > threshold) {
    if (isRising) {
      // Detect rising edge and update peak
      if (sensorValue > peakValue) {
        peakValue = sensorValue;
      } else {
        // Signal starts to fall, mark peak and prepare for note trigger
        isRising = false;
      }
    }
    if (!noteActive) {
      noteActive = true;
      lastStrikeTime = millis();
    }
  } else if (noteActive && millis() - lastStrikeTime > debounceDelay) {
    // Once the signal drops below threshold, send the MIDI note
    signalToMIDI(peakValue, MIDINote);
    noteActive = false;
    peakValue = 0;
    isRising = true;  // Reset for the next strike
  }
  previousValue = sensorValue;
}

// *********

// void scanChannelsNew() {
//   enum channelState {ch_idle, ch_triggered};                       // list of possible channel states
//   static unsigned int peakLevel[numChannels] = { 0 };              // store peak adc readings for each channel
//   static byte chState[numChannels] = { 0 };                        // current state of each channel, start at idle

//   for (int i = 0; i < numChannels; i++) {
//     unsigned int chReading = analogRead(inputPin[i]);              // read current adc input

//   if (chState[i] == ch_idle) {
//     if (chReading > (peakLevel[i] + somePercentOfMax)) {          // if input rises significantly while idle
//       chState[i] = ch_triggered;                                  // set channel as triggered (signal may still fluctuate)
//       peakLevel[i] = chReading;                                   // set channel peak level to new reading
//     } else if (chReading <= (peakLevel[i] - somePercentOfMax)) {  // if input falls significantly while idle
//       peakLevel[i] = chReading;                                   // trail channel peak level to new lower reading
//       }
//   }
//     // if triggered, continue tracking the highest peak detected
//     // until the input drops by some % of the peak, then consider
//     // the signal settled and activate the trigger (play sample)
//     if (chState[i] == ch_triggered) {                               // if channel is triggered (and the input may not have settled yet)
//       if (chReading > peakLevel[i]) {                               // if there is a higher peak detected during settling time, store it as the new peak
//         peakLevel[i] = chReading;
//       }
//       else if (chReading <= (peakLevel[i] - somePercentOfMax)) {    // if the input level has dropped some amount lower than the peak, consider trigger finalized
//         float level = (peakLevel[i] / float(adcMax));               // set sample playback level based on peak level detected (touch sensitive playback)

//         Serial.print("A"); Serial.print(inputPin[i]);
//         Serial.print(" Triggered at peak level ");
//         Serial.print (peakLevel[i]); Serial.print("/1023 Max. Playing sample at ");
//         Serial.print(round(level * 100)); Serial.print("% level.");
//         Serial.println();

//         playSample[i]();             // call function to play back sound sample assigned to this channel number [i]
//         peakLevel[i] = chReading;    // trail the peak lower now that the signal has fallen enough to have triggered
//         chState[i] = ch_idle;        // triggering is complete so channel is idle, ready to re-trigger
//       }
//     }
//   }
// }

void signalToMIDI(float signalAmplitude, int midiNote) {
  float velocity = map(signalAmplitude, 1, 1024, 30, 127);
  usbMIDI.sendNoteOn(midiNote, velocity, 1);
  usbMIDI.sendNoteOff(midiNote, 0, 1);
}

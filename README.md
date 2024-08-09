# Teensy Ekit Module

This project is an implementation of a MIDI USB trigger system. The system reads analog signals from sensors, processes them, and sends MIDI notes based on the signal amplitude.  

For my system, the signals are first conditioned through several passive components to create the trigger input for the Teensy 4.1.

## Dependencies

- [USBHost_t36.h]()

- [ADC.h]()
  
## Code Structure

### MidiTrigger Struct

```cpp
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
        peakValue = sensorValue;
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
```

### MidiTrigger Array

```cpp
MidiTrigger triggers[] = {
  {A0, 20, 60, peakValue1, noteActive1, lastStrikeTime1},
  {A1, 20, 61, peakValue2, noteActive2, lastStrikeTime2},
  {A2, 20, 62, peakValue3, noteActive3, lastStrikeTime3},
  {A8, 20, 63, peakValue8, noteActive8, lastStrikeTime8},
};
```

The code loops through each struct in the array and checks if the sensor value is above the threshold. If the sensor value is above the threshold, the system will send a MIDI note based on the peak value of the sensor.

## Usage

1. Upload the code to your microcontroller.
2. Connect the sensors to the specified analog pins.
3. Open the serial monitor to view the output.
4. The system will read the sensor values and send MIDI notes based on the detected peaks.

## License

This project is licensed under the MIT License.
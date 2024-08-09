# Teensy Ekit Module

This project is an implementation of a MIDI USB trigger system. The system reads analog signals from sensors, processes them, and sends MIDI notes based on the signal's peak amplitude.  

For my system, the signals are first conditioned through several passive components to generate the signal read into the Teensy 4.1 analog pins.

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

REMEMBER to adjust the length of the trigger array if you have more or fewer sensors.  Is this how C++ programmers really live?

## To-do

- [ ] implement a more robust peak detection algorithm that monitors the trend in the signal rise and decay
- [ ] scale up sensors to match the piezos on my custom ekit, eventually scaling to the number of analog pins
- [ ] measure latency (currently manageable, but noticeable) and optimize ADC logic
- [ ] add digital QoL like MIDI channel selector to the struct
- [ ] extend to triggering local samples on an SD card and pair with i2s audio output
- [ ] rethink the serial writes to graph independent oscilloscope views for the signals to dial in sensitivities with the trim pots

## Resources

- [Gadget Reboot](https://youtu.be/y2Lmbts9IIs) - initial codebase for i2s implementation, circuit design for signal conditioning

## License

This project is licensed under the MIT License.
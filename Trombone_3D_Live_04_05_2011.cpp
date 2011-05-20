/*

Prototype sketch for a trombone-like MIDI controller based on the Arduino hardware.

Hardware:

- An set of four switches used to select an overtone. We use "chording" to allow
  the 4 switches to select overtones. I'm not sure what the most natural method
  of chording is, but let's try the following:
  
  Switch
  3210      Overtone
  0000      OT_1
  0001      OT_2
  0011      OT_3
  0111      OT_4
  1111      OT_5
  1110      OT_7
  1100      OT_8
  1000      OT_8
  
  Switches 0-3 are wired to pull Arduino digital input pins 2-5 low when
  pressed. They are actuated by the four fingers of the right hand.
  
  Additionally, a switch operated by the thumb of the right hand is
  used as a "meta" key that allows the instrment to perform control
  functions, e.g. scene selection in Ableton Live.
  
- A "slide". Currently, this produces pitch bend information, and is implemented
  with a 500mm SpectraSymbol SoftPot linear resistance strip.

- A volume controller, implemented with a FreeScale pressure sensor. The player
  blows into a tube that goes to a "T" - one leg goes to the pressure sensor, and
  the other is open (a "dump tube") so that the player can put air through the
  instrument.

April, 2011
Gordon Good (velo27 <at> yahoo <dot> com)

*/
#include <MidiUart.h>
#include <Midi.h>
#include <Debounce.h>

#include "WProgram.h"
void setup();
void enableAnalogInput(int pin, boolean enablePullup);
void enableDigitalInput(int pin, boolean enablePullup);
void enableDigitalOutput(int pin);
int getPitchBendFromLinearPot();
int quantizeSlide(int val);
int getPitchBend();
unsigned char getRawOvertoneSwitchValue();
int getOvertoneFromOvertoneSwitches();
int getMIDINote();
int getVolumeFromBreathSensor();
int getVolume();
int getXValue();
int getYValue();
void sendNoteOn(int note, int vel, byte chan, boolean debug);
void sendNoteOff(int note, int vel, byte chan, boolean debug);
void sendPitchBend(int pitchBend, boolean debug);
void sendBreathController(int volume, byte chan, boolean debug);
void sendXYControllers(int x, int y, byte chan, boolean debug);
void allNotesOff();
int sendMetaCommand(byte chan, unsigned char value);
void loop();
MidiClass Midi;

// If DEBUG == true, then the sketch will print to the serial port what
// it would send on the MIDI bus.
const boolean DEBUG = false;
//const boolean DEBUG = true;

const int BREATH_PIN = 0; // Breath sensor on analog pin 0
const int SLIDE_LPOT_PIN = 1; // Slide sensor on analog pin 1
const int X_SENSOR_PIN = 2; // X sensor hooked to analog pin 2
const int Y_SENSOR_PIN = 3; // X sensor hooked to analog pin 3

const int OT_SW_0_PIN = 3; // Overtone switch 0
const int OT_SW_1_PIN = 4; // Overtone switch 1
const int OT_SW_2_PIN = 5; // Overtone switch 2
const int OT_SW_3_PIN = 6; // Overtone switch 3
const int META_SW_PIN = 2; // Meta switch

const int SLIDE_LED_PIN = 13; //Pin that drives LED that shows slide quantization

const int PANIC_PIN = 7; // MIDI all notes off momentary switch on digital I/O 4

// The overtone series this instrument will produce. In this iteration
// of the instrument, I'm trying a sequence of "overtones" that
// ascend a perfect fifth, then a perfect fourth, then a fifth
// and fourth again, and so on
const int FUNDAMENTAL = 48;  // MIDI note value of our fundamental (C)
const int OT_1 = 55; // First overtone (G)
const int OT_2 = 60; // Second overtone (C)
const int OT_3 = 64; // Third overtone (E)
const int OT_4 = 67; // Fourth overtone (G)
const int OT_5 = 70; // Fifth overtone (B flat)
const int OT_6 = 72; // Sixth overtone (C)
const int OT_7 = 74; // Seventh overtone (D)
const int OT_NONE = -1; // No overtone key pressed (not possible with ribbon)

// All overtones for this instrument
const int overtones[8] =    {FUNDAMENTAL, OT_1, OT_2, OT_3, OT_4, OT_5, OT_6, OT_7};
// Switch values for given overtones. 0xff means that overtone can't be selected.
const int overtone_sw_values[8] = {0x00, 0x01, 0x03, 0x07, 0x0f, 0x0e, 0x0c, 0x08};

const int MIDI_VOLUME_CC = 7; // The controller number for MIDI volume data
const int MIDI_BREATH_CC = 2; // The controller number for MIDI breath controller data
const int X_CC = 16; // The controller number for the X value
const int Y_CC = 17; // The controller number for the Y value

long ccSendTime = 0;  // Last time we sent continuous data (volume, pb);
const int MIN_CC_INTERVAL = 10; // Send CC data no more often than this (in milliseconds);
const int PB_SEND_THRESHOLD = 10; // Only send pitch bend if it's this much different than the current value
const int VOLUME_SEND_THRESHOLD = 1; // Only send volume change if it's this much differnt that the current value
const int NOTE_ON_VOLUME_THRESHOLD = 60; // Raw sensor value required to turn on a note
const int VOLUME_MAX_VALUE = 500; // Maximum value from the breath sensor.

// If a value larger than this is read from a SoftPot, treat it as if the player is not touching it.
// Note: for some reason, the two SoftPots interact, e.g. just actuating the slide pot gives me
// no-touch values all above 1000, but when also touching the overtone pot, the values can go
// as low as 999. I suspect I may be taxing the 5v supply line.
const int LPOT_NO_TOUCH_VALUE = 1010;  
const int LPOT_SLIDE_POS_1 = 144; // Value at 1st position
const int LPOT_SLIDE_POS_7 = 350;  // Value at 7th position
const int MAX_PITCH_BEND_DOWN = 0; // Pitch bend value for 7th position
const int PITCH_BEND_NEUTRAL = 16383 / 2; // Neutral pitch bend value

int currentNote = -1; // The MIDI note currently sounding
int currentPitchBend = PITCH_BEND_NEUTRAL; // The current pitch bend
int currentVolume = 0; // The current 
int currentXValue = 0; // The current value of the X controller
int currentYValue = 0; // The current value of the Y controller
int slide_quant_mode = 0; // The current slide quantization mode. 0 = disabled, 1 = enabled
boolean metaMode = false; // If true, we are handing a meta keypress
unsigned char metaValue = 0; // Value to send when meta key released.

void setup() {
  enableDigitalInput(OT_SW_0_PIN, true);
  enableDigitalInput(OT_SW_1_PIN, true);
  enableDigitalInput(OT_SW_2_PIN, true);
  enableDigitalInput(OT_SW_3_PIN, true);
  enableDigitalInput(META_SW_PIN, true);
  enableDigitalInput(PANIC_PIN, true);
  enableDigitalOutput(SLIDE_LED_PIN);
  enableAnalogInput(BREATH_PIN, false);
  enableAnalogInput(SLIDE_LPOT_PIN, true);
  enableAnalogInput(X_SENSOR_PIN, true);
  enableAnalogInput(Y_SENSOR_PIN, true);
  
  if (DEBUG) {
    Serial.begin(9600);
  } else {
    MidiUart.init();  // Initialize MIDI
  }
}

/**
 * Enable a pin for analog input, and set its internal pullup.
 */
void enableAnalogInput(int pin, boolean enablePullup) {
  pinMode(pin, INPUT);
  digitalWrite(pin + 14, enablePullup ? HIGH : LOW);
}

/**
 * Enable a pin for digital input, and set its internal pullup.
 */
void enableDigitalInput(int pin, boolean enablePullup) {
  pinMode(pin, INPUT);
  digitalWrite(pin, enablePullup ? HIGH : LOW);
}

/**
 * Enable a pin for digital output.
 */
void enableDigitalOutput(int pin) {
  pinMode(pin, OUTPUT);
}


/**
 * Read the slide pot and return a pitch bend value. The values
 * returned are all bends down from the base pitch being played,
 * and are in the range 8192 (no bend) to 0 (maximum bend down).
 * This means that the synth patch needs to be adjusted to provide
 * a maximum pitch bend of seven semitones, if you want it to
 * behave like a trombone.
 *
 * Return -1 if the player is not touching the sensor.
 */
 int getPitchBendFromLinearPot() {
  
  // Get the raw value from the linear pot
  int slideVal = analogRead(SLIDE_LPOT_PIN);
  
  if (slideVal > LPOT_NO_TOUCH_VALUE) {
    return -1;
  } else {
    // Coerce out-of-range values (e.g. beyond the slide stops)
    int constrainedVal = slideVal;
    constrainedVal = constrainedVal > LPOT_SLIDE_POS_7 ? LPOT_SLIDE_POS_7 : constrainedVal;
    constrainedVal = constrainedVal < LPOT_SLIDE_POS_1 ? LPOT_SLIDE_POS_1 : constrainedVal;
    
   int  pbVal = map(constrainedVal, LPOT_SLIDE_POS_1, LPOT_SLIDE_POS_7, PITCH_BEND_NEUTRAL, MAX_PITCH_BEND_DOWN);
   if (pbVal < 0) pbVal = 0;
    
    // Quantize slide position, if requested
    if (slide_quant_mode) {
      pbVal = quantizeSlide(pbVal);
    }
    
    return pbVal;
  }
}

/*
 * Quantize the slide so that there are only seven possible values.
 * Each "position" is one position wide, centered on the actual
 * slide position value, except for 1st and 7th positions, which
 * are only half a position wide:
 *
 * 1    2    3    4    5    6    7
 * ^^^                           
 *    ^^^^^                        
 *         ^^^^^                  
 *              ^^^^^            
 *                   ^^^^^       
 *                        ^^^^^  
 *                             ^^^
 */
int quantizeSlide(int val) {
  if (val >= 0 && val <= 683) return 0;
  if (val >= 684 && val <= 2048) return 1365;
  if (val >= 2049 && val <= 3413) return 2731;
  if (val >= 3414 && val <= 4779) return 4096;
  if (val >= 4780 && val <= 6144) return 5461;
  if (val >= 6145 && val <= 7509) return 6827;
  if (val >= 7510 && val <= 8192) return 8191;
  return 0;
}

/*
 * Read the slide and return a pitch bend value in the range from
 * 8192 (1st position) to 0 (7th position)
 */
int getPitchBend() {
  return getPitchBendFromLinearPot();
}

unsigned char getRawOvertoneSwitchValue() {
  unsigned char val = !digitalRead(OT_SW_0_PIN);
  val = val << 1 | !digitalRead(OT_SW_1_PIN);
  val = val << 1 | !digitalRead(OT_SW_2_PIN);
  val = val << 1 | !digitalRead(OT_SW_3_PIN);
  return val;
}


/**
 * Read the overtone switches and return the appropriate overtone.
 * If an invalid key combination is found, return -1. Note that
 * we invert the values from digitalRead, since these switches 
 * pull to ground, so switch enabled = digital 0.
 */
int getOvertoneFromOvertoneSwitches() {
  unsigned char val = getRawOvertoneSwitchValue();
  // now select the appropriate overtone
  for (int i = 0; i < sizeof(overtone_sw_values); i++)  {
    if (val == overtone_sw_values[i]) {
      return i;
    }
  }
  return -1;
}

int getMIDINote() {
  int ot = getOvertoneFromOvertoneSwitches();
  if (-1 == ot) {
    return currentNote;
  } else {
    return overtones[ot];
  }
}

/**
 * Read the breath sensor and map it to a volume level. For now,
 * this maps to the range 0 - 127 so we can generate MIDI
 * continuous controller information.
 */
int getVolumeFromBreathSensor() {
  int volRawVal = analogRead(BREATH_PIN);
  if (volRawVal < NOTE_ON_VOLUME_THRESHOLD) {
    return 0;
  } else {
    return map(constrain(volRawVal, NOTE_ON_VOLUME_THRESHOLD, VOLUME_MAX_VALUE), NOTE_ON_VOLUME_THRESHOLD, VOLUME_MAX_VALUE, 0, 127);
  }
}

int getVolume() {
  return getVolumeFromBreathSensor();
}

int getXValue() {
  return analogRead(X_SENSOR_PIN);
}

int getYValue() {
  return analogRead(Y_SENSOR_PIN);
}

void sendNoteOn(int note, int vel, byte chan, boolean debug) {
  if (debug) {
    Serial.print("ON ");
    Serial.println(note);
  } else {
    MidiUart.sendNoteOn(chan, note, vel);
  }
}

void sendNoteOff(int note, int vel, byte chan, boolean debug) {
  if (debug) {
    Serial.print("OFF ");
    Serial.println(note);
  } else {
    MidiUart.sendNoteOff(chan, note, vel);
  }
}

void sendPitchBend(int pitchBend, boolean debug) {
  if (-1 != pitchBend) {
    if (abs(currentPitchBend - pitchBend) > PB_SEND_THRESHOLD) {
      currentPitchBend = pitchBend;
      if (debug) {
        Serial.print("BEND ");
        Serial.println(pitchBend);
      } else {
        MidiUart.sendPitchBend(pitchBend);
      }
    }
  }
}


void sendBreathController(int volume, byte chan, boolean debug) {
  if (abs(currentVolume - volume) > VOLUME_SEND_THRESHOLD) {
    currentVolume = volume;
    if (debug) {
      Serial.print("BC ");
      Serial.println(volume);
    } else {
      MidiUart.sendCC(chan, MIDI_BREATH_CC, volume );
    }
  }
}

void sendXYControllers(int x, int y, byte chan, boolean debug) {
  int mappedXValue = map(x, 0, 1024, 0, 127);
  int mappedYValue = map(y, 0, 1024, 0, 127);
  if (abs(currentXValue - mappedXValue) > VOLUME_SEND_THRESHOLD) {
    currentXValue = mappedXValue;
    if (debug) {
      Serial.print("X ");
      Serial.print(mappedXValue);
    } else {
      MidiUart.sendCC(chan, X_CC, mappedXValue);
    }
  }
  if (abs(currentYValue - mappedYValue) > VOLUME_SEND_THRESHOLD) {
    currentYValue = mappedYValue;
    if (debug) {
      Serial.print("Y ");
      Serial.print(mappedYValue);
    } else {
      MidiUart.sendCC(chan, Y_CC, mappedYValue);
    }
  }
}

void allNotesOff() {
  for (int i = 0; i < 128; i++) {
    sendNoteOff(i, 0, 1, DEBUG);
  }
}

/*
 Send whatever meta mode command.
 */
int sendMetaCommand(byte chan, unsigned char value) {
  if (DEBUG) {
      Serial.print("META ");
      Serial.println(metaValue, HEX);
    } else {
      //MidiUart.sendCC(chan, 20 + value, 1);
      MidiUart.sendNoteOn(chan, value, 127);
    }
}

void loop() {
  
  if (digitalRead(PANIC_PIN) == 0) {
    allNotesOff();
  }
  
  if (digitalRead(META_SW_PIN) == 0) {
    metaMode = true;
    metaValue = getRawOvertoneSwitchValue();
  } else if (metaMode == true) {
    // Meta switch was just released - send meta command
    metaMode = false;
    sendMetaCommand(1, metaValue);
  }
  
  int pb = getPitchBend();
  int note = getMIDINote();
  int volume = getVolume();
  int x = getXValue();
  int y = getYValue();
  
  if ((-1 != currentNote) && (0 == volume)) {
    // Breath stopped, so send a note off
    sendNoteOff(currentNote, 0, 0, DEBUG);
    currentNote = -1;
  } else if ((-1 == currentNote) && (0 != volume) && (-1 != note)) {
    // No note was playing, and we have breath and a valid overtone, so send a note on.
    // Be sure to send any updated pitch bend first, though, in case the slide moved.
    // And also send updated breath controller info so volume is correct.
    sendBreathController(volume, 0, DEBUG);
    sendPitchBend(pb, DEBUG);
    sendXYControllers(x, y, 0, DEBUG);
    sendNoteOn(note, 127, 0, DEBUG);
    currentNote = note;
  } else if ((-1 != currentNote) && (note != currentNote)) {
    // A note was playing, but the player has moved to a different note.
    // Turn off the old note and turn on the new one.
    sendNoteOff(currentNote, 0, 0, DEBUG);
    sendPitchBend(pb, DEBUG);
    sendBreathController(volume, 0, DEBUG);
    sendXYControllers(x, y, 0, DEBUG);
    sendNoteOn(note, 127, 0, DEBUG);
    currentNote = note;
  } else if (-1 != currentNote) {
    // Send updated breath controller and pitch bend values.
    if (millis() > ccSendTime + MIN_CC_INTERVAL) {
      sendPitchBend(pb, DEBUG);
      sendBreathController(volume, 0, DEBUG);
      sendXYControllers(x, y, 0, DEBUG);
      ccSendTime = millis();
    }
  }
  delay(50);
}


int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}


#include <MPR121.h>
#include <Arduino.h>
#include "Constants.h"

#include <menu.h>
#include <menuIO/u8g2Out.h>
#include <menuIO/chainStream.h>
#include <menuIO/rotaryEventIn.h>

#include <qdec.h>
#include <AceButton.h>

#include <wire.h>

// Encoder
const int ROTARY_PIN_A = 22;  // the first pin connected to the rotary encoder
const int ROTARY_PIN_B = 23;  // the second pin connected to the rotary encoder
const int ROTARY_PIN_BUT = 21;

using namespace ::ace_button;
using namespace ::SimpleHacks;
QDecoder qdec(ROTARY_PIN_A, ROTARY_PIN_B, true);  // rotary part
AceButton button(ROTARY_PIN_BUT);                 // button part


enum states { NORMAL,
              DEMO };
states state = NORMAL;

enum modes { NONE,
             MIDI,
             OSC,
             ARTNET };
modes mode = MIDI;

//Midi Settings
enum midiModes { NOTE,
                 CC };
midiModes midiMode = CC;

int midi_channel_start = 1;
int midi_cc_start = 20;

//Demo Settings
enum demoStates { SINE,
                  MIMIC,
                  DELAY };
demoStates demoState = SINE;
demoStates demoMenuState = SINE;
const byte demo_buffer_size = 120;
byte demo_buffer[demo_buffer_size];
double demo_time = millis();
//Touch Sensor
MPR121 mpr121;


//Display
#define SDA 5
#define SCL 4
#define fontName u8g2_font_7x13_mf
#define fontX 7
#define fontY 16
#define offsetX 0
#define offsetY 3
#define U8_Width 128
#define U8_Height 64
#define USE_HWI2C
#define fontMarginX 2
#define fontMarginY 2
U8G2_SSD1306_128X64_VCOMH0_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, SCL, SDA);  //allow contrast change

const colorDef<uint8_t> colors[6] MEMMODE = {
  { { 0, 0 }, { 0, 1, 1 } },  //bgColor
  { { 1, 1 }, { 1, 0, 0 } },  //fgColor
  { { 1, 1 }, { 1, 0, 0 } },  //valColor
  { { 1, 1 }, { 1, 0, 0 } },  //unitColor
  { { 0, 1 }, { 0, 0, 1 } },  //cursorColor
  { { 1, 1 }, { 1, 0, 0 } },  //titleColor
};

//Menu
#define MAX_DEPTH 4
int demoInd = 0;
// int demoMenuInd = 1;

bool b = false;
using namespace Menu;
//  TOGGLE(b, toggle, "TOGGLE B", doNothing, noEvent, wrapStyle, VALUE("YES", true, doNothing, noEvent), VALUE("NO", false, doNothing, noEvent)),

result runDemo() {
  Serial.println("Run demo");
  demoState = demoMenuState;
  state = DEMO;
  return proceed;
}

result exitDemo() {
  state = NORMAL;
  resetFaders();
  return quit;
}

//Demo Menu
SELECT(demoMenuState, select_demo, "Demo", doNothing, noEvent, noStyle,
       VALUE("SINE", SINE, doNothing, noEvent),
       VALUE("MIMIC 1", MIMIC, doNothing, noEvent),
       VALUE("DELAY 1", DELAY, doNothing, noEvent));
MENU(demo_menu, "Demos", doNothing, noEvent, wrapStyle,
     SUBMENU(select_demo),
     OP("Run Demo", runDemo, enterEvent),
     OP("<Back", exitDemo, enterEvent));
//Settings Menu
SELECT(midiMode, select_midi_mode, "MIDI Type", doNothing, noEvent, noStyle,
       VALUE("Note", NOTE, doNothing, noEvent),
       VALUE("CC", CC, doNothing, noEvent));
MENU(midi_menu, "MIDI Settings", Menu::doNothing, Menu::noEvent, Menu::wrapStyle,
     SUBMENU(select_midi_mode),
     FIELD(midi_channel_start, "Channel Start", "", 1, 11, 1, 1, doNothing, noEvent, noStyle),
     FIELD(midi_cc_start, "CC Start", "", 0, 123, 1, 1, doNothing, noEvent, noStyle),
     EXIT("<Back"));
SELECT(mode, select_mode, "Mode", doNothing, noEvent, noStyle,
       VALUE("MIDI", MIDI, doNothing, noEvent),
       VALUE("ARTNET", ARTNET, doNothing, noEvent),
       VALUE("OSC", OSC, doNothing, noEvent),
       VALUE("NONE", NONE, doNothing, noEvent));
MENU(settings_menu, "Settings", doNothing, noEvent, wrapStyle,
     SUBMENU(select_mode),
     SUBMENU(midi_menu),
     EXIT("<Back"));

//Main Menu
MENU(mainMenu, "Main Menu", Menu::doNothing, Menu::noEvent, Menu::wrapStyle,
     SUBMENU(demo_menu),
     SUBMENU(settings_menu),
     EXIT("<Back"));


RotaryEventIn reIn(
  RotaryEventIn::EventType::BUTTON_CLICKED |         // select
  RotaryEventIn::EventType::BUTTON_DOUBLE_CLICKED |  // back
  RotaryEventIn::EventType::BUTTON_LONG_PRESSED |    // also back
  RotaryEventIn::EventType::ROTARY_CCW |             // up
  RotaryEventIn::EventType::ROTARY_CW                // down
);                                                   // register capabilities, see AndroidMenu MenuIO/RotaryEventIn.h file
MENU_INPUTS(in, &reIn);

MENU_OUTPUTS(out, MAX_DEPTH, U8G2_OUT(u8g2, colors, fontX, fontY, offsetX, offsetY, { 0, 0, U8_Width / fontX, U8_Height / fontY }), NONE);
NAVROOT(nav, mainMenu, MAX_DEPTH, in, out);

int fader_tolerance = 3;
int numFaders = 5;

int pwmspeed = 200;

byte targetPositions[] = { 0, 0, 0, 0, 0 };



void IsrForQDEC(void) {
  QDECODER_EVENT event = qdec.update();
  if (event & QDECODER_EVENT_CW) {
    reIn.registerEvent(RotaryEventIn::EventType::ROTARY_CW);
  } else if (event & QDECODER_EVENT_CCW) {
    reIn.registerEvent(RotaryEventIn::EventType::ROTARY_CCW);
  }
}

void handleButtonEvent(AceButton* /* button */, uint8_t eventType, uint8_t buttonState) {
  switch (eventType) {
    case AceButton::kEventClicked:
      reIn.registerEvent(RotaryEventIn::EventType::BUTTON_CLICKED);
      Serial.println("Button Clicked");
      break;
    case AceButton::kEventDoubleClicked:
      reIn.registerEvent(RotaryEventIn::EventType::BUTTON_DOUBLE_CLICKED);
      Serial.println("Button Double Clicked");
      break;
    case AceButton::kEventLongPressed:
      reIn.registerEvent(RotaryEventIn::EventType::BUTTON_LONG_PRESSED);
      Serial.println("Button Long Pressed");
      break;
  }
}

void setup() {
  //display
  Wire.begin();
  u8g2.begin();
  u8g2.setFont(fontName);
  do {
    u8g2.drawStr(0, fontY, "MotorFader Control");
  } while (u8g2.nextPage());

  //Start Serial
  Serial.begin(115200);

  for (int i = 0; i < demo_buffer_size; i++) {
    demo_buffer[i] = 0;
  }

  //encoder
  qdec.begin();
  attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_A), IsrForQDEC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_B), IsrForQDEC, CHANGE);

  //button
  pinMode(ROTARY_PIN_BUT, INPUT);
  ButtonConfig* buttonConfig = button.getButtonConfig();
  buttonConfig->setEventHandler(handleButtonEvent);
  buttonConfig->setFeature(ButtonConfig::kFeatureClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressClickBeforeDoubleClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterDoubleClick);


  usbMIDI.setHandleNoteOn(myNoteOn);
  usbMIDI.setHandleNoteOff(myNoteOff);
  usbMIDI.setHandleControlChange(myControlChange);


  //Setup Capacitive Touch Sensor
  mpr121.setupSingleDevice(*constants::wire_ptr,
                           constants::device_address,
                           constants::fast_mode);

  mpr121.setAllChannelsThresholds(constants::touch_threshold,
                                  constants::release_threshold);
  mpr121.setDebounce(constants::device_address,
                     constants::touch_debounce,
                     constants::release_debounce);
  mpr121.setBaselineTracking(constants::device_address,
                             constants::baseline_tracking);
  mpr121.setChargeDischargeCurrent(constants::device_address,
                                   constants::charge_discharge_current);
  mpr121.setChargeDischargeTime(constants::device_address,
                                constants::charge_discharge_time);
  mpr121.setFirstFilterIterations(constants::device_address,
                                  constants::first_filter_iterations);
  mpr121.setSecondFilterIterations(constants::device_address,
                                   constants::second_filter_iterations);
  mpr121.setSamplePeriod(constants::device_address,
                         constants::sample_period);

  mpr121.startChannels(constants::physical_channel_count,
                       constants::proximity_mode);


  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  button.check();
  nav.doInput();
  if (nav.changed(0)) {  //only draw if menu changed for gfx device
    u8g2.firstPage();
    do nav.doOutput();
    while (u8g2.nextPage());
  }

  switch (state) {
    case NORMAL:
      usbMIDI.read();
      updateFaders();
      break;
    case DEMO:
      updateDemo();
      break;
    default:
      break;
  }
}

void updateFaders() {
  setPos(0, targetPositions[0]);
  setPos(1, targetPositions[1]);
  setPos(2, targetPositions[2]);
  setPos(3, targetPositions[3]);
  setPos(4, targetPositions[4]);
}

void setPos(int fInd, byte position) {
  if (fInd > numFaders - 1 || fInd < 0) {
    return;
  }
  if (position > 127 || position < 0) {
    return;
  }


  int m_A = 0;
  int m_B = 0;
  int read = 0;
  switch (fInd) {
    case 0:
      m_A = 3;
      m_B = 2;
      read = analogRead(A6);
      break;
    case 1:
      m_A = 5;
      m_B = 4;
      read = analogRead(A3);
      break;
    case 2:
      m_A = 7;
      m_B = 6;
      read = analogRead(A2);
      break;
    case 3:
      m_A = 9;
      m_B = 8;
      read = analogRead(A1);
      break;
    case 4:
      m_A = 10;
      m_B = 11;
      read = analogRead(A0);
      break;
    default:
      return;
  }
  read = map(read, 0, 1000, 127, 0);

  // if (fInd == 0) {
  //   Serial.print("Read Raw: ");
  //   Serial.print(readRaw);
  //   Serial.print("   Read: ");
  //   Serial.print(read);
  //   Serial.print("   Position: ");
  //   Serial.print(position);
  //   Serial.print("   Touched: ");
  //   Serial.println(mpr121.channelTouched(fInd));
  // }
  if (mpr121.channelTouched(fInd)) {
    analogWrite(m_A, 0);
    analogWrite(m_B, 0);

    if (abs(position - read) > fader_tolerance) {
      targetPositions[fInd] = read;

      sendMessage(read, fInd);
    }

  } else {

    if (abs(read - position) < fader_tolerance) {
      analogWrite(m_A, 0);
      analogWrite(m_B, 0);
    } else if (read - position < 0) {
      analogWrite(m_A, 0);
      analogWrite(m_B, pwmspeed);
    } else {
      analogWrite(m_A, pwmspeed);
      analogWrite(m_B, 0);
    }
  }
}

void updateDemo() {
  switch (demoState) {
    case SINE:
      for (int i = 0; i < numFaders; i++) {
        setPos(i, ((sin(millis() / 250. + i) + 1) * 63.5));
      }
      break;
    case MIMIC:
      for (int i = 0; i < numFaders; i++) {
        setPos(i, targetPositions[0]);
      }
      break;
    case DELAY:
      if (millis() - demo_time > 5) {
        demo_time = millis();
        for (int i = demo_buffer_size - 1; i > 0; i--) {
          demo_buffer[i] = demo_buffer[i - 1];
        }
        demo_buffer[0] = targetPositions[0];
      }
      // printArr();

      for (int i = 1; i < numFaders; i++) {
        setPos(i, demo_buffer[(i * 30) - 1]);
      }
      setPos(0, targetPositions[0]);

      break;
    default: break;
  }
}

void myNoteOn(byte channel, byte note, byte velocity) {
  // When using MIDIx4 or MIDIx16, usbMIDI.getCable() can be used
  // to read which of the virtual MIDI cables received this message.
  // Serial.print("Note On, ch=");
  // Serial.print(channel, DEC);
  // Serial.print(", note=");
  // Serial.print(note, DEC);
  // Serial.print(", velocity=");
  // Serial.println(velocity, DEC);

  if (mode == MIDI) {
    if (midiMode == NOTE) {
      byte fader_ind = channel - midi_channel_start;
      if (fader_ind >= 0 && fader_ind < 5) {
        targetPositions[fader_ind] = note;
      }
    }
  }
}

//Midi
void myNoteOff(byte channel, byte note, byte velocity) {
  // Serial.print("Note Off, ch=");
  // Serial.print(channel, DEC);
  // Serial.print(", note=");
  // Serial.print(note, DEC);
  // Serial.print(", velocity=");
  // Serial.println(velocity, DEC);
}

void myControlChange(byte channel, byte control, byte value) {
  // Serial.print("Control Change, ch=");
  // Serial.print(channel, DEC);
  // Serial.print(", control=");
  // Serial.print(control, DEC);
  // Serial.print(", value=");
  // Serial.println(value, DEC);
  if (mode == MIDI) {
    if (midiMode == CC) {
      if (channel == midi_channel_start) {
        byte fader_ind = control - midi_cc_start;
        if (fader_ind >= 0 && fader_ind < 5) {
          targetPositions[fader_ind] = value;
        }
      }
    }
  }
}

void sendMessage(int val, int fInd) {
  Serial.print(val);
  Serial.print("  ");
  Serial.println(fInd);
  switch (mode) {
    case NONE:
      break;
    case MIDI:
      if (midiMode == NOTE) {
        usbMIDI.sendNoteOn(val, 127, midi_channel_start + fInd);
        usbMIDI.sendNoteOff(val, 0, midi_channel_start + fInd);
      } else if (midiMode == CC) {
        usbMIDI.sendControlChange(midi_cc_start + fInd, val, midi_channel_start);
      }
      break;
    case ARTNET:
      break;
    case OSC:
      break;
    default:
      break;
  }
}

void resetFaders() {
  for (int i = 0; i < numFaders; i++) {
    targetPositions[i] = 0;
  }
}
void printArr() {
  for (int i = 0; i < demo_buffer_size; i++) {
    Serial.print(demo_buffer[i]);
    Serial.print(" ");
  }
  Serial.println("");
}

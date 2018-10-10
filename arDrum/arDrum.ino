/*
 * Copyright (c) 2015 Evan Kale
 * Email: EvanKale91@gmail.com
 * Website: www.ISeeDeadPixel.com
 *          www.evankale.blogspot.ca
 *
 * This file is part of ArduinoMidiDrums.
 *
 * ArduinoMidiDrums is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * 
 */

//Piezo defines
#define NUM_PIEZOS 8
#define START_SLOT 0        //first analog slot of piezos, they must be contiguous

#define BASS_THRESHOLD  50  //anything < TRIGGER_THRESHOLD is treated as 0
#define SNARE_THRESHOLD 30
#define HIHAT_THRESHOLD 30
#define HITOM_THRESHOLD 30
#define LOTOM_THRESHOLD 30
#define FLTOM_THRESHOLD 30
#define CRASH_THRESHOLD 30
#define RIDE_THRESHOLD  30 

#define MAX_PIEZO_READ 1023 // max value from the A/D converter

//MIDI note defines for each trigger
#define BASS_NOTE  36
#define SNARE_NOTE 38
#define HIHAT_NOTE 42
#define HITOM_NOTE 48
#define LOTOM_NOTE 45
#define FLTOM_NOTE 41
#define CRASH_NOTE 49
#define RIDE_NOTE  51 

//MIDI defines
#define MIDI_CHANNEL 10
#define NOTE_ON_CMD 0x90
#define NOTE_OFF_CMD 0x80
#define MIN_MIDI_VELOCITY 10
#define MAX_MIDI_VELOCITY 127

//MIDI baud rate
#define SERIAL_RATE 31250

//Program defines
//ALL TIME MEASURED IN MILLISECONDS
//#define SIGNAL_BUFFER_SIZE 100
#define SIGNAL_BUFFER_SIZE 70
#define PEAK_BUFFER_SIZE 30
#define MAX_TIME_BETWEEN_PEAKS 20
#define MIN_TIME_BETWEEN_NOTES 50

//map that holds the respective note to each piezo
unsigned char noteMap[NUM_PIEZOS];

//map that holds the respective threshold to each piezo
unsigned char thresholdMap[NUM_PIEZOS];

//Ring buffers to store analog signal and peaks
unsigned char currentSignalIndex[NUM_PIEZOS];
unsigned char currentPeakIndex[NUM_PIEZOS];
unsigned short signalBuffer[NUM_PIEZOS][SIGNAL_BUFFER_SIZE];
unsigned short peakBuffer[NUM_PIEZOS][PEAK_BUFFER_SIZE];

boolean noteReady[NUM_PIEZOS];
unsigned short noteReadyVelocity[NUM_PIEZOS];
boolean isLastPeakZeroed[NUM_PIEZOS];

unsigned long lastPeakTime[NUM_PIEZOS];
unsigned long lastNoteTime[NUM_PIEZOS];

void setup()
{
  Serial.begin(SERIAL_RATE);
  
  //initialize globals
  for(char i=0; i<NUM_PIEZOS; ++i)
  {
    currentSignalIndex[i] = 0;
    currentPeakIndex[i] = 0;
    memset(signalBuffer[i],0,sizeof(signalBuffer[i]));
    memset(peakBuffer[i],0,sizeof(peakBuffer[i]));
    noteReady[i] = false;
    noteReadyVelocity[i] = 0;
    isLastPeakZeroed[i] = true;
    lastPeakTime[i] = 0;
    lastNoteTime[i] = 0;    
  }
  
  thresholdMap[0] = BASS_THRESHOLD;
  thresholdMap[1] = SNARE_THRESHOLD;
  thresholdMap[2] = HIHAT_THRESHOLD;
  thresholdMap[3] = HITOM_THRESHOLD;
  thresholdMap[4] = LOTOM_THRESHOLD;
  thresholdMap[5] = FLTOM_THRESHOLD;
  thresholdMap[6] = CRASH_THRESHOLD;
  thresholdMap[7] = RIDE_THRESHOLD;
  
  noteMap[0] = BASS_NOTE;
  noteMap[1] = SNARE_NOTE;
  noteMap[2] = HIHAT_NOTE;
  noteMap[3] = HITOM_NOTE;
  noteMap[4] = LOTOM_NOTE;
  noteMap[5] = FLTOM_NOTE;  
  noteMap[6] = CRASH_NOTE;
  noteMap[7] = RIDE_NOTE; 
   
}

void loop()
{
  unsigned long currentTime = millis();
  
  for(char i=0; i<NUM_PIEZOS; ++i)
  {
    //get a new signal from analog read
    unsigned short newSignal = analogRead(START_SLOT+i);
    signalBuffer[i][currentSignalIndex[i]] = newSignal;
    
    //if new signal is 0
    if(newSignal < thresholdMap[i])
    {
      if(!isLastPeakZeroed[i] && (currentTime - lastPeakTime[i]) > MAX_TIME_BETWEEN_PEAKS)
      {
        recordNewPeak(i,0);
      }
      else
      {
        //get previous signal
        char prevSignalIndex = currentSignalIndex[i]-1;
        if(prevSignalIndex < 0) prevSignalIndex = SIGNAL_BUFFER_SIZE-1;        
        unsigned short prevSignal = signalBuffer[i][prevSignalIndex];
        
        unsigned short newPeak = 0;
        
        //find the wave peak if previous signal was not 0 by going
        //through previous signal values until another 0 is reached
        while(prevSignal >= thresholdMap[i])
        {
          if(signalBuffer[i][prevSignalIndex] > newPeak)
          {
            newPeak = signalBuffer[i][prevSignalIndex];        
          }
          
          //decrement previous signal index, and get previous signal
          --prevSignalIndex;
          if(prevSignalIndex < 0) prevSignalIndex = SIGNAL_BUFFER_SIZE-1;
          prevSignal = signalBuffer[i][prevSignalIndex];
        }
        
        if(newPeak > 0)
        {
          recordNewPeak(i, newPeak);
        }
      }
  
    }
        
    ++currentSignalIndex[i];
    if(currentSignalIndex[i] == SIGNAL_BUFFER_SIZE) currentSignalIndex[i] = 0;
  }
}

void recordNewPeak(short slot, short newPeak)
{
  isLastPeakZeroed[slot] = (newPeak == 0);
  
  unsigned long currentTime = millis();
  lastPeakTime[slot] = currentTime;
  
  //new peak recorded (newPeak)
  peakBuffer[slot][currentPeakIndex[slot]] = newPeak;
  
  //1 of 3 cases can happen:
  // 1) note ready - if new peak >= previous peak
  // 2) note fire - if new peak < previous peak and previous peak was a note ready
  // 3) no note - if new peak < previous peak and previous peak was NOT note ready
  
  //get previous peak
  short prevPeakIndex = currentPeakIndex[slot]-1;
  if(prevPeakIndex < 0) prevPeakIndex = PEAK_BUFFER_SIZE-1;        
  unsigned short prevPeak = peakBuffer[slot][prevPeakIndex];
   
  if(newPeak > prevPeak && (currentTime - lastNoteTime[slot])>MIN_TIME_BETWEEN_NOTES)
  {
    noteReady[slot] = true;
    if(newPeak > noteReadyVelocity[slot])
      noteReadyVelocity[slot] = newPeak;
  }
  else if(newPeak < prevPeak && noteReady[slot])
  {
    noteFire(noteMap[slot], map(noteReadyVelocity[slot],thresholdMap[slot],MAX_PIEZO_READ,MIN_MIDI_VELOCITY,MAX_MIDI_VELOCITY));
    noteReady[slot] = false;
    noteReadyVelocity[slot] = 0;
    lastNoteTime[slot] = currentTime;
  }
  
  currentPeakIndex[slot]++;
  if(currentPeakIndex[slot] == PEAK_BUFFER_SIZE) currentPeakIndex[slot] = 0;  
}

void noteFire(unsigned short note, unsigned short velocity)
{
  if(velocity > MAX_MIDI_VELOCITY)
    velocity = MAX_MIDI_VELOCITY;
  
  midiNoteOn(note, velocity);
  midiNoteOff(note, velocity);
}

void midiNoteOn(byte note, byte midiVelocity)
{
  Serial.write(NOTE_ON_CMD | (MIDI_CHANNEL - 1));
  Serial.write(note);
  Serial.write(midiVelocity);
}

void midiNoteOff(byte note, byte midiVelocity)
{
  Serial.write(NOTE_OFF_CMD | (MIDI_CHANNEL - 1));
  Serial.write(note);
  Serial.write(midiVelocity);
}

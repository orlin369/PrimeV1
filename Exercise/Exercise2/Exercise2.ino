
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

#define NOTE_PAUSE 0

#define PIN_BUZZER 8
#define PIN_LED 13
#define PIN_BUTTON 12

#define SONG_1 1
#define SONG_2 2

// Mario main theme melody.
int SongOneMelody_g[] = {
  NOTE_E7, NOTE_E7, 0, NOTE_E7,
  NOTE_PAUSE, NOTE_C7, NOTE_E7, NOTE_PAUSE,
  NOTE_G7, NOTE_PAUSE, NOTE_PAUSE,  NOTE_PAUSE,
  NOTE_G6, NOTE_PAUSE, NOTE_PAUSE, NOTE_PAUSE,

  NOTE_C7, NOTE_PAUSE, NOTE_PAUSE, NOTE_G6,
  NOTE_PAUSE, NOTE_PAUSE, NOTE_E6, NOTE_PAUSE,
  NOTE_PAUSE, NOTE_A6, NOTE_PAUSE, NOTE_B6,
  NOTE_PAUSE, NOTE_AS6, NOTE_A6, NOTE_PAUSE,

  NOTE_G6, NOTE_E7, NOTE_G7,
  NOTE_A7, NOTE_PAUSE, NOTE_F7, NOTE_G7,
  NOTE_PAUSE, NOTE_E7, NOTE_PAUSE, NOTE_C7,
  NOTE_D7, NOTE_B6, NOTE_PAUSE, NOTE_PAUSE,

  NOTE_C7, NOTE_PAUSE, NOTE_PAUSE, NOTE_G6,
  NOTE_PAUSE, NOTE_PAUSE, NOTE_E6, NOTE_PAUSE,
  NOTE_PAUSE, NOTE_A6, NOTE_PAUSE, NOTE_B6,
  NOTE_PAUSE, NOTE_AS6, NOTE_A6, NOTE_PAUSE,

  NOTE_G6, NOTE_E7, NOTE_G7,
  NOTE_A7, NOTE_PAUSE, NOTE_F7, NOTE_G7,
  NOTE_PAUSE, NOTE_E7, NOTE_PAUSE, NOTE_C7,
  NOTE_D7, NOTE_B6, NOTE_PAUSE, NOTE_PAUSE
};

// Mario main them tempo.
int SongOneTempo_g[] = {
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  9, 9, 9,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  9, 9, 9,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
};

// Underworld melody.
int SongTwoMelody_g[] = {
  NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4,
  NOTE_AS3, NOTE_AS4, NOTE_PAUSE,
  NOTE_PAUSE,
  NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4,
  NOTE_AS3, NOTE_AS4, NOTE_PAUSE,
  NOTE_PAUSE,
  NOTE_F3, NOTE_F4, NOTE_D3, NOTE_D4,
  NOTE_DS3, NOTE_DS4, NOTE_PAUSE,
  NOTE_PAUSE,
  NOTE_F3, NOTE_F4, NOTE_D3, NOTE_D4,
  NOTE_DS3, NOTE_DS4, NOTE_PAUSE,
  NOTE_PAUSE, NOTE_DS4, NOTE_CS4, NOTE_D4,
  NOTE_CS4, NOTE_DS4,
  NOTE_DS4, NOTE_GS3,
  NOTE_G3, NOTE_CS4,
  NOTE_C4, NOTE_FS4, NOTE_F4, NOTE_E3, NOTE_AS4, NOTE_A4,
  NOTE_GS4, NOTE_DS4, NOTE_B3,
  NOTE_AS3, NOTE_A3, NOTE_GS3,
  NOTE_PAUSE, NOTE_PAUSE, NOTE_PAUSE
};

// Underwolrd tempo.
int SongTwoTempo_g[] = {
  12, 12, 12, 12,
  12, 12, 6,
  3,
  12, 12, 12, 12,
  12, 12, 6,
  3,
  12, 12, 12, 12,
  12, 12, 6,
  3,
  12, 12, 12, 12,
  12, 12, 6,
  6, 18, 18, 18,
  6, 6,
  6, 6,
  6, 6,
  18, 18, 18, 18, 18, 18,
  10, 10, 10,
  10, 10, 10,
  3, 3, 3
};

void setup(void)
{
  // Configure buzzer.
  pinMode(PIN_BUZZER, OUTPUT);
  
  // Configure LED indicator.
  pinMode(PIN_LED, OUTPUT);

  // Configure the button.
  pinMode(PIN_BUTTON, INPUT_PULLUP);
}

void loop()
{
  // If the button is pressed play the song.
  if (digitalRead(PIN_BUTTON) == HIGH)
  {
    return;
  }
  
  // Play the songs.
  play_song(SONG_1);
  play_song(SONG_1);
  play_song(SONG_2);
}

void play_song(int songIndex)
{
  if(songIndex == SONG_1)
  {
    int SizeL = sizeof(SongOneMelody_g) / sizeof(int);
    
    for (int index = 0; index < SizeL; index++)
    {
      // To calculate the note duration,
      // take one second divided by the note type.
      // e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
      int DurationL = 1000 / SongOneTempo_g[index];

      play_note(PIN_BUZZER, SongOneMelody_g[index], DurationL);

      // To distinguish the notes,
      // set a minimum time between them.
      // the note's duration + 30% seems to work well:
      int PauseL = DurationL * 1.30;
      delay(PauseL);

      // Stop the tone playing.
      play_note(PIN_BUZZER, 0, DurationL);
    }
  }
  else if (songIndex == SONG_2)
  {
    int SizeL = sizeof(SongTwoMelody_g) / sizeof(int);
    
    for (int index = 0; index < SizeL; index++)
    {
      // To calculate the note duration,
      // take one second divided by the note type.
      // e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
      int DurationL = 1000 / SongTwoTempo_g[index];

      play_note(PIN_BUZZER, SongTwoMelody_g[index], DurationL);

      // To distinguish the notes,
      // set a minimum time between them.
      // The note's duration + 30% seems to work well.
      int PauseL = DurationL * 1.30;
      delay(PauseL);

      // Stop the tone playing.
      play_note(PIN_BUZZER, 0, DurationL);
    }
  }
}

void play_note(int pinBuzzer, long frequency, long duration)
{
  digitalWrite(PIN_LED, HIGH);
  
   // Calculate the delay value between transitions.
  long DelayL = 1000000 / frequency / 2;
  
  //// 1 second's worth of microseconds, divided by the frequency, then split in half since
  //// there are two phases to each cycle
  // Calculate the number of cycles for proper timing.
  long CyclesL = frequency * duration / 1000;
  
  // Мultiply frequency, which is really cycles per second,
  // by the number of seconds to get the total number of cycles to produce.
  // For the calculated length of time.
  for (long index = 0; index < CyclesL; index++)
  {
    // Write the buzzer pin high to push out the diaphram.
    digitalWrite(pinBuzzer, HIGH);
    
    // Wait for the calculated delay value.
    delayMicroseconds(DelayL);
    
    // Write the buzzer pin low to pull back the diaphram.
    digitalWrite(pinBuzzer, LOW);
    
    // Wait again or the calculated delay value.
    delayMicroseconds(DelayL);
  }
  
  digitalWrite(PIN_LED, LOW);
}

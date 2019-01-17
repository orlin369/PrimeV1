
// http://dlacko.org/blog/2016/01/24/remove-impulse-noise-from-ultrasonic/

#include <Ultrasonic.h>

#define PIN_BUZZER 8
#define PIN_ECHO 10
#define PIN_TRIG 11
#define PIN_BUTTON 12
#define PIN_LED 13

#define US_MIN_DISTANCE 15
#define US_MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define US_PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define US_MEASUREMENTS_SIZE 5
#define US_OFFSET 2

// Trick using XOR to swap two variables
#define swap(a,b) a ^= b; b ^= a; a ^= b;
#define sort(a,b) if(a>b){ swap(a,b); }

Ultrasonic ultrasonic(PIN_TRIG, PIN_ECHO);

// Cyclic buffer of the last readings accumulated for computing the median
int LastReadings_g[US_MEASUREMENTS_SIZE];

// The number of readings to be able to maintain the cyclic buffer
int ReadingsIndex_g = 0;

/// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long PreviousMilis_g = 0;        // will store last time LED was updated

// http://cs.engr.uky.edu/~lewis/essays/algorithms/sortnets/sort-net.html
// Median could be computed two less steps...
int median(int a, int b, int c, int d, int e)
{
    sort(a,b);
    sort(d,e);  
    sort(a,c);
    sort(b,c);
    sort(a,d);  
    sort(c,d);
    sort(b,e);
    sort(b,c);
    // this last one is obviously unnecessary for the median
    //sort(d,e);
  
    return c;
}

int echoCheck() 
{
  int lastMeasure = US_MAX_DISTANCE;

  int DistanceL = US_MIN_DISTANCE;
  
  DistanceL = (int)ultrasonic.convert(ultrasonic.timing(), Ultrasonic::CM);

  DistanceL -= US_OFFSET;
  
  if (DistanceL < US_MIN_DISTANCE)
  {
    DistanceL = US_MIN_DISTANCE;
  }
  
  if(DistanceL > US_MAX_DISTANCE)
  {
    DistanceL = US_MAX_DISTANCE;
  }


  
  LastReadings_g[ReadingsIndex_g++ % US_MEASUREMENTS_SIZE] = DistanceL;
  
  lastMeasure = median(
                  LastReadings_g[0], 
                  LastReadings_g[1],
                  LastReadings_g[2],
                  LastReadings_g[3],
                  LastReadings_g[4]);
                  
  return lastMeasure;
}

void setup(void)
{
  // This makes the first measurements returning US_MAX_DISTANCE 
  for(int i=0; i<5; i++)
  {
    LastReadings_g[i] = US_MAX_DISTANCE;
  }

    // Configure buzzer.
  pinMode(PIN_BUZZER, OUTPUT);
  
  // Configure LED indicator.
  pinMode(PIN_LED, OUTPUT);

  Serial.begin(115200);
}

void loop()
{
  unsigned long CurremtMilisL = millis();

  if (CurremtMilisL - PreviousMilis_g >= US_PING_INTERVAL)
  {
    // save the last time you blinked the LED
    PreviousMilis_g = CurremtMilisL;
    int distance = echoCheck();
    Serial.print("Distance: ");
    Serial.println(distance);

    if(distance < 42 && distance > 38)
    {
      play_note(PIN_BUZZER, 2093, 1000 / 12);
    }
  }
}

void play_note(int pinBuzzer, long frequency, long duration)
{
  digitalWrite(PIN_LED, HIGH);
  
   // Calculate the delay value between transitions.
  long DelayL = 500000 / frequency;
  
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

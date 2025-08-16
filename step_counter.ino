#include "step_counter.h"
#include <vector>
#include <algorithm>

const int xpin = A1;
const int ypin = A2;
const int zpin = A3;

const int analogMin = 0;
const int analogMax = 1023;

const int accelerationMin = -3000;
const int accelerationMax = 3000;

float dynamicThreshold = -3.0;
const float sensitivity = 0.2;

LoopState loopState = lookingForMaxPeak;

int stepCount = 0;
int possibleStepCount = 0;

int lookingForMinPeakCount = 0;

bool regulationMode = false;

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  std::vector<float> vals;

  for (int i = 0;i < 4;i++) {
    float val = lowPassFilter();
    vals.push_back(val);

    // wait for 50 ms (0.05 sec) for each (total 0.2 sec per a time window)
    delay(50);
  }
  float maxPeak = *std::max_element(vals.begin(), vals.end());
  float minPeak = *std::min_element(vals.begin(), vals.end());
  if (maxPeak - minPeak > sensitivity) {
    dynamicThreshold = (dynamicThreshold + (maxPeak + minPeak) / 2) / 2;
  }
  float maxThreshold = dynamicThreshold + sensitivity / 2;
  float minThreshold = dynamicThreshold - sensitivity / 2;

  if (loopState == lookingForMaxPeak && maxPeak > maxThreshold) {
    loopState = lookingForMinPeak;
  } else if (loopState == lookingForMinPeak && minPeak < minThreshold) {
    if (regulationMode) {
      stepCount++;
    } else {
      possibleStepCount++;
      if (possibleStepCount >= 8) {
        stepCount += 8;
        possibleStepCount = 0;
        regulationMode = true;
      }
    }
    loopState = lookingForMaxPeak;
  } else if (loopState == lookingForMinPeak && minPeak >= minThreshold) {
    if (lookingForMinPeakCount >= 5) {
      lookingForMinPeakCount = 0;
      loopState = lookingForMaxPeak;
      possibleStepCount = 0;
      regulationMode = false;
    } else {
      lookingForMinPeakCount++;
    }
  } else {
    possibleStepCount = 0;
    regulationMode = false;
  }

    for (auto it = vals.begin();it != vals.end();it++) {
    Serial.print("filtered_module:");
    Serial.print(*it);
    Serial.print(", ");
    Serial.print("maxPeak:");
    Serial.print(maxPeak);
    Serial.print(", ");
    Serial.print("minPeak:");
    Serial.print(minPeak);
    Serial.print(", ");  
    Serial.print("dynamic_threshold:");
    Serial.print(dynamicThreshold);
    Serial.print(", ");
    Serial.print("max_threshold:");
    Serial.print(maxThreshold);
    Serial.print(", ");
    Serial.print("min_threshold:");
    Serial.print(minThreshold);
    Serial.print(", ");
    Serial.print("step:");
    Serial.print(stepCount);
    Serial.print("\n");
  }

}

float analog2acceleration(int analog) {
  int millig = map(analog, analogMin, analogMax, accelerationMin, accelerationMax);
  float g = millig / 1000.0;
  return g;
}

float lowPassFilter() {
  float average = 0.0;

  for (int i = 0;i < 4;i++) {
    int xraw = analogRead(xpin);
    int yraw = analogRead(ypin);
    int zraw = analogRead(zpin);

    float xval = analog2acceleration(xraw);
    float yval = analog2acceleration(yraw);
    float zval = analog2acceleration(zraw);

    /*
    Serial.print("x:");
    Serial.print(xval);
    Serial.print(",");
    Serial.print("y:");
    Serial.print(yval);
    Serial.print(",");
    Serial.print("z:");
    Serial.print(zval);
    */

    float sum = xval + yval + zval;
    if (i == 0) {
      average = sum;
    } else {
      average = (average + sum) / 2;
    }
  }
  return average;
}


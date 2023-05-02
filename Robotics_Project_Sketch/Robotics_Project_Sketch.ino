/*
  An Arduino IDE SW for experimenting with the line tracking module
*/

#define NO_OF_SENSORS 5
#define ADC_RANGE 1024
#define PWM_RANGE 256
#define HP_FILTER_FACTOR 0.5

typedef struct
{
  unsigned pinNo;
  float    inputValue;
  float    previousInputValue;
  float    sensorSignal;
  float    sensorScaling;
  unsigned logicalLevel;
}
ts_sensor;

ts_sensor sensor[NO_OF_SENSORS];

float rightEdgePosition = 0.0;
float leftEdgePosition  = 0.0;
float    positionValue[NO_OF_SENSORS+1] = {0.5, 0.3, 0.1, -0.1, -0.3, -0.5};

int debugPrescaler = 0;

void setup()
{
  int i;

  Serial.begin(9600);
  while (!Serial);

  sensor[0].pinNo = A0;
  sensor[1].pinNo = A1;
  sensor[2].pinNo = A2;
  sensor[3].pinNo = A3;
  sensor[4].pinNo = A4;

  sensor[0].sensorScaling = 35.0 / 28.0;
  sensor[1].sensorScaling = 35.0 / 20.0;
  sensor[2].sensorScaling = 35.0 / 37.0;
  sensor[3].sensorScaling = 35.0 / 28.0;
  sensor[4].sensorScaling = 35.0 / 29.0;

  for (i=0; i<NO_OF_SENSORS; i++)
  {
    sensor[i].inputValue = ((float)analogRead(sensor[i].pinNo) / ADC_RANGE) * sensor[i].sensorScaling;
  }
}

void loop()
{
  int i;
  float inputValue;

  // Read sensors and determine their levels
  
  for (i=0; i<NO_OF_SENSORS; i++)
  {
    sensor[i].previousInputValue = sensor[i].inputValue;
    sensor[i].inputValue = ((float)analogRead(sensor[i].pinNo) / ADC_RANGE) * sensor[i].sensorScaling;

    sensor[i].sensorSignal =
      (HP_FILTER_FACTOR * sensor[i].sensorSignal) + (sensor[i].inputValue - sensor[i].previousInputValue);

    if (sensor[i].sensorSignal > 0.02)
      sensor[i].logicalLevel = 1;
    if (sensor[i].sensorSignal < -0.02)
      sensor[i].logicalLevel = 0;
  }

  // Determine the position of the line's right edge and set the position value

  for (i=NO_OF_SENSORS-1; i>=0; i--)
  {
    rightEdgePosition = positionValue[i+1];
    if (sensor[i].logicalLevel == 1)
    {
      break;
    }
    rightEdgePosition = positionValue[i];
  }

  // Determine the position of the line's left edge and set the position value

  for (i=0; i<=NO_OF_SENSORS-1; i++)
  {
    leftEdgePosition = positionValue[i];
    if (sensor[i].logicalLevel == 1)
    {
      break;
    }
    leftEdgePosition = positionValue[i+1];
  }

  // Dump debug information

  /* if (debugPrescaler++ >= 20) */
  {
    debugPrescaler = 0;

    Serial.print(" Input:  ");
    for (i=0; i<NO_OF_SENSORS; i++)
    {
      Serial.print(sensor[i].inputValue);
      Serial.print("     ");
    }
    Serial.println();

    Serial.print(" Prev.:  ");
    for (i=0; i<NO_OF_SENSORS; i++)
    {
      Serial.print(sensor[i].previousInputValue);
      Serial.print("     ");
    }
    Serial.println();

    Serial.print(" Signal: ");
    for (i=0; i<NO_OF_SENSORS; i++)
    {
      Serial.print(sensor[i].sensorSignal);
      Serial.print(":");
      Serial.print(sensor[i].logicalLevel);
      Serial.print("  ");
    }
    Serial.println();

    Serial.print(" Pos:    ");
    Serial.print(leftEdgePosition);
    Serial.print("  ");
    Serial.print(rightEdgePosition);
    Serial.println();
  
    Serial.println();
  }
  
  delay(2000);
}

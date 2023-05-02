/*
  An Arduino IDE SW for experimenting with the line tracking module
*/

#define NO_OF_SENSORS 5
#define ADC_RANGE 1024
#define PWM_RANGE 256

typedef struct
{
  unsigned pinNo;
  float    inputValue;
  float    sensorScaling;
  unsigned logicalLevel;
}
ts_sensor;

ts_sensor sensor[NO_OF_SENSORS];

float commonMax;
float commonMin;

float rightEdgePosition = 0.0;
float leftEdgePosition  = 0.0;
float positionValue[NO_OF_SENSORS+1] = {0.5, 0.3, 0.1, -0.1, -0.3, -0.5};

int debugPrescaler = 0;

void calibrate()
{
  int i;

  commonMax = 0.0;
  commonMin = 1.0;
  for (i=0; i<NO_OF_SENSORS; i++)
  {
    if (sensor[i].inputValue > commonMax) commonMax = sensor[i].inputValue;
    if (sensor[i].inputValue < commonMin) commonMin = sensor[i].inputValue;
  }
}

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
  calibrate();
}

void loop()
{
  int i;
  t_boolean someHigh = FALSE;
  t_boolean someLow  = FALSE;

  // Read sensors and determine their levels
  
  for (i=0; i<NO_OF_SENSORS; i++)
  {
    sensor[i].inputValue = ((float)analogRead(sensor[i].pinNo) / ADC_RANGE) * sensor[i].sensorScaling;

    if (sensor[i].inputValue > (commonMax + commonMin) / 2.0)
    {
      sensor[i].logicalLevel = 1;
      someHigh = TRUE;
    }
    else
    {
      sensor[i].logicalLevel = 0;
      someLow = TRUE;
    }
  }

  if (someHigh && someLow)
  {
    calibrate();
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

    Serial.print(" Max:    ");
    Serial.print(commonMax);
    Serial.println();

    Serial.print(" Input:  ");
    for (i=0; i<NO_OF_SENSORS; i++)
    {
      Serial.print(sensor[i].inputValue);
      Serial.print(":");
      Serial.print(sensor[i].logicalLevel);
      Serial.print("  ");
    }
    Serial.println();

    Serial.print(" Min:    ");
    Serial.print(commonMin);
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

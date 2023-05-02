/*
  An Arduino IDE SW for experimenting with the line tracking module
*/

#define NO_OF_SENSORS 5
#define ADC_RANGE 1024

typedef struct
{
  unsigned pinNo;
  float    minValue;
  float    maxValue;
  float    sensorLevel;
  float    sensorScaling;
  unsigned logicalLevel;
}
ts_sensor;

ts_sensor sensor[NO_OF_SENSORS];

float rightEdgePosition = 0.0;
float leftEdgePosition  = 0.0;
float positionValue[NO_OF_SENSORS+1] = {0.5, 0.2, 0.1, -0.1, -0.2, -0.5};


void setup()
{
  Serial.begin(9600);
  while (!Serial);

  sensor[0].pinNo = A0;
  sensor[1].pinNo = A1;
  sensor[2].pinNo = A2;
  sensor[3].pinNo = A3;
  sensor[4].pinNo = A4;

  sensor[0].minValue = 1.0;
  sensor[1].minValue = 1.0;
  sensor[2].minValue = 1.0;
  sensor[3].minValue = 1.0;
  sensor[4].minValue = 1.0;

  sensor[0].maxValue = 0.0;
  sensor[1].maxValue = 0.0;
  sensor[2].maxValue = 0.0;
  sensor[3].maxValue = 0.0;
  sensor[4].maxValue = 0.0;

  sensor[0].sensorScaling = 30.0 / 28.0;
  sensor[1].sensorScaling = 30.0 / 24.0;
  sensor[2].sensorScaling = 30.0 / 35.0;
  sensor[3].sensorScaling = 30.0 / 23.0;
  sensor[4].sensorScaling = 30.0 / 28.0;
}

void loop()
{
  int i;

  // Read sensors and determine their levels
  
  for (i=0; i<NO_OF_SENSORS; i++)
  {
    sensor[i].sensorLevel = ((float)analogRead(sensor[i].pinNo) / ADC_RANGE) * sensor[i].sensorScaling;

    if (sensor[i].sensorLevel < sensor[i].minValue) sensor[i].minValue = sensor[i].sensorLevel;
    if (sensor[i].sensorLevel > sensor[i].maxValue) sensor[i].maxValue = sensor[i].sensorLevel;

    if (sensor[i].sensorLevel > (sensor[i].maxValue + sensor[i].minValue) / 2.0)
      sensor[i].logicalLevel = 1;
    else
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

  Serial.print(" Max:    ");
  for (i=0; i<NO_OF_SENSORS; i++)
  {
    Serial.print(sensor[i].maxValue);
    Serial.print("    ");
  }
  Serial.println();

  Serial.print(" Value:  ");
  for (i=0; i<NO_OF_SENSORS; i++)
  {
    Serial.print(sensor[i].sensorLevel);
    Serial.print(":");
    Serial.print(sensor[i].logicalLevel);
    Serial.print("  ");
  }
  Serial.println();

  Serial.print(" Min:    ");
  for (i=0; i<NO_OF_SENSORS; i++)
  {
    Serial.print(sensor[i].minValue);
    Serial.print("    ");
  }
  Serial.println();

  Serial.print(" Pos:    ");
  Serial.print(leftEdgePosition);
  Serial.print("  ");
  Serial.print(rightEdgePosition);
  Serial.println();

  Serial.println();

  delay(2000);
}

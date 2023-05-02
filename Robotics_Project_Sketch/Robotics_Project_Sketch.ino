/*
  An Arduino IDE SW controlling the steering of the Off Road Tracer based on the position relative to a line
*/

// Pin description
int steeringInA = 31;
int steeringInB = 30;
int motionInA   = 40;
int motionInB   = 41;

int sensorLeftPin  = A0;
int sensorLeftInput;
int sensorLeftMin = 1024;
int sensorLeftMax = 0;
int sensorLeftLevel = 0;

int sensorRightPin = A1;
int sensorRightInput;
int sensorRightMin = 1024;
int sensorRightMax = 0;
int sensorRightLevel = 0;

// Motion controlling constants
int countTurn50degreesFwd = 25;
int countTurn80degreesBwd = 30;
int countRampUpAndDown = 10;
int pwmDurationPct = 20;

void setup()
{
  pinMode(steeringInA, OUTPUT);
  pinMode(steeringInB, OUTPUT);
  pinMode(motionInA,   OUTPUT);
  pinMode(motionInB,   OUTPUT);

  motionStop();
  steeringStraight();
  delay(500);

  Serial.begin(9600);
  while (!Serial);
}

void steeringStraight()
{
  digitalWrite(steeringInA, LOW);
  digitalWrite(steeringInB, LOW);
}

void steeringRight()
{
  digitalWrite(steeringInA, HIGH);
  digitalWrite(steeringInB, LOW);
}

void steeringLeft()
{
  digitalWrite(steeringInA, LOW);
  digitalWrite(steeringInB, HIGH);
}

void motionStop()
{
  digitalWrite(motionInA, LOW);
  digitalWrite(motionInB, LOW);
}

void motionFwd()
{
  digitalWrite(motionInA, LOW);
  digitalWrite(motionInB, HIGH);
}

void motionBwd()
{
  digitalWrite(motionInA, HIGH);
  digitalWrite(motionInB, LOW);
}

void loop()
{
  static int debugPrescaler = 1;
  static int pwmPrescaler = 10;
  static float sensorPosition = 0;

  // Drive forward at low speed
  
  if (pwmPrescaler <= pwmDurationPct)
  {
    //motionFwd();
  }
  else
  {
    motionStop();
  }

  pwmPrescaler += 10;
  if (pwmPrescaler > 100)
  {
    pwmPrescaler = 10;
  }

  // Determine position relative to the line
  
  sensorLeftInput  = analogRead(sensorLeftPin);
  if (sensorLeftInput < sensorLeftMin) sensorLeftMin = sensorLeftInput;
  if (sensorLeftInput > sensorLeftMax) sensorLeftMax = sensorLeftInput;
  sensorLeftLevel = 0;
  if (sensorLeftInput > (sensorLeftMax+sensorLeftMin)/2) sensorLeftLevel = 1;

  sensorRightInput = analogRead(sensorRightPin);
  if (sensorRightInput < sensorRightMin) sensorRightMin = sensorRightInput;
  if (sensorRightInput > sensorRightMax) sensorRightMax = sensorRightInput;
  sensorRightLevel = 0;
  if (sensorRightInput > (sensorRightMax+sensorRightMin)/2) sensorRightLevel = 1;

  sensorPosition = sensorLeftLevel * (-0.5) + sensorRightLevel * 0.5;

  if (debugPrescaler++ == 200)
  {
    debugPrescaler = 1;

    Serial.print(" Max:  ");
    Serial.print(sensorLeftMax);
    Serial.print("   - ");
    Serial.print(sensorRightMax);
    Serial.println();
  
    Serial.print("       ");
    Serial.print(sensorLeftInput);
    Serial.print(":");
    Serial.print(sensorLeftLevel);
    Serial.print(" - ");
    Serial.print(sensorRightInput);
    Serial.print(":");
    Serial.print(sensorRightLevel);
    Serial.print("  => pos: ");
    Serial.print(sensorPosition);
    Serial.println();
  
    Serial.print(" Min:  ");
    Serial.print(sensorLeftMin);
    Serial.print("   - ");
    Serial.print(sensorRightMin);
    Serial.println();
  
    Serial.println();
  }

  // Control the steering
  
  if (sensorPosition > 0.0)
  {
    steeringLeft();
  }
  else if (sensorPosition < 0.0)
  {
    steeringRight();
  }
  else
  {
    steeringStraight();
  }

  // Delay to provide some timing of the process

  delay(10);
}

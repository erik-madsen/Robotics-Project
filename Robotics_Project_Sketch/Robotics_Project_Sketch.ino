/*
  A simple Arduino IDE SW using programmed pulsing to control speed
*/

// Pin description
int steeringInA = 31;
int steeringInB = 30;
int motionInA   = 40;
int motionInB   = 41;

// Motion controlling constants
int countTurn50degreesFwd = 25;
int countTurn80degreesBwd = 30;
int countRampUpAndDown = 10;

int noOfLaps = 2;

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
  int i;

  if (noOfLaps > 0)
  {
    noOfLaps--;  

    delay(200);

    Serial.println("Ramp up to full speed driving straight forward");
    for (i=0; i<countRampUpAndDown; i++)
    {
      // Driving
      motionFwd();
      delay(20 + i * 80/countRampUpAndDown);
      motionStop();
      delay(80 - i * 80/countRampUpAndDown);
    }

    Serial.println("Driving full speed");
    motionFwd();
    delay(1000);

    Serial.println("Ramp down to slow speed driving straight forward");
    for (i=0; i<countRampUpAndDown; i++)
    {
      // Driving
      motionFwd();
      delay(100 - i * 80/countRampUpAndDown);
      motionStop();
      delay(i * 80/countRampUpAndDown);
    }

    Serial.println("Stop any possible coasting");
    motionBwd();
    delay(20);
    motionStop();

    delay(500);

    Serial.println("Turn approximately 50 degrees right driving slowly forward");
    steeringRight();
    delay(200);
    for (i=0; i<countTurn50degreesFwd; i++)
    {
      // Driving
      motionFwd();
      delay(20);
      motionStop();
      delay(80);
    }

    delay(500);

    Serial.println("Turn approximately 80 degrees left driving slowly backward");
    steeringLeft();
    delay(200);
    for (i=0; i<countTurn80degreesBwd; i++)
    {
      // Driving
      motionBwd();
      delay(20);
      motionStop();
      delay(80);
    }

    delay(500);

    Serial.println("Turn approximately 50 degrees right driving slowly forward");
    steeringRight();
    delay(200);
    for (i=0; i<countTurn50degreesFwd; i++)
    {
      // Driving
      motionFwd();
      delay(20);
      motionStop();
      delay(80);
    }

    steeringStraight();
  }

}

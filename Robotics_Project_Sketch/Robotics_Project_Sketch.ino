/*
  A simple "Hello World" sketch using the Serial Monitor.
*/

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void loop()
{
  int count;
  int limit = 3;

  for (count = 1; count <= limit; count++)
  {
    if (count < limit)
    {
      Serial.println(count);
    }
    else
    {
      Serial.print(count);
      Serial.println(" - Hello World");
      Serial.println();
    }
    delay(1000);
  }
}

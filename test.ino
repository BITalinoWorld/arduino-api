
#include <SoftwareSerial.h>

SoftwareSerial statusPort(8, 9); // RX, TX pins (only the TX pin will be used for status output)

boolean ledState = false;

void printStatus(boolean ok)
{
  statusPort.println(ok ? "Ok" : "Error");
}

// the setup routine runs once when you press reset
void setup()
{
  statusPort.begin(115200);  // status output serial port
  
  BITalino.begin();
  
  char verStr[30];
  BITalino.version(verStr, sizeof verStr);    // get device version string
  statusPort.print("BITalino version: ");
  statusPort.println(verStr);
  
  boolean ok = BITalino.battery(10);  // set battery threshold (optional)
  statusPort.print("Set battery: ");
  printStatus(ok);
  
  ok = BITalino.start(1000, 0x3F);   // start acquisition of all channels at 1000 Hz
  statusPort.print("Start: ");
  printStatus(ok);
}

// the loop routine runs over and over again forever
void loop()
{
  BITalinoFrame frame;

  ledState = !ledState;   // toggle LED state
  BITalino.trigger(ledState ? 4 : 0);

  word n = BITalino.read(1, &frame);  // get 1 frame from device
  if (n != 1)
  {
    statusPort.println("Error while receiving a frame");
    return;
  }
  
  // dump the first frame of each 100 frames
  statusPort.print(frame.seq);
  statusPort.print(" :");
  for(byte i = 0; i < 4; i++)
  {
    statusPort.print(' ');
    statusPort.print(frame.digital[i]);
  }
  statusPort.print(" ;");
  for(byte i = 0; i < 6; i++)
  {
    statusPort.print(' ');
    statusPort.print(frame.analog[i]);
  }
  statusPort.println();
  
  // read and ignore the next 99 frames
  for(byte i = 0; i < 99; i++)
    BITalino.read(1, &frame);  
}


// CAN Bus Transmitter Code - Blake McBride/Iron Hydroxide
// Based on Code from Adafruit and MCP_CAN

#include <mcp_can.h>
#include <SPI.h>

#define DEBOUNCE 5  // button debouncer, how many ms to debounce, 5+ ms is usually plenty
MCP_CAN CAN0(10);   // Set CS to pin 3

#define NUMBUTTONS_TOTAL 16
// we will track if a button is just pressed, just released, or 'currently pressed' 
volatile byte pressed[NUMBUTTONS_TOTAL], can_status[NUMBUTTONS_TOTAL];

//MUX control pins
int s0 = 7;
int s1 = 6;
int s2 = 5;
int s3 = 4;

// MUX Signal Input Pins
int SIG_pin = 3;

void setup()
{
  Serial.begin(115200);
    
  digitalWrite(3, INPUT_PULLUP);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);

  // CAN Control Setup
  // Initialize MCP2515 running at 8MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted  

  // Run timer2 interrupt every 15 ms 
  TCCR2A = 0;
  TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20;

  //Timer2 Overflow Interrupt Enable
  TIMSK2 |= 1<<TOIE2;  
}

SIGNAL(TIMER2_OVF_vect) 
{
  check_switches();
}

// Switch Check Function - This is run on 15ms intervals via the interrupt "SIGNAL(TIMER2_OVF_vect)"
void check_switches()
{
  static byte previousstate[NUMBUTTONS_TOTAL];
  static byte currentstate[NUMBUTTONS_TOTAL];
  static long lasttime;
  byte index;
    
  if (millis() < lasttime){ // we wrapped around, lets just try again
     lasttime = millis();
  }
  
  if ((lasttime + DEBOUNCE) > millis()) {
    // not enough time has passed to debounce
    return; 
  }
  // ok we have waited DEBOUNCE milliseconds, lets reset the timer
  lasttime = millis();
  
  for (index = 0; index < NUMBUTTONS_TOTAL; index++)
  {
    currentstate[index]=readMux(index); // read the value and store it 
    
    if (currentstate[index] == previousstate[index]) {
      if ((pressed[index] == LOW) && (currentstate[index] == LOW)) {
          // pressed
          can_status[index] = 1;
      }
      else if ((pressed[index] == HIGH) && (currentstate[index] == HIGH)) {
          // released
          can_status[index] = 0;
      }      
      pressed[index] = !currentstate[index];  // remember, digital HIGH means NOT pressed
    }
    previousstate[index] = currentstate[index];   // keep a running tally of the buttons
  }  
}

/////////////////////////

int readMux(int channel)
{ 
  int muxChannel[16][4]=
  { 
    {0,0,0,0},
    {1,0,0,0},
    {0,1,0,0},
    {1,1,0,0}, 
    {0,0,1,0},
    {1,0,1,0},
    {0,1,1,0}, 
    {1,1,1,0}, 
    {0,0,0,1}, 
    {1,0,0,1}, 
    {0,1,0,1}, 
    {1,1,0,1}, 
    {0,0,1,1},
    {1,0,1,1},
    {0,1,1,1},
    {1,1,1,1}
  };
   
  if (channel < 16)
  {
    int controlPin[] = {s0, s1, s2, s3};
    
    for(int i = 0; i < 4; i ++)
    {
      digitalWrite(controlPin[i], muxChannel[channel][i]); 
    }
    //read the value at the SIG pin 
    int val = digitalRead(SIG_pin); //return the value 
    return val; 
  }   
}

void loop()
{


  // Serial Print Button Status (for testing/development)
  for (byte i = 0; i < NUMBUTTONS_TOTAL; i++)
  {
    if (pressed[i]) {
      Serial.print("Button: ");      
      Serial.print(i, DEC);
      Serial.print(" pressed, CAN Status: ");
      Serial.println(can_status[i]);       
      // is the button pressed down at this moment
    }
  }
  
  // CAN LOOP

  // CAN Message Assignment.   These allow up to 8 messages per string, identified as "can_status[x]" with x = 0-31, identifiers of which button input (0-23) or analog input (24-31) you want to send a value from.
  
  unsigned char canMsg_1[2][8] = {
   {can_status[0], can_status[1], can_status[2], can_status[3], can_status[4], can_status[5], can_status[6], can_status[7]},
   {can_status[8], can_status[9], can_status[10], can_status[11], can_status[12], can_status[13], can_status[14], can_status[15]},
  };
  unsigned char canMsg2[8] = {can_status[8], can_status[9], can_status[10], can_status[11], can_status[12], can_status[13], can_status[14], can_status[15]};
  unsigned char canMsg3[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; 

  // Identify CAN messages by downstream module
  
  // CAN ID 1 -> rm1x - Relay Module 1, 2 channels of data
  long canID1[2] = {0x126d3131, 0x126d3132};
  // CAN ID 2 -> rm2x - Relay Module 2, 2 channels of data
  long canID2[2] = {0x126d3231, 0x126d3232};
  // CAN ID 3 -> rm3x - Relay Module 3, 2 channels of data
  long canID3[2] = {0x126d3331, 0x126d3332};

  // format:    (id, id_type, dlc, data buf)

  for(int i = 0; i < 2; i ++)
    {
      byte sndStat = CAN0.sendMsgBuf(canID1[i], 1, 8, canMsg_1[i]);
      if(sndStat == CAN_OK){
        Serial.println("Messages sent to relay control 1 successfully!");
      } 
      else 
      {
//        Serial.println("Error Sending Message to relay control 1...");
      }
    }  
  
//  delay(20);   // send data per 100ms
  byte sndStat2 = CAN0.sendMsgBuf(canID2[0], 1, 8, canMsg2);
  if(sndStat2 == CAN_OK){
    Serial.println("Message 2 Sent Successfully!");
  } else {
//    Serial.println("Error Sending Message 2...");
  }
//  delay(20);   // send data per 100ms
  byte sndStat3 = CAN0.sendMsgBuf(canID3[0], 1, 8, canMsg3);
  if(sndStat3 == CAN_OK){
    Serial.println("Message 3 Sent Successfully!");
  } else {
//    Serial.println("Error Sending Message 3...");
  }  
//  delay(20);   // send data per 100ms  

}

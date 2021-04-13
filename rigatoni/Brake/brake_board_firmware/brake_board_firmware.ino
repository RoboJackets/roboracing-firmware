#include "brake_board_firmware.h"
#include "RigatoniNetwork.h"
#include "RJNet.h"
#include <Ethernet.h>
#include <SPI.h>
#include <avr/wdt.h>

// Notes:
// Need to connect VDC GND and dir- to common GND
// Need to have feedback connected to function
// make sure encoder switch is set to run mode

/* Ethernet */
EthernetServer server(PORT);

EthernetClient estopBoard;

bool estopIsResponding = false;
unsigned long lastEstopMessageTime = 0;

//True when the car is limited or disabled
bool engageMaxBraking = true;


void setup() {
  /* Initialization for encoder*/
//  pinMode(SPI_SCLK, OUTPUT);
//  pinMode(SPI_MOSI, OUTPUT);
//  pinMode(SPI_MISO, INPUT);
//  pinMode(ENC, OUTPUT);
//  digitalWrite(ENC, HIGH); // disable encoder on setup
//  SPI.begin();
  pinMode(LED, OUTPUT);

  /* Initialization for ethernet*/
  pinMode(CS_ETH, OUTPUT);
  Ethernet.init(CS_ETH);  // SCLK pin from eth header
  Ethernet.begin(brakeMAC, brakeIP); // initialize ethernet device
  
  unsigned long loopCounter = 0;
  while(Ethernet.hardwareStatus() == EthernetNoHardware) {
    digitalWrite(LED, loopCounter++ % 4 == 0);
    Serial.println("Ethernet shield was not found.");
    delay(100);
  }
  while(Ethernet.linkStatus() == LinkOFF) {
    digitalWrite(LED, loopCounter++ % 4 > 0);
    Serial.println("Ethernet cable is not connected.");
    delay(100);
  }
  server.begin();
  
  estopBoard.setConnectionTimeout(ETH_TCP_INITIATION_DELAY);

  /* Initialization for stepper*/
  Serial.begin(BAUDRATE);
  pinMode(dirPin, OUTPUT);
  pinMode(pulsePin, OUTPUT);
  pinMode(commandInterruptPin, INPUT_PULLUP);
  digitalWrite(pulsePin, HIGH);


  /* Initialization for timer interrupt for stepper motor */
  cli();//stop interrupts

//  set timer0 interrupt at 2kHz
//  TCCR0A = 0;// set entire TCCR0A register to 0
//  TCCR0B = 0;// same for TCCR0B
//  TCNT0  = 0;//initialize counter value to 0
//  // set compare match register for 2khz increments
//  OCR0A = 124;// = (16*10^6) / (2000*64) - 1 (must be <256)
//  // turn on CTC mode
//  TCCR0A |= (1 << WGM01);
//  // Set CS01 and CS00 bits for 64 prescaler
//  TCCR0B |= (1 << CS01) | (1 << CS00);   
//  // enable timer compare interrupt
//  TIMSK0 |= (1 << OCIE0A);

  //set timer1 interrupt at 1Hz
//  TCCR1A = 0;// set entire TCCR1A register to 0
//  TCCR1B = 0;// same for TCCR1B
//  TCNT1  = 0;//initialize counter value to 0
//  // set compare match register for 1hz increments
//  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
//  // turn on CTC mode
//  TCCR1B |= (1 << WGM12);
//  // Set CS12 and CS10 bits for 1024 prescaler
//  TCCR1B |= (1 << CS12) | (1 << CS10);  
//  // enable timer compare interrupt
//  TIMSK1 |= (1 << OCIE1A);
  
  sei();//allow interrupts

//  delay(2000);
//  wdt_enable(WDTO_1S);
}
 unsigned long lastPrintTime = 0;
void loop() {
//  Serial.println(toggle1);
  readEthernet();  // check for new angle from ethernet
  askEStop();
  updateEStopState();
  
  if (millis() - 500 > lastPrintTime){
    lastPrintTime = millis();
    if(engageMaxBraking){
      Serial.println("Car stopped by estop, max braking");
    }
    else{
      Serial.println("Ordinary operation");
    }
  }
  
//  assignDirection();
//  wdt_reset();
}

void readEthernet(){ 
  EthernetClient client = server.available();    // if there is a new message from client create client object, otherwise new client object null
  while (client) {
    String data = RJNet::readData(client);  // if i get string from RJNet buffer (brake_value=%int)
    client.setConnectionTimeout(ETH_TCP_INITIATION_DELAY);   //Set connection delay so we don't hang
    if (client.remoteIP() == driveIP){
      if (data.length() != 0) {   // if data exists
        if (data.substring(0,1) == "B"){    // if client is giving us new angle in the form of S=$float
          desiredAngle = data.substring(2).toInt();
          
          if (desiredAngle > maxInt){    // checks if the new angle is too large
            Serial.print("Desired angle too large: ");
            Serial.println(desiredAngle);
            desiredAngle = maxInt;
            
          } else if (desiredAngle < minInt) {  //checks if the new angle is too small
            Serial.print("Desired angle too small: ");
            Serial.println(desiredAngle);
            desiredAngle = minInt;
          }
          
          //These should be changed to forces
          String reply = "R=" + String(currentAngle);   // reply with R= currentAngle
          RJNet::sendData(client, reply);
          Serial.println("R");
        }
      } else if (data == "F?"){  // otherwise if client just asking for angle
        String reply = "F=" + String(currentAngle);  // reply with A=currentAngle
        RJNet::sendData(client, reply);
      }
    }
    client = server.available();
  }

}

void askEStop() {
    /* Sends a request to Estop for the current car state
    */
  if(millis() - startTime >= 200){
    //Dont' spam server with messages
    startTime = millis();
    if (!estopBoard.connected()) {
      estopBoard.connect(estopIP, PORT);
      Serial.print("Trying to connect to Estop at ");
      Serial.println(estopIP);
    }
    else{
      RJNet::sendData(estopBoard, "S?");
      Serial.println("Asking state from Estop");
    }
  }
}

void updateEStopState(){
  while (estopBoard.available()) {
    String data = RJNet::readData(estopBoard);  // if i get string from RJNet buffer (brake_value=%int)
    if (data.length() != 0) {   // if data exists
      //Max braking unless car state is going
      engageMaxBraking = !(data == goMsg);
      if(!((data == goMsg)||(data == limitedMsg)||(data == stopMsg))){
        Serial.print("Illegal message from Estop: ");
        Serial.println(data);
      }
      else{
        lastEstopMessageTime = millis();
        Serial.println("New state from Estop.");
      }
    }
  }
  if(millis() - estopTimeoutMS > lastEstopMessageTime){
    //Estop is not responding. Max braking.
    engageMaxBraking = true;
    estopIsResponding = false;
  }
  else{
    estopIsResponding = true;
  }
}

ISR(TIMER1_COMPA_vect){//timer0 interrupt 2kHz toggles pin 8
//generates pulse wave of frequency 2kHz/2 = 1kHz (takes two cycles for full wave- toggle high then toggle low)
  if (toggle1){
    digitalWrite(8,HIGH);
    toggle1 = 0;
  }
  else{
    digitalWrite(8,LOW);
    toggle1 = 1;
  }
  if (prevtoggle!=toggle1 && currentAngle != desiredAngle){
    pulse();
  }
  prevtoggle = toggle1;
}

///* For setting direction of stepper motor */
void assignDirection(){       
  currentAngle = getPositionSPI();         // read current position from encoder
  if (desiredAngle != currentAngle){      // checks if motor needs to turn
    if (desiredAngle < currentAngle){      // set dirPIN to CW or CCW
      digitalWrite(dirPin, LOW);
      pulse();
    } else {
      digitalWrite(dirPin, HIGH);
      pulse();
    }
  }
}

void pulse(){                           // rotates stepper motor one step in the currently set direction
  digitalWrite(pulsePin, HIGH);
  delayMicroseconds(30);
  digitalWrite(pulsePin, LOW);
  delayMicroseconds(30);
}

void setCSLine (uint8_t csLine){   // enable or disable encoder
  digitalWrite(ENC, csLine);
}


uint8_t spiWriteRead(uint8_t sendByte, uint8_t releaseLine){
  uint8_t data;

  //set cs low, cs may already be low but there's no issue calling it again except for extra time
  setCSLine(LOW);

  //There is a minimum time requirement after CS goes low before data can be clocked out of the encoder.
  delayMicroseconds(3);

  //send the command  
  data = SPI.transfer(sendByte);
  delayMicroseconds(3); //There is also a minimum time after clocking that CS should remain asserted before we release it
  setCSLine(releaseLine); //if releaseLine is high set it high else it stays low
  
  return data;
}

void setZeroSPI(){
  spiWriteRead(NOP, false); // NOP needs to be first byte before extended command can be set

  //this is the time required between bytes as specified in the datasheet.
  delayMicroseconds(3); 
  
  spiWriteRead(ZERO, true);
  delay(250); //250 second delay to allow the encoder to reset
}

void resetAMT22(){
  spiWriteRead(NOP, false); // NOP needs to be first byte before extended command can be set

  //this is the time required between bytes as specified in the datasheet.
  delayMicroseconds(3); 
  
  spiWriteRead(RESET, true);
  
  delay(250); //250 second delay to allow the encoder to start back up
}

uint16_t getPositionSPI(){
  uint16_t currentPosition;       //16-bit response from encoder
  bool binaryArray[16];           //after receiving the position we will populate this array and use it for calculating the checksum

  //get first byte which is the high byte, shift it 8 bits. don't release line for the first byte
  currentPosition = spiWriteRead(NOP, false) << 8;   

  //this is the time required between bytes as specified in the datasheet.
  delayMicroseconds(3);

  //OR the low byte with the currentPosition variable. release line after second byte
  currentPosition |= spiWriteRead(NOP, true);
  
  //run through the 16 bits of position and put each bit into a slot in the array so we can do the checksum calculation
  for(int i = 0; i < 16; i++) binaryArray[i] = (0x01) & (currentPosition >> (i));

  //using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
  if ((binaryArray[15] == !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
          && (binaryArray[14] == !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
    {
      //we got back a good position, so just mask away the checkbits
      currentPosition &= 0x3FFF;
    }
  else
  {
    currentPosition = 0xFFFF; //bad position
  }

  return currentPosition;
}

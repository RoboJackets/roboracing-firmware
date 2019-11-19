
// Notes:
// Need to connect VDC GND and dir- to common GND
// Need to have feedback connected to function
// 15.3 (gear ratio) * 1600 = pulses per revolution
// make sure encoder switch is set to run mode

byte dirPin = 3;
byte pulsePin = 6;
byte commandInterruptPin = 2;
float desiredAngle;
float currentAngle;

#define BAUDRATE        115200

/* SPI commands */
#define NOP       0x00   // no operation, encoder just sends data back
#define RESET     0x60
#define ZERO      0x70

/* Define special ascii characters */
#define NEWLINE         0x0A
#define TAB             0x09

/* SPI pins */
#define ENC_0            2
#define SPI_MOSI        11
#define SPI_MISO        12
#define SPI_SCLK        13

void setup() {
  /* Initialization for encoder*/
  pinMode(SPI_SCLK, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);
  pinMode(ENC_0, OUTPUT);
  SPI.setClockDivider(SPI_CLOCK_DIV32);    // 500 kHz
  digitalWrite(ENC_0, HIGH); // encoder is active low
  SPI.begin();

  /* Initialization for stepper*/
  Serial.begin(BAUDRATE);
  pinMode(dirPin, OUTPUT);
  pinMode(pulsePin, OUTPUT);
  pinMode(commandInterruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(commandInterruptPin), commandInterrupt, FALLING);
  digitalWrite(dirPin, LOW); // HIGH: clockwise, LOW: counter clockwise
  timer1_counter = 34286;    // preload timer 
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  delayMicroseconds(30);
}
 
void loop() {
  currentAngle = spiWriteRead();    
  Serial.print(currentAngle);     // keep sending encoder data back to NUC
}

ISR(TIMER1_OVF_VECTOR){                  // timer interrupt to move stepper motor
  TCNT1 = timer1_counter;          // preload timer
  if (desiredAngle != currentAngle){
    digitalWrite(pulsePin, HIGH);
    delayMicroseconds(20);
    digitalWrite(pulsePin, LOW);
    delayMicroseconds(20);
  }
}

void commandInterrupt(){
  if(Serial.read() == '$') {
    desiredAngle = Serial.parseFloat(); 
    desiredAngle = constrain(desiredSteeringAngle, minSteeringAngle, maxSteeringAngle);
  }
  currentAngle = spiWriteRead();         // read current position from encoder
  if (desiredAngle < currentAngle){      // set dirPIN to CW or CCW
    digitalWrite(dirPin, LOW);
  } else {
    digitalWrite(dirPIN, HIGH);
  }
}

void setCSLine (uint8_t csLine)
{
  digitalWrite(ENC_0, csLine);
}


uint8_t spiWriteRead(uint8_t sendByte, uint8_t releaseLine)
{
  uint8_t data;

  //set cs low, cs may already be low but there's no issue calling it again except for extra time
  setCSLine(ENC_0,LOW);

  //There is a minimum time requirement after CS goes low before data can be clocked out of the encoder.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);

  //send the command  
  data = SPI.transfer(sendByte);
  delayMicroseconds(3); //There is also a minimum time after clocking that CS should remain asserted before we release it
  setCSLine(ENC_0, releaseLine); //if releaseLine is high set it high else it stays low
  
  return data;
}

void setZeroSPI()
{
  spiWriteRead(NOP, false); // NOP needs to be first byte before extended command can be set

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3); 
  
  spiWriteRead(ZERO, true);
  delay(250); //250 second delay to allow the encoder to reset
}

void resetAMT22()
{
  spiWriteRead(NOP, false); // NOP needs to be first byte before extended command can be set

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3); 
  
  spiWriteRead(RESET, true);
  
  delay(250); //250 second delay to allow the encoder to start back up
}

uint16_t getPositionSPI()
{
  uint16_t currentPosition;       //16-bit response from encoder
  bool binaryArray[16];           //after receiving the position we will populate this array and use it for calculating the checksum

  //get first byte which is the high byte, shift it 8 bits. don't release line for the first byte
  currentPosition = spiWriteRead(NOP, false) << 8;   

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
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

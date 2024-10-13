/**
 * This sketch programs the microcode EEPROMs for the 8-bit breadboard computer
 * It includes support for a flags register with carry and zero flags
 * See this video for more: https://youtu.be/Zg1NdPKoosU
 */
#define SHIFT_DATA 2
#define SHIFT_CLK 3
#define SHIFT_LATCH 4
#define EEPROM_D0 5
#define EEPROM_D7 12
#define WRITE_EN 13

enum opcode {
  NOP, 
  LDA = 1 << 4, 
  ADD = 2 << 4, 
  SUB = 3 << 4, 
  STA = 4 << 4, 
  LDI = 5 << 4, 
  JMP = 6 << 4, 
  JC = 7 << 4, 
  JZ = 8 << 4, 
  OUT = 14 << 4, 
  HLT = 15 << 4
};

// each program is 16 bytes
// each instruction is one byte: 4 bit opcode (4 msb) and operand (4 lsb)

// select program by tying address lines beginning at A4 to VCC or GND

#define NPROGRAMS 4
const uint8_t program[NPROGRAMS * 16] = 
{
  // alternatingly count up by 1, down by 1
  LDI | 1,
  STA | 15,
  OUT,
  ADD | 15,
  JC | 6,
  JMP | 2,
  SUB | 15,
  OUT,
  JZ | 2,
  JMP | 6,
  NOP,
  NOP,
  NOP,
  NOP,
  NOP,
  NOP,

  // fibonacci; see https://www.youtube.com/watch?v=a73ZXDJtU48
  // note because program is in ROM we can put variables x, y, z elsewhere in memory
  LDI | 1,
  STA | 0xe,  // y = 1
  LDI | 0,    // x = 0
  OUT,        // print x
  ADD | 0xe,  
  STA | 0xf,  // z = x + y
  LDA | 0xe,
  STA | 0xd,  // x = y
  LDA | 0xf,
  STA | 0xe,  // y = z
  LDA | 0xd,  // load x
  JC | 0,
  JMP | 0x3,
  NOP, // 0xd: x; NB: these are in RAM
  NOP, // 0xe: y
  NOP, // 0xf: z

  // multiply two numbers; see https://youtu.be/Zg1NdPKoosU?si=16P1ZqmxfTjQaq4K&t=2376
  LDA | 14,
  SUB | 12, // x - 1 (if x > 0, computing x - 1 will set carry bit since subtracting 1 is the same as adding 255) 
  JC  | 6,  // if x > 0 goto 6 (why not use JZ?)
  LDA | 13, // output product
  OUT,
  HLT,
  STA | 14, // x = x - 1
  LDA | 13, // 
  ADD | 15, //
  STA | 13, // product = product + y
  JMP | 0,  // loop
  NOP,
  NOP, // 1; NB: these are in RAM; in our ROM program we could afford to LDI 1, STA 12 at start
  NOP, // product
  NOP, // x, the number of times to add y
  NOP, // y

  NOP,
  NOP,
  NOP,
  NOP,
  NOP,
  NOP,
  NOP,
  NOP,
  NOP,
  NOP,
  NOP,
  NOP,
  NOP,
  NOP,
  NOP,
  NOP,

};

/*
 * Output the address bits and outputEnable signal using shift registers.
 */
void setAddress(int address, bool outputEnable) {
  shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, (address >> 8) | (outputEnable ? 0x00 : 0x80));
  shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, address);

  digitalWrite(SHIFT_LATCH, LOW);
  digitalWrite(SHIFT_LATCH, HIGH);
  digitalWrite(SHIFT_LATCH, LOW);
}


/*
 * Read a byte from the EEPROM at the specified address.
 */
byte readEEPROM(int address) {
  for (int pin = EEPROM_D0; pin <= EEPROM_D7; pin += 1) {
    pinMode(pin, INPUT);
  }
  setAddress(address, /*outputEnable*/ true);

  byte data = 0;
  for (int pin = EEPROM_D7; pin >= EEPROM_D0; pin -= 1) {
    data = (data << 1) + digitalRead(pin);
  }
  return data;
}


/*
 * Write a byte to the EEPROM at the specified address.
 */
void writeEEPROM(int address, byte data) {
  setAddress(address, /*outputEnable*/ false);
  for (int pin = EEPROM_D0; pin <= EEPROM_D7; pin += 1) {
    pinMode(pin, OUTPUT);
  }

  for (int pin = EEPROM_D0; pin <= EEPROM_D7; pin += 1) {
    digitalWrite(pin, data & 1);
    data = data >> 1;
  }
  digitalWrite(WRITE_EN, LOW);
  delayMicroseconds(1);
  digitalWrite(WRITE_EN, HIGH);
  delay(10);
}


/*
 * Read the contents of the EEPROM and print them to the serial monitor.
 */
void printContents(int start, int length) {
  for (int base = start; base < length; base += 16) {
    byte data[16];
    for (int offset = 0; offset <= 15; offset += 1) {
      data[offset] = readEEPROM(base + offset);
    }

    char buf[80];
    sprintf(buf, "%03x:  %02x %02x %02x %02x %02x %02x %02x %02x   %02x %02x %02x %02x %02x %02x %02x %02x",
            base, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
            data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);

    Serial.println(buf);
  }
}


void setup() {
  // put your setup code here, to run once:

  pinMode(SHIFT_DATA, OUTPUT);
  pinMode(SHIFT_CLK, OUTPUT);
  pinMode(SHIFT_LATCH, OUTPUT);
  digitalWrite(WRITE_EN, HIGH);
  pinMode(WRITE_EN, OUTPUT);
  Serial.begin(57600);

  // Program data bytes
  Serial.print("Programming EEPROM");

  for (int address = 0; address < NPROGRAMS*16; address++) {

    uint8_t data = program[address];

    writeEEPROM(address, data);

    if (address % 64 == 0) {
      Serial.print(".");
    }
  }

  Serial.println(" done");

  // Read and print out the contents of the EERPROM
  Serial.println("Reading EEPROM");
  printContents(0, NPROGRAMS*16);
}


void loop() {
  // put your main code here, to run repeatedly:

}


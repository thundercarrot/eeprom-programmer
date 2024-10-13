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

#define HLT 0b100000000000000000000000  // Halt clock
#define MI  0b010000000000000000000000  // Memory address register in
#define RI  0b001000000000000000000000  // RAM data in
#define RO  0b000100000000000000000000  // RAM data out
#define IO  0b000010000000000000000000  // Instruction register out
#define II  0b000001000000000000000000  // Instruction register in
#define AI  0b000000100000000000000000  // A register in
#define AO  0b000000010000000000000000  // A register out
#define EO  0b000000001000000000000000  // ALU out
#define SU  0b000000000100000000000000  // ALU subtract
#define BI  0b000000000010000000000000  // B register in
#define OI  0b000000000001000000000000  // Output register in
#define CE  0b000000000000100000000000  // Program counter enable
#define CO  0b000000000000010000000000  // Program counter out
#define J   0b000000000000001000000000  // Jump (program counter in)
#define FI  0b000000000000000100000000  // Flags in
#define ZS  0b000000000000000010000000  // Zero step
#define PO  0b000000000000000001000000  // ROM out


#define FLAGS_Z0C0 0
#define FLAGS_Z0C1 1
#define FLAGS_Z1C0 2
#define FLAGS_Z1C1 3

#define JC  0b0111
#define JZ  0b1000

const uint32_t ucode[16][8] = {
  { MI|CO,  PO|II|CE|ZS,  0,      0,      0,           0, 0, 0 },   // 0000 - NOP
  { MI|CO,  PO|II|CE,  IO|MI,  RO|AI|ZS,  0,           0, 0, 0 },   // 0001 - LDA
  { MI|CO,  PO|II|CE,  IO|MI,  RO|BI,  EO|AI|FI|ZS,    0, 0, 0 },   // 0010 - ADD
  { MI|CO,  PO|II|CE,  IO|MI,  RO|BI,  EO|AI|SU|FI|ZS, 0, 0, 0 },   // 0011 - SUB
  { MI|CO,  PO|II|CE,  IO|MI,  AO|RI|ZS,  0,           0, 0, 0 },   // 0100 - STA
  { MI|CO,  PO|II|CE,  IO|AI|ZS,  0,      0,           0, 0, 0 },   // 0101 - LDI
  { MI|CO,  PO|II|CE,  IO|J|ZS,   0,      0,           0, 0, 0 },   // 0110 - JMP
  { MI|CO,  PO|II|CE|ZS,  0,      0,      0,           0, 0, 0 },   // 0111 - JC
  { MI|CO,  PO|II|CE|ZS,  0,      0,      0,           0, 0, 0 },   // 1000 - JZ
  { MI|CO,  PO|II|CE|ZS,  0,      0,      0,           0, 0, 0 },   // 1001
  { MI|CO,  PO|II|CE|ZS,  0,      0,      0,           0, 0, 0 },   // 1010
  { MI|CO,  PO|II|CE|ZS,  0,      0,      0,           0, 0, 0 },   // 1011
  { MI|CO,  PO|II|CE|ZS,  0,      0,      0,           0, 0, 0 },   // 1100
  { MI|CO,  PO|II|CE|ZS,  0,      0,      0,           0, 0, 0 },   // 1101
  { MI|CO,  PO|II|CE,  AO|OI|ZS,  0,      0,           0, 0, 0 },   // 1110 - OUT
  { MI|CO,  PO|II|CE,  HLT|ZS,    0,      0,           0, 0, 0 },   // 1111 - HLT
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

  for (int address = 0; address < 2048; address++) { // 11 bits address
    int byte_sel2   = (address & 0b10000000000) >> 10;
    int ZF          = (address & 0b01000000000) >> 9;
    int CF          = (address & 0b00100000000) >> 8;
    int byte_sel    = (address & 0b00010000000) >> 7;
    int instruction = (address & 0b00001111000) >> 3;
    int step        = (address & 0b00000000111);

    uint32_t control = ucode[instruction][step];

    // char buf[200];
    // sprintf(buf, "instruction = %d, step = %d, control = %03x", instruction, step, control);
    // Serial.println(buf);

    // patch for flags
    if (CF && (instruction == JC) && (step == 1))
      control = PO|II|CE;
    if (CF && (instruction == JC) && (step == 2))
      control = IO|J|ZS;

    if (ZF && (instruction == JZ) && (step == 1))
      control = PO|II|CE;
    if (ZF && (instruction == JZ) && (step == 2))
      control = IO|J|ZS;

    byte data = 0;
    if (byte_sel2)
      data = control;
    else if (byte_sel)
      data = control >> 8;
    else
      data = control >> 16;

    //data = address;
    writeEEPROM(address, data);

    if (address % 64 == 0) {
      Serial.print(".");
    }
  }

  Serial.println(" done");


  // Read and print out the contents of the EERPROM
  Serial.println("Reading EEPROM");
  printContents(0, 2048);
}


void loop() {
  // put your main code here, to run repeatedly:

}


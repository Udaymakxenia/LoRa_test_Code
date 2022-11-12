#include <SX126x.h>
#include <EEPROM.h>
uint8_t sf_arr[8] = {5, 6, 7, 8, 9, 10, 11, 12};
float bw_arr[10] = {7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 12, 250, 500};
uint8_t sf;
uint8_t bw;
void(* resetFunc) (void) = 0;
SX126x LoRa;
int8_t nssPin = 10, resetPin = 9, busyPin = 4, irqPin = 2, txenPin = -1, rxenPin = -1;

// Message to transmit
char message[] = "HeLoRa World!";
uint8_t nBytes = sizeof(message);
uint32_t counter = 0;

void setup() {

  // Begin serial communication
  Serial.begin(115200);
  sf = EEPROM.read(0);
  bw = EEPROM.read(1);
  sf = EEPROM.read(0);
  Serial.print("SF is changed to: ");
  Serial.println(sf_arr[sf]);
  bw = EEPROM.read(1);
  Serial.print("Bandwidth is changed to: ");
  Serial.println((uint32_t)bw_arr[bw] * 1000);
  // Begin LoRa radio and set NSS, reset, busy, IRQ, txen, and rxen pin with connected arduino pins
  // IRQ pin not used in this example (set to -1). Set txen and rxen pin to -1 if RF module doesn't have one
  Serial.println("Begin LoRa radio");
  if (!LoRa.begin(nssPin, resetPin, busyPin, irqPin, txenPin, rxenPin)) {
    Serial.println("Something wrong, can't begin LoRa radio");
    while (1);
  }

  // uncomment code below to use XTAL
  uint8_t xtalA = 0x12;
  uint8_t xtalB = 0x12;
  LoRa.setXtalCap(xtalA, xtalB);

  // Optionally configure DIO2 as RF switch control
  // This is usually used for a LoRa module without TXEN and RXEN pins
  LoRa.setDio2RfSwitch(true);

  // Set frequency to 915 Mhz
  Serial.println("Set frequency to 868 Mhz");
  LoRa.setFrequency(868000000);

  // Set TX power, default power for SX1262 and SX1268 are +22 dBm and for SX1261 is +14 dBm
  // This function will set PA config with optimal setting for requested TX power
  Serial.println("Set TX power to +17 dBm");
  LoRa.setTxPower(22, SX126X_TX_POWER_SX1262);                        // TX power +17 dBm for SX1262

  // Configure modulation parameter including spreading factor (SF), bandwidth (BW), and coding rate (CR)
  // Receiver must have same SF and BW setting with transmitter to be able to receive LoRa packet
  uint8_t SF = sf_arr[sf];                                                     // LoRa spreading factor: 7
  uint32_t BW = bw_arr[bw] * 1000;                                             // Bandwidth: 125 kHz
  uint8_t cr = 8;                                                     // Coding rate: 4/5
  LoRa.setLoRaModulation(SF, BW, cr, true);

  // Configure packet parameter including header type, preamble length, payload length, and CRC type
  // The explicit packet includes header contain CR, number of byte, and CRC type
  // Receiver can receive packet with different CR and packet parameters in explicit header mode
  uint8_t headerType = SX126X_HEADER_EXPLICIT;                        // Explicit header mode
  uint16_t preambleLength = 12;                                       // Set preamble length to 12
  uint8_t payloadLength = 17;                                         // Initialize payloadLength to 15
  bool crcType = true;                                                // Set CRC enable
  LoRa.setLoRaPacket(headerType, preambleLength, payloadLength, crcType);

  // Set syncronize word for public network (0x3444)
  LoRa.setSyncWord(0x3445);

  Serial.println("\n-- LORA TRANSMITTER --\n");
  Serial.print("Set BW is :");
  Serial.println(BW);
  Serial.print("Set SF is :");
  Serial.println(SF);

}

void loop() {
  if (Serial.available()) {
    sf = Serial.read() - 48;
    Serial.println(sf);
    EEPROM.write(0, sf);
    while (!Serial.available());
    bw = Serial.read() - 48;
    Serial.println(bw);
    EEPROM.write(1, bw);
    resetFunc(); //call reset
  }
  // Transmit message and counter
  // write() method must be placed between beginPacket() and endPacket()
  LoRa.beginPacket();
  LoRa.write(message, nBytes);
  uint8_t cntrbytes[4];
  for ( char i = 0; i < 4; i++) {
    cntrbytes[i] = (uint8_t) (counter >> 8 * i) ;
  }

  LoRa.write(cntrbytes, 4);
  LoRa.endPacket();
  LoRa.wait();

  // Print message and counter in serial
  Serial.print(message);
  Serial.print("  ");
  Serial.println(counter++);

  // Wait until modulation process for transmitting packet finish

  // Print transmit time
  Serial.print("Transmit time: ");
  Serial.print(LoRa.transmitTime());
  Serial.println(" ms");
  Serial.println();

  // Don't load RF module with continous transmit
  delay(2000);

}

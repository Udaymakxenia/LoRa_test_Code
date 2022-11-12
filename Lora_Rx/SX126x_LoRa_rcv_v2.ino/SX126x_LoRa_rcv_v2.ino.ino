#include <SX126x.h>
#include <EEPROM.h>
uint8_t sf_arr[8] = {5, 6, 7, 8, 9, 10, 11, 12};
float bw_arr[10] = {7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 12, 250, 500};
float avg_snr = 0;
float avg_rssi = 0;
uint8_t sf;
uint8_t bw;
void(* resetFunc) (void) = 0;
SX126x LoRa;
int curr_data;
int last_data = 0;
int pckt_lost = 0;
int temp_lost = 0;
int first = 0;
uint16_t avg_counter;
uint32_t pckt_loss[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint32_t pckt_loss_avg = 0;
uint8_t packtpoint = 0;
void setup() {

  // Begin serial communication
  Serial.begin(115200);
  sf = EEPROM.read(0);
  bw = EEPROM.read(1);
  Serial.print("SF: ");
  Serial.println(sf_arr[sf]);
  Serial.print("BW: ");
  Serial.println(bw_arr[bw]);
  while (!Serial.available());
  if (Serial.available()) {
    sf = Serial.read() - 48;
    Serial.println(sf);
    EEPROM.write(0, sf);
    while (!Serial.available());
    bw = Serial.read() - 48;
    Serial.println(bw);
    EEPROM.write(1, bw);
  }
  sf = EEPROM.read(0);
  bw = EEPROM.read(1);
  Serial.print("Set SF is: ");
  Serial.println(sf_arr[sf]);
  Serial.print("Set Bandwidth is: ");
  Serial.println(bw_arr[bw]);

  // Begin LoRa radio and set NSS, reset, busy, txen, and rxen pin with connected arduino pins
  // IRQ pin not used in this example (set to -1). Set txen and rxen pin to -1 if RF module doesn't have one
  Serial.println("Begin LoRa radio");
  int8_t nssPin = 10, resetPin = 9, busyPin = 4, irqPin = 2, txenPin = -1, rxenPin = -1;
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
  LoRa.setFrequency(868000000);

  // Set RX gain. RX gain option are power saving gain or boosted gain
  LoRa.setRxGain(SX126X_RX_GAIN_BOOSTED);                        // Power saving gain

  // Configure modulation parameter including spreading factor (SF), bandwidth (BW), and coding rate (CR)
  // Transmitter must have same SF and BW setting so receiver can receive LoRa packet
  uint8_t SF = sf_arr[sf];                                                     // LoRa spreading factor: 7
  uint32_t BW = bw_arr[bw] * 1000;                                             // Bandwidth: 125 kHz
  uint8_t cr = 8;                                                     // Coding rate: 4/5
  LoRa.setLoRaModulation(SF, BW, cr, true);

  // Configure packet parameter including header type, preamble length, payload length, and CRC type
  // The explicit packet includes header contain CR, number of byte, and CRC type
  // Packet with explicit header can't be received by receiver with implicit header mode
  uint8_t headerType = SX126X_HEADER_EXPLICIT;                        // Explicit header mode
  uint16_t preambleLength = 128;                                       // Set preamble length to 12
  uint8_t payloadLength = 17;                                         // Initialize payloadLength to 15
  bool crcType = true;                                                // Set CRC enable
  LoRa.setLoRaPacket(headerType, preambleLength, payloadLength, crcType);

  // Set syncronize word for public network (0x3444)
  LoRa.setSyncWord(0x3444);

  Serial.println("\n-- LORA RECEIVER --\n");

}

void loop() {
  int rssii;
  float snrr;
  // Request for receiving new LoRa packet
  LoRa.request();
  // Wait for incoming LoRa packet
  LoRa.wait();

  // Put received packet to message and counter variable
  // read() and available() method must be called after request() or listen() method
  const uint8_t msgLen = LoRa.available();
  Serial.print("MsgLen :");
  Serial.print(msgLen);
  uint8_t message[msgLen];
  uint32_t counter;
  // available() method return remaining received payload length and will decrement each read() or get() method called
  uint8_t i = 0;
  while (LoRa.available() >= 1) {
    message[i++] = LoRa.read();
  }
  uint32_t cntrbytesval[4];
  counter = 0;
  for (uint8_t i = 0; i < 4; i++) {
    cntrbytesval[i] = ((uint32_t)message[msgLen - 4 + i]) << 8 * i;
    counter += cntrbytesval[i];
  }
  if (first == 0)
  {
    first = counter;
    last_data = counter - 1;
  }
  temp_lost = counter - last_data - 1 ;
  last_data = counter;
  pckt_loss[packtpoint] = temp_lost;
  if (packtpoint >= 10) {
    packtpoint = 0;
  }
  else
    packtpoint += 1;
  pckt_lost = pckt_lost + temp_lost;
  avg_counter += 1;
  pckt_loss_avg = 0;
  for (uint8_t i = 0; i < 10; i++) {              //Sum of array of packet loss of last 10 packets
    pckt_loss_avg = pckt_loss_avg + pckt_loss[i];
  }
  pckt_loss_avg = pckt_loss_avg / 10;             //Avg of last 10 packets

  // Print received message and counter in serial

  Serial.print("; Ctr = ");
  Serial.print(counter);
  Serial.print("; PktLst = ");
  Serial.print(pckt_lost);
  //  Serial.print("; AvgPktLst = ");
  Serial.print(pckt_loss_avg);
  // Print packet/signal status including package RSSI and SNR
  rssii = LoRa.packetRssi();
  Serial.print("; RSSI = ");
  Serial.print(rssii);
  avg_rssi = (avg_rssi + rssii) / avg_counter;
  snrr = LoRa.snr();
  Serial.print("; SNR = ");
  Serial.println(snrr);
  avg_snr = (avg_snr + snrr) / avg_counter;

  // Show received status in case CRC or header error occur
  uint8_t status = LoRa.status();
  if (status == SX126X_STATUS_CRC_ERR) Serial.println("CRC error");
  else if (status == SX126X_STATUS_HEADER_ERR) Serial.println("Packet header error");

}

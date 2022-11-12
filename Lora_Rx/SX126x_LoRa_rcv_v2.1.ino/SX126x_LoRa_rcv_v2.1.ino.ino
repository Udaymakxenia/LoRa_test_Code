#include <SX126x.h>
#include <EEPROM.h>
#include <SD.h>
#include <SPI.h>
File Log;
int8_t nssPin = 10, resetPin = 9, busyPin = 4, irqPin = 2, txenPin = -1, rxenPin = -1;
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
uint8_t SF;
uint32_t BW;
const uint8_t SD_CS = 5;
void setup() {
  // Begin serial communication
  Serial.begin(115200);
  pinMode(SD_CS, OUTPUT);
  sf = EEPROM.read(0);
  bw = EEPROM.read(1);
  Serial.print("SF in EEPROM is  : ");
  Serial.println(sf_arr[sf]);
  Serial.print("BW in EEPROM is: ");
  Serial.println((uint32_t)bw_arr[bw] * 1000);
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
  Serial.println("\n-- LORA RECEIVER --\n");
  SF = sf_arr[sf];                                                     // LoRa spreading factor: 7
  BW = bw_arr[bw] * 1000;                                             // Bandwidth: 125 kHz
  Serial.print("Set BW is :");
  Serial.println(BW);
  Serial.print("Set SF is :");
  Serial.println(SF);
  String SF_BW = (String)("**********") + (String)("SF: ") + (String)SF + (String)(" ") + (String)("BW: ") + (String)BW + (String)("**********");
  SD_Card(SF_BW);

}

void loop() {
  String datalog = "";
  LoRa_initialise();
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
  Serial.print("; AvgPktLst = ");
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
  digitalWrite(nssPin, LOW);                                                          //Disable LoRa
}
void LoRa_initialise(void) {
  // Begin LoRa radio and set NSS, reset, busy, txen, and rxen pin with connected arduino pins
  // IRQ pin not used in this example (set to -1). Set txen and rxen pin to -1 if RF module doesn't have one
  if (!LoRa.begin(nssPin, resetPin, busyPin, irqPin, txenPin, rxenPin)) {
    Serial.println("Something wrong, can't begin LoRa radio");
    while (1);
  }
  // uncomment code below to use XTAL
  uint8_t xtalA = 0x12;
  uint8_t xtalB = 0x12;
  LoRa.setXtalCap(xtalA, xtalB);


  // Set frequency to 868 Mhz
  LoRa.setFrequency(868000000);

  // Set RX gain. RX gain option are power saving gain or boosted gain
  LoRa.setRxGain(SX126X_RX_GAIN_BOOSTED);                        // Power saving gain
  uint8_t cr = 8;                                                     // Coding rate: 4/5
  LoRa.setLoRaModulation(SF, BW, cr);

  uint8_t headerType = SX126X_HEADER_EXPLICIT;                        // Explicit header mode
  uint16_t preambleLength = 128;                                       // Set preamble length to 12
  uint8_t payloadLength = 17;                                         // Initialize payloadLength to 15
  bool crcType = true;                                                // Set CRC enable
  LoRa.setLoRaPacket(headerType, preambleLength, payloadLength, crcType);

  // Set syncronize word for public network (0x3444)
  LoRa.setSyncWord(0x3444);
}
void SD_Card(String Str) {
  if (SD.begin())
  {
    Serial.println("SD card is ready to use.");
  } else
  {
    Serial.println("SD card initialization failed");
    return;
  }
  Log = SD.open("Logs.txt", FILE_WRITE);
  if (Log) {
    Log.println(Str);
    Log.close(); // close the file
  }
  else {
    Serial.println("error opening Logs.txt");
  }
  delay(3000);
  digitalWrite(SD_CS, LOW);
}

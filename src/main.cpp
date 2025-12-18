#include <Arduino.h>
#include <pubsubclient.h>
#include <WiFi.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <SPI.h>
#include <MFRC522.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
// MPU6050 accelgyro(0x69); // <-- use for AD0 high
// MPU6050 accelgyro(0x68, &Wire1); // <-- use for AD0 low, but 2nd Wire (TWI/I2C) object

int16_t ax, ay, az;
int16_t gx, gy, gz;

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// Pins utilisées pour la carte RFID
#define RST_PIN 9
#define SS_PIN 10
MFRC522 mfrc522(SS_PIN, RST_PIN);

// Pins utilisées pour le capteur cardiaque
#define CRD_PLUS_PIN 7
#define CRD_MINUS_PIN 6
#define CRD_ANA A2

// LED de couleur
#define GREEN_LED_PIN 2
#define RED_LED_PIN 3

static unsigned long lastDebugPublishTime = 0;

// Timing pour échantillonnage rapide
unsigned long lastECGSampleTime = 0;
const unsigned long ecgSampleInterval = 10; // 10ms = 100Hz échantillonnage

// Filtrage du signal
const int BUFFER_SIZE = 10;
int ecgBuffer[BUFFER_SIZE];
int bufferIndex = 0;
int ecgFiltered = 0;
int ecgPrevious = 0;

// Détection de battements
unsigned long lastBeat = 0;
const unsigned long MIN_BEAT_INTERVAL = 300;  // 300ms = max 200 BPM
const unsigned long MAX_BEAT_INTERVAL = 2000; // 2000ms = min 30 BPM
bool beatDetected = false;
int peakValue = 0;

// Seuil adaptatif
int signalMax = 0;
int signalMin = 1023;
int threshold = 512;
unsigned long lastThresholdUpdate = 0;

// Calcul BPM moyenné
const int BPM_BUFFER_SIZE = 5;
int bpmBuffer[BPM_BUFFER_SIZE];
int bpmBufferIndex = 0;
int bpmSum = 0;
int bpmCount = 0;
int averageBPM = 0;

// Publication MQTT ECG (toutes les 2 secondes comme avant)
unsigned long lastECGPublishTime = 0;
const unsigned long ecgPublishInterval = 2000;

// Paramètres Wifi obligatoires
// A remplacer par votre propre connexion
const char *ssid = "";
const char *password = "";

// MQTT Broker settings
// TODO : Update with your MQTT broker settings here if needed
const char *mqtt_broker = "broker.emqx.io"; // EMQX broker endpoint
const char *mqtt_mac_address = "homeTrainerCastres/Group3-C/MAC-Address";
const char *mqtt_gyro_ax = "homeTrainerCastres/Group3-C/Gyro-AX";
const char *mqtt_gyro_ay = "homeTrainerCastres/Group3-C/Gyro-AY";
const char *mqtt_gyro_az = "homeTrainerCastres/Group3-C/Gyro-AZ";
const char *mqtt_gyro_gx = "homeTrainerCastres/Group3-C/Gyro-GX";
const char *mqtt_gyro_gy = "homeTrainerCastres/Group3-C/Gyro-GY";
const char *mqtt_gyro_gz = "homeTrainerCastres/Group3-C/Gyro-GZ";
const char *mqtt_rfid_info = "homeTrainerCastres/Group3-C/RFID-Info";
const char *mqtt_rfid_disconnect = "Disconnect";
const char *mqtt_heartbeat_instant = "homeTrainerCastres/Group3-C/Heartbeat-Instant";
const char *mqtt_heartbeat_average = "homeTrainerCastres/Group3-C/Heartbeat-Average";
const char *mqtt_heartbeat_debug = "homeTrainerCastres/Group3-C/Heartbeat-Debug";
const int mqtt_port = 1883; // MQTT port (TCP)
String client_id = "ArduinoClient-";
String MAC_address = "";
boolean cardDetected = false;

// Other global variables
static unsigned long lastPublishTime = 0;
WiFiClient espClient;
PubSubClient mqtt_client(espClient);

void connectToWiFi();
void connectToMQTTBroker();
void mqttCallback(char *topic, byte *payload, unsigned int length);

void setup()
{
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(9600);
  connectToWiFi();
  mqtt_client.setServer(mqtt_broker, mqtt_port);
  mqtt_client.setCallback(mqttCallback);
  connectToMQTTBroker();

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  while (!Serial)
    ;
  SPI.begin();
  mfrc522.PCD_Init();
  mfrc522.PCD_DumpVersionToSerial(); // Output details of the reader
  Serial.println(F("Scan PICC to see UID, type, and data blocks..."));

  pinMode(CRD_PLUS_PIN, INPUT);  // Setup for leads off detection LO +
  pinMode(CRD_MINUS_PIN, INPUT); // Setup for leads off detection LO -

  pinMode(RED_LED_PIN, OUTPUT);   // Setup red LED
  pinMode(GREEN_LED_PIN, OUTPUT); // Setup green LED
}

void printMacAddress()
{
  byte mac[6];
  Serial.print("MAC Address: ");
  WiFi.macAddress(mac);
  for (int i = 0; i < 6; i++)
  {
    MAC_address += String(mac[i], HEX);
    if (i < 5)
      MAC_address += ":";
    if (mac[i] < 16)
    {
      client_id += "0";
    }
    client_id += String(mac[i], HEX);
  }
  Serial.println(MAC_address);
  // Publish message upon successful connection
  String message = "Hello EMQX I'm " + client_id;
  mqtt_client.publish(mqtt_mac_address, message.c_str());
}

void connectToWiFi()
{
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  delay(3000);
  printMacAddress();
  Serial.println("Connected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
}

void connectToMQTTBroker()
{
  while (!mqtt_client.connected())
  {
    Serial.print("Connecting to MQTT Broker as ");
    Serial.print(client_id.c_str());
    Serial.println(".....");
    if (mqtt_client.connect(client_id.c_str()))
    {
      Serial.println("Connected to MQTT broker");
      mqtt_client.subscribe(mqtt_rfid_disconnect);
    }
    else
    {
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// Callback via Node-RED, comme la seule utilisation du callback est pour la carte RFID, l'utilisateur est déconnecté dès
// qu'un signal est détecté
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  Serial.println("Déconnexion en cours");
  String messageDeconnexion = "Aucun utilisateur connecté";
  mqtt_client.publish(mqtt_rfid_info, messageDeconnexion.c_str());
  cardDetected = false;
}

// ECG : Filtrage du signal (moyenne mobile)
int filterECGSignal(int rawValue)
{
  ecgBuffer[bufferIndex] = rawValue;
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

  long sum = 0;
  for (int i = 0; i < BUFFER_SIZE; i++)
  {
    sum += ecgBuffer[i];
  }
  return sum / BUFFER_SIZE;
}

// ECG : Mise à jour du seuil adaptatif
void updateThreshold(int value)
{
  if (value > signalMax)
    signalMax = value;
  if (value < signalMin)
    signalMin = value;

  // Recalculer le seuil toutes les 2 secondes
  if (millis() - lastThresholdUpdate > 2000)
  {
    int range = signalMax - signalMin;
    threshold = signalMin + (range * 6) / 10; // 60% entre min et max

    // Reset progressif pour adaptation continue
    signalMax = (signalMax * 9 + value) / 10;
    signalMin = (signalMin * 9 + value) / 10;

    lastThresholdUpdate = millis();

    Serial.print("Seuil: ");
    Serial.print(threshold);
    Serial.print(" | Range: [");
    Serial.print(signalMin);
    Serial.print(" - ");
    Serial.print(signalMax);
    Serial.println("]");
  }
}

// ECG : Calcul BPM moyenné
int calculateAverageBPM(int newBPM)
{
  // Ignorer les valeurs aberrantes
  if (newBPM < 30 || newBPM > 200)
  {
    return averageBPM; // Garder la valeur précédente
  }

  // Ajouter au buffer
  if (bpmCount < BPM_BUFFER_SIZE)
  {
    bpmSum += newBPM;
    bpmBuffer[bpmBufferIndex] = newBPM;
    bpmCount++;
  }
  else
  {
    bpmSum = bpmSum - bpmBuffer[bpmBufferIndex] + newBPM;
    bpmBuffer[bpmBufferIndex] = newBPM;
  }

  bpmBufferIndex = (bpmBufferIndex + 1) % BPM_BUFFER_SIZE;
  return bpmSum / bpmCount;
}

// ECG : Calcul de la fréquence cardiaque
void processECG()
{
  unsigned long currentTime = millis();

  // Échantillonnage rapide (100Hz)
  if (currentTime - lastECGSampleTime >= ecgSampleInterval)
  {
    lastECGSampleTime = currentTime;

    // Lecture et filtrage
    int ecgRaw = analogRead(CRD_ANA);
    ecgFiltered = filterECGSignal(ecgRaw);

    // Mise à jour du seuil adaptatif
    updateThreshold(ecgFiltered);

    // Détection de battement
    unsigned long timeSinceLastBeat = currentTime - lastBeat;

    // Détection de montée (crossing threshold)
    if (!beatDetected && ecgFiltered > threshold && ecgPrevious <= threshold)
    {
      if (timeSinceLastBeat > MIN_BEAT_INTERVAL)
      {
        beatDetected = true;
        peakValue = ecgFiltered;
      }
    }

    if (ecgFiltered > peakValue)
    {
      peakValue = ecgFiltered;
    }
    // Détection de descente (fin du pic)
    int instantBPM = 60000 / timeSinceLastBeat;
    averageBPM = calculateAverageBPM(instantBPM);

    Serial.print(">>> BATTEMENT! BPM instant: ");
    Serial.print(instantBPM);
    Serial.print(" | BPM moyen: ");
    Serial.println(averageBPM);

    String instantBPMMessage = (String)instantBPM;
    String averageBPMMessage = (String)averageBPM;
    mqtt_client.publish(mqtt_heartbeat_instant, instantBPMMessage.c_str());
    mqtt_client.publish(mqtt_heartbeat_average, averageBPMMessage.c_str());

    lastBeat = currentTime;
    beatDetected = false;
  }

  ecgPrevious = ecgFiltered;
}

/**
 * @return the card UID or 0 when an error occurred
 */
unsigned long getID()
{
  unsigned long hex_num;
  hex_num = mfrc522.uid.uidByte[0] << 24;
  hex_num += mfrc522.uid.uidByte[1] << 16;
  hex_num += mfrc522.uid.uidByte[2] << 8;
  hex_num += mfrc522.uid.uidByte[3];
  mfrc522.PICC_HaltA(); // Stop reading
  return hex_num;
}

void loop()
{
  if (!mqtt_client.connected())
  {
    connectToMQTTBroker();
  }
  mqtt_client.loop();

  // A chaque loop on vérifie si une carte a été détectée
  if (!cardDetected)
  {
    String messageDeconnexion = "Aucun utilisateur connecté";
    mqtt_client.publish(mqtt_rfid_info, messageDeconnexion.c_str());
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, 0);

    // Si détection d'une nouvelle carte, l'id est publiée et cardDetected devient vraie 
    if (!mfrc522.PICC_IsNewCardPresent())
    {
      return;
    }
    if (!mfrc522.PICC_ReadCardSerial())
    {
      return;
    }
    mfrc522.PICC_DumpToSerial(&(mfrc522.uid));
    unsigned long uid = getID();
    if (uid != 0)
    {
      String uidUser = (String)uid;
      mqtt_client.publish(mqtt_rfid_info, uidUser.c_str());
    }
    cardDetected = true;
  }
  else
  {
    // L'utilisateur est connecté : on lit les données de l'ECG et du gyroscope
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(RED_LED_PIN, 0);

    if ((digitalRead(CRD_PLUS_PIN) == 1) || (digitalRead(CRD_MINUS_PIN) == 1))
    {
      Serial.println('!');
    }
    else
    {
      unsigned long currentTime = millis();

      // send the value of analog input 0:
      Serial.println(analogRead(A0));
      if (analogRead(A0) > 0)
      {
        processECG();
      }
      // Message de debug toutes les 10s
      if (currentTime - lastDebugPublishTime >= 10000)
      {
        String debugMsg = "Alive " + String(currentTime / 1000) + "s | ECG BPM: " + String(averageBPM);
        mqtt_client.publish(mqtt_heartbeat_debug, debugMsg.c_str());
        lastDebugPublishTime = currentTime;
      }
    }
    // Wait for a bit to keep serial data from saturating
    delay(1);

    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    // accelgyro.getAcceleration(&ax, &ay, &az);
    // accelgyro.getRotation(&gx, &gy, &gz);

#ifdef OUTPUT_READABLE_ACCELGYRO
    // display tab-separated accel/gyro x/y/z values
    Serial.print("a/g:\t");
    Serial.print(ax);
    Serial.print("\t");
    Serial.print(ay);
    Serial.print("\t");
    Serial.print(az);
    Serial.print("\t");
    Serial.print(gx);
    Serial.print("\t");
    Serial.print(gy);
    Serial.print("\t");
    Serial.println(gz);
    String message_ax = String(ax);
    String message_ay = String(ay);
    String message_az = String(az);
    String message_gx = String(gx);
    String message_gy = String(gy);
    String message_gz = String(gz);

    // Publication des données du gyroscope sur Node-RED
    mqtt_client.publish(mqtt_gyro_ax, message_ax.c_str());
    mqtt_client.publish(mqtt_gyro_ay, message_ay.c_str());
    mqtt_client.publish(mqtt_gyro_az, message_az.c_str());
    mqtt_client.publish(mqtt_gyro_gx, message_gx.c_str());
    mqtt_client.publish(mqtt_gyro_gy, message_gy.c_str());
    mqtt_client.publish(mqtt_gyro_gz, message_gz.c_str());
#endif

#ifdef OUTPUT_BINARY_ACCELGYRO
    Serial.write((uint8_t)(ax >> 8));
    Serial.write((uint8_t)(ax & 0xFF));
    Serial.write((uint8_t)(ay >> 8));
    Serial.write((uint8_t)(ay & 0xFF));
    Serial.write((uint8_t)(az >> 8));
    Serial.write((uint8_t)(az & 0xFF));
    Serial.write((uint8_t)(gx >> 8));
    Serial.write((uint8_t)(gx & 0xFF));
    Serial.write((uint8_t)(gy >> 8));
    Serial.write((uint8_t)(gy & 0xFF));
    Serial.write((uint8_t)(gz >> 8));
    Serial.write((uint8_t)(gz & 0xFF));
#endif
  }
}
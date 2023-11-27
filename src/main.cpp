// data in: {"mac":"64:B7:08:CA:18:DC","name":"marshall","timeSleep":30}
// data out:{"type":"tank","name":"marshall","mac":"64:B7:08:CA:18:DC","count":1,"value":76.7715}

#include <ArduinoJson.h>
#include <Average.h>

/* ============================== SENSORS ============================== */

#include "HX711.h"

HX711 scale;

uint8_t dataPin = 5;
uint8_t clockPin = 18;

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

const int16_t read_n_max = 1000; // 1500 // max samples number

const float alpha_a = 0.1; // alpha filter for acc
const float alpha_l = 0.1; // alpha filter for load

unsigned long startTime, endTime;

// Diagnosis tips
float v_diagnosis[10];
double fill;
double spm = 0.0;

/* ============================== SIM7000G CONFIGURATIONS ============================== */
#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 4096 // Set RX buffer to 1Kb

#define SerialMon Serial
#define SerialAT Serial1
#define TINY_GSM_DEBUG SerialMon
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
#define GSM_PIN ""

const char apn[]      = "claro.pe";
const char gprsUser[] = "";
const char gprsPass[] = "";

#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

TinyGsmClient client(modem);

// LilyGO T-SIM7000G Pinout
#define UART_BAUD           115200
#define PIN_DTR             25
#define PIN_TX              27
#define PIN_RX              26
#define PWR_PIN             4

#define LED_PIN             12

/* ============================== WIFI CONFIGURATIONS ============================== */
#include <WiFi.h>
/*
#include <WiFi.h>
const char *ssid = "fly01";
const char *password = "blackmon2023";
WiFiClient client;
*/

/* ============================== MQTT CONFIGURATIONS ============================== */

#include <PubSubClient.h>
PubSubClient mqtt(client);
//const char *broker = "broker.emqx.io";
const char *broker = "broker.hivemq.com"; // < ================================= SERVER BROKER
const int mqtt_port = 1883;
const char *mqtt_user = "web_client";
const char *mqtt_pass = "080076C";
const char *mqtt_id = "kjljihiuklfff";
const char *topicSubscribe = "jphOandG/mexdevice"; // < ================================= TOPIC
const char *topicPublish = "jphOandG/mexdata";

/* ====================== DEVICE SETTINGS ======================== */
#define uS_TO_S_FACTOR 1000000
int TIME_TO_SLEEP = 900; // 5 minutes sleep

//#define BUTTON_PIN_BITMASK 0x1000000000 // 2^36 in hex
RTC_DATA_ATTR unsigned int bootCount = 0; // data counter
boolean Available = true; // false: only level / true: level and dynachart
String DeviceName = "well2";
String idTest;

String status = ""; // global status
int WaitTime = 150;
unsigned long previousMillis = 0;

float a0[50] = {-0.161094, 0.133767, 0.509254, 0.586077, 0.588004, 0.572422, 0.577633, 0.567187, 0.58284, 0.574648, 0.567952, 0.564416, 0.566411, 0.591799, 0.592944, 0.584578, 0.607816, 0.609944, 0.606361, 0.607037, 0.625326, 0.663207, 0.661868, 0.597974, 0.369072, 0.179431, -0.0484264, -0.113182, -0.097836, -0.0759193, -0.0531394, -0.0452247, -0.0245321, -0.0140658, -0.010772, -0.0177663, -0.0145408, -0.00956059, -0.0108517, 0.00724792, 0.0163409, 0.0166782, 0.0187881, 0.0292116, 0.0331711, 0.0459051, 0.0554916, -0.0497948, -0.202742, -0.291714};

void mqttCallback(char *topic, byte *payload, unsigned int length);

/* ====================== SIM FUNCTIONS ======================== */
void modemPowerOn(){
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(1000);
  digitalWrite(PWR_PIN, HIGH);  
}

void modemPowerOff(){
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(1500);
  digitalWrite(PWR_PIN, HIGH);
}

void modemRestart(){
  modemPowerOff();
  delay(1000);
  modemPowerOn();
}

/* ====================== RECONNECT ======================== */
void reconnect()
{
  int count = 0;
  while (!mqtt.connected())
  {
    //Serial.println("...try coinnecte");
    //if (mqtt.connect(mqtt_id, mqtt_user, mqtt_pass, topicPublish, 0, false, "Device conected"))
    if (mqtt.connect(mqtt_id))
    {
      String topicS = String(topicSubscribe) + "/#";
      mqtt.subscribe(topicS.c_str());
      //Serial.println(topicS);
      //mqtt.subscribe(topicSubscribe);
    }
    else
    {
      if (count == 5)
      {
        Serial.println("reset mqtt");
        modemRestart();
        ESP.restart();
      }
      delay(5000);
    }
    count++;
  }
}

/* ============================== SETUP GRPS ============================== */
boolean setup_grps()
{
  Serial.println("gprs stup");

  modemPowerOn();

  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(3000);

  if (!modem.init()) {
    modemRestart();
    delay(2000);
    Serial.println("Failed to restart modem, attempting to continue without restarting");
    return false;
  }

  String modemInfo = modem.getModemInfo();
  Serial.println(modemInfo);

  if (modemInfo == "")
  {
    Serial.println("modem fail");
    return false;
  }
  /* */
  if (!modem.waitForNetwork(240000L))
  {
    Serial.println("nt fail");
    return false;
  }
  
  if (modem.isNetworkConnected())
  {
    Serial.println("connected");
  }
  else
  {
    return false;
  }

  if (!modem.gprsConnect(apn))
  //if (!modem.gprsConnect(apn, gprsUser, gprsPass))
  {
    Serial.println("grps fail");
    return false;
  }
  else
  {
    Serial.println("grps ok"); // sim  grps ok
    return true;
  }
}

/* ============================== NEURAL NETWORKS FUNTIONS ============================== */
float relu(float n)
{
  if (n >= 0)
    return n;
  else if (n < 0)
    return 0;
}

float sigmoid(float n)
{
  return 1.0 / (1.0 + exp(-n));
}

/* ============================== MAP FUNCTION ============================== */
float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

/* ============================== NEURAL NETWORK FOR DIAGNOSIS ============================== */
void diagnosis()
{
  /* NN variables */
  // float a0[50];

  //Serial.println("--------a");
  float W1[15][50] = {{-0.665,-0.618,-0.007,0.067,-0.158,-0.053,0.093,0.248,-0.03,0.53,0.254,-0.104,0.244,0.254,0.487,0.121,0.479,0.383,0.409,0.185,0.587,0.493,0.282,-0.13,0.322,0.386,0.371,-0.194,-0.373,-0.686,-0.351,-0.127,0.278,0.047,0.287,0.349,-0.013,0.169,0.394,1.148,1.182,1.415,0.364,-0.439,-0.717,-0.295,-0.351,0.387,1.054,0.735},{-0.608,-0.306,-0.135,0.904,-0.184,-0.358,-0.417,-0.97,-0.074,0.055,0.432,0.411,0.099,0.081,-0.035,0.097,0.162,-0.178,-0.079,0.41,-0.21,0.264,0.556,0.006,0.233,-0.247,-0.171,0.04,0.903,0.85,0.068,-0.403,0.114,0.286,0.065,-0.341,-0.528,-0.35,-0.126,-0.465,-1.132,-0.642,-0.528,0.556,1.327,1.251,0.288,-0.056,-1.015,-1.187},{-0.161,0.119,0.09,0.041,-0.248,-0.068,-0.298,0.14,0.03,0.261,0.364,-0.122,0.232,0.581,0.199,0.026,0.551,0.142,-0.021,0.004,0.164,-0.04,0.582,0.27,0.173,0.461,0.113,-0.328,-0.501,-0.348,-0.389,-0.238,-0.045,0.011,0.177,-0.522,-0.089,-0.074,0.097,-0.019,-0.575,-0.155,0.001,-0.013,0.815,0.426,0.194,-0.454,-1.346,-0.838},{-1.106,-0.07,-0.403,-0.232,0.187,-0.006,0.302,0.005,0.202,0.012,0.168,0.229,0.077,0.04,0.399,0.246,0.324,0.208,0.457,0.424,0.013,0.052,0.05,-0.179,0.01,-0.019,0.273,0.094,-0.592,-0.081,0.03,-0.302,-0.038,-0.162,-0.462,0.193,-0.474,0.035,-0.152,0.016,0.542,0.348,0.002,-0.012,-1.171,-0.781,-0.247,0.363,1.231,0.58},{-0.099,0.424,-0.204,-0.457,0.067,0.523,0.354,0.326,0.336,0.412,0.424,0.198,-0.055,0.548,0.08,0.541,0.208,0.184,0.239,-0.16,0.149,0.137,0.049,-0.398,0.081,-0.39,0.138,0.218,0.471,-0.03,0.192,-0.079,-0.003,-0.116,-0.015,0.002,0.118,0.059,0.18,0.245,0.473,0.541,0.713,1.017,0.959,-0.275,0.16,-0.946,-0.818,-1.245},{0.665,-0.154,0.335,-0.077,0.312,0.288,-0.026,-0.331,0.242,0.21,0.065,0.115,-0.092,-0.119,-0.039,0.305,-0.178,0.033,-0.23,-0.179,0.326,0.06,0.062,-0.088,0.339,0.236,-0.006,-0.039,0.684,0.246,0.821,0.308,-0.289,0.574,0.268,0.567,0.497,0.299,-0.343,-0.527,0.187,-0.194,-0.894,-0.967,-0.715,-0.031,-0.071,1.115,1.272,1.472},{-0.376,0.569,-0.434,-0.512,-0.361,0.228,0.178,0.487,0.134,0.228,0.274,0.412,-0.143,0.258,-0.055,0.175,0.203,0.315,0.022,0.187,-0.273,-0.04,0.194,-0.214,0.124,0.081,0.343,-0.328,-0.106,-0.119,-0.221,0.163,-0.381,0.126,0.044,0.191,0.015,0.085,0.173,0.765,1.548,1.05,1.327,0.479,-0.455,-0.363,0.251,-1.001,-0.574,-0.88},{-1.49,-0.317,0.072,-0.132,-0.327,-0.498,0.465,-0.094,0.111,-0.101,-0.297,-0.356,0.41,0.193,0.163,-0.14,0.196,0.153,0.353,0.094,0.519,0.246,-0.038,0.202,-0.495,-0.231,0.06,-0.993,-1.315,-1.028,-0.056,0.476,0.259,0.356,-0.291,0.372,0.203,0.485,0.219,-0.217,0.038,0.247,0.008,-0.199,-1.054,-1.005,-0.852,0.083,0.888,-0.265},{0.019,0.175,-0.157,0.39,-0.124,-0.465,-0.32,-0.447,-0.219,-0.341,0.268,-0.174,0.191,-0.207,0.128,0.231,-0.202,0.107,-0.119,-0.058,-0.198,-0.216,0.124,-0.234,-0.174,-0.223,0.271,0.352,0.053,0.795,0.448,0.886,0.243,0.727,0.685,0.246,1.007,0.608,0.758,0.204,-0.21,-0.165,-0.086,0.674,0.571,0.132,0.135,-0.443,-0.6,-0.135},{-0.251,-0.131,0.622,0.841,0.299,-0.152,-0.378,-0.508,-0.296,-0.948,-0.869,-0.444,-0.118,0.164,0.114,-0.175,0.01,0.434,0.59,0.299,0.447,0.237,0.358,0.341,0.204,-0.272,-0.008,0.064,-0.018,0.901,0.841,-0.63,-1.338,-1.959,-2.413,-1.12,-0.051,-0.602,-0.381,-0.356,-0.599,-0.633,-0.834,-0.268,-0.378,-0.567,-0.476,0.273,1.39,0.406},{0.842,0.412,0.227,0.073,0.205,0.241,-0.053,-0.255,-0.395,-0.381,0.235,-0.289,0.144,-0.196,-0.222,-0.312,0.167,0.271,0.421,-0.184,0.133,0.25,-0.256,0.142,0.104,-0.13,0.391,0.051,0.558,0.705,0.457,0.473,-0.324,-0.344,-0.157,-0.174,-0.171,-0.15,-0.554,-0.287,1.529,1.104,0.711,0.673,-0.374,-0.242,-0.654,0.466,1.359,0.617},{0.76,-0.249,-0.12,0.419,-0.025,-0.579,0.216,-0.212,-0.156,-0.015,-0.05,0.432,-0.238,0.183,0.242,-0.021,-0.002,0.113,0.201,0.023,0.014,0.169,0.312,0.247,-0.056,0.135,-0.004,0.007,0.574,0.462,0.286,0.277,0.246,0.499,0.61,0.159,0.22,0.04,0.633,0.292,-0.395,-0.425,-0.607,-0.717,-0.421,-0.716,-0.603,-0.137,0.862,1.008},{0.156,-0.01,-0.296,-0.013,-0.412,0.174,0.019,-0.371,0.118,-0.01,0.129,-0.416,-0.297,0.128,-0.025,-0.422,-0.358,0.213,0.176,0.032,0.234,0.284,0.217,0.375,-0.101,0.082,-0.371,0.102,0.135,0.04,-0.18,-0.081,-0.009,-0.345,0.051,-0.058,0.091,-0.182,-0.259,-0.048,-0.189,0.131,-0.177,0.053,-0.126,0.121,0.067,0.019,-0.08,-0.152},{0.042,-0.553,0.368,0.632,0.541,0.065,-0.24,-0.089,0.134,0.08,-0.364,0.041,0.11,-0.352,0.02,0.124,-0.137,-0.399,0.035,0.054,-0.186,0.49,0.605,0.298,0.721,0.473,0.402,0.098,0.303,0.399,0.108,0.453,0.194,0.009,-0.08,-0.345,-0.38,-0.237,-0.267,-0.543,-0.719,-0.893,-1.63,-1.652,-0.649,0.441,-0.36,1.206,1.622,1.762},{0.276,0.229,0.282,0.318,0.624,0.082,0.139,0.738,0.54,0.436,0.222,-0.348,-0.196,-0.578,-0.321,-0.679,-0.849,-0.486,-0.609,-0.414,-0.6,-0.412,-0.091,-0.173,-0.5,-0.068,0.588,0.076,-0.036,0.19,0.422,-0.528,-0.64,-0.169,-0.525,-0.367,-0.743,-0.622,-1.089,-0.833,-0.428,-0.571,-1.231,-0.996,-0.87,-0.001,-0.338,0.365,-0.065,-0.847}};
  float a1[15];
  float W2[10][15] = {{0.449,-0.577,0.694,0.656,0.033,-0.58,0.186,1.735,-1.684,-0.556,0.025,-0.383,-0.32,-0.083,-1.59},{-0.302,-0.243,0.178,-0.058,-0.114,-0.449,-0.694,-0.476,-0.133,-1.245,0.086,-0.016,0.328,-0.619,-0.55},{-0.098,-0.659,-0.546,0.197,-0.542,-0.132,0.162,-0.605,0.057,-0.198,-0.112,-0.154,0.181,-0.378,-0.906},{-0.335,-0.843,-0.127,-0.428,-0.488,-0.526,-0.403,-0.931,-0.118,-0.677,-0.528,-0.578,-0.271,-0.086,-0.51},{-0.771,0.376,-0.637,0.189,0.041,-0.028,-1.285,-1.278,0.182,1.208,1.067,-0.006,-0.191,0.33,-2.253},{-0.163,0.651,0.659,-1.294,-0.723,-0.077,-1.053,0.193,0.533,-1.554,-2.058,0.638,-0.228,0.504,-2.301},{-0.334,-1.489,-0.617,-0.171,0.784,-0.246,0.459,0.464,0.225,-0.684,1.395,0.267,-0.177,-1.121,-0.885},{-1.241,0.576,0.528,-0.097,-0.006,-0.269,0.314,-2.552,0.656,-0.08,0.09,-0.591,0.262,-0.355,-0.27},{0.079,-0.198,0.227,-0.641,-0.164,0.189,-0.477,-1.408,-1.596,-1.651,-0.321,-0.276,0.27,0.241,0.973},{-0.275,1.414,0.865,-0.927,0.75,-0.976,0.175,-0.888,0.203,-0.892,-1.281,-0.286,-0.256,-1.092,-1.81}};
  float a2[10]; 
  float b1[15]= {-0.239,1.094,0.358,-0.568,0.015,0.238,-0.145,1.516,1.057,1.409,-0.233,0.156,-0.067,0.319,1.625};
  float b2[10]= {-0.478,-0.21,-0.207,-0.259,-0.115,0.281,-0.565,0.024,-0.101,0.437};
  float aux = 0.0;

  //Serial.println("--------b");

  /* ***** Neural network running ***** */
  for(int i = 0 ; i<15; i++ ) {aux=0.0;for(int j = 0 ; j <50 ; j++ ) { aux=aux+W1[i][j]*a0[j];} a1[i]=relu(aux+b1[i]);}
  for(int i = 0 ; i<10; i++ ) {aux=0.0;for(int j = 0 ; j <15 ; j++ ) { aux=aux+W2[i][j]*a1[j];} a2[i]=sigmoid(aux+b2[i]);}


  for (int i = 0; i < 10; i++)
  {
    v_diagnosis[i] = a2[i];
  }
}

/* ============================== NEURAL NETWORK FOR FILL PUMP ============================== */
void fillPump()
{
  /* NN variables */
  float W1[15][50] = {{0.097,0.204,0.412,-0.062,-0.456,-0.222,0.209,0.255,-0.414,0.323,-0.205,0.131,-0.173,-0.375,-0.199,0.027,0.391,0.483,0.255,0.312,0.443,-0.063,-0.112,-0.08,0.428,-0.233,-0.198,-0.003,-0.222,0.045,0.022,0.185,-0.076,-0.278,-0.141,0.009,-0.05,0.215,-0.181,0.283,-0.367,0.176,0.255,-0.121,0.032,-0.126,-0.189,-0.141,-0.373,-0.232},{0.522,0.634,0.116,0.806,0.089,-0.024,0.126,-0.389,-0.217,-0.427,0.047,-0.259,-0.679,-0.07,-0.766,-0.605,-0.447,-0.363,-0.556,-0.416,-0.672,-0.654,-0.696,-0.655,-0.606,-0.544,-0.095,-0.396,-0.645,-0.029,-0.487,-0.411,-0.393,-0.461,-0.721,-0.481,-0.446,-0.566,-0.423,0.132,0.342,-0.487,-0.291,-0.412,-0.327,-0.588,-0.296,-0.545,-0.192,-0.191},{-0.356,-0.036,0.466,-0.358,0.087,-0.077,-0.067,0.208,0.03,0.042,-0.114,0.113,-0.248,-0.028,0.089,0.079,0.25,-0.081,0.192,0.339,0.226,0.413,-0.183,0.021,0.269,0.284,0.115,-0.342,-0.006,-0.071,-0.134,-0.117,-0.096,0.302,-0.377,-0.443,0.094,0.14,-0.191,-0.083,-0.16,-0.103,-0.57,-0.179,-0.229,0.298,-0.285,0.286,-0.185,0.065},{-0.457,-0.061,-0.258,-0.065,-0.096,-0.055,0.217,-0.17,-0.278,0.133,0.037,-0.418,-0.21,0.065,-0.16,0.451,0.238,0.15,0.493,0.087,0.324,0.096,-0.283,0.434,0.123,0.046,0.419,0.248,-0.002,-0.008,-0.075,-0.138,0.372,0.004,-0.144,-0.099,-0.038,0.181,-0.044,-0.069,-0.449,0.032,-0.04,-0.005,-0.299,0.294,-0.203,0.236,-0.127,0.278},{0.135,0.32,0.001,0.341,0.136,-0.047,-0.011,0.172,-0.042,0.036,0.092,-0.053,0.291,-0.4,-0.038,0.068,0.142,-0.214,-0.12,-0.068,0.119,0.263,-0.032,0.17,-0.079,-0.059,0.285,-0.013,0.2,0.424,0.061,0.231,-0.142,-0.085,-0.334,-0.033,-0.037,0.321,0.287,0.182,-0.071,0.428,0.045,0.054,-0.037,-0.187,0.011,-0.447,0.162,-0.15},{-0.031,0.367,0.597,-0.237,0.245,-0.037,-0.2,-0.556,-0.291,-0.511,-0.138,0.528,0.285,0.314,-0.072,0.14,-0.055,0.095,0.302,-0.209,0.226,0.159,-0.026,0.387,-0.121,-0.089,-0.035,-0.006,-0.643,-0.238,-0.486,-0.069,-0.534,-0.497,-0.948,-0.214,-0.031,-0.061,-0.227,0.051,-0.66,0.575,0.42,0.402,0.17,-0.03,0.122,-0.659,-0.381,-0.537},{0.027,0.058,-0.128,0.036,0.083,-0.055,-0.341,0.027,0.078,0.066,0.091,0.047,-0.071,0.015,-0.219,-0.147,-0.151,0.007,-0.164,0.016,0.117,-0.066,-0.161,-0.27,-0.337,-0.342,0.152,0.049,0.149,0.049,0.156,-0.454,0.133,0.058,-0.291,0.203,0.198,0.122,-0.065,0.122,0.02,0.063,0.179,0.155,0.245,0.14,0.297,0.233,0.233,0.106},{-0.244,0.3,-0.37,0.084,-0.021,-0.229,0.021,0.007,-0.519,-0.195,0.313,0.299,0.328,-0.118,0.253,-0.034,-0.007,-0.194,0.051,0.325,-0.057,-0.066,0.307,-0.011,0.219,-0.141,-0.317,0.142,-0.031,-0.081,0.155,-0.018,0.116,0.207,0.007,-0.012,-0.019,0.146,-0.152,0.203,-0.072,-0.02,0.143,-0.139,-0.121,-0.358,0.25,-0.039,-0.242,-0.42},{-0.128,-0.116,0.103,-0.143,-0.412,0.263,0.267,-0.043,0.313,0.026,-0.439,-0.027,-0.081,0.301,0.067,-0.002,0.068,0.212,0.007,0.06,0.326,0.361,-0.17,0.136,0.04,-0.288,-0.415,0.05,0.041,-0.057,-0.142,0.172,0.312,-0.128,-0.268,-0.251,0.17,0.143,-0.081,0.083,0.206,0.222,0.07,-0.128,-0.118,-0.163,-0.593,0.063,-0.145,0.155},{0.196,0.221,0.552,0.673,0.314,0.128,-0.232,0.049,-0.629,0.094,-0.673,-0.677,-0.586,-0.722,-0.392,-0.3,-0.17,-0.45,-0.282,-0.426,-0.67,-0.056,-0.679,-0.503,-0.396,-0.572,-0.555,-0.347,-0.736,-0.331,-0.409,-0.766,-0.333,-0.513,-0.188,-0.24,-0.485,-0.286,0.094,-0.057,0.825,-0.187,-0.099,-0.739,-0.502,-0.27,-0.211,-0.547,-0.205,0.001},{0.03,-0.052,0.152,-0.052,0.034,0.021,-0.219,-0.116,0.141,-0.282,0.155,0.089,-0.235,0.135,0.367,-0.05,-0.235,-0.022,-0.22,-0.422,0.067,0.148,-0.143,0.258,0.022,-0.061,0.287,-0.05,-0.114,-0.307,-0.413,-0.243,0.044,0.194,0.382,0.24,0.015,0.105,-0.112,-0.025,-0.127,0.073,0.219,0.112,0.071,0.13,0.192,0.182,0.345,-0.218},{0.143,-0.476,-0.059,-0.202,0.041,-0.071,0.253,0.041,0.073,-0.06,0.134,-0.264,-0.098,-0.163,0.055,-0.199,0.172,-0.045,-0.082,0.398,-0.029,0.064,-0.297,-0.161,-0.21,-0.128,0.326,0.017,0.035,0.13,0.077,-0.333,-0.146,0.002,-0.125,0.084,0.298,-0.145,0.204,-0.04,0.066,0.232,-0.116,0.05,-0.121,0.473,-0.02,0.082,0.333,-0.004},{0.237,-0.039,-0.231,-0.147,-0.051,-0.466,0.184,-0.323,-0.241,0.021,-0.064,-0.13,-0.091,0.003,-0.4,0.227,0.329,0.064,0.116,0.14,-0.19,0.253,0.086,-0.228,-0.318,0.226,0.003,-0.376,0.182,0.128,-0.045,0.007,0.263,0.123,-0.133,-0.013,0.13,0.045,0.219,0.285,-0.285,-0.346,-0.003,-0.365,0.126,0.129,-0.213,0.206,0.229,0.257},{0.243,0.099,0.237,0.053,0.028,-0.274,-0.35,-0.087,-0.358,-0.111,0.02,-0.185,-0.11,0.185,-0.037,0.194,0.379,-0.416,0.16,0.212,0.216,-0.144,0.29,0.021,-0.104,0.23,0.022,0.389,-0.027,-0.007,0.058,0.098,-0.178,0.42,-0.123,0.108,0.113,-0.084,0.22,-0.071,0.306,0.151,-0.094,-0.094,-0.424,0.145,0.309,-0.603,-0.294,-0.068},{0.37,-0.2,-0.218,0.217,-0.273,0.117,0.497,-0.172,0.239,-0.383,0.446,-0.301,0.109,-0.277,0.341,0.378,0.016,-0.24,0.454,0.254,0.056,-0.203,-0.057,0.162,-0.066,-0.133,0.181,0.228,0.027,0.158,-0.166,-0.239,-0.488,0.131,-0.424,-0.219,0.029,0.217,0.006,-0.558,-0.268,-0.185,-0.418,0.195,0.735,0.195,-0.396,0.407,0.192,0.152}};
  float a1[15];
  float W2[1][15] = {{0.658,-0.49,0.355,0.354,-0.63,1.43,0.426,-0.358,0.223,-0.519,0.48,0.492,0.239,-0.472,0.535}};
  float a2[1]; 
  float b1[15]= {-0.149,0.324,0.003,-0.169,0.222,1.139,-0.117,0.123,-0.23,0.365,-0.019,-0.05,-0.048,0.211,0.114};
  float b2[1]= {-0.215};
  float aux = 0.0;

  /* ***** Neural network running ***** */
  for(int i = 0 ; i<15; i++ ) {aux=0.0;for(int j = 0 ; j <50 ; j++ ) { aux=aux+W1[i][j]*a0[j];} a1[i]=relu(aux+b1[i]);}
  for(int i = 0 ; i<1; i++ ) {aux=0.0;for(int j = 0 ; j <15 ; j++ ) { aux=aux+W2[i][j]*a1[j];} a2[i]=sigmoid(aux+b2[i]);}

  fill = a2[0];
}

/* ========== GET DATA FROM SENSORS =========== */
float acceleration(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  return a.acceleration.y; 
}

float loadHx711(){
  if (scale.is_ready())
  {
    return (scale.get_units(1));
  }
}

/* ==================== MAIN PROCESS ===================== */
void main_process(){
  
  Average<float> acc_raw(read_n_max);
  Average<float> load_raw(read_n_max);

  float f_load; // load raw filter
  float f_acc;  // acc raw filter
  float s_acc = 0;
  float s_load = 0;

  mpu.enableSleep(false);
  mpu.enableCycle(false);

  /* ------- data preread ------- */
  for (uint16_t i = 0; i < 100; i++)
  {
    s_acc += acceleration();
    s_load += loadHx711();
    delay(35);
  }

  f_load = s_load * 0.01;
  f_acc = s_acc * 0.01;

  /* ------- main data reading ------- */
  startTime = millis();

  for (uint16_t i = 0; i < read_n_max; i++)
  {
    
    f_acc = (alpha_a * acceleration()) + ((1 - alpha_a) * f_acc);
    f_load = (alpha_l * loadHx711()) + ((1 - alpha_l) * f_load);
    Serial.println(String(i) + "-> " +String(f_acc) + " - " + String(f_load));
    acc_raw.push(f_acc);
    load_raw.push(f_load);
    delay(35);
  }
  
  endTime = millis() - startTime;
  Serial.println(endTime);

  mpu.enableCycle(false);
  mpu.enableSleep(true);

  /* ----------------- Preprocessing ----------------- */
  float maxAcc = 0;
  float minAcc = 0;
  float maxLoad = 0;
  float minLoad = 0;

  int max_acc_index = 0;
  int min_acc_index = 0;
  int max_load_index = 0;
  int min_load_index = 0;

  maxAcc = acc_raw.maximum(&max_acc_index);
  minAcc = acc_raw.minimum(&min_acc_index);
  maxLoad = load_raw.maximum(&max_load_index);
  minLoad = load_raw.minimum(&min_load_index);

  Average<float> load(50);
  Average<float> pos(50);

  /* ----------------- ----------------- Main process service ----------------- ----------------- */
  /* ************ Detect pump stopped service ************ */
  if ((maxAcc - minAcc) <= 0.50)
  {
    status = "stopped";
    //Serial.println("stopped");
  }
  /*
  else if (abs(maxLoad - minLoad) <= 100.00)
  {
    status = "running";
    //Serial.println("rods broken");
  }
  */
  else{
    /* ************ Separe stroke ************ */
    status = "running";
    //Serial.println("Separe stroke");

    float value;
    int16_t i_start = 0;
    int16_t i_flag = 0;
    int16_t i_end = 0;
    float tp = 0;
    float diff;
    float range = maxAcc - minAcc;

    for (int16_t i = 0; i < read_n_max - 1; i++)
    {
      value = acc_raw.get(i);
      diff = value - minAcc;
      if (diff > 0.8 * range && i_flag == 0 && i_end == 0)
      {
        if (diff > tp)
        {
          tp = diff;
          i_start = i;
        }
      }
      else if (i_start != 0 && diff < 0.2 * range && i_end == 0)
      {
        if (diff < tp)
        {
          tp = diff;
          i_flag = i;
        }
      }
      else if (i_flag != 0 && diff > 0.8 * range)
      {
        if (diff > tp)
        {
          tp = diff;
          i_end = i;
        }
      }
      else if (diff < 0.2 * range && i_end != 0)
      {
        break;
      }
    }

    Serial.println("acc: " + String(minAcc) + "," + String(maxAcc) + "; mmin index " + String(i_start) + " min index: " + String(i_end));

    /* ************ detect stroke integrity ************ */
    if (i_start == 0 || i_end == 0)
    {
      Serial.println("Incomplete dynachart.");
      //Serial.println("------------------ end");
    }

    /* ************ resize array to 50 ************ */
    float length = float(i_end - i_start) / 49.00;

    //Average<float> load(50);
    //Average<float> pos(50);

    float temp;
    int index;
    for (int i = 0; i < 49; i++)
    {
      index = i_start + i * length;
      temp = mapfloat(load_raw.get(index),-40,30,6.5,0);
      //float t_load = load_raw.get(index);
      float t_pos = acc_raw.get(index);
      load.push(temp);
      pos.push(t_pos);
      //Serial.print(String(acc_raw.get(index)) + ",");
      // a0[i] = temp;
    }

    temp = mapfloat(load_raw.get(index),-40,30,6.5,0);
    load.push(temp);
    //Serial.println(String(acc_raw.get(i_end)));

    pos.push(acc_raw.get(i_end));
    // a0[49] = temp;

    /* ************ NN for diagnosis ************ */
    diagnosis();
    /*
    for (int i = 0; i < 10; i++)
    {
      Serial.print(String(v_diagnosis[i]) + ",");
    }
    */

    //Serial.println();

    /* ************ NN for fillpump ************ */
    fillPump();
    //Serial.println(String(fill));
  }

  /* ------- payload to send data by json ------- */
  String payload = "";
  StaticJsonDocument<2048> docOut;
  docOut["type"] = "analyzer";
  docOut["name"] = DeviceName;
  docOut["mac"] = WiFi.macAddress();
  docOut["count"] = bootCount;
  docOut["status"] = status;
  docOut["idTest"] = idTest;
  if (status == "stopped")
  {
    docOut["fill"] = 0;
    docOut["SPM"] = 0;
  }
  else
  {
    docOut["fill"] = serialized(String(fill,2));
    JsonArray diag = docOut.createNestedArray("diag");
    for (int8_t i = 0; i < 10; i++)
    {
      if (v_diagnosis[i] > 0.7) // tresholder 0.7
      {
        diag.add(i);
      }
    }
    
    JsonArray p = docOut.createNestedArray("p");
    JsonArray l = docOut.createNestedArray("l");

    for (int i = 0; i < 50; i++)
    {
      //Serial.print(i);
      //Serial.print(" ");
      //Serial.println(String(pos.get(i),2));

      p.add(pos.get(i));
      l.add(load.get(i));
      //l.add((float) ( load.get(i));

    }
  }

  serializeJson(docOut, payload);
  Serial.println(payload);

  setup_grps();
  //setup_wifi();
  mqtt.setServer(broker, mqtt_port);
  mqtt.setCallback(mqttCallback);
  reconnect();
  mqtt.publish(topicPublish, payload.c_str());

}

/* ====================== MQTT CALLBACK ======================== */
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  Serial.println(topic);
  if (String(topic) == String(topicSubscribe)+ "/start")
  {
    String msg_in = "";
    for (int i = 0; i < length; i++)
    {
      msg_in += String((char)payload[i]);
    }
    Serial.println(msg_in);

    StaticJsonDocument<200> docIn;
    DeserializationError error = deserializeJson(docIn, msg_in);
    if (error) {
      // Serial.print(F("deserializeJson() failed: "));
      //Serial.println(error.f_str());
      return;
    }
    //const char* Mac = docIn["mac"];
    const char* Name = docIn["name"];
    int Time = docIn["timeSleep"];
    const char* idtest = docIn["dt"];
    idTest = idtest;

    TIME_TO_SLEEP = Time - 40;
    DeviceName = String(Name);
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    
    main_process();
    /*
    if(String(Mac) == WiFi.macAddress()){
    TIME_TO_SLEEP = Time;
    DeviceName = String(Name);
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    
    main_process();
    }
    */

  }else if(String(topic) == String(topicSubscribe) + "/finish"){
    String msg_in = "";
    for (int i = 0; i < length; i++)
    {
      msg_in += String((char)payload[i]);
    }
    Serial.println(msg_in);

    StaticJsonDocument<200> docIn;
    DeserializationError error = deserializeJson(docIn, msg_in);
    if (error) {
      // Serial.print(F("deserializeJson() failed: "));
      //Serial.println(error.f_str());
      return;
    }
    const char* Name = docIn["name"];
    int Time = docIn["timeSleep"];
    const char* Mac = docIn["mac"];

    if(String(Mac) == WiFi.macAddress()){
      Serial.println("start deep sleep.......");
      TIME_TO_SLEEP = Time - 40;
      DeviceName = String(Name);
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

      gpio_hold_dis(GPIO_NUM_4);
      modemPowerOff();
      esp_deep_sleep_start();
    }
      
  }
}

/* ====================== SETUP HX711 ======================== */
void setup_HX711() {
  scale.begin(dataPin, clockPin);

  scale.set_scale(127.15);
  scale.tare(10);

}

/* ====================== SETUP ======================== */
void setup() {
  SerialMon.begin(115200);
  modemPowerOn();

  setup_HX711();

  if (!mpu.begin()) {
    Serial.println("mpu failed");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(false);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  ++bootCount;
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  //setup_grps();
  //Serial.println(WiFi.macAddress());
  //mqtt.setServer(broker, mqtt_port);
  //mqtt.setCallback(mqttCallback);
  
  //reconnect();
  //Serial.println(WiFi.macAddress());

  main_process();

}

void loop() {
  if (!mqtt.connected()) {
    reconnect();
  }
  mqtt.loop();
  delay(50);

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= WaitTime * 1000) {
    Serial.println("start deep sleep without response");
    gpio_hold_dis(GPIO_NUM_4);
    modemPowerOff();
    esp_deep_sleep_start();
  }
}
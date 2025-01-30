#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <SD.h>
#include <FS.h>



#define CS_PIN 5
#define MOSI_PIN 23
#define MISO_PIN 19
#define SCK_PIN 18


#define READINGS 4

#define training_acquisition 0

#if training_acquisition
  #define STATE_CONDITION_BTN 27
  volatile int road_state = 0;
#endif

const uint16_t OLED_Color_Black = 0x0000;
const uint16_t OLED_Color_Blue = 0x001F;
const uint16_t OLED_Color_Red = 0xF800;
const uint16_t OLED_Color_Green = 0x07E0;
const uint16_t OLED_Color_Cyan = 0x07FF;
const uint16_t OLED_Color_Magenta = 0xF81F;
const uint16_t OLED_Color_Yellow = 0xFFE0;
const uint16_t OLED_Color_White = 0xFFFF;

uint16_t OLED_Text_Color = OLED_Color_Yellow;
uint16_t OLED_Backround_Color = OLED_Color_Black;

const uint8_t OLED_pin_scl_sck = 14;
const uint8_t OLED_pin_sda_mosi = 13;
const uint8_t OLED_pin_cs_ss = 15;
const uint8_t OLED_pin_res_rst = 4;
const uint8_t OLED_pin_dc_rs = 16;

Adafruit_SSD1331 display = Adafruit_SSD1331(OLED_pin_cs_ss, OLED_pin_dc_rs, OLED_pin_sda_mosi, OLED_pin_scl_sck, OLED_pin_res_rst);

unsigned long update_display = 0;
unsigned long send_message = 0;
unsigned long time_log = 0;
unsigned long send_time = 0;
unsigned long last_sent = 0;
unsigned long gps_del = -5000;

float ax, ay, az, gx, gy, gz;
int tempo;
String success;
String filename = "/data_log.csv";
esp_now_peer_info peer_info[3];
int count = 0;

//SoftwareSerial gps_serial(12, 2);
HardwareSerial gps_serial(2);
TinyGPSPlus gps;

int satellites = 0;
float decimalLatitude = 0.0;
float decimalLongitude = 0.0;
bool fix = false;
int sent = 0;
int prev_value = 0;
volatile int value = 0;

uint8_t BROADCST_ADDRESS[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct struct_message {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    int distance;
    int tempo;
} struct_message;

struct_message incoming_readings[READINGS];
#define TFT_GRAY 0xBDF7

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    success = (status == ESP_NOW_SEND_SUCCESS) ? "Delivery Success :)" : "Delivery Fail :(";
}

volatile boolean recording = false;
//struct_message incomingReadings;

void start_recordings() {
    static unsigned long lastPress = millis();
    while (millis() - lastPress < 100);
    if(digitalRead(36) == LOW) {
        recording = !recording;
    }
}

int store_data(float latitude, float longitude) {
    if (!recording) {
        return 0;
    }
    File file = SD.open(filename, FILE_APPEND);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return -1;
    }
    #if training_acquisition
    road_state = digitalRead(STATE_CONDITION_BTN);
    #endif
    for (int i = 0; i <= READINGS-1; i++) {
        file.print(incoming_readings[i].tempo);
        if(i > 0) {
             Serial.println(incoming_readings[i].tempo - incoming_readings[i-1].tempo);
        }
        file.print(";");
        file.print(latitude, 6);
        file.print(";");
        file.print(longitude, 6);
        file.print(";");
        file.print(satellites);
        file.print(";");
        file.print(incoming_readings[i].distance);
        file.print(";");
        file.print(incoming_readings[i].ax);
        file.print(";");
        file.print(incoming_readings[i].ay);
        file.print(";");
        file.print(incoming_readings[i].az);
        file.print(";");
        file.print(incoming_readings[i].gx);
        file.print(";");
        file.print(incoming_readings[i].gy);
        file.print(";");
        file.print(incoming_readings[i].gz);
        file.print(";");
        #if training_acquisition
            file.print(road_state);
        #endif
        file.println();
    }
    file.close();
    return 1;
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    memcpy(&incoming_readings, incomingData, sizeof(incoming_readings));
    Serial.print("Bytes received: ");
    Serial.println(len);
    //tempo = incoming_readings[0].tempo;
    store_data(decimalLatitude, decimalLongitude);
    Serial.println(incoming_readings[0].ay);
}



void setup() {
  #if training_acquisition
    pinMode(STATE_CONDITION_BTN, INPUT);
  #endif

    update_display = millis();
    Serial.begin(9600);
    WiFi.mode(WIFI_STA);
    //gps_serial.begin(9600);
    gps_serial.begin(9600, SERIAL_8N1, 12, 2);
    display.begin();
    display.setFont();
    display.fillScreen(OLED_Backround_Color);
    display.setTextColor(OLED_Text_Color);
    display.setTextSize(1);

    if (esp_now_init()) {
        Serial.println("Não foi possível iniciar");
        return;
    } else {
        Serial.println("Iniciado com sucesso");
    }

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    memcpy(peer_info[0].peer_addr, BROADCST_ADDRESS, 6);
    peer_info[0].channel = 0;
    peer_info[0].encrypt = false;

    if (esp_now_add_peer(&peer_info[0]) != ESP_OK) {
        Serial.println("Falha ao adicionar peer");
        return;
    }
    Serial.println("Setup feito");

    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
    if (!SD.begin(CS_PIN)) {
        Serial.println("Card Mount Failed");
        return;
    } else {
        Serial.println("Card Mount Success");


        
    if (SD.exists(filename)) {
        int fileIndex = 1;
        String newFilename;
        while (SD.exists(newFilename = "/data_log(" + String(fileIndex) + ").csv")) {
            fileIndex++;
        }
        filename = newFilename;
    }

        File file = SD.open(filename, FILE_WRITE);
    
        if (file) { // Check if the file opened successfully
        #if training_acquisition
            file.println("Timestamp;Latitude;Longitude;N_Satelites;distance;ax;ay;az;gx;gy;gz;Road condition;");
        #else
            file.println("Timestamp;Latitude;Longitude;N_Satelites;distance;ax;ay;az;gx;gy;gz;");
            file.close();
        #endif
        }
    }

    

    // Use the updated filename for logging
    Serial.println("Logging to: " + filename);

    pinMode(36, INPUT_PULLUP);
    attachInterrupt(36, start_recordings, FALLING);

    
        if (!SD.begin()) {
            Serial.println("SD Card initialization failed!");
            return;
        }

        
        
    //}
}

void loop() {

    value = recording;
    //if(prev_value != value) {
        if(value != 0){
            value = 1;
        }
        if(millis()>= send_time + 500){
            esp_now_send(BROADCST_ADDRESS, (uint8_t*)&value, sizeof(int));
            send_time = millis();
        }
        
        
  //  }
   // Serial.println(recording);
   
if(millis() > 0){
while (gps_serial.available() > 0) {
    char gpsChar = gps_serial.read();
    gps.encode(gpsChar);
   // Serial.print(gpsChar);
    if (gps.location.isUpdated()) {
        decimalLatitude = gps.location.lat();
        decimalLongitude = gps.location.lng();
        fix = gps.location.isValid();
        satellites = gps.satellites.value();

      /*  Serial.print("Latitude: ");
        Serial.println(decimalLatitude, 6);
        Serial.print("Longitude: ");
        Serial.println(decimalLongitude, 6);
        Serial.print("Fix: ");
        Serial.println(fix ? "Yes" : "No");
        Serial.print("Satellites: ");
        Serial.println(satellites);*/
    }
}
}       

    

    if (millis() >= update_display + 1000) {
        display.fillScreen(OLED_Backround_Color);
        display.setCursor(0, 0);

        display.setTextColor(OLED_Color_Blue);
        display.print("Satellites: ");
        display.setTextColor(OLED_Color_Yellow);
        display.println(satellites);

        display.setTextColor(OLED_Color_Blue);
        display.print("Lat: ");
        display.setTextColor(OLED_Color_Yellow);
        display.println(decimalLatitude, 6);

        display.setTextColor(OLED_Color_Blue);
        display.print("Lon: ");
        display.setTextColor(OLED_Color_Yellow);
        display.println(decimalLongitude, 6);

        display.setTextColor(OLED_Color_Blue);
        display.print("Fix: ");
        display.setTextColor(fix ? OLED_Color_Green : OLED_Color_Red);
        display.println(fix ? "Yes" : "No");
        display.setTextColor(OLED_Color_Blue);
        display.print("Recording: ");
        display.setTextColor(OLED_Color_Red);
        display.setTextColor(recording ? OLED_Color_Green : OLED_Color_Red);
        display.println(recording ? "Yes" : "No");
        display.setTextColor(OLED_Color_Blue);
        display.print("Wifi:");
        if(WiFi.status() == WL_CONNECTED){
            display.setTextColor(OLED_Color_Green);
            display.println(" Conected");
            display.setTextColor(OLED_Color_White);
            display.println("");
            display.print(" ");
            display.println(WiFi.localIP());
        }
        else{
            display.setTextColor(OLED_Color_Red);
            display.println("Disconected");
            display.setTextColor(OLED_Color_White);
            display.println("");
            display.println(filename);
        }
        update_display = millis();
    }

   

   
}


/*
void loop() {
    while (gps_serial.available()) {
        char c = gps_serial.read();  // Read a character from the GPS
        gps.encode(c);               // Feed the character to TinyGPS++
        Serial.write(c);             // Optional: Output raw data for debugging
    }

   
}*/
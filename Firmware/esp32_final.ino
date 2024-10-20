#include <WiFi.h>
#include "time.h"

#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>

/* Set your Hotspot's/Router's SSID and Password */
/* Configure your hotspot/router to 2.4 GHz band, otherwise ESP32 won't be able to connect */
// WiFi AP SSID
#define WIFI_SSID "_SSID_"
// WiFi password
#define WIFI_PASSWORD "_PASSWORD_"

// UART pins for communication with MCU
#define RXD0 3
#define TXD0 1

// Write your API key in the INFLUXDB_TOKEN field
// Configure other InfluxDB fields as well
#define INFLUXDB_URL "https://us-east-1-1.aws.cloud2.influxdata.com"
#define INFLUXDB_TOKEN "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"
#define INFLUXDB_ORG "95b49947c1671f19"
#define INFLUXDB_BUCKET "SHM_EDL"

// Time zone info
#define TZ_INFO "IST-5"

// Declare InfluxDB client instance with preconfigured InfluxCloud certificate
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);

// Declare Data point
Point sensor("Temp_Hum_Acc");

// Instantaneous values acquired from sensors
char temp[20] = {0};
char hum[20] = {0};
char acc_x[20] = {0};
char acc_y[20] = {0};
char acc_z[20] = {0};
char temp_iis[20] = {0};
char time_[20] = {0};

// Above values are appended and sent to InfluxDB in CSV format
String time_array = "", acc_x_array = "", acc_y_array = "", acc_z_array = "";
time_t startTime;

// NOT USED due to innacurate values of DHT11
/* Read DHT11's temperature and humidity values */
void read_temp_hum(){
  int i = 0;
  while(Serial2.read() != '`');
  while(i < 20){
    if(Serial2.available() > 0){
      temp[i] = Serial2.read();
      if(temp[i] == ','){
        temp[i] = '\0';
        break;
      }
      i++;
    }
  }

  i = 0;
  while(i < 20){
    if(Serial2.available() > 0){
      hum[i] = Serial2.read();
      if(hum[i] == ','){
        hum[i] = '\0';
        return;
      }
      i++;
    }
  }
}

/* Read IIS3DWB's acceleration values */
void read_acc(){
  int i = 0;
  while(Serial2.read() != '^');

  while(i < 20){
    if(Serial2.available() > 0){
      time_[i] = Serial2.read();
      time_array += time_[i];
      if(time_[i] == ','){
        time_[i] = '\0';
        break;
      }
      i++;
    }
  }

  while(i < 20){
    if(Serial2.available() > 0){
      acc_x[i] = Serial2.read();
      acc_x_array += acc_x[i];
      if(acc_x[i] == ','){
        acc_x[i] = '\0';
        break;
      }
      i++;
    }
  }

  i = 0;
  while(i < 20){
    if(Serial2.available() > 0){
      acc_y[i] = Serial2.read();
      acc_y_array += acc_y[i];
      if(acc_y[i] == ','){
        acc_y[i] = '\0';
        break;
      }
      i++;
    }
  }

  i = 0;
  while(i < 20){
    if(Serial2.available() > 0){
      acc_z[i] = Serial2.read();
      acc_z_array += acc_z[i];
      if(acc_z[i] == ',') {
        acc_z[i] = '\0';
        return;
      }
      i++;
    }
  }
}

/* Read IIS3DWB's temperature values */
void read_temp_iis(){
  int i = 0;
  while(Serial2.read() != '*');
  while(i < 20){
    if(Serial2.available() > 0){
      temp_iis[i] = Serial2.read();
      if(temp_iis[i] == ',') {
        temp_iis[i] = '\0';
        return;
      }
      i++;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD0, TXD0);

  // Setup wifi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());

  client.setWriteOptions(WriteOptions().writePrecision(WritePrecision::MS));

  // Accurate time is necessary for certificate validation and writing in batches
  // We use the NTP servers in your area as provided by: https://www.pool.ntp.org/zone/
  // Syncing progress and the time will be printed to Serial.
  timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");

  // Check server connection
  if (client.validateConnection()) {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  } else {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }

  startTime = time(NULL);
}

void loop() {
  read_acc();
  read_temp_iis();

  // Appended values are sent to InfluxDB atleast after 2 seconds
  // Writing such a huge string takes around 5 seconds
  // FUTURE IMPROVEMENT: Write values after the entire 2 minute run is over, before the MCU goes to STANDBY; Can be implemented by some sort of handshake via UART
  if (time(NULL) - startTime > 2) {
    // Clear fields for reusing the point. Tags will remain the same as set above.
    sensor.clearFields();

    // Store measured value into point
    sensor.addField("timestamp", time_array);
    sensor.addField("acc_x", acc_x_array);
    sensor.addField("acc_y", acc_y_array);
    sensor.addField("acc_z", acc_z_array);
    sensor.addField("temp_iis", temp_iis);

    // Write point to cloud
    client.writePoint(sensor);

    // Below lines shouldn't really be required as the ESP32 should restart after EN pin goes high in the next run
    time_array = "";
    acc_x_array = "";
    acc_y_array = "";
    acc_z_array = "";
    startTime = time(NULL);
  }
}
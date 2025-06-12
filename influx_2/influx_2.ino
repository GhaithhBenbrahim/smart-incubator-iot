#include <WiFi.h>
#include <HTTPClient.h>

// WiFi Credentials
const char* ssid = "Redmi Note 12";
const char* password = "1234567890";

const char* influx_url = "https://us-east-1-1.aws.cloud2.influxdata.com/api/v2/write?org=myorg&bucket=incub_dataa&precision=s";
const char* influx_token = "dGOqXRopIwGjg0mSoZLzFFfaeej4Xdusrah5cs5mi9pwj0b2jJi2d9l5ESLUNEBZ60qdnBGhsSoU5Q0DQltQcQ==";  

#define RX_PIN 16
#define TX_PIN 17

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  
    
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi");
}

void sendToInfluxDB(float temperature, float humidity, float tmin, float tmax, float hmin, float hmax, const String& uptime, const String& doorState) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected. Cannot send data.");
        return;
    }

    HTTPClient http;
    http.begin(influx_url);
    http.addHeader("Authorization", "Token " + String(influx_token));
    http.addHeader("Content-Type", "text/plain; charset=utf-8");

    // Line protocol format
    String payload = "environment,sensor=stm32 ";
    payload += "temperature=" + String(temperature) + ",";
    payload += "humidity=" + String(humidity) + ",";
    payload += "tmin=" + String(tmin) + ",";
    payload += "tmax=" + String(tmax) + ",";
    payload += "hmin=" + String(hmin) + ",";
    payload += "hmax=" + String(hmax) + ",";
    payload += "uptime=\"" + uptime + "\",";         // Note the quotes for string fields
    payload += "door_state=\"" + doorState + "\"";

    Serial.print("Sending to InfluxDB: ");
    Serial.println(payload);

    int httpResponseCode = http.POST(payload);

    if (httpResponseCode == 204) {
        Serial.println("✅ Data successfully sent to InfluxDB.");
    } else {
        Serial.println("❌ Failed to send data. HTTP response: " + String(httpResponseCode));
        Serial.println("Response: " + http.getString());
    }

    http.end();
}

void loop() {
    if (Serial2.available()) {
        String uartData = Serial2.readStringUntil('\n');

        float temperature, humidity, tmin, tmax, hmin, hmax;
        char uptimeBuf[20];
        char doorBuf[10];

        Serial.print("Received from STM32: ");
        Serial.println(uartData);

        int numFields = sscanf(uartData.c_str(),
                               "T:%fF H:%f%% Tmin:%f Tmax:%f Hmin:%f Hmax:%f Uptime:%19s Door:%9s",
                               &temperature, &humidity, &tmin, &tmax, &hmin, &hmax, uptimeBuf, doorBuf);

        if (numFields == 8) {
            sendToInfluxDB(temperature, humidity, tmin, tmax, hmin, hmax, String(uptimeBuf), String(doorBuf));
        } else {
            Serial.println("❌ Error: Invalid data format.");
        }
    }

    delay(100); // CPU relief
}

//wifi test programm
#include <WiFiS3.h>

char ssid[] = "GroupW7";
char pass[] = "test1234";
WiFiServer server(5200);

bool firstConnect = true;

void setup() {
  Serial.begin(9600);
  

  WiFi.beginAP(ssid, pass);
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address:");
  Serial.println(ip);
  server.begin();
}

void loop() {
  WiFiClient client = server.available();
  if (client.connected() && firstConnect) {
    client.write("Hello Client");
    Serial.println("Connected to client");
    firstConnect = false;
  }
  Serial.println(client.read());
}

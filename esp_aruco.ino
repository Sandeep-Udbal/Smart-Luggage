#include <WiFi.h>

// WiFi credentials
const char* ssid = "Sandeep";     
const char* password = "12345678";    
const uint16_t port = 8002;
const char* host = "192.168.208.210";      



// Motor control pins
int IN1 = 18;
int IN2 = 19;
int IN3 = 21;
int IN4 = 22;

char incomingPacket[80];
WiFiClient client;

void setup() {
  Serial.begin(115200); // Serial for debugging

  // Pin Modes
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);


  // Initialize outputs
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  

  // Connecting to WiFi
  WiFi.begin(ssid, password);
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    Serial.print(".");
    timeout++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected with IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Failed to connect to WiFi.");
    while (1); // Halt execution
  }
}

void loop() {
  if (!client.connect(host, port)) {
    Serial.println("Connection to host failed");
    delay(2000); // Retry after delay
    return;
  }

  while (client.connected()) {
    if (client.available()) {
      String message = client.readStringUntil('\n'); // Read message until newline
      

      // Debugging received message
      Serial.println("Received: " + message);
     
      message.trim();
      
      if (message == "forward") {
        Serial.println("Executing forward");
        forward();
        delay(100);
        stop();
      } else if (message == "left") {
        Serial.println("Executing left");
        left();
        delay(100);
        stop();
      } else if (message == "right") {
        Serial.println("Executing right");
        right();
        delay(100);
        stop();
      } else if (message == "stop") {
        Serial.println("Executing stop");
        stop();
      } else if (message == "Sleft") {
        Serial.println("Executing Sleft");
        left();
        delay(200);
        stop();
      } else if (message == "Sright") {
        Serial.println("Executing Sright");
        right();
        delay(200);
        stop();
      }
      
    }
  }
}

// Motor functions
void forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void left() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void right() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

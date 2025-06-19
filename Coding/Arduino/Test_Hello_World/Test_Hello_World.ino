// Pin definitions
//#define RED_PIN    2
//#define GREEN_PIN  3
//#define BLUE_PIN   4
#define BUZZER_PIN 14

void setup() {
  // Set pins as output
  //pinMode(RED_PIN, OUTPUT);
  //pinMode(GREEN_PIN, OUTPUT);
  //pinMode(BLUE_PIN, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);

  Serial.begin(115200);  // Start serial communication
  delay(1000);           // Give it time to start
  Serial.println("Hello, ESP32-C6!");

  pinMode(2, OUTPUT);
}

void loop() {

  tone(BUZZER_PIN, 500);
  delay(500);
  tone(BUZZER_PIN, 1000);
  delay(500);
  tone(BUZZER_PIN, 1500);
  delay(500);
  tone(BUZZER_PIN, 2000);
  delay(500);
  noTone(BUZZER_PIN);
  delay(1000);

  // Turn on RED
  //digitalWrite(RED_PIN, HIGH);
  //digitalWrite(GREEN_PIN, LOW);
  //digitalWrite(BLUE_PIN, LOW);
  Serial.println("Still alive...");
  delay(1000);  // Wait 1 second

  // Turn on GREEN
  //digitalWrite(RED_PIN, LOW);
  //digitalWrite(GREEN_PIN, HIGH);
  //digitalWrite(BLUE_PIN, LOW);
  //Serial.println("Still alive...");
  //delay(1000);

  // Turn on BLUE
  //digitalWrite(RED_PIN, LOW);
  //digitalWrite(GREEN_PIN, LOW);
  //digitalWrite(BLUE_PIN, HIGH);
  //Serial.println("Still alive...");
  //delay(1000);

  // Turn on all (WHITE if common cathode)
  //digitalWrite(RED_PIN, HIGH);
  //digitalWrite(GREEN_PIN, HIGH);
  //digitalWrite(BLUE_PIN, HIGH);
  //Serial.println("Still alive...");
  //delay(1000);

  // Turn off all
  //digitalWrite(RED_PIN, LOW);
  //digitalWrite(GREEN_PIN, LOW);
  //digitalWrite(BLUE_PIN, LOW);
  //Serial.println("Still alive...");
  //delay(1000);
}


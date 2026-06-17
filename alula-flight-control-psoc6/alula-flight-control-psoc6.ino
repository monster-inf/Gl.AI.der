#include <WiFi.h>
#include <WiFiUdp.h>
#include "secrets.h"
#include <ServoC.h>

// WLAN access
const char* ssid     = NET_SECRET_SSID;
const char* password = NET_SECRET_PASSWORD;

// UDP settings
WiFiUDP udp;
unsigned int localPort = 5005;

// Servos
ServoC servo_left;
ServoC servo_right;
unsigned long lastServoUpdate = 0;
const int SERVO_INTERVAL = 50;

// Shared state (updated from UDP packets)
volatile uint8_t target_roll  = 90;
volatile uint8_t target_pitch = 95; 
volatile bool    steer_active = false;

// Smoothing state
int last_left = 90;
int last_right = 90;
float smooth_pitch = 95;
float smooth_roll = 90;

// Trimming to neutral
const int ROLL_TRIM = 0;
const int PITCH_TRIM = 0;

//Sensibility
const float PITCH_RATE = 0.15;
const float ROLL_RATE = 0.20;

//Failsafe variables
unsigned long lastPackageTime = millis();
const unsigned long TIMEOUT_CONNECTION_LOST = 1500;
bool hasEverReceived = false;

// Track previous steer_active to detect transitions
bool prev_steer_active = false;


// Mixing function for elevon mixing and interpolation
void elevon_mixing(int target_pitch, int target_roll, int* out_left, int* out_right){
    float interpolation = 0.45;

    //Absolute change from neutral
      float delta_pitch = (int)target_pitch - 95;
      float delta_roll = (int)target_roll - 90;
    //Scaling down movement
      float target_smooth_pitch = delta_pitch * PITCH_RATE;
      float target_smooth_roll  = delta_roll * ROLL_RATE;
    //Interpolation
      smooth_pitch += (target_smooth_pitch - smooth_pitch) * interpolation;
      smooth_roll += (target_smooth_roll - smooth_roll) * interpolation;
    //Mixing
      *out_left  = constrain((int)(SERVO_LEFT_NEUTRAL + smooth_pitch - smooth_roll), 50, 120);
      *out_right = constrain((int)(SERVO_RIGHT_NEUTRAL + smooth_pitch + smooth_roll), 50, 120);
}


// Failsafe: full up-elevator (both elevons up) + slight roll
void Failsafe(){
    target_roll  = 80;
    target_pitch = 110;
}


void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("--- PSoC 6 SoftAP & UDP Server ---");

  // WiFi first
  WiFi.beginAP(ssid, password);
  delay(500);

  // Attach servos after WiFi
  servo_left.attach(0);
  servo_right.attach(1);

  Serial.print("AP active! SSID: ");
  Serial.println(ssid);
  Serial.print("PSoC IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(localPort);
  Serial.print("UDP server ready on port: ");
  Serial.println(localPort);
  Serial.println("Packet format: [0xAA, armed, roll, pitch]");
  Serial.println("----------------------------------");
}


void loop() {
  // Drain UDP buffer - keep only the latest packet to avoid command lag
  uint8_t last_steer = 0, last_roll = 90, last_pitch = 95;
  bool got_packet = false;
  unsigned long currentMillis = millis();

  while (true) {
          int n = udp.parsePacket();
          if (n < 4) break; 

          uint8_t header = udp.read();
          if (header != 0xAA) continue;  // misaligned - discard

          last_steer = udp.read();   // byte[1]: steering active flag
          last_roll  = udp.read();   // byte[2]: roll left  servo angle (0–180)
          last_pitch = udp.read();   // byte[3]: pitch angle (0–180, 90 = neutral)
          got_packet = true;
  }

  if (got_packet) 
      {
          steer_active  = (last_steer != 0);
          target_roll   = last_roll;
          target_pitch  = last_pitch;
          lastPackageTime = currentMillis;
          hasEverReceived = true;

        Serial.print("RX: armed=");
        Serial.print(steer_active ? 1 : 0);
        Serial.print(" R="); Serial.print(target_roll);
        Serial.print(" P="); Serial.println(target_pitch);
      }
    
    if (hasEverReceived && (currentMillis - lastPackageTime > TIMEOUT_CONNECTION_LOST)) 
      {
        Failsafe();
      }

  if(currentMillis - lastServoUpdate >= SERVO_INTERVAL)
      {
        lastServoUpdate = currentMillis;
        
        int goal_left, goal_right;
          if (steer_active) 
              {
                elevon_mixing(target_pitch, target_roll, &goal_left, &goal_right);
                prev_steer_active = true;
              } 
          else 
              {
                // snap smooth state to neutral on disarm transition
                if (prev_steer_active) {
                    smooth_pitch = 95;
                    smooth_roll  = 90;
                    prev_steer_active = false;
                }
                elevon_mixing(95, 90, &goal_left, &goal_right);
              }

          // write every cycle — no deadband
          servo_left.write(goal_left);
          last_left = goal_left;

          servo_right.write(goal_right);
          last_right = goal_right;
      }
}

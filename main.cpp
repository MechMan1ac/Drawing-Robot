#include <math.h>

#include "motor.h"
motor motor_left(25, 26);
motor motor_rear(17, 21);
motor motor_right(32, 33);

int pinA_left = 27; 
int pinB_left = 14;

int pinA_rear = 16; 
int pinB_rear = 4;

int pinA_right = 19; 
int pinB_right = 18;

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "html.h"

const char gcode_line1[] = "G01 X0.0 Y200.0";
const char gcode_line2[] = "G01 X200.0 Y200.0";
const char gcode_line3[] = "G01 X200.0 Y0.0";
const char gcode_line4[] = "G01 X0.0 Y0.0";
const char gcode_line5[] = "G01 X200.0 Y200.0";
const char gcode_line6[] = "G00 X0.0 Y200.0";
const char gcode_line7[] = "G01 X200.0 Y0.0";
const char gcode_line8[] = "G00 X0.0 Y0.0";

char ram_buffer[32];
char cmd[4];

const char* ssid = "Robot";
const char* password = "12345678";

AsyncWebServer server(80);

float speed_percent = 50.0;

float wheel_radius = 0.03; //m
float robot_radius = 0.11605; //m
float max_motor_speed = 18.64; //rad/s

float max_robot_speed = 0.5592; //m/s
float max_robot_spin = 3.856; //rad/s

float a_inverse = -0.5;  float b_inverse = 0.866;  float c_inverse = robot_radius;
float d_inverse = -0.5;  float e_inverse = -0.866;  float f_inverse = robot_radius;
float g_inverse = 1.0;  float h_inverse = 0;  float i_inverse = robot_radius;

float PPR = 616.0;
float d_per_count = (2.0*PI*wheel_radius)/PPR; //m/count

volatile long count_left = 0; volatile long count_rear = 0; volatile long count_right = 0;

int target_count_right;
int target_count_left;
int target_count_rear;

float current_x = 0.0;
float current_y = 0.0;

float target_x = 0.0;
float target_y = 0.0;

bool robot_moving = false;

float dx = 0.0;
float dy = 0.0;

bool start_gcode = false;

#include <ESP32Servo.h>

Servo myservo;

bool servo_active = true;

void isr_left(){
  if (digitalRead(pinB_left) == HIGH){
    count_left++;
  }else{
    count_left--;
  }
}

void isr_rear(){
  if (digitalRead(pinB_rear) == HIGH){
    count_rear++;
  }else{
    count_rear--;
  }
}

void isr_right(){
  if (digitalRead(pinB_right) == HIGH){
    count_right++;
  }else{
    count_right--;
  }
}

int pwm_convert(int angular_velocity){
  int pwm_value = angular_velocity * 255/max_motor_speed;
  return pwm_value;
}

void setMotorSpeeds(float x_speed, float y_speed, float w_speed){
  int speed_right = x_speed * a_inverse/wheel_radius + y_speed * b_inverse/wheel_radius + w_speed * c_inverse/wheel_radius;
  int speed_left = x_speed * d_inverse/wheel_radius + y_speed * e_inverse/wheel_radius + w_speed * f_inverse/wheel_radius;
  int speed_rear = x_speed * g_inverse/wheel_radius + y_speed * h_inverse/wheel_radius + w_speed * i_inverse/wheel_radius;

  Serial.print(pwm_convert(speed_left));
  Serial.print(", ");
  Serial.print(pwm_convert(speed_right));
  Serial.print(", ");
  Serial.println(pwm_convert(speed_rear));

  motor_right.move(pwm_convert(speed_right));
  motor_left.move(pwm_convert(speed_left));
  motor_rear.move(pwm_convert(speed_rear));
}

float getGcodeValue(char letter, float default_value){
  char* letter_ptr = strchr(ram_buffer, letter);
  if (letter_ptr != NULL){
    return strtof(letter_ptr + 1, NULL);
  }

  return default_value;
}

void move(String orientation){
  if(current_x != target_x || current_y != target_y){
    robot_moving = true;
    while(robot_moving){
      if(abs(count_left) >= abs(target_count_left) && abs(count_rear) >= abs(target_count_rear) && abs(count_right) >= abs(target_count_right)){
        Serial.println("Target reached");
        setMotorSpeeds(0, 0, 0);
        robot_moving = false;
        current_x = target_x;
        current_y = target_y;
        count_left = 0;
        count_right = 0;
        count_rear = 0;
      }else{
        if (orientation == "diagonal"){
          if (dx > 0.0 && dy > 0.0){
            setMotorSpeeds(max_robot_speed*((speed_percent-5.0)/100.0), max_robot_speed*((speed_percent-5.0)/100.0), 0.0);
          }else if (dx < 0.0 && dy > 0.0){
            setMotorSpeeds(-max_robot_speed*((speed_percent-5.0)/100.0), max_robot_speed*((speed_percent-5.0)/100.0), 0.0);
          }else if (dx > 0.0 && dy < 0.0){
            setMotorSpeeds(max_robot_speed*((speed_percent-5.0)/100.0), -max_robot_speed*((speed_percent-5.0)/100.0), 0.0);
          }else if (dx < 0.0 && dy < 0.0){
            setMotorSpeeds(-max_robot_speed*((speed_percent-5.0)/100.0), -max_robot_speed*((speed_percent-5.0)/100.0), 0.0);
          }
        }else if (orientation == "x-axis"){
          if (dx > 0.0){
            setMotorSpeeds(max_robot_speed*(speed_percent/100.0), 0.0, 0.0);
          }else if (dx < 0.0){
            setMotorSpeeds(-max_robot_speed*(speed_percent/100.0), 0.0, 0.0);
          }
        }else if (orientation == "y-axis"){
          if (dy > 0.0){
            setMotorSpeeds(0.0, max_robot_speed*(speed_percent/100.0), 0);
          }else if (dy < 0.0){
            setMotorSpeeds(0.0, -max_robot_speed*(speed_percent/100.0), 0.0);
          }
        }else{
          // No movement needed
        }
      }
    }
  }
}

void processGcode(const char* gcode){
  strncpy(ram_buffer, gcode, sizeof(ram_buffer) - 1);
  ram_buffer[sizeof(ram_buffer) - 1] = '\0'; // Ensure null termination

  sscanf(gcode, "%3s", cmd);

  if(strcmp(cmd, "G00") == 0){
    myservo.write(30); // set servo to 30 degrees
  }

  if(strcmp(cmd, "G01") == 0){
    myservo.write(0); // set servo to 0 degrees
  }

  target_x = getGcodeValue('X', 0.0);
  target_y = getGcodeValue('Y', 0.0);

  dx = target_x - current_x;
  dy = target_y - current_y;

  target_count_right = (dx * a_inverse + dy * b_inverse + 0 * c_inverse)/(d_per_count*1000);
  target_count_left = (dx * d_inverse + dy * e_inverse + 0 * f_inverse)/(d_per_count*1000);
  target_count_rear = (dx * g_inverse + dy * h_inverse + 0 * i_inverse)/(d_per_count*1000);

  Serial.print(target_count_left);
  Serial.print(", "); 
  Serial.print(target_count_right);
  Serial.print(", ");
  Serial.println(target_count_rear);

  if (dx != 0.0 && dy != 0.0){
    move("diagonal");
  }else if (dx != 0.0 && dy == 0.0){
    move("x-axis");
  }else if (dx == 0.0 && dy != 0.0){
    move("y-axis");
  }else{
    // No movement needed
  }
}

String updateSlider(const String& var){
  if (var == "SLIDER_VALUE"){
    return String(speed_percent);
  }

  return String();
}

void setup() {
  Serial.begin(115200);

  pinMode(pinA_left, INPUT_PULLUP); 
  pinMode(pinB_left, INPUT_PULLUP);

  pinMode(pinA_rear, INPUT_PULLUP); 
  pinMode(pinB_rear, INPUT_PULLUP);
  
  pinMode(pinA_right, INPUT_PULLUP); 
  pinMode(pinB_right, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pinA_left), isr_left, RISING);
  attachInterrupt(digitalPinToInterrupt(pinA_rear), isr_rear, RISING);
  attachInterrupt(digitalPinToInterrupt(pinA_right), isr_right, RISING);

  WiFi.softAP(ssid, password);
  Serial.println("Access Point Started");
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());

  Serial.print(target_count_left);
  Serial.print(", ");
  Serial.print(target_count_right);
  Serial.print(", ");
  Serial.println(target_count_rear);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", html_page, updateSlider);
  });

  server.on("/slider", HTTP_GET, [](AsyncWebServerRequest *request){
    String inputMessage;
    if(request->hasParam("value")){
      inputMessage = request->getParam("value")->value();
      speed_percent = inputMessage.toFloat();
    }else{
      inputMessage = "No message sent";
    }

    Serial.print(speed_percent);
    Serial.println("%");
    request->send(200, "text/plain", "ok");
  });

  server.on("/q", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Turning left");
    setMotorSpeeds(0.0, 0.0, max_robot_spin*(speed_percent/100.0));
    request->send(200, "text/plain", "Turning left");
  });

  server.on("/f", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Moving forward");
    setMotorSpeeds(0.0, max_robot_speed*(speed_percent/100.0), 0.0);
    request->send(200, "text/plain", "Moving forward");
  });

  server.on("/e", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Turning right");
    setMotorSpeeds(0.0, 0.0, -max_robot_spin*(speed_percent/100.0));
    request->send(200, "text/plain", "Turning right");
  });

  server.on("/l", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Moving left");
    setMotorSpeeds(-max_robot_speed*(speed_percent/100.0), 0.0, 0.0);
    request->send(200, "text/plain", "Moving left");
  });

  server.on("/s", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Stop");
    current_x = 0.0;
    current_y = 0.0;
    target_x = 0.0;
    target_y = 0.0;
    setMotorSpeeds(0.0, 0.0, 0.0);
    count_left = 0;
    count_right = 0;
    count_rear = 0;
    request->send(200, "text/plain", "Stop");
  });

  server.on("/r", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Moving right");
    setMotorSpeeds(max_robot_speed*(speed_percent/100.0), 0.0, 0.0);
    request->send(200, "text/plain", "Moving right");
  });

  server.on("/b", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Moving backward");
    setMotorSpeeds(0.0, -max_robot_speed*(speed_percent/100.0), 0.0);
    request->send(200, "text/plain", "Moving backward");
  });

  server.on("/g", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Run Gcode");
    start_gcode = true;
    request->send(200, "text/plain", "Run Gcode");
  });

  server.on("/h", HTTP_GET, [](AsyncWebServerRequest *request){
    if (servo_active){
      myservo.write(30); // set servo to 30 degrees
      servo_active = false;   
    }else{
      myservo.write(0); // set servo to 0 degrees
      servo_active = true;
    }
    request->send(200, "text/plain", "Servo");
  });

  server.begin();

  myservo.attach(13); 
  myservo.write(0); // set servo to 30 degrees
}

void loop() {
  if(start_gcode){
    processGcode(gcode_line1);
    processGcode(gcode_line2);
    processGcode(gcode_line3);
    processGcode(gcode_line4);
    processGcode(gcode_line5);
    processGcode(gcode_line6);
    processGcode(gcode_line7);
    processGcode(gcode_line8);
    Serial.print(current_x);
    Serial.print(", ");
    Serial.println(current_y);
    start_gcode = false;
  }
 
}

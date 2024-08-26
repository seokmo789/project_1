/*
자동차 제어 부분 입니다.

1. ESP32 wifi 라이브러리 추가
2. ESP32 ESPAsyncWebServer 라이브러리 추가

board : ESP32
IDE : Arduino IDE
*/


#include <WiFi.h>
#include <ESPAsyncWebServer.h>


#define motor1_pin 16
#define motor2_pin 17
//
int defult_speed =140;
int set_speed =0;
int increase_speed=0;
int decrease_speed=0;

int flag=0;

const int pwmPin1 = 17;
const int direction_Pin1 = 16;

const int pwmPin2 = 19;
const int direction_Pin2 = 18;

const int pwmChannel1 = 0;
const int pwmChannel2 = 0;
const int pwmFreq = 20000;
const int pwmResolution = 8;
const int dutyCycle=158;

//
const char* ssid = "Weare2";
const char* password = "12345678";

AsyncWebServer server(80);

// 전역 변수 선언
String receivedBody = "";

void setup() 
{
  ledcSetup(pwmChannel1, pwmFreq, pwmResolution);
  ledcSetup(pwmChannel2, pwmFreq, pwmResolution);
  ledcAttachPin(pwmPin1, pwmChannel1);
  ledcAttachPin(pwmPin2, pwmChannel2);
  pinMode(direction_Pin1, OUTPUT);
  pinMode(direction_Pin2, OUTPUT);
  // GPIO 핀을 HIGH로 설정
  digitalWrite(direction_Pin1, HIGH);
  digitalWrite(direction_Pin2, HIGH);
  ledcWrite(pwmChannel1, 0);
  ledcWrite(pwmChannel2, 0);

  Serial.begin(115200);
  WiFi.begin(ssid, password);

  // Wi-Fi 연결 대기
  while (WiFi.status() != WL_CONNECTED) 
  {
    Serial.print(".");
    delay(500);
  }

  Serial.println("WiFi에 연결되었습니다");
  Serial.print("IP 주소: ");
  Serial.println(WiFi.localIP());

  // POST 요청 처리
  server.on("/control", HTTP_POST, [](AsyncWebServerRequest *request){
    // POST 요청을 수신한 후 즉시 응답을 보냅니다
    request->send(200, "text/plain", "Data received");
  }, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
    // 요청의 바디 데이터가 수신될 때 호출됩니다
    if (index == 0) {
      receivedBody = ""; // 새 요청 시작 시, 전역 변수를 초기화합니다
    }
    // 데이터가 들어올 때마다 전역 변수에 추가합니다
    receivedBody += String((char*)data);
    if (index + len == total) {
      // 전체 데이터 수신 완료
      //Serial.print("수신된 데이터: ");
      //Serial.println(receivedBody);
    }
  });

  server.begin();
}


void car_right_func()
{
  Serial.println("r");
  ledcWrite(pwmChannel1, set_speed+5);
  ledcWrite(pwmChannel2, set_speed+5);
  digitalWrite(direction_Pin1, LOW);
  digitalWrite(direction_Pin2, HIGH);
}

void car_left_func()

{
  Serial.println("l");
  ledcWrite(pwmChannel1, set_speed+5);
  ledcWrite(pwmChannel2, set_speed+5);
  digitalWrite(direction_Pin1, HIGH);
  digitalWrite(direction_Pin2, LOW);
}

void car_stop_func()
{
  ledcWrite(pwmChannel1, 0);
  ledcWrite(pwmChannel2, 0);
  digitalWrite(direction_Pin1, LOW);
  digitalWrite(direction_Pin2, LOW);
}

void car_go_func()
{
  ledcWrite(pwmChannel1, set_speed);
  ledcWrite(pwmChannel2, set_speed);
  digitalWrite(direction_Pin1, HIGH);
  digitalWrite(direction_Pin2, HIGH);
}

void car_back_func()
{
  ledcWrite(pwmChannel1, set_speed);
  ledcWrite(pwmChannel2, set_speed);
  digitalWrite(direction_Pin1, HIGH);
  digitalWrite(direction_Pin2, HIGH);
}


void car_increase_speed_func()
{
  defult_speed+=1;
  set_speed=defult_speed;
  Serial.println(set_speed);
  ledcWrite(pwmChannel1, set_speed);
  ledcWrite(pwmChannel2, set_speed);
  digitalWrite(direction_Pin1, HIGH);
  digitalWrite(direction_Pin2, HIGH);
  
}

 void car_reset_speed_func()
 {
  defult_speed =140;
  set_speed =0;
  increase_speed=0;
  decrease_speed=0;
 }

void loop() {
  // loop()에서 수신된 데이터 처리
  if (receivedBody.length() > 0) {
    // 데이터가 있는 경우
    receivedBody.trim(); // 문자열의 양쪽 끝에서 공백 및 개행 문자 제거

    if (receivedBody.equals("s")) {car_stop_func();}
    else if (receivedBody.equals("r")) {car_right_func();} 
    else if (receivedBody.equals("l")) {car_left_func();}
    else if (receivedBody.equals("g")) {car_go_func();}
    //else if (receivedBody.equals("d")) {car_decrease_speed_func();}
    else if (receivedBody.equals("u")) {car_increase_speed_func();}

    // 데이터 처리 후, 전역 변수 초기화 (필요에 따라)
    receivedBody = "";
  }
}

#include <Arduino.h>     // ESP32-S3 기본 라이브러리 선언
#include <Wire.h>        // SDA, SCL 핀을 통한 I2C 통신 프로토콜 라이브러리 선언
#include "VL53L0X.h"     // ToF API 활성화
#include "vl53l0x_platform.h"
#include "vl53l0x_api.h"
#include "vl53l0x_def.h"
#include "WiFi.h"        // ESP32-S3 WiFi 기능 활성화
#include <HTTPClient.h>  // 서버 API 전송 라이브러리 선언
#include <hp_BH1750.h>   // BH1750 조도 센서 라이브러리 선언

HardwareSerial mmUART(2); // Serial Communication을 위해 UART 정의
#define mmRX 5            // mmWave RX와 ESP32-S3 GPIO5 연결
#define mmTX 6            // mmWave TX와 ESP32-S3 GPIO6 연결
#define BaudRate 115200   // 통신 속도 115200 정의

// [0. 와이파이 및 서버 설정 관련]
// [1. ToF 센서 설정 관련]
// [2. mmWave 센서 설정 관련]
// [3. MAX9814 센서 설정 관련]
// [4. BH1750 센서 설정 관련]
// [5. 센서별 정규화]

// [3. MAX9814 센서 설정]
#define maxout 1                       // MAX9814와 ESP32-S3 GPI01 연결
const unsigned int sampleWindow = 50;  // 50ms 샘플링 창
String soundStatus = "대기 중";         // 실시간 공간 상태 저장용 변수 
unsigned int globalPeakToPeak = 0;     // 전역 변수

// [4. BH1750 센서 설정]
hp_BH1750 lightMeter;    // BH1750 조도 센서 객체 생성
float globalLux = 0.0;   // 실시간 조도값 저장용 변수
String luxStatus = "어두움"; // 실시간 조도 상태 저장용 변수

// [0. 와이파이 설정]
const char* ssid = "SSID";           // 공유기 SSID 입력
const char* password = "PASSWORD"; // 공유기 비밀번호 입력

// [0. 서버 설정]
const char* serverUrl = "https://api.studio.udtt.org/api/v1/space/telemetry";  // 서버 엔드포인트 정의
const char* deviceId = "ESP32_S3_1";                                                  // 기기 식별 ID 정의

// [0. 비동기 타이머 변수 정의]
unsigned long lastWiFiCheckTime = 0;
const unsigned long wifiCheckInterval = 5000;   // 5초마다 와이파이 연결 상태 점검

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 3000;        // 3초마다 센서 데이터를 서버로 전송

// [5. 센서별 정규화]
int breathOfSpaceIndex = 0;

// [1. ToF 센서 I2C 구조체]
VL53L0X_Dev_t vl53dev;                     // ToF 센서의 I2C 주소 등의 고유 물리 정보를 기록할 구조체 정의
VL53L0X_RangingMeasurementData_t vl53data; // 측정한 거리, 신호 강도, 에러율 정보를 담을 구조체 정의

// [1. ToF 인원 카운팅 관련 변수]
int baselineDistance = 1500;  // 센서 오류 방지용 최적 측정 거리인 1.5m 정의
const int tofThreshold = 150; // 평소 거리보다 최소 15cm 이상 움직여야 사람으로 인지할 문턱값 정의

volatile int totalVisitorCount = 0;    // 들어오는 방향으로 완전히 통과한 순수 누적 방문자 수
volatile int currentPeopleCount = 0;   // 현재 해당 공간 내부에 머물고 있는 실시간 재실 중인 인원수
int currentAlgorithmState = 0;         // 45도 비스듬히 설치 후, 통과 판정을 위한 상태 머신 변수 (0: 대기 상태, 1: 사람이 감지되어 통과 중)
unsigned long lastStateTime = 0;       // 센서 바로 밑에 서 있는 등 currentAlgorithmState = 1에서 멈췄을 때의 풀기 위한 용도
const unsigned long TIMEOUT_MS = 2000; // 사람이 센서 구역을 빠져나가지 않고 버틸 때 강제 리셋할 제한 시간(2초)

// [2. mmWave 움직임 감지 관련 변수]
float lastDistance = 0.0;        // 센서 - 사람 간 측정했던 마지막 거리 저장
unsigned long lastMoveTime = 0;  // 사람이 마지막으로 움직인 시간 저장
const int motionTimeout = 2000;  // 2초 동안 움직임이 없으면 정지로 판정
const float moveThreshold = 0.1; // 0.1m 이상 변하면 움직임으로 판정
bool isMoving = false;           // 현재 레이더 범위 내에 활발하게 움직이는 대상이 있는지 판정용
String inputBuffer = "";         // 센서가 보내는 데이터 문자들을 한 문장으로 모으는 버퍼

// [0. 서버로 JSON 데이터 전송]
void sendDataToBackend() 
{
  if (WiFi.status() != WL_CONNECTED) // 와이파이 연결 시에만 서버 전송
    {
      Serial.println("[🔴 서버 전송 제한] 와이파이 미연결");
      return;
    }

    HTTPClient http;
    
    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(1500); // 1.5초 타임아웃 제한하여 ESP32-S3 락 방지

    // JSON 데이터
    String jsonPayload = "{";
    jsonPayload += "\"device_id\":\"" + String(deviceId) + "\",";                     // 기기 식별 ID 전송
    jsonPayload += "\"breathOfSpaceIndex\":" + String(breathOfSpaceIndex) + ",";      // 공간의 숨결 지수 전송
    jsonPayload += "\"currentPeopleCount\":" + String((int)currentPeopleCount) + ","; // [ToF] 재실 인원 수 전송
    jsonPayload += "\"totalVisitorCount\":" + String((int)totalVisitorCount) + ",";   // [ToF] 누적 방문자 수 전송
    jsonPayload += "\"isMoving\":" + String(isMoving ? "true" : "false") + ",";       // [mmWave] 활성화, 비활성화 구분
    jsonPayload += "\"globalPeakToPeak\":" + String(globalPeakToPeak) + ",";          // [MAX9814] 소음 Peak-to-Peak 전송
    jsonPayload += "\"soundStatus\":\"" + soundStatus + "\",";                        // [MAX9814] 공간 소음 단계 전송
    jsonPayload += "\"globalLux\":" + String((int)globalLux) + ",";                   // [BH1750] 실시간 조도값 전송
    jsonPayload += "\"luxStatus\":\"" + luxStatus + "\"";                             // [BH1750] 실시간 조도 상태 전송
    jsonPayload += "}";

    int httpResponseCode = http.POST(jsonPayload);

    if (httpResponseCode > 0) 
    {
      Serial.printf("[🟢 백엔드 전송 성공] 응답 코드: %d\n", httpResponseCode);
    } 
    else // 서버 다운, 포트 미개방 등 에러 발생 시에도 ESP32-S3는 정상 동작
    {
      Serial.printf("[🔴 백엔드 전송 실패] 에러 내용: %s\n", http.errorToString(httpResponseCode).c_str());
    }
    http.end();
}

// [2. mmWave 데이터 처리 관련]
void processRadarData(String data) // 센서 데이터의 마지막에는 \n이 붙으므로, \n 직전까지의 문자를 변수 data에 저장 ex) "ON", "Range 105"
{
  data.trim();                     // 변수 앞뒤의 공백, \n 같은 문자 제거하여 순수한 센서 데이터만 저장 ex) "Range 105"           
  if (data.length() == 0)
  return; 
  if (data == "ON" || data == "OFF")
  return;                                   

  String datanum = "";                    // 숫자 데이터만 담기 위해 변수 datanum 정의
  for (int i = 0; i < data.length(); i++) // 변수 data의 글자 수만큼 반복 검사
  { 
    char c = data.charAt(i);              // data에서 i번째에 있는 글자 한 개 추출
    if (isDigit(c) || c == '.')           // 추출한 글자가 숫자나 소수점 기호라면
    { 
      datanum += c;                       // datanum에 저장 ex) "105"
    }
  }

  if (datanum.length() > 0)                    // datanum에 숫자가 추출되면
  { 
    float currentDistance = datanum.toFloat(); // 문자열 "105"를 실수형 숫자 105.0로 변환
    currentDistance = currentDistance / 100.0; // cm -> m 단위 변환 ex) 105.0 -> 1.05(m)

    if (currentDistance > 0.1)                                  // 센서 바로 앞 0.1m보다 큰 데이터라면            
    { 
      float Distancediff = abs(currentDistance - lastDistance); // |방금 측정한 거리 - 직전에 기억해둔 거리|
      
      if (Distancediff >= moveThreshold)                        // 거리 변화량이 설정한 움직임 기준치보다 크다면
      { 
        lastMoveTime = millis();                                // 그 시간을 타임스탬프로 저장
        lastDistance = currentDistance;                         // 다음 비교를 위해 그 거리를 저장
      }
    }
  }
}

// [0. 와이파이 및 센서 활성]
void setup()
{
  uint8_t status = VL53L0X_ERROR_NONE;  // ToF 센서 API 오류 상태를 체크하기 위한 통신 판정 상태 변수 선언
  Serial.begin(BaudRate);               // 컴퓨터 시리얼 모니터 디버깅용 기본 UART 포트 개방          
  delay(1000);                          // 하드웨어 전원이 안정화되고 시리얼 포트가 확실히 연결될 때까지 1초간 대기                    

  pinMode(maxout, INPUT);         // MAX9814 설정
  analogSetAttenuation(ADC_11db); // ESP32-S3 아날로그 감쇄 최적화 (Gain - 3.3V, 40dB 대응)

  WiFi.mode(WIFI_STA);   
  WiFi.begin(ssid, password);
  Serial.println("\n[시스템] 와이파이 연결 중");

  // [2. mmWave 센서 통신 시작]
  mmUART.begin(BaudRate, SERIAL_8N1, mmRX, mmTX); // mmUART 활성화
  mmUART.setTimeout(5);                           // 최대 지연 시간을 5ms로 한계 설정              
  Serial.println("[🟢 시스템] mmUART 정상 활성화");

  // [1. ToF 센서 통신 시작]
  Wire.begin(11, 12);                             // I2C 통신 버스 가동 및 ESP32-S3 보드의 SDA=11번, SCL=12번 핀 할당           
  status = vl53l0x_Init(&vl53dev, 2);             // 2번 포트로 지정하여 ToF 센서 장치 드라이버 초기화 알고리즘 구동
  if(status != VL53L0X_ERROR_NONE)                // 초기화 반환 값이 에러 코드라면
  {
    Serial.println("[🔴 경고] VL53L0X 비활성화");   // 컴퓨터 화면에 에러를 공지
    while(1);                                       // 하드웨어가 비정상이므로 무한 루프 진입                                                                
  }
  else
  {
    Serial.println("[🟢 시스템] VL53L0X 정상 활성화");
  }

  // [4. BH1750 센서 초기화]
  if (lightMeter.begin(0x23)) // 기본 주소 0x23을 할당하여, I2C 조도 센서 초기화
  {
    lightMeter.start();       // 센서 연결에 성공한 경우, 실시간 측정 시작
    Serial.println("[🟢 시스템] BH1750 정상 활성화");
  }
  else
  {                           // 센서 연결에 실패한 경우
    Serial.println("[🔴 경고] BH1750 비활성화");
  }

  // [1. ToF 센서 초기 베이스라인 측정]
  Serial.println("[시스템] 거리 측정 중입니다. 센서를 가리지 마세요.");
  long sum = 0;                               // 15번 측정하는 동안의 모든 누적 거리 합산 값을 담아둘 변수 정의
  int validSamples = 0;                       // 순간 노이즈를 제외하고 통과한 순수 정상 데이터의 개수를 세는 카운터 변수
  for(int i = 0; i < 15; i++)                 // 환경 맞춤형 자동 거리 세팅을 위해 총 15회 샘플링 루프 가동
  { 
    vl53l0x_single_test(&vl53dev, &vl53data); // ToF 레이저 단발성 1회 정밀 거리 측정
    int measured = vl53data.RangeMilliMeter;  // 센서가 수집한 거리값(mm)을 임시 변수에 격리
    if (measured <= 2000 && measured > 20)    // 측정값이 유효 측정 반경(2cm 초과 ~ 2m 이하) 내에 있는 정상 데이터인 경우
    { 
      sum += measured;                        // 전체 합산 변수에 누적해서 더함              
      validSamples++;                         // 유효 데이터 샘플 개수를 1 올림
    }
    delay(100);                               // 0.1초 간격으로 측정하여 하드웨어 부하와 간섭을 방지      
  }
  
  baselineDistance = (validSamples > 0) ? (sum / validSamples) : 1500; // 유효 데이터가 단 1개라도 있다면 평균값을 계산하고, 15번 모두 에러가 났다면 baselineDistance = 1500 활성화

  Serial.print("[시스템] Baseline Distance Set: ");
  Serial.print(baselineDistance);                                       // 최종적으로 시스템의 기준선이 된 바닥/벽면 거리를 출력                 
  Serial.println(" mm\n--------------------------------------------------");
}

// [무한 반복 루프]
void loop()
{
  // [0. 5초 주기 와이파이 연결 상태 점검] Non-blocking 타이머 기반 와이파이 연결 상태 점검 (5초 주기)
  if (millis() - lastWiFiCheckTime >= wifiCheckInterval) 
  {
    lastWiFiCheckTime = millis();
    if (WiFi.status() == WL_CONNECTED) 
    {
      static bool ipPrinted = false;
      if (!ipPrinted) 
      {
        Serial.print("[🟢 와이파이 연결] 연결 성공! 할당 IP: ");
        Serial.println(WiFi.localIP());
        ipPrinted = true;
      }
    } 
    else 
    {
      Serial.println("[🔴 와이파이 미연결] 현재 오프라인 상태입니다. (센서 정상 가동 중)");
      WiFi.disconnect();
      WiFi.begin(ssid, password); // 자동으로 재연결 시도
    }
  }

  // [2. mmWave 레이더 센서 데이터 수신]
  while (mmUART.available() > 0)  // ESP32-S3 수신 버퍼에 센서 데이터가 있다면
  {
    char c = mmUART.read(); // 버퍼에서 대기 중인 문자 한 개를 실시간으로 즉시 낚아채서 변수에 저장
    if (c == '\n')          // 만약 읽어온 문자가 문장의 끝을 알리는 줄바꿈 문자(\n)라면 완벽한 문장이 완성된 것임
    { 
      processRadarData(inputBuffer); // 완성된 한 문장의 데이터 덩어리를 파싱 함수로 토스하여 움직임 정밀 분석 처리
      inputBuffer = "";              // 다음 문장을 새롭게 깨끗이 수신하기 위해 임시 문자열 버퍼를 초기화 
    } else           
    { 
      inputBuffer += c;              // 버퍼 문자열의 꼬리에 현재 문자를 하나씩 연쇄적으로 이어 붙임 
    }
  }

  // [2. mmWave 움직임 판단 결과 갱신]
  if (millis() - lastMoveTime < motionTimeout && lastMoveTime > 0)  // 마지막 움직임 감지 시간으로부터 아직 2초가 지나지 않았고, 감지 기록이 있다면 실시간 움직임 활성화 상태 유지
  {
    isMoving = true;  // 대시보드 플래그 참(활성) 전환
  } else                                                            // 마지막 움직임 포착 후 아무런 움직임 없이 2초가 흘러버렸다면, 사물이 정지한 것으로 판단      
  { 
    isMoving = false; // 대시보드 플래그 거짓(비활성) 전환
  }

  // [3. MAX9814 마이크로폰 소리 세기 측정 (50ms 샘플링)]
  unsigned long micStartMillis = millis();
  unsigned int signalMax = 0;
  unsigned int signalMin = 4095;

  while (millis() - micStartMillis < sampleWindow)
  {
    unsigned int sample = analogRead(maxout);
    if (sample < 4095)
    {
      if (sample > signalMax) signalMax = sample;
      if (sample < signalMin) signalMin = sample;
    }
  }

  // [3. MAX9814, 전역 변수에 실시간 Peak-to-Peak 값 동기화]
  globalPeakToPeak = signalMax - signalMin; 

  // [3. MAX9814 소리 데이터 노이즈 필터 및 3단계 판정]
  if (globalPeakToPeak >= 1000) // 1000 이상으로 튀는 노이즈는 무시하고 이전의 soundStatus 상태 유지
  {
  } 
  else
  {
    if (globalPeakToPeak >= 100 && globalPeakToPeak < 150)       // 100 이상 150 미만이면 조용함으로 판단
    {
      soundStatus = "조용함";
    }
    else if (globalPeakToPeak >= 150 && globalPeakToPeak <= 400) // 150 이상 400 이하라면 적당함으로 판단
    {
      soundStatus = "적당함";
    }
    else if (globalPeakToPeak > 400 && globalPeakToPeak < 1000)  // 400 초과 1000 미만이면 활발함으로 판단
    {
      soundStatus = "활발함";
    }
    else
    {
      soundStatus = "정적";                                       // 100 미만이면 정적으로 판단
    }
  }

  // [4. BH1750 실시간 조도 측정]
  globalLux = lightMeter.getLux(); // 센서 데이터 완성시, 측정된 조도 값(Lux)을 가져옴
  lightMeter.start();              // 다음 측정을 위해, 센서 재가동 명령
  
  if (globalLux >= 80.0)
  {
    luxStatus = "밝음";            // 측정값이 80 lx 이상이면 밝음 출력
  }
  else
  {
    luxStatus = "어두움";          // 그렇지 않으면 어두움 출력
  }


  // [1. ToF 인원 카운팅 알고리즘]
  vl53l0x_single_test(&vl53dev, &vl53data);   // 실시간 레이저 거리 측정 명령을 내려 반사 시간 데이터 수집
  int currentDistance = vl53data.RangeMilliMeter; // 획득한 거리 결과값을 mm 형태의 정밀 정수로 변환

  if (currentDistance > 2000 || currentDistance < 20)  // 측정거리가 2m를 초과해 튀어버리거나 너무 가까운 값이 측정되면
  { 
    currentDistance = baselineDistance;                // 알고리즘 연산 꼬임을 막기 위해 강제로 학습된 기준 거리로 치환
  }

  static int entryDistance = 0;      // 사람이 센서 시야에 처음 들어왔을 때의 최초 진입 거리를 기억할 내부 정적 변수
  static int lastValidDistance = 0;  // 센서 시야를 완전히 벗어나기 직전까지 유효했던 마지막 측정 거리를 기억할 내부 정적 변수
  
  if (currentAlgorithmState == 0)                             // [상태 0: 센서 구역에 사물이 없는 상태]
  { 
    if ((baselineDistance - currentDistance) > tofThreshold)  // (평소 거리 - 현재 측정 거리) 즉, 사물이 문턱값(15cm) 이상 움직였다면
    {
      entryDistance = currentDistance;                        // 물체가 포착된 최초의 시점 거리를 기록    
      lastValidDistance = currentDistance;                    // 탈출 직전 거리 계산을 위한 최초 비교값으로 동시 대입
      currentAlgorithmState = 1;                              // 상태를 사물이 통과 중인 상황인 [상태 1]로 변경
      lastStateTime = millis();                               // 락 현상 대비 감지 시작 시점의 가동 시간을 타임스탬프 처리
    }
  } 
  else if (currentAlgorithmState == 1)                               // [상태 1: 사물이 감지되어 현재 센서 구역 내부에서 움직이는 상태]
  { 
    if ((baselineDistance - currentDistance) > (tofThreshold - 50))  // 센서 시야에서 벗어나기 전, 문턱값(10cm) 내에 사물이 여전히 머물고 있다면,
    { 
      lastValidDistance = currentDistance;                                                                         // 사물의 사선 이동 경로를 추적하기 위해 직전 유효 거리를 계속 현재 값으로 실시간 동기화 업데이트         
    }

    if ((baselineDistance - currentDistance) < (tofThreshold - 50))  // 사물이 센서 시야를 완전히 통과하여 거리가 평균값으로 복구되면,
    { 
      int exitDistance = lastValidDistance;                                                                          // 감지 유효 범위 내에서 마지막으로 기억했던 거리를 최종 탈출 직전 거리로 확정          

      // 궤적 변동폭 검증용 Serial Monitor에 데이터 로그 출력
      Serial.println("\n----- 45도 설치 기준 궤적 판정 로그 -----");
      Serial.printf("평균 바닥 거리 : %d mm\n", baselineDistance);
      Serial.printf("입장 시점 거리 : %d mm\n", entryDistance);
      Serial.printf("퇴장 시점 거리 : %d mm\n", exitDistance);
      Serial.println("-------------------------------------");

      if (exitDistance - entryDistance > 80) // [입장 판정 연산] '가까이 진입해서 점점 멀어졌다'
      { 
        totalVisitorCount++;                 // 누적   방문자 수 +1     
        currentPeopleCount++;                // 재실 인원수 +1
        Serial.printf("[결과] 🏃 입장 | 현재 인원: %d명\n", currentPeopleCount);
      } 
      else if (entryDistance - exitDistance > 80) // [퇴장 판정 연산] '멀리서 진입하여 점점 가까워진다.'
      {
        if (currentPeopleCount > 0)
        {
          currentPeopleCount--;                   // 0명일 때만, 재실 인원수 -1
        }
        else
        {
          currentPeopleCount = 0;                 // 음수 카운트 방지
        }          
        Serial.printf("[결과] 🚶 퇴장 | 현재 인원: %d명\n", currentPeopleCount);
      } 
      else                                        // 변화량이 크지 않으면 노이즈로 판정하여 카운트 제외
      { 
        Serial.println("[🔴 경고] 변화량이 적어 카운트 제외 (노이즈 판정)\n");
      }
      currentAlgorithmState = 0;                  // 입장 및 퇴장에 대한 판정과 연산 처리가 끝났으므로, 다음 감지를 위해 대기 상태인 0으로 설정
    }

    if (millis() - lastStateTime > TIMEOUT_MS)    // 2초 이상 센서 시야에서 벗어나지 않으면 강제로 상태를 초기화
    {
      currentAlgorithmState = 0;                  // 대기 상태로 전환하여 시스템 락 현상 방지
    }  
  }

  // [5. 공간의 숨결 데이터를 위한 센서별 정규화]

  // 1. 인원 점수화
  float scorePeople = (currentPeopleCount / 10.0) * 100.0;    // 공간 규모에 맞게 분모 조절 필요, 현재는 10.0명 기준
  if (scorePeople > 100.0) scorePeople = 100.0; 

  // 2. 소음 점수화
  float scoreNoise = map(globalPeakToPeak, 50, 600, 0, 100);  // 공간 소음 정도에 맞게 조절 필요, 현재는 '활발함' 판단 기준인 600 기준
  if (scoreNoise > 100.0) scoreNoise = 100.0;
  if (scoreNoise < 0.0) scoreNoise = 0.0;

  // 3. 조도 점수화
  float scoreLux = (globalLux / 350.0) * 100.0;               // 공간 조도 정도에 맞게 조절 필요, 현재는 일반 가정집 LED 밝기 350 lx 기준
  if (scoreLux > 100.0) scoreLux = 100.0;

  // 4. 최종 공간의 숨결 지수 연산
  int baseScore = (int)((scorePeople * 0.4) + (scoreNoise * 0.4) + (scoreLux * 0.2)); // 재실 인원 40% + 소음 40% + 조도 20%

  // 5. mmWave 레이더 움직임 점수 추가
  if (isMoving)
  {
    baseScore += 10;                                          // 움직임이 있으면 활력 점수 +10점 부가
  }
  
  if (baseScore > 100) baseScore = 100;                       // 최종 점수가 100점이 넘지 않도록 제한
  breathOfSpaceIndex = baseScore;                             // 전역 변수에 점수 대입


  // [0. 서버 설정]
  if (millis() - lastSendTime >= sendInterval) 
  {
    lastSendTime = millis();
    sendDataToBackend(); // 3초 주기로 서버에 데이터 전송
  }

  // [0. Serial Monitor 확인용 디버그 출력]
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 1000) // 1초마다 결과 출력
  {
    lastPrintTime = millis();
    Serial.printf("[로컬 모니터링] 재실: %d명 | 누적: %d명 | 움직임: %s | 소음: %s (P-P: %d) | 조도: %.0f lx | 숨결 지수: %d점\n",
                  currentPeopleCount, totalVisitorCount, isMoving ? "활성화" : "비활성화", soundStatus.c_str(), globalPeakToPeak, globalLux, breathOfSpaceIndex);
  }
}

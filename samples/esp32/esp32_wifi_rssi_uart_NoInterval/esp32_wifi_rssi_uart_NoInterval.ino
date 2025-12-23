#include "WiFi.h"

#define TX2 17
#define RX2 16

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX2, TX2);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  Serial.println("\n[ESP32] RSSI Max Speed Mode Started");
  
  // 첫 번째 스캔 시작 (비동기)
  WiFi.scanNetworks(true); 
}

void loop() {
  // 스캔 상태 확인
  int n = WiFi.scanComplete();

  // n >= 0 이면 스캔이 완료된 것임
  if (n >= 0) {
    Serial.printf("\n--- Scan Finished: %d networks found ---\n", n);

    for (int i = 0; i < n; ++i) {
      // 드라이버 sscanf용 데이터 포맷
      String payload = "SSID:" + WiFi.SSID(i) + ",RSSI:" + String(WiFi.RSSI(i)) + "\n";
      
      Serial2.print(payload); // 라즈베리 파이 드라이버로 전송
      Serial.print(payload);   // PC 시리얼 모니터 확인용
      
      delay(2); // UART 전송 안정성을 위한 최소한의 딜레이
    }

    // 스캔 결과 삭제 (메모리 확보)
    WiFi.scanDelete();

    // 지체 없이 즉시 다음 스캔 시작!
    WiFi.scanNetworks(true);
    Serial.println("Next scan started immediately...");
  }

  // 루프를 돌면서 다른 작업을 할 수 있는 구조입니다.
  // 여기서는 아무것도 안 하고 스캔 완료만 기다립니다.
}
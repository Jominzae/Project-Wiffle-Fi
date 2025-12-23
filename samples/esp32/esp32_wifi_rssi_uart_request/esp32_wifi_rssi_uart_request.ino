#include "WiFi.h"

void setup() {
  // UART 통신 시작 (라즈베리 파이와 보드레이트 일치 필수)
  Serial.begin(115200);

  // WiFi를 스테이션 모드로 설정하고 기존 연결 해제
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  // Serial.println("Setup Done. Waiting for GETRSSI command...");
}

void loop() {
  // UART로 데이터가 들어왔는지 확인
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    // 명령어가 "GETRSSI"인 경우 주변 모든 WiFi 스캔 시작
    if (command == "GETRSSI") {
      // Serial.println("SCAN_START"); // 스캔 시작 알림 (필요 시 활용)
      
      // WiFi 스캔 실행 (동기 방식)
      int n = WiFi.scanNetworks();
      
      if (n == 0) {
        Serial.println("NO_NETWORKS_FOUND");
      } else {
        // 찾은 WiFi 개수만큼 반복하며 정보 전송
        for (int i = 0; i < n; ++i) {
          // 데이터 포맷: "SSID:이름,RSSI:값"
          Serial.print("SSID:");
          Serial.print(WiFi.SSID(i));
          Serial.print(",RSSI:");
          Serial.println(WiFi.RSSI(i));
          delay(10);
        }
      }
      // Serial.println("SCAN_END"); // 스캔 종료 알림 (필요 시 활용)
    }
  }
}
//******************** Includes ********************//
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <esp_timer.h>
#include <Arduino.h>

//******************** Macros ********************//
// Controle de tempo
#define MAXTIMERADC 20
#define MAXTIMEROUT 250
#define MAXTIMERPRINT 3000
#define MINRPM 500
#define DEBOUNCETIME 10000
#define NLEITURAS 50

// Filtros
#define FACTORADC 0.01
#define KP 64.00
#define KI 0.0001

// Pinos
#define A1 34         // Entrada analógica para o NTC 1 = 36
#define A2 34         // Entrada analógica para o NTC 2 = 39
#define A3 34         // Entrada analógica para o NTC 3 = 34
#define A4 36         // Entrada analógica para o NTC 4 = 35
#define A5 36         // Entrada analógica para o NTC 5 = 32
#define A6 36         // Entrada analógica para o NTC 6 = 33
#define RPM 13        // Entrada de RPM Coller
#define PWM_A 19      // Saída lado A 13
#define PWM_B 4      // Saída lado B 17
#define Sel_A 18      // Saída Q/F A 18
#define Sel_B 16      // Saída Q/F B 16
#define Pump_A 17      // Saída Bomba A 4
#define Pump_B 15     // Saída Bomba B 15
#define LED 23        // Saída LED 

// Parâmetros NTC
#define BETA 3950.0               //Beta do NTC
#define TEMPREF 298.15            // 25°C em Kelvin
#define RESISTENCIAREF 10000.0    // NTC 10k ohms
#define RESISTENCIASERIE 10000.0  // Resistor série utilizado

//******************** Type deff enum ********************//
enum SensorIndex {
  RET_A,
  RET_B,
  OUT_A,
  OUT_B,
  DISS,
  AMB,
  NUM_SENSORES
};

enum Mode {
  OFF,
  AUTO,
  SIDE_A,
  SIDE_B,
  ALL,
  MODE
};

Mode currentMode = OFF;


//******************** Variáveis globais ********************//
// Wifi
const char* ssid = "M&M";
const char* password = "39402100";

// Web server na porta 80
WebServer server(80);
TaskHandle_t webServerTaskHandle = NULL;

// Pinos analógicos
const int pinosADC[NUM_SENSORES] = {A1, A2, A3, A4, A5, A6};

// Setpoints
float setpointA = 24.0;
float setpointB = 26.0;

float filTemp[NUM_SENSORES] = {25.0};
float temp[NUM_SENSORES]    = {25.0};
float Ia = 0.00;
float Ib = 0.00;

uint16_t rpmVector[NLEITURAS] = {0};

// Controle de tempo
uint32_t timerControl = 0;
uint32_t timerADC = 0;
uint32_t timerOut = 0;
uint32_t timerPrint = 0;
uint32_t pulse = 0;
uint32_t lastPulse = 0;

uint16_t state = 0;
uint16_t rpm = 0;
uint16_t rpmMedia = 0;

int16_t PIa = 0;
int16_t PIb = 0;

bool flagOutA = false;
bool flagOutB = false;
bool flagSensorA = false;
bool flagSensorB = false;
bool power = false;
bool fanFlag = false;
volatile bool pulseDetected = false;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;


//******************** Timer de interrupção ********************//
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void onTimer(void* arg) {
  if (timerControl) timerControl--;
  if (timerADC) timerADC--;
  if (timerOut) timerOut--;
  if (timerPrint) timerPrint--;
  if (state < 3000) state++;
  else state = 0;
}

void IRAM_ATTR counter() {
  portENTER_CRITICAL_ISR(&mux);
  pulseDetected = true;
  portEXIT_CRITICAL_ISR(&mux);
}

//******************** Funções OTA e WebServer ********************//
void handleRoot() {
  String html = R"rawliteral(
  <html>
  <head>
    <style>
      body {
        background-color: #121212;
        color: #e0e0e0;
        font-family: Arial, sans-serif;
      }

      h2, h3 {
        color: #ffffff;
      }

      select, button {
        background-color: #333;
        color: #fff;
        border: 1px solid #555;
        border-radius: 5px;
        padding: 5px 10px;
        margin: 5px;
      }

      span, label {
        font-weight: bold;
      }

      ul {
        list-style: none;
        padding: 0;
      }

      li {
        margin-bottom: 8px;
      }

      div {
        margin: 10px 0;
      }

      progress {
        width: 150px;
        height: 15px;
        vertical-align: middle;
        appearance: none;
        -webkit-appearance: none;
      }

      .progress-red::-webkit-progress-value {
        background-color: red;
      }

      .progress-blue::-webkit-progress-value {
        background-color: blue;
      }

      .progress-red::-moz-progress-bar {
        background-color: red;
      }

      .progress-blue::-moz-progress-bar {
        background-color: blue;
      }

    </style>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <script>
      let setpointA = 0, setpointB = 0;

      function updatePage() {
        fetch('/data')
          .then(res => res.json())
          .then(data => {
            setpointA = data.setpointA;
            setpointB = data.setpointB;

            document.getElementById('POWER').innerText = data.power ? 'On' : 'Off';
            document.getElementById('modeSelect').value = data.MODE;
            document.getElementById('setA').innerText = setpointA.toFixed(1);
            document.getElementById('setB').innerText = setpointB.toFixed(1);

            const labels = ['RET_A','RET_B','OUT_A','OUT_B','DISS','AMB'];
            data.temps.forEach((t, i) => {
              document.getElementById(labels[i]).innerText = t.toFixed(1) + ' °C';
            });

            const progressA = document.getElementById('progress_OUTPUT_A');
            progressA.value = data.OUTPUT_A;
            progressA.className = data.SEL_A ? 'progress-red' : 'progress-blue';

            const progressB = document.getElementById('progress_OUTPUT_B');
            progressB.value = data.OUTPUT_B;
            progressB.className = data.SEL_B ? 'progress-red' : 'progress-blue';
            

            //document.getElementById('progress_OUTPUT_A').value = data.OUTPUT_A;
            //document.getElementById('text_OUTPUT_A').innerText = data.OUTPUT_A.toFixed(1) + ' %';

            //document.getElementById('progress_OUTPUT_B').value = data.OUTPUT_B;
            //document.getElementById('text_OUTPUT_B').innerText = data.OUTPUT_B.toFixed(1) + ' %';

            document.getElementById('PUMP_A').innerText = data.PUMP_A ? 'ON' : 'OFF';
            document.getElementById('PUMP_B').innerText = data.PUMP_B ? 'ON' : 'OFF';
            document.getElementById('PUMP_C').innerText = data.PUMP_C ? 'ON' : 'OFF';
            document.getElementById('COOLER').innerText = data.COOLER ? 'ON' : 'OFF';
          });
      }

      function changeSetpoint(side, delta) {
        if (side === 'A') setpointA += delta;
        else setpointB += delta;
        sendValues();
      }

      function sendValues(mode = null) {
        const form = new FormData();
        form.append('setpointA', setpointA);
        form.append('setpointB', setpointB);

        if (mode !== null) {
          form.append('MODE', mode);
        }

        fetch('/setValues', { method: 'POST', body: form }).then(() => updatePage());
      }

      function setMode() {
        const selectedMode = document.getElementById("modeSelect").value;
        sendValues(selectedMode);
      }

      setInterval(updatePage, 1000);
      window.onload = updatePage;
    </script>
  </head>
  <body>
    <h2>Night Pro</h2>

    <div>
      Power: <span id="POWER">--</span>
    </div>

    <div>
      <label for="modeSelect">Modo:</label>
      <select id="modeSelect" onchange="setMode()">
        <option value="OFF">OFF</option>
        <option value="AUTO">AUTO</option>
        <option value="SIDE_A">SIDE_A</option>
        <option value="SIDE_B">SIDE_B</option>
        <option value="ALL">ALL</option>
      </select>
    </div>

    <div>
      Setpoint A:
      <button onclick="changeSetpoint('A', -0.5)">-</button>
      <span id="setA">--</span>
      <button onclick="changeSetpoint('A', 0.5)">+</button>
    </div>

    <div>
      Setpoint B:
      <button onclick="changeSetpoint('B', -0.5)">-</button>
      <span id="setB">--</span>
      <button onclick="changeSetpoint('B', 0.5)">+</button>
    </div>

    <h3>Temperaturas</h3>
    <ul>
      <li>RET_A: <span id="RET_A">--</span></li>
      <li>RET_B: <span id="RET_B">--</span></li>
      <li>OUT_A: <span id="OUT_A">--</span></li>
      <li>OUT_B: <span id="OUT_B">--</span></li>
      <li>DISS: <span id="DISS">--</span></li>
      <li>AMB: <span id="AMB">--</span></li>
    </ul>

    <h3>Saídas</h3>
    <ul>
      <li>OUTPUT_A: <progress id="progress_OUTPUT_A" max="100" value="0" class="progress-blue"></progress></li>
      <li>OUTPUT_B: <progress id="progress_OUTPUT_B" max="100" value="0" class="progress-blue"></progress></li>
      <li>PUMP_A: <span id="PUMP_A">--</span></li>
      <li>PUMP_B: <span id="PUMP_B">--</span></li>
      <li>PUMP_C: <span id="PUMP_C">--</span></li>
      <li>COOLER: <span id="COOLER">--</span></li>
    </ul>
  </body>
  </html>
  )rawliteral";

  server.send(200, "text/html; charset=utf-8", html);
}

void handleData() {
  String json = "{";
  json += "\"MODE\":\"" + String(modeToString(currentMode)) + "\",";
  json += "\"setpointA\":" + String(setpointA, 1) + ",";
  json += "\"setpointB\":" + String(setpointB, 1) + ",";
  json += "\"power\":" + String(power ? "true" : "false") + ",";
  json += "\"temps\":[";

  for (int i = 0; i < NUM_SENSORES; i++) {
    json += String(temp[i], 1);
    if (i < NUM_SENSORES - 1) json += ",";
  }

  json += "],";

  json += "\"OUTPUT_A\":" + String(PIa / 2.55);
  json += ",\"OUTPUT_B\":" + String(PIb / 2.55);
  json += ",\"SEL_A\":" + String(digitalRead(Sel_A));
  json += ",\"SEL_B\":" + String(digitalRead(Sel_B));
  json += ",\"PUMP_A\":" + String(digitalRead(Pump_A));
  json += ",\"PUMP_B\":" + String(digitalRead(Pump_B));
  json += ",\"PUMP_C\":" + String(digitalRead(Pump_A));
  json += ",\"COOLER\":" + String(digitalRead(Pump_B));

  json += "}";


  server.send(200, "application/json", json);
}

void handleSetValues() {
  if (server.hasArg("MODE")) {
    String modeStr = server.arg("MODE");
    if      (modeStr == "OFF")      currentMode = OFF;
    else if (modeStr == "AUTO")     currentMode = AUTO;
    else if (modeStr == "SIDE_A")   currentMode = SIDE_A;
    else if (modeStr == "SIDE_B")   currentMode = SIDE_B;
    else if (modeStr == "ALL")      currentMode = ALL;
  }

  if (server.hasArg("setpointA")) setpointA = server.arg("setpointA").toFloat();
  if (server.hasArg("setpointB")) setpointB = server.arg("setpointB").toFloat();
  
  if (setpointA > 30) setpointA = 30.00;
  else if (setpointA < 16) setpointA = 16.00;

  if (setpointB > 30) setpointB = 30.00;
  else if (setpointB < 16) setpointB = 16.00;

  server.send(200, "text/plain", "OK");
}

String modeToString(int mode) {
  switch (mode) {
    case 0: return "OFF";
    case 1: return "AUTO";
    case 2: return "SIDE_A";
    case 3: return "SIDE_B";
    case 4: return "ALL";
    default: return "UNKNOWN";
  }
}

void webServerLoop(void *pvParameters) {
  for (;;) {
    server.handleClient();
    ArduinoOTA.handle();
    vTaskDelay(1); // Pequeno delay para não travar o core
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(RPM, INPUT_PULLUP);
  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(Sel_A, OUTPUT);
  pinMode(Sel_B, OUTPUT);
  pinMode(Pump_A, OUTPUT);
  pinMode(Pump_B, OUTPUT);
  pinMode(LED, OUTPUT);

  ledcSetup(0, 20000, 8);  // Define a frequência
  ledcAttachPin(PWM_A, 0);

  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
  
  // Inicializa WIFI e aguarda conexão
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  ArduinoOTA.begin();

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/setValues", HTTP_POST, handleSetValues);
  server.begin();

  xTaskCreatePinnedToCore(
    webServerLoop,     // função da task
    "WebServerTask",   // nome da task
    10000,              // stack size
    NULL,               // parâmetro
    1,                  // prioridade
    &webServerTaskHandle, // handle da task
    0                   // núcleo (0 = geralmente reservado p/ Wi-Fi)
  );

  // Timer periódico 1ms
  esp_timer_create_args_t timer_args = {
    .callback = &onTimer,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "1ms_timer"
  };

  esp_timer_handle_t periodic_timer;
  esp_timer_create(&timer_args, &periodic_timer);
  esp_timer_start_periodic(periodic_timer, 1000);

  attachInterrupt(digitalPinToInterrupt(RPM), counter, FALLING);


  if (!MDNS.begin("esp32")) {
    Serial.println("Erro ao iniciar mDNS");
    return;
  }
  
  MDNS.addService("http", "tcp", 80);
  Serial.println("mDNS iniciado: http://esp32.local");
}

void loop() {
  
  if(!timerControl) checkMode();
  if(!timerADC) readADC();
  if(!timerOut) outControl();
  if(!timerPrint) serialPrint();
  if (pulseDetected) readRPM();
  else if((micros() - lastPulse) > 1000000) readRPM(); // Atualiza RPM mesmo sem pulso a cada 1000 ms
 
}

void checkMode() {
  switch (currentMode) {
    case OFF:
    // Desligado
    power = false;
    flagOutA = false;
    flagOutB = false;

    break;

    case AUTO:
    // Ativacao automatica?
    if (flagSensorA || flagSensorB) power = true;
    else power = false;

    if (flagSensorA) {
      flagOutA = true;
    }
    else flagOutA = false;

    if (flagSensorB) {
      flagOutB = true;
    }
    else flagOutB = false;
    
    break;

    case SIDE_A:
    // Somente lado A operando
    power = true;
    flagOutA = true;
    flagOutB = false;

    break;

    case SIDE_B:
    // Somente lado B operando
    power = true;
    flagOutA = false;
    flagOutB = true;

    break;

    case ALL:
    // Ambos os lados operando
    power = true;
    flagOutA = true;
    flagOutB = true;

    break;
  }

}

void readADC() {
  for (int i = 0; i < NUM_SENSORES; i++) {
    int leituraADC = analogRead(pinosADC[i]);
    float tensao = leituraADC * (3.3 / 4095.0);

    if (tensao <= 0.01) continue;

    float resistenciaNTC = (3.3 * RESISTENCIASERIE / tensao) - RESISTENCIASERIE;
    float temperaturaK = 1.0 / (1.0 / TEMPREF + (1.0 / BETA) * log(resistenciaNTC / RESISTENCIAREF));
    float temperaturaAtual = temperaturaK - 273.15;

    filTemp[i] = (1.0 - FACTORADC) * filTemp[i] + FACTORADC * temperaturaAtual;
    temp[i] = filTemp[i];
  }

  timerADC = MAXTIMERADC;
}

void outControl() {
  // **************** Controle das Saidas Peltier ****************
  int16_t Pa = (temp[RET_A] - setpointA) * KP;
  int16_t Pb = (temp[RET_B] - setpointB) * KP;

  Ia += Pa * KI;
  Ib += Pb * KI;

  if (Ia > 255) Ia = 255.00;
  else if (Ia < -255) Ia = -255.00;

  if (Ib > 255) Ib = 255.00;
  else if (Ia < -255) Ia = -255.00;

  PIa = Pa + Ia;
  PIb = Pb + Ib;

  if (PIa > 255) PIa = 255;
  else if (PIa < -255) PIa = -255;

  if (PIb > 255) PIb = 255;
  else if (PIb < -255) PIb = -255;

 // Controle da saída A
  if (flagOutA) {
    if (PIa < 0) {
      PIa = abs(PIa); // Converte valor negativo
      digitalWrite(Sel_A, HIGH); // Liga saida para aquecer
    }
    else digitalWrite(Sel_A, LOW);
    analogWrite(PWM_A, PIa); // Liga saida peltier A
    //if (PIa > 125) digitalWrite(PWM_A, HIGH);
    //else digitalWrite(PWM_A, LOW);
  } 
  else {  // Desliga saidas
    analogWrite(PWM_A, 0);
    //digitalWrite(PWM_A, LOW);
    digitalWrite(Sel_A, LOW);
  } 

  // Controle da saída B
  if (flagOutB) {
    if (PIb < 0) {
      PIb = abs(PIb); // Converte valor negativo
      digitalWrite(Sel_B, HIGH); // Liga saida para aquecer
    }
    else digitalWrite(Sel_B, LOW);
    analogWrite(PWM_B, PIb);  // Liga saida peltier B
  }
  else {  // Desliga saidas
    analogWrite(PWM_B, 0);
    digitalWrite(Sel_B, LOW);
  } 
  // **************** Controle das Saidas das Bombas ****************
  if (PIa >= 200) {
    analogWrite(Pump_A, 255);
  }
  else if (PIa >= 50 && PIa < 200) {
    analogWrite(Pump_A, 128);
  }
  else if (PIa && PIa < 50) {
    analogWrite(Pump_A, 100);
  }
  else analogWrite(Pump_A, 0);


  if (PIb || PIb < 200) {
    analogWrite(Pump_B, 128);
  }
  else if (PIb >= 200) {
    analogWrite(Pump_B, 255);
  }
  else analogWrite(Pump_B, 0);

  
  
  int8_t dif = (temp[DISS] / temp[AMB]) * 10; // logica para acionamento do cooler de maneira digital
  if (dif != 10) digitalWrite(LED, HIGH);
  else digitalWrite(LED, LOW);


  // **************** Controle Led Indicador ****************
  if (state < 150 || fanFlag) digitalWrite(LED,HIGH);
  else digitalWrite(LED, LOW);

  timerOut = MAXTIMEROUT;
}

void readRPM() {

  portENTER_CRITICAL(&mux);
  pulseDetected = false;
  portEXIT_CRITICAL(&mux);

  unsigned long now = micros();
  pulse = now - lastPulse;
  lastPulse = now;

  rpm = (1e6 / pulse) * 30;

  static uint8_t indiceLeitura = 0;

  rpmVector[indiceLeitura] = rpm;

  if(indiceLeitura < NLEITURAS) indiceLeitura++;
  else indiceLeitura = 0;

  int soma;
  for(int i = 0; i < NLEITURAS; i++){
    soma += rpmVector[i];
  }

  rpmMedia = soma / NLEITURAS;

  if (rpmMedia <= MINRPM) fanFlag = true;
  else fanFlag = false;
  
}

void serialPrint() {
   Serial.println("Temperaturas:");
  for (int i = 0; i < NUM_SENSORES; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(temp[i]);
    Serial.println(" °C");

  }
  Serial.print("RPM: ");
  Serial.println(rpmMedia);
  Serial.print("PIa: ");
  Serial.println(PIa);
  Serial.print("PIb: ");
  Serial.println(PIb);


  Serial.println();

  timerPrint = MAXTIMERPRINT;
}


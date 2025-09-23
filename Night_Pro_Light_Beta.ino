//******************** Includes ********************//
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <esp_timer.h>

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
#define KP 128.00
#define KI 0.01

// Pinos
#define A1 36         // Entrada analógica para o NTC 1 = 36
#define A2 36         // Entrada analógica para o NTC 2 = 39
#define A3 39         // Entrada analógica para o NTC 3 = 34
#define A4 34         // Entrada analógica para o NTC 4 = 35
#define A5 32         // Entrada analógica para o NTC 5 = 32
#define A6 33         // Entrada analógica para o NTC 6 = 33
#define RPM 13        // Entrada de RPM Coller
#define PWM_A 19      // Saída lado A 19
#define PWM_B 4      // Saída lado B 17
#define Sel_A 18      // Saída Q/F A 18
#define Sel_B 16      // Saída Q/F B 16
#define Pump_A 15      // Saída Bomba A 4
#define Pump_B 17     // Saída Bomba B 15
#define LED 23        // Saída LED 23

// Parâmetros NTC
#define BETA 3950.0               //Beta do NTC
#define TEMPREF 298.15            // 25°C em Kelvin
#define RESISTENCIAREF 10000.0    // NTC 10k ohms
#define RESISTENCIASERIE 10000.0  // Resistor série utilizado

//******************** Type deff enum ********************//
enum SensorIndex {
  RET_A,
  OUT_A,
  DISS,
  AMB,
  NUM_SENSORES
};

enum Mode {
  OFF,
  AUTO,
  MANUAL,
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
const int pinosADC[NUM_SENSORES] = {A1, A2, A3, A4};

// Setpoints
float setpointA = 24.0;

float filTemp[NUM_SENSORES] = {25.0};
float temp[NUM_SENSORES]    = {25.0};
float Ia = 0.00;

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

uint8_t Pump_A_val = 0;
uint8_t Pump_B_val = 0;

bool flagOutA = false;
bool flagSensorA = false;
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
      let setpointA = 0;

      function updatePage() {
        fetch('/data')
          .then(res => res.json())
          .then(data => {
            setpointA = data.setpointA;

            document.getElementById('POWER').innerText = data.power ? 'On' : 'Off';
            document.getElementById('modeSelect').value = data.MODE;
            document.getElementById('setA').innerText = setpointA.toFixed(1);

            const labels = ['RET_A','OUT_A','DISS','AMB'];
            data.temps.forEach((t, i) => {
              document.getElementById(labels[i]).innerText = t.toFixed(1) + ' °C';
            });

            const progressA = document.getElementById('progress_Peltier');
            progressA.value = data.OUTPUT_A;
            progressA.className = data.SEL_A ? 'progress-red' : 'progress-blue';
            

            document.getElementById('PUMP_A').innerText = data.PUMP_A + ' %';
            document.getElementById('PUMP_B').innerText = data.PUMP_B + ' %';
            document.getElementById('COOLER').innerText = data.COOLER ? 'ON' : 'OFF';
          });
      }

      function changeSetpoint(delta) {
        setpointA += delta;
        sendValues();
      }

      function sendValues(mode = null) {
        const form = new FormData();
        form.append('setpointA', setpointA);

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
        <option value="MANUAL">MANUAL</option>
      </select>
    </div>

    <div>
      Setpoint:
      <button onclick="changeSetpoint(-0.5)">-</button>
      <span id="setA">--</span>
      <button onclick="changeSetpoint(0.5)">+</button>
    </div>

    <h3>Temperaturas</h3>
    <ul>
      <li>Retorno: <span id="RET_A">--</span></li>
      <li>Saida: <span id="OUT_A">--</span></li>
      <li>Dissipador: <span id="DISS">--</span></li>
      <li>Ambiente: <span id="AMB">--</span></li>
    </ul>

    <h3>Saídas</h3>
    <ul>
      <li>Peltier: <progress id="progress_Peltier" max="100" value="0" class="progress-blue"></progress></li>
      <li>Bomba Cama: <span id="PUMP_A">--</span></li>
      <li>Bomba Dissipador: <span id="PUMP_B">--</span></li>
      <li>Cooler: <span id="COOLER">--</span></li>
    </ul>
  </body>
  </html>
  )rawliteral";

  server.send(200, "text/html; charset=utf-8", html);
}

void handleData() {

  //portENTER_CRITICAL(&mux);

  String json = "{";
  json += "\"MODE\":\"" + String(modeToString(currentMode)) + "\",";
  json += "\"setpointA\":" + String(setpointA, 1) + ",";
  json += "\"power\":" + String(power ? "true" : "false") + ",";
  json += "\"temps\":[";

  for (int i = 0; i < NUM_SENSORES; i++) {
    json += String(temp[i], 1);
    if (i < NUM_SENSORES - 1) json += ",";
  }

  json += "],";

  json += "\"OUTPUT_A\":" + String(PIa / 2.55);
  json += ",\"SEL_A\":" + String(digitalRead(Sel_A));
  json += ",\"PUMP_A\":" + String(Pump_A_val / 2.55);
  json += ",\"PUMP_B\":" + String(Pump_B_val / 2.55);
  json += ",\"COOLER\":" + String(digitalRead(LED));

  json += "}";

  //portEXIT_CRITICAL(&mux);

  server.send(200, "application/json", json);
}

void handleSetValues() {
  if (server.hasArg("MODE")) {
    String modeStr = server.arg("MODE");
    if      (modeStr == "OFF")      currentMode = OFF;
    else if (modeStr == "AUTO")     currentMode = AUTO;
    else if (modeStr == "MANUAL")   currentMode = MANUAL;
  }

  if (server.hasArg("setpointA")) setpointA = server.arg("setpointA").toFloat();
  
  if (setpointA > 30) setpointA = 30.00;
  else if (setpointA < 16) setpointA = 16.00;

  server.send(200, "text/plain", "OK");
}

String modeToString(int mode) {
  switch (mode) {
    case 0: return "OFF";
    case 1: return "AUTO";
    case 2: return "MANUAL";
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
  pinMode(Sel_A, OUTPUT);
  pinMode(Pump_A, OUTPUT);
  pinMode(Pump_B, OUTPUT);
  pinMode(LED, OUTPUT);

  ledcAttach(PWM_A, 25000, 8);
  ledcAttach(Pump_A, 25000, 8);
  ledcAttach(Pump_B, 25000, 8);

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

    break;

    case AUTO:
    // Ativacao automatica?
    power = true;
    if (flagSensorA) {
      flagOutA = true;
    }
    else {
      flagOutA = false;
    }
    break;

    case MANUAL:
    // Ativacao manual
    power = true;
    flagOutA = true;

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
  // **************** Controle da Saida Peltier ****************
  int16_t Pa = 0;
  
  if (flagOutA) Pa = (temp[RET_A] - setpointA) * KP;
  else Pa = 0;

  Ia += Pa * KI;

  if (Ia > 255) Ia = 255.00;
  else if (Ia < -255) Ia = -255.00;

  PIa = Pa + Ia;

  if (PIa > 255) PIa = 255;
  else if (PIa < -255) PIa = -255;

 // Controle da saída
  if (flagOutA) {
    if (PIa < 0) {
      PIa = abs(PIa); // Converte valor negativo
      digitalWrite(Sel_A, HIGH); // Liga saida para aquecer
    }
    else digitalWrite(Sel_A, LOW);
    
    ledcWrite(PWM_A, PIa); // Liga saida peltier
  } 
  else {  // Desliga saida
    ledcWrite(PWM_A, 0);
    digitalWrite(Sel_A, LOW);
    PIa = 0;
  } 

  // **************** Controle das Saidas das Bombas ****************
  int8_t dif = (temp[DISS] / temp[AMB]) * 10; // logica para acionamento do cooler

  if (PIa >= 200) {
    Pump_A_val = 255;
  }
  else if (PIa >= 50 && PIa < 200) {
    Pump_A_val = 128;
  }
  else {
    Pump_A_val = 51;
  } 

  if (PIa >= 50) {
    Pump_B_val = 255;
  }
  else if (PIa >= 5 && PIa < 50) {
    Pump_B_val = 127;
  }
  else {
    if (dif >= 12 || dif <= 8) { //percentual de diferenca de temperatura entre dissipador e ambiente 8 e 12 = 20%
      Pump_B_val = 127;
    } 
    else if (dif >= 11 || dif <= 9 || PIa) {
      Pump_B_val = 60;
    }
    else Pump_B_val = 51;
  }

  if (!power) {
    Pump_A_val = 0;
    Pump_B_val = 0;
  }

  ledcWrite(Pump_A, Pump_A_val);
  ledcWrite(Pump_B, Pump_B_val);

  // **************** Controle Led Indicador ****************
  if (state < 150 || fanFlag) digitalWrite(LED,LOW);
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
  Serial.print("Setpoint: ");
  Serial.println(setpointA);
  Serial.print("Power: ");
  Serial.println(power);
  Serial.print("Flag Out: ");
  Serial.println(flagOutA);


  Serial.println();

  timerPrint = MAXTIMERPRINT;
}


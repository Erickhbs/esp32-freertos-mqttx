#include <ESP32Servo.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Dados de rede Wi-Fi
const char* ssid = "batata";
const char* password = "123456789";

// Configuração do servidor MQTT
const char* mqtt_server = "test.mosquitto.org";

WiFiClient espClient;
PubSubClient client(espClient);

Servo myServo1;
Servo myServo2;
Servo myServo3;

TaskHandle_t entrada;
TaskHandle_t entrada2;
TaskHandle_t saida;
TaskHandle_t display;
TaskHandle_t lerSensores;
TaskHandle_t taskMQTTHandle;

SemaphoreHandle_t SMF1;

TimerHandle_t temporizador1;

#define TEMPO_DEBOUNCE 500 //ms
volatile unsigned long timestamp_ultimo_acionamento = 0;

const byte sensorEntrada = 2;
const byte sensorEntrada2 = 4;
const byte sensorSaida = 0; // usar 14 no real e 0 no simulador

const byte vaga1 = 27;
const byte vaga2 = 26;
const byte vaga3 = 25;
const byte vaga4 = 33;

const byte ledVermelho1 = 35;
const byte ledVermelho2 = 32;
const byte ledVermelho3 = 5;
const byte ledVermelho4 = 18;

const byte ledBranco1 = 12;
const byte ledBranco2 = 13;
const byte ledBranco3 = 25;
const byte ledBranco4 = 27;

volatile int vagasDisponiveis = 4;
volatile boolean alarmeAcionado = false;

// vagas disponiveis
volatile boolean sensor1 = true;
volatile boolean sensor2 = true;
volatile boolean sensor3 = true;
volatile boolean sensor4 = true;

// vagas resevadas
volatile boolean resevada1 = false;
volatile boolean resevada2 = false;
volatile boolean resevada3 = false;
volatile boolean resevada4 = false;

void entrarVeiculo(void *arg){
  while(true){
    if(digitalRead(sensorEntrada) == LOW){
      if(vagasDisponiveis > 0){
        myServo1.write(90);
        while(true){
          if(digitalRead(sensorEntrada)){          
            vTaskDelay(2000);
            myServo1.write(0);
            xSemaphoreTake(SMF1, portMAX_DELAY );
            vagasDisponiveis--;
            xSemaphoreGive(SMF1);
            break;
          }
        }
      }
    }
    vTaskDelay(250);
  }
}

void entrarVeiculo2(void *arg){
  while(true){
    if(digitalRead(sensorEntrada2) == LOW){
      if(vagasDisponiveis > 0){
        myServo3.write(90);
        while(true){
          if(digitalRead(sensorEntrada2)){          
            vTaskDelay(2000);
            myServo3.write(0);          
            xSemaphoreTake(SMF1, portMAX_DELAY );
            vagasDisponiveis--;
            xSemaphoreGive(SMF1);
            break;
          }
        }
      }
    }
    vTaskDelay(250);
  }
}

void sairVeiculo(void *arg){
  while(true){
    if(digitalRead(sensorSaida) == LOW){
      if(vagasDisponiveis < 4){
        myServo2.write(90);
        while(true){
          if(digitalRead(sensorSaida)){                    
            vTaskDelay(2000);
            myServo2.write(0);
            xSemaphoreTake(SMF1, portMAX_DELAY );
            vagasDisponiveis++;
            xSemaphoreGive(SMF1);
            break;
          }
        }
      }
    }
    vTaskDelay(500);
  }
}

void exibirDisplay(void *arg){
  while(true){
    Serial.print("Vagas disponiveis: ");
    Serial.println(vagasDisponiveis);
    vTaskDelay(250);
  }
}

void lerSensor(void *arg){
  while(true){
    sensor1 = digitalRead(vaga1);
    if(sensor1){
      digitalWrite(ledVermelho1, HIGH);
    } else{
      digitalWrite(ledVermelho1, LOW);
      resevada1 = false;
    }

    sensor2 = digitalRead(vaga2);
    if(sensor2){
      digitalWrite(ledVermelho2, HIGH);
    } else{
      digitalWrite(ledVermelho2, LOW);
      resevada2 = false;
    }

    sensor3 = digitalRead(vaga3);
    if(sensor3){
      digitalWrite(ledVermelho3, HIGH);
    } else{
      digitalWrite(ledVermelho3, LOW);
      resevada3 = false;
    }

    sensor4 = digitalRead(vaga4);
    if(sensor4){
      digitalWrite(ledVermelho4, HIGH);
    } else{
      digitalWrite(ledVermelho4, LOW);
      resevada4 = false;
    }

    // led branco
    if(resevada1){
      digitalWrite(ledBranco1, HIGH);
    } else{
      digitalWrite(ledBranco1, LOW);
    }

    if(resevada2){
      digitalWrite(ledBranco2, HIGH);
    } else{
      digitalWrite(ledBranco2, LOW);
    }

    if(resevada3){
      digitalWrite(ledBranco3, HIGH);
    } else{
      digitalWrite(ledBranco3, LOW);
    }

    if(resevada4){
      digitalWrite(ledBranco4, HIGH);
    } else{
      digitalWrite(ledBranco4, LOW);
    }

    vTaskDelay(1000);
  }
}

void alarmar(){
  vTaskSuspend(entrada);
  vTaskSuspend(entrada2);
  vTaskSuspend(saida);
  vTaskSuspend(display);

  myServo1.write(90);
  myServo2.write(90);
  myServo3.write(90);
    
  Serial.print("Interrupção acionada");
  tone(18, 262, 5000); // Toca um tom de 262Hz por 5 segundos 
}

void isrAlarme() {
  if ((millis() - timestamp_ultimo_acionamento) >= TEMPO_DEBOUNCE ){
    timestamp_ultimo_acionamento = millis();
    alarmeAcionado = true;
  }
}

void timerCallback(TimerHandle_t xTimer) {
  if(alarmeAcionado){
    alarmar();
    alarmeAcionado = !alarmeAcionado;
  }
}

void setup_wifi() {
  vTaskDelay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(500);
    Serial.print(".");
  }

  Serial.println("WiFi connected");
  Serial.println("IP address: " + String(WiFi.localIP()));
}

void setup() {
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  myServo1.attach(23);
  myServo1.write(0);
  myServo2.attach(22);
  myServo2.write(0);
  myServo3.attach(21);
  myServo3.write(0);
  
  pinMode(sensorEntrada, INPUT);
  pinMode(sensorEntrada2, INPUT);
  pinMode(sensorSaida, INPUT);

  pinMode(vaga1, INPUT);
  pinMode(vaga2, INPUT);
  pinMode(vaga3, INPUT);
  pinMode(vaga4, INPUT);

  pinMode(ledVermelho1, OUTPUT);
  pinMode(ledVermelho2, OUTPUT);
  pinMode(ledVermelho3, OUTPUT);
  pinMode(ledVermelho4, OUTPUT);

  pinMode(ledBranco1, OUTPUT);
  pinMode(ledBranco2, OUTPUT);
  pinMode(ledBranco3, OUTPUT);
  pinMode(ledBranco4, OUTPUT);
    
  Serial.begin(115200);

  attachInterrupt(16, isrAlarme, RISING);

  xTaskCreatePinnedToCore(entrarVeiculo, "entrada", 4096, NULL, 1, &entrada, 1);
  xTaskCreatePinnedToCore(entrarVeiculo2, "entrada2", 4096, NULL, 1, &entrada2, 1);
  xTaskCreatePinnedToCore(sairVeiculo, "saida", 4096, NULL, 1, &saida, 1);
  xTaskCreatePinnedToCore(exibirDisplay, "display", 4096, NULL, 1, &display, 0);
  xTaskCreatePinnedToCore(lerSensor, "lerSensores", 4096, NULL, 1, &lerSensores, 0);
  xTaskCreatePinnedToCore(taskMQTT, "MQTT Task", 4096, NULL, 1, &taskMQTTHandle, 1);

  SMF1 = xSemaphoreCreateBinary();
  xSemaphoreGive(SMF1);

  temporizador1 = xTimerCreate("temporizador1", pdMS_TO_TICKS(2000), pdTRUE, (void*)0, timerCallback);
  xTimerStart(temporizador1, 0);
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  
  char *token = strtok((char*)payload, "_");
  if (token != NULL) vagasDisponiveis = atoi(token);

  token = strtok(NULL, "_");
  if (token != NULL) {
    sensor1 = token[0] - '0';
    sensor2 = token[1] - '0';
    sensor3 = token[2] - '0';
    sensor4 = token[3] - '0';
  }

  token = strtok(NULL, "_");
  if (token != NULL) {
    resevada1 = token[0] - '0';
    resevada2 = token[1] - '0';
    resevada3 = token[2] - '0';
    resevada4 = token[3] - '0';
  }

  Serial.println("Message received: " + message);
}

void taskMQTT(void *pvParameters) {
  while (true) {

    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    
    // Publicar status das vagas e sensores
    String message = String(vagasDisponiveis) + "_" +
                     String(sensor1) + String(sensor2) + String(sensor3) + String(sensor4) + "_" +
                     String(resevada1) + String(resevada2) + String(resevada3) + String(resevada4);
    
    client.publish("parking/vagas", message.c_str());

    vTaskDelay(2000 / portTICK_PERIOD_MS); // Delay for 2 seconds
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      client.subscribe("parking/vagas");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
  }
}

void loop() {
  vTaskDelete(NULL);
}

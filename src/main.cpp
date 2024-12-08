#include <Wire.h>
#undef I2C_BUFFER_LENGTH // Remove definição anterior
#define I2C_BUFFER_LENGTH 128 // Usa o valor maior para evitar problemas
#include "MAX30105.h" // Biblioteca para o sensor MAX30105 (compatível com MAX30102)

// Objeto para o sensor MAX30105
MAX30105 particleSensor;

// Variáveis para cálculo de SpO2
double avered = 0, aveir = 0;
double sumirrms = 0, sumredrms = 0;
int i = 0, Num = 100; // Intervalo de amostragem
float ESpO2 = 0;      // Valor estimado inicial do SpO2
double FSpO2 = 0.7;   // Fator de filtragem do SpO2
double frate = 0.95;  // Fator de filtragem de baixa frequência
#define FINGER_ON 30000 // Limite para detecção de dedo no sensor

void setup() {
  Serial.begin(115200);
  Wire.begin(18, 19); // Inicializa o I2C nos pinos SDA = 18 e SCL = 19

  Serial.println("Inicializando o sensor MAX30102...");
  while (!particleSensor.begin(Wire, I2C_SPEED_FAST)) { // Configura o sensor com velocidade I2C de 400kHz
    Serial.println("MAX30102 não encontrado. Verifique as conexões.");
    delay(1000);
  }
  Serial.println("Sensor inicializado!");

  // Configurações do sensor
  particleSensor.setup(0x7F, 4, 2, 200, 411, 16384); // Configuração padrão
}

void loop() {
  uint32_t red, ir;

  // Lê dados do sensor
  particleSensor.check(); // Verifica se há dados disponíveis
  while (particleSensor.available()) {
    red = particleSensor.getFIFORed(); // Lê o valor do LED vermelho
    ir = particleSensor.getFIFOIR();  // Lê o valor do LED infravermelho

    // Processamento dos valores lidos
    double fred = (double)red, fir = (double)ir;
    avered = avered * frate + fred * (1.0 - frate);
    aveir = aveir * frate + fir * (1.0 - frate);
    sumredrms += (fred - avered) * (fred - avered);
    sumirrms += (fir - aveir) * (fir - aveir);

    // Calcula o SpO2 quando o intervalo de amostragem é atingido
    if (++i % Num == 0) {
      double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
      double SpO2 = -23.3 * (R - 0.4) + 100; // Fórmula para estimativa de SpO2
      ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2; // Filtro passa-baixa

      // Exibe o valor estimado de SpO2 se o dedo for detectado
      if (ir > FINGER_ON) {
        Serial.print("SpO2 estimado: ");
        Serial.print(ESpO2);
        Serial.println(" ");
      } else {
        Serial.println("Nenhum dedo detectado no sensor.");
      }

      // Reseta acumuladores
      sumredrms = 0.0;
      sumirrms = 0.0;
      i = 0;
    }
    particleSensor.nextSample(); // Move para a próxima amostra
  }
}

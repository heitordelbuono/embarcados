#include <stdio.h>
#define EVENTO_INCREMENTA_RELOGIO 0x01

int relogio_horas = 0, relogio_minutos = 0, relogio_segundos = 0;
bool relogio_mudou_segundos = false;

void relogio_incrementa(void){
        relogio_mudou_segundos = true;
        relogio_segundos++;
        if (relogio_segundos == 60){
            relogio_segundos = 0;
            relogio_minutos++;
        }
        if (relogio_minutos == 60){
            relogio_minutos = 0;
            relogio_horas++;
        }
        push(millis() + 1000, EVENTO_INCREMENTE_RELOGIO, 0);
}
// Configuração do timer
hw_timer_t* timer = nullptr;
volatile bool flagTick = false;

void IRAM_ATTR onTimer() {
  flagTick = true;
}

void setupTickInterrupt(long uSecs) {
  // clock base do timer ESP32 = 80MHz
  // prescaler 80 → 1 tick = 1µs
  timer = timerBegin(0, 80, true);         // timer 0, prescaler 80, contagem crescente
  timerAttachInterrupt(timer, &onTimer, true); // true = borda de subida
  timerAlarmWrite(timer, uSecs, true);     // dispara a cada uSecs µs, repetindo
  timerAlarmEnable(timer);
}


/* void loop() {
  if (flagTick) {
    flagTick = false;
    // lógica do escalonador aqui
  }
} */


void setup(){
    Serial.begin(115200);
    setupTickInterrupt(long uSecs); // tick a cada 1000µs = 1ms
    iniciaMaquinaEstados();
    ModuloGerenciador gerenciador;
    gerenciador.iniciaModuloGerenciador();
    int estado_atual = ESPERANDO;
    int evento_ocorrido = NENHUM_EVENTO;
    int acao;
}

void loop(){
    evento_ocorrido = gerenciador.fila.lerProximoEvento()
    while(True){
        switch (matrizTransicaoEstados[estado_atual][evento_ocorrido.tipo].acao){
            case A01: // Habilita sistema de equilíbrio
                printf("Botao pressionado, espere para atravessar.\n");
            break;

            case A02:
                printf("Botao ja pressionado, ignorando.\n");
            break;

            case A03:
                printf("Pode atravessar.\n");
            break;

            case A04:
                printf("Tempo para travessia acabando, emitindo aviso sonoro.\n");
            break;

            case A05:
                printf("Tempo esgotado para travessia, acionar para atravessar.\n");
            break;
        } 

        estado_atual = matrizTransicaoEstados[estado_atual][input_usuario].prox_estado;
    }
}
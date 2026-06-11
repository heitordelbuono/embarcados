#include <stdio.h>
#define EVENTO_INCREMENTA_RELOGIO 0x01
#include "classes/Bluetooth.h"
#include "classes/ControladorPID.h"
#include "classes/DriverDeAtuacao.h"
#include "classes/FilaDeEventos.h"
#include "classes/ModuloGerenciador.h"
#include "MaquinaEstados/maquinaEstados.h"

int relogio_horas = 0, relogio_minutos = 0, relogio_segundos = 0;
bool relogio_mudou_segundos = false;
int evento_atual;
int estado_atual;
int acao_atual;
ModuloGerenciador gerenciador;

/*void relogio_incrementa(void){
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
        */
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

void taskObterEvento(void){ // verifica a existência de novos eventos para inserir na fila
        // EVENTO BOTÃO
        if (gerenciador.bluetooth.botao_lido == false){
            gerenciador.fila.push(micros() + 20000, BOTAO, 0);
            gerenciador.bluetooth.botao_lido = true;
        }

        // EVENTO DESEQUILIBRIO
        if (gerenciador.bluetooth.pos_x_atual < (240.0 - 20) || gerenciador.bluetooth.pos_x_atual > (240.0 + 20) || 
            gerenciador.bluetooth.pos_y_atual > (240.0 + 20) || gerenciador.bluetooth.pos_y_atual < (240.0 - 20)){
                if(gerenciador.fila.numeroEventos == 0 || gerenciador.fila.Evento[gerenciador.fila.numeroEventos-1].tipo != DESEQUILIBRIO){
                    gerenciador.fila.push(micros() + 20000, DESEQUILIBRIO, 0);
                }
            }

        // EVENTO EQUILIBRIO
        if (gerenciador.bluetooth.pos_x_atual > (240.0 - 20) && gerenciador.bluetooth.pos_x_atual < (240.0 + 20) && 
            gerenciador.bluetooth.pos_y_atual < (240.0 + 20) && gerenciador.bluetooth.pos_y_atual > (240.0 - 20)){
                if (gerenciador.fila.numeroEventos == 0 || gerenciador.fila.Evento[gerenciador.fila.numeroEventos-1].tipo != EQUILIBRIO){
                    gerenciador.fila.push(micros() + 20000, EQUILIBRIO, 0);
                }
            }
}

void taskMaquinaEstados(void){ // altera os estados e ações atuais com base no evento atual
    acao_atual = matrizTransicaoEstados[estado_atual][evento_atual].acao;
    estado_atual = matrizTransicaoEstados[estado_atual][evento_atual].prox_estado;
}

void taskCorrigePosicao(void){ // calcula correção com controlador e envia para o driver de atuação
    gerenciador.calculaAcaoControle_emX(L, gerenciador.bluetooth.pos_x_atual);
    gerenciador.calculaAcaoControle_emY(L, gerenciador.bluetooth.pos_y_atual);
}

void setup(){
    Serial.begin(115200);
    setupTickInterrupt(1000); // tick a cada 1000µs = 1ms
    iniciaMaquinaEstados();
    gerenciador.iniciaModuloGerenciador();
    estado_atual = ESPERANDO;
    evento_atual = NENHUM_EVENTO;
    acao_atual = NENHUMA_ACAO;
}

void loop(){
    if (flagTick){ // a cada 1ms
        flagTick = false;
        taskObterEvento(); // verifico novos eventos para serem inseridos na fila
        if (gerenciador.fila.numeroEventos > 0 && micros() >= gerenciador.fila.Evento[0].instante ){// se houverem eventos na fila e estiver na hora de executar o primeiro da fila
        evento_atual = gerenciador.fila.lerProximoEvento().tipo; // atualizo o evento atual
        taskMaquinaEstados(); // atualizo minha ação atual e meu estado atual com a máquina de estados
        }
    }
    gerenciador.bluetooth.temDadoNovo(); // sempre verifico dados novos do bluetooth 
    switch (acao_atual){
        case A01: // Habilita sistema de equilíbrio
            gerenciador.bluetooth.sistema_ligado = true; // defino estado do sistema como ligado
        break;

        case A02: // Corrige posição com o controlador PD
            // ...
        break;

        case A03: // Deixa de solicitar correção ao controlador PD
            // não faz nada?
        break;

        case A04: // Desliga sistema de equilíbrio
            gerenciador.bluetooth.sistema_ligado = false; // defino sistema como desligado
        break;
    }

    static unsigned long ultimo_tempo_pid = 0;
    if(estado_atual == EQUILIBRANDO){
        if (micros() - ultimo_tempo_pid >= 20000 && gerenciador.bluetooth.sistema_ligado){
            taskCorrigePosicao();// Calcula sinais de correção com controlador e envia para os servos
            ultimo_tempo_pid = micros();
        }
    }
    acao_atual = NENHUMA_ACAO;
}

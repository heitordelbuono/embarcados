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

int main(void){
    int estado_atual = ESPERANDO;
    int input_usuario;
    int acao;
    while(True){
        printf("Insira o evento:\n");
        scanf("%d", &input_usuario);
        switch (matrizTransicaoEstados[estado_atual][input_usuario].acao){
            case A01:
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
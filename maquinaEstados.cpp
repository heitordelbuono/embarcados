#define NUM_EVENTOS 3
#define NUM_ESTADOS 4
#define NENHUMA_ACAO -1
enum ESTADOS {ESPERANDO, SISTEMA_LIGADO, EQUILIBRANDO, EQUILIBRADO};
enum ACOES { A01, A02, A03, A04};
enum EVENTOS {BOTAO, DESEQUILIBRIO, EQUILIBRIO, NENHUM_EVENTO};
struct proximo {
    int acao;
    int prox_estado;
};

struct proximo matrizTransicaoEstados[NUM_ESTADOS][NUM_EVENTOS];
void iniciaMaquinaEstados(void){
    int i, j;
    for (i=0; i<NUM_ESTADOS; i++){
        for (j=0; j<NUM_EVENTOS; j++){
            matrizTransicaoEstados[i][j].acao = NENHUMA_ACAO;
            matrizTransicaoEstados[i][j].prox_estado = i;
        }
    }
    // ESTADO ESPERANDO
    matrizTransicaoEstados[ESPERANDO][botao].acao = A01;
    matrizTransicaoEstados[ESPERANDO][botao].prox_estado = SISTEMA_LIGADO;

    // ESTADO SISTEMA LIGADO

        // DETECTA DESEQUILIBRIO
    matrizTransicaoEstados[SISTEMA_LIGADO][desequilibrio].acao = A02;
    matrizTransicaoEstados[SISTEMA_LIGADO][desequilibrio].prox_estado = EQUILIBRANDO;

        // DETECTA EQUILIBRIO
    matrizTransicaoEstados[SISTEMA_LIGADO][equilibrio].acao = A03;
    matrizTransicaoEstados[SISTEMA_LIGADO][equilibrio].prox_estado = EQUILIBRADO;

    // EQUILIBRANDO

        // APERTA BOTAO
    matrizTransicaoEstados[EQUILIBRANDO][botao].acao = A04;
    matrizTransicaoEstados[EQUILIBRANDO][botao].prox_estado = ESPERANDO;

        // DETECTA EQUILIBRIO
    matrizTransicaoEstados[EQUILIBRANDO][equilibrio].acao = A03;
    matrizTransicaoEstados[EQUILIBRANDO][equilibrio].prox_estado = EQUILIBRADO;

    // EQUILIBRADO
        
        // APERTA BOTAO
    matrizTransicaoEstados[EQUILIBRADO][botao].acao = A04;
    matrizTransicaoEstados[EQUILIBRADO][botao].prox_estado = ESPERANDO;

        // DETECTA DESEQUILIBRIO
    matrizTransicaoEstados[EQUILIBRADO][desequilibrio].acao = A02;
    matrizTransicaoEstados[EQUILIBRADO][desequilibrio].prox_estado = EQUILIBRANDO;
}

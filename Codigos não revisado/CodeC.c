#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <opencv2>
#include <iostream>
#include <vector>
#include <string>

// ----------- MACROS / CONSTANTES ------------------------------------------------
// resolução de trabalho (warp, YOLO, máscaras)
const int IMG_W = 1280;
const int IMG_H = 720;

// resolução de trabalho (warp, YOLO, máscaras)
const int IMG_Ws = 640;
const int IMG_Hs = 480;

// tamanho das janelas do cv2.imshow
const int SCR_W = 800;
const int SCR_H = 600;

// conversões pista-real (ajuste conforme a largura real da faixa)
const double YM_PER_PIX = 2.40 / 480.0; // m por pixel no eixo Y (≈ altura)
const double XM_PER_PIX = 1.00 / 350.0; // m por pixel no eixo X (≈ largura da pista)

const double CONF_THRESHOLD = 0.30; // limiar de confiança para YOLO

//----------------------------------------------------------------------------------

int posix_ponto_1_x = 0;
int posix_ponto_1_y = 0;

int posix_ponto_2_x = 0;
int posix_ponto_2_y = 0;

int posix_ponto_3_x = 0;
int posix_ponto_3_y = 0;

int posix_ponto_4_x = 0;
int posix_ponto_4_y = 0;


//  Equivalente a def escolher_porta(portas):
void escolher_porta(char** portas, int num_portas) {
    printf("Portas disponíveis:\n");
    for (int i = 0; i < num_portas; i++) {
        printf("%d: %s\n", i + 1, portas[i]);
    }
    int escolha;
    printf("Escolha uma porta pelo número: ");
    scanf("%d", &escolha);
    escolha--; // Adjust for zero-based index
    if (escolha >= 0 && escolha < num_portas) {
        printf("Porta escolhida: %s\n", portas[escolha]);
    } else {
        printf("Escolha inválida.\n");
    }
}


//  Equivalente a def enviar_dado(almost_can, dado, id=0x00):
void enviar_dado(void* almost_can, int dado, int id) {
    unsigned char data_bytes[4];
    data_bytes[0] = (dado >> 24) & 0xFF;
    data_bytes[1] = (dado >> 16) & 0xFF;
    data_bytes[2] = (dado >> 8) & 0xFF;
    data_bytes[3] = dado & 0xFF;

    // Assuming almost_can has a function send_data
    // almost_can_send_data(almost_can, id, data_bytes); // Placeholder for actual send function
    printf("Dado %d enviado com sucesso\n", dado);
}


//   Equivalente a map_in_range(x: float, in_min: float, in_max: float, out_min: float, out_max: float)
float map_in_range(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



// Equivalente a def calcular_angulo_do_volante(distancia_em_metros):
int calcular_angulo_do_volante(float distancia_em_metros) {
	int angulo = map_in_range(distancia_em_metros, -1, 1, 50, 0)
	return (int)angulo;
}
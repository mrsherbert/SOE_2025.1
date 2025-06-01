#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <stdint.h>
#include <unistd.h>

#ifdef _WIN32
    #include <windows.h>
    #include <commdlg.h>
#else
    #include <termios.h>
    #include <fcntl.h>
    #include <sys/types.h>
    #include <dirent.h>
    #include <glob.h>
#endif

using namespace cv;

// ---------- MACROS / CONSTANTES ------------------------------------------------
#define IMG_W 1280
#define IMG_H 720           // resolução de trabalho (warp, YOLO, máscaras)
#define SCR_W 800
#define SCR_H 600            // tamanho das janelas do cv2.imshow

// conversões pista-real (ajuste conforme a largura real da faixa):
#define YM_PER_PIX (2.40 / 720.0)            // m por pixel no eixo Y      (≈ altura)
#define XM_PER_PIX (1.00 / 700.0)            // m por pixel no eixo X      (≈ largura da pista)

#define CONF_THRESHOLD 0.30              // limiar de confiança p/ YOLO
// ------------------------------------------------------------------------------

int posix_ponto_1_x = 0;
int posix_ponto_1_y = 0;

int posix_ponto_2_x = 0;
int posix_ponto_2_y = 0;

int posix_ponto_3_x = 0;
int posix_ponto_3_y = 0;

int posix_ponto_4_x = 0;
int posix_ponto_4_y = 0;

typedef struct {
    int SUCCESS;
    int ERROR_TIMEOUT;
    int ERROR_INVALID_SOF;
    int ERROR_INVALID_EOF;
    int ERROR_CRC_MISMATCH;
    int ERROR_INVALID_LENGTH;
    
    int serial_fd;
    int baud_rate;
    uint8_t start_byte;
    uint8_t end_byte;
    int timeout;
    char port_name[256];
} AlmostCan;

typedef struct {
    int width;
    int height;
} Resolution;

// Função para configurar porta serial no Linux/Unix
#ifndef _WIN32
int configure_serial_port(int fd, int baud_rate) {
    struct termios tty;
    
    if (tcgetattr(fd, &tty) != 0) {
        return -1;
    }
    
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;
    
    tty.c_cc[VTIME] = 10;
    tty.c_cc[VMIN] = 0;
    
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        return -1;
    }
    
    return 0;
}
#endif

void AlmostCan_init(AlmostCan* ac, const char* serial_port, int baud_rate, uint8_t start_byte, uint8_t end_byte, int timeout) {
    ac->SUCCESS = 0;
    ac->ERROR_TIMEOUT = 1;
    ac->ERROR_INVALID_SOF = 2;
    ac->ERROR_INVALID_EOF = 3;
    ac->ERROR_CRC_MISMATCH = 4;
    ac->ERROR_INVALID_LENGTH = 5;
    
    ac->serial_fd = -1;
    ac->baud_rate = baud_rate;
    ac->start_byte = start_byte;
    ac->end_byte = end_byte;
    ac->timeout = timeout;
    strcpy(ac->port_name, serial_port);
}

int AlmostCan_begin(AlmostCan* ac) {
#ifdef _WIN32
    ac->serial_fd = (int)CreateFileA(ac->port_name, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (ac->serial_fd == (int)INVALID_HANDLE_VALUE) {
        return -1;
    }
    
    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    
    if (!GetCommState((HANDLE)ac->serial_fd, &dcbSerialParams)) {
        CloseHandle((HANDLE)ac->serial_fd);
        return -1;
    }
    
    dcbSerialParams.BaudRate = ac->baud_rate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    
    if (!SetCommState((HANDLE)ac->serial_fd, &dcbSerialParams)) {
        CloseHandle((HANDLE)ac->serial_fd);
        return -1;
    }
    
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = ac->timeout;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    
    if (!SetCommTimeouts((HANDLE)ac->serial_fd, &timeouts)) {
        CloseHandle((HANDLE)ac->serial_fd);
        return -1;
    }
#else
    ac->serial_fd = open(ac->port_name, O_RDWR | O_NOCTTY | O_SYNC);
    if (ac->serial_fd < 0) {
        return -1;
    }
    
    if (configure_serial_port(ac->serial_fd, ac->baud_rate) < 0) {
        close(ac->serial_fd);
        return -1;
    }
#endif
    
    return 0;
}

uint16_t AlmostCan_calculate_crc(AlmostCan* ac, uint8_t* data, int length) {
    uint16_t crc = 0;
    for (int i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void AlmostCan_send_data(AlmostCan* ac, uint8_t id, uint8_t* data, int length) {
    uint8_t packet[256];
    int packet_length = 0;
    
    packet[packet_length++] = ac->start_byte;
    packet[packet_length++] = id;
    packet[packet_length++] = length;
    
    for (int i = 0; i < length; i++) {
        packet[packet_length++] = data[i];
    }
    
    uint16_t crc = AlmostCan_calculate_crc(ac, &packet[1], 2 + length);
    packet[packet_length++] = crc & 0xFF;
    packet[packet_length++] = (crc >> 8) & 0xFF;
    packet[packet_length++] = ac->end_byte;
    
#ifdef _WIN32
    DWORD bytes_written;
    WriteFile((HANDLE)ac->serial_fd, packet, packet_length, &bytes_written, NULL);
#else
    write(ac->serial_fd, packet, packet_length);
#endif
}

int AlmostCan_wait_for_data(AlmostCan* ac, int expected_bytes) {
    clock_t start_time = clock();
    double timeout_sec = ac->timeout / 1000.0;
    
    while (1) {
        int available = 0;
#ifdef _WIN32
        COMSTAT comStat;
        DWORD errors;
        ClearCommError((HANDLE)ac->serial_fd, &errors, &comStat);
        available = comStat.cbInQue;
#else
        ioctl(ac->serial_fd, FIONREAD, &available);
#endif
        
        if (available >= expected_bytes) {
            return 1;
        }
        
        double elapsed = ((double)(clock() - start_time)) / CLOCKS_PER_SEC;
        if (elapsed >= timeout_sec) {
            return 0;
        }
        
        usleep(1000); // 1ms delay
    }
}

int AlmostCan_receive_data(AlmostCan* ac, uint8_t* id, uint8_t* data, int* length) {
    if (!AlmostCan_wait_for_data(ac, 1)) {
        return ac->ERROR_TIMEOUT;
    }
    
    uint8_t start_byte;
#ifdef _WIN32
    DWORD bytes_read;
    ReadFile((HANDLE)ac->serial_fd, &start_byte, 1, &bytes_read, NULL);
#else
    read(ac->serial_fd, &start_byte, 1);
#endif
    
    if (start_byte != ac->start_byte) {
        return ac->ERROR_INVALID_SOF;
    }
    
    if (!AlmostCan_wait_for_data(ac, 2)) {
        return ac->ERROR_TIMEOUT;
    }
    
    uint8_t recv_id, recv_length;
#ifdef _WIN32
    ReadFile((HANDLE)ac->serial_fd, &recv_id, 1, &bytes_read, NULL);
    ReadFile((HANDLE)ac->serial_fd, &recv_length, 1, &bytes_read, NULL);
#else
    read(ac->serial_fd, &recv_id, 1);
    read(ac->serial_fd, &recv_length, 1);
#endif
    
    if (recv_length > 32) {
        return ac->ERROR_INVALID_LENGTH;
    }
    
    if (recv_length > 0) {
        if (!AlmostCan_wait_for_data(ac, recv_length)) {
            return ac->ERROR_TIMEOUT;
        }
        
#ifdef _WIN32
        ReadFile((HANDLE)ac->serial_fd, data, recv_length, &bytes_read, NULL);
#else
        read(ac->serial_fd, data, recv_length);
#endif
    }
    
    if (!AlmostCan_wait_for_data(ac, 3)) {
        return ac->ERROR_TIMEOUT;
    }
    
    uint8_t crc_low, crc_high, eof;
#ifdef _WIN32
    ReadFile((HANDLE)ac->serial_fd, &crc_low, 1, &bytes_read, NULL);
    ReadFile((HANDLE)ac->serial_fd, &crc_high, 1, &bytes_read, NULL);
    ReadFile((HANDLE)ac->serial_fd, &eof, 1, &bytes_read, NULL);
#else
    read(ac->serial_fd, &crc_low, 1);
    read(ac->serial_fd, &crc_high, 1);
    read(ac->serial_fd, &eof, 1);
#endif
    
    if (eof != ac->end_byte) {
        return ac->ERROR_INVALID_EOF;
    }
    
    uint16_t received_crc = (crc_high << 8) | crc_low;
    uint8_t crc_data[34];
    crc_data[0] = recv_id;
    crc_data[1] = recv_length;
    for (int i = 0; i < recv_length; i++) {
        crc_data[2 + i] = data[i];
    }
    
    uint16_t calculated_crc = AlmostCan_calculate_crc(ac, crc_data, 2 + recv_length);
    
    if (calculated_crc != received_crc) {
        return ac->ERROR_CRC_MISMATCH;
    }
    
    *id = recv_id;
    *length = recv_length;
    return ac->SUCCESS;
}

void AlmostCan_set_start_byte(AlmostCan* ac, uint8_t start_byte) {
    ac->start_byte = start_byte;
}

void AlmostCan_set_end_byte(AlmostCan* ac, uint8_t end_byte) {
    ac->end_byte = end_byte;
}

void AlmostCan_set_timeout(AlmostCan* ac, int timeout) {
    ac->timeout = timeout;
}

void listar_portas_seriais(char portas[][256], int* count) {
    *count = 0;
    
#ifdef _WIN32
    for (int i = 1; i <= 255; i++) {
        char port_name[256];
        sprintf(port_name, "COM%d", i);
        HANDLE handle = CreateFileA(port_name, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
        if (handle != INVALID_HANDLE_VALUE) {
            strcpy(portas[*count], port_name);
            (*count)++;
            CloseHandle(handle);
        }
    }
#else
    glob_t glob_result;
    glob("/dev/tty[A-Za-z]*", GLOB_TILDE, NULL, &glob_result);
    
    for (size_t i = 0; i < glob_result.gl_pathc; i++) {
        int fd = open(glob_result.gl_pathv[i], O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd >= 0) {
            strcpy(portas[*count], glob_result.gl_pathv[i]);
            (*count)++;
            close(fd);
        }
    }
    
    globfree(&glob_result);
#endif
}

char* escolher_porta(char portas[][256], int count) {
    printf("Portas disponíveis:\n");
    for (int i = 0; i < count; i++) {
        printf("%d: %s\n", i + 1, portas[i]);
    }
    printf("Escolha uma porta pelo número: ");
    
    int escolha;
    scanf("%d", &escolha);
    
    if (escolha >= 1 && escolha <= count) {
        return portas[escolha - 1];
    }
    
    return NULL;
}

void enviar_dado(AlmostCan* almost_can, int dado, uint8_t id) {
    uint8_t data_bytes[4];
    data_bytes[0] = (dado >> 24) & 0xFF;
    data_bytes[1] = (dado >> 16) & 0xFF;
    data_bytes[2] = (dado >> 8) & 0xFF;
    data_bytes[3] = dado & 0xFF;
    
    AlmostCan_send_data(almost_can, id, data_bytes, 4);
    printf("Dado %d enviado com sucesso\n", dado);
}

AlmostCan* inicializar_serial() {
    char portas[256][256];
    int count;
    listar_portas_seriais(portas, &count);
    
    if (count > 0) {
        char* porta_escolhida = escolher_porta(portas, count);
        if (porta_escolhida) {
            printf("Porta escolhida: %s\n", porta_escolhida);
            
            AlmostCan* ac = (AlmostCan*)malloc(sizeof(AlmostCan));
            AlmostCan_init(ac, porta_escolhida, 115200, 0xFF, 0xF7, 1000);
            
            if (AlmostCan_begin(ac) == 0) {
                sleep(2);
                return ac;
            } else {
                printf("Erro ao acessar a porta %s\n", porta_escolhida);
                free(ac);
                return NULL;
            }
        }
    } else {
        printf("Nenhuma porta serial disponível.\n");
        return NULL;
    }
    
    return NULL;
}

float map_in_range(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int calcular_angulo_do_volante(float distancia_em_metros) {
    int angulo = (int)map_in_range(distancia_em_metros, -1.0, 1.0, 50.0, 0.0);
    return angulo;
}

void warp(Mat& img, Mat& warped_img, Mat& M_inv) {
    Point2f src[4];
    Point2f dst[4];
    
    src[0] = Point2f(0, IMG_H);
    src[1] = Point2f(IMG_W, IMG_H);
    src[2] = Point2f(200, 430);
    src[3] = Point2f(IMG_W - 200, 430);
    
    dst[0] = Point2f(0, IMG_H);
    dst[1] = Point2f(IMG_W, IMG_H);
    dst[2] = Point2f(0, 0);
    dst[3] = Point2f(IMG_W, 0);
    
    Mat M = getPerspectiveTransform(src, dst);
    M_inv = getPerspectiveTransform(dst, src);
    
    warpPerspective(img, warped_img, M, Size(IMG_W, IMG_H));
}

void find_lane_pixels_using_histogram(Mat& binary_warped, std::vector<int>& leftx, std::vector<int>& lefty, 
                                     std::vector<int>& rightx, std::vector<int>& righty) {
    leftx.clear();
    lefty.clear();
    rightx.clear();
    righty.clear();
    
    // Criar histograma da metade inferior da imagem
    Mat lower_half = binary_warped(Rect(0, binary_warped.rows / 2, binary_warped.cols, binary_warped.rows / 2));
    Mat histogram;
    reduce(lower_half, histogram, 0, REDUCE_SUM, CV_32S);
    
    int midpoint = histogram.cols / 2;
    int leftx_base = 0, rightx_base = 0;
    int max_left = 0, max_right = 0;
    
    // Encontrar picos do histograma
    for (int i = 0; i < midpoint; i++) {
        if (histogram.at<int>(0, i) > max_left) {
            max_left = histogram.at<int>(0, i);
            leftx_base = i;
        }
    }
    
    for (int i = midpoint; i < histogram.cols; i++) {
        if (histogram.at<int>(0, i) > max_right) {
            max_right = histogram.at<int>(0, i);
            rightx_base = i;
        }
    }
    
    int nwindows = 15;
    int margin = 100;
    int minpix = 50;
    int window_height = binary_warped.rows / nwindows;
    
    // Encontrar pixels não-zero
    std::vector<Point> nonzero_points;
    findNonZero(binary_warped, nonzero_points);
    
    int leftx_current = leftx_base;
    int rightx_current = rightx_base;
    
    std::vector<int> left_lane_inds;
    std::vector<int> right_lane_inds;
    
    for (int window = 0; window < nwindows; window++) {
        int win_y_low = binary_warped.rows - (window + 1) * window_height;
        int win_y_high = binary_warped.rows - window * window_height;
        int win_xleft_low = leftx_current - margin;
        int win_xleft_high = leftx_current + margin;
        int win_xright_low = rightx_current - margin;
        int win_xright_high = rightx_current + margin;
        
        std::vector<int> good_left_inds;
        std::vector<int> good_right_inds;
        
        for (size_t i = 0; i < nonzero_points.size(); i++) {
            int x = nonzero_points[i].x;
            int y = nonzero_points[i].y;
            
            if (y >= win_y_low && y < win_y_high &&
                x >= win_xleft_low && x < win_xleft_high) {
                good_left_inds.push_back(i);
            }
            
            if (y >= win_y_low && y < win_y_high &&
                x >= win_xright_low && x < win_xright_high) {
                good_right_inds.push_back(i);
            }
        }
        
        left_lane_inds.insert(left_lane_inds.end(), good_left_inds.begin(), good_left_inds.end());
        right_lane_inds.insert(right_lane_inds.end(), good_right_inds.begin(), good_right_inds.end());
        
        if (good_left_inds.size() > minpix) {
            int sum_x = 0;
            for (int idx : good_left_inds) {
                sum_x += nonzero_points[idx].x;
            }
            leftx_current = sum_x / good_left_inds.size();
        }
        
        if (good_right_inds.size() > minpix) {
            int sum_x = 0;
            for (int idx : good_right_inds) {
                sum_x += nonzero_points[idx].x;
            }
            rightx_current = sum_x / good_right_inds.size();
        }
    }
    
    // Extrair coordenadas dos pixels das faixas
    for (int idx : left_lane_inds) {
        leftx.push_back(nonzero_points[idx].x);
        lefty.push_back(nonzero_points[idx].y);
    }
    
    for (int idx : right_lane_inds) {
        rightx.push_back(nonzero_points[idx].x);
        righty.push_back(nonzero_points[idx].y);
    }
}

void fit_poly(Mat& binary_warped, std::vector<int>& leftx, std::vector<int>& lefty, 
              std::vector<int>& rightx, std::vector<int>& righty,
              std::vector<double>& left_fit, std::vector<double>& right_fit,
              std::vector<double>& left_fitx, std::vector<double>& right_fitx,
              std::vector<double>& ploty) {
    
    left_fit.resize(3);
    right_fit.resize(3);
    left_fitx.clear();
    right_fitx.clear();
    ploty.clear();
    
    // Criar vetor ploty
    for (int i = 0; i < binary_warped.rows; i++) {
        ploty.push_back(i);
    }
    
    // Ajustar polinômio de 2ª ordem para a faixa esquerda
    if (lefty.size() >= 3) {
        Mat A = Mat::zeros(lefty.size(), 3, CV_64F);
        Mat B = Mat::zeros(lefty.size(), 1, CV_64F);
        
        for (size_t i = 0; i < lefty.size(); i++) {
            double y = lefty[i];
            A.at<double>(i, 0) = y * y;
            A.at<double>(i, 1) = y;
            A.at<double>(i, 2) = 1.0;
            B.at<double>(i, 0) = leftx[i];
        }
        
        Mat left_coeffs;
        solve(A, B, left_coeffs, DECOMP_SVD);
        
        left_fit[0] = left_coeffs.at<double>(0, 0);
        left_fit[1] = left_coeffs.at<double>(1, 0);
        left_fit[2] = left_coeffs.at<double>(2, 0);
    } else {
        left_fit[0] = 1.0;
        left_fit[1] = 1.0;
        left_fit[2] = 0.0;
    }
    
    // Ajustar polinômio de 2ª ordem para a faixa direita
    if (righty.size() >= 3) {
        Mat A = Mat::zeros(righty.size(), 3, CV_64F);
        Mat B = Mat::zeros(righty.size(), 1, CV_64F);
        
        for (size_t i = 0; i < righty.size(); i++) {
            double y = righty[i];
            A.at<double>(i, 0) = y * y;
            A.at<double>(i, 1) = y;
            A.at<double>(i, 2) = 1.0;
            B.at<double>(i, 0) = rightx[i];
        }
        
        Mat right_coeffs;
        solve(A, B, right_coeffs, DECOMP_SVD);
        
        right_fit[0] = right_coeffs.at<double>(0, 0);
        right_fit[1] = right_coeffs.at<double>(1, 0);
        right_fit[2] = right_coeffs.at<double>(2, 0);
    } else {
        right_fit[0] = 1.0;
        right_fit[1] = 1.0;
        right_fit[2] = 0.0;
    }
    
    // Calcular pontos das faixas
    for (double y : ploty) {
        try {
            double left_x = left_fit[0] * y * y + left_fit[1] * y + left_fit[2];
            double right_x = right_fit[0] * y * y + right_fit[1] * y + right_fit[2];
            left_fitx.push_back(left_x);
            right_fitx.push_back(right_x);
        } catch (...) {
            printf("The function failed to fit a line!\n");
            double left_x = 1.0 * y * y + 1.0 * y;
            double right_x = 1.0 * y * y + 1.0 * y;
            left_fitx.push_back(left_x);
            right_fitx.push_back(right_x);
        }
    }
}

void measure_curvature_meters(std::vector<double>& ploty, std::vector<double>& left_fitx, 
                             std::vector<double>& right_fitx, double& left_c, double& right_c) {
    
    // Converter para metros
    std::vector<double> ploty_m, left_fitx_m, right_fitx_m;
    for (size_t i = 0; i < ploty.size(); i++) {
        ploty_m.push_back(ploty[i] * YM_PER_PIX);
        left_fitx_m.push_back(left_fitx[i] * XM_PER_PIX);
        right_fitx_m.push_back(right_fitx[i] * XM_PER_PIX);
    }
    
    // Ajustar novos polinômios em metros
    Mat A = Mat::zeros(ploty_m.size(), 3, CV_64F);
    Mat B_left = Mat::zeros(ploty_m.size(), 1, CV_64F);
    Mat B_right = Mat::zeros(ploty_m.size(), 1, CV_64F);
    
    for (size_t i = 0; i < ploty_m.size(); i++) {
        double y = ploty_m[i];
        A.at<double>(i, 0) = y * y;
        A.at<double>(i, 1) = y;
        A.at<double>(i, 2) = 1.0;
        B_left.at<double>(i, 0) = left_fitx_m[i];
        B_right.at<double>(i, 0) = right_fitx_m[i];
    }
    
    Mat left_fit_cr, right_fit_cr;
    solve(A, B_left, left_fit_cr, DECOMP_SVD);
    solve(A, B_right, right_fit_cr, DECOMP_SVD);
    
    double y_eval = ploty.back() * YM_PER_PIX;
    
    double left_a = left_fit_cr.at<double>(0, 0);
    double left_b = left_fit_cr.at<double>(1, 0);
    double right_a = right_fit_cr.at<double>(0, 0);
    double right_b = right_fit_cr.at<double>(1, 0);
    
    left_c = pow((1 + pow(2 * left_a * y_eval + left_b, 2)), 1.5) / abs(2 * left_a);
    right_c = pow((1 + pow(2 * right_a * y_eval + right_b, 2)), 1.5) / abs(2 * right_a);
}

double measure_position_meters(std::vector<double>& left_fit, std::vector<double>& right_fit) {
    double y_max = IMG_H;
    double left_x = left_fit[0] * y_max * y_max + left_fit[1] * y_max + left_fit[2];
    double right_x = right_fit[0] * y_max * y_max + right_fit[1] * y_max + right_fit[2];
    double lane_mid = (left_x + right_x) / 2.0;
    double veh_pos = ((IMG_W / 2.0) - lane_mid) * XM_PER_PIX;
    return veh_pos - 0.10;
}

Mat project_lane_info(Mat& img, Mat& binary_warped, std::vector<double>& ploty, 
                     std::vector<double>& left_fitx, std::vector<double>& right_fitx,
                     Mat& M_inv, double left_curverad, double right_curverad, 
                     double veh_pos, double& avg_curverad) {
    
    Mat warp_zero = Mat::zeros(binary_warped.size(), CV_8UC1);
    Mat color_warp = Mat::zeros(binary_warped.size(), CV_8UC3);
    
    // Criar pontos para a região da pista
    std::vector<Point> pts_left, pts_right, pts;
    
    for (size_t i = 0; i < ploty.size(); i++) {
        pts_left.push_back(Point((int)left_fitx[i], (int)ploty[i]));
    }
    
    for (int i = ploty.size() - 1; i >= 0; i--) {
        pts_right.push_back(Point((int)right_fitx[i], (int)ploty[i]));
    }
    
    pts.insert(pts.end(), pts_left.begin(), pts_left.end());
    pts.insert(pts.end(), pts_right.begin(), pts_right.end());
    
    // Preencher polígono
    std::vector<std::vector<Point>> contours;
    contours.push_back(pts);
    fillPoly(color_warp, contours, Scalar(0, 255, 0));
    
    // Aplicar transformação inversa
    Mat newwarp;
    warpPerspective(color_warp, newwarp, M_inv, Size(img.cols, img.rows));
    
    // Combinar com a imagem original
    Mat out_img;
    addWeighted(img, 1.0, newwarp, 0.3, 0, out_img);
    
    avg_curverad = (left_curverad + right_curverad) / 2.0;
    
    // Adicionar texto
    char text1[100], text2[100];
    sprintf(text1, "Curve Radius [m]: %.3f", avg_curverad);
    sprintf(text2, "Center Offset [m]: %.3f", veh_pos);
    
    putText(out_img, text1, Point(40, 70), FONT_HERSHEY_COMPLEX_SMALL, 1.6, Scalar(255, 0, 0), 2, LINE_AA);
    putText(out_img, text2, Point(40, 150), FONT_HERSHEY_COMPLEX_SMALL, 1.6, Scalar(255, 0, 0), 2, LINE_AA);
    
    return out_img;
}

Mat resize_for_screen(Mat& img) {
    int h = img.rows;
    int w = img.cols;
    double scale = std::min((double)SCR_W / w, (double)SCR_H / h);
    Mat resized;
    resize(img, resized, Size((int)(w * scale), (int)(h * scale)));
    return resized;
}

Mat binary_thresholder(Mat& img, double& processing_time) {
    clock_t start = clock();
    
    // TODO: Filtro HSV
    Mat hsv;
    cvtColor(img, hsv, COLOR_RGB2HSV);
    
    // Threshold adaptativo HSV
    std::vector<Mat> hsv_channels;
    split(hsv, hsv_channels);
    Mat adapt_white_hsv;
    adaptiveThreshold(hsv_channels[2], adapt_white_hsv, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 161, -27);
    
    // TODO: Fim Filtro HSV
    
    // Aplicado erosao na imagem gerada pelo HSV Adaptativo, eliminar pequenos ruídos
    Mat kernel_erode = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(adapt_white_hsv, adapt_white_hsv, kernel_erode, Point(-1, -1), 1);
    
    Mat final = adapt_white_hsv.clone();
    
    // Aplicando filtro de mediana na imagem combinada
    int median_kernel_size = 11; // Deve ser um numero Impar, como 3, 5, 7
    medianBlur(final, final, median_kernel_size);
    
    // Aplicando processamento morfologico de dilatacao para realcar as areas brancas
    Mat kernel_dilate = getStructuringElement(MORPH_RECT, Size(6, 6));
    Mat img_dilate;
    dilate(final, img_dilate, kernel_dilate, Point(-1, -1), 1);
    
    clock_t end = clock();
    processing_time = ((double)(end - start)) / CLOCKS_PER_SEC;
    
    return img_dilate;
}

int main() {
    // Open the ZED camera
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        return -1;
    }
    
    Resolution image_size;
    image_size.width = 1280;
    image_size.height = 720;
    
    // Set the video resolution to HD720
    cap.set(CAP_PROP_FRAME_WIDTH, image_size.width * 2);
    cap.set(CAP_PROP_FRAME_HEIGHT, image_size.height);
    
    int actual_width = (int)cap.get(CAP_PROP_FRAME_WIDTH);
    int actual_height = (int)cap.get(CAP_PROP_FRAME_HEIGHT);
    printf("Actual resolution: %dx%d\n", actual_width, actual_height);
    
    AlmostCan* almost_can = inicializar_serial();
    if (!almost_can) {
        return -1;
    }
    
    try {
        while (true) {
            try {
                clock_t inicio = clock();
                
                Mat images;
                bool retval = cap.read(images);
                if (!retval) continue;
                
                // Split stereo image
                Mat frame = images(Rect(0, 0, images.cols/2, images.rows));
                
                // Load test image instead (comment out for real camera)
                frame = imread("imagens_17_31_0063.png");
                if (frame.empty()) {
                    printf("Could not load test image\n");
                    continue;
                }
                
                int target_width = 1280;
                int target_height = 720;
                resize(frame, frame, Size(target_width, target_height));
                
                imshow("left RAW", frame);
                waitKey(1);
                
                Mat warped_frame, perspective_transform;
                warp(frame, warped_frame, perspective_transform);
                Mat resized_warped_frame = resize_for_screen(warped_frame);
                imshow("Warped Image", resized_warped_frame);
                waitKey(1);
                
                double tempo;
                Mat img_bin = binary_thresholder(warped_frame, tempo);
                
                printf("tempo de execucao bin: %.3f seg\n", tempo);
                
                imshow("Binary Image", img_bin);
                waitKey(1);
                
                std::vector<int> leftx, lefty, rightx, righty;
                find_lane_pixels_using_histogram(img_bin, leftx, lefty, rightx, righty);
                
                std::vector<double> left_fit, right_fit, left_fitx, right_fitx, ploty;
                fit_poly(img_bin, leftx, lefty, rightx, righty, left_fit, right_fit, left_fitx, right_fitx, ploty);
                
                double left_curverad, right_curverad;
                measure_curvature_meters(ploty, left_fitx, right_fitx, left_curverad, right_curverad);
                
                double vehicle_position = measure_position_meters(left_fit, right_fit);
                
                int steering_angle = calcular_angulo_do_volante(vehicle_position);
                
                double avg_curvature;
                Mat lane_overlay = project_lane_info(frame, img_bin, ploty, left_fitx, right_fitx,
                                                   perspective_transform, left_curverad, right_curverad, 
                                                   vehicle_position, avg_curvature);
                
                Mat resized_lane_overlay = resize_for_screen(lane_overlay);
                imshow("Lane Detection Result", resized_lane_overlay);
                waitKey(1);
                
                clock_t fim = clock();
                double tempo_total = ((double)(fim - inicio)) / CLOCKS_PER_SEC;
                
                printf("tempo de execucao: %.3f seg\n", tempo_total);
                
                enviar_dado(almost_can, steering_angle, 0x00);
                
                if (waitKey(1) == 'q') {
                    break;
                }
            } catch (...) {
                printf("Erro durante execução\n");
                continue;
            }
        }
    } catch (...) {
        printf("Execução interrompida pelo usuário.\n");
    }
    
    destroyAllWindows();
    
    if (almost_can) {
#ifdef _WIN32
        CloseHandle((HANDLE)almost_can->serial_fd);
#else
        close(almost_can->serial_fd);
#endif
        free(almost_can);
    }
    
    printf("Recursos liberados. Execução finalizada.\n");
    
    return 0;
}

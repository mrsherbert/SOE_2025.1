#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <thread>
#include <fstream>
#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <cstdio>       // Incluir as funções do C
#include <cstdint>

// Seleciona qual biblioteca será usada de acordo com o sistema operacional
#ifdef _WIN32
    #include <windows.h>
    #include <setupapi.h>
    #pragma comment(lib, "setupapi.lib")
#else
    #include <unistd.h>
    #include <fcntl.h>
    #include <termios.h>
    #include <dirent.h>
    #include <sys/stat.h>
    #include <glob.h>
#endif

using namespace cv;
using namespace std;

// ----------- MACROS / CONSTANTES ------------------------------------------------Add commentMore actions
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

// Diretiva para lidar com as portas serial
class SerialPort {
private:
#ifdef _WIN32
    HANDLE hSerial;
#else
    int fd;
#endif
    
public:
    SerialPort() {
#ifdef _WIN32
        hSerial = INVALID_HANDLE_VALUE;
#else
        fd = -1;
#endif
    }

// Aparentemente correta
class AlmostCan {
public:
    static const int SUCCESS = 0;
    static const int ERROR_TIMEOUT = 1;
    static const int ERROR_INVALID_SOF = 2;
    static const int ERROR_INVALID_EOF = 3;
    static const int ERROR_CRC_MISMATCH = 4;
    static const int ERROR_INVALID_LENGTH = 5;

private:
    int serial_fd;
    int baud_rate;
    unsigned char start_byte;
    unsigned char end_byte;
    int timeout_ms;
    char serial_port[256];

public:
    AlmostCan(const char* port, int baud, unsigned char start = 0xFF, unsigned char end = 0xF7, int timeout = 200) {
        strcpy(serial_port, port);
        baud_rate = baud;
        start_byte = start;
        end_byte = end;
        timeout_ms = timeout;
        serial_fd = -1;
    }

    ~AlmostCan() {
        if (serial_fd >= 0) {
            close(serial_fd);
        }
    }

    bool begin() {
        serial_fd = open(serial_port, O_RDWR | O_NOCTTY);
        if (serial_fd < 0) {
            return false;
        }

        struct termios tty;
        if (tcgetattr(serial_fd, &tty) != 0) {
            close(serial_fd);
            serial_fd = -1;
            return false;
        }

        speed_t speed;
        switch (baud_rate) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            default: speed = B115200; break;
        }

        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

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
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;

        tty.c_cc[VTIME] = timeout_ms / 100;
        tty.c_cc[VMIN] = 0;

        if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
            close(serial_fd);
            serial_fd = -1;
            return false;
        }

        return true;
    }

    void send_data(unsigned char id, unsigned char* data, int length) {
        unsigned char* packet = new unsigned char[length + 6];
        packet[0] = start_byte;
        packet[1] = id;
        packet[2] = length;
        
        memcpy(&packet[3], data, length);
        
        unsigned short crc = calculate_crc(&packet[1], 2 + length);
        packet[3 + length] = crc & 0xFF;
        packet[4 + length] = (crc >> 8) & 0xFF;
        packet[5 + length] = end_byte;
        
        write(serial_fd, packet, length + 6);
        delete[] packet;
    }

    int receive_data(unsigned char* id, unsigned char* data, int* length) {
        if (!wait_for_data(1)) {
            return ERROR_TIMEOUT;
        }

        unsigned char sof;
        if (read(serial_fd, &sof, 1) != 1 || sof != start_byte) {
            return ERROR_INVALID_SOF;
        }

        if (!wait_for_data(2)) {
            return ERROR_TIMEOUT;
        }

        unsigned char recv_id, recv_length;
        if (read(serial_fd, &recv_id, 1) != 1 || read(serial_fd, &recv_length, 1) != 1) {
            return ERROR_TIMEOUT;
        }

        if (recv_length > 32) {
            return ERROR_INVALID_LENGTH;
        }

        *id = recv_id;
        *length = recv_length;

        if (recv_length > 0) {
            if (!wait_for_data(recv_length)) {
                return ERROR_TIMEOUT;
            }
            if (read(serial_fd, data, recv_length) != recv_length) {
                return ERROR_TIMEOUT;
            }
        }

        if (!wait_for_data(3)) {
            return ERROR_TIMEOUT;
        }

        unsigned char crc_low, crc_high, eof;
        if (read(serial_fd, &crc_low, 1) != 1 || 
            read(serial_fd, &crc_high, 1) != 1 || 
            read(serial_fd, &eof, 1) != 1) {
            return ERROR_TIMEOUT;
        }

        if (eof != end_byte) {
            return ERROR_INVALID_EOF;
        }

        unsigned short received_crc = (crc_high << 8) | crc_low;
        
        unsigned char* crc_data = new unsigned char[recv_length + 2];
        crc_data[0] = recv_id;
        crc_data[1] = recv_length;
        memcpy(&crc_data[2], data, recv_length);
        
        unsigned short calculated_crc = calculate_crc(crc_data, recv_length + 2);
        delete[] crc_data;

        if (calculated_crc != received_crc) {
            return ERROR_CRC_MISMATCH;
        }

        return SUCCESS;
    }

    void set_start_byte(unsigned char start) {
        start_byte = start;
    }

    void set_end_byte(unsigned char end) {
        end_byte = end;
    }

    void set_timeout(int timeout) {
        timeout_ms = timeout;
        if (serial_fd >= 0) {
            struct termios tty;
            tcgetattr(serial_fd, &tty);
            tty.c_cc[VTIME] = timeout_ms / 100;
            tcsetattr(serial_fd, TCSANOW, &tty);
        }
    }

private:
    unsigned short calculate_crc(unsigned char* data, int length) {
        unsigned short crc = 0;
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

    bool wait_for_data(int expected_bytes) {
        struct timeval start, current;
        gettimeofday(&start, NULL);
        
        while (true) {
            int bytes_available;
            if (ioctl(serial_fd, FIONREAD, &bytes_available) == 0 && bytes_available >= expected_bytes) {
                return true;
            }
            
            gettimeofday(&current, NULL);
            long elapsed = (current.tv_sec - start.tv_sec) * 1000 + (current.tv_usec - start.tv_usec) / 1000;
            if (elapsed >= timeout_ms) {
                return false;
            }
            
            usleep(1000);
        }
    }
};


// Codigo equivalente ao implementado em python
// Aparentemente está funcional
vector<string> listar_portas_seriais() {
    vector<string> portas_disponiveis;
    
#ifdef __linux__
    glob_t glob_result;
    if (glob("/dev/tty[A-Za-z]*", GLOB_TILDE, NULL, &glob_result) == 0) {
        for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
            string porta = glob_result.gl_pathv[i];
            int fd = open(porta.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (fd >= 0) {
                close(fd);
                portas_disponiveis.push_back(porta);
            }
        }
    }
    globfree(&glob_result);
#elif defined(_WIN32)
    for (int i = 1; i < 256; i++) {
        string porta = "COM" + to_string(i);
        portas_disponiveis.push_back(porta);
    }
#else
    printf("Plataforma não suportada\n");
#endif
    
    return portas_disponiveis;
}

// Função aparentemente correta
string escolher_porta(const vector<string>& portas) {
    printf("Portas disponíveis:\n");
    for (size_t i = 0; i < portas.size(); i++) {
        printf("%zu: %s\n", i + 1, portas[i].c_str());
    }
    
    printf("Escolha uma porta pelo número: ");
    int escolha;
    scanf("%d", &escolha);
    
    if (escolha >= 1 && escolha <= (int)portas.size()) {
        return portas[escolha - 1];
    }
    return "";
}

// Função aparentemente correta
void enviar_dado(AlmostCan* almost_can, int dado, unsigned char id = 0x00) {
    // Equivalente try python
    try {
        // Ordenação de dados em C++
        unsigned char data_bytes[4];
        data_bytes[0] = (dado >> 24) & 0xFF;
        data_bytes[1] = (dado >> 16) & 0xFF;
        data_bytes[2] = (dado >> 8) & 0xFF;
        data_bytes[3] = dado & 0xFF;
        
        // Equivalência almost_can.send_data(id, data_bytes)
        almost_can->send_data(id, data_bytes, 4);
        printf("Dado %d enviado com sucesso\n", dado);
    // Equivalente except python
    } catch (const exception& e) {
        printf("Erro ao enviar dado: %s\n", e.what());
    }
}

// Função aparentemente correta
// Correto se a função for implementada corretamente
AlmostCan* inicializar_serial() {

    // Correto dependendo de listar_portas
    vector<string> portas_disponiveis = listar_portas_seriais();
    if (!portas_disponiveis.empty()) {

        // Correto dependendo de escolher_porta
        string porta_escolhida = escolher_porta(portas_disponiveis);
        printf("Porta escolhida: %s\n", porta_escolhida.c_str());
        
        try {
            AlmostCan* ac = new AlmostCan(porta_escolhida.c_str(), 115200, 0xFF, 0xF7, 1000);
            // Não aparenta estar certo
            if (ac->begin()) {
                sleep(2);
                return ac;
            } else {
                delete ac;
                printf("Erro ao abrir a porta %s\n", porta_escolhida.c_str());
                return nullptr;
            }
        } catch (const exception& e) {

            // Corretamente implementado
            printf("Erro ao acessar a porta %s: %s\n", porta_escolhida.c_str(), e.what());
            return nullptr;
        }
    } else {

        // Corretamente implementado
        printf("Nenhuma porta serial disponível.\n");
        return nullptr;
    }
}

// Função aparentemente correta
//  Equivalente a map_in_range(x: float, in_min: float, in_max: float, out_min: float, out_max: float)Add commentMore actions
//  Está equivalente sem necessidade de alteração
float map_in_range(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Função aparentemente correta
// Equivalente a def calcular_angulo_do_volante(distancia_em_metros):
//  Está equivalente sem necessidade de alteração
int calcular_angulo_do_volante(float distancia_em_metros) {
	int angulo = map_in_range(distancia_em_metros, -1, 1, 50, 0)
	return (int)angulo;
}

// Função aparentemente correta
pair<Mat, Mat> warp(const Mat& img) {
    // Equivalente a src = np.float32([[0, IMG_Hs], [IMG_Ws, IMG_Hs], [100, 287], [IMG_W-100, 287]]) e 
    // dst = np.float32([[0, IMG_Hs], [IMG_Ws, IMG_Hs], [0, 0], [IMG_W, 0]])
    Point2f src[4] = {
        Point2f(0, IMG_Hs),
        Point2f(IMG_Ws, IMG_Hs),
        Point2f(100, 287),
        Point2f(IMG_W - 100, 287)
    };
    
    Point2f dst[4] = {
        Point2f(0, IMG_Hs),
        Point2f(IMG_Ws, IMG_Hs),
        Point2f(0, 0),
        Point2f(IMG_W, 0)
    };
    
    // getPerspectiveTransform e Equivalente a cv2.getPerspectiveTransform
    Mat M = getPerspectiveTransform(src, dst);
    Mat M_inv = getPerspectiveTransform(dst, src);
    
    // warpPerspective e Equivalente a cv2.warpPerspective
    Mat warped;
    warpPerspective(img, warped, M, Size(IMG_Ws, IMG_Hs));
    
    // Retorna o par de valores
    return make_pair(warped, M_inv);
}

// Função aparentemente correta
// Cria uma estrutura que será usada para retornar 4 valores de uma vez
struct LanePixels {
    vector<int> leftx, lefty, rightx, righty;
};

// Função aparentemente correta
// Implementa uma função que vai retornar a variavel de 4 valores
LanePixels find_lane_pixels_using_histogram(const Mat& binary_warped) {
    
    // criação de matriz unidimensional / histograma
    Mat histogram = Mat::zeros(1, binary_warped.cols, CV_32F);

    // Soma equivalente ao codigo em python
    int start_row = binary_warped.rows / 2;
    
    for (int x = 0; x < binary_warped.cols; x++) {
        int sum = 0;
        for (int y = start_row; y < binary_warped.rows; y++) {
            if (binary_warped.at<uchar>(y, x) > 0) {
                sum++;
            }
        }
        histogram.at<float>(0, x) = sum;
    }
    
    // Encontra o valor médio dos valores
    int midpoint = histogram.cols / 2;
    double max_val_left, max_val_right;
    Point max_loc_left, max_loc_right;
    Mat left_half = histogram(Rect(0, 0, midpoint, 1));
    Mat right_half = histogram(Rect(midpoint, 0, histogram.cols - midpoint, 1));
    // A função minMaxLoc encontra os minimos e maximos locais 
    // https://docs.opencv.org/4.x/d2/de8/group__core__array.html
    minMaxLoc(left_half, nullptr, &max_val_left, nullptr, &max_loc_left);
    minMaxLoc(right_half, nullptr, &max_val_right, nullptr, &max_loc_right);
    
    // Implementando igualmente em python
    int leftx_base = max_loc_left.x;
    int rightx_base = max_loc_right.x + midpoint;
    
    // Parametros corretos
    int nwindows = 7;
    int margin = 100;
    int minpix = 50;
    int window_height = (binary_warped.rows / 2) / nwindows;
    
    // Procura os valores não zerados
    vector<Point> nonzero_points;
    for (int y = 0; y < binary_warped.rows; y++) {
        for (int x = 0; x < binary_warped.cols; x++) {
            if (binary_warped.at<uchar>(y, x) > 0) {
                nonzero_points.push_back(Point(x, y));
            }
        }
    }

    // Implementado correto aparentemente
    int leftx_current = leftx_base;
    int rightx_current = rightx_base;
    
    // Implementado correto aparentemente
    vector<vector<int>> left_lane_inds(nwindows);
    vector<vector<int>> right_lane_inds(nwindows);
    
    // Implementação correta
    for (int window = 0; window < nwindows; window++) {
        // Implementação correta
        int win_y_low = binary_warped.rows - (window + 1) * window_height;
        int win_y_high = binary_warped.rows - window * window_height;
        int win_xleft_low = leftx_current - margin;
        int win_xleft_high = leftx_current + margin;
        int win_xright_low = rightx_current - margin;
        int win_xright_high = rightx_current + margin;
        
        for (size_t i = 0; i < nonzero_points.size(); i++) {
            Point p = nonzero_points[i];
            
            // Check if point is in left window
            if (p.y >= win_y_low && p.y < win_y_high && 
                p.x >= win_xleft_low && p.x < win_xleft_high) {
                left_lane_inds[window].push_back(i);
            }
            
            // Check if point is in right window
            if (p.y >= win_y_low && p.y < win_y_high && 
                p.x >= win_xright_low && p.x < win_xright_high) {
                right_lane_inds[window].push_back(i);
            }
        }
        

        // NÃO CONSEGUI ENTENDER 
        if (left_lane_inds[window].size() > minpix) {
            int sum_x = 0;
            for (int idx : left_lane_inds[window]) {
                sum_x += nonzero_points[idx].x;
            }
            leftx_current = sum_x / left_lane_inds[window].size();
        }
        
        if (right_lane_inds[window].size() > minpix) {
            int sum_x = 0;
            for (int idx : right_lane_inds[window]) {
                sum_x += nonzero_points[idx].x;
            }
            rightx_current = sum_x / right_lane_inds[window].size();
        }
    }
    
    // Concatenate arrays of indices
    LanePixels result;
    for (int window = 0; window < nwindows; window++) {
        for (int idx : left_lane_inds[window]) {
            result.leftx.push_back(nonzero_points[idx].x);
            result.lefty.push_back(nonzero_points[idx].y);
        }
        for (int idx : right_lane_inds[window]) {
            result.rightx.push_back(nonzero_points[idx].x);
            result.righty.push_back(nonzero_points[idx].y);
        }
    }
    
    return result;
}

// Função aparentemente correta
// Estrutura usada muitas vezes ao longo do codigo que aparenta se adequar para transformação em C++
struct PolyFitResult {
    vector<double> left_fit, right_fit;
    vector<double> left_fitx, right_fitx, ploty;
};

// Função aparentemente correta
// Essa função imita a função np.polyfit
vector<double> polyfit(const vector<int>& x, const vector<int>& y, int degree) {
    if (x.size() != y.size() || x.size() < degree + 1) {
        return vector<double>(degree + 1, 1.0);
    }
    
    // 
    int n = x.size();
    int m = degree + 1;
    
    // Create Vandermonde matrix
    vector<vector<double>> A(n, vector<double>(m));
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            A[i][j] = pow(x[i], j);
        }
    }
    
    // Create b vector
    vector<double> b(n);
    for (int i = 0; i < n; i++) {
        b[i] = y[i];
    }
    
    // Solve normal equations: A^T * A * coeffs = A^T * b
    vector<vector<double>> AtA(m, vector<double>(m, 0));
    vector<double> Atb(m, 0);
    
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < m; j++) {
            for (int k = 0; k < n; k++) {
                AtA[i][j] += A[k][i] * A[k][j];
            }
        }
        for (int k = 0; k < n; k++) {
            Atb[i] += A[k][i] * b[k];
        }
    }
    
    // Gaussian elimination
    vector<double> coeffs(m);
    for (int i = 0; i < m; i++) {
        int pivot = i;
        for (int j = i + 1; j < m; j++) {
            if (abs(AtA[j][i]) > abs(AtA[pivot][i])) {
                pivot = j;
            }
        }
        
        if (pivot != i) {
            swap(AtA[i], AtA[pivot]);
            swap(Atb[i], Atb[pivot]);
        }
        
        if (abs(AtA[i][i]) < 1e-10) {
            return vector<double>(degree + 1, 1.0);
        }
        
        for (int j = i + 1; j < m; j++) {
            double factor = AtA[j][i] / AtA[i][i];
            for (int k = i; k < m; k++) {
                AtA[j][k] -= factor * AtA[i][k];
            }
            Atb[j] -= factor * Atb[i];
        }
    }
    
    // Back substitution
    for (int i = m - 1; i >= 0; i--) {
        coeffs[i] = Atb[i];
        for (int j = i + 1; j < m; j++) {
            coeffs[i] -= AtA[i][j] * coeffs[j];
        }
        coeffs[i] /= AtA[i][i];
    }
    
    // Reverse coefficients for numpy polyfit compatibility
    reverse(coeffs.begin(), coeffs.end());
    return coeffs;
}

// Função aparentemente correta
PolyFitResult fit_poly(const Mat& binary_warped, const vector<int>& leftx, const vector<int>& lefty, 
                      const vector<int>& rightx, const vector<int>& righty) {
    // Instancia a variavel result
    PolyFitResult result;
    
    // polyfit é a função equivalente a cv2.polyfit
    result.left_fit = polyfit(lefty, leftx, 2);
    result.right_fit = polyfit(righty, rightx, 2);
    
    // Equivalente a fazer np.linspace
    for (int y = 0; y < binary_warped.rows; y++) {
        result.ploty.push_back(y);
    }
    
    // Usa a try equivalente em c++
    // Aparentemente correta
    try {
        for (double y : result.ploty) {
            double left_x = result.left_fit[0] * y * y + result.left_fit[1] * y + result.left_fit[2];
            double right_x = result.right_fit[0] * y * y + result.right_fit[1] * y + result.right_fit[2];
            result.left_fitx.push_back(left_x);
            result.right_fitx.push_back(right_x);
        }
    } catch (const exception& e) {
        printf("The function failed to fit a line!\n");
        for (double y : result.ploty) {
            result.left_fitx.push_back(y * y + y);
            result.right_fitx.push_back(y * y + y);
        }
    }
    
    return result;
}

// Função aparentemente correta
// Pair para retornar dois dobles
pair<double, double> measure_curvature_meters(const vector<double>& ploty, 
                                             const vector<double>& left_fitx, 
                                             const vector<double>& right_fitx) {
    // For para criação de vetor de conversão
    vector<int> ploty_world, left_fitx_world, right_fitx_world;
    for (size_t i = 0; i < ploty.size(); i++) {
        ploty_world.push_back((int)(ploty[i] * YM_PER_PIX));
        left_fitx_world.push_back((int)(left_fitx[i] * XM_PER_PIX));
        right_fitx_world.push_back((int)(right_fitx[i] * XM_PER_PIX));
    }
    
    // Aparentemente correto a depender de polyfit
    vector<double> left_fit_cr = polyfit(ploty_world, left_fitx_world, 2);
    vector<double> right_fit_cr = polyfit(ploty_world, right_fitx_world, 2);
    
    // max_element encontra o valor maximo num conjunto de valores
    // Equivalente a np.max() em python
    // https://en.cppreference.com/w/cpp/algorithm/max_element.html
    double y_eval = *max_element(ploty.begin(), ploty.end());
    
    // Aparentemente correto
    double left_c  = pow(1 + pow(2 * left_fit_cr[0] * y_eval * YM_PER_PIX + left_fit_cr[1], 2), 1.5) / std::abs(2 * left_fit_cr[0]);
    double right_c = pow(1 + pow(2 * right_fit_cr[0] * y_eval * YM_PER_PIX + right_fit_cr[1], 2), 1.5) / std::abs(2 * right_fit_cr[0]);
    
    return make_pair(left_c, right_c);
}

// Função aparentemente correta
// Aparentemente correta
double measure_position_meters(const vector<double>& left_fit, const vector<double>& right_fit) {
    // Aparentemente correto
    double y_max = IMG_Hs;
    double left_x = left_fit[0] * y_max * y_max + left_fit[1] * y_max + left_fit[2];
    double right_x = right_fit[0] * y_max * y_max + right_fit[1] * y_max + right_fit[2];
    double lane_mid = (left_x + right_x) / 2;
    double veh_pos = ((IMG_Ws / 2.0) - lane_mid) * XM_PER_PIX;
    return veh_pos - 0.10;
}

// Função aparentemente correta
// A função pair se usa quando vai tratar de duas variaveis ao mesmo tempo ou retornar duas variaveis
// https://workat.tech/problem-solving/tutorial/cpp-stl-pair-complete-guide-ia62jqg0dszu
pair<Mat, double> project_lane_info(const Mat& img, const Mat& binary_warped, 
                                   const vector<double>& ploty, const vector<double>& left_fitx, 
                                   const vector<double>& right_fitx, const Mat& M_inv,
                                   double left_curverad, double right_curverad, double veh_pos) {
    
    // Correta aplicação da função Mat::zeros 
    Mat warp_zero = Mat::zeros(binary_warped.size(), CV_8UC1);

    // Esse parte converte uma imagem em escala de cinza (warp_zero, com 1 canal) 
    // para uma imagem colorida RGB (na verdade, BGR no OpenCV) com 3 canais.
    // Usando COLOR_GRAY2BGR, equivalente a np.dstack((warp_zero, warp_zero, warp_zero))
    Mat color_warp;
    cvtColor(warp_zero, color_warp, COLOR_GRAY2BGR);
    
    // Cria os pontos para as duplas
    vector<Point> pts_left, pts_right;

    // Montam os vetores, ordenados em pares (x0,y0),(x1,y1),...
    for (size_t i = 0; i < ploty.size(); i++) {
        pts_left.push_back(Point((int)left_fitx[i], (int)ploty[i]));
    }
    for (int i = ploty.size() - 1; i >= 0; i--) {
        pts_right.push_back(Point((int)right_fitx[i], (int)ploty[i]));
    }

    // pts.insert(...): concatena as listas como em np.hstack
    // https://cplusplus.com/reference/vector/vector/insert/#google_vignette
    vector<Point> pts;
    pts.insert(pts.end(), pts_left.begin(), pts_left.end());
    pts.insert(pts.end(), pts_right.begin(), pts_right.end());
    
    // Equivalente c++ para cv2.fillPoly
    // Aplicado no escalar na color_warp
    fillPoly(color_warp, pts, Scalar(0, 255, 0));
    
    // Equivalente c++ cv2.warpPerspective
    // Aplica uma transformação na perspectiva da imagem
    // Aplica o valor color_warp transformado em newwarp
    Mat newwarp;
    warpPerspective(color_warp, newwarp, M_inv, Size(img.cols, img.rows));
    
    // Equivalente c++ para cv2.addWeighted
    // Os parametros estão devidamente ajustados
    Mat out_img;
    addWeighted(img, 1, newwarp, 0.3, 0, out_img);
    
    // Aplicada semelhantemente ao codigo
    double avg_curverad = (left_curverad + right_curverad) / 2;
    
    // Aparentemente certa, a depender se houver estouro de 7 bits
    string curve_text = "Curve Radius [m]: " + to_string(avg_curverad).substr(0, 7);
    string offset_text = "Center Offset [m]: " + to_string(veh_pos).substr(0, 7);
    
    // Equivalente c++ cv2.putText
    putText(out_img, curve_text, Point(40, 70), FONT_HERSHEY_COMPLEX_SMALL, 1.6, Scalar(255, 0, 0), 2, LINE_AA);
    putText(out_img, offset_text, Point(40, 150), FONT_HERSHEY_COMPLEX_SMALL, 1.6, Scalar(255, 0, 0), 2, LINE_AA);
    
    return make_pair(out_img, avg_curverad);
}

// Função aparentemente correta
// Aparentemente correta
Mat resize_for_screen(const Mat& img) {
    double scale_w = (double)SCR_W / img.cols;
    double scale_h = (double)SCR_H / img.rows;

    // Implementa a função python
    // https://cplusplus.com/reference/algorithm/min/
    double scale = min(scale_w, scale_h);
    
    // Cria uma varkavel do tipo Mat
    Mat resized;

    // Correta aplicação da função resize
    resize(img, resized, Size((int)(img.cols * scale), (int)(img.rows * scale)));
    return resized;
}

// Função aparentemente correta
pair<Mat, double> binary_thresholder(const Mat& img) {

    // Corretamente equivalente a função python time.time()
    // https://en.cppreference.com/w/cpp/chrono/high_resolution_clock/now
    auto start = chrono::high_resolution_clock::now();
    
    // TODO: Filtro HSV
    Mat hsv;
    
    // Função equivalente a do python cv2.cvtColor()
    // https://docs.opencv.org/4.x/db/d64/tutorial_js_colorspaces.html 
    cvtColor(img, hsv, COLOR_RGB2HSV);
    
    // Aparentemente aplicavel
    Mat roi = hsv(Rect(0, 480/2, hsv.cols, hsv.rows - 480/2));
    
    // Extract V channel and apply adaptive threshold
    vector<Mat> hsv_channels;
    split(roi, hsv_channels);
    Mat v_channel = hsv_channels[2];
    
    // adaptiveThreshold é equivalente a cv2.adaptiveThreshold em c++
    Mat adapt_white_hsv;
    adaptiveThreshold(v_channel, adapt_white_hsv, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 161, -27);
    // TODO: Fim Filtro HSV
    
    // Aplicado erosao na imagem gerada pelo HSV Adaptativo, eliminar pequenos ruIdos
    Mat kernel_erode = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(adapt_white_hsv, adapt_white_hsv, kernel_erode, Point(-1, -1), 1);
    
    Mat final = adapt_white_hsv;
    
    // Aplicando filtro de mediana na imagem combinada
    // medianBLur equivalente cv2.medianBlur
    int median_kernel_size = 7;  // Deve ser um numero Impar, como 3, 5, 7
    medianBlur(final, final, median_kernel_size);
    
    // Aplicando processamento morfologico de dilatacao para realcar as areas brancas
    Mat kernel_dilate = getStructuringElement(MORPH_RECT, Size(6, 6));
    Mat img_dilate;

    // dilate equivalente a cv2.dilate
    dilate(final, img_dilate, kernel_dilate, Point(-1, -1), 1);
    
    // Aplicação devida do processo de adiquirir o tempo
    auto end = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(end - start);
    double elapsed_time = duration.count() / 1000000.0;
    
    // Retorno da função implementado
    return make_pair(img_dilate, elapsed_time);
}

// Função aparentemente correta
// Estrutura com as definições width e height disponibilizaveis como class em python
// Aparentemente correta
// https://www.inf.pucrs.br/~pinho/LaproI/Structs/Structs.htm
typedef struct {
    int width;
    int height;
} Resolution;

// Função aparentemente correta
// Aparentemente correta
int main() {

    // Criação de struct
    Resolution image_size;
    image_size.width = 1280;
    image_size.height = 720;

    // Valores informados
    int target_width = 1280;
    int target_height = 720;
    int target_widthsmall = 640;
    int target_heightsmall = 480;
    int roi_start_row = target_heightsmall / 2;
    
    // top_padding = np.zeros((roi_start_row, target_widthsmall), dtype=np.uint8)
    // Cria número de linhas roi_start_row, e numero de colunas target_widthsmall
    // Cria uma imagem preta (preenchida com zeros) de tamanho roi_start_row x target_widthsmall
    // zeros (int rows, int cols, int type) disponivel em https://docs.opencv.org/4.x/d3/d63/classcv_1_1Mat.html
    Mat top_padding = Mat::zeros(roi_start_row, target_widthsmall, CV_8UC1);
    

    // Estrutura para exceptions em C++
    // Definida em https://www.w3schools.com/cpp/cpp_exceptions.asp
    // try em python parecido com try em c++
    // except em python parecido com catch em c++ 
    try {
        while (true) {
            // Estrutura para exceptions em C++
            // Definida em https://www.w3schools.com/cpp/cpp_exceptions.asp
            // try em python parecido com try em c++
            // except em python parecido com catch em c++ 
            try {

                // Em C++, a palavra-chave auto faz o compilador deduzir automaticamente 
                // o tipo da variável com base no valor usado para inicializá-la. Isso é 
                // especialmente útil com tipos complexos, como os da biblioteca <chrono>.
                // https://en.cppreference.com/w/cpp/chrono/high_resolution_clock/now
                // A função std::chrono::high_resolution_clock::now() retorna um ponto no 
                // tempo que representa a hora atual. 
                auto inicio = chrono::high_resolution_clock::now();
                
                // cv::Mat frame: cria uma matriz que irá armazenar os dados da imagem.
                Mat frame = imread("imagens_17_31_0063.png");
                
                // resize(imagem entrada, imagem saída, size(largura, altura) )
                // aparenta estar correto 
                resize(frame, frame, Size(target_widthsmall, target_heightsmall));

                // Essa parte está correta a depender da função warp
                auto warp_result = warp(frame);
                Mat warped_frame = warp_result.first;
                Mat perspective_transform = warp_result.second;


                // Essa parte está correta a depender da função binary_thresholder
                auto binary_result = binary_thresholder(warped_frame);
                Mat img_bin = binary_result.first;
                double tempo = binary_result.second;
                
                printf("tempo de execucao bin: %.3f seg\n", tempo);
                
                // Add top padding
                Mat padded_img_bin;
                vconcat(top_padding, img_bin, padded_img_bin);
                img_bin = padded_img_bin;

                // Equivalente da função python cv2.imshow() -> https://docs.opencv.org/4.x/d7/dfc/group__highgui.html
                imshow("Binary Image", img_bin);
                // Equivalente da função python cv2.waitKey() -> https://docs.opencv.org/4.x/d7/dfc/group__highgui.html
                waitKey(1);
                
                // Correto, vai depender apenas da correta implementação de find_lane_pixels_using_histogram
                LanePixels lane_pixels = find_lane_pixels_using_histogram(img_bin);
                PolyFitResult poly_result = fit_poly(img_bin, lane_pixels.leftx, lane_pixels.lefty, 
                                                   lane_pixels.rightx, lane_pixels.righty);
                
                // Correto, vai depender apenas da correta implementação de measure_curvature_meters
                auto curvature = measure_curvature_meters(poly_result.ploty, poly_result.left_fitx, poly_result.right_fitx);
                double left_curverad = curvature.first;
                double right_curverad = curvature.second;

                // Correto, vai depender apenas da correta implementação de measure_position_meters
                double vehicle_position = measure_position_meters(poly_result.left_fit, poly_result.right_fit);

                // Correto, vai depender apenas da correta implementação de calcular_angulo_do_volante
                int steering_angle = calcular_angulo_do_volante(vehicle_position);
                int steering_angle_int = steering_angle;
                
                // Correto, vai depender apenas da correta implementação de project_lane_info
                auto lane_info = project_lane_info(
                    frame, img_bin, poly_result.ploty, poly_result.left_fitx, poly_result.right_fitx,
                    perspective_transform, left_curverad, right_curverad, vehicle_position
                );
                Mat lane_overlay = lane_info.first;
                double avg_curvature = lane_info.second;
                
                // Correto, vai depender apenas da correta implementação de resize_for_screen
                Mat resized_lane_overlay = resize_for_screen(lane_overlay);

                // Equivalente da função python cv2.imshow() -> https://docs.opencv.org/4.x/d7/dfc/group__highgui.html
                imshow("Lane Detection Result", resized_lane_overlay);
                // Equivalente da função python cv2.waitKey() -> https://docs.opencv.org/4.x/d7/dfc/group__highgui.html
                waitKey(1);
                
                // https://en.cppreference.com/w/cpp/chrono/high_resolution_clock/now 
                auto fim = chrono::high_resolution_clock::now();

                // A função chrono::duration_cast<chrono::microseconds> pode medir o tempo passado https://en.cppreference.com/w/cpp/chrono/duration/duration_cast
                auto duration = chrono::duration_cast<chrono::microseconds>(fim - inicio);
                // Aparentemente certo
                double elapsed_time = duration.count() / 1000000.0;
                
                printf("tempo de execucao: %.3f seg\n", elapsed_time);

                //enviar_dado(almost_can, steering_angle_int, 0x00);

                if (waitKey(1) == 'q') {
                    break;
                }

            } catch (const exception& ex) {
                printf("Erro: %s\n", ex.what());
                continue;
            }
        }
    } catch (const exception& e) {
    // Não é necessario.
    }
    
    destroyAllWindows();
    //Não foi encontrado equivalente para
    // import gc
    // gc.collect()
    printf("Recursos liberados. Execução finalizada.\n");
    
    return 0;
}
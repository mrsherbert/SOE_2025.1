
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
#include <cstdint>

// Platform-specific includes for serial communication
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

// ---------- MACROS / CONSTANTES ------------------------------------------------
const int IMG_W = 1280, IMG_H = 720;           // resolução de trabalho (warp, YOLO, máscaras)
const int IMG_Ws = 640, IMG_Hs = 480;          // resolução de trabalho (warp, YOLO, máscaras)
const int SCR_W = 800, SCR_H = 600;            // tamanho das janelas do cv2.imshow

// conversões pista-real (ajuste conforme a largura real da faixa):
const double YM_PER_PIX = 2.40 / 480;          // m por pixel no eixo Y      (≈ altura)
const double XM_PER_PIX = 1.00 / 350;          // m por pixel no eixo X      (≈ largura da pista)

const double CONF_THRESHOLD = 0.30;            // limiar de confiança p/ YOLO
// ------------------------------------------------------------------------------

int posix_ponto_1_x = 0;
int posix_ponto_1_y = 0;

int posix_ponto_2_x = 0;
int posix_ponto_2_y = 0;

int posix_ponto_3_x = 0;
int posix_ponto_3_y = 0;

int posix_ponto_4_x = 0;
int posix_ponto_4_y = 0;

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
    
    bool open(const string& port, int baudRate) {
#ifdef _WIN32
        hSerial = CreateFileA(port.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
        if (hSerial == INVALID_HANDLE_VALUE) return false;
        
        DCB dcbSerialParams = {0};
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
        if (!GetCommState(hSerial, &dcbSerialParams)) return false;
        
        dcbSerialParams.BaudRate = baudRate;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;
        
        if (!SetCommState(hSerial, &dcbSerialParams)) return false;
        
        COMMTIMEOUTS timeouts = {0};
        timeouts.ReadIntervalTimeout = 50;
        timeouts.ReadTotalTimeoutConstant = 50;
        timeouts.ReadTotalTimeoutMultiplier = 10;
        timeouts.WriteTotalTimeoutConstant = 50;
        timeouts.WriteTotalTimeoutMultiplier = 10;
        
        if (!SetCommTimeouts(hSerial, &timeouts)) return false;
        return true;
#else
        fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0) return false;
        
        struct termios tty;
        if (tcgetattr(fd, &tty) != 0) return false;
        
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 5;
        
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        
        if (tcsetattr(fd, TCSANOW, &tty) != 0) return false;
        return true;
#endif
    }
    
    void close() {
#ifdef _WIN32
        if (hSerial != INVALID_HANDLE_VALUE) {
            CloseHandle(hSerial);
            hSerial = INVALID_HANDLE_VALUE;
        }
#else
        if (fd >= 0) {
            ::close(fd);
            fd = -1;
        }
#endif
    }
    
    bool isOpen() const {
#ifdef _WIN32
        return hSerial != INVALID_HANDLE_VALUE;
#else
        return fd >= 0;
#endif
    }
    
    int write(const vector<uint8_t>& data) {
#ifdef _WIN32
        DWORD bytesWritten;
        if (!WriteFile(hSerial, data.data(), data.size(), &bytesWritten, NULL)) return -1;
        return bytesWritten;
#else
        return ::write(fd, data.data(), data.size());
#endif
    }
    
    int read(vector<uint8_t>& buffer, int maxBytes) {
#ifdef _WIN32
        DWORD bytesRead;
        buffer.resize(maxBytes);
        if (!ReadFile(hSerial, buffer.data(), maxBytes, &bytesRead, NULL)) return -1;
        buffer.resize(bytesRead);
        return bytesRead;
#else
        buffer.resize(maxBytes);
        int bytesRead = ::read(fd, buffer.data(), maxBytes);
        if (bytesRead > 0) buffer.resize(bytesRead);
        return bytesRead;
#endif
    }
    
    int available() {
#ifdef _WIN32
        COMSTAT comstat;
        DWORD errors;
        if (ClearCommError(hSerial, &errors, &comstat)) {
            return comstat.cbInQue;
        }
        return 0;
#else
        int bytes;
        ioctl(fd, FIONREAD, &bytes);
        return bytes;
#endif
    }
};

class AlmostCan {
public:
    static const int SUCCESS = 0;
    static const int ERROR_TIMEOUT = 1;
    static const int ERROR_INVALID_SOF = 2;
    static const int ERROR_INVALID_EOF = 3;
    static const int ERROR_CRC_MISMATCH = 4;
    static const int ERROR_INVALID_LENGTH = 5;

private:
    SerialPort serialPort;
    int baud;
    uint8_t start_byte;
    uint8_t end_byte;
    int timeout;

public:
    AlmostCan(const string& serial_port, int baud_rate, uint8_t start_byte = 0xFF, uint8_t end_byte = 0xF7, int timeout = 200)
        : baud(baud_rate), start_byte(start_byte), end_byte(end_byte), timeout(timeout) {
        serialPort.open(serial_port, baud_rate);
    }

    void begin() {
        if (!serialPort.isOpen()) {
            // Port should already be opened in constructor
        }
    }

    void send_data(uint8_t id, const vector<uint8_t>& data) {
        int length = data.size();
        vector<uint8_t> packet;
        packet.push_back(start_byte);
        packet.push_back(id);
        packet.push_back(length);
        packet.insert(packet.end(), data.begin(), data.end());

        vector<uint8_t> crc_data;
        crc_data.push_back(id);
        crc_data.push_back(length);
        crc_data.insert(crc_data.end(), data.begin(), data.end());
        
        uint16_t crc = calculate_crc(crc_data);
        packet.push_back(crc & 0xFF);
        packet.push_back((crc >> 8) & 0xFF);
        packet.push_back(end_byte);

        serialPort.write(packet);
    }

    int receive_data(uint8_t& id, vector<uint8_t>& data) {
        if (!wait_for_data(1)) {
            return ERROR_TIMEOUT;
        }

        vector<uint8_t> buffer;
        if (serialPort.read(buffer, 1) != 1 || buffer[0] != start_byte) {
            return ERROR_INVALID_SOF;
        }

        if (!wait_for_data(2)) {
            return ERROR_TIMEOUT;
        }

        if (serialPort.read(buffer, 1) != 1) return ERROR_TIMEOUT;
        id = buffer[0];
        
        if (serialPort.read(buffer, 1) != 1) return ERROR_TIMEOUT;
        uint8_t length = buffer[0];

        if (length > 32) {
            return ERROR_INVALID_LENGTH;
        }

        data.clear();
        if (length > 0) {
            if (!wait_for_data(length)) {
                return ERROR_TIMEOUT;
            }

            if (serialPort.read(data, length) != length) {
                return ERROR_TIMEOUT;
            }
        }

        if (!wait_for_data(3)) {
            return ERROR_TIMEOUT;
        }

        vector<uint8_t> crc_eof_buffer;
        if (serialPort.read(crc_eof_buffer, 3) != 3) {
            return ERROR_TIMEOUT;
        }
        
        uint8_t crc_low = crc_eof_buffer[0];
        uint8_t crc_high = crc_eof_buffer[1];
        uint8_t eof = crc_eof_buffer[2];

        if (eof != end_byte) {
            return ERROR_INVALID_EOF;
        }

        uint16_t received_crc = (crc_high << 8) | crc_low;
        
        vector<uint8_t> crc_data;
        crc_data.push_back(id);
        crc_data.push_back(length);
        crc_data.insert(crc_data.end(), data.begin(), data.end());
        
        uint16_t calculated_crc = calculate_crc(crc_data);

        if (calculated_crc != received_crc) {
            return ERROR_CRC_MISMATCH;
        }

        return SUCCESS;
    }

    void set_start_byte(uint8_t start_byte) {
        this->start_byte = start_byte;
    }

    void set_end_byte(uint8_t end_byte) {
        this->end_byte = end_byte;
    }

    void set_timeout(int timeout) {
        this->timeout = timeout;
    }

    uint16_t calculate_crc(const vector<uint8_t>& data) {
        uint16_t crc = 0;
        for (uint8_t b : data) {
            crc ^= b;
            for (int i = 0; i < 8; i++) {
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
        auto start_time = chrono::steady_clock::now();
        while (serialPort.available() < expected_bytes) {
            auto current_time = chrono::steady_clock::now();
            auto elapsed = chrono::duration_cast<chrono::milliseconds>(current_time - start_time);
            if (elapsed.count() >= timeout) {
                return false;
            }
            this_thread::sleep_for(chrono::milliseconds(1));
        }
        return true;
    }
    
    void close() {
        serialPort.close();
    }
};

vector<string> listar_portas_seriais() {
    vector<string> portas_disponiveis;
    
#ifdef _WIN32
    for (int i = 1; i < 256; i++) {
        portas_disponiveis.push_back("COM" + to_string(i));
    }
#else
    // Linux/Unix
    glob_t glob_result;
    int glob_status = glob("/dev/tty[A-Za-z]*", GLOB_TILDE, NULL, &glob_result);
    
    if (glob_status == 0) {
        for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
            string porta = glob_result.gl_pathv[i];
            // Try to open the port to check if it's available
            int test_fd = open(porta.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (test_fd >= 0) {
                close(test_fd);
                portas_disponiveis.push_back(porta);
            }
        }
    }
    globfree(&glob_result);
#endif
    
    return portas_disponiveis;
}

string escolher_porta(const vector<string>& portas) {
    cout << "Portas disponíveis:" << endl;
    for (size_t i = 0; i < portas.size(); i++) {
        cout << (i + 1) << ": " << portas[i] << endl;
    }
    cout << "Escolha uma porta pelo número: ";
    int escolha;
    cin >> escolha;
    return portas[escolha - 1];
}

void enviar_dado(AlmostCan& almost_can, int dado, uint8_t id = 0x00) {
    try {
        vector<uint8_t> data_bytes;
        data_bytes.push_back((dado >> 24) & 0xFF);
        data_bytes.push_back((dado >> 16) & 0xFF);
        data_bytes.push_back((dado >> 8) & 0xFF);
        data_bytes.push_back(dado & 0xFF);
        
        almost_can.send_data(id, data_bytes);
        cout << "Dado " << dado << " enviado com sucesso" << endl;
    } catch (const exception& e) {
        cout << "Erro ao enviar dado: " << e.what() << endl;
    }
}

AlmostCan* inicializar_serial() {
    vector<string> portas_disponiveis = listar_portas_seriais();
    if (!portas_disponiveis.empty()) {
        string porta_escolhida = escolher_porta(portas_disponiveis);
        cout << "Porta escolhida: " << porta_escolhida << endl;
        try {
            AlmostCan* ac = new AlmostCan(porta_escolhida, 115200, 0xFF, 0xF7, 1000);
            ac->begin();
            this_thread::sleep_for(chrono::seconds(2));
            return ac;
        } catch (const exception& e) {
            cout << "Erro ao acessar a porta " << porta_escolhida << ": " << e.what() << endl;
            return nullptr;
        }
    } else {
        cout << "Nenhuma porta serial disponível." << endl;
        return nullptr;
    }
}

double map_in_range(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int calcular_angulo_do_volante(double distancia_em_metros) {
    int angulo = static_cast<int>(map_in_range(distancia_em_metros, -1, 1, 50, 0));
    return angulo;
}

pair<Mat, Mat> warp(const Mat& img) {
    vector<Point2f> src = {
        Point2f(0, IMG_Hs),
        Point2f(IMG_Ws, IMG_Hs),
        Point2f(100, 287),
        Point2f(IMG_W - 100, 287)
    };
    
    vector<Point2f> dst = {
        Point2f(0, IMG_Hs),
        Point2f(IMG_Ws, IMG_Hs),
        Point2f(0, 0),
        Point2f(IMG_W, 0)
    };
    
    Mat M = getPerspectiveTransform(src, dst);
    Mat M_inv = getPerspectiveTransform(dst, src);
    
    Mat warped;
    warpPerspective(img, warped, M, Size(IMG_Ws, IMG_Hs));
    
    return make_pair(warped, M_inv);
}

void find_lane_pixels_using_histogram(const Mat& binary_warped, vector<int>& leftx, vector<int>& lefty, vector<int>& rightx, vector<int>& righty) {
    // Create histogram
    Mat bottom_half = binary_warped(Rect(0, binary_warped.rows / 2, binary_warped.cols, binary_warped.rows / 2));
    Mat histogram;
    reduce(bottom_half, histogram, 0, REDUCE_SUM, CV_32S);
    
    int midpoint = histogram.cols / 2;
    Point leftx_base_point, rightx_base_point;
    minMaxLoc(histogram(Rect(0, 0, midpoint, 1)), nullptr, nullptr, nullptr, &leftx_base_point);
    minMaxLoc(histogram(Rect(midpoint, 0, histogram.cols - midpoint, 1)), nullptr, nullptr, nullptr, &rightx_base_point);
    
    int leftx_base = leftx_base_point.x;
    int rightx_base = rightx_base_point.x + midpoint;
    
    int nwindows = 7;
    int margin = 100;
    int minpix = 50;
    int window_height = (binary_warped.rows / 2) / nwindows;
    
    vector<Point> nonzero_points;
    findNonZero(binary_warped, nonzero_points);
    
    vector<int> nonzeroy, nonzerox;
    for (const Point& p : nonzero_points) {
        nonzeroy.push_back(p.y);
        nonzerox.push_back(p.x);
    }
    
    int leftx_current = leftx_base;
    int rightx_current = rightx_base;
    
    vector<int> left_lane_inds, right_lane_inds;
    
    for (int window = 0; window < nwindows; window++) {
        int win_y_low = binary_warped.rows - (window + 1) * window_height;
        int win_y_high = binary_warped.rows - window * window_height;
        int win_xleft_low = leftx_current - margin;
        int win_xleft_high = leftx_current + margin;
        int win_xright_low = rightx_current - margin;
        int win_xright_high = rightx_current + margin;
        
        vector<int> good_left_inds, good_right_inds;
        
        for (size_t i = 0; i < nonzeroy.size(); i++) {
            if (nonzeroy[i] >= win_y_low && nonzeroy[i] < win_y_high &&
                nonzerox[i] >= win_xleft_low && nonzerox[i] < win_xleft_high) {
                good_left_inds.push_back(i);
            }
            if (nonzeroy[i] >= win_y_low && nonzeroy[i] < win_y_high &&
                nonzerox[i] >= win_xright_low && nonzerox[i] < win_xright_high) {
                good_right_inds.push_back(i);
            }
        }
        
        left_lane_inds.insert(left_lane_inds.end(), good_left_inds.begin(), good_left_inds.end());
        right_lane_inds.insert(right_lane_inds.end(), good_right_inds.begin(), good_right_inds.end());
        
        if (good_left_inds.size() > minpix) {
            double sum = 0;
            for (int idx : good_left_inds) {
                sum += nonzerox[idx];
            }
            leftx_current = static_cast<int>(sum / good_left_inds.size());
        }
        
        if (good_right_inds.size() > minpix) {
            double sum = 0;
            for (int idx : good_right_inds) {
                sum += nonzerox[idx];
            }
            rightx_current = static_cast<int>(sum / good_right_inds.size());
        }
    }
    
    leftx.clear(); lefty.clear(); rightx.clear(); righty.clear();
    
    for (int idx : left_lane_inds) {
        leftx.push_back(nonzerox[idx]);
        lefty.push_back(nonzeroy[idx]);
    }
    
    for (int idx : right_lane_inds) {
        rightx.push_back(nonzerox[idx]);
        righty.push_back(nonzeroy[idx]);
    }
}

void fit_poly(const Mat& binary_warped, const vector<int>& leftx, const vector<int>& lefty, 
              const vector<int>& rightx, const vector<int>& righty,
              vector<double>& left_fit, vector<double>& right_fit,
              vector<double>& left_fitx, vector<double>& right_fitx, vector<double>& ploty) {
    
    // Convert to double vectors for polyfit
    vector<double> lefty_d(lefty.begin(), lefty.end());
    vector<double> leftx_d(leftx.begin(), leftx.end());
    vector<double> righty_d(righty.begin(), righty.end());
    vector<double> rightx_d(rightx.begin(), rightx.end());
    
    // Simple polynomial fitting (2nd degree)
    if (lefty_d.size() >= 3) {
        // Create matrices for least squares fitting
        Mat A_left(lefty_d.size(), 3, CV_64F);
        Mat b_left(lefty_d.size(), 1, CV_64F);
        
        for (size_t i = 0; i < lefty_d.size(); i++) {
            A_left.at<double>(i, 0) = lefty_d[i] * lefty_d[i];
            A_left.at<double>(i, 1) = lefty_d[i];
            A_left.at<double>(i, 2) = 1.0;
            b_left.at<double>(i, 0) = leftx_d[i];
        }
        
        Mat left_coeffs;
        solve(A_left, b_left, left_coeffs, DECOMP_SVD);
        left_fit = {left_coeffs.at<double>(0, 0), left_coeffs.at<double>(1, 0), left_coeffs.at<double>(2, 0)};
    } else {
        left_fit = {1.0, 1.0, 0.0};
    }
    
    if (righty_d.size() >= 3) {
        Mat A_right(righty_d.size(), 3, CV_64F);
        Mat b_right(righty_d.size(), 1, CV_64F);
        
        for (size_t i = 0; i < righty_d.size(); i++) {
            A_right.at<double>(i, 0) = righty_d[i] * righty_d[i];
            A_right.at<double>(i, 1) = righty_d[i];
            A_right.at<double>(i, 2) = 1.0;
            b_right.at<double>(i, 0) = rightx_d[i];
        }
        
        Mat right_coeffs;
        solve(A_right, b_right, right_coeffs, DECOMP_SVD);
        right_fit = {right_coeffs.at<double>(0, 0), right_coeffs.at<double>(1, 0), right_coeffs.at<double>(2, 0)};
    } else {
        right_fit = {1.0, 1.0, binary_warped.cols};
    }
    
    // Generate plot points
    ploty.clear();
    for (int i = 0; i < binary_warped.rows; i++) {
        ploty.push_back(i);
    }
    
    left_fitx.clear();
    right_fitx.clear();
    
    try {
        for (double y : ploty) {
            left_fitx.push_back(left_fit[0] * y * y + left_fit[1] * y + left_fit[2]);
            right_fitx.push_back(right_fit[0] * y * y + right_fit[1] * y + right_fit[2]);
        }
    } catch (...) {
        cout << "The function failed to fit a line!" << endl;
        left_fitx.clear();
        right_fitx.clear();
        for (double y : ploty) {
            left_fitx.push_back(1 * y * y + 1 * y);
            right_fitx.push_back(1 * y * y + 1 * y);
        }
    }
}

pair<double, double> measure_curvature_meters(const vector<double>& ploty, const vector<double>& left_fitx, const vector<double>& right_fitx) {
    // Convert pixel coordinates to real world coordinates
    vector<double> ploty_m, left_fitx_m, right_fitx_m;
    
    for (size_t i = 0; i < ploty.size(); i++) {
        ploty_m.push_back(ploty[i] * YM_PER_PIX);
        left_fitx_m.push_back(left_fitx[i] * XM_PER_PIX);
        right_fitx_m.push_back(right_fitx[i] * XM_PER_PIX);
    }
    
    // Fit polynomial in world coordinates
    vector<double> left_fit_cr(3), right_fit_cr(3);
    
    if (ploty_m.size() >= 3) {
        Mat A_left(ploty_m.size(), 3, CV_64F);
        Mat b_left(ploty_m.size(), 1, CV_64F);
        
        for (size_t i = 0; i < ploty_m.size(); i++) {
            A_left.at<double>(i, 0) = ploty_m[i] * ploty_m[i];
            A_left.at<double>(i, 1) = ploty_m[i];
            A_left.at<double>(i, 2) = 1.0;
            b_left.at<double>(i, 0) = left_fitx_m[i];
        }
        
        Mat left_coeffs;
        solve(A_left, b_left, left_coeffs, DECOMP_SVD);
        left_fit_cr = {left_coeffs.at<double>(0, 0), left_coeffs.at<double>(1, 0), left_coeffs.at<double>(2, 0)};
        
        Mat A_right(ploty_m.size(), 3, CV_64F);
        Mat b_right(ploty_m.size(), 1, CV_64F);
        
        for (size_t i = 0; i < ploty_m.size(); i++) {
            A_right.at<double>(i, 0) = ploty_m[i] * ploty_m[i];
            A_right.at<double>(i, 1) = ploty_m[i];
            A_right.at<double>(i, 2) = 1.0;
            b_right.at<double>(i, 0) = right_fitx_m[i];
        }
        
        Mat right_coeffs;
        solve(A_right, b_right, right_coeffs, DECOMP_SVD);
        right_fit_cr = {right_coeffs.at<double>(0, 0), right_coeffs.at<double>(1, 0), right_coeffs.at<double>(2, 0)};
    }
    
    double y_eval = *max_element(ploty.begin(), ploty.end());
    double left_c = pow(1 + pow(2 * left_fit_cr[0] * y_eval * YM_PER_PIX + left_fit_cr[1], 2), 1.5) / abs(2 * left_fit_cr[0]);
    double right_c = pow(1 + pow(2 * right_fit_cr[0] * y_eval * YM_PER_PIX + right_fit_cr[1], 2), 1.5) / abs(2 * right_fit_cr[0]);
    
    return make_pair(left_c, right_c);
}

double measure_position_meters(const vector<double>& left_fit, const vector<double>& right_fit) {
    double y_max = IMG_Hs;
    double left_x = left_fit[0] * y_max * y_max + left_fit[1] * y_max + left_fit[2];
    double right_x = right_fit[0] * y_max * y_max + right_fit[1] * y_max + right_fit[2];
    double lane_mid = (left_x + right_x) / 2;
    double veh_pos = ((IMG_Ws / 2.0) - lane_mid) * XM_PER_PIX;
    return veh_pos - 0.10;
}

pair<Mat, double> project_lane_info(const Mat& img, const Mat& binary_warped, const vector<double>& ploty,
                                   const vector<double>& left_fitx, const vector<double>& right_fitx,
                                   const Mat& M_inv, double left_curverad, double right_curverad, double veh_pos) {
    Mat warp_zero = Mat::zeros(binary_warped.size(), CV_8UC1);
    Mat color_warp;
    cvtColor(warp_zero, color_warp, COLOR_GRAY2BGR);
    
    vector<Point> pts_left, pts_right;
    for (size_t i = 0; i < ploty.size(); i++) {
        pts_left.push_back(Point(static_cast<int>(left_fitx[i]), static_cast<int>(ploty[i])));
    }
    
    for (int i = ploty.size() - 1; i >= 0; i--) {
        pts_right.push_back(Point(static_cast<int>(right_fitx[i]), static_cast<int>(ploty[i])));
    }
    
    vector<Point> pts;
    pts.insert(pts.end(), pts_left.begin(), pts_left.end());
    pts.insert(pts.end(), pts_right.begin(), pts_right.end());
    
    vector<vector<Point>> contours = {pts};
    fillPoly(color_warp, contours, Scalar(0, 255, 0));
    
    Mat newwarp;
    warpPerspective(color_warp, newwarp, M_inv, Size(img.cols, img.rows));
    
    Mat out_img;
    addWeighted(img, 1, newwarp, 0.3, 0, out_img);
    
    double avg_curverad = (left_curverad + right_curverad) / 2;
    
    putText(out_img, "Curve Radius [m]: " + to_string(avg_curverad).substr(0, 7),
            Point(40, 70), FONT_HERSHEY_COMPLEX_SMALL, 1.6, Scalar(255, 0, 0), 2, LINE_AA);
    putText(out_img, "Center Offset [m]: " + to_string(veh_pos).substr(0, 7),
            Point(40, 150), FONT_HERSHEY_COMPLEX_SMALL, 1.6, Scalar(255, 0, 0), 2, LINE_AA);
    
    return make_pair(out_img, avg_curverad);
}

Mat resize_for_screen(const Mat& img) {
    int h = img.rows;
    int w = img.cols;
    double scale = min(static_cast<double>(SCR_W) / w, static_cast<double>(SCR_H) / h);
    Mat resized;
    resize(img, resized, Size(static_cast<int>(w * scale), static_cast<int>(h * scale)));
    return resized;
}

pair<Mat, double> binary_thresholder(const Mat& img) {
    auto start = chrono::high_resolution_clock::now();
    
    // TODO: Filtro HSV
    Mat hsv;
    cvtColor(img, hsv, COLOR_RGB2HSV);
    
    Mat roi = hsv(Rect(0, 480/2, hsv.cols, hsv.rows - 480/2));
    
    // Threshold adaptativo HSV
    vector<Mat> hsv_channels;
    split(roi, hsv_channels);
    Mat adapt_white_hsv;
    adaptiveThreshold(hsv_channels[2], adapt_white_hsv, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 161, -27);
    
    // TODO: Fim Filtro HSV
    
    // Aplicado erosao na imagem gerada pelo HSV Adaptativo, eliminar pequenos ruIdos
    Mat kernel_erode = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(adapt_white_hsv, adapt_white_hsv, kernel_erode, Point(-1, -1), 1);
    
    Mat final = adapt_white_hsv;
    
    // Aplicando filtro de mediana na imagem combinada
    int median_kernel_size = 7;  // Deve ser um numero Impar, como 3, 5, 7
    medianBlur(final, final, median_kernel_size);
    
    // cv2.imshow('final filtered', final)
    
    // Aplicando processamento morfologico de dilatacao para realcar as areas brancas
    Mat kernel_dilate = getStructuringElement(MORPH_RECT, Size(6, 6));  // Kernel menor para erosao
    Mat img_dilate;
    dilate(final, img_dilate, kernel_dilate, Point(-1, -1), 1);
    
    // cv2.imshow('Adaptativo', adapt_white_hsv)
    
    auto end = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(end - start);
    double tempo = duration.count() / 1000000.0;
    
    return make_pair(img_dilate, tempo);
}

struct Resolution {
    int width = 1280;
    int height = 720;
};

int main() {
    // Open the ZED camera
    // VideoCapture cap(0);
    // if (!cap.isOpened()) {
    //     return -1;
    // }

    Resolution image_size;
    image_size.width = 1280;
    image_size.height = 720;

    // Set the video resolution to HD720
    // cap.set(CAP_PROP_FRAME_WIDTH, image_size.width * 2);
    // cap.set(CAP_PROP_FRAME_HEIGHT, image_size.height);
    // int actual_width = static_cast<int>(cap.get(CAP_PROP_FRAME_WIDTH));
    // int actual_height = static_cast<int>(cap.get(CAP_PROP_FRAME_HEIGHT));
    // cout << "Actual resolution: " << actual_width << "x" << actual_height << endl;
    
    // AlmostCan* almost_can = inicializar_serial();
    // if (!almost_can) {
    //     return -1;
    // }
    
    int target_width = 1280;
    int target_height = 720;
    int target_widthsmall = 640;
    int target_heightsmall = 480;
    int roi_start_row = target_heightsmall / 2;
    Mat top_padding = Mat::zeros(roi_start_row, target_widthsmall, CV_8UC1);
    
    try {
        while (true) {
            try {
                auto inicio = chrono::high_resolution_clock::now();
                
                // error_code = camera.grab(runtime_params)
                // camera.retrieve_image(image, sl.VIEW.LEFT)
                // retval, images = cap.read()
                // vector<Mat> images(2);
                // Mat full_frame;
                // cap >> full_frame;
                // images[0] = full_frame(Rect(0, 0, full_frame.cols/2, full_frame.rows));
                // images[1] = full_frame(Rect(full_frame.cols/2, 0, full_frame.cols/2, full_frame.rows));
                // Mat frame = images[0];
                
                Mat frame = imread("imagens_17_31_0063.png");
                // resize(frame, frame, Size(target_width, target_height));

                // imshow("left RAW", frame);
                // waitKey(1);

                resize(frame, frame, Size(target_widthsmall, target_heightsmall));
                
                auto warp_result = warp(frame);
                Mat warped_frame = warp_result.first;
                Mat perspective_transform = warp_result.second;
                
                // Mat resized_warped_frame = resize_for_screen(warped_frame);
                // imshow("Warped Image", resized_warped_frame);
                // waitKey(1);

                auto bin_result = binary_thresholder(warped_frame);
                Mat img_bin = bin_result.first;
                double tempo = bin_result.second;
                
                cout << "tempo de execucao bin: " << fixed << setprecision(3) << tempo << " seg" << endl;
                
                Mat img_bin_full;
                vconcat(top_padding, img_bin, img_bin_full);

                imshow("Binary Image", img_bin_full);
                waitKey(1);
                
                vector<int> leftx, lefty, rightx, righty;
                find_lane_pixels_using_histogram(img_bin_full, leftx, lefty, rightx, righty);
                
                vector<double> left_fit, right_fit, left_fitx, right_fitx, ploty;
                fit_poly(img_bin_full, leftx, lefty, rightx, righty, left_fit, right_fit, left_fitx, right_fitx, ploty);

                auto curvature = measure_curvature_meters(ploty, left_fitx, right_fitx);
                double left_curverad = curvature.first;
                double right_curverad = curvature.second;

                double vehicle_position = measure_position_meters(left_fit, right_fit);

                int steering_angle = calcular_angulo_do_volante(vehicle_position);
                int steering_angle_int = steering_angle;
                
                auto lane_result = project_lane_info(
                    frame, img_bin_full, ploty, left_fitx, right_fitx,
                    perspective_transform, left_curverad, right_curverad, vehicle_position
                );
                Mat lane_overlay = lane_result.first;
                double avg_curvature = lane_result.second;
                
                Mat resized_lane_overlay = resize_for_screen(lane_overlay);
                imshow("Lane Detection Result", resized_lane_overlay);
            
                waitKey(1);
                
                auto fim = chrono::high_resolution_clock::now();
                auto duration = chrono::duration_cast<chrono::microseconds>(fim - inicio);
                double tempo_total = duration.count() / 1000000.0;
                
                cout << "tempo de execucao: " << fixed << setprecision(3) << tempo_total << " seg" << endl;

                // enviar_dado(*almost_can, steering_angle_int, 0x00);

                if (waitKey(1) & 0xFF == 'q') {
                    break;
                }
            } catch (const exception& ex) {
                cout << "Erro: " << ex.what() << endl;
                continue;
            }
        }
    } catch (...) {
        cout << "Execução interrompida pelo usuário." << endl;
    }
    
    destroyAllWindows();
    // camera.close();
    // if (almost_can) {
    //     almost_can->close();
    //     delete almost_can;
    // }
    cout << "Recursos liberados. Execução finalizada." << endl;
    
    return 0;
}

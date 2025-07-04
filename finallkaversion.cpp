// Converted C++ version of show_lka_version.py with full visual overlay
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace std;
using namespace cv;

const int IMG_W = 1280, IMG_H = 720;
const int IMG_Ws = 640, IMG_Hs = 480;
const int SCR_W = 800, SCR_H = 600;
const double YM_PER_PIX = 2.40 / 480.0;
const double XM_PER_PIX = 1.00 / 90.0;

float map_in_range(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int calcular_angulo_do_volante(float distancia_em_metros) {
    return static_cast<int>(map_in_range(distancia_em_metros, -1, 1, 50, 0));
}

void send_can_message(int direcao) {
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) return;
    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s, (struct sockaddr *)&addr, sizeof(addr));
    frame.can_id = 0x101;
    frame.can_dlc = 8;
    memset(frame.data, 0, 8);
    frame.data[6] = (50 - direcao) & 0xFF;
    write(s, &frame, sizeof(struct can_frame));
    close(s);
}

Mat warp(const Mat& img, Mat& M_inv) {
    Point2f src[] = {{0, IMG_Hs}, {IMG_Ws, IMG_Hs}, {100, 287}, {IMG_W - 100, 287}};
    Point2f dst[] = {{0, IMG_Hs}, {IMG_Ws, IMG_Hs}, {0, 0}, {IMG_W, 0}};
    Mat M = getPerspectiveTransform(src, dst);
    M_inv = getPerspectiveTransform(dst, src);
    Mat warped;
    warpPerspective(img, warped, M, Size(IMG_Ws, IMG_Hs));
    return warped;
}

Mat binary_thresholder(const Mat& img) {
    Mat hsv;
    cvtColor(img, hsv, COLOR_RGB2HSV);
    Mat roi = hsv(Rect(0, IMG_Hs / 2, hsv.cols, hsv.rows - IMG_Hs / 2));
    vector<Mat> hsv_channels;
    split(roi, hsv_channels);
    Mat v_channel = hsv_channels[2];
    Mat adapt_white_hsv;
    adaptiveThreshold(v_channel, adapt_white_hsv, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 161, -45);
    Mat kernel_erode = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(adapt_white_hsv, adapt_white_hsv, kernel_erode);
    medianBlur(adapt_white_hsv, adapt_white_hsv, 7);
    Mat kernel_dilate = getStructuringElement(MORPH_RECT, Size(6, 6));
    dilate(adapt_white_hsv, adapt_white_hsv, kernel_dilate);
    return adapt_white_hsv;
}

Mat resize_for_screen(const Mat& img) {
    int h = img.rows, w = img.cols;
    double scale = min(static_cast<double>(SCR_W) / w, static_cast<double>(SCR_H) / h);
    Mat resized;
    resize(img, resized, Size(int(w * scale), int(h * scale)));
    return resized;
}

Vec3d polyfit(const vector<double>& x, const vector<double>& y) {
    int N = x.size();
    double X[5] = {0};
    for (int i = 0; i < N; i++) {
        X[0] += 1;
        X[1] += x[i];
        X[2] += x[i] * x[i];
        X[3] += x[i] * x[i] * x[i];
        X[4] += x[i] * x[i] * x[i] * x[i];
    }
    double Y[3] = {0};
    for (int i = 0; i < N; i++) {
        Y[0] += y[i];
        Y[1] += x[i] * y[i];
        Y[2] += x[i] * x[i] * y[i];
    }
    Mat A = (Mat_<double>(3, 3) << X[0], X[1], X[2], X[1], X[2], X[3], X[2], X[3], X[4]);
    Mat B = (Mat_<double>(3, 1) << Y[0], Y[1], Y[2]);
    Mat coeffs;
    solve(A, B, coeffs, DECOMP_SVD);
    return Vec3d(coeffs.at<double>(2), coeffs.at<double>(1), coeffs.at<double>(0));
}

double evaluate_poly(const Vec3d& coef, double y) {
    return coef[0] * y * y + coef[1] * y + coef[2];
}

Mat draw_lane_overlay(const Mat& base_img, const Mat& binary_warped, const Vec3d& left_fit, const Vec3d& right_fit, const Mat& Minv, double vehicle_position) {
    Mat warp_zero = Mat::zeros(binary_warped.size(), CV_8UC1);
    Mat color_warp;
    cvtColor(warp_zero, color_warp, COLOR_GRAY2BGR);

    vector<Point> left_pts, right_pts;
    for (int y = 0; y < binary_warped.rows; y++) {
        int left_x = evaluate_poly(left_fit, y);
        int right_x = evaluate_poly(right_fit, y);
        left_pts.emplace_back(left_x, y);
        right_pts.emplace_back(right_x, y);
    }

    vector<Point> pts;
    pts.insert(pts.end(), left_pts.begin(), left_pts.end());
    pts.insert(pts.end(), right_pts.rbegin(), right_pts.rend());
    vector<vector<Point>> poly_pts = { pts };
    fillPoly(color_warp, poly_pts, Scalar(0, 255, 0));

    Mat newwarp;
    warpPerspective(color_warp, newwarp, Minv, base_img.size());
    Mat result;
    addWeighted(base_img, 1, newwarp, 0.3, 0, result);

    putText(result, format("Center Offset [m]: %.2f", vehicle_position), Point(40, 150), FONT_HERSHEY_COMPLEX_SMALL, 1.6, Scalar(255, 0, 0), 2);
    return result;
}

using namespace cv;
using namespace std;

void find_lane_pixels_using_histogram(const Mat& binary_warped,
                                      vector<double>& leftx, vector<double>& lefty,
                                      vector<double>& rightx, vector<double>& righty)
{
    int height = binary_warped.rows;
    int width = binary_warped.cols;

    // Sum lower half of image along columns
    Mat lower_half = binary_warped(Range(height / 2, height), Range::all());
    Mat histogram;
    reduce(lower_half, histogram, 0, REDUCE_SUM, CV_32S);  // 1D row, width elements

    int midpoint = width / 2;

    // Find base points
    Point left_max_loc, right_max_loc;
    minMaxLoc(histogram.colRange(0, midpoint), nullptr, nullptr, nullptr, &left_max_loc);
    minMaxLoc(histogram.colRange(midpoint, width), nullptr, nullptr, nullptr, &right_max_loc);
    int leftx_base = left_max_loc.x;
    int rightx_base = right_max_loc.x + midpoint;

    // Hyperparameters
    int nwindows = 7;
    int margin = 100;
    int minpix = 50;
    int window_height = (height / 2) / nwindows;

    // Find all nonzero points
    vector<Point> nonzero;
    findNonZero(binary_warped, nonzero);

    vector<int> nonzerox, nonzeroy;
    for (auto& pt : nonzero) {
        nonzerox.push_back(pt.x);
        nonzeroy.push_back(pt.y);
    }

    int leftx_current = leftx_base;
    int rightx_current = rightx_base;

    vector<int> left_lane_inds, right_lane_inds;

    for (int window = 0; window < nwindows; ++window) {
        int win_y_low = height - (window + 1) * window_height;
        int win_y_high = height - window * window_height;

        int win_xleft_low = leftx_current - margin;
        int win_xleft_high = leftx_current + margin;
        int win_xright_low = rightx_current - margin;
        int win_xright_high = rightx_current + margin;

        vector<int> good_left_inds, good_right_inds;

        for (int i = 0; i < nonzeroy.size(); ++i) {
            if (nonzeroy[i] >= win_y_low && nonzeroy[i] < win_y_high) {
                if (nonzerox[i] >= win_xleft_low && nonzerox[i] < win_xleft_high)
                    good_left_inds.push_back(i);
                if (nonzerox[i] >= win_xright_low && nonzerox[i] < win_xright_high)
                    good_right_inds.push_back(i);
            }
        }

        left_lane_inds.insert(left_lane_inds.end(), good_left_inds.begin(), good_left_inds.end());
        right_lane_inds.insert(right_lane_inds.end(), good_right_inds.begin(), good_right_inds.end());

        if (good_left_inds.size() > minpix) {
            int sum = 0;
            for (int idx : good_left_inds) sum += nonzerox[idx];
            leftx_current = sum / good_left_inds.size();
        }

        if (good_right_inds.size() > minpix) {
            int sum = 0;
            for (int idx : good_right_inds) sum += nonzerox[idx];
            rightx_current = sum / good_right_inds.size();
        }
    }

    for (int idx : left_lane_inds) {
        leftx.push_back(nonzerox[idx]);
        lefty.push_back(nonzeroy[idx]);
    }

    for (int idx : right_lane_inds) {
        rightx.push_back(nonzerox[idx]);
        righty.push_back(nonzeroy[idx]);
    }
}

int main() {
	VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "Camera not found" << endl;
        return -1;
    }
    
    
    Mat top_padding = Mat::zeros(IMG_Hs / 2, IMG_Ws, CV_8UC1);

    while (true) {
        auto start = chrono::high_resolution_clock::now();
        Mat full_frame;
		cap >> full_frame;

		if (full_frame.empty()) {
			cerr << "Empty frame!" << endl;
			return -1;
		}

		static int width = full_frame.cols;
		static int height = full_frame.rows;

		// Split image in half (axis=1 in Python)
		Rect left_roi(0, 0, width / 2, height);
		Rect right_roi(width / 2, 0, width / 2, height);

		Mat frame = full_frame(left_roi);
		// Mat right_image = full_frame(right_roi); // if needed
        resize(frame, frame, Size(IMG_Ws, IMG_Hs));
        if (frame.empty()) continue;
        Mat M_inv;
        Mat warped_frame = warp(frame, M_inv);
        //imshow("Warped_frame", warped_frame);
        //waitKey(1);
        Mat img_bin = binary_thresholder(warped_frame);
        vconcat(top_padding, img_bin, img_bin);
        //imshow("Binary Image", img_bin);
        //waitKey(1);

        vector<Point> nonzero;
        findNonZero(img_bin, nonzero);
        vector<double> leftx, lefty, rightx, righty;
        //for (auto& pt : nonzero) {
        //    if (pt.x < IMG_Ws / 2) leftx.push_back(pt.y), lefty.push_back(pt.x);
        //    else rightx.push_back(pt.y), righty.push_back(pt.x);
        //}
		find_lane_pixels_using_histogram(img_bin, leftx, lefty, rightx, righty);
		
        if (leftx.size() < 3 || rightx.size() < 3) continue;
        
        //int max_print = 10;

		//cout << "leftx (first " << max_print << " values): ";
		//for (size_t i = 0; i < min(leftx.size(), size_t(max_print)); ++i) {
//			cout << leftx[i] << " ";
		//}
		//cout << endl;
//
	//	cout << "rightx (first " << max_print << " values): ";
		//for (size_t i = 0; i < min(rightx.size(), size_t(max_print)); ++i) {
//			cout << rightx[i] << " ";
		//}
		//cout << endl;

        Vec3d left_fit = polyfit(lefty, leftx);
        Vec3d right_fit = polyfit(righty, rightx);
        
        //cout << fixed << setprecision(6);
		//cout << "left_fit: A = " << left_fit[0]
		//	 << ", B = " << left_fit[1]
		//	 << ", C = " << left_fit[2] << endl;

        double y_max = IMG_Hs;
        double left_x = evaluate_poly(left_fit, y_max);
        double right_x = evaluate_poly(right_fit, y_max);
        double lane_mid = (left_x + right_x) / 2.0;
        double veh_pos = ((((IMG_Ws / 2.0) - lane_mid) * XM_PER_PIX) + 0.10);

        int steering_angle = calcular_angulo_do_volante(veh_pos);
        send_can_message(steering_angle);

        auto end = chrono::high_resolution_clock::now();
        chrono::duration<double> elapsed = end - start;
        cout << "Execution time: " << elapsed.count() << " s" << endl;

        if (waitKey(1) == 'q') break;
    }

    destroyAllWindows();
    return 0;
}

//g++ -std=c++17 -O3 -march=native -mtune=native -flto -o ExeFlka_version finallkaversion.cpp `pkg-config --cflags --libs opencv4`

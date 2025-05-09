import cv2
import numpy as np
import pyzed.sl as sl
from ultralytics import YOLO
import serial
import time
import sys
import glob
import torch


# ---------- MACROS / CONSTANTES ------------------------------------------------
IMG_W, IMG_H = 1280, 720           # resolução de trabalho (warp, YOLO, máscaras)
SCR_W, SCR_H = 800, 600            # tamanho das janelas do cv2.imshow

# conversões pista-real (ajuste conforme a largura real da faixa):
YM_PER_PIX = 2.40 / 720            # m por pixel no eixo Y      (≈ altura)
XM_PER_PIX = 1.00 / 700            # m por pixel no eixo X      (≈ largura da pista)

CONF_THRESHOLD = 0.30              # limiar de confiança p/ YOLO
# ------------------------------------------------------------------------------



posix_ponto_1_x = 0
posix_ponto_1_y = 0

posix_ponto_2_x = 0
posix_ponto_2_y = 0

posix_ponto_3_x = 0
posix_ponto_3_y = 0

posix_ponto_4_x = 0
posix_ponto_4_y = 0


class AlmostCan:
    SUCCESS = 0
    ERROR_TIMEOUT = 1
    ERROR_INVALID_SOF = 2
    ERROR_INVALID_EOF = 3
    ERROR_CRC_MISMATCH = 4
    ERROR_INVALID_LENGTH = 5

    def __init__(self, serial_port, baud_rate, start_byte=0xFF, end_byte=0xF7, timeout=200):
        self.serial = serial.Serial(serial_port, baud_rate, timeout=timeout / 1000.0)
        self.baud = baud_rate
        self.start_byte = start_byte
        self.end_byte = end_byte
        self.timeout = timeout

    def begin(self):
        if not self.serial.is_open:
            self.serial.open()

    def send_data(self, id, data):
        length = len(data)
        packet = bytearray([self.start_byte, id, length] + data)

        crc = self.calculate_crc(packet[1:3 + length])
        packet += bytearray([crc & 0xFF, (crc >> 8) & 0xFF, self.end_byte])

        self.serial.write(packet)

    def receive_data(self):
        if not self.wait_for_data(1):
            return self.ERROR_TIMEOUT, None, None

        if self.serial.read() != bytes([self.start_byte]):
            return self.ERROR_INVALID_SOF, None, None

        if not self.wait_for_data(2):
            return self.ERROR_TIMEOUT, None, None

        id = self.serial.read()[0]
        length = self.serial.read()[0]

        if length > 32:
            return self.ERROR_INVALID_LENGTH, None, None

        data = bytearray()
        if length > 0:
            if not self.wait_for_data(length):
                return self.ERROR_TIMEOUT, None, None

            data = self.serial.read(length)

        if not self.wait_for_data(3):
            return self.ERROR_TIMEOUT, None, None

        crc_low = self.serial.read()[0]
        crc_high = self.serial.read()[0]
        eof = self.serial.read()[0]

        if eof != self.end_byte:
            return self.ERROR_INVALID_EOF, None, None

        received_crc = (crc_high << 8) | crc_low
        calculated_crc = self.calculate_crc(bytearray([id, length]) + data)

        if calculated_crc != received_crc:
            return self.ERROR_CRC_MISMATCH, None, None

        return self.SUCCESS, id, data

    def set_start_byte(self, start_byte):
        self.start_byte = start_byte

    def set_end_byte(self, end_byte):
        self.end_byte = end_byte

    def set_timeout(self, timeout):
        self.timeout = timeout
        self.serial.timeout = timeout / 1000.0

    def calculate_crc(self, data):
        crc = 0
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def wait_for_data(self, expected_bytes):
        start_time = time.time()
        while self.serial.in_waiting < expected_bytes:
            if time.time() - start_time >= self.timeout / 1000.0:
                return False
        return True
    
def listar_portas_seriais():
    if sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        portas_existentes = glob.glob('/dev/tty[A-Za-z]*')
        portas_disponiveis = []
        for porta in portas_existentes:
            try:
                s = serial.Serial(porta)
                s.close()
                portas_disponiveis.append(porta)
            except (OSError, serial.SerialException):
                pass
        return portas_disponiveis
    elif sys.platform.startswith('win'):
        return [f"COM{i}" for i in range(1, 256)]
    else:
        raise EnvironmentError('Plataforma não suportada')

def escolher_porta(portas):
    print("Portas disponíveis:")
    for i, porta in enumerate(portas, start=1):
        print(f"{i}: {porta}")
    escolha = input("Escolha uma porta pelo número: ")
    return portas[int(escolha) - 1]

def enviar_dado(almost_can, dado, id=0x00):
    try:
        data_bytes = list(dado.to_bytes(4, byteorder='big'))
        almost_can.send_data(id, data_bytes)
        print(f"Dado {dado} enviado com sucesso")
    except Exception as e:
        print(f"Erro ao enviar dado: {e}")
        
def inicializar_serial():
    portas_disponiveis = listar_portas_seriais()
    if portas_disponiveis:
        porta_escolhida = escolher_porta(portas_disponiveis)
        print(f"Porta escolhida: {porta_escolhida}")
        try:
            ac = AlmostCan(porta_escolhida, 115200, timeout=1000)
            ac.begin()
            time.sleep(2)
            return ac
        except (OSError, serial.SerialException) as e:
            print(f"Erro ao acessar a porta {porta_escolhida}: {e}")
            return None
    else:
        print("Nenhuma porta serial disponível.")
        return None
    
def map_in_range(x: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
    return float((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def calcular_angulo_do_volante(distancia_em_metros):    
    angulo = int(map_in_range(distancia_em_metros, -1, 1, 50, 0))
    return angulo

# def warp(img):
#     height = img.shape[0]
#     width = img.shape[1]
#     img_size = (1280, 720)
#     src = np.float32([[0, height], [width, height], [200, 430], [(width-200), 430]])
#     dst = np.float32([[0, height], [width, height], [0, 0], [width, 0]])
#     M = cv2.getPerspectiveTransform(src, dst)
#     M_inv = cv2.getPerspectiveTransform(dst, src)
#     warp = cv2.warpPerspective(img, M, img_size)
#     return warp, M_inv

def warp(img):
    src = np.float32([[0, IMG_H], [IMG_W, IMG_H], [200, 430], [IMG_W-200, 430]])
    dst = np.float32([[0, IMG_H], [IMG_W, IMG_H], [0, 0], [IMG_W, 0]])
    M     = cv2.getPerspectiveTransform(src, dst)
    M_inv = cv2.getPerspectiveTransform(dst, src)
    return cv2.warpPerspective(img, M, (IMG_W, IMG_H)), M_inv


def find_lane_pixels_using_histogram(binary_warped):
    histogram = np.sum(binary_warped[binary_warped.shape[0] // 2:, :], axis=0)
    midpoint = int(histogram.shape[0] // 2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint
    nwindows = 15
    margin = 100
    minpix = 50
    window_height = int(binary_warped.shape[0] // nwindows)
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    leftx_current = leftx_base
    rightx_current = rightx_base
    left_lane_inds = []
    right_lane_inds = []
    for window in range(nwindows):
        win_y_low = binary_warped.shape[0] - (window + 1) * window_height
        win_y_high = binary_warped.shape[0] - window * window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                          (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                           (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        if len(good_left_inds) > minpix:
            leftx_current = int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = int(np.mean(nonzerox[good_right_inds]))
    try:
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
    except ValueError:
        pass
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]
    return leftx, lefty, rightx, righty

def fit_poly(binary_warped, leftx, lefty, rightx, righty):
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)
    ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
    try:
        left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
    except TypeError:
        print('The function failed to fit a line!')
        left_fitx = 1 * ploty ** 2 + 1 * ploty
        right_fitx = 1 * ploty ** 2 + 1 * ploty
    return left_fit, right_fit, left_fitx, right_fitx, ploty

# def measure_curvature_meters(binary_warped, left_fitx, right_fitx, ploty):
#     ym_per_pix = 2.40 / 720 #
#     xm_per_pix = 1.00 / 700
#     left_fit_cr = np.polyfit(ploty * ym_per_pix, left_fitx * xm_per_pix, 2)
#     right_fit_cr = np.polyfit(ploty * ym_per_pix, right_fitx * xm_per_pix, 2)
#     y_eval = np.max(ploty)
#     left_curverad = ((1 + (2 * left_fit_cr[0] * y_eval * ym_per_pix + left_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
#         2 * left_fit_cr[0])
#     right_curverad = ((1 + (2 * right_fit_cr[0] * y_eval * ym_per_pix + right_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
#         2 * right_fit_cr[0])
#     return left_curverad, right_curverad

def measure_curvature_meters(ploty, left_fitx, right_fitx):
    left_fit_cr  = np.polyfit(ploty * YM_PER_PIX, left_fitx * XM_PER_PIX, 2)
    right_fit_cr = np.polyfit(ploty * YM_PER_PIX, right_fitx * XM_PER_PIX, 2)
    y_eval = np.max(ploty)
    left_c  = ((1 + (2*left_fit_cr[0]*y_eval*YM_PER_PIX+left_fit_cr[1])**2)**1.5) / abs(2*left_fit_cr[0])
    right_c = ((1 + (2*right_fit_cr[0]*y_eval*YM_PER_PIX+right_fit_cr[1])**2)**1.5) / abs(2*right_fit_cr[0])
    return left_c, right_c


# def measure_position_meters(binary_warped, left_fit, right_fit):
#     xm_per_pix = 1.00 / 700 
#     y_max = binary_warped.shape[0]
#     left_x_pos = left_fit[0] * y_max ** 2 + left_fit[1] * y_max + left_fit[2]
#     right_x_pos = right_fit[0] * y_max ** 2 + right_fit[1] * y_max + right_fit[2]
#     center_lanes_x_pos = (left_x_pos + right_x_pos) // 2
#     veh_pos = ((binary_warped.shape[1] // 2) - center_lanes_x_pos) * xm_per_pix
#     return veh_pos - 0.10

def measure_position_meters(left_fit, right_fit):
    y_max  = IMG_H
    left_x  = left_fit[0]*y_max**2 + left_fit[1]*y_max + left_fit[2]
    right_x = right_fit[0]*y_max**2 + right_fit[1]*y_max + right_fit[2]
    lane_mid = (left_x + right_x) / 2
    veh_pos  = ((IMG_W/2) - lane_mid) * XM_PER_PIX
    return veh_pos - 0.10


def project_lane_info(img, binary_warped, ploty, left_fitx, right_fitx, M_inv, left_curverad, right_curverad, veh_pos):
    warp_zero = np.zeros_like(binary_warped).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))
    cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
    newwarp = cv2.warpPerspective(color_warp, M_inv, (img.shape[1], img.shape[0]))
    out_img = cv2.addWeighted(img, 1, newwarp, 0.3, 0)
    avg_curverad = (left_curverad + right_curverad) / 2
    cv2.putText(out_img, 'Curve Radius [m]: ' + str(avg_curverad)[:7], (40, 70),
                cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.6, (255, 0, 0), 2, cv2.LINE_AA)
    cv2.putText(out_img, 'Center Offset [m]: ' + str(veh_pos)[:7], (40, 150), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.6,
                (255, 0, 0), 2, cv2.LINE_AA)
    return out_img, avg_curverad

# def resize_image_to_screen(image):
#     screen_width, screen_height = (800, 600)
#     image_height, image_width = image.shape[:2]
    
#     scale_width = screen_width / image_width
#     scale_height = screen_height / image_height
#     scale = min(scale_width, scale_height)
    
#     new_width = int(image_width * scale)
#     new_height = int(image_height * scale)
#     resized_image = cv2.resize(image, (new_width, new_height))
#     return resized_image

def resize_for_screen(img):
    h, w = img.shape[:2]
    scale = min(SCR_W / w, SCR_H / h)
    return cv2.resize(img, (int(w*scale), int(h*scale)))


def main():
    camera = sl.Camera()
    init_params = sl.InitParameters()
    
    #Opening and configuring zed camera, not strictly necessary 
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA
    init_params.coordinate_units = sl.UNIT.METER
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP

    print("Abrindo a câmera ZED...")
    status = camera.open(init_params)

    
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"Erro ao abrir a câmera ZED: {repr(status)}")
        exit()
    
    #More camera configurations
    camera.set_camera_settings(sl.VIDEO_SETTINGS.GAIN, -1)
    camera.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, -1)
    camera.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_AUTO, 1)
    
    #Used to get the images and see if theres erros, not necessary
    image = sl.Mat()
    runtime_params = sl.RuntimeParameters()

    #Our device can only run in cpu
    #device = "cuda" if torch.cuda.is_available() else "cpu"
    device ="cpu"
    print(f"Dispositivo em uso: {device}")

    #Selecting ncnn model, you must specify you are predicting with it
    model = YOLO("./best_ncnn_model").to(device)

    image_width, image_height = IMG_W, IMG_H
    
    #This function is utilized to communicate with the ECU's via usb
    #almost_can = inicializar_serial()
    #if not almost_can:
    #    return
    
    try:
        while True:
            try:
                #error_code = camera.grab(runtime_params)

                #if error_code == sl.ERROR_CODE.SUCCESS:

                #Gets the image from the camera
                camera.retrieve_image(image, sl.VIEW.LEFT)
                frame = image.get_data()

                #First resizes the image for the size expected in functions, then makes the view bird eye
                warped_frame, perspective_transform = warp(frame)
                #Makes the image a good size to show on screen
                resized_warped_frame = resize_for_screen(warped_frame)
                cv2.imshow('Warped Image', resized_warped_frame)

                #Makes the image RGB from RGBA to utilize with YOLO
                frame_rgb = cv2.cvtColor(warped_frame, cv2.COLOR_RGBA2RGB)

                #Makes polygons around poossible lane markings
                results = model.predict(
                    source=frame_rgb,
                    imgsz=416,
                    device=device
                )

                #Makes a white mask that will be added with the found polygons
                combined_mask = np.zeros((image_height, image_width), dtype=np.uint8)

                #Gets the mask given from the model and compares the confidence of the found points with a treshold
                #Then makes an image of the combination of where the polygons are and the white mask
                for result in results:
                    masks = result.masks  
                    confidences = result.boxes.conf if result.boxes is not None else None
                    if masks is not None and masks.data is not None and confidences is not None:
                        for mask, conf in zip(masks.data, confidences):
                            if conf >= CONF_THRESHOLD:
                                mask = mask.cpu().numpy()
                                mask = (mask * 255).astype(np.uint8)
                                resized_mask = cv2.resize(mask, (image_width, image_height), interpolation=cv2.INTER_NEAREST)
                                combined_mask = cv2.add(combined_mask, resized_mask)

                cv2.imshow('Binary Image', combined_mask)
                
                #Utilizing an histogram the position of each lane is found by finding the two local maximuns
                leftx, lefty, rightx, righty = find_lane_pixels_using_histogram(combined_mask)
                
                #Fits a line through the found points from the histogram
                left_fit, right_fit, left_fitx, right_fitx, ploty = fit_poly(combined_mask, leftx, lefty, rightx, righty)
                # left_curverad, right_curverad = measure_curvature_meters(combined_mask, left_fitx, right_fitx, ploty)
                
                #Utilizing the found lines gets the curvature
                left_curverad, right_curverad = measure_curvature_meters(ploty, left_fitx, right_fitx)

                #Gets the vehicle offset
                vehicle_position = measure_position_meters(left_fit, right_fit)

                #Returns a value between 0-50 for the steering angle
                steering_angle = calcular_angulo_do_volante(vehicle_position)
                steering_angle_int = int(steering_angle)
                
                #Shows the lane detection result
                lane_overlay, avg_curvature = project_lane_info(
                    frame[:, :, 0:3], combined_mask, ploty, left_fitx, right_fitx,
                    perspective_transform, left_curverad, right_curverad, vehicle_position
                )
                resized_lane_overlay = resize_for_screen(lane_overlay)
                cv2.imshow('Lane Detection Result', resized_lane_overlay)
                
                cv2.waitKey(1)

                #Sends steering angle to CAN
                #enviar_dado(almost_can, steering_angle_int, 0x00)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except KeyboardInterrupt:
                print("Execução interrompida pelo usuário.")
                break
            except Exception as ex:
                print(f"Erro: {ex}")
                continue
    finally:
        cv2.destroyAllWindows()
        camera.close()
        print("Liberando memória da GPU...")
        torch.cuda.empty_cache()
        #almost_can.serial.close()
        import gc
        gc.collect()
        print("Recursos liberados. Execução finalizada.")

if __name__ == "__main__":
    main()

import cv2
import numpy as np
import serial
import time
import sys
import glob
import can


# ---------- MACROS / CONSTANTES ------------------------------------------------
IMG_W, IMG_H = 1280, 720           # resoluÃ§Ã£o de trabalho (warp, YOLO, mÃ¡scaras)
IMG_Ws, IMG_Hs = 640, 480          
SCR_W, SCR_H = 800, 600            # tamanho das janelas do cv2.imshow

# conversÃµes pista-real (ajuste conforme a largura real da faixa):
YM_PER_PIX = 2.40 / 480            # m por pixel no eixo Y      (? altura)
XM_PER_PIX = 1.00 / 90            # m por pixel no eixo X      (? largura da pista)

CONF_THRESHOLD = 0.30              # limiar de confianÃ§a p/ YOLO
# ------------------------------------------------------------------------------



posix_ponto_1_x = 0
posix_ponto_1_y = 0

posix_ponto_2_x = 0
posix_ponto_2_y = 0

posix_ponto_3_x = 0
posix_ponto_3_y = 0

posix_ponto_4_x = 0
posix_ponto_4_y = 0


def send_can_message(direcao):
    try:
        # Connect to CAN interface via MCP2515 (socketcan)
        bus = can.interface.Bus(channel='can0', bustype='socketcan')

        # Construct the data payload
        data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, (50 - int(direcao)) & 0xFF, 0x00]

        # Create CAN message (Standard ID 0x101)
        msg = can.Message(arbitration_id=0x101,
                          data=data,
                          is_extended_id=False)

        # Send the message
        bus.send(msg)
        #print(f"Message sent: ID=0x{msg.arbitration_id:X} Data={msg.data}")

    except can.CanError as e:
        print(f"CAN message failed to send: {e}")
    
def map_in_range(x: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
    return float((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def calcular_angulo_do_volante(distancia_em_metros):    
    angulo = int(map_in_range(distancia_em_metros, -1, 1, 50, 0))
    return angulo

def warp(img):
    src = np.float32([[0, IMG_Hs], [IMG_Ws, IMG_Hs], [100, 287], [IMG_W-100, 287]])
    dst = np.float32([[0, IMG_Hs], [IMG_Ws, IMG_Hs], [0, 0], [IMG_W, 0]])
    M     = cv2.getPerspectiveTransform(src, dst)
    M_inv = cv2.getPerspectiveTransform(dst, src)
    return cv2.warpPerspective(img, M, (IMG_Ws, IMG_Hs)), M_inv


def find_lane_pixels_using_histogram(binary_warped):
    histogram = np.sum(binary_warped[binary_warped.shape[0] // 2:, :], axis=0)
    midpoint = int(histogram.shape[0] // 2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint
    nwindows = 7
    margin = 100
    minpix = 50
    window_height = int((binary_warped.shape[0]//2) // nwindows)
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

def measure_curvature_meters(ploty, left_fitx, right_fitx):
    left_fit_cr  = np.polyfit(ploty * YM_PER_PIX, left_fitx * XM_PER_PIX, 2)
    right_fit_cr = np.polyfit(ploty * YM_PER_PIX, right_fitx * XM_PER_PIX, 2)
    y_eval = np.max(ploty)
    left_c  = ((1 + (2*left_fit_cr[0]*y_eval*YM_PER_PIX+left_fit_cr[1])**2)**1.5) / abs(2*left_fit_cr[0])
    right_c = ((1 + (2*right_fit_cr[0]*y_eval*YM_PER_PIX+right_fit_cr[1])**2)**1.5) / abs(2*right_fit_cr[0])
    return left_c, right_c

def measure_position_meters(left_fit, right_fit):
    y_max  = IMG_Hs
    left_x  = left_fit[0]*y_max**2 + left_fit[1]*y_max + left_fit[2]
    right_x = right_fit[0]*y_max**2 + right_fit[1]*y_max + right_fit[2]
    lane_mid = (left_x + right_x) / 2
    veh_pos  = ((IMG_Ws/2) - lane_mid) * XM_PER_PIX
    return (veh_pos - 0.055)


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

def resize_for_screen(img):
    h, w = img.shape[:2]
    scale = min(SCR_W / w, SCR_H / h)
    return cv2.resize(img, (int(w*scale), int(h*scale)))

def binary_thresholder(img):

    #start = time.time()
    # TODO: Filtro HSV

    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    roi = hsv[480//2:, :,:]
    # Threshold adaptativo HSV
    adapt_white_hsv = cv2.adaptiveThreshold(roi[:,:,2], 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 161, -66)


    # TODO: Fim Filtro HSV

    # Aplicado erosao na imagem gerada pelo HSV Adaptativo, eliminar pequenos ruIdos
    kernel_erode = np.ones((5,5), np.uint8)
    adapt_white_hsv = cv2.erode(adapt_white_hsv, kernel_erode, iterations=1)
    
    #final = adapt_white_hsv

    # Aplicando filtro de mediana na imagem combinada
    median_kernel_size =  7 # Deve ser um numero Impar, como 3, 5, 7
    adapt_white_hsv = cv2.medianBlur(adapt_white_hsv, median_kernel_size)
    
    #cv2.imshow('final filtered', final)
 
    # Aplicando processamento morfologico de dilatacao para realcar as areas brancas
    kernel_dilate = np.ones((6, 6), np.uint8)  # Kernel menor para erosao
    img_dilate = cv2.dilate(adapt_white_hsv, kernel_dilate, iterations=1)

    #cv2.imshow('Adaptativo', adapt_white_hsv)

    #end = time.time()
    return img_dilate, 1#end - start

class Resolution :
    width = 1280
    height = 720

def main():
    # Open the ZED camera
    cap = cv2.VideoCapture(0)
    if cap.isOpened() == 0:
        exit(-1)

    image_size = Resolution()
    image_size.width = 1280
    image_size.height = 720

    # Set the video resolution to HD720
    #cap = cv2.VideoCapture(0)
    #cap.set(cv2.CAP_PROP_FRAME_WIDTH, image_size.width*2)
    #cap.set(cv2.CAP_PROP_FRAME_HEIGHT, image_size.height)
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Actual resolution: {actual_width}x{actual_height}")
    
    #almost_can = inicializar_serial()
    #if not almost_can:
    #    return
    target_width = 1280
    target_height = 720
    target_widthsmall = 640
    target_heightsmall = 480
    roi_start_row = target_heightsmall//2
    top_padding = np.zeros((roi_start_row, target_widthsmall), dtype=np.uint8)
    
    frame2 = cv2.imread("imagens_17_31_0063.png")
    frame2 = cv2.resize(frame2, (target_widthsmall, target_heightsmall))
    
    try:
        while True:
            try:
                inicio = time.time()
                #error_code = camera.grab(runtime_params)

                retval,images = cap.read()
                images = np.split(images, 2, axis=1)
                frame = images[0]
                
                frame = cv2.resize(frame, (target_widthsmall, target_heightsmall))
                #cv2.imshow("left RAW", frame)
                #cv2.waitKey(1)
                
                
                warped_frame, perspective_transform = warp(frame)
                #resized_warped_frame = resize_for_screen(warped_frame)
                #cv2.imshow('Warped Image', resized_warped_frame)
                #cv2.waitKey(1)

                img_bin, tempo = binary_thresholder(warped_frame)
                
                #print(f"tempo de execucao bin: {tempo: .3f} seg")
                img_bin = np.vstack((top_padding, img_bin))

                #cv2.imshow('Binary Image', img_bin)
                #cv2.waitKey(1)
                
                leftx, lefty, rightx, righty = find_lane_pixels_using_histogram(img_bin)
                left_fit, right_fit, left_fitx, right_fitx, ploty = fit_poly(img_bin, leftx, lefty, rightx, righty)

                #left_curverad, right_curverad = measure_curvature_meters(ploty, left_fitx, right_fitx)

                vehicle_position = measure_position_meters(left_fit, right_fit)

                steering_angle = calcular_angulo_do_volante(vehicle_position)
                steering_angle_int = int(steering_angle)
                
                #lane_overlay, avg_curvature = project_lane_info(
                #    frame[:, :, 0:3], img_bin, ploty, left_fitx, right_fitx,
                #    perspective_transform, left_curverad, right_curverad, vehicle_position
                #)
                #resized_lane_overlay = resize_for_screen(lane_overlay)
                #cv2.imshow('Lane Detection Result', resized_lane_overlay)
                
                #cv2.waitKey(1)
                
                

                send_can_message(steering_angle_int)
                fim = time.time()
                
                print(f"tempo de execucao: {fim - inicio: .3f} seg")

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except KeyboardInterrupt:
                print("Execucao interrompida pelo usuario.")
                break
            except Exception as ex:
                print(f"Erro: {ex}")
                continue
    finally:
        cv2.destroyAllWindows()
        #almost_can.serial.close()
        import gc
        gc.collect()
        print("Recursos liberados. Execucao finalizada.")

if __name__ == "__main__":
    main()

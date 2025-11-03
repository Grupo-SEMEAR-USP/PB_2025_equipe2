import cv2
import numpy as np
from scipy.optimize import curve_fit

## STATUS:
#           'F' --> Free
#           'W' --> Warning
#           'C' --> Critical
#           'L' --> Left
#           'R' --> Right
#           'S' --> Straight
#           'I' --> Indetermined

w_radius = 1                        # AJUSTAR O RAIO DA RODA
base_width = 1                      # EM CM

## ------------------ Distância horizontal robô até linha --------------------

## Modelo de função
def exp_model(y, a, b, c):
    return a * np.exp(b * y) + c

## Calibra a curva
def calibrate_distance(y_pixels, real_distances):
    y_pixels = np.array(y_pixels, dtype= float)
    real_distances = np.array(real_distances, dtype= float)

    popt, _ = curve_fit(exp_model, y_pixels, real_distances, maxfev= 5000)
    return popt

## Coloca o ponto na curva
def estimate_distances(y_pixel, params):
    a, b, c = params
    return float(exp_model(y_pixel, a, b, c))

## Função Geral
def collision_manager(closest_inter, params, current_vel, frame_height= 360):

    if closest_inter is None:
        status = 'I'
        return None, None, None, status
    
    _, y = closest_inter
    y = np.clip(y, 0, frame_height-1)

    D = estimate_distances(y, params)
    D_rad = D / w_radius

    collision_time = D_rad / current_vel if current_vel != 0 else -1

    if D <= base_width:
        status = 'C'

    elif D > base_width:

        if collision_time >= 1.5:
            status = 'F'
        
        elif 0.5 <= collision_time < 1.5:
            status = 'W'
        
        elif 0 < collision_time < 0.5:
            status = 'C'
        
        else:
            status = 'I'

    return status


## ------------------ Análise áreas --------------------

def area_comp(area_l, area_r):

    if area_l > area_r:

        larger_side = area_l
        f = area_l / area_r

    elif area_r > area_l:

        larger_side = area_r
        f = area_r / area_l

    else:
        f = 1

    return f, larger_side

## ------------------ Análise ângulo --------------------

def ang_comp(min_angle, last_min_angle):

    var_angle = min_angle - last_min_angle
    tx_var_angle = min_angle / last_min_angle  if last_min_angle != 0 else 0

    last_min_angle = min_angle

    return var_angle, tx_var_angle, last_min_angle
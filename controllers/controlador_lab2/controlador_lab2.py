from controller import Robot
import csv

SCENARIO_NAME = "complejo"  # Cambiar a "simple" o "complejo"

#puede ser "CRUDO", "FILTRADO" o "KALMAN"
MODO_NAVEGACION = "KALMAN"

iteracion = 0
MAX_ITERACIONES = 5000 

robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

WHEEL_RADIUS = 0.0205
MAX_SPEED = 6.28

ADVANCE_SPEED = MAX_SPEED * 0.4   # Lento por defecto mas que nada para tener mayor control.
BACKUP_SPEED = MAX_SPEED * 0.4
TURN_SPEED = MAX_SPEED * 0.6

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

left_encoder = robot.getDevice('left wheel sensor')
right_encoder = robot.getDevice('right wheel sensor')
left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)

front_right_sensor = robot.getDevice('ps0')
front_left_sensor = robot.getDevice('ps7')
diag_right_sensor = robot.getDevice('ps1')
diag_left_sensor = robot.getDevice('ps6')
left_sensor = robot.getDevice('ps2')
right_sensor = robot.getDevice('ps5')

for s in [front_right_sensor, front_left_sensor, diag_right_sensor, diag_left_sensor, left_sensor, right_sensor]:
    s.enable(TIME_STEP)

prev_enc_left = 0.0
prev_enc_right = 0.0
first_iteration = True

# Kalman
d_est = 0.05
P = 1.0
Q = 0.001
R = 0.0005

alpha_ema = 0.3
filtro_simple_val = 0.05

historial_crudo = []
historial_filtrado = []
historial_kalman = []
historial_avance = []

T_s_segundos = TIME_STEP / 1000.0
f_s = 1.0 / T_s_segundos

print(f"Controlador iniciado. Ts = {TIME_STEP} ms")
print(f"Frecuencia de muestreo (fs) = {f_s:.2f} Hz")
print(f"Modo de navegación activo: {MODO_NAVEGACION}")

# Parámetros de navegación
SAFE_DISTANCE = 0.045
OBSTACLE_RAW_THRESHOLD = 200
BACKUP_STEPS = 15
MIN_TURN_STEPS = 20       # giro mínimo
MAX_TURN_STEPS = 120      # cap por si nunca se libera
ESCAPE_TURN_STEPS = 90   # giro forzado cuando está atascado (ajustar si gira más o menos de 180 grados)

# Parámetros Anti-Atasco
STUCK_WINDOW = 600             # ventana de memoria para detectar bucles infinitos
STUCK_THRESHOLD = 4            # si hace N giros en la ventana, está atrapado en un bucle
TIEMPO_MAXIMO_ATASCADO = 150   # iteraciones máximas permitidas sin lograr avanzar
tiempo_sin_avanzar = 0

# Máquina de estados
STATE_ADVANCING = 0
STATE_BACKING = 1
STATE_TURNING = 2
nav_state = STATE_ADVANCING

backup_count = 0
turn_steps_done = 0
turn_max_current = MIN_TURN_STEPS   # se ajusta por giro (normal o escape)
is_turning_left = False
turn_history = []        # iteraciones donde se inició cada giro
is_escape_turn = False   # True = giro de escape (no salir antes del tope)

while robot.step(TIME_STEP) != -1:

    val_enc_left = left_encoder.getValue()
    val_enc_right = right_encoder.getValue()

    if first_iteration:
        prev_enc_left = val_enc_left
        prev_enc_right = val_enc_right
        first_iteration = False

    delta_theta_left = val_enc_left - prev_enc_left
    delta_theta_right = val_enc_right - prev_enc_right
    avance_lineal = WHEEL_RADIUS * ((delta_theta_left + delta_theta_right) / 2.0)
    prev_enc_left = val_enc_left
    prev_enc_right = val_enc_right

    v_fr = front_right_sensor.getValue()
    v_fl = front_left_sensor.getValue()
    v_dr = diag_right_sensor.getValue()
    v_dl = diag_left_sensor.getValue()
    v_l = left_sensor.getValue()
    v_r = right_sensor.getValue()

    dist_fr = 0.05 * (1.0 - (v_fr / 4095.0))
    dist_fl = 0.05 * (1.0 - (v_fl / 4095.0))
    # Expandir la visión a las diagonales para no estrellarse en las esquinas
    dist_dr = 0.05 * (1.0 - (v_dr / 4095.0))
    dist_dl = 0.05 * (1.0 - (v_dl / 4095.0))
    
    z_k = min(dist_fr, dist_fl, dist_dr, dist_dl)

    # Filtro EMA
    filtro_simple_val = alpha_ema * z_k + (1 - alpha_ema) * filtro_simple_val

    # Kalman
    delta_d_k = -avance_lineal
    d_pred = d_est + delta_d_k
    P_pred = P + Q
    K_k = P_pred / (P_pred + R)
    d_est = d_pred + K_k * (z_k - d_pred)
    P = (1 - K_k) * P_pred

    historial_crudo.append(z_k)
    historial_filtrado.append(filtro_simple_val)
    historial_kalman.append(d_est)
    historial_avance.append(avance_lineal)

    # Lógica de detección basada en el modo que se selecciono
    if MODO_NAVEGACION == "CRUDO":
        obstacle_detected = z_k <= SAFE_DISTANCE
    elif MODO_NAVEGACION == "FILTRADO":
        obstacle_detected = filtro_simple_val <= SAFE_DISTANCE
    elif MODO_NAVEGACION == "KALMAN":
        obstacle_detected = d_est <= SAFE_DISTANCE
    else:
        raise ValueError(f"MODO_NAVEGACION inválido: '{MODO_NAVEGACION}'. Debe ser 'CRUDO', 'FILTRADO' o 'KALMAN'.")

    # frustación del robot
    if nav_state == STATE_ADVANCING and not obstacle_detected:
        tiempo_sin_avanzar = 0  # Libre de obstáculos
    else:
        tiempo_sin_avanzar += 1 # Lidiando con problemas

    # Máquina de estados
    if nav_state == STATE_ADVANCING:
        if obstacle_detected:
            # 1. Registrar este intento de giro
            turn_history.append(iteracion)
            turn_history = [t for t in turn_history if iteracion - t < STUCK_WINDOW]

            # 2. Evaluar trampas
            bucle_infinito = len(turn_history) >= STUCK_THRESHOLD
            atascado_tiempo = tiempo_sin_avanzar > TIEMPO_MAXIMO_ATASCADO

            stuck = bucle_infinito or atascado_tiempo

            if stuck:
                # Escape: giro forzado de 180 grados
                motivo = "BUCLE" if bucle_infinito else "TIEMPO"
                print(f">>> [It {iteracion}] ¡Atascado por {motivo}! Forzando escape 180°")
                is_turning_left = not is_turning_left
                turn_max_current = ESCAPE_TURN_STEPS
                is_escape_turn = True
                turn_history = []
                tiempo_sin_avanzar = 0 
            else:
                peso_derecho = v_r + v_dr
                peso_izquierdo = v_l + v_dl
                is_turning_left = (peso_derecho > peso_izquierdo)
                turn_max_current = MAX_TURN_STEPS
                is_escape_turn = False

            backup_count = BACKUP_STEPS
            turn_steps_done = 0
            nav_state = STATE_BACKING
            left_speed = -BACKUP_SPEED
            right_speed = -BACKUP_SPEED
        else:
            left_speed = ADVANCE_SPEED
            right_speed = ADVANCE_SPEED

    elif nav_state == STATE_BACKING:
        backup_count -= 1
        if backup_count > 0:
            left_speed = -BACKUP_SPEED
            right_speed = -BACKUP_SPEED
        else:
            nav_state = STATE_TURNING
            if is_turning_left:
                left_speed = -TURN_SPEED
                right_speed = TURN_SPEED
            else:
                left_speed = TURN_SPEED
                right_speed = -TURN_SPEED

    elif nav_state == STATE_TURNING:
        turn_steps_done += 1
        min_done = turn_steps_done >= MIN_TURN_STEPS
        max_reached = turn_steps_done >= turn_max_current
        
        # Validar que el frente esté libre según la señal seleccionada
        if MODO_NAVEGACION == "CRUDO":
            front_clear = z_k > SAFE_DISTANCE
        elif MODO_NAVEGACION == "FILTRADO":
            front_clear = filtro_simple_val > SAFE_DISTANCE
        else:  # KALMAN
            front_clear = d_est > SAFE_DISTANCE

        # En giro de escape: completar todos los pasos sin salida anticipada
        if is_escape_turn:
            done = max_reached
        else:
            done = max_reached or (min_done and front_clear)

        if done:
            nav_state = STATE_ADVANCING
            left_speed = ADVANCE_SPEED
            right_speed = ADVANCE_SPEED
            tiempo_sin_avanzar = 0  # Resetear frustración al terminar cualquier maniobra
            is_escape_turn = False
        else:
            if is_turning_left:
                left_speed = -TURN_SPEED
                right_speed = TURN_SPEED
            else:
                left_speed = TURN_SPEED
                right_speed = -TURN_SPEED

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    iteracion += 1

    # para no saturar la consola, se imprime cada 50
    if iteracion % 50 == 0:
        estado_str = ["AVANZA", "RETROCEDE", "GIRA"][nav_state]
        print(f"It:{iteracion} | Estado:{estado_str} | Zk:{z_k:.4f} | Filtr:{filtro_simple_val:.4f} | Kal:{d_est:.4f} | MODO:{MODO_NAVEGACION}")

    if iteracion >= MAX_ITERACIONES:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        break


def _guardar_csv():
    archivo_csv = f'datos_sensores_{SCENARIO_NAME}_{MODO_NAVEGACION}.csv'
    with open(archivo_csv, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Z_Crudo', 'Z_Filtrado', 'D_Kalman', 'Avance_Lineal'])
        for i in range(len(historial_crudo)):
            writer.writerow([historial_crudo[i], historial_filtrado[i], historial_kalman[i], historial_avance[i]])
    print(f"¡Datos guardados exitosamente en {archivo_csv}! ({len(historial_crudo)} muestras)")


_guardar_csv()
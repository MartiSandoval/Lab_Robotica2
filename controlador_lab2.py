from controller import Robot
import csv

iteracion = 0
MAX_ITERACIONES = 1000

robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

WHEEL_RADIUS = 0.0205  
MAX_SPEED = 6.28       

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
left_sensor = robot.getDevice('ps2')
right_sensor = robot.getDevice('ps5')

for sensor in [front_right_sensor, front_left_sensor, left_sensor, right_sensor]:
    sensor.enable(TIME_STEP)

prev_enc_left = 0.0
prev_enc_right = 0.0
first_iteration = True


d_est = 0.05 
P = 1.0       
Q = 0.0001    
R = 0.01      

alpha_ema = 0.3
filtro_simple_val = 0.05

historial_crudo = []
historial_filtrado = []
historial_kalman = []
turn_counter = 0
is_turning_left = False

T_s_segundos = TIME_STEP / 1000.0
f_s = 1.0 / T_s_segundos

print(f"Controlador iniciado. Ts = {TIME_STEP} ms")
print(f"Frecuencia de muestreo (fs) = {f_s:.2f} Hz")

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
    v_l = left_sensor.getValue()
    v_r = right_sensor.getValue()

    dist_fr = 0.05 * (1.0 - (v_fr / 4095.0))
    dist_fl = 0.05 * (1.0 - (v_fl / 4095.0))

    z_k = min(dist_fr, dist_fl)
    
    filtro_simple_val = alpha_ema * z_k + (1 - alpha_ema) * filtro_simple_val

    delta_d_k = -avance_lineal
    d_pred = d_est + delta_d_k 
    P_pred = P + Q

    K_k = P_pred / (P_pred + R)                     
    d_est = d_pred + K_k * (z_k - d_pred)           
    P = (1 - K_k) * P_pred                         
    
    historial_crudo.append(z_k)
    historial_filtrado.append(filtro_simple_val)
    historial_kalman.append(d_est)
    
    SAFE_DISTANCE = 0.025
    
    if turn_counter > 0:
        turn_counter -= 1
        if is_turning_left:
            left_speed = -MAX_SPEED * 0.6
            right_speed = MAX_SPEED * 0.6
        else:
            left_speed = MAX_SPEED * 0.6
            right_speed = -MAX_SPEED * 0.6
    else:
        if d_est > SAFE_DISTANCE:
            left_speed = MAX_SPEED * 0.6
            right_speed = MAX_SPEED * 0.6
        else:
            turn_counter = 50  
            if v_l > v_r:
                is_turning_left = False
                left_speed = MAX_SPEED * 0.6
                right_speed = -MAX_SPEED * 0.6
            else:
                is_turning_left = True
                left_speed = -MAX_SPEED * 0.6
                right_speed = MAX_SPEED * 0.6

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    iteracion += 1
  
    if iteracion == MAX_ITERACIONES:
        with open('datos_sensores.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Z_Crudo', 'Z_Filtrado', 'D_Kalman'])
            for i in range(len(historial_crudo)):
                writer.writerow([historial_crudo[i], historial_filtrado[i], historial_kalman[i]])
        print("¡Datos guardados exitosamente en datos_sensores.csv!")
    

    print(f"Zk: {z_k:.4f} | Kalman: {d_est:.4f} | Accion: {'Avanza' if d_est > SAFE_DISTANCE else 'Gira'}")
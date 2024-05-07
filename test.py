# -- coding: utf-8 --
import time
import pigpio

motor1_pwm_pin = 12
motor1_dir_pin = 24
motor1_en_pin = 22
motor2_pwm_pin = 13
motor2_dir_pin = 25
motor2_en_pin = 23

PIN_ENCODER_A = 4
PIN_ENCODER_B = 17
INTERVALO = 1  # Intervalo de 1 segundo en segundos
numero_flancos_A = 0
numero_flancos_B = 0
tiempo_anterior = 0
RPS = 0.0
RPM = 0.0
tiempo_actual = 0

PIN_ENCODER2_A = 16
PIN_ENCODER2_B = 19
INTERVALO = 1  # Intervalo de 1 segundo en segundos
numero_flancos_A2 = 0
numero_flancos_B2 = 0
tiempo_anterior2 = 0
RPS2 = 0.0
RPM2 = 0.0
tiempo_actual2 = 0


pi = pigpio.pi()


pi.set_mode(PIN_ENCODER_A, pigpio.INPUT)
pi.set_pull_up_down(PIN_ENCODER_A, pigpio.PUD_UP)
pi.set_mode(PIN_ENCODER_B, pigpio.INPUT)
pi.set_pull_up_down(PIN_ENCODER_B, pigpio.PUD_UP)

pi.set_mode(PIN_ENCODER2_A, pigpio.INPUT)
pi.set_pull_up_down(PIN_ENCODER2_A, pigpio.PUD_UP)
pi.set_mode(PIN_ENCODER2_B, pigpio.INPUT)
pi.set_pull_up_down(PIN_ENCODER2_B, pigpio.PUD_UP)

def contador_flancos_encoder(gpio, level, tick):
    global numero_flancos_A
    numero_flancos_A += 1

def contador_flancos_encoder_b(gpio, level, tick):
    global numero_flancos_B
    numero_flancos_B += 1

def contador_flancos_encoder2(gpio, level, tick):
    global numero_flancos_A2
    numero_flancos_A2 += 1

def contador_flancos_encoder_b2(gpio, level, tick):
    global numero_flancos_B2
    numero_flancos_B2 += 1


cb1 = pi.callback(PIN_ENCODER_A, pigpio.EITHER_EDGE, contador_flancos_encoder)
cb2 = pi.callback(PIN_ENCODER_B, pigpio.EITHER_EDGE, contador_flancos_encoder_b)

cb3 = pi.callback(PIN_ENCODER2_A, pigpio.EITHER_EDGE, contador_flancos_encoder2)
cb4 = pi.callback(PIN_ENCODER2_B, pigpio.EITHER_EDGE, contador_flancos_encoder_b2)

def control_motor(pin_pwm, pin_dir, speed_percent, direction):
    duty_cycle = int(speed_percent * 255 / 100)
    pi.set_PWM_dutycycle(pin_pwm, duty_cycle)

    if direction == 'forward':
        pi.write(pin_dir, 1)  # Establecer dirección hacia adelante
    elif direction == 'backward':
        pi.write(pin_dir, 0)  # Establecer dirección hacia atrás
    else:
        raise ValueError("Dirección no válida. Usa 'forward' o 'backward'.")

def main():
    global numero_flancos_A, numero_flancos_B, tiempo_anterior, numero_flancos_A2, numero_flancos_B2, tiempo_anterior2
    tiempo_anterior = time.time()  
    tiempo_anterior2 = time.time()  
    pi.write(motor1_en_pin, 1)  # Habilitar motor 1
    pi.write(motor2_en_pin, 1)  # Habilitar motor 2

    file_path = '/home/santiago/Documents/dispensador/dispensador/Pbrs.txt'

    with open(file_path, 'r') as file:
        lines = file.readlines()
        total_lines = len(lines)
        current_line1 = 0
        current_line2 = 1
       
        start_time = time.time()
        while time.time() - start_time <= 40:  # Ejemplo: Ejecutar durante 20 segundos
            tiempo_actual = time.time()
            tiempo_actual2 = time.time()
            line1 = lines[current_line1].strip()
            line2 = lines[current_line2].strip()
            motor1_speed = int(line1) 
            motor2_speed = int(line2) 

            control_motor(motor1_pwm_pin, motor1_dir_pin, 60, 'forward')
            control_motor(motor2_pwm_pin, motor2_dir_pin, 60, 'forward')

            print('Leyendo línea {}: {}'.format(current_line1 + 1, line1))  # Mostrar la línea que se está leyendo

            current_line1 = (current_line1 + 1) % total_lines  # Avanzar al siguiente valor circularmente
            current_line2 = (current_line2 + 1) % total_lines  # Avanzar al siguiente valor circularmente
            print('Velocidad motor 1:', motor1_speed)
            print('Velocidad motor 2:', motor2_speed)
            
            # Calcular RPM usando los flancos contados
            tiempo_pasado = tiempo_actual - tiempo_anterior
            tiempo_pasado2 = tiempo_actual2 - tiempo_anterior2
            if tiempo_pasado >= INTERVALO:
                RPS = (numero_flancos_A + numero_flancos_B) / (1200)  # Se divide por 1200 ya que se cuentan flancos A y B
                RPM = RPS * 60
                print("Revoluciones por segundo M1: {:.20f} | Revoluciones por Minuto M1: {:.20f}" (RPS, RPM))

                RPS2 = (numero_flancos_A2 + numero_flancos_B2) / (1200)  # Se divide por 2 ya que se cuentan flancos A y B
                totalflan2= numero_flancos_A2 + numero_flancos_B2
                print(RPM)
                RPM2 = RPS2 * 60
                print("Revoluciones por segundo M2: {:.20f} | Revoluciones por Minuto M2: {:.20f}" (RPS2, RPM2))

                numero_flancos_B = 0
                numero_flancos_A = 0
                tiempo_anterior = tiempo_actual
                numero_flancos_B2 = 0
                numero_flancos_A2 = 0
                tiempo_anterior2 = tiempo_actual2

            time.sleep(1)

        pi.set_PWM_dutycycle(motor1_pwm_pin, 0)
        pi.set_PWM_dutycycle(motor2_pwm_pin, 0)

        pi.write(motor1_en_pin, 0)  # Deshabilitar motor 1
        pi.write(motor2_en_pin, 0)  # Deshabilitar motor 2

        pi.stop()
        print('Tiempo de funcionamiento de los motores completado.')

main()

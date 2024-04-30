# -- coding: utf-8 --
import time
import pigpio

motor1_pwm_pin = 12
motor1_dir_pin = 24
motor1_en_pin = 22
motor2_pwm_pin = 13
motor2_dir_pin = 25
motor2_en_pin = 23
pinA_motor1 = 17  # Pin A del encoder del motor 1
pinA_motor2 = 18  # Pin A del encoder del motor 2

pi = pigpio.pi()
pi.set_mode(pinA_motor1, pigpio.INPUT)
pi.set_mode(pinA_motor2, pigpio.INPUT)

global rpm_count_motor1
global rpm_count_motor2
rpm_count_motor1 = 0
rpm_count_motor2 = 0
last_state_motor1 = pi.read(pinA_motor1)
last_state_motor2 = pi.read(pinA_motor2)

# Función para contar las RPM del motor 1
def count_rpm_motor1(gpio, level, tick):
    global rpm_count_motor1
    global last_state_motor1

    state = pi.read(pinA_motor1)
    if state != last_state_motor1:
        rpm_count_motor1 += 1

    last_state_motor1 = state

cb_motor1 = pi.callback(pinA_motor1, pigpio.RISING_EDGE, count_rpm_motor1)

# Función para contar las RPM del motor 2
def count_rpm_motor2(gpio, level, tick):
    global rpm_count_motor2
    global last_state_motor2

    state = pi.read(pinA_motor2)
    if state != last_state_motor2:
        rpm_count_motor2 += 1

    last_state_motor2 = state

cb_motor2 = pi.callback(pinA_motor2, pigpio.RISING_EDGE, count_rpm_motor2)

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
    global rpm_count_motor1
    global rpm_count_motor2

    pi.write(motor1_en_pin, 1)  # Habilitar motor 1
    pi.write(motor2_en_pin, 1)  # Habilitar motor 2

    file_path = '/home/santiago/Documents/dispensador/dispensador/Pbrs.txt'

    with open(file_path, 'r') as file:
        lines = file.readlines()
        total_lines = len(lines)
        current_line1 = 0
        current_line2 = 1
       
        start_time = time.time()
        while time.time() - start_time <= 20:  # Ejemplo: Ejecutar durante 20 segundos
            line1 = lines[current_line1].strip()
            line2 = lines[current_line2].strip()
            motor1_speed = int(line1) 
            motor2_speed = int(line2) 

            control_motor(motor1_pwm_pin, motor1_dir_pin, motor1_speed, 'forward')
            control_motor(motor2_pwm_pin, motor2_dir_pin, motor2_speed, 'forward')

            print('Leyendo línea {}: {}'.format(current_line1 + 1, line1))  # Mostrar la línea que se está leyendo

            current_line1 = (current_line1 + 1) % total_lines  # Avanzar al siguiente valor circularmente
            current_line2 = (current_line2 + 1) % total_lines  # Avanzar al siguiente valor circularmente
            print('Velocidad motor 1:', motor1_speed)
            print('Velocidad motor 2:', motor2_speed)
            print('RPM motor 1:', rpm_count_motor1)
            print('RPM motor 2:', rpm_count_motor2)
            time.sleep(0.5)  # Esperar 0.5 segundos antes de leer la siguiente línea

        pi.set_PWM_dutycycle(motor1_pwm_pin, 0)
        pi.set_PWM_dutycycle(motor2_pwm_pin, 0)

        pi.write(motor1_en_pin, 0)  # Deshabilitar motor 1
        pi.write(motor2_en_pin, 0)  # Deshabilitar motor 2

        cb_motor1.cancel()  # Cancelar la lectura de RPM del motor 1
        cb_motor2.cancel()  # Cancelar la lectura de RPM del motor 2

        pi.stop()
        print('Tiempo de funcionamiento de los motores completado.')

main()

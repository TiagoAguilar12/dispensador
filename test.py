# -- coding: utf-8 --
import time
import pigpio

motor1_pwm_pin = 12
motor1_dir_pin = 24
motor1_en_pin = 22
motor1_enc_pinA = 17  # Pin del encoder para el motor 1
motor1_enc_pinB = 18  # Pin del encoder para el motor 1

motor2_pwm_pin = 13
motor2_dir_pin = 25
motor2_en_pin = 23
motor2_enc_pinA = 19  # Pin del encoder para el motor 2
motor2_enc_pinB = 20  # Pin del encoder para el motor 2

pi = pigpio.pi()

def control_motor(pin_pwm, pin_dir, speed_percent, direction):
    duty_cycle = int(speed_percent * 255 / 100)
    pi.set_PWM_dutycycle(pin_pwm, duty_cycle)

    if direction == 'forward':
        pi.write(pin_dir, 1)  # Establecer dirección hacia adelante
    elif direction == 'backward':
        pi.write(pin_dir, 0)  # Establecer dirección hacia atrás
    else:
        raise ValueError("Dirección no válida. Usa 'forward' o 'backward'.")

def read_encoder(encoder_pinA, encoder_pinB):
    return pi.read(encoder_pinA) ^ pi.read(encoder_pinB)

def calculate_rpm(encoder_pinA, encoder_pinB, start_time, elapsed_time):
    ticks = 0
    while time.time() - start_time <= elapsed_time:
        if read_encoder(encoder_pinA, encoder_pinB):
            ticks += 1
        time.sleep(0.001)  # Esperar un breve tiempo para evitar lecturas demasiado frecuentes
    rpm = (ticks * 60) / (elapsed_time * 360)  # Calcular RPM basado en el número de ticks y el tiempo transcurrido
    return rpm

def main():
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

            # Calcular RPM para cada motor
            motor1_rpm = calculate_rpm(motor1_enc_pinA, motor1_enc_pinB, start_time, 0.5)
            motor2_rpm = calculate_rpm(motor2_enc_pinA, motor2_enc_pinB, start_time, 0.5)
            
            print('Velocidad motor 1:', motor1_speed)
            print('RPM motor 1:', motor1_rpm)
            print('Velocidad motor 2:', motor2_speed)
            print('RPM motor 2:', motor2_rpm)

            time.sleep(0.5)  # Esperar 0.5 segundos antes de leer la siguiente línea

        pi.set_PWM_dutycycle(motor1_pwm_pin, 0)
        pi.set_PWM_dutycycle(motor2_pwm_pin, 0)

        pi.write(motor1_en_pin, 0)  # Deshabilitar motor 1
        pi.write(motor2_en_pin, 0)  # Deshabilitar motor 2

        pi.stop()
        print('Tiempo de funcionamiento de los motores completado.')

main()

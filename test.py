# -- coding: utf-8 --
import time
import pigpio

motor1_pwm_pin = 12
motor1_dir_pin = 24
motor1_en_pin = 22
motor1_enc_a_pin = 4  # Pin del encoder A del motor 1
motor1_enc_b_pin = 5  # Pin del encoder B del motor 1

motor2_pwm_pin = 13
motor2_dir_pin = 25
motor2_en_pin = 23
motor2_enc_a_pin = 6  # Pin del encoder A del motor 2
motor2_enc_b_pin = 13  # Pin del encoder B del motor 2

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

def count_encoder_pulse(gpio, level, tick):
    global motor1_encoder_count, motor2_encoder_count
    if gpio == motor1_enc_a_pin:
        motor1_encoder_count += 1 if pi.read(motor1_enc_b_pin) == 0 else -1
    elif gpio == motor2_enc_a_pin:
        motor2_encoder_count += 1 if pi.read(motor2_enc_b_pin) == 0 else -1

def main():
    global motor1_encoder_count, motor2_encoder_count
    motor1_encoder_count = 0
    motor2_encoder_count = 0

    pi.write(motor1_en_pin, 1)  # Habilitar motor 1
    pi.write(motor2_en_pin, 1)  # Habilitar motor 2

    # Configurar detección de pulsos del encoder
    pi.callback(motor1_enc_a_pin, pigpio.EITHER_EDGE, count_encoder_pulse)
    pi.callback(motor2_enc_a_pin, pigpio.EITHER_EDGE, count_encoder_pulse)

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

            # Calcular RPM basado en el conteo del encoder
            motor1_rpm = motor1_encoder_count * 60 / (64 * 20)  # 64 CPR * 20:1 gearbox
            motor2_rpm = motor2_encoder_count * 60 / (64 * 20)  # 64 CPR * 20:1 gearbox
            print('RPM motor 1:', motor1_rpm)
            print('RPM motor 2:', motor2_rpm)

            time.sleep(0.5)  # Esperar 0.5 segundos antes de leer la siguiente línea

        pi.set_PWM_dutycycle(motor1_pwm_pin, 0)
        pi.set_PWM_dutycycle(motor2_pwm_pin, 0)

        pi.write(motor1_en_pin, 0)  # Deshabilitar motor 1
        pi.write(motor2_en_pin, 0)  # Deshabilitar motor 2

        pi.stop()
        print('Tiempo de funcionamiento de los motores completado.')

main()

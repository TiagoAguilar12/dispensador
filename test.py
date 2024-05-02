# -- coding: utf-8 --
import time
import pigpio

motor1_pwm_pin = 12
motor1_dir_pin = 24
motor1_en_pin = 22
motor1_encoder_pinA = 4  # Pin del encoder del motor 1 para la señal A
motor1_encoder_pinB = 5  # Pin del encoder del motor 1 para la señal B

motor2_pwm_pin = 13
motor2_dir_pin = 25
motor2_en_pin = 23
motor2_encoder_pinA = 6  # Pin del encoder del motor 2 para la señal A
motor2_encoder_pinB = 13  # Pin del encoder del motor 2 para la señal B

pi = pigpio.pi()

encoder_counts_per_rev = 64  # Número de cuentas por revolución del encoder

motor1_encoder_count = 0
motor2_encoder_count = 0

def control_motor(pin_pwm, pin_dir, speed_percent, direction):
    duty_cycle = int(speed_percent * 255 / 100)
    pi.set_PWM_dutycycle(pin_pwm, duty_cycle)

    if direction == 'forward':
        pi.write(pin_dir, 1)  # Establecer dirección hacia adelante
    elif direction == 'backward':
        pi.write(pin_dir, 0)  # Establecer dirección hacia atrás
    else:
        raise ValueError("Dirección no válida. Usa 'forward' o 'backward'.")

def encoder_callback1A(gpio, level, tick):
    global motor1_encoder_count
    motor1_encoder_count += 1 if level else -1

def encoder_callback1B(gpio, level, tick):
    global motor1_encoder_count
    motor1_encoder_count += -1 if level else 1

def encoder_callback2A(gpio, level, tick):
    global motor2_encoder_count
    motor2_encoder_count += 1 if level else -1

def encoder_callback2B(gpio, level, tick):
    global motor2_encoder_count
    motor2_encoder_count += -1 if level else 1

def calcular_rpm(encoder_count, elapsed_time):
    return (encoder_count / (encoder_counts_per_rev * elapsed_time)) * 60

def main():
    global motor1_encoder_count, motor2_encoder_count

    pi.write(motor1_en_pin, 1)  # Habilitar motor 1
    pi.write(motor2_en_pin, 1)  # Habilitar motor 2

    # Configurar callbacks para los flancos del encoder
    pi.callback(motor1_encoder_pinA, pigpio.EITHER_EDGE, encoder_callback1A)
    pi.callback(motor1_encoder_pinB, pigpio.EITHER_EDGE, encoder_callback1B)
    pi.callback(motor2_encoder_pinA, pigpio.EITHER_EDGE, encoder_callback2A)
    pi.callback(motor2_encoder_pinB, pigpio.EITHER_EDGE, encoder_callback2B)

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

            current_line1 = (current_line1 + 1) % total_lines  # Avanzar al siguiente valor circularmente
            current_line2 = (current_line2 + 1) % total_lines  # Avanzar al siguiente valor circularmente

            # Calcular las RPM de cada motor
            elapsed_time = time.time() - start_time
            rpm_motor1 = calcular_rpm(motor1_encoder_count, elapsed_time)
            rpm_motor2 = calcular_rpm(motor2_encoder_count, elapsed_time)

            # Mostrar las RPM de ambos motores
            print('RPM motor 1:', rpm_motor1)
            print('RPM motor 2:', rpm_motor2)

            time.sleep(0.5)  # Esperar 0.5 segundos antes de leer la siguiente línea

        pi.set_PWM_dutycycle(motor1_pwm_pin, 0)
        pi.set_PWM_dutycycle(motor2_pwm_pin, 0)

        pi.write(motor1_en_pin, 0)  # Deshabilitar motor 1
        pi.write(motor2_en_pin, 0)  # Deshabilitar motor 2

        pi.stop()
        print('Tiempo de funcionamiento de los motores completado.')

if __name__ == '__main__':
    main()

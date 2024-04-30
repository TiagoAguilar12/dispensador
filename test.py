# -- coding: utf-8 --
import time
import pigpio

motor1_pwm_pin = 12
motor1_dir_pin = 24
motor1_en_pin = 22

pinA_motor1 = 17  # Pin A del encoder del motor 1

pi = pigpio.pi()
pi.set_mode(pinA_motor1, pigpio.INPUT)

rpm_count_motor1 = 0  # Variable global para contar las RPM del motor 1

def count_rpm(gpio, level, tick):
    global rpm_count_motor1
    rpm_count_motor1 += 1

cb_motor1 = pi.callback(pinA_motor1, pigpio.RISING_EDGE, count_rpm)

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
    rpm_count_motor1 = 0  # Reiniciar el contador de RPM del motor 1

    pi.write(motor1_en_pin, 1)  # Habilitar motor 1

    file_path = '/home/santiago/Documents/dispensador/dispensador/Pbrs.txt'

    with open(file_path, 'r') as file:
        lines = file.readlines()
        total_lines = len(lines)
        current_line1 = 0

        start_time = time.time()
        while time.time() - start_time <= 20:  # Ejemplo: Ejecutar durante 20 segundos
            line1 = lines[current_line1].strip()
            motor1_speed = int(line1) 

            control_motor(motor1_pwm_pin, motor1_dir_pin, motor1_speed, 'forward')

            print('Leyendo línea {}: {}'.format(current_line1 + 1, line1))  # Mostrar la línea que se está leyendo
            print('Velocidad motor 1:', motor1_speed)

            # Esperar 1 segundo antes de leer las RPM
            time.sleep(1)

            print('RPM motor 1:', rpm_count_motor1)

            rpm_count_motor1 = 0  # Reiniciar el contador de RPM para la próxima lectura

            current_line1 = (current_line1 + 1) % total_lines  # Avanzar al siguiente valor circularmente

        pi.set_PWM_dutycycle(motor1_pwm_pin, 0)
        pi.write(motor1_en_pin, 0)  # Deshabilitar motor 1

        cb_motor1.cancel()  # Cancelar la callback del encoder del motor 1

        pi.stop()
        print('Tiempo de funcionamiento del motor 1 completado.')

main()

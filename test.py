import time
import pigpio

motor1_pwm_pin = 12
motor1_dir_pin = 24
motor1_en_pin = 22
motor2_pwm_pin = 13
motor2_dir_pin = 25
motor2_en_pin = 23
pinA = 17  # Pin del encoder, modifícalo según tu configuración
pinB = 18  # Si utilizas otro pin del encoder, modifícalo aquí

pi = pigpio.pi()
pi.set_mode(pinA, pigpio.INPUT)

global rpm_count  # Declarar rpm_count como global

rpm_count = 0
last_state = pi.read(pinA)

# Función para contar las RPM
def count_rpm(gpio, level, tick):
    global rpm_count, last_state

    state = pi.read(pinA)
    if state != last_state:
        rpm_count += 1

    last_state = state

cb = pi.callback(pinA, pigpio.RISING_EDGE, count_rpm)

# Función para controlar la velocidad y dirección de los motores
def control_motor(pin_pwm, pin_dir, speed_percent, direction):
    duty_cycle = int(speed_percent * 255 / 100)  # Convertir el porcentaje de velocidad a ciclo de trabajo (0-255)
    pi.set_PWM_dutycycle(pin_pwm, duty_cycle)

    if direction == 'forward':
        pi.write(pin_dir, 1)  # Establecer dirección hacia adelante
    elif direction == 'backward':
        pi.write(pin_dir, 0)  # Establecer dirección hacia atrás
    else:
        raise ValueError("Dirección no válida. Usa 'forward' o 'backward'.")

def main():
    global rpm_count  # Declarar rpm_count como global dentro de main

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

            # Calcular las RPM y mostrarlas
            rpm = (rpm_count * 60) / 19  # Fórmula para calcular las RPM basado en el contador de pulsos
            print("RPM: {:.2f}".format(rpm))

            rpm_count = 0  # Reiniciar el contador de pulsos después de calcular las RPM

            time.sleep(0.5)  # Esperar 0.5 segundos antes de leer la siguiente línea

        pi.set_PWM_dutycycle(motor1_pwm_pin, 0)
        pi.set_PWM_dutycycle(motor2_pwm_pin, 0)

        pi.write(motor1_en_pin, 0)  # Deshabilitar motor 1
        pi.write(motor2_en_pin, 0)  # Deshabilitar motor 2

        pi.stop()
        print('Tiempo de funcionamiento de los motores completado.')

main()

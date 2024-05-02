import time
import pigpio

motor1_pwm_pin = 12
motor1_dir_pin = 24
motor1_en_pin = 22
motor2_pwm_pin = 13
motor2_dir_pin = 25
motor2_en_pin = 23
encoder1_pin = 18  # Pin del encoder del motor 1
encoder2_pin = 19  # Pin del encoder del motor 2
encoder_resolution = 32  # Resolución del encoder (counts por revolución)

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

def measure_rps(encoder_pin, encoder_resolution, elapsed_time):
    encoder_count = 0
    prev_encoder_state = pi.read(encoder_pin)
    start_time = time.time()

    while time.time() - start_time <= elapsed_time:
        encoder_state = pi.read(encoder_pin)
        if encoder_state != prev_encoder_state:
            encoder_count += 1
        prev_encoder_state = encoder_state
        time.sleep(0.01)  # Esperar un breve período para no saturar el bucle

    rps = (encoder_count / encoder_resolution) / elapsed_time
    return rps

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
            print('Velocidad motor 1:', motor1_speed)
            print('Velocidad motor 2:', motor2_speed)

            # Medir RPS durante 1 segundo para cada motor
            rps_motor1 = measure_rps(encoder1_pin, encoder_resolution, 1.0)
            rps_motor2 = measure_rps(encoder2_pin, encoder_resolution, 1.0)
            print('RPS Motor 1:', rps_motor1)
            print('RPS Motor 2:', rps_motor2)

            time.sleep(1.0)  # Esperar 1 segundo antes de leer la siguiente línea

        pi.set_PWM_dutycycle(motor1_pwm_pin, 0)
        pi.set_PWM_dutycycle(motor2_pwm_pin, 0)

        pi.write(motor1_en_pin, 0)  # Deshabilitar motor 1
        pi.write(motor2_en_pin, 0)  # Deshabilitar motor 2

        pi.stop()
        print('Tiempo de funcionamiento de los motores completado.')

main()

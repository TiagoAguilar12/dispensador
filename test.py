# -- coding: utf-8 --
import time
import pigpio

motor1_pwm_pin = 12
motor1_dir_pin = 24
motor1_en_pin = 22
motor1_encoder_pinA = 18  # Pin del encoder del motor 1 para la señal A
motor1_encoder_pinB = 19  # Pin del encoder del motor 1 para la señal B

motor2_pwm_pin = 13
motor2_dir_pin = 25
motor2_en_pin = 23
motor2_encoder_pinA = 20  # Pin del encoder del motor 2 para la señal A
motor2_encoder_pinB = 27  # Pin del encoder del motor 2 para la señal B

pi = pigpio.pi()

encoder_counts_per_rev = 64  # Número de cuentas por revolución del encoder
motor_resolution = 19  # Resolución del motor (19:1)

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
    motor1_encoder_count += 1

def encoder_callback2A(gpio, level, tick):
    global motor2_encoder_count
    motor2_encoder_count += 1
def encoder_callback1B(gpio, level, tick):
    global motor1_encoder_count
    motor1_encoder_count += 1

def encoder_callback2B(gpio, level, tick):
    global motor2_encoder_count
    motor2_encoder_count += 1

def calcular_rps(encoder_count, elapsed_time):
    return ((encoder_count / encoder_counts_per_rev)/ motor_resolution) / elapsed_time

def main():
    global motor1_encoder_count, motor2_encoder_count

    pi.write(motor1_en_pin, 1)  # Habilitar motor 1
    pi.write(motor2_en_pin, 1)  # Habilitar motor 2

    # Configurar callbacks para los flancos del encoder
    pi.callback(motor1_encoder_pinA, pigpio.FALLING_EDGE, encoder_callback1A)
    pi.callback(motor2_encoder_pinA, pigpio.FALLING_EDGE, encoder_callback2A)
    pi.callback(motor1_encoder_pinB, pigpio.FALLING_EDGE, encoder_callback1B)
    pi.callback(motor2_encoder_pinB, pigpio.FALLING_EDGE, encoder_callback2B)

    # Establecer velocidad al 100% para ambos motores
    control_motor(motor1_pwm_pin, motor1_dir_pin, 100, 'forward')
    control_motor(motor2_pwm_pin, motor2_dir_pin, 100, 'forward')

    start_time = time.time()

    while time.time() - start_time <= 10:  # Ejemplo: Medir durante 10 segundos
        # Calcular las RPS de cada motor
        elapsed_time = time.time() - start_time
        rps_motor1 = calcular_rps(motor1_encoder_count, elapsed_time)
        rps_motor2 = calcular_rps(motor2_encoder_count, elapsed_time)

        # Mostrar las RPS de ambos motores
        print('RPS motor 1:', rps_motor1)
        print('RPS motor 2:', rps_motor2)

        # Reiniciar contadores para la siguiente medición
        motor1_encoder_count = 0
        motor2_encoder_count = 0

        time.sleep(1)  # Esperar 1 segundo antes de medir nuevamente

    pi.set_PWM_dutycycle(motor1_pwm_pin, 0)
    pi.set_PWM_dutycycle(motor2_pwm_pin, 0)

    pi.write(motor1_en_pin, 0)  # Deshabilitar motor 1
    pi.write(motor2_en_pin, 0)  # Deshabilitar motor 2

    pi.stop()
    print('Medición de RPS completada.')

if __name__ == '__main__':
    main()

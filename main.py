import time
import pigpio

# Configuración de pines
motor1_pwm_pin = 12
motor1_dir_pin = 24
motor1_en_pin = 22
motor2_pwm_pin = 13
motor2_dir_pin = 25
motor2_en_pin = 23

# Inicialización de la conexión con pigpio
pi = pigpio.pi()

# Función para controlar la velocidad y dirección de los motores
def control_motor(motor_pwm, speed_percent, direction):
    if direction == 'forward':
        pi.write(motor1_dir_pin, 1)  # GPIO.HIGH
    elif direction == 'backward':
        pi.write(motor1_dir_pin, 0)  # GPIO.LOW
    else:
        raise ValueError("Dirección no válida. Usa 'forward' o 'backward'.")

    motor_pwm.start(speed_percent)

def main() -> int:
    # Configurar pines de habilitación (enable) de los motores
    pi.write(motor1_en_pin, 1)  # GPIO.HIGH
    pi.write(motor2_en_pin, 1)  # GPIO.HIGH

    motor1_pwm = pi.set_PWM_frequency(motor1_pwm_pin, 1000)  # Frecuencia de PWM: 1000 Hz
    motor2_pwm = pi.set_PWM_frequency(motor2_pwm_pin, 1000)

    control_motor(motor1_pwm, 50, 'forward')
    control_motor(motor2_pwm, 50, 'forward')

    time.sleep(2)
    motor1_pwm.stop()
    motor2_pwm.stop()
    pi.stop()
    print('Movimiento de los motores completado.')
    return 0

if __name__ == '__main__':
    print('Iniciando programa...')
    main()

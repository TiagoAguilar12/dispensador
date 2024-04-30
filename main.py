# -- coding: utf-8 --
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
def control_motor(pin_pwm, speed_percent, direction):
    duty_cycle = int(speed_percent * 255 / 100)  # Convertir el porcentaje de velocidad a ciclo de trabajo (0-255)
    pi.set_PWM_dutycycle(pin_pwm, duty_cycle)

    if direction == 'forward':
        pi.write(motor1_dir_pin, 1)  # GPIO.HIGH
    elif direction == 'backward':
        pi.write(motor1_dir_pin, 0)  # GPIO.LOW
    else:
        raise ValueError("Dirección no válida. Usa 'forward' o 'backward'.")

def main():
    # Configurar pines de habilitación (enable) de los motores
    pi.write(motor1_en_pin, 1)  # GPIO.HIGH
    pi.write(motor2_en_pin, 1)  # GPIO.HIGH

    control_motor(motor1_pwm_pin, 50, 'forward')
    control_motor(motor2_pwm_pin, 50, 'forward')

    time.sleep(5)
    pi.set_PWM_dutycycle(motor1_pwm_pin, 0)  # Detener motor 1
    pi.set_PWM_dutycycle(motor2_pwm_pin, 0)  # Detener motor 2

    pi.write(motor1_en_pin, 0)  # Deshabilitar motor 1
    pi.write(motor2_en_pin, 0)  # Deshabilitar motor 2

    pi.stop()
    print('Movimiento de los motores completado.')

if __name__ == '_main_':
    print('Iniciando programa...')
    main()
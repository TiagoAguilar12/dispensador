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

# Contadores de flancos de subida
flancos_motor1 = 0
flancos_motor2 = 0

# Función para controlar la velocidad y dirección de los motores
def control_motor(pin_pwm, speed_percent, direction):
    duty_cycle = int(speed_percent * 255 / 100)  # Convertir el porcentaje de velocidad a ciclo de trabajo (0-255)
    pi.set_PWM_dutycycle(pin_pwm, duty_cycle)

    if direction == 'forward':
        pi.write(motor1_dir_pin, 1)  # GPIO.HIGH
        pi.write(motor2_dir_pin, 1)  # GPIO.HIGH
    elif direction == 'backward':
        pi.write(motor1_dir_pin, 0)  # GPIO.LOW
        pi.write(motor2_dir_pin, 0)  # GPIO.LOW
    else:
        raise ValueError("Dirección no válida. Usa 'forward' o 'backward'.")

# Funciones de callback para contar flancos de subida
def motor1_callback(gpio, level, tick):
    global flancos_motor1
    flancos_motor1 += 1

def motor2_callback(gpio, level, tick):
    global flancos_motor2
    flancos_motor2 += 1

def calcular_rps(flancos, resolucion, relacion, tiempo):
    return flancos / (resolucion * relacion * tiempo)

def main():
    global flancos_motor1, flancos_motor2

    # Configurar pines de habilitación (enable) de los motores
    pi.write(motor1_en_pin, 1)  # GPIO.HIGH
    pi.write(motor2_en_pin, 1)  # GPIO.HIGH

    # Configurar callbacks para los flancos de subida
    pi.callback(motor1_pwm_pin, pigpio.RISING_EDGE, motor1_callback)
    pi.callback(motor2_pwm_pin, pigpio.RISING_EDGE, motor2_callback)

    control_motor(motor1_pwm_pin, 50, 'forward')
    control_motor(motor2_pwm_pin, 50, 'forward')

    time.sleep(10)  # Esperar 10 segundos para contar flancos

    print('Flancos de subida en 10 segundos:')
    print('Motor 1:', flancos_motor1)
    print('Motor 2:', flancos_motor2)

    rps_motor1 = calcular_rps(flancos_motor1/(32*19*10))
    rps_motor2 = calcular_rps(flancos_motor2/(32*19*10))

    print('RPS Motor 1:', rps_motor1)
    print('RPS Motor 2:', rps_motor2)

    pi.set_PWM_dutycycle(motor1_pwm_pin, 0)  # Detener motor 1
    pi.set_PWM_dutycycle(motor2_pwm_pin, 0)  # Detener motor 2

    pi.write(motor1_en_pin, 0)  # Deshabilitar motor 1
    pi.write(motor2_en_pin, 0)  # Deshabilitar motor 2

    pi.stop()
    print('Movimiento de los motores completado.')

if __name__ == '__main__':
    print('Iniciando programa...')
    main()

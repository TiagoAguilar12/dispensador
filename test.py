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
pinA = 17
pinB = 18
# Inicialización de la conexión con pigpio
pi = pigpio.pi()
pi.set_mode(pinA, pigpio.INPUT)

rpm_count = 0
rpm = 0
last_state = pi.read(pinA)

# Función para contar las RPM
def count_rpm(gpio, level, tick):
    global rpm_count
    global last_state

    state = pi.read(pinA)
    if state != last_state:
        rpm_count += 1

    last_state = state

cb = pi.callback(pinA, pigpio.RISING_EDGE, count_rpm)

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

    control_motor(motor1_pwm_pin, 100, 'forward')
    control_motor(motor2_pwm_pin, 100, 'forward')

    try:
        while True:
            start_time = time.time()
            time.sleep(1)  # Esperar 1 segundo
            end_time = time.time()

            time_elapsed = end_time - start_time
            rpm = (rpm_count / 20) / time_elapsed  # Calcular las RPM
            print("RPM: {:.2f}".format(rpm))

            rpm_count = 0
    except KeyboardInterrupt:
        cb.cancel()
        time.sleep(5)
        pi.set_PWM_dutycycle(motor1_pwm_pin, 0)  # Detener motor 1
        pi.set_PWM_dutycycle(motor2_pwm_pin, 0)  # Detener motor 2

        pi.write(motor1_en_pin, 0)  # Deshabilitar motor 1
        pi.write(motor2_en_pin, 0)  # Deshabilitar motor 2

        pi.stop()
        print('Movimiento de los motores completado.')

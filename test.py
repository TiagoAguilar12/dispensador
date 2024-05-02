# -- coding: utf-8 --

import time
import pigpio

encoder_pin = 18  # Pin del encoder
encoder_resolution = 64  # Resolución del encoder (counts por revolución)
motor_pwm_pin = 12  # Pin PWM del motor
motor_en_pin = 22  # Pin de habilitación del motor

pi = pigpio.pi()
pi.set_mode(encoder_pin, pigpio.INPUT)

rpm_count = 0  # Contador de pulsos del encoder
last_state = pi.read(encoder_pin)

# Función para contar las RPM
def count_rpm(gpio, level, tick):
    global rpm_count
    global last_state

    state = pi.read(encoder_pin)
    if state != last_state:
        rpm_count += 1

    last_state = state

cb = pi.callback(encoder_pin, pigpio.EITHER_EDGE, count_rpm)

# Función para controlar la velocidad del motor
def control_motor(speed_percent):
    duty_cycle = int(speed_percent * 255 / 100)  # Convertir porcentaje de velocidad a ciclo de trabajo
    pi.set_PWM_dutycycle(motor_pwm_pin, duty_cycle)

def main():
    global rpm_count

    pi.write(motor_en_pin, 1)  # Habilitar motor

    start_time = time.time()
    while time.time() - start_time <= 20:  # Medir RPM durante 20 segundos
        control_motor(50)  # Ejemplo: velocidad al 50%

        time.sleep(1)  # Esperar 1 segundo para medir RPM
        rpm = (rpm_count / encoder_resolution) / 1.0  # Calcular RPM (pulsos por segundo)
        print("RPM: {:.2f}".format(rpm))

        rpm_count = 0  # Reiniciar contador para la siguiente medición

    cb.cancel()
    pi.set_PWM_dutycycle(motor_pwm_pin, 0)  # Detener motor
    pi.write(motor_en_pin, 0)  # Deshabilitar motor
    pi.stop()
    print('Medición de RPM finalizada.')

if __name__ == "__main__":
    main()

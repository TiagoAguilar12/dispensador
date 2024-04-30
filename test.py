# -- coding: utf-8 --
import time
import pigpio

motor1_pwm_pin = 12
motor1_dir_pin = 24
motor1_en_pin = 22
motor2_pwm_pin = 13
motor2_dir_pin = 25
motor2_en_pin = 23
pinA = 17
pinB = 18

pi = pigpio.pi()
pi.set_mode(pinA, pigpio.INPUT)

global rpm_count  # Declarar rpm_count como global

rpm_count = 0
rpm = 0
last_state = pi.read(pinA)

# Función para contar las RPM
def count_rpmA(gpio, level, tick):
    global rpm_count
    global last_state

    state = pi.read(pinA)
    if state != last_state:
        rpm_count += 1

    last_state = state
def count_rpmB(gpio, level, tick):
    global rpm_count
    global last_state

    state = pi.read(pinB)
    if state != last_state:
        rpm_count += 1

    last_state = state

cb = pi.callback(pinA, pigpio.EITHER_EDGE, count_rpmA)
cb = pi.callback(pinB, pigpio.EITHER_EDGE, count_rpmB)

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

    # Configurar pines de habilitación (enable) de los motores
    pi.write(motor1_en_pin, 1)  # GPIO.HIGH
    pi.write(motor2_en_pin, 1)  # GPIO.HIGH

    control_motor(motor1_pwm_pin, motor1_dir_pin, 100, 'forward')
    control_motor(motor2_pwm_pin, motor2_dir_pin, 100, 'forward')
    start_time=time.time()

    while time.time()-start_time:
            start_time1 = time.time()
            time.sleep(1)  # Esperar 1 segundo
            end_time = time.time()
            time_elapsed = end_time - start_time1
            print("contflag: " + rpm_count + "time: " + time_elapsed)
            rpm = (rpm_count / 64) / time_elapsed  # Calcular las RPM
            print("RPM: {:.2f}".format(rpm))

            rpm_count = 0
    cb.cancel()
    time.sleep(5)
    pi.set_PWM_dutycycle(motor1_pwm_pin, 0)  # Detener motor 1
    pi.set_PWM_dutycycle(motor2_pwm_pin, 0)  # Detener motor 2

    pi.write(motor1_en_pin, 0)  # Deshabilitar motor 1
    pi.write(motor2_en_pin, 0)  # Deshabilitar motor 2

    pi.stop()
    print('Movimiento de los motores completado.')

main()
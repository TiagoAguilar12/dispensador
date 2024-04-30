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

global start_tick, last_tick, rpm  # Declarar variables globales para los ticks y la RPM

start_tick = 0
last_tick = 0
rpm = 0

# Función para contar las RPM
def count_rpm(gpio, level, tick):
    global start_tick, last_tick, rpm

    if start_tick == 0:
        start_tick = tick
    else:
        last_tick = tick
        rpm = (1000000.0 / (last_tick - start_tick)) * 60.0 / 16.0  # Calcular RPM
        start_tick = last_tick

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
    global rpm  # Declarar rpm como global dentro de main

    # Configurar pines de habilitación (enable) de los motores
    pi.write(motor1_en_pin, 1)  # GPIO.HIGH
    pi.write(motor2_en_pin, 1)  # GPIO.HIGH

    control_motor(motor1_pwm_pin, motor1_dir_pin, 100, 'forward')
    control_motor(motor2_pwm_pin, motor2_dir_pin, 100, 'forward')
    start_time = time.time()
        
    while time.time() - start_time <= 5:  # Ejemplo: Ejecutar durante 5 segundos
        time.sleep(1)  # Esperar 1 segundo
        print("RPM: {:.2f}".format(rpm))

    cb.cancel()
    time.sleep(5)
    pi.set_PWM_dutycycle(motor1_pwm_pin, 0)  # Detener motor 1
    pi.set_PWM_dutycycle(motor2_pwm_pin, 0)  # Detener motor 2

    pi.write(motor1_en_pin, 0)  # Deshabilitar motor 1
    pi.write(motor2_en_pin, 0)  # Deshabilitar motor 2

    pi.stop()
    print('Movimiento de los motores completado.')

main()

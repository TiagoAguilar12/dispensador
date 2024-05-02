# -- coding: utf-8 --
import time
import pigpio

motor1_pwm_pin = 12
motor1_dir_pin = 24
motor1_en_pin = 22
motor2_pwm_pin = 13
motor2_dir_pin = 25
motor2_en_pin = 23
pinA = 18


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


cb = pi.callback(pinA, pigpio.EITHER_EDGE, count_rpmA)

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


    control_motor(motor1_pwm_pin, motor1_dir_pin, 100, 'forward')
    start_time=time.time()

    while time.time()-start_time<= 20:
    
            print("contflag: " + rpm_count )
            rpm = ((rpm_count / 32) /19)  # Calcular las RPM
            print("RPM: {:.2f}".format(rpm))

            rpm_count = 0
            time.sleep(1)  # Esperar 1 segundo
    cb.cancel()
 
    pi.set_PWM_dutycycle(motor1_pwm_pin, 0)  # Detener motor 1

    pi.write(motor1_en_pin, 0)  # Deshabilitar motor 1


    pi.stop()
    print('Movimiento de los motores completado.')


main()
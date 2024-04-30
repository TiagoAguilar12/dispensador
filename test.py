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
pi.set_mode(pinB, pigpio.INPUT)

rpm_count_up = 0
rpm_count_down = 0
gearbox_count = 0
flancos_por_rev = 64  # Suma de flancos de subida y bajada para una revolución completa

def control_motor(pin_pwm, speed_percent, direction):
    duty_cycle = int(speed_percent * 255 / 100)
    pi.set_PWM_dutycycle(pin_pwm, duty_cycle)

    if direction == 'forward':
        pi.write(motor1_dir_pin, 1)
    elif direction == 'backward':
        pi.write(motor1_dir_pin, 0)
    else:
        raise ValueError("Dirección no válida. Usa 'forward' o 'backward'.")

def count_rpm(gpio, level, tick):
    global rpm_count_up, rpm_count_down, gearbox_count

    if gpio == pinA:
        rpm_count_up += 1
    elif gpio == pinB:
        rpm_count_down += 1

    total_flancos = rpm_count_up + rpm_count_down
    if total_flancos >= flancos_por_rev:
        rpm_count_up = 0
        rpm_count_down = 0
        global gearbox_count  # Indicar que se usará la variable global
        gearbox_count += 1
        print("Caja reductora contador:", gearbox_count)

cb_A = pi.callback(pinA, pigpio.EITHER_EDGE, count_rpm)
cb_B = pi.callback(pinB, pigpio.EITHER_EDGE, count_rpm)

def main():
    global gearbox_count  # Indicar que se usará la variable global

    pi.write(motor1_en_pin, 1)
    pi.write(motor2_en_pin, 1)

    control_motor(motor1_pwm_pin, 50, 'forward')
    control_motor(motor2_pwm_pin, 50, 'forward')

    try:
        while True:
            start_time = time.time()
            time.sleep(1)
            end_time = time.time()

            time_elapsed = end_time - start_time
            rpm = (gearbox_count / 19) / time_elapsed  # Calcular las RPM para una relación de 19:1
            print("RPM: {:.2f}".format(rpm))

            gearbox_count = 0
    except KeyboardInterrupt:
        cb_A.cancel()
        cb_B.cancel()
        time.sleep(5)
        pi.set_PWM_dutycycle(motor1_pwm_pin, 0)
        pi.set_PWM_dutycycle(motor2_pwm_pin, 0)

        pi.write(motor1_en_pin, 0)
        pi.write(motor2_en_pin, 0)

        pi.stop()
        print('Movimiento de los motores completado.')

if __name__ == '__main__':
    print('Iniciando programa...')
    main()

# -- coding: utf-8 --
import time
import pigpio

motor1_pwm_pin = 12
motor1_dir_pin = 24
motor1_en_pin = 22
motor1_enc_pinA = 17  # Pin del encoder para el motor 1
motor1_enc_pinB = 18  # Pin del encoder para el motor 1

motor2_pwm_pin = 13
motor2_dir_pin = 25
motor2_en_pin = 23
motor2_enc_pinA = 19  # Pin del encoder para el motor 2
motor2_enc_pinB = 20  # Pin del encoder para el motor 2

pi = pigpio.pi()

def control_motor(pin_pwm, pin_dir, speed_percent, direction):
    duty_cycle = int(speed_percent * 255 / 100)
    pi.set_PWM_dutycycle(pin_pwm, duty_cycle)

    if direction == 'forward':
        pi.write(pin_dir, 1)  # Establecer dirección hacia adelante
    elif direction == 'backward':
        pi.write(pin_dir, 0)  # Establecer dirección hacia atrás
    else:
        raise ValueError("Dirección no válida. Usa 'forward' o 'backward'.")

def read_encoder(encoder_pinA, encoder_pinB):
    return pi.read(encoder_pinA) ^ pi.read(encoder_pinB)

def main():
    pi.write(motor1_en_pin, 1)  # Habilitar motor 1
    pi.write(motor2_en_pin, 1)  # Habilitar motor 2


    start_time = time.time()
    start_ticks_motor1 = 0
    start_ticks_motor2 = 0
    ticks_per_rev = 64  # Número de flancos para una vuelta completa

    while time.time() - start_time <= 20:  # Ejemplo: Ejecutar durante 20 segundos
            

            control_motor(motor1_pwm_pin, 100,  'forward')
            control_motor(motor2_pwm_pin, 100,  'forward')

            # Contar flancos de subida y bajada para motor 1
            current_ticks_motor1 = read_encoder(motor1_enc_pinA, motor1_enc_pinB)
            ticks_diff_motor1 = current_ticks_motor1 - start_ticks_motor1
            if ticks_diff_motor1 >= ticks_per_rev:
                start_ticks_motor1 = current_ticks_motor1
                print("Motor 1: Vuelta completada")

            # Contar flancos de subida y bajada para motor 2
            current_ticks_motor2 = read_encoder(motor2_enc_pinA, motor2_enc_pinB)
            ticks_diff_motor2 = current_ticks_motor2 - start_ticks_motor2
            if ticks_diff_motor2 >= ticks_per_rev:
                start_ticks_motor2 = current_ticks_motor2
                print("Motor 2: Vuelta completada")

          

            

    pi.set_PWM_dutycycle(motor1_pwm_pin, 0)
    pi.set_PWM_dutycycle(motor2_pwm_pin, 0)

    pi.write(motor1_en_pin, 0)  # Deshabilitar motor 1
    pi.write(motor2_en_pin, 0)  # Deshabilitar motor 2

    pi.stop()
    print('Tiempo de funcionamiento de los motores completado.')

main()

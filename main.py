# -- coding: utf-8 --
import os
import time
import pigpio

motor1_pwm_pin = 12
motor1_dir_pin = 24
motor1_en_pin = 22
motor2_pwm_pin = 13
motor2_dir_pin = 25
motor2_en_pin = 23

pi = pigpio.pi()

def control_motor(pin_pwm, speed_percent, direction):
    duty_cycle = int(speed_percent * 255 / 100)
    pi.set_PWM_dutycycle(pin_pwm, duty_cycle)

    if direction == 'forward':
        pi.write(pin_pwm, 1)  # GPIO.HIGH
    elif direction == 'backward':
        pi.write(pin_pwm, 0)  # GPIO.LOW
    else:
        raise ValueError("Dirección no válida. Usa 'forward' o 'backward'.")

def main():
    pi.write(motor1_en_pin, 1)  # GPIO.HIGH
    pi.write(motor2_en_pin, 1)  # GPIO.HIGH

    file_path = '/home/santiago/Documents/dispensador/dispensador/Pbrs.txt'

    with open(file_path, 'r') as file:
        lines = file.readlines()
        total_lines = len(lines)
        current_line = 0

        start_time = time.time()
        while time.time() - start_time <= 10:  # Ejemplo: Ejecutar durante 60 segundos
            line = lines[current_line].strip()
            motor1_speed = int(line)
            motor2_speed = int(line)

            control_motor(motor1_pwm_pin, motor1_speed, 'forward')
            control_motor(motor2_pwm_pin, motor2_speed, 'forward')

            current_line = (current_line + 1) % total_lines  # Avanzar al siguiente valor circularmente

            time.sleep(0.5)  # Esperar 0.5 segundos antes de leer la siguiente línea

        pi.set_PWM_dutycycle(motor1_pwm_pin, 0)
        pi.set_PWM_dutycycle(motor2_pwm_pin, 0)

        pi.write(motor1_en_pin, 0)
        pi.write(motor2_en_pin, 0)

        pi.stop()
        print('Tiempo de funcionamiento de los motores completado.')

if __name__ == '__main__':
    print('Iniciando programa...')
    main()

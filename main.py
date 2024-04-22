import time
import RPi.GPIO as GPIO

# Configuración de pines
motor1_pwm_pin = 12
motor1_dir_pin = 16
motor2_pwm_pin = 18
motor2_dir_pin = 22

# Inicialización de la librería GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup([motor1_pwm_pin, motor1_dir_pin, motor2_pwm_pin, motor2_dir_pin], GPIO.OUT)

# Definir los objetos PWM para los motores
motor1_pwm = GPIO.PWM(motor1_pwm_pin, 1000)  # Frecuencia de PWM: 1000 Hz
motor2_pwm = GPIO.PWM(motor2_pwm_pin, 1000)

# Función para controlar la velocidad y dirección de los motores
def control_motor(motor_pwm, speed_percent, direction):
    if direction == 'forward':
        GPIO.output(motor1_dir_pin, GPIO.HIGH)
    elif direction == 'backward':
        GPIO.output(motor1_dir_pin, GPIO.LOW)
    else:
        raise ValueError("Dirección no válida. Usa 'forward' o 'backward'.")

    motor_pwm.start(speed_percent)

# Ejemplo de uso: mover motor 1 hacia adelante al 50% de velocidad
control_motor(motor1_pwm, 50, 'forward')

# Esperar un tiempo
time.sleep(2)

# Detener el motor 1
motor1_pwm.stop()

# Limpiar los pines GPIO al finalizar
GPIO.cleanup()

def main() -> int:
    print('hola')
    return 0

if __name__ == '__main__':
    sys.exit(main())  # next section explains the use of sys.exit
#!/usr/bin/env python3
import pigpio
from hx711 import HX711

pi = pigpio.pi()

try:
    hx = HX711(dout_pin=21, pd_sck_pin=20, pi=pi)

    hx.reset()
    hx.tare()

    reading = hx.get_raw_data_mean()
    if reading:
        print('Datos restados por compensación pero todavía no convertidos a unidades:', reading)
    else:
        print('Dato inválido', reading)

    input('Coloque un peso conocido en la balanza y luego presione Enter')
    reading = hx.get_data_mean()
    if reading:
        print('Valor medio de HX711 restado para compensar:', reading)
        known_weight_grams = input('Escriba cuántos gramos eran y presiona Enter: ')
        try:
            value = float(known_weight_grams)
            print(value, 'gramos')
        except ValueError:
            print('Se esperaba un número entero o flotante y se obtuvo:', known_weight_grams)

        ratio = reading / value
        hx.set_scale_ratio(ratio)
        print('Relación de peso establecida.')
    else:
        raise ValueError('No se puede calcular el valor medio. ERROR', reading)

    print("Ahora, leeré datos en un bucle infinito. Para salir presione 'CTRL + C'")
    input('Presione Enter para comenzar a leer')
    
    while True:
        print("El peso actual en gramos es de %.2f" % (hx.get_weight_mean(20)))

except (KeyboardInterrupt, SystemExit):
    print('Chau :)')

finally:
    pi.stop()
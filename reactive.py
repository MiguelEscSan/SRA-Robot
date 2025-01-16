#!/usr/bin/env python3

from ev3dev2.sensor import INPUT_1, INPUT_2
from ev3dev2.sensor.lego import UltrasonicSensor, ColorSensor
from ev3dev2.sound import Sound
from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C
from ev3dev2.led import Leds
from time import time
from movement import RobotMovement
import threading
import math

def find_obstacle(movement, ultrasonic_sensor, threshold=80, step_angle=5):
    """
    Esta función tiene que buscar un objeto basado en un umbral.

    Args:
        movement (RobotMovement): Objeto que maneja el movimiento del robot.
        ultrasonic_sensor (UltrasonicSensor): Sensor ultrasónico.
        threshold (int): Distancia mínima para detener la búsqueda (en cm).
        step_angle (int): Incremento del ángulo en cada paso (en grados).
    """    

    print("----------------------------- Searching for obstacle -----------------------------")
    min_distance = float('inf')
    best_angle = 0
    current_angle = 0
    distance_lock = threading.Lock()

    shared_distance = {"distance": None}

    def measure_distance():
        """Hilo que mide la distancia continuamente."""
        while not stop_flag.is_set():
            with distance_lock:
                shared_distance["distance"] = ultrasonic_sensor.distance_centimeters

    stop_flag = threading.Event()
    distance_thread = threading.Thread(target=measure_distance)
    distance_thread.start()

    try:
        # Crear la secuencia de ángulos: primero 60 a un lado, luego 60 al otro
        angles_to_check = list(range(0, -61, -step_angle)) + list(range(0, 61, step_angle))

        for angle in angles_to_check:
            movement.turn(angle - current_angle)
            current_angle = angle

            with distance_lock:
                distance = shared_distance["distance"]

            if distance is not None and distance < min_distance:
                min_distance = distance
                best_angle = current_angle

            if min_distance < threshold:
                break

        # Regresar al ángulo inicial
        movement.turn(-current_angle)
    finally:
        # Detener el hilo y esperar a que termine
        stop_flag.set()
        distance_thread.join()
    
    print("----------------------------- Obstacle found -----------------------------")

    return min_distance, best_angle

def follow_obstacle(movement, ultrasonic_sensor, object_distance, object_angle, threshold=4, step_distance=5, security_distance=15):
    """
    Sigue un objeto a una distancia específica.
    
    Args: 
        movement (RobotMovement): Objeto que maneja el movimiento del robot.
        ultrasonic_sensor (UltrasonicSensor): Sensor ultrasónico.
        object_distance (int): Distancia del objeto (en cm).
        object_angle (int): Ángulo del objeto (en grados).
        threshold (int): Distancia mínima para detener la búsqueda (en cm).
        step_distance (int): Distancia a avanzar o retroceder (en cm).
    """

    print("----------------------------- Following obstacle -----------------------------")
    movement.turn(object_angle)

    while True:
        movement.move(step_distance)
        current_distance = ultrasonic_sensor.distance_centimeters

        if current_distance < security_distance:
            print("Security distance reached")
            movement.stop()
            movement.move(-step_distance)
            # TODO se podría mirar que devolviera la distancia actual que mide el sensor pero puede fallar.
            current_distance += step_distance
            break

        # Si la distancia actual es mayor que la distancia a la que se encontraba el objeto más un umbral, se perdió el objeto y se debe encontrar de nuevo
        if current_distance > object_distance + threshold:
            # Se asume que lo va a encontrar en el nuevo barrido, si el sensor va bien
            object_distance, object_angle = find_obstacle(
                movement, 
                ultrasonic_sensor, 
                threshold=30, 
                step_angle=5
            )

            # Nos posicionamos mirando hacia él para seguirlo nuevamente
            movement.turn(object_angle)

    print("----------------------------- Obstacle followed -----------------------------")

    return current_distance

def avoid_first_obstacle(movement, object_distance, object_angle, turn_degrees=30, step_distance=15):
    """
    Evita un obstáculo girando hacia la izquierda, caminando una cierta distancia y girando de vuelta.
    
    Args:
        movement (RobotMovement): Objeto que maneja el movimiento del robot.
        object_distance (int): Distancia del obstáculo (en cm).
        object_angle (int): Ángulo del objeto (en grados).
        turn_degrees (int): Ángulo de giro en cada paso (en grados).
        step_distance (int): Distancia a avanzar o retroceder (en cm).
    """

    print("----------------------------- Avoiding first obstacle -----------------------------")
    movement.turn(-turn_degrees)
    movement.move(step_distance + object_distance)
    movement.turn(turn_degrees)

    return

def turn_till_find_second_obstacle(movement, ultrasonic_sensor, turn_angle=25, step_distance=10, threshold=30, first_object_distance=0):
    """
    Gira hasta encontrar un segundo obstáculo.

    Args: 
        movement (RobotMovement): Objeto que maneja el movimiento del robot.
        ultrasonic_sensor (UltrasonicSensor): Sensor ultrasónico.
        turn_angle (int): Ángulo de giro en cada paso (en grados).
        step_distance (int): Distancia a avanzar o retroceder (en cm).
    """

    print("----------------------------- Turning till find second obstacle -----------------------------")
    second_object_distance = 0

    while True:

        # TODO mirar como decirle que solo se fije en la siguiente distancia que sea superior a x distancia que representa la primera lata
        second_object_distance, second_object_angle = find_obstacle(movement, ultrasonic_sensor, threshold=30, step_angle=5)
        
        if second_object_distance > 80:
            movement.turn(turn_angle)
            movement.move(step_distance)
        else:
            break

    return second_object_distance, second_object_angle

def avoid_second_obstacle(movement, ultrasonic_sensor, object_distance, tolerance_distance, turn_direction, turn_angle=15):
    """
    Evita un obstáculo girando hasta que no haya obstáculo en frente o la distancia sea mayor a un umbral.
    Luego, camina hacia adelante una cierta distancia.
    
    Args:
        movement (RobotMovement): Objeto que maneja el movimiento del robot.
        ultrasonic_sensor (UltrasonicSensor): Sensor ultrasónico.
        object_distance (int): Distancia del obstáculo (en cm).
        tolerance_distance (int): Distancia añadida al objeto para considerar que no hay obstáculo (en cm).
        turn_direction (str): Dirección de giro ("left" o "right").
        turn_angle (int): Ángulo de giro en cada paso (en grados).
        step_distance (int): Distancia a avanzar o retroceder (en cm).
    """
    print("----------------------------- Avoiding second obstacle -----------------------------")
    turned_degrees = 0

    # Girar hasta perder el obstáculo o que la distancia del obstáculo sea mayor a un umbral
    turn_angle = -turn_angle if turn_direction == "left" else turn_angle
    while True:
        movement.turn(turn_angle)
        turned_degrees += turn_angle
        second_object_distance = ultrasonic_sensor.distance_centimeters
        
        print("Second object distance", second_object_distance)
        print("object_distance + tolerance_distance", object_distance + tolerance_distance)
        if second_object_distance > object_distance + tolerance_distance:
            break

    # Giramos un poco más para evitar el obstáculo
    # movement.turn(turn_angle)

    return second_object_distance, turned_degrees


def find_line(movement, color_sensor, turned_degrees, step_distance=15, threshold=10):
    """
    Mueve el robot hacia adelante mientras un hilo separado monitorea la intensidad de luz reflejada.
    Si se detecta un cambio significativo en la intensidad, el robot se detiene y gira.

    Args:
        movement (RobotMovement): Objeto que maneja el movimiento del robot.
        color_sensor (ColorSensor): Sensor de color.
        turned_degrees (int): Ángulo de giro acumulado que se corregirá tras encontrar la línea.
        step_distance (int): Distancia a avanzar o retroceder (en cm).
        threshold (float): Umbral como porcentaje (0.0 a 1.0) para detectar un cambio significativo.
    """
    def monitor_light():
        """
        Monitorea continuamente la intensidad de luz reflejada en el sensor y detiene el movimiento si detecta un cambio significativo.
        """
        nonlocal stop_movement
        print("Iniciando monitor de luz en un hilo separado...")

        # Configurar el sensor de luz al modo de intensidad reflejada
        color_sensor.mode = 'COL-REFLECT'

        while not stop_movement:
            # Leer intensidad actual
            current_intensity = color_sensor.reflected_light_intensity

            # Verificar si el cambio en la intensidad supera el umbral
            if current_intensity < threshold:
                print("Cambio detectado: ")
                stop_movement = True  # Señal para detener el movimiento
                break
            
    # Variable de control para detener el movimiento
    stop_movement = False

    # Iniciar el hilo de monitoreo
    monitor_thread = threading.Thread(target=monitor_light)
    monitor_thread.daemon = True  # Finalizar el hilo al terminar el programa principal
    monitor_thread.start()

    # Mover el robot continuamente hasta que se detecte un cambio
    while not stop_movement:
        movement.move(step_distance)

    # Detener el movimiento y realizar la corrección de giro
    movement.stop()
    movement.turn(-turned_degrees*2)
    print("Movimiento finalizado tras detectar linea.")


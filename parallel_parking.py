#!/usr/bin/env python3

from ev3dev2.sensor import INPUT_1, INPUT_2
from ev3dev2.sensor.lego import UltrasonicSensor, ColorSensor
from ev3dev2.sound import Sound
from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C
from time import time
from movement import RobotMovement
import threading
import time

def find_obstacle(movement, ultrasonic_sensor, threshold=0, step_angle=5, prefer_left=True):
    """
    Gira 180 grados (90 a un lado y 90 al otro) para buscar el obstáculo más cercano.
    Si encuentra un valor de distancia menor al threshold, se detiene y devuelve el ángulo exacto.

    Args:
        movement (RobotMovement): Objeto que maneja el movimiento del robot.
        ultrasonic_sensor (UltrasonicSensor): Sensor ultrasónico.
        threshold (float): Distancia mínima para detener la búsqueda (en cm).
        step_angle (int): Incremento del ángulo en cada paso (en grados).
        prefer_left (bool): Indica si debe comenzar girando hacia la izquierda.

    Returns:
        tuple: (distancia mínima, ángulo correspondiente).
    """
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
        # Crear la secuencia de ángulos: primero 90 a un lado, luego 90 al otro
        if prefer_left:
            angles_to_check = list(range(0, -91, -step_angle)) + list(range(-step_angle, 91, step_angle))
        else:
            angles_to_check = list(range(0, 91, step_angle)) + list(range(step_angle, -91, -step_angle))

        for angle in angles_to_check:
            movement.turn_to(angle - current_angle)
            current_angle = angle

            with distance_lock:
                distance = shared_distance["distance"]

            if distance is not None and distance < min_distance:
                min_distance = distance
                best_angle = current_angle

            if min_distance <= threshold:
                print(f"Threshold reached: {min_distance} cm at angle {best_angle}")
                break

        # Regresar al ángulo inicial
        movement.turn_to(-current_angle)
    finally:
        # Detener el hilo y esperar a que termine
        stop_flag.set()
        distance_thread.join()

    return min_distance, best_angle


def follow_obstacle(movement, ultrasonic_sensor, distance, tolerance_distance, min_obstacle_distance=80):
    """
    Sigue un obstáculo a una distancia constante. Si pierde el obstáculo,
    usa last_distance y last_degrees para determinar si se aleja y llama a
    find_obstacle para reencontrarlo con la preferencia de giro adecuada.

    Args:
        movement (RobotMovement): Objeto que maneja el movimiento del robot.
        ultrasonic_sensor (UltrasonicSensor): Sensor ultrasónico.
        distance (int): Distancia a avanzar o retroceder (en cm).
        tolerance_distance (int): Distancia de tolerancia máxima al obstáculo (en cm).
        min_obstacle_distance (int): Distancia mínima para considerar un obstáculo (en cm).

    Returns:
        tuple: (última distancia, ángulo actual, dirección inicial).
    """

    last_distance = ultrasonic_sensor.distance_centimeters
    current_degrees = 0  # Ángulo actual del robot 
    last_degrees = 0  # Último ángulo registrado cuando se encontró el obstáculo
    initial_direction = None  # Dirección inicial en la que se encontró el obstáculo

    while True:
        current_distance = ultrasonic_sensor.distance_centimeters

        if current_distance is None or current_distance > min_obstacle_distance or current_distance > last_distance:
            # Si no detecta el obstáculo o se está alejando
            print("Obstacle lost, searching...")

            # Determinar la preferencia de giro basado en los últimos ángulos registrados
            prefer_left = last_degrees > current_degrees

            # Llamar a find_obstacle con la preferencia calculada
            _, best_angle = find_obstacle(
                movement, ultrasonic_sensor, threshold=last_distance, prefer_left=prefer_left
            )

            # Establecer la dirección inicial si aún no se ha establecido
            if initial_direction is None and  current_degrees != 0:
                initial_direction = "left" if prefer_left else "right"
                
            # Moverse al ángulo donde se encontró el obstáculo
            movement.turn(best_angle)
            current_degrees += best_angle
                
        elif current_distance > tolerance_distance:
            # Si está lejos del obstáculo, avanzar
            print("Too far from obstacle, moving closer...")
            movement.move(distance)  # Avanzar 5 cm
            
        # TODO: Revisar si es necesario agregar el caso para volver hacia atrás porque puede generar un loop infinito
        elif current_distance < tolerance_distance:
            # Si está demasiado cerca del obstáculo, retroceder
            print("Too close to obstacle, moving back...")
            movement.move(-distance)  # Retroceder 5 cm
            
        else:
            break

        last_distance = current_distance
        last_degrees = current_degrees

    return last_distance, current_degrees, initial_direction
        
def avoid_obstacle(movement, ultrasonic_sensor, object_distance, tolerance_distance, turn_direction, turn_angle=15, step_distance=15):
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
    # Girar hasta perder el obstáculo o que la distancia del obstáculo sea mayor a un umbral
    while True:
        turn_angle = turn_angle if turn_direction == "left" else -turn_angle
        movement.turn(turn_angle)
        if ultrasonic_sensor.distance_centimeters > object_distance + tolerance_distance:
            break
        
    # Caminar hacia delante una cierta distancia
    movement.move(step_distance)


def find_line(movement, ultrasonic_sensor, color_sensor, object_distance, tolerance_distance, turn_direction, turn_angle=15, step_distance=15, line_color=1):
    """
    Esta función trata de girar hasta encontrar una distancia mayor al objeto y luego camina hacia adelante hasta encontrar una línea negra.
    Encontrada la línea, se detiene y gira hacia la dirección inicial de la que giró para encontrar la línea.

    Args:
        movement (RobotMovement): Objeto que maneja el movimiento del robot.
        ultrasonic_sensor (UltrasonicSensor): Sensor ultrasónico.
        color_sensor (ColorSensor): Sensor de color.
        object_distance (int): Distancia del obstáculo (en cm).
        tolerance_distance (int): Distancia añadida al objeto para considerar que no hay obstáculo (en cm).
        turn_direction (str): Dirección de giro ("left" o "right").
        turn_angle (int): Ángulo de giro en cada paso (en grados).
        step_distance (int): Distancia a avanzar o retroceder (en cm).
        line_color (int): Color de la línea a detectar. Negro es 1 y Blanco es 6.
    """
    # Grados que ha girado
    turned_degrees = 0
    
    # Gira hasta encontrar una distancia mayor al objeto
    while True:
        turn_angle = turn_angle if turn_direction == "left" else -turn_angle
        movement.turn(turn_angle)
        turned_degrees += turn_angle
        if ultrasonic_sensor.distance_centimeters > object_distance + tolerance_distance:
            break

    while True:
        # Move forward
        movement.move(step_distance)
        
        # Color function returns 1 if black line is detected
        if color_sensor.color() == line_color:
            movement.stop()
            movement.turn(-turned_degrees)
            break
    
def main():

    # Inicializar motor
    motor_left = LargeMotor(OUTPUT_C)
    motor_right = LargeMotor(OUTPUT_B)

    # Inicializar sensores
    ultrasonic_sensor = UltrasonicSensor(INPUT_1) 
    color_sensor = ColorSensor(INPUT_2)

    # Inicializar sonido
    sound = Sound()
    
    # Parámetros del robot
    WHEEL_DIAMETER = 5.5
    DISTANCE_BETWEEN_WHEELS = 20
    SPEED = 30
    TRANSMISSION_RATIO = 3

    # Clase que maneja el movimiento del robot
    movement = RobotMovement(
        motor_left,
        motor_right,
        WHEEL_DIAMETER,
        DISTANCE_BETWEEN_WHEELS,
        SPEED,
        TRANSMISSION_RATIO
    )
    
    print("Starting competition")
    sound.beep()
    start_time = time()
    
    # PRIMER PASO: Encontrar y seguir el obstaculo
    last_distance, current_degrees, initial_direction = follow_obstacle(
        movement=movement,
        ultrasonic_sensor=ultrasonic_sensor, 
        distance=5, 
        tolerance_distance=10, 
        min_obstacle_distance=80
    )
    
    # SEGUNDO PASO: Evitar la lata y encontrar la segunda lata. Una vez encontrada caminar hacia ella hasta la mitad de la distancia
    turn_direction = "left" if initial_direction == "right" else "right"
    avoid_obstacle(
        movement, 
        ultrasonic_sensor, 
        object_distance=last_distance, 
        tolerance_distance=5, 
        turn_direction=turn_direction
    )
    
    # TERCER PASO: Girar y caminar hasta detectar la linea negra. Luego, girar hacia la dirección inicial
    find_line(
        movement, 
        ultrasonic_sensor, 
        color_sensor, 
        object_distance=last_distance, 
        tolerance_distance=5, 
        turn_direction=turn_direction
    )

    end_time = time()
    sound.beep()
    print(f"Total time: {end_time - start_time:.2f} seconds")

if __name__ == '__main__':
    main()
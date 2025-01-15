#!/usr/bin/env python3

from ev3dev2.sensor import INPUT_1, INPUT_2
from ev3dev2.sensor.lego import UltrasonicSensor, ColorSensor
from ev3dev2.sound import Sound
from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C
from time import time
from movement import RobotMovement
import threading
import math

def find_obstacle(movement, ultrasonic_sensor, threshold=0, step_angle=5, prefer_left=True, up_threshold=0):
    """
    Gira 120 grados (60 a un lado y 60 al otro) para buscar el obstáculo más cercano.
    Si encuentra un valor de distancia menor al threshold, se detiene y devuelve el ángulo exacto.

    Args:
        movement (RobotMovement): Objeto que maneja el movimiento del robot.
        ultrasonic_sensor (UltrasonicSensor): Sensor ultrasónico.
        threshold (float): Distancia mínima para detener la búsqueda (en cm).
        step_angle (int): Incremento del ángulo en cada paso (en grados).
        prefer_left (bool): Indica si debe comenzar girando hacia la izquierda.
        up_threshold (float): Umbral a partir del cual se considera que la distancia es valida.

    Returns:
        tuple: (distancia mínima, ángulo correspondiente).
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
        # Crear la secuencia de ángulos: primero 45 a un lado, luego 45 al otro
        if prefer_left:
            angles_to_check = list(range(0, -46, -step_angle)) + list(range(0, 46, step_angle))
            # angles_to_check = list(range(0, -91, -step_angle))
        else:
            angles_to_check = list(range(0, 46, step_angle)) + list(range(-0, -46, -step_angle))
            # angles_to_check = list(range(0, 91, step_angle))

        for angle in angles_to_check:
            movement.turn(angle - current_angle)
            current_angle = angle

            with distance_lock:
                distance = shared_distance["distance"]

            if distance is not None and distance < min_distance and distance > up_threshold:
                min_distance = distance
                best_angle = current_angle

            if min_distance < threshold:
                # print(f"Threshold reached: {min_distance} cm at angle {best_angle}")
                break

        # Regresar al ángulo inicial
        movement.turn(-current_angle)
    finally:
        # Detener el hilo y esperar a que termine
        stop_flag.set()
        distance_thread.join()
    
    print("----------------------------- Obstacle found -----------------------------")

    return min_distance, best_angle

# def follow_obstacle(movement, ultrasonic_sensor, distance, tolerance_distance, min_obstacle_distance=80):
#     """
#     Sigue un obstáculo a una distancia constante. Si pierde el obstáculo,
#     usa last_distance y last_degrees para determinar si se aleja y llama a
#     find_obstacle para reencontrarlo con la preferencia de giro adecuada.

#     Args:
#         movement (RobotMovement): Objeto que maneja el movimiento del robot.
#         ultrasonic_sensor (UltrasonicSensor): Sensor ultrasónico.
#         distance (int): Distancia a avanzar o retroceder (en cm).
#         tolerance_distance (int): Distancia de tolerancia máxima al obstáculo (en cm).
#         min_obstacle_distance (int): Distancia mínima para considerar un obstáculo (en cm).

#     Returns:
#         tuple: (última distancia, ángulo actual, dirección inicial).
#     """

#     best_distance = 0
#     current_degrees = 0  # Ángulo actual del robot 
#     last_degrees = 0  # Último ángulo registrado cuando se encontró el obstáculo
#     initial_direction = None  # Dirección inicial en la que se encontró el obstáculo
#     threshold = 10

#     while True:
#         current_distance = ultrasonic_sensor.distance_centimeters
#         print("Current distance: ", current_distance)
#         print("Best distance", best_distance)

#         if current_distance is None or current_distance > min_obstacle_distance or math.trunc(current_distance) > math.trunc(best_distance) + threshold:
#             # Si no detecta el obstáculo o se está alejando
#             print("Obstacle lost, searching...")

#             # Determinar la preferencia de giro basado en los últimos ángulos registrados
#             prefer_left = last_degrees > current_degrees

#             # Llamar a find_obstacle con la preferencia calculada
#             _, best_angle = find_obstacle(
#                 movement, ultrasonic_sensor, threshold=best_distance, prefer_left=prefer_left
#             )

#             # Establecer la dirección inicial si aún no se ha establecido
#             if initial_direction is None and current_degrees != 0:                
#                 initial_direction = "left" if prefer_left else "right"
#                 print("initial direction = ", initial_direction)
                
#             # Moverse al ángulo donde se encontró el obstáculo
#             movement.turn(best_angle)
#             current_degrees += best_angle
                
#         elif current_distance > tolerance_distance:
#             # Si está lejos del obstáculo, avanzar
#             print("Too far from obstacle, moving closer...")
#             movement.move(distance)  # Avanzar 5 cm
            
#         # TODO: Revisar si es necesario agregar el caso para volver hacia atrás porque puede generar un loop infinito
#         elif current_distance <= tolerance_distance:
#             # Si está demasiado cerca del obstáculo, retroceder
#             print("Too close to obstacle, moving back...")
#             movement.move(-distance)  # Retroceder 5 cm
#             break

#         if current_distance < best_distance or best_distance == 0:
#             best_distance = current_distance
#         last_degrees = current_degrees

#     return best_distance, current_degrees, initial_direction

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

    best_distance = 0
    current_degrees = 0  # Ángulo actual del robot 
    last_degrees = 0  # Último ángulo registrado cuando se encontró el obstáculo
    threshold = 10

    while True:
        current_distance = ultrasonic_sensor.distance_centimeters
        print("Current distance: ", current_distance)
        print("Best distance", best_distance)

        if current_distance is None or current_distance > min_obstacle_distance or math.trunc(current_distance) > math.trunc(best_distance) + threshold:
            # Si no detecta el obstáculo o se está alejando
            print("Obstacle lost, searching...")

            # Determinar la preferencia de giro basado en los últimos ángulos registrados
            prefer_left = last_degrees > current_degrees

            # Llamar a find_obstacle con la preferencia calculada
            _, best_angle = find_obstacle(
                movement, ultrasonic_sensor, threshold=best_distance, prefer_left=prefer_left
            )
                
            # Moverse al ángulo donde se encontró el obstáculo
            movement.turn(best_angle)
            current_degrees += best_angle
                
        elif current_distance > tolerance_distance:
            # Si está lejos del obstáculo, avanzar
            print("Too far from obstacle, moving closer...")
            movement.move(distance)  # Avanzar 5 cm
            
        # TODO: Revisar si es necesario agregar el caso para volver hacia atrás porque puede generar un loop infinito
        elif current_distance <= tolerance_distance:
            # Si está demasiado cerca del obstáculo, retroceder
            print("Too close to obstacle, moving back...")
            movement.move(-distance)  # Retroceder 5 cm
            break

        if current_distance < best_distance or best_distance == 0:
            best_distance = current_distance
        last_degrees = current_degrees

    return best_distance, current_degrees
        
# def avoid_obstacle(movement, ultrasonic_sensor, object_distance, tolerance_distance, turn_direction, turn_angle=20, step_distance=15):
#     """
#     Evita un obstáculo girando hasta que no haya obstáculo en frente o la distancia sea mayor a un umbral.
#     Luego, camina hacia adelante una cierta distancia.
    
#     Args:
#         movement (RobotMovement): Objeto que maneja el movimiento del robot.
#         ultrasonic_sensor (UltrasonicSensor): Sensor ultrasónico.
#         object_distance (int): Distancia del obstáculo (en cm).
#         tolerance_distance (int): Distancia añadida al objeto para considerar que no hay obstáculo (en cm).
#         turn_direction (str): Dirección de giro ("left" o "right").
#         turn_angle (int): Ángulo de giro en cada paso (en grados).
#         step_distance (int): Distancia a avanzar o retroceder (en cm).
#     """
#     print("----------------------------- Avoiding obstacle -----------------------------")
    
#     # Si desconocemos a donde girar busca el segundo obstáculo
#     # TODO: Lo hace en ambas latas, sólo puede hacerlo en la primera lata. second_obstacle_degrees tiene que ser < 0 en la primera lata
#     if turn_direction is None:
#         print("Tiene que encontrar el obstaculo porque no hay turn_direction")
#         _, second_obstacle_degrees = find_obstacle(
#             movement, 
#             ultrasonic_sensor, 
#             up_threshold = object_distance*1.1
#         )
#         turn_direction = "left" if second_obstacle_degrees < 0 else "right"
    
    
#     # Girar hasta perder el obstáculo o que la distancia del obstáculo sea mayor a un umbral
#     turn_angle = turn_angle if turn_direction == "left" else -turn_angle
#     while True:
#         movement.turn(turn_angle)
#         second_object_distance = ultrasonic_sensor.distance_centimeters
#         print("Second object distance", second_object_distance)
#         print("object_distance + tolerance_distance", object_distance + tolerance_distance)
#         if second_object_distance > object_distance + tolerance_distance:
#             break
        
#     # Gira de más para asegurarse que no choque
#     movement.turn(turn_angle*2)

#     # Caminar hacia delante una cierta distancia
#     movement.move(step_distance + object_distance)

#     # Giramos de vuelta para poder encontrar el segundo obstáculo
#     movement.turn(-turn_angle*3)
    
#     return turn_direction

def avoid_obstacle(movement, ultrasonic_sensor, object_distance, tolerance_distance, turn_angle=20, step_distance=15):
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
    print("----------------------------- Avoiding obstacle -----------------------------")
    
    # Si desconocemos a donde girar busca el segundo obstáculo
    _, second_obstacle_degrees = find_obstacle(
        movement, 
        ultrasonic_sensor, 
        up_threshold = object_distance*1.1
    )
    turn_direction = "left" if second_obstacle_degrees < 0 else "right"
    
    print("turn_direction: ", turn_direction)
    
    
    # Girar hasta perder el obstáculo o que la distancia del obstáculo sea mayor a un umbral
    turn_angle = turn_angle if turn_direction == "left" else -turn_angle
    while True:
        movement.turn(turn_angle)
        second_object_distance = ultrasonic_sensor.distance_centimeters
        print("Second object distance", second_object_distance)
        print("object_distance + tolerance_distance", object_distance + tolerance_distance)
        if second_object_distance > object_distance + tolerance_distance:
            break
        
    # Gira de más para asegurarse que no choque
    movement.turn(turn_angle*2)

    # Caminar hacia delante una cierta distancia
    movement.move(step_distance + object_distance)

    # Giramos de vuelta para poder encontrar el segundo obstáculo
    movement.turn(-turn_angle*3)
    
    return turn_direction

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
    DISTANCE_BETWEEN_WHEELS = 19.79
    SPEED = 20
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
    # last_distance, current_degrees, initial_direction = follow_obstacle(
    #     movement=movement,
    #     ultrasonic_sensor=ultrasonic_sensor, 
    #     distance=5, 
    #     tolerance_distance=25, 
    #     min_obstacle_distance=150
    # )
    
    last_distance, current_degrees = follow_obstacle(
        movement=movement,
        ultrasonic_sensor=ultrasonic_sensor, 
        distance=5, 
        tolerance_distance=25, 
        min_obstacle_distance=150
    )
    
    
    # TODO: Revisar esta función, ya que asume que siempre que se evite un obstáculo, la siguiente distancia será la del segundo obstáculo
    # SEGUNDO PASO: Evitar la lata y encontrar la segunda lata. Una vez encontrada caminar hacia ella hasta la mitad de la distancia
    # turn_direction = "left" if initial_direction == "right" else "right" if initial_direction == "left" else None
    # turn_direction = avoid_obstacle(
    #     movement, 
    #     ultrasonic_sensor, 
    #     object_distance=last_distance, 
    #     tolerance_distance=5, 
    #     turn_direction=turn_direction,
    #     step_distance=10,
    #     turn_angle=10
    # )
    turn_direction = avoid_obstacle(
        movement, 
        ultrasonic_sensor, 
        object_distance=last_distance, 
        tolerance_distance=5, 
        step_distance=10,
        turn_angle=10
    )

    # TERCER PASO:Encontrar el segundo obstáculo
    last_distance, current_degrees, _ = follow_obstacle(
        movement=movement,
        ultrasonic_sensor=ultrasonic_sensor, 
        distance=5, 
        tolerance_distance=25, 
        min_obstacle_distance=80
    )

    # print("Second obstacle distance: ", second_obstacle_distance)
    # print("Current degrees: ", current_degrees)

    # CUARTO PASO: Girar para esquivar el segundo obstáculo
    _, current_degrees = avoid_second_obstacle(
        movement, 
        ultrasonic_sensor, 
        object_distance=last_distance, 
        tolerance_distance=5, 
        turn_direction=turn_direction
    )

    # QUINTO PASO: Caminar hasta encontrar la línea y girar hacia la dirección inicial
    find_line(
        movement, 
        color_sensor, 
        turned_degrees=current_degrees,
        step_distance=10,
        threshold=15
    )

    end_time = time()
    sound.beep()
    print("Total time: {:.2f} seconds".format(end_time - start_time))   


if __name__ == '__main__':
    main()
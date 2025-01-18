import threading
import math

def find_obstacle(movement, ultrasonic_sensor, down_threshold=10, step_angle=5, max_angle=60, only_left=False):
    """
    Esta función busca un objeto basado en un umbral y almacena las distancias medidas en cada ángulo.

    Args:
        movement (RobotMovement): Objeto que maneja el movimiento del robot.
        ultrasonic_sensor (UltrasonicSensor): Sensor ultrasónico.
        down_threshold (int): Distancia mínima para considerar un objeto (en cm).
        up_threshold (int): Distancia máxima para considerar un objeto (en cm).
        step_angle (int): Incremento del ángulo en cada paso (en grados).
    """
    
    print("----------------------------- Searching for obstacle -----------------------------")
    min_distance = float('inf')
    best_angle = 0
    current_angle = 0

    # Crear la secuencia de ángulos: primero de 0 a -max_angle, luego de 0 a max_angle
    if only_left:
        angles_to_check = list(range(0, -max_angle, -step_angle))
    else:
        angles_to_check = list(range(0, -max_angle, -step_angle)) + list(range(0, max_angle, step_angle))

    for angle in angles_to_check:
        movement.turn(angle - current_angle)
        current_angle = angle
        
        # Medir la distancia directamente
        distance = ultrasonic_sensor.distance_centimeters

        print("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")
        print("Current angle, distance : ", current_angle, distance)
        print("Min_distance: ", min_distance, "| Down_threshold: ", down_threshold)
        print("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")

        
        if distance is not None and distance < down_threshold: 
            min_distance = distance
            best_angle = current_angle

    # Regresar al ángulo inicial
    movement.turn(-current_angle)

    print("Return in find_obstacle: ", min_distance, "|" ,best_angle)
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

    best_distance = 0
    current_degrees = 0  # Ángulo actual del robot 
    threshold = 10

    while True:
        current_distance = ultrasonic_sensor.distance_centimeters
        print("Current distance: ", current_distance)
        print("Best distance", best_distance)

        if current_distance is None or current_distance > min_obstacle_distance or math.trunc(current_distance) > math.trunc(best_distance) + threshold:
            # Si no detecta el obstáculo o se está alejando
            print("Obstacle lost, searching...")
            
            # Moverse un poco antes de buscar el obstáculo

            #TODO mirar si la distancia devuelta deberia ser current_distance
            
            # Llamar a find_obstacle con la preferencia calculada
            _, best_angle = find_obstacle(
                movement, ultrasonic_sensor, down_threshold=best_distance + threshold   
            )

            # Moverse al ángulo donde se encontró el obstáculo
            movement.turn(best_angle)
            current_degrees += best_angle
                
        elif current_distance > tolerance_distance:
            # Si está lejos del obstáculo, avanzar
            print("Too far from obstacle, moving closer...")
            movement.move(distance)  # Avanzar 10 cm
            
        elif current_distance <= tolerance_distance:
            # Si está demasiado cerca del obstáculo, retroceder
            print("Too close to obstacle, moving back...")
            movement.move(-distance)  # Retroceder 5 cm
            break

        if current_distance < best_distance or best_distance == 0:
            best_distance = current_distance

    return best_distance, current_degrees
        
def avoid_obstacle(movement, ultrasonic_sensor, object_distance, tolerance_distance, turn_direction, turn_angle=20, step_distance=15, first_obstacle=True):
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
    turned_degrees = 0
    
    if turn_direction == "left":
        turn_angle = -turn_angle
        
    
    while True:
        turned_degrees += turn_angle
        movement.turn(turn_angle)
        distance = ultrasonic_sensor.distance_centimeters
        print("Distance: ", distance)
        print("object_distance + tolerance_distance", object_distance + tolerance_distance)
        if distance > object_distance + tolerance_distance:
            break
        
    # Gira de más para asegurarse que no choque
    turned_degrees += turn_angle
    movement.turn(turn_angle)

    if first_obstacle:
        # Caminar hacia delante una cierta distancia
        movement.move(step_distance + object_distance)

        # Giramos de vuelta para poder encontrar el segundo obstáculo
        turned_degrees -= turn_angle
        movement.turn(-turn_angle)
    
    return turned_degrees

def move_untill_found_obstacle(movement, ultrasonic_sensor, distance, degrees, min_obstacle_distance=80, left=False):
    """
    Esta función busca un objeto basado en un umbral y almacena las distancias medidas en cada ángulo.

    Args:
        movement (RobotMovement): Objeto que maneja el movimiento del robot.
        ultrasonic_sensor (UltrasonicSensor): Sensor ultrasónico.
        distance (int): Distancia a avanzar o retroceder (en cm).
        tolerance_distance (int): Distancia de tolerancia máxima al obstáculo (en cm).
        min_obstacle_distance (int): Distancia mínima para considerar un objeto (en cm).
    """
    
    print("----------------------------- Searching for obstacle -----------------------------")
    
    while True:
        current_distance = ultrasonic_sensor.distance_centimeters
        
        if current_distance is None or current_distance > min_obstacle_distance:
            # Girar un poco antes de buscar el obstáculo y caminar un poco
            if left:
                movement.turn(-degrees)
            else:
                movement.turn(degrees)   
            movement.move(distance)
            
        elif current_distance <= min_obstacle_distance:
            # Si está demasiado cerca del obstáculo, retroceder
            movement.move(-distance)
            break
        
    return
            

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
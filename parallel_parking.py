#!/usr/bin/env python3

import os
import uuid
from ev3dev2.sensor import INPUT_1, INPUT_2
from ev3dev2.sensor.lego import UltrasonicSensor, ColorSensor
from ev3dev2.sound import Sound
from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
from math import pi
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
    current_angle = 0  # Para rastrear el ángulo actual
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
            movement.turn_to(angle - current_angle)  # Girar al ángulo deseado
            current_angle = angle  # Actualizar el ángulo actual

            with distance_lock:
                distance = shared_distance["distance"]

            if distance is not None and distance < min_distance:
                min_distance = distance
                best_angle = current_angle

            # Si la distancia es menor al umbral, detener la búsqueda
            if min_distance <= threshold:
                print(f"Threshold reached: {min_distance} cm at angle {best_angle}")
                break

        # Regresar al ángulo inicial
        movement.turn_to(-current_angle)
    finally:
        stop_flag.set()
        distance_thread.join()

    return min_distance, best_angle


def follow_obstacle(movement, ultrasonic_sensor, distance, tolerance_distance):
    """
    Sigue un obstáculo a una distancia constante. Si pierde el obstáculo,
    usa last_distance y last_degrees para determinar si se aleja y llama a
    find_obstacle para reencontrarlo con la preferencia de giro adecuada.

    Args:
        movement (RobotMovement): Objeto que maneja el movimiento del robot.
        ultrasonic_sensor (UltrasonicSensor): Sensor ultrasónico.
        distance (int): Distancia a avanzar o retroceder (en cm).
        tolerance_distance (int): Distancia de tolerancia para avanzar o retroceder (en cm).
        
    """

    last_distance = ultrasonic_sensor.distance_centimeters
    current_degrees = 0  # Ángulo actual del robot
    last_degrees = 0  # Último ángulo registrado cuando se encontró el obstáculo

    while True:
        current_distance = ultrasonic_sensor.distance_centimeters

        if current_distance is None or current_distance > 100 or (last_distance and current_distance > last_distance):
            # Si no detecta el obstáculo o se está alejando
            print("Obstacle lost, searching...")

            # Determinar la preferencia de giro basado en los últimos ángulos registrados
            prefer_left = last_degrees > current_degrees

            # Llamar a find_obstacle con la preferencia calculada
            _, best_angle = find_obstacle(
                movement, ultrasonic_sensor, threshold=last_distance, prefer_left=prefer_left
            )

            # Moverse al ángulo donde se encontró el obstáculo
            movement.turn(best_angle)
            current_degrees += best_angle
        elif current_distance > tolerance_distance:
            # Si está lejos del obstáculo, avanzar
            print("Too far from obstacle, moving closer...")
            movement.move(distance)  # Avanzar 5 cm
        elif current_distance < tolerance_distance:
            # Si está demasiado cerca del obstáculo, retroceder
            print("Too close to obstacle, moving back...")
            movement.move(-distance)  # Retroceder 5 cm
        else:
            break

        # Actualizar los registros de distancia y ángulo
        last_distance = current_distance
        last_degrees = current_degrees

def find_line(movement, color_sensor):
    """
    This function continuously checks for a black line using the color sensor.
    When a black line is detected, it stops the robot.

    Args:
        movement (RobotMovement): Object handling the robot's movement.
        color_sensor (ColorSensor): Color sensor.
    """

    while True:
        # Color function returns 1 if black line is detected
        if color_sensor.color() == 1:
            movement.stop()
            movement.turn(90)
            print("Black line detected. Stopping robot.")
            return True
    
def main():

    # Initialize motors
    motor_left = LargeMotor(OUTPUT_C)
    motor_right = LargeMotor(OUTPUT_B)

    # Initialize sensors
    ultrasonic_sensor = UltrasonicSensor(INPUT_1) 
    color_sensor = ColorSensor(INPUT_2)

    # Initialize sound
    sound = Sound()
    
    # Robot parameters
    WHEEL_DIAMETER = 5.5
    DISTANCE_BETWEEN_WHEELS = 20
    SPEED = 30
    TRANSMISSION_RATIO = 3

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

    # Iniciar hilo para encontrar la linea negra
    line_thread = threading.Thread(target=find_line, args=(movement, color_sensor))
    line_thread.daemon = True 
    line_thread.start()
    
    # PRIMER PASO: Encontrar y seguir el obstaculo
    initial_distance = ultrasonic_sensor.distance_centimeters
    follow_obstacle(movement=movement,ultrasonic_sensor=ultrasonic_sensor, distance=5, tolerance_distance=10)
    
    # SEGUNDO PASO: Evitar la lata y encontrar la segunda lata. Una vez encontrada caminar hacia ella hasta la mitad de la distancia
    
    
    # TERCER PASO: Girar y caminar hasta detectar la linea negra
    

    end_time = time()
    sound.beep()
    print(f"Total time: {end_time - start_time:.2f} seconds")

if __name__ == '__main__':
    main()
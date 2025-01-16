#!/usr/bin/env python3

from ev3dev2.sensor import INPUT_1, INPUT_2
from ev3dev2.sensor.lego import UltrasonicSensor, ColorSensor
from ev3dev2.sound import Sound
from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C
from time import time
from movement import RobotMovement
from reactive import follow_obstacle, avoid_obstacle, avoid_second_obstacle, find_line


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
    last_distance, current_degrees, initial_direction = follow_obstacle(
        movement=movement,
        ultrasonic_sensor=ultrasonic_sensor, 
        distance=5, 
        tolerance_distance=25, 
        min_obstacle_distance=150
    )
    
    # TODO: Revisar esta función, ya que asume que siempre que se evite un obstáculo, la siguiente distancia será la del segundo obstáculo
    # SEGUNDO PASO: Evitar la lata y encontrar la segunda lata. Una vez encontrada caminar hacia ella hasta la mitad de la distancia
    turn_direction = "left" if initial_direction == "right" else "right" if initial_direction == "left" else None
    turn_direction = avoid_obstacle(
        movement, 
        ultrasonic_sensor, 
        object_distance=last_distance, 
        tolerance_distance=5, 
        turn_direction=turn_direction,
        step_distance=10,
        turn_angle=10
    )
    
    
    # TERCER PASO:Encontrar el segundo obstáculo y seguirlo
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

from ev3dev2.sensor import INPUT_1, INPUT_2
from ev3dev2.sensor.lego import UltrasonicSensor, ColorSensor
from ev3dev2.sound import Sound
from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C
from ev3dev2.led import Leds
from time import time
from movement import RobotMovement
import threading
import math
from reactive import *

def main():

    # Inicializar motor
    motor_left = LargeMotor(OUTPUT_C)
    motor_right = LargeMotor(OUTPUT_B)

    # Inicializar sensores
    ultrasonic_sensor = UltrasonicSensor(INPUT_1) 
    color_sensor = ColorSensor(INPUT_2)

    # Inicializar sonido Y leds
    sound = Sound()
    leds = Leds()
    
    # Parámetros del robot
    WHEEL_DIAMETER = 5.5
    DISTANCE_BETWEEN_WHEELS = 19.79
    SPEED = 10
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

    leds.set_color('LEFT', 'RED')
    leds.set_color('RIGHT', 'RED')

    start_time = time()
    
    #---------------------------------------------------------------------------------------------------

    # Primer paso: Encontrar el primer obstáculo y seguirlo hasta una distancia de seguridad
    first_obstacle_distance, first_obstacle_angle = find_obstacle(
        movement,
        ultrasonic_sensor, 
        threshold=30, 
        step_angle=5
    )

    follow_obstacle(
        movement,
        ultrasonic_sensor,
        first_obstacle_distance,
        first_obstacle_angle,
        threshold=4
    )

    # Segundo paso: Esquivar el primer obstáculo hacia la izquierda, caminar distancia umbral + distancia obstáculo y girar ciertos grados de vuelta
    avoid_first_obstacle(
        movement,
        ultrasonic_sensor,
        first_obstacle_distance,
        first_obstacle_angle,
        threshold_distance=20,
        turn_degrees=30
    )
    
    # Tercer paso: Girar y caminar mientras no se haya encontrado el segundo obstáculo hacia la derecha. Girar poco y caminar poco.
    second_object_distance, second_object_angle = turn_till_find_second_obstacle(
        movement,
        ultrasonic_sensor,
        turn_angle=25,
        step_distance=10,
        threshold=30,
        first_object_distance=first_obstacle_distance
    )
    
    # Cuarto paso: Acercarse hasta el segundo obstáculo hasta una distancia de seguridad
    follow_obstacle(
        movement,
        ultrasonic_sensor,
        second_object_distance,
        second_object_angle,
        threshold=20,
        step_distance=5,
        security_distance=15
    )
    
    # Quinto paso: Girar hasta dejar de ver el segundo obstáculo
    _, current_degrees = avoid_second_obstacle(
        movement, 
        ultrasonic_sensor, 
        object_distance=second_object_distance, 
        tolerance_distance=5, 
        turn_direction=second_object_angle
    )
    
    # Sexto paso: Caminar hasta ver la línea, detenerse y colocarse correctamente
    find_line(
        movement, 
        color_sensor, 
        turned_degrees=current_degrees,
        step_distance=10,
        threshold=15
    )
    
    

    
    

    

    #---------------------------------------------------------------------------------------------------

    end_time = time()
    sound.beep()

    leds.set_color('LEFT', 'GREEN')
    leds.set_color('RIGHT', 'GREEN')

    print("Total time: {:.2f} seconds".format(end_time - start_time))   


if __name__ == '__main__':
    main()
    
    
    
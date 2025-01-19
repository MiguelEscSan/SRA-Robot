#!/usr/bin/env python3

from ev3dev2.sensor import INPUT_1, INPUT_2
from ev3dev2.sensor.lego import UltrasonicSensor, ColorSensor
from ev3dev2.sound import Sound
from ev3dev2.led import Leds
from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C
from time import time
from movement import RobotMovement
from reactive import follow_obstacle, avoid_obstacle, find_line, move_until_found_obstacle


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

    leds.set_color('LEFT', 'RED')
    leds.set_color('RIGHT', 'RED')

    start_time = time()
    
    #---------------------------------------------------------------------------------------------------

    # Primer paso: Encontrar el primer obstáculo y seguirlo hasta una distancia de seguridad
    object_distance, _ = follow_obstacle(
        movement, 
        ultrasonic_sensor,
        distance=10,
        tolerance_distance=20,
        min_obstacle_distance=50
    )
    
    # Segundo paso: Esquivar el primer obstáculo hacia la izquierda, caminar distancia umbral + distancia obstáculo y girar ciertos grados de vuelta
    _ = avoid_obstacle(
        movement,
        ultrasonic_sensor,
        object_distance,
        tolerance_distance=20,
        turn_direction="left",
        turn_angle=10,
        step_distance=15,
        first_obstacle=True
    )
    
    # Tercer paso: Girar y caminar mientras no se haya encontrado el segundo obstáculo hacia la derecha. Girar poco y caminar poco.
    move_until_found_obstacle(
        movement,
        ultrasonic_sensor,
        distance=4,
        degrees=8,
    )
    
    # Cuarto paso: Acercarse hasta el segundo obstáculo hasta una distancia de seguridad
    object_distance, _ = follow_obstacle(
        movement,
        ultrasonic_sensor,
        distance=5,
        tolerance_distance=22,
        min_obstacle_distance=60,
        max_angle=40
    )
    
    # Quinto paso: Girar hasta dejar de ver el segundo obstáculo
    turned_degrees = avoid_obstacle(
        movement,
        ultrasonic_sensor,
        object_distance,
        tolerance_distance=20,
        turn_direction="right",
        turn_angle=15,
        step_distance=15,
        first_obstacle=False
    )
    
    # Sexto paso: Caminar hasta ver la línea, detenerse y colocarse correctamente
    find_line(
        movement, 
        color_sensor, 
        turned_degrees,
        step_distance=10,
        threshold=10
    )
    
    end_time = time()
    
    leds.set_color('LEFT', 'GREEN')
    leds.set_color('RIGHT', 'GREEN')

    sound.beep()

    print("Competition finished in: {:.2f} seconds".format(end_time - start_time))

    



if __name__ == '__main__':
    main()
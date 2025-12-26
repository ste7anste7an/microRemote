from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor, InfraredSensor, TouchSensor, UltrasonicSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = EV3Brick()

from microuart import MicroUART

uu=MicroUART(Port.S2)


while True:
    but = hub.buttons.pressed()
    if Button.LEFT in but:
        print('left')
        uu.call('led',-1)
    elif Button.RIGHT in but:
        uu.call('led',1)
        print('right')
    wait(100)
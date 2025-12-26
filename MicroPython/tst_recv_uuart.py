from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor, InfraredSensor, TouchSensor, UltrasonicSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = EV3Brick()

from microuart import MicroUART

uu=MicroUART(Port.S2)
print(uu.decode(b'\x03cmdS\x03catN\x03123'))

# i=0
# s=b'\x40'+bytearray([i+65 for i in range(0x40)])
# while True:
#     send_str(i)
#     print(i)
#     i+=1
#     i%=32
#     wait(100)

def test(a,b):
    return a==b,

#uu.send_command('test',123,12)
i=0
while True:
    #data=uu.receive_bytes()
    data=uu.receive_command()
    uu.send_command('test',i,i+2)
    print(i,data)#,uu.decode(data))
    i+=1
    if i> 1000:
        i=0
    #wait(10)
    
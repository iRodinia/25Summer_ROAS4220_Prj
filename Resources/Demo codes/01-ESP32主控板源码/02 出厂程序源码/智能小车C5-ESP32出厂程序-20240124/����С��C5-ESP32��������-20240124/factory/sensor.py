from factory.hcsr04 import HCSR04
from factory.z_uart import ZL_UART
from machine import Pin
import time

systick_ms_gs = 0
systick_ms_zybz = 0
systick_ms_xun = 0
systick_ms_xjbz = 0
xunji_l_pin = 0
xunji_r_pin = 0
car_speed = 600    # 小车速度，范围0~1000
xunji_speed = 450  # 小车循迹速度，范围0~1000

def setup_sensor():
    global sensor,uart
    setup_xunji(34,36)
    uart = ZL_UART()                             # 实例化串口对象
    sensor = HCSR04(trigger_pin=2, echo_pin=4)   # 定义超声波模块Tring控制管脚及超声波模块Echo控制管脚,S3接口

def loop_sensor():
    ziyou_bizhang()

# 定距跟随
def dingju_gensui():
    global systick_ms_gs, sensor
    if millis() - systick_ms_gs > 50:
        systick_ms_gs = millis()
        dis = sensor.distance_cm()
        if dis < 20:
            car_run(0-car_speed,0-car_speed,0-car_speed,0-car_speed,200)
        elif 25 < dis < 35 or dis > 70:
            car_stop()
        elif 40 < dis < 60:
            car_run(car_speed,car_speed,car_speed,car_speed,0)

# 自由避障
def ziyou_bizhang():
    global systick_ms_xjbz, sensor
    if millis() - systick_ms_xjbz > 50:
        systick_ms_xjbz = millis()
        dis = sensor.distance_cm()
        if dis < 20:
            car_run(car_speed,0-car_speed,car_speed,0-car_speed,200)
        else:
            car_run(car_speed,car_speed,car_speed,car_speed,0)

# 智能循迹
def car_xunji():
    global systick_ms_xun
    if millis() - systick_ms_xun > 50:
        systick_ms_xun = millis()
        if xunji_l() == 0 and xunji_r() == 0:
            car_run(xunji_speed,xunji_speed,xunji_speed,xunji_speed,0)
        elif xunji_l() == 1 and xunji_r() == 0:
            car_run(xunji_speed,0,xunji_speed,0,0)
        elif xunji_l() == 0 and xunji_r() == 1:
            car_run(0,xunji_speed,0,xunji_speed,0)
            

# 循迹避障
def xunji_bizhang():
    global systick_ms_zybz, sensor
    if millis() - systick_ms_zybz > 50:
        systick_ms_zybz = millis()
        dis = sensor.distance_cm()
        if dis < 20:
            car_stop()
        else:
            if xunji_l() == 0 and xunji_r() == 0:
                car_run(xunji_speed,xunji_speed,xunji_speed,xunji_speed,0)
            elif xunji_l() == 1 and xunji_r() == 0:
                car_run(xunji_speed,0,xunji_speed,0,0)
            elif xunji_l() == 0 and xunji_r() == 1:
                car_run(0,xunji_speed,0,xunji_speed,0)

def setup_xunji(xunji_l_PIN,xunji_r_PIN):
    global xunji_l_pin,xunji_r_pin
    xunji_l_pin = Pin(xunji_l_PIN, Pin.IN)    # 将对应引脚设置为输入模式
    xunji_r_pin = Pin(xunji_r_PIN, Pin.IN)    # 将对应引脚设置为输入模式

def xunji_l():
    return xunji_l_pin.value()

def xunji_r():
    return xunji_r_pin.value()

#获取系统时间，毫秒为单位
def millis():
    return int(time.time_ns()//1000000)

def car_run(speed_qz,speed_qy,speed_hz,speed_hy,runtime):
    global uart
    carSrt = '#006P{0:0>4d}T{4:0>4d}!#007P{1:0>4d}T{4:0>4d}!#008P{2:0>4d}T{4:0>4d}!#009P{3:0>4d}T{4:0>4d}!'.format(1500+speed_qz,1500-speed_qy,1500+speed_hz,1500-speed_hy,runtime)
    print(carSrt)
    uart.uart_send_str(carSrt)

#小车停止
def car_stop():
    global uart
    uart.uart_send_str('#006P1500T1000!#007P1500T1000!#008P1500T1000!#009P1500T1000!')

# 程序入口
if __name__ == '__main__':
    uart = ZL_UART()                                   # 实例化串口对象
    setup_sensor()
    while 1:
        #ziyou_bizhang() 
        car_xunji()
    
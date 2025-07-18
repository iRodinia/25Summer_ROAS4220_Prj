from factory.z_uart import ZL_UART
import time
  
'''
通过串口发送指令控制电机的转速,时间
参数：
speed_l1---左前轮
speed_r1---右前轮
speed_l2---左后轮
speed_r2---右后轮
(-1000~1000)负值后退，正值前进，绝对值越大转速越高。
time 代表车轮转动时间，0代表一直转动，1000代表转动1秒，以此类推。
'''
def car_run(speed_qz,speed_qy,speed_hz,speed_hy,runtime):
    carSrt = '#006P{0:0>4d}T{4:0>4d}!#007P{1:0>4d}T{4:0>4d}!#008P{2:0>4d}T{4:0>4d}!#009P{3:0>4d}T{4:0>4d}!'.format(1500+speed_qz,1500-speed_qy,1500+speed_hz,1500-speed_hy,runtime)
    print(carSrt)
    uart.uart_send_str(carSrt)

#小车停止
def car_stop():
    uart.uart_send_str('#006P1500T1000!#007P1500T1000!#008P1500T1000!#009P1500T1000!')
 
if __name__ == '__main__':
    uart = ZL_UART()                                   # 实例化串口对象
    #麦克纳姆轮控制
    car_run(400,400,400,400,1000) #前进
    time.sleep(1)
    car_run(-400,-400,-400,-400,1000) #后退
    time.sleep(1)
    car_run(-400,400,-400,400,1000) #左转
    time.sleep(1)
    car_run(400,-400,400,-400,1000) #右转
    time.sleep(1)
    car_run(-400,400,400,-400,1000) #左平移
    time.sleep(1)
    car_run(400,-400,-400,400,1000) #右平移
    time.sleep(1)
    car_stop()  

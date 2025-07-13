'''
K210颜色跟随
实验效果：
    本实验以跟随红色木块为例。
    将红色木块在k210前至少10厘米开外上下左右移动时，机械臂摄像头会跟随红色木块运动。
还可自行将红色阈值替换为其他颜色阈值来识别。
# 注意：环境光会对识别有影响，找比较合适的环境来运行此程序。
颜色阈值采集方法：
方法一：
    1.可以在‘工具’-->‘机器视觉’-->‘阈值编辑器’-->‘帧缓冲区’中采集，
    2.拖动6个滑块来调整图像，直到要采集的颜色都呈白色，其他都呈黑色为止，
    3.采集好之后将LAB阈值复制到代码中的阈值处使用即可。
方法二：
    1.可以在右侧‘帧缓冲区’和‘直方图’的’LAB色彩空间‘配合采集
    2.先在帧缓冲区中用鼠标左键框出要采集的颜色区域，
    3.然后在直方图中的LAB色彩空间中，分别复制L、A、B的最小值和最大值到代码中的颜色阈值元组中使用即可
'''
import sensor, image, time, lcd
import utime
from fpioa_manager import fm
from machine import UART

#颜色阈值索引，可以改变索引值来识别不同颜色此处0为红色，1为绿色，2为蓝色，此处默认以红色为例
threshold_index = 0

thresholds = [
              (20, 48, 40, 80, 10, 50),    #红色阈值
              (32, 87, -106, -23, -67, 93),   #绿色阈值
              (5, 58, 15, 45, -72, -30)   #蓝色阈值
              ]

pwm_value1 = 1500     #云台下方舵机的pwm值
pwm_value2 = 1000     #003号舵机的pwm值
systick_ms_bak = 0    #记录系统运行时间

#机械臂呈半弯曲状态的指令
jibot_qs_action_commond = "{#000P1500T1500!#001P2000T1500!#002P2000T1500!#003P1000T1500!#004P1500T1500!#005P1200T1500!}"

uart_send_str_bak = 0

#初始化摄像头
def init_sensor():
    global sensor, clock
    sensor.reset()                       #重置并初始化摄像头
    sensor.set_pixformat(sensor.RGB565)  #将像素格式设置为彩色RGB565
    sensor.set_framesize(sensor.QVGA)    #将帧大小设置为QVGA（320x240）
    sensor.skip_frames(time=100)         #等待设置生效
    clock = time.clock()                 #创建一个时钟对象来跟踪FPS

#初始化LCD屏
def init_lcd():
    global lcd
    lcd.init()                           #初始化LCD显示
    lcd.clear(lcd.RED)                   #清屏

#初始化串口2
def init_uart2(baud=115200):
    global uart2
    #串口注册
    # binding UART2 IO:6->RX, 8->TX
    fm.register(6, fm.fpioa.UART2_RX)
    fm.register(8, fm.fpioa.UART2_TX)
    uart2 = UART(UART.UART2, baud, 8, 0, 0, timeout=1000, read_buf_len=4096)
    print('初始化完成')

    utime.sleep(3)                       #延时3秒
    #使机械臂呈半弯曲状态，方便摄像头查看物体
    uart2_send(jibot_qs_action_commond)
    utime.sleep(1)                       #延时1秒
    #使机械臂呈半弯曲状态，方便摄像头查看物体
    uart2_send(jibot_qs_action_commond)
    utime.sleep(1)                       #延时1秒

#串口接收数据
def uart2_recv():
    recv_data = uart2.read()
    return recv_data

#串口发送数据
def uart2_send(send_data):
    global uart2
    uart2.write(send_data)

#释放串口2
def deinit_uart2():
    global uart2
    uart2.deinit()
    del uart2

#查找识别到的目标颜色区域中最大区域的函数
def find_max(blobs):
    max_size = 0
    for blob in blobs:
        if blob.w() * blob.h() > max_size:
            max_blob = blob
            max_size = blob.w() * blob.h()
    return max_blob

#摄像头跟随
def camera_follow(x_bias, y_bias):
    global pwm_value1, pwm_value2, uart_send_str_bak
    if abs(x_bias) > 2 or abs(y_bias) > 2:
        pwm_value1 = pwm_value1 - x_bias//1.3   #当跟随的幅度太大（或太小）时，将1.3调大（或调小）一点
        pwm_value2 = pwm_value2 - y_bias//1.3   #当跟随的幅度太大（或太小）时，将1.3调大（或调小）一点
        #pwm值超出设定范围时的处理
        if pwm_value1 > 2000:
            pwm_value1 = 2000
        if pwm_value1 < 1000:
            pwm_value1 = 1000
        if pwm_value2 > 1300:
            pwm_value2 = 1300
        if pwm_value2 < 600:
            pwm_value2 = 600
        Str = '{#000P%04dT0000!#003P%04dT0000!}' % (pwm_value1, pwm_value2)
        if uart_send_str_bak != Str:
            print('str: ', Str)
            uart2_send(Str)
            uart_send_str_bak = Str

#主函数
def loop_main():
    global systick_ms_bak
    while 1:
        #每隔50毫秒进入执行一次
        if utime.ticks_ms() - systick_ms_bak >= 50:
            systick_ms_bak = utime.ticks_ms()
            clock.tick()                    #更新FPS时钟
            img = sensor.snapshot()         #拍张照片并返回图像
            #查找图像中的目标颜色，[thresholds[threshold_index]]为列表格式
            blobs = img.find_blobs([thresholds[threshold_index]], pixels_threshold=100, area_threshold=100)

            #如果图像中有目标颜色
            if blobs:
                max_blob = find_max(blobs)           #找到最大的目标颜色区域
                img.draw_rectangle(max_blob.rect())  #将最大的目标颜色区域框出来
                img.draw_cross(max_blob.cx(), max_blob.cy())  #在目标颜色区域的中心点处画十字

                #获取目标颜色区域的宽度和高度
                c_w = max_blob.w()
                c_h = max_blob.h()
                #print('c_w: ', c_w, end='\t')
                #print('c_h: ', c_h)

                #获取目标颜色区域的中点坐标和画面中心点的坐标差值
                x_bias = max_blob.cx() - img.width()/2
                y_bias = max_blob.cy() - img.height()/2
                #print('x_bias: ', x_bias, end='\t')
                #print('y_bias: ', y_bias)

                #当目标颜色区域的宽度和高度满足条件时再跟随
                if 10 < c_w < 150 and 10 < c_h < 150:
                    camera_follow(x_bias, y_bias)

            lcd.display(img)                #显示画面到LCD屏上
            #print(clock.fps())              #注意：连接IDE后，k210的运行速度大约是原来的一半

#程序入口
if __name__ == '__main__':
    init_sensor()      #初始化摄像头
    init_lcd()         #初始化LCD显示屏
    init_uart2()       #初始化串口2
    try:
        loop_main()    #执行主函数
    except:
        #使机械臂呈半弯曲状态，方便摄像头查看物体
        uart2_send(jibot_qs_action_commond)
    deinit_uart2()     #释放串口2

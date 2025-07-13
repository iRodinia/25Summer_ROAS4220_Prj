'''
视觉巡线小车实验：
准备工作：
    为了使这个脚本正常工作，请确保黑线在相机的视野内。
实验效果：
    小车开机后，等待几秒钟后，小车开始沿着黑线循迹。
注意：若小车速度过低，可能是电池快没电了需要充电，或者将代码中的循迹速度改快一些。
'''
import sensor, image, time, utime, lcd, math
from fpioa_manager import fm
from machine import UART

#机械臂最适合识别黑线的位置的动作组
detection_position_action = '{#000P1500T1000!#001P1500T1000!#002P1980T1000!#003P0730T1000!#004P15000T1000!}'

#限制目标颜色区域的最小面积
target_area_min = 600

thresholds = (0, 30, -50, 50, -50, 50)  #黑色阈值

#循迹速度，范围为0-1000，可根据实际使用场景修改
xunji_speed = 450

#初始化摄像头
def init_sensor():
    global sensor, clock
    sensor.reset()                      #重置并初始化摄像头
    sensor.set_pixformat(sensor.RGB565) #将像素格式设置为彩色RGB565
    sensor.set_framesize(sensor.QVGA)   #将帧大小设置为QVGA（320x240）
    sensor.skip_frames(time=2000)       #等待设置生效
    sensor.set_auto_gain(False)         #颜色跟踪必须关闭自动增益
    sensor.set_auto_whitebal(False)     #颜色跟踪必须关闭白平衡
    clock = time.clock()                #创建一个时钟对象来跟踪FPS

#初始化LCD屏
def init_lcd():
    global lcd
    lcd.init()                          #初始化LCD显示
    lcd.clear(lcd.RED)                  #清屏

#初始化串口2
def init_uart2(baud=115200):
    global uart2
    #串口注册
    # binding UART2 IO:6->RX, 8->TX
    fm.register(6, fm.fpioa.UART2_RX)
    fm.register(8, fm.fpioa.UART2_TX)
    uart2 = UART(UART.UART2, baud, 8, 0, 0, timeout=1000, read_buf_len=4096)
    print('串口初始化完成!')
    utime.sleep(3)
    uart2_send(detection_position_action)
    utime.sleep(1)
    uart2_send(detection_position_action)
    utime.sleep(1)
    print('初始化完成')

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

'''
通过串口发送指令控制电机的转速,时间
参数：
speed_left---左轮
speed_right---右轮
(-1000~1000)负值后退，正值前进，绝对值越大转速越高。
time 代表车轮转动时间，0代表一直转动，1000代表转动1秒，以此类推。
'''
def car_run(speed_left, speed_right, time=0):
    textStr = '#006P{0:0>4d}T{2:0>4d}!#007P{1:0>4d}T{2:0>4d}!#008P{0:0>4d}T{2:0>4d}!#009P{1:0>4d}T{2:0>4d}!'.format(1500+speed_left, 1500-speed_right, time)
    textStr = "{" + textStr + "}"
    print(textStr)
    uart2_send(textStr)

#查找识别到的目标颜色区域中最大区域的函数
def find_max(blobs):
    max_size = 0
    for blob in blobs:
        if blob.w() * blob.h() > max_size:
            max_blob = blob
            max_size = blob.w() * blob.h()
    return max_blob


#主函数
def loop_main():
    while 1:
        clock.tick()                    #更新FPS时钟
        img = sensor.snapshot()         #拍张照片并返回图像

        #查找图像中的目标颜色，[thresholds]为列表格式
        blobs = img.find_blobs([thresholds], roi=(0,40,320,200),pixels_threshold=100, area_threshold=100)

        #如果图像中有目标颜色
        if blobs:
            #找到最大的目标颜色区域
            max_blob = find_max(blobs)
            #将最大的目标颜色区域框出来
            #img.draw_rectangle(max_blob.rect(),color=(0,255,0),thickness=3)
            #在目标颜色区域的中心点处画十字
            img.draw_cross(max_blob.cx(), max_blob.cy(),color=(0,255,0),thickness=3)
            #print('max:', max_blob)

            #获取目标颜色区域的中心点的x坐标
            c_x = max_blob.cx()

            #获取目标颜色区域的宽度和高度
            c_w = max_blob.w()
            c_h = max_blob.h()
            #print('c_w: ', c_w, end='\t')
            #print('c_h: ', c_h)
            #计算目标颜色区域的面积
            area = c_w * c_h

            #去除面积非常小的目标区域
            if area >= target_area_min:
                #print('area: ', area)

                # 根据X坐标在画面中的位置来调用不同速度的car_run()函数，让小车动
                '''
                if c_x < 100:
                    car_run(0, xunji_speed, 0)   # 左转
                elif 100 <= c_x <= 220:
                    car_run(xunji_speed, xunji_speed, 0)  # 前进
                elif 220 < c_x:
                    car_run(xunji_speed, 0, 0)   # 右转
                '''
                # 0  40  110  160  210  280  320
                if c_x <= 40:
                    car_run(100-xunji_speed, xunji_speed, 0)       # 大左转
                elif 40 < c_x <= 110:
                    car_run(0, xunji_speed-50, 0)    # 小左转
                elif 110 < c_x < 210:
                    car_run(xunji_speed-50, xunji_speed-50, 0)  # 前进
                elif 210 <= c_x < 280:
                    car_run(xunji_speed-50, 0, 0)    # 小右转
                elif c_x >= 280:
                    car_run(xunji_speed, 100-xunji_speed, 0)       # 大右转

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
        car_run(0, 0)  #小车停止
    deinit_uart2()     #释放串口2

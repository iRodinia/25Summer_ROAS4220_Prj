'''
k210在搬运机器人C5J上的测试程序：颜色识别做动作
实验效果：
    程序运行后，机械臂会朝下识别桌面部分，
    当识别到红色物体时，机械臂会点头；
    当识别到绿色物体时，机械臂会摇头。
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
import sensor, image, time, utime, lcd, math
from fpioa_manager import fm
from machine import UART

# 目标颜色实际出现的次数，每次目标颜色出现的次数超过5次时，才认为真的识别到了目标颜色，是为了防止误识别
target_color_appear_count = 0

# 设置颜色阈值，此处设置红绿蓝三种颜色的阈值
thresholds = {
              'red': (20, 48, 40, 80, 10, 50),        # 红色阈值
              'green': (19, 60, -100, 0, -100, 48),   # 绿色阈值(32, 87, -106, -23, -67, 93)
              #'blue': (5, 58, 15, 45, -72, -30)      # 蓝色阈值
             }

# 各颜色对应要执行的动作组
actions = {
           'detection_position_action': '{#001P1500T1000!#002P1980T1000!#003P0730T1000!}', # 机械臂最适合识别木块的位置的动作
           'up_action': '#003P0850T0700!',            # 抬头动作
           'down_action': '#003P0730T0700!',          # 低头动作
           'first_left_action': '#000P1550T1000!',    # 识别到目标颜色后的第一次左摇头动作
           'left_action': '#000P1550T1000!',          # 左摇头动作
           'right_action': '#000P1450T1000!',         # 右摇头动作
           '000_fuwei_action': '#000P1500T0500!',     # 头部归中复位动作
          }

action_need_time = 0.5   # 动作执行消耗的时间，单位为秒

# 限制目标颜色区域的最小面积
target_area_min = 2000


# 初始化摄像头
def init_sensor():
    global sensor, clock
    sensor.reset()                      # 重置并初始化摄像头
    sensor.set_pixformat(sensor.RGB565) # 将像素格式设置为彩色RGB565
    sensor.set_framesize(sensor.QVGA)   # 将帧大小设置为QVGA（320x240）
    sensor.skip_frames(time=2000)       # 等待设置生效
    sensor.set_auto_gain(False)         # 颜色跟踪必须关闭自动增益
    sensor.set_auto_whitebal(False)     # 颜色跟踪必须关闭白平衡
    clock = time.clock()                # 创建一个时钟对象来跟踪FPS

# 初始化LCD屏
def init_lcd():
    global lcd
    lcd.init()                          # 初始化LCD显示
    lcd.clear(lcd.RED)                  # 清屏

# 初始化串口2
def init_uart2(baud=115200):
    global uart2
    # 串口注册
    # binding UART2 IO:6->RX, 8->TX
    fm.register(6, fm.fpioa.UART2_RX)
    fm.register(8, fm.fpioa.UART2_TX)
    uart2 = UART(UART.UART2, baud, 8, 0, 0, timeout=1000, read_buf_len=4096)
    print('串口初始化完成!')

# 初始化函数
def init_setup():
    init_sensor()     # 初始化摄像头
    init_lcd()        # 初始化LCD屏
    init_uart2()      # 初始化串口2
    print('初始化完成')
    utime.sleep(3)
    uart2_send(actions['detection_position_action'])
    utime.sleep(1)
    uart2_send(actions['detection_position_action'])
    utime.sleep(1)

# 串口接收数据
def uart2_recv():
    recv_data = uart2.read()
    return recv_data

# 串口发送数据
def uart2_send(send_data):
    global uart2
    uart2.write(send_data)

# 释放串口2
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


# 点头动作
def diantou_action(times=3):
    for i in range(times):
        uart2_send(actions['up_action'])     # 串口将动作指令发出去
        utime.sleep(action_need_time)        # 延时以便让动作执行完成
        uart2_send(actions['down_action'])   # 串口将动作指令发出去
        utime.sleep(action_need_time)        # 延时以便让动作执行完成

# 摇头动作
def yaotou_action(times=3):
    uart2_send(actions['first_left_action']) # 串口将动作指令发出去
    utime.sleep(action_need_time)            # 延时以便让动作执行完成
    uart2_send(actions['right_action'])      # 串口将动作指令发出去
    utime.sleep(action_need_time)            # 延时以便让动作执行完成
    for i in range(times-2):
        uart2_send(actions['left_action'])   # 串口将动作指令发出去
        utime.sleep(action_need_time)        # 延时以便让动作执行完成
        uart2_send(actions['right_action'])  # 串口将动作指令发出去
        utime.sleep(action_need_time)        # 延时以便让动作执行完成
    uart2_send(actions['000_fuwei_action'])  # 串口将动作指令发出去
    utime.sleep(action_need_time)            # 延时以便让动作执行完成

# 主函数
def loop_main():
    global target_color_appear_count
    while 1:
        clock.tick()                    # 更新FPS时钟
        img = sensor.snapshot()         # 拍张照片并返回图像

        #循环几种颜色阈值，在图像中查看是否有对应颜色并做一些操作，thresholds需为字典格式
        for threshold in thresholds:
            color_blobs = img.find_blobs([thresholds[threshold]], pixels_threshold=100, area_threshold=100)  #在图像中查找目标颜色区域

            #当查找到目标颜色区域后
            if color_blobs:
                max_blob = find_max(color_blobs)             # 查找图像中最大的目标色块
                img.draw_rectangle(max_blob.rect())          # 将找到的目标颜色区域用矩形框出来
                img.draw_cross(max_blob.cx(), max_blob.cy()) # 在目标颜色中心位置画个十字
                target_area = max_blob.area()                # 获取目标颜色区域的面积
                print('area is  ', target_area)              # 打印识别到的目标颜色区域面积

                if target_area > target_area_min:            # 面积合适时
                    target_color_appear_count += 1           # 目标颜色实际出现的次数+1

                if target_color_appear_count > 5:            # 识别到的面积合适时的次数大于5次时串口发数据
                    #print('uart3 send color is ', threshold)

                    # 识别到对应颜色木块后执行对应动作
                    if threshold == 'red':
                        diantou_action()    # 串口将红色木块对应的动作指令发出去
                    elif threshold == 'green':
                        yaotou_action()     # 串口将绿色木块对应的动作指令发出去

                    target_color_appear_count = 0

        lcd.display(img)                # 显示画面到LCD屏上
        #print(clock.fps())              # 注意：连接IDE后，k210的运行速度大约是原来的一半

# 程序入口
if __name__ == '__main__':
    init_setup()            # 执行初始化函数
    try:
        loop_main()
    except:
        pass
    deinit_uart2()

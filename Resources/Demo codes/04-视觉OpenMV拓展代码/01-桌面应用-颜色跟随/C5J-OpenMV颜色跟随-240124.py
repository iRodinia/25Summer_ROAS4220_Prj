'''
颜色跟随
实验效果：
    本实验以跟随红色木块为例。
    将红色木块在OpenMV前至少10厘米开外上下左右移动时，机械臂摄像头会跟随红色木块运动。
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
from pyb import millis, UART, Servo
from math import pi, isnan
import sensor, image, time
import utime

target_threshold  = (18, 74, 20, 127, -25, 127)    # 此处以红色为例
pwm_value1 = 1500     # 云台下方舵机的pwm值
pwm_value2 = 1000     # 003号舵机的pwm值
systick_ms_bak = 0    # 记录系统运行时间

# 初始化函数
def init_setup():
    global sensor, clock, uart3
    sensor.reset()                               # 初始化感光元件
    sensor.set_pixformat(sensor.RGB565)          # 图像格式设置为彩色
    sensor.set_framesize(sensor.QQVGA)           # 图像大小设置为QQVGA，即160x120
    sensor.skip_frames(10)                       # 跳过10帧，使以上设置生效
    sensor.set_auto_whitebal(False)              # 关闭白平衡
    clock = time.clock()                         # 创建时钟对象
    uart3 = UART(3, 115200)                      # 初始化串口3，设置波特率为115200
    print('初始化完成')

    utime.sleep(5)                               # 延时5秒
    # 使机械臂呈半弯曲状态，方便摄像头查看物体
    uart3_send("{#000P1500T1500!#001P2000T1500!#002P2000T1500!#003P1000T1500!#004P1500T1500!#005P1200T1500!}")
    utime.sleep(2)                               # 延时2秒


# 串口接收数据
def uart3_recv():
    recv_data = uart3.read()
    print('recv data: %s' % recv_data)
    return recv_data

# 串口发送数据
def uart3_send(send_data):
    global uart3
    uart3.write(send_data)

# 查找识别到的目标颜色区域中最大区域的函数
def find_max(blobs):
    max_size = 0
    for blob in blobs:
        if blob[2] * blob[3] > max_size:
            max_blob = blob
            max_size = blob[2] * blob[3]
    return max_blob

# 摄像头跟随
def camera_follow(x_bias, y_bias):
    global pwm_value1, pwm_value2
    if abs(x_bias) > 2 or abs(y_bias) > 2:
        pwm_value1 = pwm_value1 - x_bias//1.3   # 当跟随的幅度太大（或太小）时，将1.3调大（或调小）一点
        pwm_value2 = pwm_value2 - y_bias//1.3   # 当跟随的幅度太大（或太小）时，将1.3调大（或调小）一点
        # pwm值超出设定范围时的处理
        if pwm_value1 > 2000:
            pwm_value1 = 2000
        if pwm_value1 < 1000:
            pwm_value1 = 1000
        if pwm_value2 > 1300:
            pwm_value2 = 1300
        if pwm_value2 < 600:
            pwm_value2 = 600
        Str = '{#000P%04dT0000!#003P%04dT0000!}' % (pwm_value1, pwm_value2)
        print('str: ', Str)
        uart3_send(Str)

# 主函数
def main():
    global systick_ms_bak
    while 1:
        if millis() - systick_ms_bak >= 50:
            systick_ms_bak = millis()
            clock.tick()                             # 更新FPS帧率时钟
            img = sensor.snapshot()                  # 拍一张照片并返回图像

            blobs = img.find_blobs([target_threshold])  # 查找图像中的目标颜色

            # 如果图像中有目标颜色
            if blobs:
                max_blob = find_max(blobs)           # 找到最大的目标颜色区域
                img.draw_rectangle(max_blob.rect())  # 将最大的目标颜色区域框出来
                img.draw_cross(max_blob.cx(), max_blob.cy())  # 在目标颜色区域的中心点处画十字

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

                # 当目标颜色区域的宽度和高度满足条件时再跟随
                if 4 < c_w < 80 and 4 < c_h < 80:
                    camera_follow(x_bias, y_bias)

# 程序入口
if __name__ == '__main__':
    init_setup()    # 执行初始化函数
    try:
        main()      # 执行主函数
    finally:
        # 使机械臂呈半弯曲状态，方便摄像头查看物体
        uart3_send("{#000P1500T1500!#001P2000T1500!#002P2000T1500!#003P1000T1500!#004P1500T1500!#005P1200T1500!}")

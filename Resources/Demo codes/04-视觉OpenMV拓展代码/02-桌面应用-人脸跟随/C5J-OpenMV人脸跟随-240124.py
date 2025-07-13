'''
人脸跟随
# 注意：环境光会对识别有影响，找比较合适的环境来运行此程序。
'''
from pyb import millis, UART, Servo
from math import pi, isnan
import sensor, image, time
import utime

target_threshold  = (35, 67, 67, 102, 86, 10)    # 此处以红色为例
pwm_value1 = 1500     # 云台下方舵机的pwm值
pwm_value2 = 1000     # 003号舵机的pwm值
systick_ms_bak = 0    # 记录系统运行时间

# 初始化函数
def init_setup():
    global sensor, clock, uart3, face_cascade
    sensor.reset()                               # 初始化感光元件
    sensor.set_contrast(3)                       # 对比度
    sensor.set_gainceiling(16)                   # 自动增益
    sensor.set_pixformat(sensor.GRAYSCALE)       # 图像格式设置为灰度
    sensor.set_framesize(sensor.HQVGA)           # 图像大小设置为HQVGA，即240x160
    sensor.skip_frames(10)                       # 跳过10帧，使以上设置生效
    sensor.set_auto_whitebal(False)              # 关闭白平衡
    face_cascade = image.HaarCascade("frontalface", stages=25)

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

# 查找识别到的目标区域中最大人脸的函数
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
    if abs(x_bias) > 1 or abs(y_bias) > 1:
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
            # 查找图像中的人脸
            faces = img.find_features(face_cascade, threshold=0.75, scale=1.25)

            # 如果图像中有人脸
            if faces:
                face = find_max(faces)               # 找到画面中最大的人脸
                #print('face: ', face)
                img.draw_rectangle(face, color=(0, 255, 0), thickness=5)  # 将最大的人脸框出来

                #获取人脸的宽度和高度
                c_w = face[2]
                c_h = face[3]

                #print('c_w: ', c_w, end='\t')
                #print('c_h: ', c_h)

                #获取人脸的中点坐标和画面中心点的坐标差值
                x_bias = face[0] - img.width()/2
                y_bias = face[1] - img.height()/2

                #print('x_bias: ', x_bias, end='\t')
                #print('y_bias: ', y_bias)

                # 当人脸的宽度和高度满足条件时再跟随
                if 4 < c_w < 160 and 4 < c_h < 120:
                    camera_follow(x_bias, y_bias)

# 程序入口
if __name__ == '__main__':
    init_setup()    # 执行初始化函数
    try:
        main()      # 执行主函数
    finally:
        # 使机械臂呈半弯曲状态，方便摄像头查看物体
        uart3_send("{#000P1500T1500!#001P2000T1500!#002P2000T1500!#003P1000T1500!#004P1500T1500!#005P1200T1500!}")

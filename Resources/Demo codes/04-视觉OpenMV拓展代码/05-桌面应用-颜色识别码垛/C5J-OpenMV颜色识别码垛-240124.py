'''
颜色识别码垛
实验效果：
    将红绿蓝其中任意一种颜色的木块放置在小车前方摄像头能看到的范围内，
    OpenMV识别到木块后会将木块夹取放置到左侧堆叠起来，最多叠三层。
    三层叠满后会重新从第一层开始堆叠。
    未识别到红绿蓝木块时，保持不动。
红绿蓝三种颜色可以同时识别，还可自行添加其他颜色阈值来识别其他颜色
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
import sensor, time
from pyb import UART
import utime
from math import *

wood_h_mm = 30        # 木块高30mm

img_w_px = 320        # 画面宽（即画面宽对应的像素）
img_h_px = 240        # 画面高（即画面高对应的像素）

img_to_car_mm = 105   # 画面最近端和机械臂0号舵机主轴中心点之间的距离，单位为mm
img_w_mm = 200        # 画面宽（即在识别位置start_position_cmd时，画面照射到的实际宽度，单位为mm）
img_h_mm = 150        # 画面高（即在识别位置start_position_cmd时，画面照射到的实际高度，单位为mm）

kms_x_bias = 15       # 逆运动学目标位置x坐标的偏差
kms_y_bias = 50       # 逆运动学目标位置y坐标的偏差

# 目标颜色实际出现的次数，每次目标颜色出现的次数超过指定最少次数时，才认为真的识别到了目标颜色，是为了防止误识别
color_appear_count = {
                     'red': 0,     # 记录红色出现的次数
                     'green': 0,   # 记录绿色出现的次数
                     'blue': 0     # 记录蓝色出现的次数
                     }
color_appear_count_min = 10        # 目标颜色至少需要出现的次数默认设置为10次
target_area_min = 2000             # 目标颜色至少要达到的最小区域
target_area_max = 20000            # 目标颜色不能超过的最大区域

level_cur = 0         # 记录当前正在堆叠第几层
level_max = 3         # 最大可堆叠层数，默认设置为3层

car_stop_cmd = "{#006P1500T0000!#007P1500T0000!#008P1500T0000!#009P1500T0000!}" #小车停止指令
start_position_cmd = '$KMS:0,120,120,1500!' # 机械臂运行到识别位置的指令
claw_open_cmd = '#005P0600T1000!'           # 爪子张开指令
claw_close_cmd = '#005P2400T1400!'          # 爪子闭合指令
clamp_after_cmd = '$KMS:0,150,150,1500!'    # 夹取到木块后，机械臂抬起来一些的指令

# 码垛位置正上方的z轴坐标，即高度
stack_place_up_h = int(wood_h_mm * (level_max + 1))
# 机械臂运行到码垛位置正上方的指令
stack_place_up_cmd = '$KMS:-200,70,%s,1000!' % stack_place_up_h

# 设置颜色阈值，此处设置红绿蓝三种颜色的阈值
thresholds = {
              'red': (18, 74, 34, 127, -25, 127),     # 红色阈值
              'green': (31, 60, -128, -25, -80, 36),  # 绿色阈值
              'blue': (12, 77, -29, 56, -126, -25)    # 蓝色阈值
             }

# 初始化函数
def init_setup():
    global sensor, clock, uart3            # 设置为全局变量
    sensor.reset()                         # 初始化感光元件
    sensor.set_pixformat(sensor.RGB565)    # 设置感光元件图像色彩格式为 RGB565 (RGB565为彩图，GRAYSCALE为灰度图)
    sensor.set_framesize(sensor.QVGA)      # 设置感光元件分辨率大小为 QVGA (QVGA是320x240)
    sensor.skip_frames(10)                 # 跳过一些帧，使以上的设置生效
    sensor.set_auto_gain(True)             # 打开自动增益, 默认打开；追踪颜色时需关闭白平衡
    sensor.set_auto_whitebal(True)         # 打开自动白平衡。默认打开；追踪颜色时需关闭白平衡
    sensor.set_auto_exposure(True)         # 打开自动曝光。默认打开
    clock = time.clock()                   # 创建时钟对象
    uart3 = UART(3, 115200)                # 初始化串口3，设置波特率为115200
    print('初始化完成')
    utime.sleep(5)
    uart3_send(car_stop_cmd)               # 让小车保持静止状态
    utime.sleep(1)                         # 延时以便让动作执行完成
    uart3_send(start_position_cmd)         # 机械臂运行到识别位置
    utime.sleep(2)                         # 延时以便让动作执行完成

# 串口接收数据
def uart3_recv():
    recv_data = uart3.read()
    print('recv data: %s' % recv_data)
    return recv_data

# 串口发送数据
def uart3_send(send_data):
    global uart3
    uart3.write(send_data)

# 查找图像中最大的色块
def find_max(blobs):
    max_size = 0
    for blob in blobs:
        if blob[2] * blob[3] > max_size:
            max_blob = blob
            max_size = blob[2] * blob[3]
    return max_blob

# 夹取木块
def clamp_wood(cx, cy, c_angle):
    # 计算逆运动学的X参数与Y参数
    # 将画面坐标换算为逆运动学坐标，由于机械臂在旋转，这里计算的是机械臂坐标系旋转后的坐标位置
    kms_x = int((cx-img_w_px/2) * img_w_mm / img_w_px)
    if kms_x < 0:
        kms_x = kms_x - kms_x_bias
    else:
        kms_x = kms_x + kms_x_bias
    kms_y = int(img_to_car_mm + img_h_mm/2 + (img_h_px/2-cy)*img_h_mm/img_h_px) + kms_y_bias

    #计算云台旋转角度
    if kms_x == 0 and kms_y != 0:
        theta0 = 0.0
    elif kms_y == 0 and kms_x > 0:
        theta0 = 90
    elif kms_y == 0 and kms_x < 0:
        theta0 = -90
    else:
        theta0 = atan(kms_x/kms_y) * 180.0 / pi
    #print('theta0: ', theta0)

    # 当机械臂在偏左位置时
    if theta0 < 0:
        #计算爪子旋转角度
        if c_angle >= int(abs(theta0)):
            if c_angle - abs(theta0) <= 45:
                servo_zhuazi = int(1500-2000.0*(c_angle-abs(theta0))/270.0)
                kms_y = kms_y - 5
            else:
                servo_zhuazi = int(1500+2000.0*(abs(theta0)+90-c_angle)/270.0)
                kms_y = kms_y + 2
        else:
            if abs(theta0) - c_angle <= 45:
                servo_zhuazi = int(1500+2000.0*(abs(theta0)-c_angle)/270.0)
                kms_y = kms_y + 2
            else:
                servo_zhuazi = int(1500-2000.0*(90-abs(theta0)+c_angle)/270.0)
                kms_y = kms_y - 5
    # 当机械臂在偏右位置时
    else:
        #计算爪子旋转角度
        if theta0 + c_angle <= 90:
            if 90 - theta0 - c_angle <= 45:
                servo_zhuazi = int(1500+2000.0*(90-theta0-c_angle)/270.0)
                #kms_y = kms_y + 2
            else:
                servo_zhuazi = int(1500-2000.0*(theta0+c_angle)/270.0)
                if kms_y <= img_to_car_mm + img_h_mm/2 + kms_y_bias:
                    kms_y = kms_y + 7
                elif img_to_car_mm+img_h_mm+kms_y_bias > kms_y and kms_y > img_to_car_mm+img_h_mm/2+kms_y_bias:
                    kms_y = kms_y - 0
        else:
            if theta0 + c_angle - 90 <= 45:
                servo_zhuazi = int(1500-2000.0*(theta0+c_angle-90)/270.0)
                kms_y = kms_y + 6
            else:
                servo_zhuazi = int(1500+2000.0*(180-theta0-c_angle)/270.0)
                kms_y = kms_y - 2

    #print('$KMS:%s,%s,20,1500!' % (kms_x, kms_y)) # 串口发送机械臂运行到夹取位置的指令
    # 串口发送机械臂运行到夹取位置的指令
    if kms_y <= img_to_car_mm + img_h_mm/2 + kms_y_bias:
        uart3_send('$KMS:%s,%s,20,1500!' % (kms_x, kms_y))
    else:
        uart3_send('$KMS:%s,%s,39,1500!' % (kms_x, kms_y))
    utime.sleep(2)                         # 延时以便让动作执行完成
    # 爪子旋转至合适的角度准备进行夹取
    #print('#004P%04dT1000!' % servo_zhuazi)
    uart3_send('#004P%04dT1000!' % servo_zhuazi)
    utime.sleep(1)                         # 延时以便让动作执行完成
    # 爪子闭合，即夹取
    uart3_send(claw_close_cmd)
    utime.sleep(1.5)                       # 延时以便让动作执行完成
    # 夹取木块后，机械臂抬起来一些
    uart3_send(clamp_after_cmd)
    utime.sleep(1)                         # 延时以便让动作执行完成
    # 爪子转正
    uart3_send('#004P1500T1000!')
    utime.sleep(1)                         # 延时以便让动作执行完成

# 放置木块
def place_wood(level):
    print('level cur is ', level)
    # 串口发送机械臂运行到码垛位置正上方的指令
    uart3_send(stack_place_up_cmd)
    utime.sleep(1.5)                       # 延时以便让动作执行完成
    # 准备放置到码垛位置的z轴坐标，即高度
    level_h = int(wood_h_mm*(level-1)+wood_h_mm/2)
    # 串口发送机械臂运行到码垛位置的指令
    #print('$KMS:-200,70,%s,2000!' % level_h)
    uart3_send('$KMS:-200,70,%s,2000!' % level_h)
    utime.sleep(2.5)                       # 延时以便让动作执行完成
    uart3_send(claw_open_cmd)              # 爪子张开来放下木块
    utime.sleep(1)                         # 延时以便让动作执行完成
    # 串口发送机械臂运行到码垛位置正上方的指令
    #print('stack up: ', stack_place_up_cmd)
    uart3_send(stack_place_up_cmd)
    utime.sleep(1)                         # 延时以便让动作执行完成
    for color in color_appear_count:
        color_appear_count[color] = 0

# 主函数
def main():
    global level_cur
    while 1:                               # 无限循环
        clock.tick()                       # 更新FPS帧率时钟
        img = sensor.snapshot()            # 拍一张照片并返回图像
        # 循环3种颜色阈值，在图像中查看是否有对应颜色并做一些操作，thresholds需为字典格式
        for threshold in thresholds:
            blobs = img.find_blobs([thresholds[threshold]])  # 在图像中查找目标颜色区域
            # 当查找到目标颜色区域后
            if blobs:
                max_blob = find_max(blobs)      # 查找图像中最大的目标色块
                dot_1 = max_blob.corners()[0]   # 获取第一个点的坐标
                dot_2 = max_blob.corners()[1]   # 获取第二个点的坐标
                dot_3 = max_blob.corners()[2]   # 获取第三个点的坐标
                dot_4 = max_blob.corners()[3]   # 获取第四个点的坐标

                # 用线将木块框出来
                img.draw_line(dot_1[0], dot_1[1], dot_2[0], dot_2[1], thickness=2)
                img.draw_line(dot_2[0], dot_2[1], dot_3[0], dot_3[1], thickness=2)
                img.draw_line(dot_3[0], dot_3[1], dot_4[0], dot_4[1], thickness=2)
                img.draw_line(dot_4[0], dot_4[1], dot_1[0], dot_1[1], thickness=2)

                target_area = max_blob.area()           # 获取目标颜色区域的面积

                # 面积合适时
                if target_area_min < target_area < target_area_max:
                    color_appear_count[threshold] += 1  # 目标颜色实际出现的次数+1

                # 确认识别到了目标颜色后，开始做对应操作
                # 识别到的面积合适时的次数大于需要识别到的最小次数时
                if color_appear_count[threshold] > color_appear_count_min:
                    # 记录当前准备堆叠第几层
                    level_cur += 1
                    # 当堆叠到设置的默认最大层数后，重新从第一层开始堆叠
                    if level_cur > level_max:
                        level_cur = 1
                    # 计算木块倾斜角度
                    c_angle = atan((dot_1[1]-dot_2[1])/(dot_2[0]-dot_1[0])) * 180.0 / pi
                    if c_angle < 0:
                        c_angle = int(90 + c_angle)
                    else:
                        c_angle = int(c_angle)
                    #print('c angle: ', c_angle)

                    # 爪子张开
                    uart3_send(claw_open_cmd)
                    utime.sleep(1)                      # 延时以便让动作执行完成
                    # 机械臂运行到夹取区域并夹取木块
                    clamp_wood(max_blob.cx(),max_blob.cy(),c_angle)
                    # 放置木块到目标区域
                    place_wood(level_cur)
                    # 放置完木块后，再让机械臂运行到识别位置
                    uart3_send(start_position_cmd)
                    utime.sleep(2)                      # 延时以便让动作执行完成
                    # 让小车保持静止状态
                    uart3_send(car_stop_cmd)
                    utime.sleep(1)                      # 延时


# 程序入口
if __name__ == '__main__':
    init_setup()            # 执行初始化函数
    try:                    # 异常处理
        main()              # 执行主函数
    except:
        pass

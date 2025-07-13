'''
二维码识别夹取木块
实验用到的二维码（可以将二维码贴到木块上来使用）：
    第一个二维码内容为QR1
    第二个二维码内容为QR2
    第三个二维码内容为QR3
实验效果：
    将贴有上述三个二维码中任意一个的木块放置在小车前方摄像头能看到的范围内，
    OpenMV识别到带有二维码的木块后，会将木块夹取放置到左侧堆叠起来，最多叠三层。
    三层叠满后会重新从第一层开始堆叠。
    对应的木块放置到对应位置：
    未识别到二维码时，保持不动。
还可自行修改要识别的二维码内容来夹取带有该二维码的木块。不过需要在程序中修改对应位置。
# 注意：环境光会对识别有影响，找比较合适的环境来运行此程序。
'''
import sensor, time
from pyb import UART, millis
import utime
from math import *

systick_ms_bak_zhuan = 0 # 记录当前时间，用于让云台左右稍微转动，便于识别
zhuan_flag = True        # 用于让云台左右稍微转动的标志位
servo_yuntai_bias = 0    # 用于让云台左右稍微转动的幅度

# 要识别的二维码内容
qr1_content = "QR1"
qr2_content = "QR2"
qr3_content = "QR3"

wood_h_mm = 30        # 木块高30mm

img_w_px = 480        # 画面宽（即画面宽对应的像素）
img_h_px = 320        # 画面高（即画面高对应的像素）

img_to_car_mm = 130   # 画面最近端和机械臂0号舵机主轴中心点之间的距离，单位为mm
img_w_mm = 150        # 画面宽（即在识别位置start_position_cmd时，画面照射到的实际宽度，单位为mm）
img_h_mm = 100        # 画面高（即在识别位置start_position_cmd时，画面照射到的实际高度，单位为mm）

kms_x_bias = 15       # 逆运动学目标位置x坐标的偏差
kms_y_bias = 50       # 逆运动学目标位置y坐标的偏差

# 目标二维码实际出现的次数，每次目标二维码出现的次数超过指定最少次数时，才认为真的识别到了目标二维码，是为了防止误识别
qr_appear_count = {
                     qr1_content: 0,     # 记录一号二维码出现的次数
                     qr2_content: 0,     # 记录二号二维码出现的次数
                     qr3_content: 0      # 记录三号二维码出现的次数
                     }
qr_appear_count_min = 1            # 目标二维码至少需要出现的次数默认设置为1次
qr_w_min = 50                      # 目标二维码宽度至少要达到的最小宽度
qr_w_max = 300                     # 目标二维码宽度不能超过的最大宽度
qr_h_min = 50                      # 目标二维码高度至少要达到的最小高度

level_cur = 0         # 记录当前正在堆叠第几层
level_max = 3         # 最大可堆叠层数，默认设置为3层

car_stop_cmd = "{#006P1500T0000!#007P1500T0000!#008P1500T0000!#009P1500T0000!}" #小车停止指令
start_position_cmd = '$KMS:0,125,120,1500!' # 机械臂运行到识别位置的指令
claw_open_cmd = '#005P0600T1000!'           # 爪子张开指令
claw_close_cmd = '#005P2400T1400!'          # 爪子闭合指令
clamp_after_cmd = '$KMS:0,150,150,1500!'    # 夹取到木块后，机械臂抬起来一些的指令

# 码垛位置正上方的z轴坐标，即高度
stack_place_up_h = int(wood_h_mm * (level_max + 1))
# 机械臂运行到码垛位置正上方的指令
stack_place_up_cmd = '$KMS:-200,70,%s,1000!' % stack_place_up_h

# 初始化函数
def init_setup():
    global sensor, clock, uart3            # 设置为全局变量
    sensor.reset()                         # 初始化感光元件
    sensor.set_pixformat(sensor.GRAYSCALE) # 此处设置的是灰度
    sensor.set_framesize(sensor.VGA)       # 设置图像分辨率为VGA，即640x480
    sensor.set_windowing((480, 320))       # 只查看画面中心的480x320的范围
    sensor.skip_frames(10)                 # 等待一些帧，使以上设置生效
    sensor.set_auto_gain(False)            # 必须关闭白平衡功能，以防止图像冲洗…
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

# 查找图像中最大的二维码
def find_max(codes):
    max_size = 0
    for code in codes:
        if code[2] * code[3] > max_size:
            max_code = code
            max_size = code[2] * code[3]
    return max_code

# 夹取木块
# 参数：cx为二维码中点x坐标，cy为二维码中点y坐标，c_angle为二维码倾斜角度
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
            if (c_angle - abs(theta0)) <= 45:
                servo_zhuazi = int(1500-2000.0*(c_angle-abs(theta0))/270.0)
            else:
                servo_zhuazi = int(1500+2000.0*(abs(theta0)+90-c_angle)/270.0)
                kms_y = kms_y + 5
        else:
            if (abs(theta0) - c_angle) <= 45:
                servo_zhuazi = int(1500+2000.0*(abs(theta0)-c_angle)/270.0)
                kms_y = kms_y + 5
            else:
                servo_zhuazi = int(1500-2000.0*(90-abs(theta0)+c_angle)/270.0)
    # 当机械臂在偏右位置时
    else:
        #计算爪子旋转角度
        if theta0 + c_angle <= 90:
            if 90 - theta0 - c_angle <= 45:
                servo_zhuazi = int(1500+2000.0*(90-theta0-c_angle)/270.0)
                kms_y = kms_y - 3
            else:
                servo_zhuazi = int(1500-2000.0*(theta0+c_angle)/270.0)
                if kms_y <= img_to_car_mm + img_h_mm/4*3 + kms_y_bias:
                    kms_y = kms_y + 10
                elif kms_y > img_to_car_mm + img_h_mm/7*6 + kms_y_bias:
                    kms_y = kms_y - 15
        else:
            if theta0 + c_angle - 90 <= 45:
                servo_zhuazi = int(1500-2000.0*(theta0+c_angle-90)/270.0)
                #kms_y = kms_y + 1
            else:
                servo_zhuazi = int(1500+2000.0*(180-theta0-c_angle)/270.0)
                #kms_y = kms_y + 5

    #print('$KMS:%s,%s,25,1500!' % (kms_x, kms_y)) # 串口发送机械臂运行到夹取位置的指令
    # 串口发送机械臂运行到夹取位置的指令
    if kms_y <= img_to_car_mm + img_h_mm/2 + kms_y_bias:
        uart3_send('$KMS:%s,%s,25,1500!' % (kms_x, kms_y)) # 放到近处时，目标位置z坐标低一点
    else:
        uart3_send('$KMS:%s,%s,40,1500!' % (kms_x, kms_y)) # 放到远处时，目标位置z坐标高一点
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
    if level == 1:
        level_h = 16
    else:
        level_h = int(wood_h_mm*(level-1)+28)
    # 串口发送机械臂运行到码垛位置的指令
    #print('$KMS:-200,70,%s,2000!' % level_h)
    uart3_send('$KMS:-200,70,%s,2000!' % level_h)
    utime.sleep(2.5)                       # 延时以便让动作执行完成
    uart3_send(claw_open_cmd)              # 爪子张开来放下木块
    utime.sleep(1)                         # 延时以便让动作执行完成
    # 串口发送机械臂运行到码垛位置正上方的指令
    #print('stack up: ', stack_place_up_cmd)
    uart3_send(stack_place_up_cmd)
    utime.sleep(1.5)                       # 延时以便让动作执行完成
    for qr in qr_appear_count:
        qr_appear_count[qr] = 0

# 主函数
def main():
    global systick_ms_bak_zhuan, zhuan_flag, servo_yuntai_bias, level_cur
    while 1:                               # 无限循环
        clock.tick()                       # 更新FPS帧率时钟
        img = sensor.snapshot()            # 拍一张照片并返回图像
        codes = img.find_qrcodes()         # 查找图像中的二维码
        if codes:
            max_code = find_max(codes)     # 查找图像中最大的二维码

            dot_1 = max_code.corners()[0]  # 获取第一个点的坐标
            dot_2 = max_code.corners()[1]  # 获取第二个点的坐标
            dot_3 = max_code.corners()[2]  # 获取第三个点的坐标
            dot_4 = max_code.corners()[3]  # 获取第四个点的坐标

            # 用红线将贴有二维码的木块框出来
            img.draw_line(dot_1[0], dot_1[1], dot_2[0], dot_2[1], color=(255, 0, 0), thickness=5)
            img.draw_line(dot_2[0], dot_2[1], dot_3[0], dot_3[1], color=(255, 0, 0), thickness=5)
            img.draw_line(dot_3[0], dot_3[1], dot_4[0], dot_4[1], color=(255, 0, 0), thickness=5)
            img.draw_line(dot_4[0], dot_4[1], dot_1[0], dot_1[1], color=(255, 0, 0), thickness=5)

            qr_msg = max_code.payload()    # 获取二维码的内容
            qr_w = max_code.w()            # 获取二维码宽度
            qr_h = max_code.h()            # 获取二维码高度

            # 面积合适时
            if qr_w_min < qr_w < qr_w_max and qr_h_min < qr_h:
                qr_appear_count[qr_msg] += 1  # 目标二维码实际出现的次数+1

            # 确认识别到了目标二维码后，开始做对应操作
            # 识别到的二维码大小合适时的次数大于需要识别到的最小次数时
            if qr_appear_count[qr_msg] >= qr_appear_count_min:
                # 记录当前准备堆叠第几层
                level_cur += 1
                # 当堆叠到设置的默认最大层数后，重新从第一层开始堆叠
                if level_cur > level_max:
                    level_cur = 1
                # 计算二维码中点x坐标
                c_x = (dot_1[0] + dot_3[0]) / 2
                #print('c x: ', c_x)
                # 计算二维码中点y坐标
                c_y = (dot_1[1] + dot_3[1]) / 2
                #print('c y: ', c_y)
                # 计算二维码倾斜角度
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
                clamp_wood(c_x,c_y,c_angle)
                # 放置木块到目标区域
                place_wood(level_cur)
                # 放置完木块后，再让机械臂运行到识别位置
                uart3_send(start_position_cmd)
                utime.sleep(2)                      # 延时以便让动作执行完成
                # 让小车保持静止状态
                uart3_send(car_stop_cmd)
                utime.sleep(1)                      # 延时
        # 未识别到二维码时，左右稍微转动，便于识别
        else:
            if millis() - systick_ms_bak_zhuan > 200:
                systick_ms_bak_zhuan = millis()
                if zhuan_flag:
                    servo_yuntai_bias += 3
                    if servo_yuntai_bias >= 3:
                        zhuan_flag = not zhuan_flag
                else:
                    servo_yuntai_bias -= 3
                    if servo_yuntai_bias <= -3:
                        zhuan_flag = not zhuan_flag
                testStr = '#000P{0:0>4d}T{1:0>4d}!'.format(int(1500-2000*servo_yuntai_bias/270),200)
                #print(testStr)
                uart3_send(testStr)

# 程序入口
if __name__ == '__main__':
    init_setup()            # 执行初始化函数
    try:                    # 异常处理
        main()              # 执行主函数
    except:
        pass

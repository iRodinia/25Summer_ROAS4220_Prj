'''
二维码控制小车：
实验效果：
    小车开机后，等待几秒钟后，小车左右稍微晃动一下，说明启动完成，可以开始操作了。
    将二维码放在摄像头前方合适位置（不要太远也不要紧贴着摄像头），小车识别到对应二维码后会执行对应功能。
        识别到前进二维码（$QJ!）时，小车前进1秒，
        识别到后退二维码（$HT!）时，小车后退1秒，
        识别到左转二维码（$ZZ!）时，小车左转1秒，
        识别到右转二维码（$YZ!）时，小车右转1秒。
    本例程中所用的有4个二维码，二维码内容与对应的功能分别为：
          二维码内容   对应功能
            $QJ!       前进
            $HT!       后退
            $ZZ!       左转
            $YZ!       右转
    二维码可以通过草料二维码官网生成：https://cli.im/
'''
import sensor, image, time, lcd, math
import utime
from fpioa_manager import fm
from machine import UART

qr_appear_count_min = 1   #目标二维码至少需要出现的次数默认设置为1次
qr_w_min = 30             #目标二维码宽度至少要达到的最小宽度
qr_h_min = 30             #目标二维码高度至少要达到的最小高度

#要识别的二维码内容
qr_qj = "$QJ!"            #前进对应的二维码内容
qr_ht = "$HT!"            #后退对应的二维码内容
qr_zz = "$ZZ!"            #左转对应的二维码内容
qr_yz = "$YZ!"            #右转对应的二维码内容

#目标二维码实际出现的次数，每次目标二维码出现的次数超过指定最少次数时，才认为真的识别到了目标二维码，是为了防止误识别
qr_appear_count = {
                  qr_qj: 0,   #记录前进二维码出现的次数
                  qr_ht: 0,   #记录后退二维码出现的次数
                  qr_zz: 0,   #记录左转二维码出现的次数
                  qr_yz: 0    #记录右转二维码出现的次数
                  }


#机械臂呈半弯曲状态的指令
jibot_qs_action_commond = "{#000P1500T1500!#001P2000T1500!#002P2000T1500!#003P0850T1500!#004P1500T1500!#005P1200T1500!}"


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
    utime.sleep(3)                       #延时
    #使机械臂呈半弯曲状态，方便摄像头查看物体
    uart2_send(jibot_qs_action_commond)
    utime.sleep(1)                       #延时1秒
    car_run(-450, 450, 70)               #小车左转
    utime.sleep(0.07)                    #延时
    car_run(450, -450, 100)              #小车右转
    utime.sleep(0.1)                     #延时
    car_run(0, 0)                        #小车停止
    utime.sleep(0.5)                     #延时

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


# 查找图像中最大的二维码
def find_max(codes):
    max_size = 0
    for code in codes:
        if code.w() * code.h() > max_size:
            max_code = code
            max_size = code.w() * code.h()
    return max_code


#主函数
def loop_main():
    global code_content_bak
    while 1:
        clock.tick()                    #更新FPS时钟
        img = sensor.snapshot()         #拍张照片并返回图像

        codes = img.find_qrcodes()      #获取图像中的二维码
        #如果图像中有二维码
        if codes:
            max_code = find_max(codes)  #查找图像中最大的二维码
            img.draw_rectangle(max_code.rect(), color=(0, 255, 0), thickness=5)  #用红线将二维码框出来

            qr_msg = max_code.payload() #获取二维码的内容
            img.draw_string(max_code.x(),max_code.y()-20,qr_msg,color=(255,0,0),scale=2)
            #print('二维码的内容为：%s' % qr_msg)    #打印二维码的内容

            qr_w = max_code.w()         #获取二维码宽度
            qr_h = max_code.h()         #获取二维码高度
            #print('w',qr_w,'h',qr_h)

            #面积合适时
            if qr_w_min < qr_w and qr_h_min < qr_h:
                #目标二维码实际出现的次数+1
                qr_appear_count[qr_msg] += 1

            #确认识别到了目标二维码后，开始做对应操作
            #识别到的二维码大小合适时的次数大于需要识别到的最小次数时
            if qr_appear_count[qr_msg] > qr_appear_count_min:
                if qr_msg == qr_qj:
                    car_run(450, 450, 1000)    #小车前进1秒
                elif qr_msg == qr_ht:
                    car_run(-450, -450, 1000)  #小车后退1秒
                elif qr_msg == qr_zz:
                    car_run(-450, 450, 1000)   #小车左转1秒
                elif qr_msg == qr_yz:
                    car_run(450, -450, 1000)   #小车右转1秒
                utime.sleep(1)                 #延时
                #将二维码识别次数重新置为0
                for i in qr_appear_count:
                    qr_appear_count[i] = 0

        lcd.display(img)                #显示画面到LCD屏上

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

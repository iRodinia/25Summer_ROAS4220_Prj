'''
    特殊说明：
    1、手柄启动：等待10秒才可以控制
'''

import re
import time
import _thread
from factory.z_led import ZL_LED
from factory.z_beep import ZL_BEEP
from factory.z_key import ZL_KEY
from factory.z_adc import ZL_ADC
from factory.z_uart import ZL_UART
from factory.z_file import ZL_FILE
from factory.z_servo import ZL_SERVO
from factory.z_kinematics import ZL_KINEMATICS
from factory.z_ps2 import ZL_PS2
import factory.sensor as ZL_AI

#全局变量定义
systick_ms_group_bak = 0
group_next_time = 20

group_start = 0
group_end = 0
group_times = 0

group_start_bak = group_start
group_end_bak = group_end
group_times_bak = group_times

group_ok = 1

systick_ms_save = 0
flag_save = 0

file_action = 0
file_global = 0
servo = 0
pre_cmd = ''


ai_mode = 0  #智能模式切换
'''
ai_mode=0 时，停止
ai_mode=1时，定距跟随
ai_mode=2时，自由避障
ai_mode=3时，智能循迹
ai_mode=4时，循迹避障
'''
systick_ms_ps2 = 0

car_speed = 600   #小车速度，范围0~1000

voice_flag = 0    #语音识别标志位，用于控制语音识别时的基础动作执行时间为2秒

speed_id6_bak = 0
speed_id7_bak = 0
speed_id8_bak = 0
speed_id9_bak = 0
ps2_start_flag = 0


def loop_nled():
    nled.loop_nled()                           # led灯循环亮灭

def loop_ps2():
    global ps2,systick_ms_ps2,ps2_start_flag,ai_mode
    global speed_id6_bak,speed_id7_bak,speed_id8_bak,speed_id9_bak
    if millis() - systick_ms_ps2 > 50:
        systick_ms_ps2 = millis()
        if(ps2.data_is_ready()):
            #处理手柄按键数据
            button_str = ps2.get_str()

            #10秒后手柄按键数据才处理
            if millis() > 3000:
                if(len(button_str) > 2):
                    print('get_str:',button_str)
                    if button_str[0] == '$':
                        #print('cmd:',button_str)
                        if button_str == "$AIMODE!":
                            ai_mode += 1
                            if ai_mode > 4:
                                ai_mode = 0
                                ZL_AI.car_stop()

                            beep.beep_on_times(ai_mode, 0.1)

                            if ai_mode == 3 or ai_mode == 4:
                                uart.uart_send_str('#000P1500T1000!#001P2150T1000!#002P2300T1000!#003P1000T1000!#004P1500T1000!#005P1500T1000!') # 循迹时让机械臂呈弯曲状态

                        parse_cmd(button_str)

                        if button_str == "$DJR!":
                            ai_mode = 0
                            beep.beep_on_times(1, 0.1)
                            uart.uart_send_str('#255P1500T1000!') # 机械臂复位
                    elif button_str == 'ZPY':    # 左摇杆按下时左平移
                        ZL_AI.car_run(0-car_speed, car_speed, car_speed, 0-car_speed, 0)
                    elif button_str == 'YPY':    # 右摇杆按下时右平移
                        ZL_AI.car_run(car_speed, 0-car_speed, 0-car_speed, car_speed, 0)
                    elif button_str == 'PY_TZ':  # 左摇杆或右摇杆抬起时停止
                        ZL_AI.car_stop()
                    else:
                        #print('action:',button_str)
                        parse_action(button_str)

            #红灯模式
            if(ps2.get_mode() == ps2.PS2_LED_RED):
                joysticks_left_x,joysticks_left_y,joysticks_right_x,joysticks_right_y = ps2.get_joysticks()            
                speed_id6 = 1500+int(7.8*(128-joysticks_left_y))
                speed_id7 = 1500-int(7.8*(128-joysticks_right_y))
                
                if(abs(speed_id6-1500) < 150):
                    speed_id6 = 1500
                if(abs(speed_id7-1500) < 150):
                    speed_id7 = 1500
                speed_id8 = speed_id6
                speed_id9 = speed_id7
                
                if speed_id6_bak != speed_id6 or speed_id7_bak != speed_id7 or speed_id8_bak != speed_id8 or speed_id9_bak != speed_id9:
                    speed_id6_bak = speed_id6
                    speed_id7_bak = speed_id7
                    speed_id8_bak = speed_id8
                    speed_id9_bak = speed_id9
      
                    '''
                    if ps2_start_flag == 0:
                        ps2_start_flag = 1
                    '''
                    car_str = '{#006P%04dT0000!#007P%04dT0000!#008P%04dT0000!#009P%04dT0000!}' % (speed_id6,speed_id7,speed_id8,speed_id9)
                    print(car_str)
                    parse_action(car_str)
                    '''
                    if ps2_start_flag == 1:
                        ps2_start_flag = 2
                        beep.beep_on_times(1,0.5)
                    '''

        #key = ps2.data_buf()
        #print("数据：",hex(key[0]),hex(key[1]),hex(key[2]),hex(key[3]),hex(key[4]),hex(key[5]),hex(key[6]),hex(key[7]),hex(key[8]))

# 根据ai_mode的值执行对应的功能
def loop_sensor():
    global ai_mode
    if ai_mode == 1:
        ZL_AI.dingju_gensui()
    elif ai_mode == 2:
        ZL_AI.ziyou_bizhang()
    elif ai_mode == 3:
        ZL_AI.car_xunji()
    elif ai_mode == 4:
        ZL_AI.xunji_bizhang()

def loop_uart():
    global uart, flag_save, systick_ms_save, pre_cmd
    uart.recv_str()                           # 串口接收并根据格式处理数据
    #类似$111！
    if uart.uart_get_ok == 4:
        #指令模式
        parse_cmd(uart.uart_receive_str)   
        uart.uart_receive_str = ''
        uart.uart_get_ok = 0
    #类似#11111！ {#11111！}
    elif uart.uart_get_ok == 2 or uart.uart_get_ok == 3:
        #动作模式
        #uart.uart_send_str(uart.uart_receive_str)
        #PWM相关的解析
        parse_action(uart.uart_receive_str)
        uart.uart_receive_str = ''
        uart.uart_get_ok = 0
    #类似<#11111！> <$11111！>
    elif uart.uart_get_ok == 1:
        #存储模式
        uart.uart_receive_str = uart.uart_receive_str+'\r\n'
        uart.uart_receive_str = uart.uart_receive_str.replace('<', '{')
        uart.uart_receive_str = uart.uart_receive_str.replace('>', '}')
        
        if '$' in uart.uart_receive_str:
            pre_cmd = uart.uart_receive_str
            systick_ms_save = 0
            flag_save = 1
        else:
            #print(uart.uart_receive_str)
            file_action.write(uart.uart_receive_str)
        
        uart.uart_send_str('A')
        uart.uart_receive_str = ''
        uart.uart_get_ok = 0

def parse_action(myaction):
    global flag_save,systick_ms_save,uart,servo
    #处理总线舵机
    uart.uart_send_str(myaction)
    print(myaction,group_next_time)
    
    #处理pwm舵机
    regex = re.compile("[#!]")
    timeList = regex.split(myaction)
    #print(timeList)
    for temp in timeList:
        if len(temp)==13 and ("P" in temp) and ("T" in temp):
            myindex = int(temp[0:3])
            mypwm = int(temp[4:8])
            mytime = int(temp[9:13])
            #print(temp)
            #print('index:',myindex)
            #print(mypwm)
            #print(mytime)
            servo.servo_set(myindex,mypwm,mytime)
    
    #处理类似#000PDST!
    if  len(myaction)>8 and myaction[0] == '#' and myaction[4] == 'P' and myaction[5] == 'D' and myaction[6] == 'S' and myaction[7] == 'T':
        myindex = int(myaction[1:4])
        if myindex<servo.servo_num:
            servo.servo_stop(myindex)
        elif myindex==255:
            for i in range(servo.servo_num):
                servo.servo_stop(i)
    #处理类似#004PSCK+002!
    elif len(myaction)>12 and  myaction[0] == '#' and myaction[4] == 'P' and myaction[5] == 'S' and myaction[6] == 'C' and myaction[7] == 'K':
        myindex = int(myaction[1:4])
        mysck = int(myaction[9:12])
        
        if myaction[8] == '+':
            mysck = mysck
        elif myaction[8] == '-':
            mysck = -mysck
        servo.servo_dict['bias'][myindex] =  mysck
        flag_save = 1
        systick_ms_save = millis()

#处理串口接收到的数据
def uart_data_handle(uart_data):
    global ai_mode, voice_flag, kms
    if '$KMS:' in uart_data:
        aim_x = int(uart_data.split(':')[1].split(',')[0])
        aim_y = int(uart_data.split(':')[1].split(',')[1])
        aim_z = int(uart_data.split(':')[1].split(',')[2])
        aim_time = int(uart_data.split(':')[1].split(',')[3].split('!')[0])
        #print('x,y,z,t ', aim_x, aim_y, aim_z, aim_time)
        if kms.kinematics_move(aim_x, aim_y, aim_z, aim_time):
            text_str = kms.kinematics_move(aim_x, aim_y, aim_z, aim_time)  # 通过逆运动学获得机械臂各舵机需要运行到的pwm值
            uart.uart_send_str(text_str)   # 串口发送对应pwm值使机械臂运行到目标点
            beep.beep_on_times(1,0.1)      # 蜂鸣器响一声
    elif '$WAKE!' in uart_data:
        ZL_AI.car_stop()
        ai_mode = 0
        voice_flag = 1
    elif '$TZ!' in uart_data:
        beep.beep_on_times(1,0.1)   # 蜂鸣器响一声
        #parse_cmd("$DJR!")
        ZL_AI.car_stop()
        ai_mode = 0
    elif '$QJ!' in uart_data:
        ZL_AI.car_run(car_speed, car_speed, car_speed, car_speed, 0)
        if voice_flag == 1:
            time.sleep(2)
            ZL_AI.car_stop()
            voice_flag = 0
    elif '$HT!' in uart_data:
        ZL_AI.car_run(0-car_speed, 0-car_speed, 0-car_speed, 0-car_speed, 0)
        if voice_flag == 1:
            time.sleep(2)
            ZL_AI.car_stop()
            voice_flag = 0
    elif '$ZZ!' in uart_data:
        ZL_AI.car_run(0-car_speed, car_speed, 0-car_speed, car_speed, 0)
        if voice_flag == 1:
            time.sleep(0.5)
            ZL_AI.car_stop()
            voice_flag = 0
    elif '$YZ!' in uart_data:
        ZL_AI.car_run(car_speed, 0-car_speed, car_speed, 0-car_speed, 0)
        if voice_flag == 1:
            time.sleep(0.5)
            ZL_AI.car_stop()
            voice_flag = 0
    elif '$ZPY!' in uart_data:
        ZL_AI.car_run(0-car_speed, car_speed, car_speed, 0-car_speed, 0)
        if voice_flag == 1:
            time.sleep(2)
            ZL_AI.car_stop()
            voice_flag = 0
    elif '$YPY!' in uart_data:
        ZL_AI.car_run(car_speed, 0-car_speed, 0-car_speed, car_speed, 0)
        if voice_flag == 1:
            time.sleep(2)
            ZL_AI.car_stop()
            voice_flag = 0
    elif '$FW!' in uart_data:       # 复位
        uart.uart_send_str('#255P1500T1000!') # 机械臂复位至竖直状态
    elif '$QS!' in uart_data:       # 蜷缩
        uart.uart_send_str('#000P1500T1000!#001P2150T1000!#002P2300T1000!#003P1000T1000!#004P1500T1000!#005P1500T1000!') # 让机械臂呈弯曲状态
    elif '$DJGS!' in uart_data:
        beep.beep_on_times(1,0.1)   # 蜂鸣器响一声
        ai_mode = 1
    elif '$ZYBZ!' in uart_data:
        beep.beep_on_times(1,0.1)   # 蜂鸣器响一声
        ai_mode = 2
    elif '$ZNXJ!' in uart_data or '$XJMS!' in uart_data:
        beep.beep_on_times(1,0.1)   # 蜂鸣器响一声
        uart.uart_send_str('#000P1500T1000!#001P2150T1000!#002P2300T1000!#003P1000T1000!#004P1500T1000!#005P1500T1000!') # 循迹时让机械臂呈弯曲状态
        ai_mode = 3
    elif '$XJBZ!' in uart_data:
        beep.beep_on_times(1,0.1)   # 蜂鸣器响一声
        uart.uart_send_str('#000P1500T1000!#001P2150T1000!#002P2300T1000!#003P1000T1000!#004P1500T1000!#005P1500T1000!') # 循迹时让机械臂呈弯曲状态
        ai_mode = 4

def parse_cmd(mycmd):
    global group_ok, systick_ms_group_bak, group_next_time, group_start, group_start_bak, group_end, group_end_bak, group_times, group_times_bak  
    
    uart_data_handle(mycmd)
    
    if mycmd.find('$DST!') >= 0:
        group_ok = 1
        parse_action("#255PDST!")
            
    elif mycmd.find('$DST:') >= 0:
        index = mycmd[5]
        if(index<6):
            parse_action("#00"+index+"PDST!")
            
    elif mycmd.find('$DGT:') >= 0:
        regex = re.compile("[:-]")
        timeList = regex.split(mycmd)
        #print(timeList)
        group_start = int(timeList[1])
        
        regex = re.compile("[-,]")
        timeList = regex.split(timeList[2])
        #print(timeList)
        group_end = int(timeList[0])
        
        regex = re.compile("[,!]")
        timeList = regex.split(timeList[1])
        #print(timeList)
        group_times = int(timeList[0])
                
        group_start_bak = group_start
        group_end_bak = group_end
        group_times_bak = group_times
        #print(group_start,group_end,group_times)
        group_ok = 0
    # 打印动作
    elif mycmd.find('$PTG:') >= 0:
        regex = re.compile("[:-]")
        timeList = regex.split(mycmd)
        #print(timeList)
        mystart = int(timeList[1])

        regex = re.compile("[-!]")
        timeList = regex.split(timeList[2])
        #print(timeList)
        myend = int(timeList[0])
        
        if mystart == myend:
            mystr = file_action.readline(mystart)
            uart.uart_send_str(mystr)
        elif mystart < myend:
            for index in range(mystart, myend+1):      
                mystr = file_action.readline(index)
                uart.uart_send_str(mystr)
        else:
            for index in range(mystart, myend-1, -1):      
                mystr = file_action.readline(index)
                uart.uart_send_str(mystr)             
    
    elif mycmd == '$GETA!':
        uart.uart_send_str("AAA")
        file_action.clear()
        
    elif mycmd == '$RST!':
        #esp_restart()
        pass

#获取系统时间，毫秒为单位
def millis():
    return int(time.time_ns()//1000000)

#获取动作组中最大时间
def get_max_time(group_str):
    maxNum = 20
    
    regex = re.compile("[T!]")
    timeList = regex.split(group_str)

    for tmp in timeList:
        if len(tmp) == 4:
            #print(int(tmp))
            tmpTime = int(tmp)
            if tmpTime <= 9999 and tmpTime>maxNum:
                maxNum = tmpTime

    return maxNum

#动作组执行一次
def do_group_once(index):
    global group_next_time
    mystr = file_action.readline(index)
    if mystr.find('G') >=0 and mystr.find('#') >=0 and mystr.find('P') >=0 and mystr.find('T') >=0:
        pass
    else:
        return 0
    group_next_time = get_max_time(mystr)
    #print(mystr,group_next_time)
    parse_action(mystr)
    return group_next_time

#动作组解析
def loop_action():
    global group_ok, systick_ms_group_bak, group_next_time, group_start, group_start_bak, group_end, group_end_bak, group_times, group_times_bak  
    if (group_ok == 0) and (millis()- systick_ms_group_bak >= int(group_next_time)):
    
        systick_ms_group_bak = millis()
        print('time:',systick_ms_group_bak)
        group_next_time = do_group_once(group_start)
        
        if(group_next_time==0):
            group_ok = 1
            print("Group Err!")

        if group_start < group_end:
            group_start = group_start+1
            if group_start > group_end:
                if group_times_bak == 0:
                    #无限循环 time值为0
                    pass
                else:
                    group_times = group_times - 1
                    if group_times == 0:
                        group_ok = 1
                        return
                    else:
                        pass
                group_start = group_start_bak
                group_end = group_end_bak
        
        else:
            group_end = group_end-1
            if group_end < group_start:
                if group_times_bak == 0:
                    #无限循环 time值为0
                    pass
                else:
                    group_times = group_times - 1
                    if group_times == 0:
                        group_ok = 1
                        return
                    else:
                        pass
                group_start = group_start_bak
                group_end = group_end_bak        

#监控数据存储
def loop_save():
    global flag_save, file_global, systick_ms_save, servo, pre_cmd
    if flag_save and millis() - systick_ms_save > 3000:
        flag_save = 0
        file_global.clear()
        
        #存储偏差
        for index in range(servo.servo_num):
            #bias0=10
            mystr = 'bias'+str(index)+'='+str(servo.servo_dict['bias'][index])+'\n'
            file_global.write(mystr)
            
            print(mystr)
        
        #存储预存指令
        file_global.write(pre_cmd)
    
def read_data():
    global file_global,servo
    index = 1
    while 1:
        temp = file_global.readline(index)
        if len(temp):
            print(temp)
            #处理偏差bias0=1
            regex = re.compile("[s=]")
            timeList = regex.split(temp)
            if len(timeList)>2:
                myindex = int(timeList[1])
                mybias = int(timeList[2])
                servo.servo_dict['bias'][myindex] = mybias

            #读取预存指令
            if ('$' in temp)  and ('!' in temp):
                parse_cmd(temp[1:-1])
                print(temp[1:-3])
     
            index += 1
        else:
            break

def z_main():
    global nled,beep,key,servo,uart,file_action,file_global,kms,ps2
    nled = ZL_LED()                                    # 实例化一个led灯对象
    beep = ZL_BEEP()                                   # 实例化一个蜂鸣器对象
    key = ZL_KEY()                                     # 实例化按键对象
    servo = ZL_SERVO()                                 # 实例化舵机对象
    uart = ZL_UART()                                   # 实例化串口对象
    file_action = ZL_FILE("factory/action.txt")        # 实例化动作组文件对象
    file_global = ZL_FILE("factory/global.txt")        # 实例化全局变量文件对象
    kms = ZL_KINEMATICS(159,105,75,185)                # 实例化逆运动学
    ps2 = ZL_PS2()                                     # 实例化手柄对象

    #read_data()                                        # 读取偏差和预存指令
    ZL_AI.setup_sensor()
    beep.beep_on_times(3,0.1)                          # 启动完成
    print('main init ok')
    
    # 开机之后机械臂呈蜷缩状态
    uart.uart_send_str('#000P1500T1000!#001P2150T1000!#002P2300T1000!#003P1000T1000!#004P1500T1000!#005P1500T1000!\r\n')
    
    while 1:                                           # 无限循环
        loop_nled()
        loop_uart()
        loop_action()
        loop_save()
        loop_ps2()                                     # 手柄数据读写
        loop_sensor()

# 程序入口
if __name__ == '__main__':
    z_main()

import re
import time
import _thread
from factory.z_led import ZL_LED
from factory.z_beep import ZL_BEEP
from factory.z_uart import ZL_UART
import factory.sensor as ZL_AI

car_speed = 600   #小车速度，范围0~1000

def loop_nled():
    nled.loop_nled()                           # led灯循环亮灭

def loop_uart():
    global uart, flag_save, systick_ms_save, pre_cmd
    uart.recv_str()                            # 串口接收并根据格式处理数据
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
    #print(myaction,group_next_time)
    
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

def parse_cmd(mycmd):
    global group_ok, systick_ms_group_bak, group_next_time, group_start, group_start_bak, group_end, group_end_bak, group_times, group_times_bak  

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

def z_main():
    global nled,beep,key,servo,uart,file_action,file_global,kms,ps2
    nled = ZL_LED()                                    # 实例化一个led灯对象
    beep = ZL_BEEP()                                   # 实例化一个蜂鸣器对象
    uart = ZL_UART()                                   # 实例化串口对象
    ZL_AI.setup_sensor()                               # 初始化传感器
    beep.beep_on_times(3,0.1)                          # 启动完成
    print('main init ok')
    while 1:                                           # 无限循环
        loop_nled()
        loop_uart()
        ZL_AI.xunji_bizhang()                          # 执行循迹避障功能

# 程序入口
if __name__ == '__main__':
    z_main()

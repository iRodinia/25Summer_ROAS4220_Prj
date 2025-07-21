# Set the EXP32 as a WiFi client
import socket
import time
import network

ssid = 'kz_test'
password = '11223344'
# authmode = 4  # WPA2-PSK

def setup_connection():
    # Connect to WiFi first
    wlan = network.WLAN(network.STA_IF)  # STA mode means client
    wlan.active(False)  # clear buffer, then reset
    wlan.active(True)
    if not wlan.isconnected():
        print('connecting to network...')
        wlan.connect(ssid, password)  # WiFi name and pswd
        i = 1
        while not wlan.isconnected():
            print("Trying ... {}".format(i))
            i += 1
            time.sleep(1)
    print('ESP32 ip address:', wlan.ifconfig()[0])
    
    # Then setup the udp host
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind(("0.0.0.0", 7788))
    
    return udp_socket


def handle_data(data_str):
    """
    extract information from data string
    """
    for i, num_str in enumerate(data_str.split()):
        try:
            _num = int(num_str)
            print(f"Received {i}-th numer: {_num}; ")
        except:
            print(f"Received {i}-th invalid command: {num_str}; ")
            

def main():
    udp_socket = setup_connection()
    
    while True:
        recv_data, sender_info = udp_socket.recvfrom(1024)  # buffer size is 1024
        
        # parse the data received
        try:
            recv_data_str = recv_data.decode("utf-8")
            handle_data(recv_data_str)
        except Exception as ret:
            print("data decode error:", ret)
            continue
        finally:
            msg ="Message received!"  # construct a response
            dest_ip = sender_info[0]  # client ip
            dest_port = int(sender_info[1])  # client port
            udp_socket.sendto(msg.encode("utf-8"), (dest_ip, dest_port))
            
    udp_socket.close()
        
    
if __name__ == "__main__":
    main()
    time.sleep(1)
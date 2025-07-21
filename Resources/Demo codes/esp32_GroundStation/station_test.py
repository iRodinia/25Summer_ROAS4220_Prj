import socket
import time

ESP32_IP = "192.168.137.83"  # ESP32 IP address
ESP32_PORT = 7788  # ESP32 IP udp port

def send_udp_data():
    # Create UDP socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_socket.settimeout(5.0)  # 5 second timeout for receiving response
    
    try:
        # Test data to send (space-separated numbers and some invalid data)
        test_messages = [
            "123 456 789",
            "1 2 3 4 5",
            "100 hello 200 world 300",  # Mix of valid and invalid
            "invalid_data",
            "42"
        ]
        
        for i, message in enumerate(test_messages):
            print(f"\n--- Test {i+1} ---")
            print(f"Sending: '{message}'")
            
            # Send data to ESP32
            client_socket.sendto(message.encode('utf-8'), (ESP32_IP, ESP32_PORT))
            
            try:
                # Wait for response from ESP32
                response, server_address = client_socket.recvfrom(1024)
                print(f"Response from {server_address}: {response.decode('utf-8')}")
            except socket.timeout:
                print("No response received (timeout)")
            except Exception as e:
                print(f"Error receiving response: {e}")
            
            time.sleep(1)  # Wait 1 second between messages
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        client_socket.close()
        print("\nClient socket closed")


if __name__ == "__main__":
    print("ESP32 UDP Client Test")
    print("====================")
    
    send_udp_data()
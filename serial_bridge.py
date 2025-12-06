#!/usr/bin/env python3
"""
Bidirectional Serial-UDP Bridge for STM32 ROS2 Bridge
- Serial -> UDP: IMU data from STM32 to ROS2
- UDP -> Serial: cmd_vel from ROS2 to STM32
"""

import socket
import serial
import threading
import sys
import select

COM_PORT = 'COM5'
BAUD_RATE = 921600
UDP_HOST = '192.168.0.65'  # WSL IP
UDP_PORT_STM32_TO_ROS = 8888  # STM32 -> ROS2 (IMU data)
UDP_PORT_ROS_TO_STM32 = 8889  # ROS2 -> STM32 (cmd_vel)

def serial_to_udp(ser, sock_send):
    """Forward data from serial to UDP (STM32 -> ROS2)"""
    try:
        while True:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                if data:
                    sock_send.sendto(data, (UDP_HOST, UDP_PORT_STM32_TO_ROS))
    except Exception as e:
        print(f"Serial->UDP error: {e}")

def udp_to_serial(ser, sock_recv):
    """Forward data from UDP to serial (ROS2 -> STM32)"""
    print("[DEBUG] udp_to_serial thread started, waiting for UDP packets...")
    sock_recv.settimeout(0.1)  # 100ms timeout for blocking recv
    try:
        count = 0
        while True:
            try:
                data, addr = sock_recv.recvfrom(1024)
                if data:
                    count += 1
                    # Send data with newline terminator for STM32
                    ser.write(data)
                    if not data.endswith(b'\n'):
                        ser.write(b'\n')
                        print(f"[UDP->Serial] #{count}: {len(data)}B + \\n -> COM5 from {addr}")
                    else:
                        print(f"[UDP->Serial] #{count}: {len(data)}B (has \\n) -> COM5 from {addr}")
            except socket.timeout:
                continue  # No data, continue loop
    except Exception as e:
        print(f"UDP->Serial error: {e}")

def main():
    print("=== STM32 Bidirectional Serial-UDP Bridge ===")
    print(f"COM Port: {COM_PORT} @ {BAUD_RATE} baud")
    print(f"STM32->ROS2: UDP {UDP_HOST}:{UDP_PORT_STM32_TO_ROS}")
    print(f"ROS2->STM32: UDP listen on :{UDP_PORT_ROS_TO_STM32}")
    print("=" * 50)
    
    ser = None
    sock_send = None
    sock_recv = None
    
    try:
        # Open serial port
        try:
            ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
            print(f"✓ Serial port {COM_PORT} opened")
        except Exception as e:
            print(f"✗ Failed to open {COM_PORT}: {e}")
            sys.exit(1)
        
        # Create UDP socket for sending (to ROS2)
        try:
            sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(f"✓ UDP send socket created")
        except Exception as e:
            print(f"✗ Failed to create UDP send socket: {e}")
            if ser:
                ser.close()
            sys.exit(1)
        
        # Create UDP socket for receiving (from ROS2)
        try:
            sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock_recv.bind(('0.0.0.0', UDP_PORT_ROS_TO_STM32))
            # Timeout will be set in udp_to_serial thread
            print(f"✓ UDP receive socket listening on port {UDP_PORT_ROS_TO_STM32}")
        except Exception as e:
            print(f"✗ Failed to create UDP receive socket: {e}")
            if ser:
                ser.close()
            if sock_send:
                sock_send.close()
            sys.exit(1)
        
        print(f"\n✓ Bidirectional bridge running")
        print(f"  - Forwarding: COM5 -> UDP {UDP_HOST}:{UDP_PORT_STM32_TO_ROS} (IMU)")
        print(f"  - Forwarding: UDP :{UDP_PORT_ROS_TO_STM32} -> COM5 (cmd_vel)")
        print(f"Press Ctrl+C to stop\n")
        
        # Start both forwarding threads
        thread_serial_to_udp = threading.Thread(target=serial_to_udp, args=(ser, sock_send), daemon=True)
        thread_udp_to_serial = threading.Thread(target=udp_to_serial, args=(ser, sock_recv), daemon=True)
        
        thread_serial_to_udp.start()
        thread_udp_to_serial.start()
        
        # Keep main thread alive
        thread_serial_to_udp.join()
        thread_udp_to_serial.join()
        
    except KeyboardInterrupt:
        print("\n✓ Bridge stopped by user")
    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")
    finally:
        # Clean up resources
        print("Cleaning up resources...")
        if ser and ser.is_open:
            ser.close()
            print("✓ Serial port closed")
        if sock_send:
            sock_send.close()
            print("✓ UDP send socket closed")
        if sock_recv:
            sock_recv.close()
            print("✓ UDP receive socket closed")

if __name__ == '__main__':
    main()

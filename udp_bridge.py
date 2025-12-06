#!/usr/bin/env python3
"""
Serial-to-UDP Bridge for micro-ROS agent
Forwards data between Windows COM port and UDP socket for WSL agent
"""

import socket
import serial
import threading
import sys

COM_PORT = 'COM5'
BAUD_RATE = 921600
UDP_PORT = 8888
AGENT_IP = '192.168.0.65'  # WSL IP

last_addr = None

def serial_to_udp(ser, sock):
    """Forward data from serial to UDP"""
    global last_addr
    try:
        while True:
            data = ser.read(ser.in_waiting or 1)
            if data:
                # Send to agent
                sock.sendto(data, (AGENT_IP, UDP_PORT))
                if last_addr:
                    print(f"→ Serial->UDP: {len(data)} bytes to {AGENT_IP}:{UDP_PORT}")
    except Exception as e:
        print(f"Serial->UDP error: {e}")

def udp_to_serial(sock, ser):
    """Forward data from UDP to serial"""
    global last_addr
    try:
        while True:
            data, addr = sock.recvfrom(4096)
            if data:
                last_addr = addr
                ser.write(data)
                print(f"← UDP->Serial: {len(data)} bytes from {addr}")
    except Exception as e:
        print(f"UDP->Serial error: {e}")

def main():
    print("=== micro-ROS UDP Bridge for Windows ===")
    print(f"COM Port: {COM_PORT} @ {BAUD_RATE} baud")
    print(f"UDP: 0.0.0.0:{UDP_PORT} <-> {AGENT_IP}:{UDP_PORT}")
    print("=" * 45)
    
    # Open serial port
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
        print(f"✓ Serial port {COM_PORT} opened")
    except Exception as e:
        print(f"✗ Failed to open {COM_PORT}: {e}")
        sys.exit(1)
    
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', UDP_PORT))
    print(f"✓ UDP socket ready on port {UDP_PORT}")
    
    print(f"\nRun in WSL: MicroXRCEAgent udp4 -p {UDP_PORT} -v6")
    print("Bridge active, forwarding data...\n")
    
    # Start bidirectional forwarding threads
    t1 = threading.Thread(target=serial_to_udp, args=(ser, sock), daemon=True)
    t2 = threading.Thread(target=udp_to_serial, args=(sock, ser), daemon=True)
    t1.start()
    t2.start()
    
    # Keep main thread alive
    try:
        while True:
            threading.Event().wait(1)
    except KeyboardInterrupt:
        print("\n✓ Bridge stopped")

if __name__ == '__main__':
    main()

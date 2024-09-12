import serial.tools.list_ports

def get_serial_ports():
    list_port_info = serial.tools.list_ports.comports()
    return list_port_info

def main():
    print(get_serial_ports())

if __name__ == '__main__':
    main()
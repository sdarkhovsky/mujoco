# https://docs.python.org/3/library/socket.html
# Echo client program
import socket
import sys
import argparse

'''
Examples of commands:
python aisim_client.py -an elbow_left -av 1
python aisim_client.py -an elbow_right -av 2

python aisim_client.py -an shoulder1_right -av 1
python aisim_client.py -an shoulder2_right -av 2
python aisim_client.py -an shoulder2_left -av 3
python aisim_client.py -an knee_right -av 2
python aisim_client.py -an knee_left -av 2
python aisim_client.py -an knee_left -av -2
'''


# A remote or the local host
HOST = 'localhost'  # '127.0.0.1' also works for the local host
PORT = 8080         # The same port as used by the server

def communicate(command_bytes):
    s = None
    for res in socket.getaddrinfo(HOST, PORT, socket.AF_UNSPEC, socket.SOCK_STREAM):
        af, socktype, proto, canonname, sa = res
        try:
            s = socket.socket(af, socktype, proto)
        except OSError as msg:
            s = None
            continue
        try:
            s.connect(sa)
        except OSError as msg:
            s.close()
            s = None
            continue
        break
    if s is None:
        print('could not open socket')
        sys.exit(1)
    with s:
        s.sendall(command_bytes)
        recv_data = s.recv(1024)
    return recv_data

def parse_recv_data(recv_data):
    recv_words = recv_data.decode("utf-8").split(" ")

    iterator = iter(recv_words)
    try:
        while True:
            sensor_name = next(iterator)
            print(sensor_name)
            sensor_dim = next(iterator)
            for i in range(int(sensor_dim)):
                sensor_data = next(iterator)
                print("     ", sensor_data)
    except StopIteration:
        pass

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'aisim commander')
    parser.add_argument('-an', type = str, help='actuator name, see actuator in humanoid.xml, e.g. elbow_right, hip_x_right, etc.')
    parser.add_argument('-av', type=float, help='actuator value')    
    args = parser.parse_args()

    command = args.an + " " + str(args.av)
    send_data = bytes(command, 'utf-8')

    recv_data = communicate(send_data)
    #print('Received', repr(recv_data))
    parse_recv_data(recv_data)


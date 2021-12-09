from socket import socket, AF_INET, SOCK_DGRAM

MYPC_ADDRESS = "172.29.168.59"
MYPC_PORT = 6789

mypc_socket = socket(AF_INET, SOCK_DGRAM)
mypc_socket.bind((MYPC_ADDRESS, MYPC_PORT))

while True:
    array = mypc_socket.recv(1248)

    for x in array:
        print(x, end=" ")
    print("\n")



mypc_socket.close()


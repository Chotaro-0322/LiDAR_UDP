from socket import socket, AF_INET, SOCK_DGRAM

HOST_ADDRESS = "172.29.168.59" # wifi経由先のPCアドレス(今回は俺のデスクトップPC)
HOST_PORT = 6789 # おそらく任意の番号

# RS-Lidar
# LiDAR_ADDRESS = "192.168.1.102" # LiDARのドキュメントを確認
# LiDAR_PORT = 6699 # LiDARのドキュメントを確認

#Velodyne 16
LiDAR_ADDRESS = "255.255.255.255" # LiDARのドキュメントを確認
LiDAR_PORT = 2368 # LiDARのドキュメントを確認


my_socket = socket(AF_INET, SOCK_DGRAM) # ソケットの作成
my_socket.bind((LiDAR_ADDRESS, LiDAR_PORT)) # 受取口のIPとポート番号を設定

host_socket = socket(AF_INET, SOCK_DGRAM) # ソケットの作成

while True:
    array = my_socket.recv(1248) # メッセージを受信 ()中はバッファサイズ

    for x in array:
        print(x, end=" ")
    print("\n")

    host_socket.sendto(array, (HOST_ADDRESS, HOST_PORT))

my_socket.close()
host_socket.close()


from rclpy.node import Node
from socket import socket, AF_INET, SOCK_DGRAM
import numpy as np
import threading
import binascii
from mayavi import mlab

import matplotlib.pyplot as plt

class Lidar_udp_catch(Node):
    def __init__(self):
        super().__init__("lidar_udp_catch")

        self.MYPC_ADDRESS = "172.29.168.59"
        self.MYPC_PORT = 6789

        self.mypc_socket = socket(AF_INET, SOCK_DGRAM)
        self.mypc_socket.bind((self.MYPC_ADDRESS, self.MYPC_PORT))

        print("Build Socket!!!")

        self.udp_thread = threading.Thread(target=self.run)
        self.udp_thread.start()

    def udp_catch(self):
        data_start_pos  = 42 # rs-lidarでのデータの始まりが42byte目
        headffee_size = 2 # blockの区切りであるffeeは2bytes
        azimuth_size = 2 # azimuth(角度)は2bytes
        child_block_size = 2 # 1blockにつき, 2回 1 ~ 16列を検出
        channel_size = 3 # 各点につき, 3bytes格納されている
        block_size = 3*16 * 2 # 1 blockで2回1~16列を検出(1 channel 3 bytes)
        number_of_velodyne = 16 # velodyne16 -> 16, velodyne32 -> 32, rs-lidar16 -> 16
        distance_size = 2 # 距離は2byteで格納されている
        intensity_size = 1 # 反射強度は1byteで格納されている
        laser_angle_list = [-15, -13, -11, -9, -7, -5, -3, -1, 15, 13, 11, 9, 7, 5, 3, 1] # lidarのレーザーの角度の順番
        num_of_block = 12 # blockの数

        current_binary = data_start_pos + headffee_size

        array = self.mypc_socket.recv(1248)
        azimuth_list = []
        laserdist_block_dict = {}
        laserinten_block_dict = {}
        for block in range(num_of_block):
            r = ""
            for bin_num in range(current_binary, current_binary + azimuth_size): # azimuthの算出
                r += "{0:02x}".format(array[bin_num])
            r = bytes.fromhex(r)
            azimuth_deg = int.from_bytes(r, "big") * 0.01 # 角度の算出
            # print("azimuth : ", azimuth_deg)
            azimuth_list.append(azimuth_deg)

            current_binary += azimuth_size
            
            laserdist_col_list = []
            laserinten_col_list = []

            for child_block in range(child_block_size):
                for laser_col in range(number_of_velodyne):
                    r = ""
                    for bin_num in range(current_binary, current_binary + distance_size): # distanceの算出
                        r += "{0:02x}".format(array[bin_num])
                    r = bytes.fromhex(r)
                    distance_m = int.from_bytes(r, "big") * 0.01 # m(メートル)に直す
                    # print("distance_m : ", distance_m)
                    laserdist_col_list.append(distance_m)

                    current_binary += distance_size 

                    r = ""
                    for bin_num in range(current_binary, current_binary + intensity_size): # intensityの算出
                        r += "{0:02x}".format(array[bin_num])
                    r = bytes.fromhex(r)
                    intensity = int.from_bytes(r, "big") # 反射強度を返す
                    # print("intensity : ", intensity)
                    laserinten_col_list.append(intensity)

                    current_binary += intensity_size

            laserdist_block_dict[str(block)] = laserdist_col_list
            laserinten_block_dict[str(block)] = laserinten_col_list
            
            current_binary += headffee_size

        # print("laser_dist_block : ", laserdist_block_dict)
        # print("laser_intensity_block : ", laserinten_block_dict)
        
        # azimuthのデータを補正する必要がある
        for i in range(len(azimuth_list)):
            if i + 2 < len(azimuth_list):
                if azimuth_list[i + 2] < azimuth_list[i]:
                    azimuth_list[i + 2] += 360
                else:
                    azimuth_list[i + 1] = azimuth_list[i + 2] - azimuth_list[i]/2
                if azimuth_list[i + 1] > 360:
                    azimuth_list[i + 1] = azimuth_list[i + 1] - 360

        # print("azimuth list : ", azimuth_list)
        result_dict = {}
        point_block_list = []
        
        x_list = []
        y_list = []
        z_list = []
        for block in range(num_of_block):
            laser_list = laserdist_block_dict[str(block)]
            intensity_list = laserinten_block_dict[str(block)]
            azimuth = azimuth_list[block]

            for i, (dist, intensity) in enumerate(zip(laser_list, intensity_list)):
                if i >= number_of_velodyne:
                    omega = np.radians(laser_angle_list[i - number_of_velodyne]) # 2段目になると16引く必要性 (velodyne16のとき, 16を引く)
                else:    
                    omega = np.radians(laser_angle_list[i])
                alpha = np.radians(azimuth)
                x = dist * np.cos(omega) * np.sin(alpha)
                y = dist * np.cos(omega) * np.cos(alpha)
                z = dist * np.sin(omega)

                point_block_list.append([x, y, z, intensity])
                x_list.append(float(x))
                y_list.append(float(y))
                z_list.append(float(z))

            result_dict[str(block)] = point_block_list
        
        print("x_list : ", x_list)
        # print("result data (x, y, z, intensity) : \n", result_dict)

        print("\n")

        # mlab.points3d(x_list[0], y_list[0], z_list[0], colormap="copper", scale_factor=1)
        mlab.points3d([0], [0], [0], colormap="copper", scale_factor=1)
        mlab.show()

        # plt.scatter(x_list, y_list)
        # plt.xlabel("x")
        # plt.ylabel("y")
        # plt.title("check_point_x_y")
        # plt.show()


    def run(self):
        while True:
       # for i in range(1):
            self.udp_catch()

        self.mypc_socket.close()


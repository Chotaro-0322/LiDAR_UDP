#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>//memset function requires this.
#include <string>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <math.h>
#include <bits/stdc++.h> // M_PIを使うため
#include <unistd.h>

// #include <pcl/point_cloud.h>
// #include <pcl/impl/point_types.hpp>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>

#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>


class LiDARUdp{
        int sock;
        struct sockaddr_in addr;
    public:
        LiDARUdp(std::string address, int port);
        void udp_send(unsigned char *word, int buf_size);
        void udp_bind();
        void udp_recv(unsigned char *buf, int buf_size);
        void analysis_lidar(unsigned char *buf);
        void lidar_pushback();
        
        ~LiDARUdp()
        {
            close(sock);
        }
    private:
        ros::NodeHandle nh;
        ros::Publisher lidar_pub;

        // parameter
        int data_start_pos = 42;
        int headffee_size = 2;
        int azimuth_size = 2;
        int child_block_size = 2;
        int channel_size = 3;
        int block_size = 3*16 * 2;
        int number_of_velodyne = 16;
        int distance_size = 2;
        int intensity_size = 1;
        std::vector<int> laser_angle_list = {-15, -13, -11, -9, -7, -5, -3, -1, 15, 13, 11, 9, 7, 5, 3, 1};
        int num_of_block = 12;
        int max_size_lidarvec = 896; // max_size_lidarvec % number_of_velodyne == 0にしなければならない.(rvizで表示できなくなる)
        int xyz_or_xyzi_size = 4;
        
        // bytesからの変換に必要な変数
        int current_binary;
        int block;
        float azimuth_deg;
        int channel;
        float distance_m;
        float intensity;
        float omega;
        float alpha;
        float x;
        float y;
        float z;
        int num_ffff = 0;
        std::vector<float> azimuth_vec;
        std::vector<float> lidar_X;
        std::vector<float> lidar_Y;
        std::vector<float> lidar_Z;
        std::vector<float> lidar_I;

};

    LiDARUdp::LiDARUdp(std::string address, int port)
    {
            sock = socket(AF_INET, SOCK_DGRAM, 0);
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = inet_addr(address.c_str());
            addr.sin_port = htons(port);

            lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("/points_raw", 1);
    }

    void LiDARUdp::udp_send(unsigned char *word, int buf_size)
    {   
        //sendto(ソケット, 送信するデータ, データのバイト数, フラグ, アドレス情報, アドレス情報のサイズ)
        sendto(sock, word, buf_size, 0, (struct sockaddr *)&addr, sizeof(addr));
    }

    void LiDARUdp::udp_bind()
    {
        bind(sock, (const struct sockaddr *)&addr, sizeof(addr));
    }

    void LiDARUdp::udp_recv(unsigned char *buf, int buf_size)
    {
        memset(buf, 0, buf_size);//バッファを0でクリア
        recv(sock, buf, buf_size, 0);//recv(ソケット, 受信するデータの格納先, データのバイト数, フラグ)
    }

    void LiDARUdp::analysis_lidar(unsigned char *buf)
    {   
        current_binary = data_start_pos + headffee_size;
        for(block = 0; block < num_of_block; block++){
            // azimuth_degの計算
            // azimuth [deg]
            // 00101010　と　00011111の場合, "00101010 00011111" になる. (前のバイトを8bit分ずらす <ビッグエンディアン>)
            azimuth_deg = (buf[current_binary] << 8 | buf[current_binary + 1] ) * 0.01;
            
            current_binary += azimuth_size;

            for(channel = 0; channel < number_of_velodyne; channel++){
                // 1ポイント : 3byte
                // distance [m]
                // printf("azimuth deg : %f\n", azimuth_deg);
                if ((buf[current_binary] << 8 | buf[current_binary + 1]) != 0b1111111111111111)
                {
                    distance_m = (buf[current_binary] << 8 | buf[current_binary + 1]) * 0.01;
                    // intensity (0 ~ 255)
                
                    intensity = buf[current_binary + 2];
                    
                    azimuth_vec.push_back(azimuth_deg);

                    // if (channel > number_of_velodyne){
                    //     omega = laser_angle_list[channel - number_of_velodyne] * M_PI / 180;
                    // }else{
                    //     omega = laser_angle_list[channel] * M_PI / 180;
                    // }
                  
                    omega = laser_angle_list[channel] * M_PI / 180;
                    alpha = azimuth_deg * M_PI / 180;

                    x = distance_m * cos(omega) * sin(alpha);
                    y = distance_m * cos(omega) * cos(alpha);
                    z = distance_m * sin(omega);

                    // データを格納
                    lidar_X.push_back(x);
                    lidar_Y.push_back(y);
                    lidar_Z.push_back(z);
                    lidar_I.push_back(intensity);
                }else{
                    num_ffff += 1;
                }

                // 一定以上ポイントデータを格納したら削除(一周したらデータを削除)
                // if(lidar_X.size() > max_size_lidarvec * xyz_or_xyzi_size * child_block_size){
                // if((lidar_X.size()/2) >= (max_size_lidarvec * number_of_velodyne)){
                if((lidar_X.size() + num_ffff) >= max_size_lidarvec * number_of_velodyne){
                    while (lidar_X.size() % number_of_velodyne != 0){
                        // パディング
                        lidar_X.push_back(0);
                        lidar_Y.push_back(0);
                        lidar_Z.push_back(0);
                        lidar_I.push_back(0);
                    }
                    // printf("lidar size : %d", lidar_X.size() * 4);
                    printf("lidar_X + num_ffff : %d\n", lidar_X.size()+num_ffff);
                    printf("lidar_X.size() : %d\n", lidar_X.size());
                    printf("num_ffff : %d\n", num_ffff);

                    // for (size_t i = 0; i < azimuth_vec.size(); i++){
                    //     std::cout << azimuth_vec[i] << " ";
                    // }
                    std::cout << "\n\n" << std::endl;
                    // printf("lidar_x size : %d", lidar_X.size());
                    lidar_pushback();
                    lidar_X.clear();
                    lidar_Y.clear();
                    lidar_Z.clear();
                    lidar_I.clear();
                    azimuth_vec.clear();
                    num_ffff = 0;
                }
                current_binary += channel_size;
            }

            // printf("binary_result : %hhx, %hhx\n", buf[current_binary], buf[current_binary + 1]);

            current_binary += (number_of_velodyne * channel_size);
            current_binary += headffee_size;
        }
        // printf("\n\n");
    }

    void LiDARUdp::lidar_pushback(){
        pcl::PointCloud<pcl::PointXYZI>::Ptr points_XYZI(new pcl::PointCloud<pcl::PointXYZI>);
        // points_XYZI.reset();
        pcl::PointXYZI pt;
        // printf("lidar size : %d\n", lidar_X.size());
        for(int i = 0; i < lidar_X.size(); i++){
            pt.x = lidar_X[i];
            pt.y = lidar_Y[i];
            pt.z = lidar_Z[i];
            pt.intensity = lidar_I.at(i);
            points_XYZI->push_back(pt);
        }

        sensor_msgs::PointCloud2 result_cloud_msg;
        pcl::toROSMsg(*points_XYZI, result_cloud_msg);
        result_cloud_msg.header.stamp = ros::Time::now();
        result_cloud_msg.header.frame_id = "world";
        result_cloud_msg.height = number_of_velodyne;//number_of_velodyne;
        result_cloud_msg.width = lidar_X.size() / number_of_velodyne; //lidar_X.size() * 4 / number_of_velodyne;
        result_cloud_msg.fields[0].name = "x";
        result_cloud_msg.fields[0].offset = 0;
        result_cloud_msg.fields[0].datatype = 7;
        result_cloud_msg.fields[0].count = 1; //lidar_X.size();
        result_cloud_msg.fields[1].name = "y";
        result_cloud_msg.fields[1].offset = 4;
        result_cloud_msg.fields[1].datatype = 7;
        result_cloud_msg.fields[1].count = 1; //lidar_Y.size();
        result_cloud_msg.fields[2].name = "z";
        result_cloud_msg.fields[2].offset = 8;
        result_cloud_msg.fields[2].datatype = 7;
        result_cloud_msg.fields[2].count = 1; //lidar_Z.size();
        result_cloud_msg.fields[3].name = "intensity";
        result_cloud_msg.fields[3].offset = 16;
        result_cloud_msg.fields[3].datatype = 7;
        result_cloud_msg.fields[3].count = 1; //lidar_I.size();
        result_cloud_msg.point_step = 32;
        // result_cloud_msg.row_step = 56700;//lidar_X.size() * 4;

        lidar_pub.publish(result_cloud_msg);
        // printf("push back!!");
        // sleep(1);
    }

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "lidar_udp_publisher");
    LiDARUdp lidar("192.168.1.102", 6699);//LiDAR: IP adress & Port
    lidar.udp_bind();
    LiDARUdp host("172.29.168.59", 6789);

    unsigned char data[1248];
    
    while (ros::ok())
    {
        lidar.udp_recv(data, sizeof(data));

        lidar.analysis_lidar(data);

        host.udp_send(data, sizeof(data));
    }

    return 0;
}
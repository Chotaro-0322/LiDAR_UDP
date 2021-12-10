#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>//memset function requires this.
#include <string>
#include <iostream>
#include <stdio.h>

class SimpleUdp{
        int sock;
        struct sockaddr_in addr;
    public:
        SimpleUdp(std::string address, int port)
        {
            sock = socket(AF_INET, SOCK_DGRAM, 0);
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = inet_addr(address.c_str());
            addr.sin_port = htons(port);        
        }

        void udp_send(char *word, int buf_size)
        {   
            //sendto(ソケット, 送信するデータ, データのバイト数, フラグ, アドレス情報, アドレス情報のサイズ)
            sendto(sock, word, buf_size, 0, (struct sockaddr *)&addr, sizeof(addr));
        }

        void udp_bind()
        {
            bind(sock, (const struct sockaddr *)&addr, sizeof(addr));
        }

        void udp_recv(char *buf, int buf_size)
        {
            memset(buf, 0, buf_size);//バッファを0でクリア
            recv(sock, buf, buf_size, 0);//recv(ソケット, 受信するデータの格納先, データのバイト数, フラグ)
        }

        void analysis_lidar(char *buf)
        {
            
        }
        
        ~SimpleUdp()
        {
            close(sock);
        }
};

int main(int argc, char **argv)
{
    SimpleUdp lidar("192.168.1.102", 6699);//LiDAR: IP adress & Port
    lidar.udp_bind();
    SimpleUdp host("172.29.168.59", 6789);

    char data[1248];
    
    while (1)
    {
        lidar.udp_recv(data, sizeof(data));

        host.udp_send(data, sizeof(data));
    }

    return 0;
}
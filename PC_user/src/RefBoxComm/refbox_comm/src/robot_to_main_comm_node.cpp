#include <ros/ros.h>

#include<arpa/inet.h> 

#define TCPPORT 9002

using namespace std;

int main(int argc, char** argv) {

    int status, valread, client_fd;
    struct sockaddr_in serv_addr;
    char buffer[1024] = { 0 };
    if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("\n Socket creation error \n");
        return -1;
    }
 
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(TCPPORT);
 
    // Convert IPv4 and IPv6 addresses from text to binary
    // form
    if (inet_pton(AF_INET, "192.168.0.123", &serv_addr.sin_addr)
        <= 0) {
        printf(
            "\nInvalid address/ Address not supported \n");
        return -1;
    }
 
    if ((status
         = connect(client_fd, (struct sockaddr*)&serv_addr,
                   sizeof(serv_addr)))
        < 0) {
        printf("\nConnection Failed \n");
        return -1;
    }

    ros::init(argc, argv, "robot_to_main_comm_node");
    ros::NodeHandle n;

    //CREATE SUBSCRIBERS AND PUBLISHERS TO COMUNICATE WITH ROBOTS

    string msg_to_server;
    std::ostringstream oss;

    ros::Rate r(10);
    while (ros::ok()) {



        std::cout << "write a message to the server";
        cin >> msg_to_server;
        oss << msg_to_server;

        write(client_fd, oss.str().c_str(), oss.str().size());
    
        valread = read(client_fd, buffer,
                    1024 - 1);
        printf("Server message: %s\n", buffer);

        ros::spinOnce();
    }

    // closing the connected socket
    close(client_fd);
    return 0;
}

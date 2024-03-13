//
// Created by shuxy on 24-3-12.
//

#ifndef BUILD_SOCKET_IO_HPP
#define BUILD_SOCKET_IO_HPP



// std
#include "string"
#include "future"
#include "thread"
#include "vector"
#include "memory"


// linux sys
#include "sys/socket.h"
#include "netinet/in.h"
#include "arpa/inet.h"
#include "cstdint"
#include "unistd.h"

//nav interface
#include "nav_client/nav_packet.hpp"
#include "nav_auto_aim_interfaces/msg/target.hpp"
#include "nav_interfaces/msg/socket_msg.hpp"

#include "socket_io.hpp"


namespace nav_client{

    class SocketInterf{
    public:
        explicit SocketInterf();
        ~SocketInterf();

        void initSocket(std::string serv_ip, uint16_t serv_port);

        int sendToServer();

    private:
        int64_t socket_fd_;
        uint16_t serv_port_;
        std::string serv_ip_;
        struct sockaddr_in serv_addr_in;
        char buffer_[256];
    };


}

#endif //BUILD_SOCKET_IO_HPP

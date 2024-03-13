////
//// Created by shuxy on 24-3-12.
////
//
//#ifndef BUILD_SOCKET_IO_HPP
//#define BUILD_SOCKET_IO_HPP
//
//
//#include <netdb.h>
//#include <netinet/in.h>
//#include <sys/socket.h>
//#include <unistd.h>
//
//#include "string"
//#include <cstdio>
//#include <cstring>
//#include <memory>
//
//
//
//
//namespace Io
//{
//#define fp32 float
//
//    enum ROBOT_MODE
//    {
//        ROBOT_NO_FORCE,
//        ROBOT_FOLLOW_GIMBAL,
//        ROBOT_NOT_FOLLOW
//    };
//
//
//    struct Robot_set
//    {
//        uint8_t header;
//        /** chassis_control **/
//        fp32 vx_set = 0.f;
//        fp32 vy_set = 0.f;
//        fp32 wz_set = 0.f;
//
//        /** gimbal_control **/
//        fp32 yaw_set = 0.f;
//        fp32 pitch_set = 0.f;
//
//        /** IMU **/
//        fp32 ins_yaw = 0.f;
//        fp32 ins_pitch = 0.f;
//        fp32 ins_roll = 0.f;
//        fp32 ins_yaw_v = 0.f;
//        fp32 ins_pitch_v = 0.f;
//        fp32 ins_roll_v = 0.f;
//
//        /** other **/
//        fp32 yaw_relative = 0.f;
//        fp32 pitch_relative = 0.f;
//        ROBOT_MODE mode = ROBOT_MODE::ROBOT_NO_FORCE;
//    } __attribute__((packed));
//
//    class Client_socket_interface
//    {
//    public:
//        Client_socket_interface();
//        ~Client_socket_interface();
//        void task();   // send thread
//        //void task1();  // send thread
//        void task2();  // read thread
//        inline void pack();
//
//    private:
//        int64_t port_num, sockfd;
//        sockaddr_in serv_addr, cli_addr;
//        struct hostent *server;
//        char buffer[256];
//        //char buffer_w[256];
//        char buffer_r[256];
//        std::string host_name;
//
//    public:
//    protected:
//
//        Io::Robot_set* p_robot_set = new Io::Robot_set;
//    };
//}  // namespace Io
//
//#endif //BUILD_SOCKET_IO_HPP

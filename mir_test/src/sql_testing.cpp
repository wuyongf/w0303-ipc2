#include <nanodbc/nanodbc.h>
#include <iostream>
#include <exception>

#include <stdio.h>
#include <time.h>

// time
#include <chrono>
#include <thread>

// sql
#include "../include/w0303-ipc2/sql_ubuntu.h"

// glog
#include <glog/logging.h>

std::string getCurrentTimeStr(){
    time_t t = time(NULL);
    char ch[64] = {0};
    char result[100] = {0};
    //2021-01-28 18:02:00.000
    strftime(ch, sizeof(ch) - 1, "%Y-%m-%d %H:%M:%S.000", localtime(&t));
    sprintf(result, "%s", ch);
    return std::string(result);
}


int main(int argc, char *argv[])
try
{
    // glog
    //
    // Start Logging...
    FLAGS_alsologtostderr = true;
    FLAGS_log_dir = "/home/nw/catkin_ws/src/mir_test/logs";
    google::InitGoogleLogging(argv[0]);

//    nanodbc::connection conn("Driver={ODBC Driver 17 for SQL Server};Server=192.168.0.8;Database=NW_mobile_robot_sys;Uid=sa;Pwd=Willsonic2010");

//    nanodbc::connection conn1("Driver={FreeTDS};Server=192.168.0.8;Database=NW_mobile_robot_sys;Uid=sa;Pwd=Willsonic2010");

    nanodbc::connection conn1("Driver={FreeTDS};Server=192.168.0.8;Port=1433;Database=NW_mobile_robot_sys;Uid=sa;Pwd=Willsonic2010");

    auto list = nanodbc::list_drivers();

    std::cout << "connected? " << conn1.connected() << std::endl;


    std::string query_update = "SELECT control_mode FROM sys_status where ID = 1 AND name = 'nw_sys' ";

    std::string sys_control_mode;

    auto result = nanodbc::execute(conn1,query_update);

    while(result.next())
    {
       std::cout << "control mode result: " << result.get<std::string>(0, "null") << std::endl;
    };


    std::cout << conn1.driver_name()<< std::endl;

    conn1.disconnect();

    std::cout << "time now: " << getCurrentTimeStr() << std::endl;

    yf::sql::sql_server sql_ubuntu;

#if 0
    while (true)
    {
        std::cout << "sys control mode: " << sql_ubuntu.GetSysControlMode() << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(200));  // Sleep for 200 ms

        if(sql_ubuntu.GetSysControlMode() == 2 || sql_ubuntu.GetSysControlMode() != 1 || sql_ubuntu.GetSysControlMode() != 3)
        {
            break;
        }
    }
#endif

#if 0
    std::cout << "--------" << std::endl;

    sql_ubuntu.UpdateDeviceConnectionStatus("arm",0);

    std::cout << "--^--^--" << std::endl;
#endif

    sql_ubuntu.GetTaskMode(1);

}
catch (std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
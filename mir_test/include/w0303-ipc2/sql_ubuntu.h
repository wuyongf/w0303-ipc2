//
// Created by nw on 27/2/2021.
//

#pragma once

#include <iostream>
#include <string>
#include <stdio.h>
#include <time.h>
#include <deque>
#include <vector>
#include <chrono>
#include <exception>
#include <thread>

//glog
#include <glog/logging.h>

#include <nanodbc/nanodbc.h>

namespace yf
{
namespace sql
{
class sql_server
{
public:
    sql_server();
    sql_server(const std::string& driver,const std::string& server,const std::string& database,
               const std::string& Uid, const std::string& Pwd );
    ~sql_server();

public:
    // setter
    void setODBCConnectionStr(const std::string &driver, const std::string &server, const std::string &database,
                              const std::string &Uid, const std::string &Pwd);

    // getter
    std::string getODBCConnectionStr();

    std::string get_execute_time();

    // properties
    bool isConnected();

    void Connect();

    void Disconnect();

    // time function
    std::string TimeNow();

    std::vector<float> Time_str2vector(std::string str);

    bool isLatestSchedule(std::string time_from_database, std::string time_now);

    std::string CountdownTime(const std::string& time_now, const int& countdown_min);

    void WaitForExecute(const std::string &execute_time);

public:
    //
    //methods
    //
    // Schedules
    // dataReader
    void RetrieveAllAvailableScheduleId();

    void WaitAvailableSchedules();

    std::deque<int> GetSchedulesId();

    std::deque<int> GetJobsId(const int& cur_schedule_id);

    std::deque<int> GetTasksId(const int& cur_job_id);

    void GetScheduleExecTime(const int& id);

    // Schedule
    //
    void UpdateScheduleData(const int& schedule_id, const int& schedule_status);

    void UpdateScheduleLog(const int& schedule_id, const int& schedule_status);

    // Task
    //
    int GetTaskMode(const int& cur_task_id);
    int GetTaskCommand(const int& cur_task_id);

    void UpdateTaskLog(const int& cur_task_id, const int& task_status);

    // Device Status
    //
    void UpdateDeviceConnectionStatus(const std::string& device_name, const int& connection_status);
    void UpdateDeviceMissionStatus(const std::string& device_name, const int& mission_status);
    void UpdateDeviceBatteryCapacity(const std::string& device_name, const float& battery_capacity);

    // Sys Status
    int GetSysControlMode();

private:

    nanodbc::connection conn_;

    // connection info
    std::string ODBCConnectionStr_ = "" ;
    std::string driver_ = "ODBC Driver 17 for SQL Server" ;
    std::string server_ = "192.168.0.8";
    std::string database_ = "NW_mobile_robot_sys";
    std::string Uid_ = "sa";
    std::string Pwd_ = "Willsonic2010";

    // properties
    std::string time_now_;

    std::deque<int> available_schedules;
    std::deque<int> available_jobs;
    std::deque<int> available_tasks;

    std::string schedule_id_execute_time;

};
}
}


yf::sql::sql_server::sql_server()
{
    //"Driver={SQL Server};Server=192.168.0.8;Database=NW_mobile_robot_system;Uid=sa;Pwd=Willsonic2010"
    ODBCConnectionStr_ = "Driver={"+driver_+"};Server="+server_+";Database="+database_+";Uid="+Uid_+";Pwd="+Pwd_;
}

yf::sql::sql_server::sql_server(const std::string &driver, const std::string &server, const std::string &database,
                                const std::string &Uid, const std::string &Pwd)
        : driver_(driver),  server_(server), database_(database), Uid_(Uid), Pwd_(Pwd)
{
    ODBCConnectionStr_ = "Driver={"+driver_+"};Server="+server_+";Database="+database_+";Uid="+Uid_+";Pwd="+Pwd_;
}

yf::sql::sql_server::~sql_server()
{

}

void yf::sql::sql_server::setODBCConnectionStr(const std::string &driver, const std::string &server, const std::string &database,
                                               const std::string &Uid, const std::string &Pwd)
{
    ODBCConnectionStr_ = "Driver={"+driver+"};Server="+server+";Database="+database+";Uid="+Uid+";Pwd="+Pwd;
}

std::string yf::sql::sql_server::getODBCConnectionStr()
{
    return ODBCConnectionStr_;
}

bool yf::sql::sql_server::isConnected()
{
    return conn_.connected();
}

void yf::sql::sql_server::Connect()
{
    try
    {
        nanodbc::connection connection(ODBCConnectionStr_);
        conn_ = connection;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

void yf::sql::sql_server::Disconnect()
{
    try
    {
        conn_.disconnect();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

// TimeNow(): Ubuntu version
//
std::string yf::sql::sql_server::TimeNow()
{
    time_t t = time(NULL);
    char ch[64] = {0};
    char result[100] = {0};
    //2021-01-28 18:02:00.000
    strftime(ch, sizeof(ch) - 1, "%Y-%m-%d %H:%M:%S.000", localtime(&t));
    sprintf(result, "%s", ch);
    return std::string(result);
}

void yf::sql::sql_server::UpdateScheduleData(const int& schedule_id, const int& schedule_status)
{
    std::string status_str = std::to_string(schedule_status);
    std::string id_str = std::to_string(schedule_id);
    std::string query_update;

    try
    {
        Connect();

        switch (schedule_status)
        {
            case 2: // in processing
            {
                query_update = "UPDATE sys_schedule SET status = " + status_str + ", actual_start='"+TimeNow()+"' WHERE ID = "+ id_str;
                break;
            }

            case 3: // finish
            {
                query_update = "UPDATE sys_schedule SET status = " + status_str + ", actual_end='"+TimeNow()+"' WHERE ID = "+ id_str;
                break;
            }

            case 4: // cancel
            {
                query_update = "UPDATE sys_schedule SET status = " + status_str + ", actual_end='"+TimeNow()+"' WHERE ID = "+ id_str;
                break;
            }

            case 5: // error
            {
                query_update = "UPDATE sys_schedule SET status = " + status_str + ", actual_end='"+TimeNow()+"' WHERE ID = "+ id_str;
                break;
            }

            case 6: // pause
            {
                query_update = "UPDATE sys_schedule SET status = " + status_str + ", actual_end='"+TimeNow()+"' WHERE ID = "+ id_str;
                break;
            }

        }

        nanodbc::execute(conn_,query_update);

        Disconnect();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

void yf::sql::sql_server::UpdateScheduleLog(const int &schedule_id, const int &schedule_status)
{
    std::string status_str = std::to_string(schedule_status);
    std::string schedule_id_str = std::to_string(schedule_id);
    std::string query_update;

    try
    {
        Connect();

        switch (schedule_status)
        {
            case 2: // in processing
            {
                query_update = "INSERT INTO sys_schedule_log(schedule_id, actual_start, status) VALUES (" +
                               schedule_id_str + ",'" + TimeNow() + "'," + status_str + ")";
                break;
            }

            case 3: // finish
            {
                query_update = "UPDATE sys_schedule_log SET status = " + status_str + ", actual_end='" + TimeNow() + "' WHERE schedule_id = " + schedule_id_str + " AND status=2";
                break;
            }

            case 4: // cancel
            {
                query_update = "UPDATE sys_schedule_log SET status = " + status_str + ", actual_end='" + TimeNow() + "' WHERE schedule_id = " + schedule_id_str + " AND status=2";
                break;
            }

                // todo:
                // case 5: // pause

        }

        nanodbc::execute(conn_,query_update);

        Disconnect();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

void yf::sql::sql_server::RetrieveAllAvailableScheduleId()
{
    std::string query_update;

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT ID FROM sys_schedule where status=1 AND planned_start > '"+ TimeNow() +"'";

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            std::string id_str = result.get<std::string>(0, "null");

            int id = std::stoi(id_str);

            available_schedules.push_back(id);
        };

        Disconnect();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

void yf::sql::sql_server::WaitAvailableSchedules()
{
    while (available_schedules.empty())
    {
        RetrieveAllAvailableScheduleId();

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    LOG(INFO) <<  "New schedules available!";
}

std::deque<int> yf::sql::sql_server::GetSchedulesId() {

    std::deque<int> q_schedules_id = available_schedules;

    available_schedules.clear();

    return q_schedules_id;
}

std::deque<int> yf::sql::sql_server::GetJobsId(const int& cur_schedule_id)
{
    std::string query_update;

    std::string schedule_id_str = std::to_string(cur_schedule_id);

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT job_id FROM sys_schedule_job where schedule_id = " + schedule_id_str + " ORDER BY job_order" ;

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            std::string id_str = result.get<std::string>(0, "null");

            int id = std::stoi(id_str);

            available_jobs.push_back(id);

        };

        Disconnect();

        std::deque<int> q_jobs_id = available_jobs;

        available_jobs.clear();

        return q_jobs_id;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

std::deque<int> yf::sql::sql_server::GetTasksId(const int& cur_job_id)
{
    std::string query_update;

    std::string job_id_str = std::to_string(cur_job_id);

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT ID FROM sys_schedule_job_task where job_id = " + job_id_str + " ORDER BY task_order" ;

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            std::string id_str = result.get<std::string>(0, "null");

            int id = std::stoi(id_str);

            available_tasks.push_back(id);

        };

        Disconnect();

        std::deque<int> q_tasks_id = available_tasks;

        available_tasks.clear();

        return q_tasks_id;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }


}

void yf::sql::sql_server::GetScheduleExecTime(const int &id)
{
    std::string query_update;

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT planned_start FROM sys_schedule where ID = "+std::to_string(id)+" AND status=1 AND planned_start > '"+ TimeNow() +"'";

        auto result = nanodbc::execute(conn_,query_update);

        while(result.next())
        {
            std::string time_str = result.get<std::string>(0, "null");
            auto index = time_str.find("+");
            time_str = time_str.substr(0,index-1)+".000";
            schedule_id_execute_time = time_str;
        };

        Disconnect();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

std::string yf::sql::sql_server::get_execute_time()
{
    return schedule_id_execute_time;
}

void yf::sql::sql_server::UpdateDeviceConnectionStatus(const std::string &device_name, const int &connection_status)
{
    std::string connection_status_str = std::to_string(connection_status);
    std::string query_update;

    try
    {
        Connect();

        query_update = "UPDATE sys_status_device SET connection_status = " + connection_status_str + ", modified_date='" + TimeNow() + "' WHERE device = '"+device_name+"'" ;

        nanodbc::execute(conn_,query_update);

        Disconnect();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

void yf::sql::sql_server::UpdateDeviceMissionStatus(const std::string &device_name, const int &mission_status)
{
    std::string mission_status_str = std::to_string(mission_status);
    std::string query_update;

    try
    {
        Connect();

        query_update = "UPDATE sys_status_device SET  mission_status = " + mission_status_str + ", modified_date='" + TimeNow() + "' WHERE device = '" + device_name +"'" ;

        nanodbc::execute(conn_,query_update);

        Disconnect();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

void yf::sql::sql_server::UpdateDeviceBatteryCapacity(const std::string &device_name, const float& battery_capacity)
{
    std::string battery_capacity_str = std::to_string(battery_capacity);
    std::string query_update;

    try
    {
        Connect();

        query_update = "UPDATE sys_status_device SET battery_capacity = " + battery_capacity_str + ", modified_date='" + TimeNow() + "' WHERE device = '"+device_name+"'" ;

        nanodbc::execute(conn_,query_update);

        Disconnect();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

std::vector<float> yf::sql::sql_server::Time_str2vector(std::string str)
{
    std::vector<float> time;

    auto char_index1 = str.find("-");
    auto char_index2 = str.find_last_of("-");

    auto char_index3 = str.find(" ");

    auto char_index4 = str.find(":");
    auto char_index5 = str.find_last_of(":");

    auto char_index6 = str.find(".");

    auto str_year = str.substr(0,char_index1);
    auto str_month = str.substr(char_index1+1, char_index2-char_index1-1);
    auto str_day = str.substr(char_index2+1, char_index3-char_index2-1);
    auto str_hour = str.substr(char_index3+1, char_index4-char_index3-1);
    auto str_min = str.substr(char_index4+1, char_index5-char_index4-1);
    auto str_second = str.substr(char_index5+1, char_index6-char_index5-1);

    std::cout << str_year << " " << str_month << " "<< str_day << " "<< str_hour << " "<< str_min << " "<< str_second << std::endl;

    time.push_back(std::stof(str_year));
    time.push_back(std::stof(str_month));
    time.push_back(std::stof(str_day));
    time.push_back(std::stof(str_hour));
    time.push_back(std::stof(str_min));
    time.push_back(std::stof(str_second));

    return time;
}

bool yf::sql::sql_server::isLatestSchedule(std::string time_from_database, std::string time_now)
{
    std::vector<float> time_db;
    std::vector<float> time_current;

    time_db = Time_str2vector(time_from_database);
    time_current = Time_str2vector(time_now);

    if(time_current[0] > time_db[0])
    {
        return false;
    }
    else
    if(time_current[0] < time_db[0])
    {
        return true;
    }
    else
    {
        if(time_current[1] > time_db[1])
        {
            return false;
        }
        else if(time_current[1] < time_db[1])
        {
            return true;
        }
        else
        if(time_current[2] > time_db[2])
        {
            return false;
        }
        else if (time_current[2] < time_db[2])
        {
            return true;
        }
        else
        if(time_current[3] > time_db[3])
        {
            return false;
        }
        else if (time_current[3] < time_db[3])
        {
            return true;
        }
        else
        if(time_current[4] > time_db[4])
        {
            return false;
        }
        else if(time_current[4] < time_db[4])
        {
            return true;
        }
        else
        if(time_current[5] >=  time_db[5])
        {
            return false;
        }
        else
        {
            return true;
        }
    }
}

void yf::sql::sql_server::WaitForExecute(const std::string &execute_time)
{
    // wait for execute time...
    //
    auto time_now = TimeNow();
    auto time_temp = time_now;
    auto time_scheduled = execute_time;

    // wait for execute time..
    while (isLatestSchedule(time_scheduled, time_temp))
    {
        time_temp = TimeNow();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
}

int yf::sql::sql_server::GetTaskMode(const int &cur_task_id)
{
    std::string query_update;

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT task_mode FROM sys_schedule_job_task where ID = "+std::to_string(cur_task_id)+" AND status=1 ";

        std::string task_mode;

        auto result = nanodbc::execute(conn_,query_update);

        while(result.next())
        {
            task_mode = result.get<std::string>(0, "null");
        };

        Disconnect();

        return std::stoi(task_mode);
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return -999;
    }
}

int yf::sql::sql_server::GetTaskCommand(const int &cur_task_id)
{
    std::string query_update;

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT command FROM sys_schedule_job_task where ID = "+std::to_string(cur_task_id)+" AND status=1 ";

        std::string task_command;

        auto result = nanodbc::execute(conn_,query_update);

        while(result.next())
        {
            task_command = result.get<std::string>(0, "null");
        };

        Disconnect();

        return std::stoi(task_command);
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

void yf::sql::sql_server::UpdateTaskLog(const int &cur_task_id, const int &task_status)
{
    std::string status_str = std::to_string(task_status);
    std::string task_id_str = std::to_string(cur_task_id);
    std::string query_update;

    try
    {
        Connect();

        switch (task_status) {
            case 2: // in processing
            {
                query_update = "INSERT INTO sys_schedule_job_task_log(task_id, actual_start, status) VALUES (" +
                               task_id_str + ",'" + TimeNow() + "'," + status_str + ")";
                break;
            }

            case 3: // finish
            {
                query_update =
                        "UPDATE sys_schedule_job_task_log SET status = " + status_str + ", actual_end='" + TimeNow() +
                        "' WHERE task_id = " + task_id_str + " AND status=2";
                break;
            }

            case 4: // cancel
            {
                query_update =
                        "UPDATE sys_schedule_job_task_log SET status = " + status_str + ", actual_end='" + TimeNow() +
                        "' WHERE task_id = " + task_id_str + " AND status=2";
                break;
            }

                // todo:
                // case 5: // pause
        }

        nanodbc::execute(conn_,query_update);

        Disconnect();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

std::string yf::sql::sql_server::CountdownTime(const std::string& time_now, const int &countdown_min)
{
    auto v_time = Time_str2vector(time_now);

    auto time_countdown =   std::to_string((int)v_time[0]) + "-" +
                            std::to_string((int)v_time[1]) + "-" +
                            std::to_string((int)v_time[2]) + " " +
                            std::to_string((int)v_time[3]) + ":" +
                            std::to_string((int)v_time[4]+countdown_min) + ":" +
                            std::to_string((int)v_time[5]) + ".000";
    return time_countdown;
}

int yf::sql::sql_server::GetSysControlMode()
{
    std::string query_update;

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT control_mode FROM sys_status where ID = 1 AND name = 'nw_sys' ";

        std::string sys_control_mode;

        auto result = nanodbc::execute(conn_,query_update);

        while(result.next())
        {
            sys_control_mode = result.get<std::string>(0, "null");
        };

        Disconnect();

        return std::stoi(sys_control_mode);
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return -999;
    }
}










// connect_database
//
// get_latest_job_task
// get_waiting_schedule_job
// get_current_schedule_task
// get_task_detail_of_job
// get_task_detail
// get_location_info
// get_location_all_cleanzone
// get_clean_zone_detail
// get_location_info
// get_device_current_status
// get_all_waiting_schedule_job
// get_schedule_type

/// update_schedule_status
// update_task_status
// update_job_status
// update_device_status
// update_device_current_status_and_log
/// update_schedule_record
// update_job_record
// update_task_record
// update_schedule_finish_date

// insert_device_log
//

// check_schedule_operation_available
// check_all_schedule_operation_available
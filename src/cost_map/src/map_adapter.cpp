#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/Pose.h>

#include <std_msgs/Int8.h>
#include <typeinfo>
#include <vector>
#include <queue>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <cmath>
#include <iomanip>
using namespace std;

struct MapInfo
{
    float resolution;
    int width;
    int height;
};

struct Cell
{
    int row;
    int column;
};

enum ClearDirection
{
    RIGHT = 0,
    DOWN,
    LEFT,
    UP,
    NOWHERE
};

void go_left(const vector<vector<int>>& cost_data, vector<vector<int>>& visit_data,
             Cell& process_point, vector<vector<int>>& order_data, int& process_order, int& node_count);

void go_right(const vector<vector<int>>& cost_data, vector<vector<int>>& visit_data, const int& mesh_column,
             Cell& process_point, vector<vector<int>>& order_data, int& process_order, int& node_count);

void go_up(const vector<vector<int>>& cost_data, vector<vector<int>>& visit_data, 
             Cell& process_point, vector<vector<int>>& order_data, int& process_order, int& node_count);

void go_down(const vector<vector<int>>& cost_data, vector<vector<int>>& visit_data,
             Cell& process_point, vector<vector<int>>& order_data, int& process_order, int& node_count);

void judge_direction(const vector<vector<int>>& cost_data, const vector<vector<int>>& visit_data, const Cell& process_point,
                     bool& is_start_point, ClearDirection& direction, ClearDirection& prior_direction);

void find_cell(const vector<vector<int>>& cost_data, const vector<vector<int>>& visit_data, const int& mesh_row, const int& mesh_column, Cell& start_point);


void occupancy_grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    vector<signed char> grid_vector = msg->data;
    nav_msgs::MapMetaData map_meta_data = msg->info;

    MapInfo room_map;
    room_map.resolution = map_meta_data.resolution;
    room_map.width = map_meta_data.width; //column of the map
    room_map.height = map_meta_data.height; //row of the map 
    ROS_INFO("map resolution is: [%f] ",room_map.resolution);
    ROS_INFO("map width is: [%d]" , room_map.width);
    ROS_INFO("map height is: [%d]" , room_map.height);

    float x = map_meta_data.origin.position.x;
    float y = map_meta_data.origin.position.y;
    int origin_cell_x = abs(x) / room_map.resolution;
    int origin_cell_y = abs(y) / room_map.resolution;


    float ox = map_meta_data.origin.orientation.x;
    float oy = map_meta_data.origin.orientation.y;
    float oz = map_meta_data.origin.orientation.z;
    float ow = map_meta_data.origin.orientation.w;

    ROS_WARN("the robot's origin position x is: %f", x);
    ROS_WARN("the robot's origin position y is: %f", y);
    ROS_WARN("the robot's origin cell position x is: [%d]", origin_cell_x);
    ROS_WARN("the robot's origin cell position y is: [%d]", origin_cell_y);
    
    ROS_WARN("the robot's origin orientation x is: %f", ox);
    ROS_WARN("the robot's origin orientation y is: %f", oy);
    ROS_WARN("the robot's origin orientation z is: %f", oz);
    ROS_WARN("the robot's origin orientation w is: %f", ow);


    //complete map_data
    vector<vector<int>> map_data(room_map.height);
    ofstream out_file, extract_file, mesh_file, cost_file, order_file;
    out_file.open("map.txt",ios::out);
    int count = 0;
    int flag = 1;
    int first_row, last_row, first_column = room_map.width,last_column = 0;
    for(int i = 0; i < room_map.height; i++)
    {
        for(int j = 0; j < room_map.width; j++)
        {
            if((int)grid_vector[count] != -1 && (int)grid_vector[count] != 0)
            {
                if(flag)
                {
                    first_row = i;
                    flag = 0;
                }
                out_file<<1;
                map_data[i].push_back(1);
                last_row = i;
                if(j > last_column)
                {
                    last_column = j;
                }
                if(j < first_column)
                {
                    first_column = j;
                }
            }
            else if((int)grid_vector[count] == 0)
            {
                map_data[i].push_back(0);
                out_file<<" ";
            }
            else
            {
                map_data[i].push_back(-1);
                out_file<<" ";
            }
            count++;
        }
        out_file<<endl;
    }
    out_file.close();

    ROS_INFO("the map's [%d] data has already put in map.txt file!!!",count);
    ROS_INFO("map file down!!!");
    
    //extract map_data
    extract_file.open("extract.txt", ios::out);
    for(int i = first_row; i <= last_row; i++)
    {
        for(int j = first_column; j<= last_column; j++)
        {
            if(i == origin_cell_y && j == origin_cell_x)
            {
                extract_file<<"*";
                continue;
            }
            if(map_data[i][j] != 0)
            {
                extract_file<<1;
            }
            else
            {
                extract_file<<" ";
            }
            
        }
        extract_file<<endl;
    }
    extract_file.close();
    ROS_INFO("extract file down!!");


    //mesh map_data
    

    int robot_center_x = origin_cell_x;
    int robot_center_y = origin_cell_y;

    int mesh_row = ceil((last_row -  first_row) * 1.0 / 5);
    int mesh_column = ceil((last_column - first_column) * 1.0 / 5);
    ROS_INFO("the map first row is : [%d]",first_row);
    ROS_INFO("the map last row is : [%d]",last_row);
    ROS_INFO("the map first column is : [%d]",first_column);
    ROS_INFO("the map last column is : [%d]",last_column);
    ROS_WARN("the map mesh_row is : [%d]",mesh_row);
    ROS_WARN("the map mesh_column is : [%d]",mesh_column);

    vector<vector<int>> mesh_data(mesh_row);
    int now_row, now_colunm, mesh_origin_cell_x, mesh_origin_cell_y;
    bool is_block = false, x_flag = true, y_flag = true;
    for(int i = 0 ;i < mesh_row; i++)
    {
        for(int j = 0;j < mesh_column; j++)
        {
            is_block = false;
            for(int map_i = 0; map_i < 5 && !is_block; map_i++)
            {
                for(int map_j = 0; map_j < 5 && !is_block; map_j++)
                {
                    now_row = first_row + 5 * i + map_i;
                    if (now_row == origin_cell_y && y_flag)
                    {
                        mesh_origin_cell_y = i;
                        y_flag = false;
                    }
                    now_colunm = first_column + 5 * j + map_j;
                    if(now_colunm == origin_cell_x && x_flag)
                    {
                        mesh_origin_cell_x = j;
                        x_flag = false;
                    }
                    if(map_data[now_row][now_colunm] != 0)
                    {
                        is_block = true;
                    }
                }
            }
            if(is_block)
            {
                mesh_data[i].push_back(1);
            }
            else
            {
                mesh_data[i].push_back(0);
            }
        }
    }


    mesh_file.open("mesh.txt", ios::out);
    for(int i = 0; i < mesh_row; i++)
    {
        for(int j = 0; j < mesh_column; j++)
        {
            if(i == mesh_origin_cell_y && j == mesh_origin_cell_x)
            {
                mesh_file<<"*";
                continue;
            }
            if(mesh_data[i][j] == 0)
            {
                mesh_file<<" ";
            }
            else
                mesh_file<<mesh_data[i][j];   
        }
        mesh_file<<endl;
    }
    mesh_file.close();
    ROS_INFO("mesh file down!!");
    ROS_WARN("the mesh map origin x cell is [%d]", mesh_origin_cell_x);
    ROS_WARN("the mesh map origin y cell is [%d]", mesh_origin_cell_y);
    int math_origin_x = (origin_cell_x - first_column) / 5;
    int math_origin_y = (origin_cell_y - first_row) / 5;
    ROS_WARN("the math  origin x cell is [%d]", math_origin_x);
    ROS_WARN("the math  origin y cell is [%d]", math_origin_y);

    //BFS
    Cell origin_node,temp_node;
    origin_node.row = mesh_origin_cell_y;
    origin_node.column = mesh_origin_cell_x;

    queue<Cell> q;
    vector<vector<int>> cost_data;
    vector<int> temp(mesh_column);
    cost_data.resize(mesh_row,temp);
    for(int i = 0; i < mesh_row; i++)
    {
        for(int j = 0; j < mesh_column; j++)
            cost_data[i][j] = 9999;
    }
    q.push(origin_node);
    int bfs_row,bfs_column,cost = 0,node_count = 1;
    cost_data[mesh_origin_cell_y][mesh_origin_cell_x] = cost;
    cost++;
    ROS_WARN("BFS START");
    while(!q.empty())
    {
        
        bfs_row = q.front().row;
        bfs_column = q.front().column;
        if(mesh_data[bfs_row][bfs_column - 1] == 0 && cost_data[bfs_row][bfs_column - 1] == 9999)
        {
            temp_node.row = bfs_row;
            temp_node.column = bfs_column - 1;
            q.push(temp_node);
            cost_data[bfs_row][bfs_column - 1] = cost_data[bfs_row][bfs_column] + 1;
            node_count++;
        }
        if(mesh_data[bfs_row][bfs_column + 1] == 0 && cost_data[bfs_row][bfs_column + 1] == 9999)
        {
            temp_node.row = bfs_row;
            temp_node.column = bfs_column + 1;
            q.push(temp_node);
            cost_data[bfs_row][bfs_column + 1] = cost_data[bfs_row][bfs_column] + 1;
            node_count++;
        }
        if(mesh_data[bfs_row - 1][bfs_column] == 0 && cost_data[bfs_row - 1][bfs_column] == 9999)
        {
            temp_node.row = bfs_row - 1;
            temp_node.column = bfs_column;
            q.push(temp_node);
            cost_data[bfs_row - 1][bfs_column] = cost_data[bfs_row][bfs_column] + 1;
            node_count++;
        }
        if(mesh_data[bfs_row + 1][bfs_column] == 0 && cost_data[bfs_row + 1][bfs_column] == 9999)
        {
            temp_node.row = bfs_row + 1;
            temp_node.column = bfs_column;
            q.push(temp_node);
            cost_data[bfs_row + 1][bfs_column] = cost_data[bfs_row][bfs_column] + 1;
            node_count++;
        }
        cost++;
        q.pop();
    }
    cost_file.open("cost.txt",ios::out);
    for(int i = 0; i < mesh_row; i++)
    {
        for(int j = 0; j < mesh_column; j++)
        {
            cost_file<<setw(5)<<cost_data[i][j];
        }
            
        cost_file<<endl;
    }
    cost_file.close();
    ROS_INFO("cost_file down!!!!");

    //****************************************************CCPP**************************************************
    vector<vector<int>> visit_data(mesh_data);
    vector<vector<int>> order_data(mesh_data); //visualization

    //vector<Cell> clear_path;

    Cell start_point = origin_node;
    Cell process_point = start_point;

    int process_order = 0;
    order_data[start_point.row][start_point.column] = process_order;
    process_order++;
    visit_data[start_point.row][start_point.column] = 1;
    node_count--;

    bool is_start_point = true;
    ClearDirection direction = NOWHERE;
    ClearDirection prior_direction = NOWHERE;

    while(node_count)
    {
        
        judge_direction(cost_data, visit_data, process_point, is_start_point, direction, prior_direction);
        while(direction != NOWHERE)
        {
            switch(direction)
            {
                case LEFT :
                    go_left(cost_data, visit_data, process_point, order_data, process_order, node_count);
                    break;
                case RIGHT :
                    go_right(cost_data, visit_data, mesh_column, process_point, order_data, process_order, node_count);
                    break;
                case UP :
                    go_up(cost_data, visit_data, process_point, order_data, process_order, node_count);
                    break;
                case DOWN :
                    go_down(cost_data, visit_data, process_point, order_data, process_order, node_count);
                    break;
                default :
                    break;
            }
            judge_direction(cost_data, visit_data, process_point, is_start_point, direction, prior_direction);
        }
        if(node_count)
        {
            find_cell(cost_data, visit_data, mesh_row, mesh_column, start_point);
            process_point = start_point;
            order_data[start_point.row][start_point.column] = process_order;
            process_order++;
            visit_data[start_point.row][start_point.column] = 1;
            node_count--;
            is_start_point = true;
        }
    }

    ROS_WARN("CCPP COMPLETED!!!!!");
    order_file.open("order.txt",ios::out);
    for(int i = 0; i < mesh_row; i++)
    {
        for(int j = 0; j < mesh_column; j++)
        {
            order_file<<setw(5)<<order_data[i][j];
        }
            
        order_file<<endl;
    }
    order_file.close();
    ROS_INFO("order_file down!!!!");
}

void go_left(const vector<vector<int>>& cost_data, vector<vector<int>>& visit_data,
             Cell& process_point, vector<vector<int>>& order_data, int& process_order, int& node_count)
{
    while((process_point.column - 1 >= 0) && (cost_data[process_point.row][process_point.column - 1] != 9999) && (visit_data[process_point.row][process_point.column - 1] == 0))
    {
        process_point.column--;
        order_data[process_point.row][process_point.column] = process_order;
        process_order++;
        visit_data[process_point.row][process_point.column] = 1;
        node_count--;
    }
}

void go_right(const vector<vector<int>>& cost_data, vector<vector<int>>& visit_data, const int& mesh_column,
             Cell& process_point, vector<vector<int>>& order_data, int& process_order, int& node_count)
{
    while((process_point.column + 1 < mesh_column) && (cost_data[process_point.row][process_point.column + 1] != 9999) && (visit_data[process_point.row][process_point.column + 1] == 0))
    {
        process_point.column++;
        order_data[process_point.row][process_point.column] = process_order;
        process_order++;
        visit_data[process_point.row][process_point.column] = 1;
        node_count--;
    }
}

void go_up(const vector<vector<int>>& cost_data, vector<vector<int>>& visit_data, 
             Cell& process_point, vector<vector<int>>& order_data, int& process_order, int& node_count)
{
    process_point.row--;
    order_data[process_point.row][process_point.column] = process_order;
    process_order++;
    visit_data[process_point.row][process_point.column] = 1;
    node_count--;
}

void go_down(const vector<vector<int>>& cost_data, vector<vector<int>>& visit_data,
             Cell& process_point, vector<vector<int>>& order_data, int& process_order, int& node_count)
{
    process_point.row++;
    order_data[process_point.row][process_point.column] = process_order;
    process_order++;
    visit_data[process_point.row][process_point.column] = 1;
    node_count--;
    //todo
}

void judge_direction(const vector<vector<int>>& cost_data, const vector<vector<int>>& visit_data, const Cell& process_point,
                     bool& is_start_point, ClearDirection& direction, ClearDirection& prior_direction)
{
    int left_row, left_column, right_row, right_column, up_row, up_column, down_row, down_column;
    left_row = process_point.row;
    left_column = process_point.column - 1;
    right_row = process_point.row;
    right_column = process_point.column + 1;
    up_row = process_point.row - 1;
    up_column = process_point.column;
    down_row = process_point.row + 1;
    down_column = process_point.column;    
    
    if(is_start_point)
    {
        if(cost_data[right_row][right_column] != 9999 && visit_data[right_row][right_column] == 0)
        {
            direction = RIGHT; //Go Right
            prior_direction = RIGHT;
            is_start_point = false;
            return;
        }

        if(cost_data[left_row][left_column] != 9999 && visit_data[left_row][left_column] == 0)
        {
            direction = LEFT; //Go Left
            prior_direction = LEFT;
            is_start_point = false; 
            return;
        }

        if(cost_data[down_row][down_column] != 9999 && visit_data[down_row][down_column] == 0)
        {
            direction = DOWN; // Go Down
            prior_direction = DOWN;
            is_start_point = false;
            return;
        }

        if(cost_data[up_row][up_column] != 9999 && visit_data[up_row][up_column] == 0)
        {
            direction = UP;  // Go up
            prior_direction = UP;
            is_start_point = false;
            return;
        }
        direction = NOWHERE;
        prior_direction = NOWHERE;
        return;            
    }

    else
    {
        if(direction == RIGHT)// judge if go down or go up
        {
            if(cost_data[down_row][down_column] != 9999 && visit_data[down_row][down_column] == 0)
            {
                direction = DOWN;
                prior_direction = RIGHT;
                return;
            }
            if(cost_data[up_row][up_column] != 9999 && visit_data[up_row][up_column] == 0)
            {
                direction = UP;
                prior_direction = RIGHT;
                return;
            }
            direction = NOWHERE;// no way to go
            return;
        }

        if(direction == LEFT)// judge if go down or go up 
        {
            if(cost_data[down_row][down_column] != 9999 && visit_data[down_row][down_column] == 0)
            {
                direction = DOWN;
                prior_direction = LEFT;
                return;
            }
            if(cost_data[up_row][up_column] != 9999 && visit_data[up_row][up_column] == 0)
            {
                direction = UP;
                prior_direction = LEFT;
                return;
            }
            direction = NOWHERE;
            return;
        }

        if(direction == DOWN)
        {
            if(prior_direction == RIGHT)// before down ,direction is right
            {
                if(cost_data[left_row][left_column] != 9999 && visit_data[left_row][left_column] == 0) // go back (left) first
                {
                    direction = LEFT;
                }
                else if(cost_data[right_row][right_column] != 9999 && visit_data[right_row][right_column] == 0)
                {
                    direction = RIGHT;
                }
                else if(cost_data[down_row][down_column] != 9999 && visit_data[down_row][down_column] == 0)
                {
                    direction = DOWN;
                }
                else
                {
                    direction = NOWHERE;
                }
                prior_direction = DOWN;
                return;
            }
            else
            {
                if(cost_data[right_row][right_column] != 9999 && visit_data[right_row][right_column] == 0)
                {
                    direction = RIGHT;
                }
                else if(cost_data[left_row][left_column] != 9999 && visit_data[left_row][left_column] == 0)
                {
                    direction = LEFT;
                }
                else if(cost_data[down_row][down_column] != 9999 && visit_data[down_row][down_column] == 0)
                {
                    direction = DOWN;
                }
                else
                {
                    direction = NOWHERE;
                }
                prior_direction = DOWN;
                return;
            }
        }

        if(direction == UP)
        {
            if(prior_direction == RIGHT)
            {
                if(cost_data[left_row][left_column] != 9999 && visit_data[left_row][left_column] == 0) // go back (left) first
                {
                    direction = LEFT;
                }
                else if(cost_data[right_row][right_column] != 9999 && visit_data[right_row][right_column] == 0)
                {
                    direction = RIGHT;
                }
                else if(cost_data[up_row][up_column] != 9999 && visit_data[up_row][up_column] == 0)
                {
                    direction = UP;
                }
                else
                {
                    direction = NOWHERE;
                }
                prior_direction = UP;
                return;
            }
            else
            {
                if(cost_data[right_row][right_column] != 9999 && visit_data[right_row][right_column] == 0)
                {
                    direction = RIGHT;
                }
                else if(cost_data[left_row][left_column] != 9999 && visit_data[left_row][left_column] == 0)
                {
                    direction = LEFT;
                }
                else if(cost_data[up_row][up_column] != 9999 && visit_data[up_row][up_column] == 0)
                {
                    direction = DOWN;
                }
                else
                {
                    direction = NOWHERE;
                }
                prior_direction = DOWN;
                return;
            }
        }
    }
}

void find_cell(const vector<vector<int>>& cost_data, const vector<vector<int>>& visit_data, const int& mesh_row, const int& mesh_column, Cell& start_point)
{
    bool find_flag = false; 
    for(int i = 0; i < mesh_row && !find_flag; i++)
    {
        for(int j = 0; j < mesh_column && !find_flag; j++)
        {
            if(cost_data[i][j] != 9999 && visit_data[i][j] == 0)
            {
                start_point.row = i;
                start_point.column = j;
                find_flag = true;
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_adapter");
    ros::NodeHandle n;

    ros::Subscriber map_grid_sub = n.subscribe("map", 10, occupancy_grid_callback);
    ros::spin();
    
    return 0;
}
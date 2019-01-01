#ifndef LOADPARAMETER_H
#define LOADPARAMETER_H

#include <ros/ros.h>

using namespace std;
class Parameter{
    private:
        ros::NodeHandle n;
        int min_walk, max_walk; //untuk menampung nilai minimum dan nilai maksimum motion kaki
        int min_action, max_action;
        std::map<int, std::map<std::string,int> > param_map;
        std::map<int, std::vector<float> > param_action_map[34];
        std::map<int, std::vector<int> > param_action_map_time;

    public:
        Parameter();
        void load_walk_param();
        void load_action_param();
        void getparam(int motion, int &sx, int &sy, int &sz, int &sh, int &sxg, int &syg, int &szg, int &seh, int &abody, int &t);
        void getparam_action(int motion,float (&sMotion)[34][20],int &max_step, int (&t)[20]);
};

Parameter::Parameter()
{
    load_walk_param();
    load_action_param();
}

void Parameter::load_walk_param() //fungsi untuk mengambil data parameter motion walking dari ROS Parameter server
{
    std::map<std::string,int> map;
    char buf[30];
    if(n.getParam("walking_list/min",min_walk))
    {
        ROS_INFO("Loadad Min Walk : %d",min_walk);
    }
    else ROS_INFO("Failed Load Min Walk");
    if(n.getParam("walking_list/max",max_walk))
    {
        ROS_INFO("Loadad Max Walk : %d",max_walk);
    }
    else ROS_INFO("Failed Load Max Walk");

    for(int motion = min_walk; motion<= max_walk; motion++)
    {
        sprintf(buf, "walking_list/motion_%d/",motion);
        if(n.getParam(buf,map))
        {
            param_map[motion]["sx"] = map["sx"];
            param_map[motion]["sy"] = map["sy"];
            param_map[motion]["sz"] = map["sz"];
            param_map[motion]["sh"] = map["sh"];
            param_map[motion]["sxg"] = map["sxg"];
            param_map[motion]["syg"] = map["syg"];
            param_map[motion]["szg"] = map["szg"];
            param_map[motion]["seh"] = map["seh"];
            param_map[motion]["abody"] = map["abody"];
            param_map[motion]["t"] = map["t"];

            /*ROS_INFO("Loaded %s -> sx:%d, sy:%d, sz:%d, sh:%d, sxg:%d, syg:%d, szg:%d, seh:%d, abody:%d, t:%d",
                     buf, param_map[motion]["sx"],param_map[motion]["sy"],param_map[motion]["sz"],param_map[motion]["sh"],param_map[motion]["sxg"],
                     param_map[motion]["syg"],param_map[motion]["szg"],param_map[motion]["seh"],param_map[motion]["abody"],param_map[motion]["t"]);
            */
        } else ROS_INFO("Failed to Load %s",buf);
    }
}

void Parameter::load_action_param() //fungsi untuk mengambil data parameter motion walking dari ROS Parameter server
{
    //std::map<std::string, std::vector<float>> map;
    std::vector<float> data;
    std::vector<int> data_t;
    bool berhasil_load;

    char buf[30];
    if(n.getParam("action_list/min",min_action))
    {
        ROS_INFO("Loadad Min Action : %d",min_action);
    }
    else ROS_INFO("Failed Load Min Action");
    if(n.getParam("action_list/max",max_action))
    {
        ROS_INFO("Loadad Max Action : %d",max_action);
    }
    else ROS_INFO("Failed Load Max Action");

    for(int action = min_action; action<= max_action; action++)
    {
        berhasil_load = true;
        //ROS_INFO("action %d",action);
        for(int id = 1; id<=34; id++)
        {
            sprintf(buf, "action_list/motion_%d/id%d",action,id);
            ROS_INFO(buf);
            if(n.getParam(buf,data))
            {
                //ROS_INFO("Loadad data action_list/motion_%d/id%d",action,id);
                param_action_map[action-min_action][id] = data;
                ROS_INFO("Loadad data action_list/motion_%d/id%d",action,id);
            }
            else
            {
                berhasil_load = false;
                ROS_INFO("action %d Gagal",action);
            }

        }
        sprintf(buf, "action_list/motion_%d/t",action);
        if(n.getParam(buf,data_t))
        {
            param_action_map_time[action-min_action] = data_t;
            ROS_INFO("Loadad data action_list/motion_%d/TTTTTTT",action);
        }
        else berhasil_load = false;

        if(berhasil_load == true) ROS_INFO("Loadad data action_list/motion_%d/",action);
        else ROS_INFO("Failed to load data action_list/motion_%d/",action);
    }
}

void Parameter::getparam_action(int motion, float (&sMotion)[34][20],int &max_step, int (&t)[20])
{
    for(int id=1; id<=20; id++)
    {
        std::vector<float> data;
        data = param_action_map[motion-min_action][id];
        max_step = data.size();
        for(int step=0; step < data.size(); step++)
        {
            if (motion == 0){
                sMotion[id][step] = 0; t[step]=1;
            } else {
                sMotion[id][step] = data[step];
            }
            /*else if(motion == 100){
                sMotion[id][step] = data[step];
            } else if(motion == 101){
                sMotion[id][step] = data[step];
            } else if(motion == 102){
                sMotion[id][step] = data[step]; \
                //ROS_INFO("MAX Step : %d", max_step);
            }*/
            if(id == 15) ROS_INFO("Step : %d V : %g", step,data[step]);
        }
    }
    std::vector<int> data_t;
    data_t = param_action_map_time[motion-min_action];
    for(int step=0; step < data_t.size(); step++)
    {
        t[step] = data_t[step];
        //ROS_INFO("t[%d] = %d", step, t[step]);
    }
}

void Parameter::getparam(int motion, int &sx, int &sy, int &sz, int &sh, int &sxg, int &syg, int &szg, int &seh, int &abody, int &t)
{
    sx = param_map[motion]["sx"];
    sy = param_map[motion]["sy"];
    sz = param_map[motion]["sz"];
    sh = param_map[motion]["sh"];
    sxg = param_map[motion]["sxg"];
    syg = param_map[motion]["syg"];
    szg = param_map[motion]["szg"];
    seh = param_map[motion]["seh"];
    abody = param_map[motion]["abody"];
    t = param_map[motion]["t"];
}

#endif // LOADPARAMETER_H

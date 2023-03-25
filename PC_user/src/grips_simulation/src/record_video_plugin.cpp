#include <grips_simulation/record_video_plugin.h>
#include <iostream>

#include <gazebo/gui/GuiIface.hh>

using namespace gazebo;

RecordVideoPlugin::RecordVideoPlugin()
{
    this->recording = false;
    this->saved = false;
}

RecordVideoPlugin::~RecordVideoPlugin()
{
    std::cout << "RecordVideoPlugin::Destroying RecordVideoPlugin!" << std::endl;
    this->connections.clear();
}

void RecordVideoPlugin::Load(int _argc, char **_argv)
{
    std::cout << "RecordVideoPlugin::Load!" << std::endl;
}

void RecordVideoPlugin::Init()
{
    std::cout << "RecordVideoPlugin::Init!" << std::endl;
    this->connections.push_back(
          event::Events::ConnectPreRender(
            std::bind(&RecordVideoPlugin::Update, this)));   
    this->node = transport::NodePtr(new transport::Node());            
    this->node->TryInit(common::Time::Maximum());
    this->statsSub = this->node->Subscribe(
      "~/world_stats", &RecordVideoPlugin::OnStats, this); 
}

void RecordVideoPlugin::OnStats(ConstWorldStatisticsPtr &_msg) 
{
    if (_msg->paused() == true && this->saved == false) 
    {
        this->camera_mutex.lock();
        std::cout << "Sim paused, saving video!" << std::endl;
        std::cout << "RecordVideoPlugin::OnStats going to call save!" << std::endl;
        if (gui::get_active_camera()->SaveVideo("simulation.mp4"))
        {
           std::cout << "RecordVideoPlugin::Saved Video!" << std::endl;
           this->saved = true;
        } 
        else
        {
            std::cerr << "RecordVideoPlugin::Error saving Video!" << std::endl;            
        }
    this->camera_mutex.unlock();
    }
}

void RecordVideoPlugin::Update()
{
    if(this->recording == true) 
    {
        return;
    }
    //std::cout << "RecordVideoPlugin::Update inner!" << std::endl;
    // Get the user camera, and start recording in the specified format
    this->camera_mutex.lock();
    if(!gui::get_active_camera()) {
                std::cout << "RecordVideoPlugin::Unable to get pointer to user camera." << std::endl;
                this->camera_mutex.unlock();
                return;
    }
    if (gui::get_active_camera()->StartVideo("mp4", "simulation_tmp.mp4"))
    {
        std::cout << "RecordVideoPlugin::Started recording the simulation!" << std::endl;
        this->recording = true;
    }
    else 
    {
        std::cerr << "RecordVideoPlugin::Error starting reording video!" << std::endl;
    }
    this->camera_mutex.unlock();
}

GZ_REGISTER_SYSTEM_PLUGIN (RecordVideoPlugin);




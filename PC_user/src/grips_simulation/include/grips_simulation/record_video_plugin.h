
#ifndef RECORD_VIDEO_PLUGIN_
#define RECORD_VIDEO_PLUGIN

#include <mutex>

#include <gazebo/gazebo.hh>
#include <gazebo/transport/Node.hh>
#include "gazebo/rendering/rendering.hh"

//see http://gazebosim.org/tutorials?tut=system_plugin
namespace gazebo
{
    class RecordVideoPlugin : public SystemPlugin
    {
        public:
          RecordVideoPlugin();
          ~RecordVideoPlugin();
          void Load(int _argc = 0, char **_argv = nullptr) override;
          void Init() override;          
          void Update();
          void OnStats(ConstWorldStatisticsPtr &_msg);
        private:
            bool recording;
            bool saved;
            std::vector<event::ConnectionPtr> connections;
            transport::NodePtr node;
            transport::SubscriberPtr statsSub;
            gazebo::rendering::UserCameraPtr camera;
            std::mutex camera_mutex;
    };
}

#endif //RECORD_VIDEO_PLUGIN_

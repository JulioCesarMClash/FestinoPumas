#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <ollama_ros/CommandPlan.h> 

// Bibliotecas de festino
#include <festino_tools/FestinoHRI.h>
#include <festino_tools/FestinoVision.h>
#include <festino_tools/FestinoNavigation.h>
#include <festino_tools/FestinoKnowledge.h>
#include <festino_tools/FestinoHardware.h>

#include <nlohmann/json.hpp>

enum SMState
{
        SM_INIT,
        SM_WAIT_FOR_DOOR,
        SM_SAY_OPEN_DOOR,
        SM_NAVIGATE_TO_START_POINT,
        SM_WAIT_FOR_INSTRUCTION,
        SM_PARSE_COMMAND,
        SM_SAY_COMMAND,
        SM_FINISH_TEST
};

// VARIABLES GLOBALES
// Banderas
bool success = false;
bool flag_command = true;

// Contadores

// Para el movimiento de la cabeza
float pitchAngle;

// Variables auxiliares de navegación
float robot_y, robot_x, robot_a;    
float gx_w, gy_w, gz_w, guest_z, host_z;    
float goalx, goaly, goala;
float dist_to_head;
float theta = 0, thetaToGoal = 0, angleHead = 0;
float pointingArmX, pointingArmY, pointingArmZ;
float pointingDirX, pointingDirY, pointingDirZ, pointingNormal;
float distanceArm = 0.6;

//Strings aux

std::stringstream ss;
std::stringstream ss2;
std::vector <std::string> objects;
std::string instruction;
std::string input;
std::string result;
//ros::NodeHandle nh;

SMState state = SM_INIT;

ollama_ros::CommandPlan srv;

// Variables de inicio
std::vector<float> goal_vec(3);

using json = nlohmann::json;


std::string generateActionPhrase(const std::string& json_str) {
    json data = json::parse(json_str);

    std::string action = data.value("action", "");
    std::string target = data.value("target", "");
    std::string location = data.value("location", "");
    std::string destination = data.value("destination", "");
    std::string secondary_action = data.value("secondary_action", "");

    std::string phrase = "I will to ";

    // Acción principal
    if (action == "count") {
        phrase += "count how many " + target;
    } else if (action == "find") {
        phrase += "find the " + target;
    } else if (action == "follow") {
        phrase += "follow the " + target;
    } else if (action == "go") {
        phrase += "go to the " + location;
        // Si sólo es ir a un lugar, ya podemos terminar
        return phrase + ".";
    } else {
        phrase += action + " the " + target;
    }

    // Lugar donde se ejecuta la acción
    if (!location.empty()) {
        phrase += " in the " + location;
    }

    // Destino si aplica
    if (!destination.empty()) {
        phrase += ", and then go to the " + destination;
    }

    // Acción secundaria si existe
    if (!secondary_action.empty()) {
        phrase += ", and then " + secondary_action + " the " + target;
    }

    phrase += ".";

    return phrase;
}

bool hasValidAction(const std::string& json_str)
{
    try {
        json data = json::parse(json_str);
        std::string action = data.value("action", "");
        return !action.empty();
    } catch (const json::parse_error& e) {
        // Error al parsear el JSON
        return false;
    } catch (...) {
        // Cualquier otro error
        return false;
    }
}

struct PlanData {
    std::string action;
    std::string target;
    std::string location;
    std::string destination;
    std::string secondary_action;
    bool valid = false;  // Se pone en true si el action no está vacío
};


PlanData extractPlanParts(const std::string& json_str) {
    PlanData plan;
    try {
        json data = json::parse(json_str);

        plan.action = data.value("action", "");
        plan.target = data.value("target", "");
        plan.location = data.value("location", "");
        plan.destination = data.value("destination", "");
        plan.secondary_action = data.value("secondary_action", "");

        plan.valid = !plan.action.empty();
    } catch (...) {
        // Si hay error, retorna el plan inválido
        plan.valid = false;
    }

    return plan;
}

int main(int argc, char **argv)
{
    std::cout << "INITIALIZING ACT_PLN BY JOSHUA M... Aaaaaaaiudaaaaa" << std::endl;
    ros::init(argc, argv, "storing_groseries_test");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    //FestinoTools
    FestinoHRI::setNodeHandle(&nh);
    FestinoNavigation::setNodeHandle(&nh);
    FestinoVision::setNodeHandle(&nh);
    FestinoKnowledge::setNodeHandle(&nh);
    FestinoHardware::setNodeHandle(&nh);


    //ESTO NO DEBERIA IR ASI XDDD
    ros::ServiceClient client = nh.serviceClient<ollama_ros::CommandPlan>("parse_command");

    FestinoHRI::say(" ",1);

    while(ros::ok() && !success)
    {
    	switch(state)
    	{
            case SM_INIT:
                        std::cout << "SM_INIT --> Start GPSR :)" << std::endl;
                        FestinoHRI::say("I'm ready for G.P.S.R. test", 3);
                        FestinoHardware::setHeadOrientation(0.0, 0.0);
                        state = SM_WAIT_FOR_DOOR;
                        break;

                case SM_WAIT_FOR_DOOR:
                        std::cout << "SM_WAIT_FOR_DOOR --> I'm waitig for the door is open" << std::endl;
                        //state = FestinoNavigation::waitForDoor() ?  SM_NAVIGATE_TO_START_POINT : SM_NAVIGATE_TO_START_POINT;
                        state = SM_NAVIGATE_TO_START_POINT;
                        break;

                case SM_SAY_OPEN_DOOR:
                        std::cout << "SM_SAY_OPEN_DOOR --> I'm saying to human that open the door" << std::endl;
                        FestinoHRI::say("Human, please open the door,", 3);
                        state = SM_WAIT_FOR_DOOR;
                        break;

                case SM_NAVIGATE_TO_START_POINT:
                        std::cout << "SM_NAVIGATE_TO_START_POINT --> I'm navigating to the start point" << std::endl;
                        FestinoHRI::say("I will navigate to the start point,", 3);
                        
                        //goal_vec = FestinoKnowledge::CoordenatesLocSrv("start_point");
                        //std::cout <<"Coordenates of start point:"<<std::endl;
                        //std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                        //if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                        //    std::cout << "Cannot move to start point" << std::endl; 
                        
                        FestinoHRI::say("I have arrived to start point",3);
                        
                        state = SM_WAIT_FOR_INSTRUCTION;
                        break;

                case SM_WAIT_FOR_INSTRUCTION:
                        std::cout << "SM_WAIT_FOR_INSTRUCTION --> I'm wait for instruction" << std::endl;
                        FestinoHRI::say("Please, put the QR Code", 3);
                        instruction = FestinoVision::enableQRDetect(true);
                        std::cout << "SM_WAIT_FOR_INSTRUCTION --> QR: " << instruction << std::endl;
                        state = (instruction != "") ? SM_PARSE_COMMAND : SM_WAIT_FOR_INSTRUCTION;
                        break;

                case SM_PARSE_COMMAND:
                        std::cout << "SM_PARSE_COMMAND --> I'm parsing the command" << std::endl;

                        srv.request.command = instruction;
                        if (client.call(srv))
                        {
                            input = srv.response.plan_json.c_str();
                            std::cout << "Response: " << input << std::endl;
                            flag_command = true;
                        }
                        else
                        {
                            std::cout << "Failed to call service parse_command" << std::endl;
                            flag_command = false;
                        }

                        if(hasValidAction(input))
                        {
                            state = SM_SAY_COMMAND;
                        }
                        else
                        {
                            state = SM_PARSE_COMMAND;
                        }
                        break;

                case SM_SAY_COMMAND:
                        std::cout << "SM_SAY_COMMAND --> I'm saying the command" << std::endl;
                        FestinoHRI::say(generateActionPhrase(input), 4);
                        state = SM_FINISH_TEST;
                        break;

                case SM_FINISH_TEST:
                        std::cout << "SM_FINISH_TEST --> I finish the test: wuuuuu :)" << std::endl;
                        success = true;
                        FestinoHRI::say("I have finished the test... wuuu",3);  

                        
        }

    }

}
#include <smc_grippers/smcService.h>

smcService::smcService()
{
}

smcService::~smcService()
{
}
/* 
bool smcService::gripperCallback(smc_grippers::gripper_service::Request &req, smc_grippers::gripper_service::Response &res)
{
    gripper->enable();
    int pos = req.position;
    gripper->setTargetPosition(500);
    gripper->disable();
    return 1;
}

bool smcService::pressCallback(smc_grippers::press_service::Request &req, smc_grippers::press_service::Response &res)
{
    press->enable();
    int pos = req.position;
    press->setTargetPosition(500);
    press->disable();
    return 1;
}

bool smcService::BgripperCallback(smc_grippers::Bgripper_service::Request &req, smc_grippers::Bgripper_service::Response &res)
{
    Bgripper->enable();
    int pos = req.position;
    Bgripper->setTargetPosition(500);
    Bgripper->disable();
    return 1;
} */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "smcService");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0); // Define Multithreadedspinner
    spinner.start();

    smcService ss;
    // Advertising all Services
   /*  ros::ServiceServer gripperService = nh.advertiseService("/smcService/gripper", &smcService::gripperCallback, &ss);
    ros::ServiceServer BgripperService = nh.advertiseService("/smcService/3bGripper", &smcService::BgripperCallback, &ss);
    ros::ServiceServer pressService = nh.advertiseService("/smcService/press", &smcService::pressCallback, &ss); */

    // Establish ethercat Communication
    EtherCAT etherCAT("10.0.0.2");
    CoE coe(etherCAT, 0.005);
    try
    {
        SMCServoJXCE1 gripper(etherCAT, coe, 0x0000);
       /*  SMCServoJXCE1 press(etherCAT, coe, 0xFFFF);
        SMCServoJXCE1 Bgripper(etherCAT, coe, 0xFFFE); */
    }
    catch (exception &e)
    {
        std::cout << e.what() << endl;
    }

    ros::waitForShutdown();

    return 0;
}
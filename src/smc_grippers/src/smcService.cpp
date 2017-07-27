#include <smc_grippers/smcService.h>

smcService::smcService()
{
}

smcService::~smcService()
{
}

bool smcService::gripperCallback(smc_grippers::gripper_service::Request &req, smc_grippers::gripper_service::Response &res)
{
    gripper->enable();
    int pos = req.position;

    gripper->setTargetPosition(500);
    gripper->disable();
    return 1;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "smcService");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0); // Define Multithreadedspinner
    spinner.start();

    smcService ss;
    ros::ServiceServer gripperService = nh.advertiseService("gripper_service", &smcService::gripperCallback, &ss);

    EtherCAT etherCAT("10.0.0.205");
    CoE coe(etherCAT, 0.005);
    SMCServoJXCE1 gripper(etherCAT, coe, 0x0000);

    ros::waitForShutdown();

    return 0;
}
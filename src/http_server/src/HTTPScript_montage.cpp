#include <http_server/HTTPScript_montage.h>


bool HTTPScript_montage::callService(double Offset_, int Ausgabestelle_)
{
    ROS_INFO("Calling Service:");
    cell_core::montage_service srv;
    srv.request.Offset = Offset_;
    srv.request.Ausgabestelle = Ausgabestelle_;
    if (clientPtr->call(srv))
    {
        return 1;
    }
    else
    {
        ROS_WARN("Could not launch montageservice");
        return 0;
    }
}

string HTTPScript_montage::call(vector<string> names, vector<string> values)
{
    string response;
    std::string::size_type sz;
    double offset_ = std::stod(values[0], &sz);
    callService(0.0, 1);

    if (true)
    {
        return response;
    }
    else
    {
        return response;
    }

    return response;
}

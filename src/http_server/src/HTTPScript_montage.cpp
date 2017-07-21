#include <http_server/HTTPScript_montage.h>

HTTPScript_montage::HTTPScript_montage(){
    SrvClient = nh.serviceClient<cell_core::montage_service>("montage_service");
}


bool HTTPScript_montage::callService(double Offset_, int Ausgabestelle_)
{
    ROS_INFO("Calling Service:");
    cell_core::montage_service srv;
    srv.request.Offset = Offset_;
    srv.request.Ausgabestelle = Ausgabestelle_;
    if (SrvClient.call(srv))
    {
        return 1;
    }
    else
    {
        ROS_WARN("Could not launch montageservice");
        return 0;
    }
    return 0;
}

string HTTPScript_montage::call(vector<string> names, vector<string> values)
{
    string response;
    std::string::size_type sz;
    double offset_ = std::stod(values[0], &sz);
    

    if (callService(0.0, 1))
    {
        return "Funktioniert";
    }
    else
    {
        return "GAT NÖD!";
    }

    return "";
}

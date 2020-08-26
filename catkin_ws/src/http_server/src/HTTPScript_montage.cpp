#include <http_server/HTTPScript_montage.h>

HTTPScript_montage::HTTPScript_montage()
{
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

    // Converting from String to int/double
    double offset_ = std::stod(values[0]);
    int ausgabe_ = std::stoi(values[1], &sz);

    // Callig service
    int result = callService(offset_, ausgabe_);

    if (result == 0)
    {
        return "Offset out of bounds or Ausgabestelle not correct";
    }
    else if (result == 1)
    {
        return "Montage finished";
    }
    else if (result == 2)
    {
        return "Pen montage not Idle";
    }

    return "";
}

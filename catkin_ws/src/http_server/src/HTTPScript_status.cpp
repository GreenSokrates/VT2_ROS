#include <http_server/HTTPScript_status.h>


HTTPScript_status::HTTPScript_status()
{
    StatusSub = nh.subscribe("status_chatter", 1000, &HTTPScript_status::statusCallback, this);
}

void HTTPScript_status::statusCallback(const cell_core::status_msg &msg)
{
    this->idle_ = msg.idle;
    this->error_ = msg.error;
    ROS_INFO("Updated the Status of Montage!");
}



string HTTPScript_status::call(vector<string> names, vector<string> values)
{
    string response;

    response += " <h2> Montagestus ist: </h2>";
    if (idle_)
    {
        response += "<p>Idle = True</p>";
    }
    else
    {
        response += "<p>Idle = False</p>";
    }
    if (error_)
    {
        response += "<p>ERROR = True</p>";
    }
    else
    {
        response += "<p>ERROR = False</p>";
    }

    return response;
}

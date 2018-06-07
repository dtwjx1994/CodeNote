#include <ros/ros.h>
#include <std_srvs/SetBool.h>
int main(int argc,char** argv){
    ros::init(argc,argv,"srv_call");
    ros::NodeHandle n;
    // arg1 服务名，arg2 回调函数
    ros::ServiceClient serviceClient=n.serviceClient<std_srvs::SetBool>("trigger_on");
    std_srvs::SetBool request;
    request.request.data= false;
    if(serviceClient.call(request)){
        std::cout<<request.response.message;
        std::cout<<"--- ";
    }
    return 0;
}

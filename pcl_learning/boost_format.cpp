//
// Created by lt on 18-6-6.
//
#include <iostream>
#include <boost/format.hpp>

int main(int argc,char **argv){
    // 占位符的使用
    int a[3]={11,22,33};
    std::cout <<boost::format("%1%\n %2%\n %3%\n")%"first"%12%a[1];

    boost::format fmt("%1%\n %2%\n %3%\n");
    fmt% "first";
    fmt%12%a[1];
    std::cout<<fmt;
    std::string s= fmt.str();
    std::cout<<s;
    std::cout<<boost::format("%s  %d  \n")%"toto:" %12.5;                                                                                                                                                      ")

}
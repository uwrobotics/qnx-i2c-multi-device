#include<inc/i2c_host.h>
#include<iostream>

#include <inc/json_helper.h>
#include <sys/neutrino.h>

int main() {
    std::unordered_map<std::string, int> item1 = {{"ID", 1}};
    std::unordered_map<std::string, std::string> item2 = {{"Name", "Yuchen"}};
    std::unordered_map<std::string, std::string> item3 = {{"Name", "Yuchen"}, {"Gender", "Male"}};
    std::vector<int> arr = {1,2,3};
    std::unordered_map<std::string, std::vector<int>>item4 ={{"Value", arr}};

    JsonEncode obj = JsonEncode();

    obj.add(item1);
    obj.add(item3);
    obj.indent("More Info");
    obj.add(item2);
    obj.add(item4);

    std::cout << obj.get_string() << std::endl;

    return 0;
}
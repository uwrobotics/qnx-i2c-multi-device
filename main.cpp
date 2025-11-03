#include<inc/i2c_host.h>
#include<iostream>

#include <inc/json_helper.h>
#include <sys/neutrino.h>

int main() {
    std::unordered_map<std::string, int> item1 = {{"ID", 1}};
    std::unordered_map<std::string, std::string> item2 = {{"Name", "Saheed"}};
    std::unordered_map<std::string, std::string> item3 = {{"Name", "Yuchen"}, {"Gender", "Male"}};
    std::vector<int> arr = {1,2,3};
    std::unordered_map<std::string, std::vector<int>>item4 ={{"Values", arr}};

    JsonEncode in_obj = JsonEncode();

    in_obj.add(item1);
    in_obj.add(item3);
    in_obj.indent("More Info");
    in_obj.add(item2);
    in_obj.add(item4);

    
    std::string json_string = in_obj.get_string();

    std::cout << "Json: " << json_string << std::endl;

    JsonDecode out_obj = JsonDecode(json_string.c_str());
    int id;
    out_obj.get_int("ID", id, false);
    std::cout << "ID: " << id << std::endl;

    std::string name, gender;
    out_obj.get_string("Name", name, false);
    std::cout << "Name: " << name << std::endl;

    out_obj.get_string("Gender", gender, false);
    std::cout << "Gender: " << gender << std::endl;

    out_obj.indent("More Info", false);
    std::cout << "Enter indent" << std::endl;

    out_obj.get_string("Name", name, false);
    std::cout << "Name: " << name << std::endl;

    std::vector<int> arr_out;
    out_obj.get_vec_int("Values", arr_out, 3, false);
    std::cout << "Values: ";

    for(auto value: arr_out){
        std::cout << value << " ";
    }

    std::cout << std::endl;
    return 0;
}
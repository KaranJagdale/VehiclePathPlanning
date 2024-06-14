#include "../inc/vehicle.h";
#include <iostream>;

int main(){
    float env_pos[2] = {0, 0};
    float env_size[2] = {20, 20};

    float obj1_pos[2] = {5, 5};
    float obj1_size[2] = {3,3};

    float obj2_pos[2] = {10, 15};
    float obj2_size[2] = {5,4};

    float obj3_pos[2] = {12, 10};
    float obj3_size[2] = {2,6};

    //creating the environment with objects as the obstacles

    Boundary env(env_pos, env_size);
    Boundary obj1(obj1_pos, obj1_size);
    Boundary obj2(obj2_pos, obj2_size);
    Boundary obj3(obj3_pos, obj3_size);


}
#pragma once

#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>
#include <math.h>

template <typename T>
void readBin(const std::string& filename, std::vector<T>& data) {
    std::fstream file(filename, std::ios::in | std::ios::binary);
    data.clear();
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        exit(1);
    } else {
        while (true) {
            T t;
            file.read(reinterpret_cast<char*>(&t), sizeof(T));
            if (file.eof()) {
                break;
            }
            data.push_back(t);
        }
    }
    file.close();
}


int main(){
    std::vector<int> result;
    result.reserve(std::pow(2,22));
    std::string resultDir = "/home/jelee/my_ws/TrajOpt/test/testdata/rt-result-sizhe/pose_21/"; // CHANGE YOUR DIRECTORY HERE
    int i = 1;
    readBin(resultDir +  "result_" + std::to_string(i) + ".bin", result);
    int collisionCount = std::count(result.begin(), result.end(), 1);
    std::cout << "Collision count: " << collisionCount << "/" << result.size() << std::endl;
    return 0;
}
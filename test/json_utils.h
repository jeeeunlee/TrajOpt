#pragma once

#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include "rossy_utils/io/json.hpp"

Eigen::VectorXd json_list_to_eigen(const nlohmann::json &j){
    Eigen::VectorXd vector(j.size());
    int element_index=0;    
    for (const auto& element : j) {
        vector(element_index++) = (double)element;
    }
    return vector;
}

Eigen::MatrixXd json_listoflist_to_eigenmat(const nlohmann::json &j){
    Eigen::MatrixXd mat(j[0].size(),j.size());
    size_t element_index=0;
    for (const auto& element : j) {
        mat.col(element_index++) = json_list_to_eigen(element);
    }
    return mat;
}

void json_listoflist_to_vecofeigen(const nlohmann::json &j, 
                    std::vector<Eigen::VectorXd> &vector,
                    bool rowwise = true){
    Eigen::MatrixXd mat =json_listoflist_to_eigenmat(j);
    vector.clear();
    if(rowwise){
        for(int i(0); i<mat.rows(); ++i)
            vector.push_back(mat.row(i));
    }else{
        for(int i(0); i<mat.cols(); ++i)
            vector.push_back(mat.col(i));
    }
    
}
#pragma once

#include <iostream>

class StoreQtable {
 private:
    double* qMat;
 public:
    double* retriveQTable;
    void storeQTable();
};

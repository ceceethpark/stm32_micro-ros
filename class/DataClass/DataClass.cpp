/*
 * DataClass.cpp
 *
 *  Created on: Nov 25, 2025
 *      Author: thpark
 */

#include "DataClass.h"
#include <cstdlib>

// Default Constructor
DataClass::DataClass() : data(0), temperature(0.0f), isActive(false) {
    // Initialization done in initializer list
}

// Parameterized Constructor
DataClass::DataClass(int initialData) 
    : data(initialData), temperature(0.0f), isActive(true) {
    // Initialization done in initializer list
}

// Destructor
DataClass::~DataClass() {
    // Cleanup if needed
}

// Getter methods
int DataClass::getData() const {
    return data;
}

float DataClass::getTemperature() const {
    return temperature;
}

bool DataClass::getIsActive() const {
    return isActive;
}

// Setter methods
void DataClass::setData(int newData) {
    data = newData;
}

void DataClass::setTemperature(float temp) {
    temperature = temp;
}

void DataClass::setIsActive(bool active) {
    isActive = active;
}

// Utility methods
void DataClass::reset() {
    data = 0;
    temperature = 0.0f;
    isActive = false;
}

void DataClass::incrementData() {
    data++;
}

void DataClass::updateTemperature(float newTemp) {
    temperature = newTemp;
    isActive = true;
}

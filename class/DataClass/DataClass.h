/*
 * DataClass.h
 *
 *  Created on: Nov 25, 2025
 *      Author: thpark
 */

#ifndef CLASS_DATACLASS_DATACLASS_H_
#define CLASS_DATACLASS_DATACLASS_H_

class DataClass {
private:
    int data;
    float temperature;
    bool isActive;

public:
    // Constructor
    DataClass();
    DataClass(int initialData);
    
    // Destructor
    ~DataClass();
    
    // Getter methods
    int getData() const;
    float getTemperature() const;
    bool getIsActive() const;
    
    // Setter methods
    void setData(int newData);
    void setTemperature(float temp);
    void setIsActive(bool active);
    
    // Utility methods
    void reset();
    void incrementData();
    void updateTemperature(float newTemp);
};

#endif /* CLASS_DATACLASS_DATACLASS_H_ */

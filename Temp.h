/*
* Copyright (C) 2016 Nicolas Bertuol, University of Pau, France
*
* nicolas.bertuol@etud.univ-pau.fr
*/

#ifndef Temp_H
#define Temp_H
#include "Sensor.h"

#define LW_SCALE _BOARD_VOLT_SCALE

class Temp : public Sensor {
  public:
    Temp(char* nomenclature, bool is_analog, bool is_connected, bool is_low_power, uint8_t pin_read, uint8_t pin_power);
    void update_data();
    double get_value();
};

#endif

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Hacked-in external temp sensor
 */

#pragma once
#include "AP_TemperatureSensor_Backend.h"

#ifndef AP_TEMPERATURE_SENSOR_EXTERNAL_ENABLED
#define AP_TEMPERATURE_SENSOR_EXTERNAL_ENABLED AP_TEMPERATURE_SENSOR_ENABLED
#endif

#if AP_TEMPERATURE_SENSOR_EXTERNAL_ENABLED

class AP_TemperatureSensor_External : public AP_TemperatureSensor_Backend {
    using AP_TemperatureSensor_Backend::AP_TemperatureSensor_Backend;

public:
    void init(void) override;

    void update() override {};

    void setTemperature(float temp) {};

private:
    // // reset device
    // bool reset(void) const;

    // // read (relevant) internal calibration registers into _k
    // bool read_prom(void);
    // uint16_t read_prom_word(const uint8_t word) const;

    // // begin an ADC conversion (min:7.40ms typ:8.22ms max:9.04ms)
    // void start_next_sample();
    // uint32_t read_adc(void) const;

    // // update the temperature, called at 20Hz
    // void _timer(void);

    // // calculate temperature using adc reading and internal calibration
    // float calculate(const uint32_t adc) const;
};
#endif // AP_TEMPERATURE_SENSOR_EXTERNAL_ENABLED

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
#pragma once

#include "AP_WindVane_Backend.h"

class AP_WindVane_NMEA : public AP_WindVane_Backend
{
public:
    // constructor
    using AP_WindVane_Backend::AP_WindVane_Backend;

    // initialization
    void init(const AP_SerialManager& serial_manager) override;

    // update state
    void update_direction() override;
    void update_speed() override;

private:
    // pointer to serial uart
    AP_HAL::UARTDriver *uart = nullptr; 

    // See if we can read in some data
    void update();

    // try and decode NMEA message
    bool decode(char c);

    // decode each term
    bool decode_latest_term();

    // Decode specific sentences
    void parse_mwv();
    void parse_xdr();

    // latest values read in
    float _speed_ms;
    float _wind_dir_deg;

    // Latest voltage inputs
    float _voltages[3];

    char _term[15];            // buffer for the current term within the current sentence
    uint8_t _term_offset;      // offset within the _term buffer where the next character should be placed
    uint8_t _term_number;      // term index within the current sentence
    uint8_t _checksum;         // checksum accumulator
    bool _term_is_checksum;    // current term is the checksum
    bool _sentence_valid;      // is current sentence valid so far
    bool _sentence_done;       // true if this sentence has already been decoded

    enum class SentenceType {

        /**
        *        1   2 3   4 5
        *        |   | |   | |
        * $--MWV,x.x,a,x.x,a*hh<CR><LF>
        * 
        * Fields:
        * 
        * 1: Wind Angle, 0 to 359 degrees
        * 2: Reference, R = Relative, T = True
        * 3: Wind Speed
        * 4: Wind Speed Units, K/M/
        * 5: Status, A = Data Valid, V = Invalid
        */
        MWV,

        /**
        *        1 2   3 4            n
        *        | |   | |            |
        * $--XDR,a,x.x,a,c--c, ..... *hh<CR><LF>
        * 
        * 
        * 1: Transducer Type 
        *   "P" pressure
        *   "C" temperature
        *   "A" angle
        * 2: Measurement Data
        * 3: Units of measurement 
        *   A = Amperes B = Bars B = Binary C = Celsius 
        *   D = Degrees H = Hertz I = liters/second 
        *   K = Kelvin K = kg/m3 M = Meters M = cubic Meters 
        *   N = Newton P = % of full range P = Pascal R = RPM 
        *   S = Parts per thousand V = Volts
        * 4: Name of transducer
        */
        XDR,
    };
    SentenceType _sentence_type;
};

#include "gt7udpparser.h"
#include "gt7wrapper.h"
#include "esp_log.h"

// Instanz des Parsers in der C++ Welt erstellen
static GT7_UDP_Parser parser;

static const char* TAG = "GT7_DEBUG";

extern "C" void gt7_process_raw_udp(uint8_t* data, size_t size) {
    // Hier rufen wir die C++ Funktion auf!
    //ESP_LOGI(TAG, "Data length: '%d'", size);
    parser.parseData(data, size);
    
}

extern "C" uint8_t gt7_get_current_Gear(void){
    return parser.getCurrentGearFromByte();
}

extern "C" uint16_t gt7_get_current_RPM(void){
    return parser.getRPM();
}

extern "C" uint16_t gt7_get_Speed(void){
    return parser.getVehicleSpeed();
}

extern "C" uint8_t gt7_get_Throttle(void){
    return parser.getcurrentThrottle();
}

extern "C" uint8_t gt7_get_Brake(void){
    return parser.getcurrentBrake();
}

extern "C" int32_t gt7_get_last_Laptime(void){
    return parser.getlastLaptime();
}

extern "C" int32_t gt7_get_best_Laptime(void){
    return parser.getbestLaptime();
}

extern "C" int32_t gt7_get_current_Laptime(void){
    return parser.getcurrentLaptime();
}

extern "C" int16_t gt7_getminAlertRPM(void){
    return parser.getminAlertRPM();
}

extern "C" int16_t gt7_getmaxAlertRPM(void){
    return parser.getmaxAlertRPM();
}

extern "C" float gt7_getcurrenttyreTemp(int index){
    return parser.getcurrenttyreTemp(index);
}

extern "C" uint8_t gt7_getcurrentBrakeFiltered(void){
    return parser.getcurrentBrakeFiltered();
}

extern "C" uint8_t gt7_getcurrentThrottleFiltered(void){
    return parser.getcurrentThrottleFiltered();
}

extern "C" float gt7_getcurrentFuelLevel(void){
    return parser.getcurrentFuelLevel();
}

extern "C" float gt7_getcurrentOilTemp(void){
    return parser.getcurrentOilTemp();
}

extern "C" float gt7_getcurrentOilPressure(void){
    return parser.getcurrentOilPressure();
}

extern "C" float gt7_getcurrentBoost(void){
    return parser.getcurrentBoost();
}

extern "C" float gt7_getcurrentCoolantTemp(void){
    return parser.getcurrentCoolantTemp();
}

extern "C" int16_t gt7_getcurrentLapCount(void){
    return parser.getcurrentLapCount();
}
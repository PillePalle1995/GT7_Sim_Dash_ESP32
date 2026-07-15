#ifndef GT7WRAPPER_H
#define GT7WRAPPER_H

#include <inttypes.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Diese C-Schnittstelle rufst du in deinem UDP-Task auf CPU 0 auf!
// Sie nimmt die rohen Daten an und reicht sie im Hintergrund an C++ weiter.
void gt7_process_raw_udp(uint8_t* data, size_t size);
uint16_t gt7_get_current_RPM(void);
uint8_t gt7_get_current_Gear(void);
uint16_t gt7_get_Speed(void);
uint8_t gt7_get_Throttle(void);
uint8_t gt7_get_Brake(void);
int32_t gt7_get_last_Laptime(void);
int32_t gt7_get_best_Laptime(void);
int32_t gt7_get_current_Laptime(void);
int16_t gt7_getminAlertRPM(void);
int16_t gt7_getmaxAlertRPM(void);
float gt7_getcurrenttyreTemp(int index);
uint8_t gt7_getcurrentBrakeFiltered(void);
uint8_t gt7_getcurrentThrottleFiltered(void);
float gt7_getcurrentFuelLevel(void);
float gt7_getcurrentOilTemp(void);
float gt7_getcurrentOilPressure(void);
float gt7_getcurrentBoost(void);
float gt7_getcurrentCoolantTemp(void);
int16_t gt7_getcurrentLapCount(void);

#ifdef __cplusplus
}
#endif

#endif
#ifndef _BASICS_H
#define _BASICS_H

void basicHwInit();
void basicSetup();

void presentBattery();
void reportBatteryVoltage();

void presentCommsStatus();
void processCommsStatus( const indication_t ind );
void reportCommsStatus();

#endif // _BASICS_H

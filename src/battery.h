#pragma once

// Battery monitoring through the Atomic Battery Base (voltage divider 1:2 on G8).
// Without the base (TailBat, USB) the pin floats; the battery is then reported as absent.
void batteryBegin();
// Call from loop(); samples the voltage periodically
void batteryUpdate();
bool batteryPresent();
float batteryVoltage();
int batteryPercent();

#pragma once
#include <ESPAsyncWebServer.h>

// Loads saved networks, registers the WiFi setup page and API on the server and
// starts the background WiFi task. The server itself is started by the caller.
void networkBegin(AsyncWebServer &server);

// True while the setup access point Compass_Setup is running
bool networkApActive();

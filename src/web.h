#pragma once
#include <ESPAsyncWebServer.h>

// Sends a page from data/ embedded into the firmware at build time
// (see scripts/embed_web.py), e.g. sendEmbeddedPage(request, "index.html")
void sendEmbeddedPage(AsyncWebServerRequest *request, const char *name);

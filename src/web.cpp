#include "web.h"
#include "web_pages.h"

void sendEmbeddedPage(AsyncWebServerRequest *request, const char *name) {
  for (const EmbeddedPage &page : EMBEDDED_PAGES) {
    if (strcmp(page.name, name) == 0) {
      AsyncWebServerResponse *response = request->beginResponse(200, "text/html", page.data, page.len);
      response->addHeader("Content-Encoding", "gzip");
      request->send(response);
      return;
    }
  }
  request->send(404, "text/plain", "Not found");
}

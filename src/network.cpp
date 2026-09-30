#include "network.h"
#include "web.h"
#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <memory>

static const size_t MAX_WIFI_NETWORKS = 5;
static const char *AP_SSID = "Compass_Setup";
static const uint32_t AP_TIMEOUT_MS = 5 * 60 * 1000;
// Keeps the AP up for a while after connecting, so the setup page can show the new IP
static const uint32_t AP_STOP_DELAY_MS = 10000;
static const uint32_t WIFI_RECONNECT_INTERVAL_MS = 30000;
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 10000;
static const uint16_t DNS_PORT = 53;

struct Credentials {
  char ssid[33];
  char password[65];
};

// Accessed only from the WiFi task after networkBegin()
static std::unique_ptr<WiFiMulti> wifiMulti;
static DNSServer dnsServer;
static int wifiNetworkCount = 0;
static bool mdnsStarted = false;

// Requests from HTTP handlers (async_tcp task) to the WiFi task
static QueueHandle_t connectQueue;
static volatile bool rebuildRequested = false;
static volatile bool scanRequested = false;
static volatile bool apActive = false;

// Saved networks are a JSON array ordered from oldest to newest
static String loadCredentialsJson() {
  Preferences prefs;
  prefs.begin("wifi-config", true);
  String creds = prefs.getString("creds", "[]");
  prefs.end();
  return creds;
}

static void storeCredentials(JsonDocument &doc) {
  String creds;
  serializeJson(doc, creds);
  Preferences prefs;
  prefs.begin("wifi-config", false);
  prefs.putString("creds", creds);
  prefs.end();
}

static void saveWiFiCredentials(const char *ssid, const char *password) {
  JsonDocument doc;
  deserializeJson(doc, loadCredentialsJson());
  JsonArray array = doc.as<JsonArray>();
  if (array.isNull()) {
    array = doc.to<JsonArray>();
  }

  // Move an existing SSID to the end
  for (size_t i = 0; i < array.size(); i++) {
    if (String(ssid) == array[i]["ssid"].as<String>()) {
      array.remove(i);
      break;
    }
  }
  JsonObject cred = array.add<JsonObject>();
  cred["ssid"] = ssid;
  cred["password"] = password;

  // Drop the oldest networks over the limit
  while (array.size() > MAX_WIFI_NETWORKS) {
    array.remove(0);
  }
  storeCredentials(doc);
  M5.Log.printf("Saved WiFi credentials for %s\n", ssid);
}

static bool deleteWiFiCredentials(const char *ssid) {
  JsonDocument doc;
  deserializeJson(doc, loadCredentialsJson());
  JsonArray array = doc.as<JsonArray>();
  for (size_t i = 0; i < array.size(); i++) {
    if (String(ssid) == array[i]["ssid"].as<String>()) {
      array.remove(i);
      storeCredentials(doc);
      M5.Log.printf("Deleted WiFi credentials for %s\n", ssid);
      return true;
    }
  }
  return false;
}

// WiFiMulti cannot remove networks, so it is rebuilt from NVS on every change
static void rebuildWiFiMulti() {
  wifiMulti.reset(new WiFiMulti());
  wifiNetworkCount = 0;

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, loadCredentialsJson());
  if (error) {
    M5.Log.printf("Failed to parse wifi creds: %s\n", error.c_str());
    return;
  }
  JsonArray array = doc.as<JsonArray>();
  // Load only the newest MAX_WIFI_NETWORKS entries
  size_t skip = array.size() > MAX_WIFI_NETWORKS ? array.size() - MAX_WIFI_NETWORKS : 0;
  for (JsonObject obj : array) {
    if (skip > 0) {
      skip--;
      continue;
    }
    const char *ssid = obj["ssid"];
    const char *password = obj["password"];
    if (ssid && password) {
      wifiMulti->addAP(ssid, password);
      wifiNetworkCount++;
    }
  }
  M5.Log.printf("Loaded %d WiFi networks.\n", wifiNetworkCount);
}

static void startMDNS() {
  if (mdnsStarted) return;
  if (!MDNS.begin("compass")) {
    M5.Log.println("Error setting up MDNS responder!");
  } else {
    M5.Log.println("mDNS responder started");
    MDNS.addService("http", "tcp", 80);
    mdnsStarted = true;
  }
}

static void startAp() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID);
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  apActive = true;
  M5.Log.printf("Setup AP %s started at %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
}

static void stopAp() {
  dnsServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  apActive = false;
  M5.Log.println("Setup AP stopped");
}

static bool connectTo(const Credentials &cred) {
  M5.Log.printf("Connecting to %s...\n", cred.ssid);
  WiFi.begin(cred.ssid, cred.password);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    if (apActive) dnsServer.processNextRequest();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  return WiFi.status() == WL_CONNECTED;
}

// All WiFi work runs here so the compass starts immediately: tries saved networks,
// once falls back to the setup AP, reconnects on drops
static void wifiTask(void *param) {
  bool apTried = false;
  bool attemptNow = true;
  uint32_t lastAttempt = 0;
  uint32_t apStartedAt = 0;
  uint32_t apStopAt = 0;

  for (;;) {
    if (rebuildRequested) {
      rebuildRequested = false;
      rebuildWiFiMulti();
    }
    if (scanRequested) {
      scanRequested = false;
      WiFi.scanNetworks(true);
    }

    Credentials cred;
    // A new network is tried right away only when offline; otherwise the page
    // that added it would lose its connection
    if (xQueueReceive(connectQueue, &cred, 0) == pdTRUE && WiFi.status() != WL_CONNECTED) {
      if (connectTo(cred)) {
        M5.Log.printf("WiFi connected to %s\n", cred.ssid);
      } else {
        M5.Log.printf("Connecting to %s failed\n", cred.ssid);
      }
    }

    if (WiFi.status() == WL_CONNECTED) {
      startMDNS();
    }

    if (apActive) {
      dnsServer.processNextRequest();
      if (WiFi.status() == WL_CONNECTED && apStopAt == 0) {
        apStopAt = millis() + AP_STOP_DELAY_MS;
      }
      if (apStopAt != 0 && (int32_t)(millis() - apStopAt) >= 0) {
        stopAp();
      } else if (WiFi.status() != WL_CONNECTED && millis() - apStartedAt >= AP_TIMEOUT_MS) {
        M5.Log.println("Setup AP timed out, running offline.");
        stopAp();
        lastAttempt = millis();
      }
      // No background attempts while the AP runs: a scan would retune the AP channel
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (WiFi.status() != WL_CONNECTED && (attemptNow || millis() - lastAttempt >= WIFI_RECONNECT_INTERVAL_MS)) {
      attemptNow = false;
      lastAttempt = millis();
      if (wifiNetworkCount > 0) {
        M5.Log.println("Trying saved WiFi networks...");
        wifiMulti->run(8000);
      }
      if (WiFi.status() == WL_CONNECTED) {
        M5.Log.printf("WiFi connected to %s\n", WiFi.SSID().c_str());
      } else if (!apTried) {
        apTried = true;
        apStartedAt = millis();
        apStopAt = 0;
        startAp();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

static const char *encryptionName(wifi_auth_mode_t mode) {
  return mode == WIFI_AUTH_OPEN ? "open" : "secured";
}

static void registerRoutes(AsyncWebServer &server) {
  server.on("/wifi", HTTP_GET, [](AsyncWebServerRequest *request) {
    sendEmbeddedPage(request, "wifi.html");
  });

  // Asynchronous scan: returns {"scanning":true} until results are available
  server.on("/api/scan", HTTP_GET, [](AsyncWebServerRequest *request) {
    int n = WiFi.scanComplete();
    if (n < 0) {
      if (n == WIFI_SCAN_FAILED) scanRequested = true;
      request->send(200, "application/json", "{\"scanning\":true}");
      return;
    }
    JsonDocument doc;
    JsonArray list = doc["networks"].to<JsonArray>();
    for (int i = 0; i < n; i++) {
      if (WiFi.SSID(i).isEmpty()) continue;
      JsonObject net = list.add<JsonObject>();
      net["ssid"] = WiFi.SSID(i);
      net["rssi"] = WiFi.RSSI(i);
      net["enc"] = encryptionName(WiFi.encryptionType(i));
    }
    WiFi.scanDelete();
    String body;
    serializeJson(doc, body);
    request->send(200, "application/json", body);
  });

  // Saved networks (without passwords) and connection state
  server.on("/api/networks", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument saved;
    deserializeJson(saved, loadCredentialsJson());
    JsonDocument doc;
    JsonArray list = doc["saved"].to<JsonArray>();
    for (JsonObject obj : saved.as<JsonArray>()) {
      list.add(obj["ssid"].as<String>());
    }
    bool connected = WiFi.status() == WL_CONNECTED;
    doc["connected"] = connected ? WiFi.SSID() : "";
    doc["ip"] = connected ? WiFi.localIP().toString() : "";
    doc["ap"] = (bool)apActive;
    doc["max"] = MAX_WIFI_NETWORKS;
    String body;
    serializeJson(doc, body);
    request->send(200, "application/json", body);
  });

  server.on("/api/networks", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("ssid", true)) {
      request->send(400, "text/plain", "missing ssid");
      return;
    }
    String ssid = request->getParam("ssid", true)->value();
    String password = request->hasParam("password", true) ? request->getParam("password", true)->value() : "";
    if (ssid.isEmpty() || ssid.length() > 32 || password.length() > 64) {
      request->send(400, "text/plain", "invalid ssid or password");
      return;
    }
    saveWiFiCredentials(ssid.c_str(), password.c_str());
    rebuildRequested = true;

    Credentials cred;
    strlcpy(cred.ssid, ssid.c_str(), sizeof(cred.ssid));
    strlcpy(cred.password, password.c_str(), sizeof(cred.password));
    xQueueSend(connectQueue, &cred, 0);
    request->send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/api/networks", HTTP_DELETE, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("ssid")) {
      request->send(400, "text/plain", "missing ssid");
      return;
    }
    bool deleted = deleteWiFiCredentials(request->getParam("ssid")->value().c_str());
    rebuildRequested = true;
    request->send(deleted ? 200 : 404, "application/json", deleted ? "{\"ok\":true}" : "{\"ok\":false}");
  });

  // Captive portal: while the AP runs, any unknown URL (incl. OS connectivity checks
  // like /generate_204 or /hotspot-detect.html) leads to the setup page
  server.onNotFound([](AsyncWebServerRequest *request) {
    if (apActive) {
      request->redirect("http://" + WiFi.softAPIP().toString() + "/wifi");
    } else {
      request->send(404, "text/plain", "Not found");
    }
  });
}

void networkBegin(AsyncWebServer &server) {
  connectQueue = xQueueCreate(2, sizeof(Credentials));
  WiFi.mode(WIFI_STA);
  rebuildWiFiMulti();
  registerRoutes(server);
  xTaskCreatePinnedToCore(wifiTask, "wifi", 8192, NULL, 1, NULL, 0);
}

bool networkApActive() {
  return apActive;
}

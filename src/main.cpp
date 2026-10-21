#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include "config.h"
#include "balance_bot.h"
#include "web_ui.h"

WebServer  server(80);
BalanceBot g_bot;

// ── WiFi ────────────────────────────────────────────────────────
static void startWifi() {
#if WIFI_USE_STATION_MODE
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
    Serial.print("WiFi connecting");
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_STA_TIMEOUT_MS) {
        delay(300); Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("Open http://"); Serial.println(WiFi.localIP());
        WiFi.setSleep(false);
        return;
    }
    Serial.println("STA join failed, falling back to AP");
#endif
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
    WiFi.setSleep(false);
    Serial.printf("AP '%s' (pass '%s')\n", WIFI_AP_SSID, WIFI_AP_PASS);
    Serial.print("Open http://"); Serial.println(WiFi.softAPIP());
}

// ── HTTP handlers ───────────────────────────────────────────────
static void handleRoot()   { server.send_P(200, "text/html", INDEX_HTML); }
static void handleParams() { server.send(200, "application/json", g_bot.paramsJson()); }

static void handleSet() {
    const String name = server.arg("name");
    const String valStr = server.arg("value");
    char* end = nullptr;
    const float v = strtof(valStr.c_str(), &end);
    if (end == valStr.c_str() || *end != '\0' || !isfinite(v)) {
        server.send(400, "text/plain", "bad value"); return;
    }
    if (!g_bot.setParam(name, v)) {
        server.send(404, "text/plain", "unknown parameter"); return;
    }
    server.send(200, "text/plain", "ok");
}

static void handleTelemetry() {
    Telemetry t = g_bot.snapshot();
    char buf[400];
    int n = snprintf(buf, sizeof(buf),
        "{\"s\":%d,\"a\":%d,\"p\":%.3f,\"t\":%.3f,\"r\":%.2f,\"u\":%.2f,"
        "\"pl\":%d,\"pr\":%d,\"hz\":%u,\"el\":%ld,\"er\":%ld,\"sp\":%.1f}",
        int(t.state), t.armed ? 1 : 0, t.pitch, t.tilt, t.rate, t.u,
        t.pwmL, t.pwmR, unsigned(t.hz), t.encL, t.encR, t.spd);
    if (n < 0 || n >= int(sizeof(buf))) { server.send(500, "text/plain", "overflow"); return; }
    server.send(200, "application/json", buf);
}

static void handleArm() {
    const bool on    = server.arg("on") == "1";
    const bool force = server.arg("force") == "1";
    g_bot.arm(on);
    if (on && force) g_bot.forceStart();
    server.send(200, "text/plain", "ok");
}

static void handleZero() {
    g_bot.zeroAngleFromCurrent();
    server.send(200, "text/plain", "ok");
}

static void handleRecal() {
    if (g_bot.isArmed()) { server.send(409, "text/plain", "disarm first"); return; }
    g_bot.requestRecalibration();
    server.send(200, "text/plain", "ok");
}

static void handleSave() {
    g_bot.requestSave();
    server.send(200, "text/plain",
                g_bot.state() == BotState::Running ? "pending" : "ok");
}

static void handleDefaults() {
    g_bot.loadDefaults();
    server.send(200, "text/plain", "ok");
}

static void registerWebRoutes() {
    server.on("/",          HTTP_GET, handleRoot);
    server.on("/params",    HTTP_GET, handleParams);
    server.on("/set",       HTTP_GET, handleSet);
    server.on("/telemetry", HTTP_GET, handleTelemetry);
    server.on("/arm",       HTTP_GET, handleArm);
    server.on("/zero",      HTTP_GET, handleZero);
    server.on("/recal",     HTTP_GET, handleRecal);
    server.on("/save",      HTTP_GET, handleSave);
    server.on("/defaults",  HTTP_GET, handleDefaults);
}

// ── Entry points ───────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("\nBalance Bot v3");

    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(IMU_I2C_CLOCK_HZ);
    Wire.setTimeOut(IMU_I2C_TIMEOUT_MS);

    if (!g_bot.begin()) {
        Serial.println("WARN: IMU not found — will retry in control loop");
    }

    startWifi();
    registerWebRoutes();
    server.begin();
    g_bot.startTasks();

    Serial.println("Ready.");
}

void loop() {
    server.handleClient();
    g_bot.serviceSave();
    delay(2);
}
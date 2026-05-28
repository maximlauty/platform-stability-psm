#pragma once
// app_http.h -- HTTP/REST server, /api endpoints, HTML dashboard.
// QM / non-safety layer.
#include "../safety/safety_defs.h"
#include "app_params.h"
#include <QNEthernet.h>
using namespace qindesign::network;

static constexpr uint32_t WEB_CLIENT_TIMEOUT_MS = 500U;

// Called once from setup()
void app_init();

// Called every loop() iteration
void app_tick();

// Internal helpers (also used from .ino during transition)
bool parse_kv(const char *body, int &pos, char *key, char *val, int kvsz);
void http_handle(EthernetClient &client);

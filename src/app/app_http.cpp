// app_http.cpp -- Ethernet init, web server, /api endpoints, HTML dashboard (QM layer).

#include "app_http.h"
#include "app_light.h"
#include "app_params.h"
#include "../safety/safety_monitor.h"
#include "../safety/safety_stability.h"
#include "../safety/safety_ahrs.h"
#include "../safety/safety_isr.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

// CR-3: change before deployment. Every POST /config must supply auth=<token>.
#define CONFIG_AUTH_TOKEN "psm-change-me-v1"

// --- Network config ----------------------------------------------------------
static const IPAddress WEB_IP    (192, 168, 168, 71);
static const IPAddress WEB_SUBNET(255, 255, 255,  0);
static const IPAddress WEB_GW    (192, 168, 168,  1);
static const IPAddress WEB_DNS   (192, 168, 168,  1);

static EthernetServer  s_web_server(80);
static EthernetClient  s_web_client;
static uint32_t        s_web_client_accept_ms = 0U;

// --- CT token compare (CR-3) -------------------------------------------------
static bool ct_token_eq(const char *provided, size_t provided_len)
{
    const size_t elen = sizeof(CONFIG_AUTH_TOKEN) - 1U;
    uint8_t diff = (uint8_t)(provided_len != elen ? 1U : 0U);
    size_t n = provided_len < elen ? provided_len : elen;
    for (size_t i = 0U; i < n; ++i)
        diff |= (uint8_t)((uint8_t)provided[i] ^ (uint8_t)CONFIG_AUTH_TOKEN[i]);
    return diff == 0U;
}

// --- URL-encoded key-value parser --------------------------------------------
bool parse_kv(const char *body, int &pos, char *key, char *val, int kvsz)
{
    if (!body[pos]) return false;
    int ki = 0, vi = 0; bool in_val = false;
    while (body[pos] && body[pos] != '&') {
        char c = body[pos++];
        if (c == '+') c = ' ';
        else if (c == '%') {
            char h1 = body[pos] ? body[pos++] : '0';
            char h2 = body[pos] ? body[pos++] : '0';
            auto hx = [](char ch) -> int {
                if (ch >= '0' && ch <= '9') return ch - '0';
                if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
                if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
                return 0;
            };
            c = (char)((hx(h1) << 4) | hx(h2));
        }
        if (c == '=') { in_val = true; continue; }
        if (!in_val) { if (ki < kvsz-1) key[ki++] = c; }
        else         { if (vi < kvsz-1) val[vi++] = c; }
    }
    if (body[pos] == '&') pos++;
    key[ki] = '\0'; val[vi] = '\0';
    return ki > 0;
}

// --- HTTP handler ------------------------------------------------------------
void http_handle(EthernetClient &client)
{
    wdt_feed();

    static char req[1024];
    int rlen = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < 200U) {
        while (client.available() && rlen < (int)sizeof(req) - 1)
            req[rlen++] = (char)client.read();
        req[rlen] = '\0';
        if (strstr(req, "\r\n\r\n")) break;
        delay(1);
    }
    wdt_feed();

    bool is_post     = (strncmp(req, "POST", 4) == 0);
    bool is_get      = (strncmp(req, "GET",  3) == 0);
    bool path_api    = (strstr(req, "GET /api/status") != nullptr);
    bool path_params = (strstr(req, "GET /api/params") != nullptr);
    bool path_js     = (strstr(req, "GET /api/js")     != nullptr);

    static char body[512];
    body[0] = '\0';
    if (is_post) {
        const char *cl_hdr = strstr(req, "Content-Length:");
        if (!cl_hdr) cl_hdr = strstr(req, "content-length:");
        int blen = cl_hdr ? atoi(cl_hdr + 15) : 0;
        if (blen > (int)sizeof(body) - 1) blen = (int)sizeof(body) - 1;
        int got = 0;
        const char *hdr_end = strstr(req, "\r\n\r\n");
        if (hdr_end) {
            int pre = (int)((req + rlen) - (hdr_end + 4));
            if (pre > blen) pre = blen;
            if (pre > 0) { memcpy(body, hdr_end + 4, pre); got = pre; }
        }
        t0 = millis();
        while (got < blen && millis() - t0 < 200U) {
            if (client.available()) body[got++] = (char)client.read();
            else delay(1);
        }
        body[got] = '\0';
        wdt_feed();
    }

    // ── POST /light/toggle ────────────────────────────────────────────────────
    if (is_post && strstr(req, "POST /light/toggle")) {
        char auth_val[40] = "";
        { int sc = 0; char ak[32], av[32];
          while (parse_kv(body, sc, ak, av, 32))
              if (strcmp(ak, "auth") == 0) { strncpy(auth_val, av, sizeof(auth_val)-1); break; } }
        if (!ct_token_eq(auth_val, strlen(auth_val))) {
            client.print("HTTP/1.1 403 Forbidden\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
            return;
        }
        light_set(!light_get());
        client.print("HTTP/1.1 200 OK\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
        return;
    }

    // ── POST /test/toggle-output ──────────────────────────────────────────────
    if (is_post && strstr(req, "POST /test/toggle-output")) {
        char auth_val[40] = "";
        { int sc = 0; char ak[32], av[32];
          while (parse_kv(body, sc, ak, av, 32))
              if (strcmp(ak, "auth") == 0) { strncpy(auth_val, av, sizeof(auth_val)-1); break; } }
        if (!ct_token_eq(auth_val, strlen(auth_val))) {
            client.print("HTTP/1.1 403 Forbidden\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
            return;
        }
        g_output_inhibit = !g_output_inhibit;
        if (Serial.availableForWrite() > 0)
            Serial.println(g_output_inhibit
                ? "[TEST] *** OUTPUT INHIBITED -- SAFE_OUT_PIN suppressed ***"
                : "[TEST] Output inhibit released -- normal operation resumed");
        client.print("HTTP/1.1 200 OK\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
        return;
    }

    // ── POST /config ──────────────────────────────────────────────────────────
    if (is_post && strstr(req, "POST /config")) {
        char auth_val[40] = "";
        { int sc = 0; char ak[32], av[32];
          while (parse_kv(body, sc, ak, av, 32))
              if (strcmp(ak, "auth") == 0) { strncpy(auth_val, av, sizeof(auth_val)-1); break; } }
        if (!ct_token_eq(auth_val, strlen(auth_val))) {
            client.print("HTTP/1.1 403 Forbidden\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
            return;
        }
        Params p = g_params;
        int pos = 0; char key[32], val[32];
        while (parse_kv(body, pos, key, val, 32)) {
            float    fv = atof(val);
            uint32_t uv = (uint32_t)atol(val);
            if      (!strcmp(key,"window_s"))             p.window_s             = uv;
            else if (!strcmp(key,"omega_stable_dps"))     p.omega_stable_dps     = fv;
            else if (!strcmp(key,"omega_instant_dps"))    p.omega_instant_dps    = fv;
            else if (!strcmp(key,"instant_hold_ms"))      p.instant_hold_ms      = uv;
            else if (!strcmp(key,"spike_accel_g"))        p.spike_accel_g        = fv;
            else if (!strcmp(key,"eval_period_ms"))       p.eval_period_ms       = uv;
            else if (!strcmp(key,"motion_samples_max"))   p.motion_samples_max   = uv;
            else if (!strcmp(key,"shock_samples_max"))    p.shock_samples_max    = uv;
            else if (!strcmp(key,"clean_streak_needed"))  p.clean_streak_needed  = uv;
            else if (!strcmp(key,"diverse_mean_tol_g"))   p.diverse_mean_tol_g   = fv;
            else if (!strcmp(key,"spread_max_deg"))       p.spread_max_deg       = fv;
            else if (!strcmp(key,"anchor_max_deg"))       p.anchor_max_deg       = fv;
            else if (!strcmp(key,"gyro_range_fault_dps")) p.gyro_range_fault_dps = fv;
            else if (!strcmp(key,"stale_fault_ticks"))    p.stale_fault_ticks    = uv;
            else if (!strcmp(key,"dual_diverge_deg"))     p.dual_diverge_deg     = fv;
        }
        p.magic   = PARAMS_MAGIC;
        p.version = PARAMS_VERSION;
        if (params_valid(p)) {
            g_params = p;
            params_save();          // saves + calls app_publish_thresholds() + refresh CRC
            s_sample_timer.end();
            fault_reset_all();
            reset_stability_state();
            s_anchor_valid = false;
            mahony_init(s_imu_a);
            mahony_init(s_imu_b);
            s_sample_timer.begin(timerISR, 1000000U / AHRS_HZ);
        }
        client.print("HTTP/1.1 200 OK\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
        return;
    }

    // ── GET /api/params ───────────────────────────────────────────────────────
    if (path_params) {
        char json[640];
        snprintf(json, sizeof(json),
            "{\"window_s\":%lu,\"omega_stable_dps\":%.4g,\"omega_instant_dps\":%.4g,"
            "\"instant_hold_ms\":%lu,\"spike_accel_g\":%.4g,\"eval_period_ms\":%lu,"
            "\"motion_samples_max\":%lu,\"shock_samples_max\":%lu,\"clean_streak_needed\":%lu,"
            "\"diverse_mean_tol_g\":%.4g,\"spread_max_deg\":%.4g,\"anchor_max_deg\":%.4g,"
            "\"gyro_range_fault_dps\":%.4g,\"stale_fault_ticks\":%lu,\"dual_diverge_deg\":%.4g}",
            (unsigned long)g_params.window_s,
            (double)g_params.omega_stable_dps,    (double)g_params.omega_instant_dps,
            (unsigned long)g_params.instant_hold_ms, (double)g_params.spike_accel_g,
            (unsigned long)g_params.eval_period_ms,
            (unsigned long)g_params.motion_samples_max,
            (unsigned long)g_params.shock_samples_max,
            (unsigned long)g_params.clean_streak_needed,
            (double)g_params.diverse_mean_tol_g,  (double)g_params.spread_max_deg,
            (double)g_params.anchor_max_deg,       (double)g_params.gyro_range_fault_dps,
            (unsigned long)g_params.stale_fault_ticks,
            (double)g_params.dual_diverge_deg);
        char hdr[128];
        snprintf(hdr, sizeof(hdr),
            "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
            "Content-Length: %d\r\nConnection: close\r\n\r\n", (int)strlen(json));
        client.print(hdr); client.print(json);
        return;
    }

    // ── GET /api/status ───────────────────────────────────────────────────────
    if (path_api) {
        // Read from g_safety_status (written by safety_tick every eval period)
        SafetyStatus ss;
        noInterrupts();
        memcpy(&ss, (const void*)&g_safety_status, sizeof(ss));
        interrupts();
        uint32_t ws = ss.window_samples > 0U ? ss.window_samples : 1U;
        char json[640];
        snprintf(json, sizeof(json),
            "{\"stable\":%s,\"inhibit\":%s,\"fault\":\"0x%04X\",\"why\":\"%s\","
            "\"fsm_s\":\"%s\",\"hw_cnt\":%u,"
            "\"fill\":%d,\"gyro_spk\":%lu,\"gyro_max\":%lu,"
            "\"accel_spk\":%lu,\"accel_max\":%lu,"
            "\"omega\":%.2f,\"roll\":%.2f,\"pitch\":%.2f,"
            "\"roll_b\":%.2f,\"pitch_b\":%.2f,"
            "\"diverge_r\":%.2f,\"diverge_p\":%.2f,"
            "\"stale_a\":%lu,\"stale_b\":%lu,\"spread\":%.2f,\"light\":%u}",
            ss.platform_stable ? "true"  : "false",
            ss.output_inhibit  ? "true"  : "false",
            (unsigned)ss.fault_mask, ss.why_str,
            ss.fsm_state_str, (unsigned)ss.fsm_stable_cnt,
            (int)(100U * (ss.buf_count < ws ? ss.buf_count : ws) / ws),
            (unsigned long)ss.spike_gyro_count,   (unsigned long)g_params.motion_samples_max,
            (unsigned long)ss.spike_accel_count,  (unsigned long)g_params.shock_samples_max,
            (double)ss.last_omega_dps,
            (double)ss.last_roll_deg,   (double)ss.last_pitch_deg,
            (double)ss.last_roll_b_deg, (double)ss.last_pitch_b_deg,
            (double)fabsf(ss.last_roll_deg  - ss.last_roll_b_deg),
            (double)fabsf(ss.last_pitch_deg - ss.last_pitch_b_deg),
            (unsigned long)ss.comm_stale_a, (unsigned long)ss.comm_stale_b,
            (double)ss.last_spread_deg, (unsigned)light_get());
        char hdr[128];
        snprintf(hdr, sizeof(hdr),
            "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
            "Content-Length: %d\r\nConnection: close\r\n\r\n", (int)strlen(json));
        client.print(hdr); client.print(json);
        return;
    }

    // ── GET /api/js ───────────────────────────────────────────────────────────
    if (path_js) {
        client.print("HTTP/1.1 200 OK\r\nContent-Type: text/javascript\r\n"
                     "Cache-Control: no-store\r\nConnection: close\r\n\r\n");
        wdt_feed();
        client.print("var wm={DRIFT:'anchor_max_deg',HOLD:'instant_hold_ms',FILL:'window_s',"
                     "GYRO_SPK:'motion_samples_max',ACCEL_SPK:'shock_samples_max',"
                     "SPREAD:'spread_max_deg',DIVERGE:'dual_diverge_deg'};");
        wdt_feed();
        client.print("setInterval(function(){"
                     "fetch('/api/status').then(function(r){return r.json();}).then(function(d){");
        client.print("document.getElementById('st').innerHTML="
                     "d.stable?\"<span class='ok'>TRUE</span>\":\"<span class='fail'>FALSE</span>\";");
        client.print("var wc=d.why==='OK'?'ok':(d.why==='SHADOW'||d.why==='CRC'||d.why==='FAULT'?'fail':'warn');"
                     "document.getElementById('why').innerHTML=\"<span class='\"+wc+\"'>\"+d.why+\"</span>\";");
        client.print("document.getElementById('flt').textContent=d.fault;"
                     "document.getElementById('fill').textContent=d.fill+'%';"
                     "document.getElementById('gsp').textContent=d.gyro_spk+' / '+d.gyro_max;"
                     "document.getElementById('asp').textContent=d.accel_spk+' / '+d.accel_max;");
        wdt_feed();
        client.print("document.getElementById('omg').textContent=d.omega.toFixed(2)+' dps';"
                     "document.getElementById('roll').textContent=d.roll.toFixed(2)+' deg';"
                     "document.getElementById('ptch').textContent=d.pitch.toFixed(2)+' deg';"
                     "document.getElementById('rollb').textContent=d.roll_b.toFixed(2)+' deg';"
                     "document.getElementById('ptchb').textContent=d.pitch_b.toFixed(2)+' deg';");
        client.print("document.getElementById('dvgr').textContent=d.diverge_r.toFixed(2)+' deg';"
                     "document.getElementById('dvgp').textContent=d.diverge_p.toFixed(2)+' deg';"
                     "document.getElementById('stla').textContent=d.stale_a;"
                     "document.getElementById('stlb').textContent=d.stale_b;"
                     "document.getElementById('sprd').textContent=d.spread.toFixed(2)+' deg';");
        client.print("var inh=document.getElementById('inh');"
                     "inh.innerHTML=d.inhibit?\"<span class='fail'>ACTIVE</span>\":\"<span class='ok'>off</span>\";"
                     "var btn=document.getElementById('inhbtn');"
                     "btn.textContent=d.inhibit?'Release Output':'Inhibit Output';"
                     "btn.style.borderColor=d.inhibit?'#f44':'#4af';"
                     "btn.style.color=d.inhibit?'#f44':'#4af';"
                     "btn.style.background=d.inhibit?'#2a0000':'#234';"
                     "var lb=document.getElementById('lightbtn');"
                     "lb.textContent=d.light?'Light ON':'Light OFF';"
                     "lb.style.borderColor=d.light?'#4f4':'#4af';"
                     "lb.style.color=d.light?'#4f4':'#4af';"
                     "lb.style.background=d.light?'#002200':'#234';");
        wdt_feed();
        client.print("document.querySelectorAll('input[type=number]').forEach(function(e){"
                     "e.style.borderColor='#555';e.style.background='#1a1a1a';});"
                     "var p=wm[d.why];"
                     "if(p){var e=document.querySelector('input[name=\"'+p+'\"]');"
                     "if(e){e.style.borderColor='#fa0';e.style.background='#2a1800';}}");
        client.print("});},300);");
        client.print("function applyParams(d){"
                     "Object.keys(d).forEach(function(k){"
                     "var c=document.getElementById('cv_'+k);if(c)c.textContent=d[k];});}");
        wdt_feed();
        client.print("document.getElementById('cfg').addEventListener('submit',function(e){"
                     "e.preventDefault();"
                     "var data=new URLSearchParams(new FormData(this)).toString();"
                     "fetch('/config',{method:'POST',"
                     "headers:{'Content-Type':'application/x-www-form-urlencoded'},body:data})"
                     ".then(function(){return fetch('/api/params');})"
                     ".then(function(r){return r.json();}).then(applyParams).catch(function(){});});");
        client.print("document.getElementById('rmb').addEventListener('click',function(){"
                     "fetch('/api/params').then(function(r){return r.json();}).then(function(d){"
                     "Object.keys(d).forEach(function(k){"
                     "var i=document.querySelector('input[name=\"'+k+'\"]');"
                     "var c=document.getElementById('cv_'+k);"
                     "if(i)i.value=d[k];if(c)c.textContent=d[k];});});});");
        // CR-3: tokens read from data-auth attributes (set server-side); never emitted here.
        client.print("document.getElementById('inhbtn').addEventListener('click',function(){"
                     "var tok=document.getElementById('inhbtn').dataset.auth||'';"
                     "fetch('/test/toggle-output',{method:'POST',"
                     "headers:{'Content-Type':'application/x-www-form-urlencoded'},"
                     "body:'auth='+encodeURIComponent(tok)}).catch(function(){});});");
        client.print("document.getElementById('lightbtn').addEventListener('click',function(){"
                     "var tok=document.getElementById('lightbtn').dataset.auth||'';"
                     "fetch('/light/toggle',{method:'POST',"
                     "headers:{'Content-Type':'application/x-www-form-urlencoded'},"
                     "body:'auth='+encodeURIComponent(tok)}).catch(function(){});});");
        wdt_feed();
        return;
    }

    // ── GET / -- HTML dashboard ───────────────────────────────────────────────
    if (is_get) {
        // Snapshot safety status atomically
        SafetyStatus ss;
        noInterrupts();
        memcpy(&ss, (const void*)&g_safety_status, sizeof(ss));
        interrupts();
        uint32_t ws = ss.window_samples > 0U ? ss.window_samples : 1U;

        client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n");
        wdt_feed(); client.flush(); wdt_feed();
        auto W = [&](const char *s) { client.print(s); };

        W("<!DOCTYPE html><html><head><meta charset='utf-8'>"
          "<title>Platform Stability v2</title>"
          "<style>body{font-family:monospace;margin:20px;background:#111;color:#ccc}"
          "h2{color:#4af}h3{color:#8cf;margin:12px 0 4px}"
          "table{border-collapse:collapse}td,th{padding:4px 10px;border:1px solid #444}"
          "th{background:#222}input[type=number]{width:90px;background:#1a1a1a;color:#eee;border:1px solid #555}"
          ".ok{color:#4f4}.fail{color:#f44}.warn{color:#fa0}"
          "input[type=submit],input[type=button]{padding:6px 18px;background:#234;color:#4af;"
          "border:1px solid #4af;cursor:pointer}.cur{color:#888;font-size:0.9em}"
          "</style></head><body>");

        W("<h2>Status</h2><table>");
        W("<tr><th>stable</th><td id='st'>");
        W(ss.platform_stable ? "<span class='ok'>TRUE</span>" : "<span class='fail'>FALSE</span>");
        W("</td></tr>");

        char tmp[160];
        { const char *wc = strcmp(ss.why_str,"OK")==0 ? "ok" :
              (strcmp(ss.why_str,"SHADOW")==0||strcmp(ss.why_str,"CRC")==0||
               strcmp(ss.why_str,"FAULT")==0 ? "fail" : "warn");
          snprintf(tmp, sizeof(tmp),
              "<tr><th>why</th><td id='why'><span class='%s'>%s</span></td></tr>",
              wc, ss.why_str);
          W(tmp); }
        snprintf(tmp, sizeof(tmp),
            "<tr><th>fault</th><td id='flt'>0x%04X</td></tr>", (unsigned)ss.fault_mask);
        W(tmp);
        snprintf(tmp, sizeof(tmp), "<tr><th>fill</th><td id='fill'>%d%%</td></tr>",
            (int)(100U * (ss.buf_count < ws ? ss.buf_count : ws) / ws));
        W(tmp);
        snprintf(tmp, sizeof(tmp), "<tr><th>gyro spikes</th><td id='gsp'>%lu / %lu</td></tr>",
            (unsigned long)ss.spike_gyro_count, (unsigned long)g_params.motion_samples_max);
        W(tmp);
        snprintf(tmp, sizeof(tmp), "<tr><th>accel spikes</th><td id='asp'>%lu / %lu</td></tr>",
            (unsigned long)ss.spike_accel_count, (unsigned long)g_params.shock_samples_max);
        W(tmp);
        snprintf(tmp, sizeof(tmp), "<tr><th>omega</th><td id='omg'>%.2f dps</td></tr>",
            (double)ss.last_omega_dps);
        W(tmp);

        W("<tr><th colspan='2' style='background:#1a2a1a;color:#8cf'>IMU-A (primary)</th></tr>");
        snprintf(tmp, sizeof(tmp), "<tr><th>roll A</th><td id='roll'>%.2f deg</td></tr>",
            (double)ss.last_roll_deg);   W(tmp);
        snprintf(tmp, sizeof(tmp), "<tr><th>pitch A</th><td id='ptch'>%.2f deg</td></tr>",
            (double)ss.last_pitch_deg);  W(tmp);
        snprintf(tmp, sizeof(tmp), "<tr><th>stale A</th><td id='stla'>%lu</td></tr>",
            (unsigned long)ss.comm_stale_a); W(tmp);

        W("<tr><th colspan='2' style='background:#1a1a2a;color:#8cf'>IMU-B (cross-check)</th></tr>");
        snprintf(tmp, sizeof(tmp), "<tr><th>roll B</th><td id='rollb'>%.2f deg</td></tr>",
            (double)ss.last_roll_b_deg);  W(tmp);
        snprintf(tmp, sizeof(tmp), "<tr><th>pitch B</th><td id='ptchb'>%.2f deg</td></tr>",
            (double)ss.last_pitch_b_deg); W(tmp);
        snprintf(tmp, sizeof(tmp), "<tr><th>stale B</th><td id='stlb'>%lu</td></tr>",
            (unsigned long)ss.comm_stale_b); W(tmp);

        W("<tr><th colspan='2' style='background:#2a1a1a;color:#8cf'>Cross-check</th></tr>");
        snprintf(tmp, sizeof(tmp), "<tr><th>|dRoll|</th><td id='dvgr'>%.2f deg</td></tr>",
            (double)fabsf(ss.last_roll_deg - ss.last_roll_b_deg));   W(tmp);
        snprintf(tmp, sizeof(tmp), "<tr><th>|dPitch|</th><td id='dvgp'>%.2f deg</td></tr>",
            (double)fabsf(ss.last_pitch_deg - ss.last_pitch_b_deg)); W(tmp);
        snprintf(tmp, sizeof(tmp), "<tr><th>spread</th><td id='sprd'>%.2f deg</td></tr>",
            (double)ss.last_spread_deg); W(tmp);
        snprintf(tmp, sizeof(tmp),
            "<tr><th>output inhibit</th>"
            "<td id='inh'><span class='%s'>%s</span></td></tr>",
            ss.output_inhibit ? "fail" : "ok",
            ss.output_inhibit ? "ACTIVE" : "off");
        W(tmp);
        W("</table>");
        // CR-3: token injected server-side into data-auth; never in /api/js response.
        { char ibtn[300];
          snprintf(ibtn, sizeof(ibtn),
              "<p style='margin:10px 0 4px'>"
              "<button id='inhbtn' data-auth='%s' "
              "style='padding:6px 18px;cursor:pointer;border:1px solid'>"
              "...</button>"
              " <span style='color:#888;font-size:0.85em'>BENCH TESTING ONLY -- "
              "suppresses SAFE_OUT_PIN. Never use during deployment.</span></p>",
              CONFIG_AUTH_TOKEN);
          W(ibtn); }
        { char lbtn[256];
          snprintf(lbtn, sizeof(lbtn),
              "<p style='margin:4px 0'>"
              "<button id='lightbtn' data-auth='%s' "
              "style='padding:6px 18px;cursor:pointer;border:1px solid #4af;"
              "color:#4af;background:#234'>Light OFF</button>"
              " <span style='color:#888;font-size:0.85em'>"
              "Odin GL remote PWM &mdash; pin 4, open-drain 93.75 Hz</span></p>",
              CONFIG_AUTH_TOKEN);
          W(lbtn); }
        wdt_feed(); client.flush(); wdt_feed();

        W("<h2>Parameters</h2>"
          "<form id='cfg' method='POST' action='/config'><table>"
          "<tr><th>Parameter</th><th>New Value</th><th>Current</th></tr>");

        auto row_f = [&](const char *label, const char *name, float v, float step) {
            char r[256];
            snprintf(r, sizeof(r),
                "<tr><td>%s</td>"
                "<td><input type='number' name='%s' value='%.4g' step='%g'></td>"
                "<td class='cur' id='cv_%s'>%.4g</td></tr>",
                label, name, (double)v, (double)step, name, (double)v);
            W(r);
        };
        auto row_u = [&](const char *label, const char *name, uint32_t v) {
            char r[256];
            snprintf(r, sizeof(r),
                "<tr><td>%s</td>"
                "<td><input type='number' name='%s' value='%lu' step='1'></td>"
                "<td class='cur' id='cv_%s'>%lu</td></tr>",
                label, name, (unsigned long)v, name, (unsigned long)v);
            W(r);
        };

        row_u("window_s (1-5)",            "window_s",             g_params.window_s);
        row_f("omega_stable_dps",          "omega_stable_dps",     g_params.omega_stable_dps,     0.5f);
        row_f("omega_instant_dps",         "omega_instant_dps",    g_params.omega_instant_dps,    0.5f);
        row_u("instant_hold_ms",           "instant_hold_ms",      g_params.instant_hold_ms);
        row_f("spike_accel_g",             "spike_accel_g",        g_params.spike_accel_g,        0.01f);
        row_u("eval_period_ms",            "eval_period_ms",       g_params.eval_period_ms);
        row_u("motion_samples_max",        "motion_samples_max",   g_params.motion_samples_max);
        row_u("shock_samples_max",         "shock_samples_max",    g_params.shock_samples_max);
        row_u("clean_streak_needed",       "clean_streak_needed",  g_params.clean_streak_needed);
        row_f("diverse_mean_tol_g",        "diverse_mean_tol_g",   g_params.diverse_mean_tol_g,   0.005f);
        row_f("spread_max_deg",            "spread_max_deg",       g_params.spread_max_deg,       0.5f);
        row_f("anchor_max_deg",            "anchor_max_deg",       g_params.anchor_max_deg,       0.5f);
        row_f("gyro_range_fault_dps",      "gyro_range_fault_dps", g_params.gyro_range_fault_dps, 1.0f);
        row_u("stale_fault_ticks",         "stale_fault_ticks",    g_params.stale_fault_ticks);
        row_f("dual_diverge_deg",          "dual_diverge_deg",     g_params.dual_diverge_deg,     0.5f);

        W("</table>"
          "<br><input type='hidden' name='auth' value='");
        W(CONFIG_AUTH_TOKEN);
        W("'><input type='submit' value='Apply + Reset Stability'>"
          " <input type='button' id='rmb' value='Read from Memory'>"
          "</form>");
        wdt_feed(); client.flush(); wdt_feed();

        W("<script src='/api/js'></script></body></html>");
    }
}

// --- app_init() --------------------------------------------------------------
void app_init()
{
    Ethernet.begin(WEB_IP, WEB_SUBNET, WEB_GW, WEB_DNS);
    s_web_server.begin();
    char ebuf[64];
    snprintf(ebuf, sizeof(ebuf), "[ETH] http://%d.%d.%d.%d/",
        WEB_IP[0], WEB_IP[1], WEB_IP[2], WEB_IP[3]);
    Serial.println(ebuf);
}

// --- app_tick() --------------------------------------------------------------
void app_tick()
{
    static uint32_t s_web_ms_max = 0U;
    uint32_t t_web0 = millis();

    if (!s_web_client || !s_web_client.connected()) {
        s_web_client = s_web_server.accept();
        if (s_web_client) s_web_client_accept_ms = millis();
    } else if (!s_web_client.available()) {
        if (millis() - s_web_client_accept_ms > WEB_CLIENT_TIMEOUT_MS)
            s_web_client.stop();
    }
    if (s_web_client && s_web_client.available()) {
        http_handle(s_web_client);
        s_web_client.stop();
    }
    Ethernet.loop();

    uint32_t t_web = millis() - t_web0;
    if (t_web > s_web_ms_max) s_web_ms_max = t_web;
}

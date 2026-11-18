#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include "NetworkConfig.h"
#include <esp_http_server.h>
#include "ControlSession.h"
#include "PhonePage.h"
#include "WebBench.h"

class PhoneLink {
 public:
  static constexpr size_t STATUS_SIZE=768, REPORT_SIZE=8192, TUNE_SIZE=3072;
  static constexpr size_t SETTINGS_SIZE=1024;
  bool begin() {
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);
    WiFi.setHostname(quadNetwork::HOSTNAME);
    if (!WiFi.softAP("QuadLevel","CHANGE_ME_123",1,false,1)) return false;
    WiFi.setAutoReconnect(true);
    WiFi.begin(quadNetwork::HOME_SSID,quadNetwork::HOME_PASSWORD);
    // Bounded startup wait, before IMU sampling or any arming is possible.
    // The direct access point stays available if the home router is absent.
    for(unsigned attempt=0;attempt<80 && WiFi.status()!=WL_CONNECTED;++attempt) delay(100);
    if(MDNS.begin(quadNetwork::HOSTNAME)) MDNS.addService("http","tcp",80);
    httpd_config_t config=HTTPD_DEFAULT_CONFIG();
    config.task_priority=2;
    config.stack_size=14336; // Report snapshot plus the previous 6 KiB stack margin.
    config.max_open_sockets=4;
    config.lru_purge_enable=true;
    config.recv_wait_timeout=1;
    config.send_wait_timeout=1;
    if (httpd_start(&server,&config)!=ESP_OK) return false;
    const httpd_uri_t routes[]={
      {.uri="/",.method=HTTP_GET,.handler=page,.user_ctx=this},
      {.uri="/status",.method=HTTP_GET,.handler=status,.user_ctx=this},
      {.uri="/control",.method=HTTP_POST,.handler=control,.user_ctx=this},
      {.uri="/report",.method=HTTP_GET,.handler=report,.user_ctx=this},
      {.uri="/tune",.method=HTTP_GET,.handler=tune,.user_ctx=this},
      {.uri="/settings",.method=HTTP_GET,.handler=settings,.user_ctx=this},
      {.uri="/setup",.method=HTTP_POST,.handler=setupCommand,.user_ctx=this}
    };
    for (const auto &route:routes)
      if (httpd_register_uri_handler(server,&route)!=ESP_OK) return false;
    return true;
  }
  bool take(ControlPacket &packet) {
    bool result=false;
    portENTER_CRITICAL(&mux);
    if (stopPending) {
      packet={}; packet.action=0;
      stopPending=pending=onlinePending=false; result=true;
    } else if (pending) { packet=mailbox; pending=false; result=true; }
    portEXIT_CRITICAL(&mux);
    return result;
  }
  bool takeOnline(OnlinePacket &packet){
    bool result=false;portENTER_CRITICAL(&mux);
    if(stopPending)onlinePending=false;
    else if(onlinePending){packet=onlineMailbox;onlinePending=false;result=true;}
    portEXIT_CRITICAL(&mux);return result;
  }
  void publishSettings(const char *text){
    portENTER_CRITICAL(&mux);snprintf(savedSettings,sizeof(savedSettings),"%s",text);portEXIT_CRITICAL(&mux);
  }
  void publish(const char *json) {
    portENTER_CRITICAL(&mux);
    snprintf(state,sizeof(state),"%s",json);
    portEXIT_CRITICAL(&mux);
  }
  void publishReport(const char *text) {
    portENTER_CRITICAL(&mux);
    snprintf(savedReport,sizeof(savedReport),"%s",text);
    portEXIT_CRITICAL(&mux);
  }
  void publishTuneReport(const char *text) {
    size_t count=0;
    while (count+1<sizeof(savedTune) && text[count]) ++count;
    portENTER_CRITICAL(&mux);
    memcpy(savedTune,text,count); savedTune[count]=0;
    portEXIT_CRITICAL(&mux);
  }
 private:
  httpd_handle_t server=nullptr;
  portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
  ControlPacket mailbox;
  bool pending=false,stopPending=false;
  OnlinePacket onlineMailbox;bool onlinePending=false;
  char savedSettings[SETTINGS_SIZE]="{}";
  char state[STATUS_SIZE]="{\"ready\":false,\"fault\":false,\"armed\":false,\"canArm\":false,\"calibration\":0,\"nonce\":0,\"roll\":0,\"pitch\":0,\"base\":1000,\"target\":1000,\"motors\":[1000,1000,1000,1000],\"reason\":\"starting\"}";
  char savedReport[REPORT_SIZE]="No stopped test recorded yet.\n";
  char savedTune[TUNE_SIZE]="No automatic PID test recorded yet. Start from USB on the secured fixture.\n";
  static PhoneLink &instance(httpd_req_t *r) { return *static_cast<PhoneLink *>(r->user_ctx); }
  static void noCache(httpd_req_t *r) { httpd_resp_set_hdr(r,"Cache-Control","no-store"); }
  static esp_err_t page(httpd_req_t *r) {
    noCache(r); httpd_resp_set_type(r,"text/html");
    return httpd_resp_send(r,PHONE_PAGE,HTTPD_RESP_USE_STRLEN);
  }
  static esp_err_t status(httpd_req_t *r) {
    auto &link=instance(r); char copy[STATUS_SIZE];
    portENTER_CRITICAL(&link.mux); memcpy(copy,link.state,sizeof(copy)); portEXIT_CRITICAL(&link.mux);
    noCache(r); httpd_resp_set_type(r,"application/json");
    return httpd_resp_send(r,copy,HTTPD_RESP_USE_STRLEN);
  }
  static esp_err_t report(httpd_req_t *r) {
    auto &link=instance(r); char copy[REPORT_SIZE];
    portENTER_CRITICAL(&link.mux); memcpy(copy,link.savedReport,sizeof(copy)); portEXIT_CRITICAL(&link.mux);
    noCache(r); httpd_resp_set_type(r,"text/plain");
    return httpd_resp_send(r,copy,HTTPD_RESP_USE_STRLEN);
  }
  // A snapshot only: this GET never creates, renews, stops, or restarts a run.
  static esp_err_t tune(httpd_req_t *r) {
    auto &link=instance(r); char copy[TUNE_SIZE];
    portENTER_CRITICAL(&link.mux); memcpy(copy,link.savedTune,sizeof(copy)); portEXIT_CRITICAL(&link.mux);
    noCache(r); httpd_resp_set_type(r,"text/plain");
    return httpd_resp_send(r,copy,HTTPD_RESP_USE_STRLEN);
  }
  static esp_err_t settings(httpd_req_t *r){
    auto &link=instance(r);char copy[SETTINGS_SIZE];
    portENTER_CRITICAL(&link.mux);memcpy(copy,link.savedSettings,sizeof(copy));portEXIT_CRITICAL(&link.mux);
    noCache(r);httpd_resp_set_type(r,"application/json");return httpd_resp_send(r,copy,HTTPD_RESP_USE_STRLEN);
  }
  static esp_err_t setupCommand(httpd_req_t *r){
    if(r->content_len<=0||r->content_len>=384)return httpd_resp_send_err(r,HTTPD_400_BAD_REQUEST,"Invalid setup length");
    char body[384];size_t got=0;
    while(got<(size_t)r->content_len){const int n=httpd_req_recv(r,body+got,r->content_len-got);if(n<=0)return ESP_FAIL;got+=n;}
    body[got]=0;OnlinePacket p;
    if(!OnlinePacket::parse(body,p))return httpd_resp_send_err(r,HTTPD_400_BAD_REQUEST,"Invalid setup values");
    p.received=millis();auto &link=instance(r);
    portENTER_CRITICAL(&link.mux);link.onlineMailbox=p;link.onlinePending=true;portEXIT_CRITICAL(&link.mux);
    noCache(r);return httpd_resp_send(r,"Queued; check settings acknowledgement",HTTPD_RESP_USE_STRLEN);
  }
  static esp_err_t control(httpd_req_t *r) {
    if (r->content_len<=0 || r->content_len>=128)
      return httpd_resp_send_err(r,HTTPD_400_BAD_REQUEST,"Invalid command length");
    char body[128]; size_t got=0;
    while (got<(size_t)r->content_len) {
      const int n=httpd_req_recv(r,body+got,r->content_len-got);
      if (n<=0) return ESP_FAIL;
      got+=(size_t)n;
    }
    body[got]=0;
    ControlPacket p;
    unsigned long nonce=0,session=0,seq=0; int consumed=-1;
    if (sscanf(body,"%lu %lu %lu %d %d %d %d %d%n",&nonce,&session,&seq,&p.action,
               &p.throttle,&p.roll,&p.pitch,&p.yaw,&consumed)!=8 || consumed!=(int)got)
      return httpd_resp_send_err(r,HTTPD_400_BAD_REQUEST,"Invalid command fields");
    p.nonce=nonce; p.session=session; p.sequence=seq; p.received=millis();
    auto &link=instance(r);
    portENTER_CRITICAL(&link.mux);
    if (p.action==0) link.stopPending=true;
    else { link.mailbox=p; link.pending=true; }
    portEXIT_CRITICAL(&link.mux);
    noCache(r);
    return httpd_resp_send(r,"OK",2);
  }
};

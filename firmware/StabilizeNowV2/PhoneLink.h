#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <esp_http_server.h>
#include "ControlSession.h"
#include "PhonePage.h"

class PhoneLink {
 public:
  static constexpr size_t STATUS_SIZE=512, REPORT_SIZE=2048;
  bool begin() {
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    if (!WiFi.softAP("QuadLevel","CHANGE_ME_123",1,false,1)) return false;
    httpd_config_t config=HTTPD_DEFAULT_CONFIG();
    config.task_priority=2;
    config.stack_size=6144;
    config.max_open_sockets=4;
    config.lru_purge_enable=true;
    config.recv_wait_timeout=1;
    config.send_wait_timeout=1;
    if (httpd_start(&server,&config)!=ESP_OK) return false;
    const httpd_uri_t routes[]={
      {.uri="/",.method=HTTP_GET,.handler=page,.user_ctx=this},
      {.uri="/status",.method=HTTP_GET,.handler=status,.user_ctx=this},
      {.uri="/control",.method=HTTP_POST,.handler=control,.user_ctx=this},
      {.uri="/report",.method=HTTP_GET,.handler=report,.user_ctx=this}
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
      stopPending=pending=false; result=true;
    } else if (pending) { packet=mailbox; pending=false; result=true; }
    portEXIT_CRITICAL(&mux);
    return result;
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
 private:
  httpd_handle_t server=nullptr;
  portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
  ControlPacket mailbox;
  bool pending=false,stopPending=false;
  char state[STATUS_SIZE]="{\"ready\":false,\"fault\":false,\"armed\":false,\"canArm\":false,\"calibration\":0,\"nonce\":0,\"roll\":0,\"pitch\":0,\"base\":1000,\"target\":1000,\"motors\":[1000,1000,1000,1000],\"reason\":\"starting\"}";
  char savedReport[REPORT_SIZE]="No stopped test recorded yet.\n";
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

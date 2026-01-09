#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPmDNS.h>
#include <WiFi.h>

#include ".config.hpp"
#include "assert.hpp"
#include "esp32-hal-adc-hack.hpp"
#include "interpoly.hpp"
#include "preferences.hpp"

Preferences preferences;

bool on = true;

inline static const std::string POLY_SUFFIX{"-poly"};
struct Pin {
  const uint8_t pin;
  const char *key;
  PolyXY poly;
  Pin(uint8_t pin, const char *key) : pin(pin), key(key) {}
  void begin(Preferences &preferences) {
    loadPoly(poly, (key + POLY_SUFFIX).c_str(), preferences);
    Serial.printf("LOADED: %s:", key);
    for (const auto &[x, y] : poly) {
      Serial.printf(" %d=%d", x, y);
    }
    Serial.println();
  }
  void calibrateDone() { CHECK_VA(storePoly(poly, (key + POLY_SUFFIX).c_str(), preferences), key); }
  void calibrateSet(uint16_t duty, float value) {
    const auto &item = poly.emplace_back(duty, (int16_t)(value * 1e3f));
    Serial.printf("CALIBRATE: %s %d %d=%d\n", key, poly.size(), item.first, item.second);
    std::sort(poly.begin(), poly.end());
  }
  uint16_t calibrateNext(float threshold) {
    switch (poly.size()) {
    case 0:
      return 1024;
    case 1:
      return 0;
    case 2:
      return (poly[0].first + poly[1].first) / 2;
    }
    const int32_t thresh = threshold * 1e3f;
    bool pf = false;
    for (size_t i = 2; i < poly.size(); ++i) {
      const auto &c = poly[i], &p = poly[i - 1], &pp = poly[i - 2];
      const auto xv = lerpi(p.first, c, pp, F2S), v = p.second;
      const auto e = abs(1000 - (int)v * 1000 / xv);
      if (e < 10) {
        pf = false;
        continue;
      }
      if (pf)
        return (p.first + pp.first) / 2;
      pf = max(c.second, p.second) > thresh;
      if ((i == 2) && (max(p.second, pp.second) > thresh))
        return (p.first + pp.first) / 2;
    }
    const auto &c = poly.back(), &p = *(poly.rbegin() + 1);
    if (pf and (max(c.second, p.second) > thresh))
      return (c.first + p.first) / 2;
    return 0xFFFF;
  }
};
struct PinPwm : Pin {
  using Base = Pin;
  const Order order;
  float value;
  uint16_t duty;
  PinPwm(uint8_t pin, const char *key, const Order &order) : Base(pin, key), order(order) {}
  bool begin(Preferences &preferences) {
    Base::begin(preferences);
    if (!ledcAttach(pin, 39138, 10))
      return false;
    setValue(0);
    return true;
  }
  void setValue(float v) {
    value = v;
    const auto mv = on ? (int16_t)(v * 1e3f) : 0;
    setDuty(interpoly(mv, poly, S2F, order));
  }
  void restoreValue() { setValue(value); }
  void setDuty(int16_t v) {
    duty = constrain(v, 0, 1024);
    Serial.printf("PWM: %s=%d\n", key, duty);
    CHECK_VA(ledcWrite(pin, duty), "(%d,%d)", pin, duty);
  }
  bool calibrate(const std::string &rest, float threshold) {
    if (rest == "DONE") {
      calibrateDone();
      restoreValue();
      return false;
    }
    if (rest == "RESTART") {
      poly.clear();
    } else if (rest != "CONTINUE") {
      calibrateSet(duty, std::stof(rest));
    }
    if (const auto nd = calibrateNext(threshold); nd != 0xFFFF)
      setDuty(nd);
    else {
      Serial.printf("CALIBRATED: %d\n", poly.size());
      return false;
    }
    return true;
  }
} pwm_v(10, "pwm-mv", DESC), pwm_a(20, "pwm-ma", ASC);

struct PinAdc : Pin {
  using Base = Pin;
  float avg;
  PinAdc(uint8_t pin, const char *key) : Base(pin, key) {}
  float getValue() const { return interpoly((int16_t)(avg * 8), poly, F2S, ASC) / 1e3f; }
} adc_v(3, "adc-mv"), adc_a(4, "adc-ma");

struct : public AsyncServer {
  using AsyncServer::_port;
} server(5555);

std::string ff3n(float f) {
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "%.3f\n", f);
  return std::string(buffer);
}
std::initializer_list<std::pair<std::string, std::function<std::string(const std::string &suffix)>>> COMMANDS{
    {"*IDN?\n", [](const auto &) { return "igelbox,EPS1,0,0.1\n"; }},
    {"OUTPUT:CVCC? CH1\n", [](const auto &) { return "CV\n"; }}, // TODO
    {"MEASURE:VOLTAGE? CH1\n", [](const auto &) { return ff3n(adc_v.getValue()); }},
    {"SOURCE1:VOLTAGE?\n", [](const auto &) { return ff3n(pwm_v.value); }},
    {"MEASURE:CURRENT? CH1\n", [](const auto &) { return ff3n(adc_a.getValue()); }},
    {"SOURCE1:CURRENT?\n", [](const auto &) { return ff3n(pwm_a.value); }},

    // TODO
    {"SOURCE1:CURRENT:PROTECTION:STATE?\n", [](const auto &) { return "OFF\n"; }},
    {"SOURCE1:CURRENT:PROTECTION:TRIPPED?\n", [](const auto &) { return "NO\n"; }},
    {"SOURCE1:CURRENT:PROTECTION:", [](const auto &) { return ""; }},

    {"OUTPUT? CH1\n", [](const auto &) { return on ? "ON\n" : "OFF\n"; }},
    {"OUTPUT CH1,",
     [](const auto &rest) {
       on = rest == "ON";
       pwm_v.restoreValue();
       pwm_a.restoreValue();
       return "";
     }},
    {"SOURCE1:VOLTAGE ",
     [](const auto &rest) {
       pwm_v.setValue(std::stof(rest));
       return "";
     }},
    {"SOURCE1:CURRENT ",
     [](const auto &rest) {
       pwm_a.setValue(std::stof(rest));
       return "";
     }},

    {"CALIBRATE:OUTPUT:VOLTAGE CH1,",
     [](const auto &rest) {
       if (pwm_v.calibrate(rest, 1 /*1.25 is XL4015 lower reliable bound*/))
         pwm_a.setDuty(1024 /*unlimited*/);
       else
         pwm_a.restoreValue();
       return "";
     }},
    {"CALIBRATE:OUTPUT:CURRENT CH1,",
     [](const auto &rest) {
       if (pwm_a.calibrate(rest, 0))
         pwm_v.setDuty(0 /*unlimited*/);
       else
         pwm_v.restoreValue();
       return "";
     }},
};

void setup() {
  pinMode(pwm_v.pin, OUTPUT);
  digitalWrite(pwm_v.pin, HIGH);

  setCpuFrequencyMhz(80); // reduce heating a bit
  delay(500);             // give a time to connect monitor
  Serial.begin(115200);

  ASSERT(preferences.begin("epsu"));
  for (const auto pwm : {&pwm_v, &pwm_a})
    ASSERT_VA(pwm->begin(preferences), "(%d)", pwm->pin);
  for (const auto adc : {&adc_v, &adc_a})
    adc->begin(preferences);
  {
    analogContinuousSetAtten(ADC_0db);
    uint8_t pins[2] = {adc_v.pin, adc_a.pin};
    ASSERT(analogContinuous(pins, 2, 4092 / 2 / SOC_ADC_DIGI_RESULT_BYTES, 1024, nullptr));
    ASSERT(analogContinuousStart());
  }

  for (auto s = WiFi.begin(WIFI_SSID, WIFI_PASW); s != WL_CONNECTED; s = WiFi.status()) {
    Serial.print(s);
    delay(1000);
  }
  CHECK(WiFi.setTxPower(WIFI_POWER_2dBm));
  Serial.printf("TxPower: %d\n", WiFi.getTxPower());
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  CHECK(MDNS.begin("epsu"));
  Serial.println("mDNS responder started");

  server.onClient(
      [](void *arg, AsyncClient *client) {
        Serial.printf("\nNew client connected: %p ", client);
        Serial.print(client->remoteIP());
        Serial.print(":");
        Serial.println(client->remotePort());

        using Buffer = std::string;
        const auto buffer = new Buffer();
        try {
          client->onDisconnect(
              [](void *arg, AsyncClient *client) {
                Serial.printf("\ndisconnected: %p\n", client);
                // Free up resources
                delete (Buffer *)arg;
                delete client;
              },
              (void *)buffer);
        } catch (...) {
          delete buffer;
          throw;
        }
        client->onData(
            [](void *arg, AsyncClient *client, void *data, size_t len) {
              // Serial.printf("\nData received from client %p %p [%d bytes]:", client, arg, len);
              const auto chars = (const char *)data;
              // for (size_t i = 0; i < len; i++) {
              //   Serial.print((char)chars[i]);
              // }
              // Serial.println();
              auto &buffer = *(Buffer *)arg;
              buffer.append(chars, len);

              if (const auto i = buffer.find('\n'); i != std::string::npos) {
                // Serial.printf("Command: '%s'", buffer.substr(0, i).c_str());
                bool found = false;
                for (const auto &[prefix, command] : COMMANDS) {
                  const auto &prefix_length = prefix.length();
                  if (prefix_length > i) {
                    if ((prefix_length == i + 1) && (prefix[i] == '\n')) {
                      // full command
                    } else
                      continue;
                  }
                  if (buffer.starts_with(prefix)) {
                    found = true;
                    // Serial.printf(" as %s\n", prefix.c_str());
                    const auto &response = command(buffer.substr(prefix_length, i - prefix_length));
                    if (!response.empty()) {
                      client->write(response.c_str(), response.length());
                    }
                  }
                }
                if (!found) {
                  Serial.printf("Unknown: %s\n", buffer.substr(0, i).c_str());
                }
                buffer.erase(0, i + 1);
              }

              if (!buffer.empty()) {
                Serial.printf("Buffer: '%s'\n", buffer.c_str());
              }
            },
            (void *)buffer);
        client->onError(
            [](void *arg, AsyncClient *client, int8_t error) {
              Serial.printf("\nConnection error for client %p. Error: %d\n", client, error);
            },
            nullptr);
      },
      nullptr);

  server.begin();
  Serial.printf("TCP Server started on port %d\n", server._port);
}

void loop() {
  adc_continuous_results_t results;
  if (CHECK(analogContinuousReadSumCount(results, 1000))) {
    for (const auto adc : {&adc_v, &adc_a}) {
      const auto &result = results[digitalPinToAnalogChannel(adc->pin)];
      // Serial.printf("%d\t%d\t%d\t%d\n", pin, digitalPinToAnalogChannel(pin), result.count, result.sum_read_raw);
      adc->avg = (float)result.sum_read_raw / (float)result.count;
    }
    Serial.printf("%d: u=%f\ti=%f\n", millis(), adc_v.avg, adc_a.avg);
  }
}

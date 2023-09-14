#include <secrets.h>
#include <math.h>
#include <Arduino.h>
#include <TCA9548.h>
#include <AHTxx.h>
#include <lcdgfx.h>
#include <lcdgfx_gui.h>

#include <WiFi.h>
#include <ArduinoHA.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

#define WDT_SECONDS 5 * 60

TCA9548 mp(0x70);
AHTxx inTempSensor(AHTXX_ADDRESS_X38, AHT2x_SENSOR);
AHTxx outTempSensor(AHTXX_ADDRESS_X38, AHT2x_SENSOR);
DisplaySH1106_128x64_I2C display(-1);
int previousMinute = -1;

#define LED_PIN 2
#define CHLOR_FLOW_PIN 13
#define HEAT_FLOW_PIN 14
#define PUMP_PIN 12

#define IN_TEMP_CH 0
#define OUT_TEMP_CH 1
#define DISP_CH 2

#define CHLOR_FLOW_PPL 12
#define HEAT_FLOW_PPL 27

#define WIFI_WAIT_TIME_MS 10000
#define TZ_DEF "AEST-10AEDT,M10.1.0,M4.1.0/3"
#define NTP_SERVER "time.google.com"

volatile unsigned long chlorFlowPulses = 0;
volatile unsigned long heatFlowPulses = 0;

AsyncWebServer server(80);
WiFiClient client;
HADevice device;
HAMqtt mqtt(client, device, 12);

HASwitch pump("pump");
HASensorNumber inTemp("in_temp", HASensorNumber::PrecisionP2);
HASensorNumber outTemp("out_temp", HASensorNumber::PrecisionP2);
HASensorNumber chlorFlow("chlor_flow", HASensorNumber::PrecisionP0);
HASensorNumber heatFlow("heat_flow", HASensorNumber::PrecisionP0);
HASensorNumber rssi("rssi", HASensorNumber::PrecisionP0);

HAHVAC hvac("pool_heater", HAHVAC::TargetTemperatureFeature | HAHVAC::PowerFeature | HAHVAC::ModesFeature);

void onTargetTemperatureCommand(HANumeric temperature, HAHVAC* sender) {
	int temp = round(temperature.toFloat());

    Serial.print("Target temperature: ");
    Serial.println(temp);

	mp.selectChannel(DISP_CH);
	display.setTextCursor(0, 24);
	display.write("Set: ");
	display.print(temp);

    sender->setTargetTemperature(temperature); // report target temperature back to the HA panel
}

void onPowerCommand(bool state, HAHVAC* sender) {
	mp.selectChannel(DISP_CH);
	display.setTextCursor(0, 8);
  if (state) {
    Serial.println("Power on");
	display.write("Power ON ");
  } else {
    Serial.println("Power off");
    display.write("Power OFF");
  }
}

void onModeCommand(HAHVAC::Mode mode, HAHVAC* sender) {
    Serial.print("Mode: ");
    if (mode == HAHVAC::OffMode) {
        Serial.println("off");
    } else if (mode == HAHVAC::HeatMode) {
        Serial.println("heat");
    }
    sender->setMode(mode); // report mode back to the HA panel
}

void IRAM_ATTR onChlorPulse() {
	chlorFlowPulses++;
}

void IRAM_ATTR onHeatPulse() {
	heatFlowPulses++;
}

void initWifi() {
	WiFi.setHostname("poolheater");
	WiFi.mode(WIFI_STA);
	WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
	WiFi.begin(WIFI_SSID, WIFI_PASSWD);
	while (WiFi.waitForConnectResult(WIFI_WAIT_TIME_MS) != WL_CONNECTED) {
		Serial.println("WiFi failed. Rebooting...");
		ESP.restart();
	}
	Serial.println("WiFi connected");
	Serial.print("IP: ");
	Serial.println(WiFi.localIP());
}

void initTime() {
	configTime(0, 0, NTP_SERVER);
	setenv("TZ", TZ_DEF, 1);
	tzset();
}

void initOta() {
	server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
		request->send(200, "text/plain", "Pool heater");
	});

	AsyncElegantOTA.begin(&server);
	server.begin();
}

void onSwitchCommand(bool state, HASwitch* sender) {
	mp.selectChannel(DISP_CH);
	display.setTextCursor(0, 16);
	display.write(state ? "Pump ON " : "Pump OFF");
    digitalWrite(LED_PIN, (state ? HIGH : LOW));
    digitalWrite(PUMP_PIN, (state ? HIGH : LOW));
    sender->setState(state); // report state back to the Home Assistant
}

void setup() {
	esp_task_wdt_init(WDT_SECONDS, true);
	esp_task_wdt_add(NULL);

	Serial.begin(115200);

	initWifi();
	initTime();
	initOta();

	byte mac[6];
	WiFi.macAddress(mac);
	device.setUniqueId(mac, sizeof(mac));
	device.setName("Pool Heater");
	device.setModel("Pool Heater");
	device.setSoftwareVersion("1.0.0");
	device.setManufacturer("Andrew");

	pump.setIcon("mdi:solar-power-variant");
	pump.setName("Pump");
	pump.onCommand(onSwitchCommand);

	rssi.setIcon("mdi:wifi");
	rssi.setName("RSSI");
	rssi.setUnitOfMeasurement("RSSI");

	inTemp.setIcon("mdi:thermometer-low");
	inTemp.setName("In Temperature");
	inTemp.setUnitOfMeasurement("C");

	outTemp.setIcon("mdi:thermometer-high");
	outTemp.setName("Out Temperature");
	outTemp.setUnitOfMeasurement("C");

	chlorFlow.setIcon("mdi:air-filter");
	chlorFlow.setName("Chlorinator Flow");
	chlorFlow.setUnitOfMeasurement("LPM");

	heatFlow.setIcon("mdi:heating-coil");
	heatFlow.setName("Heating Flow");
	heatFlow.setUnitOfMeasurement("LPM");

    hvac.setName("Pool Heater");
    hvac.setIcon("mdi:solar-power-variant");
    hvac.setModes(HAHVAC::OffMode | HAHVAC::HeatMode);
    hvac.setTemperatureUnit(HAHVAC::CelsiusUnit);
    hvac.setMinTemp(20);
    hvac.setMaxTemp(35);
    hvac.setTempStep(1);
    hvac.setRetain(true);
    hvac.onModeCommand(onModeCommand);
    hvac.onTargetTemperatureCommand(onTargetTemperatureCommand);
    hvac.onPowerCommand(onPowerCommand);

	mqtt.begin(MQTT_HOST, MQTT_USER, MQTT_PASSWD);

	Wire.begin();
	if (mp.begin() == false) {
		Serial.println("MP failed");
	}

	mp.selectChannel(IN_TEMP_CH);
	if (inTempSensor.begin() == false) {
		Serial.println("In temp failed");
	}

	mp.selectChannel(OUT_TEMP_CH);
	if (outTempSensor.begin() == false) {
		Serial.println("Out temp failed");
	}

	mp.selectChannel(DISP_CH);
	display.begin();
    display.setFixedFont(ssd1306xled_font6x8);
    display.clear();

	pinMode(LED_PIN, OUTPUT);
	pinMode(PUMP_PIN, OUTPUT);

	pinMode(CHLOR_FLOW_PIN, INPUT);
	attachInterrupt(digitalPinToInterrupt(CHLOR_FLOW_PIN), onChlorPulse, RISING);

	pinMode(HEAT_FLOW_PIN, INPUT);
	attachInterrupt(digitalPinToInterrupt(HEAT_FLOW_PIN), onHeatPulse, RISING);

	Serial.println("Started");
}

void loop() {

	struct tm time;
	getLocalTime(&time);

	if (time.tm_min != previousMinute) {
		esp_task_wdt_reset();

		previousMinute = time.tm_min;
		Serial.println(&time, "%A, %B %d %Y %H:%M:%S");

		if (WiFi.status() != WL_CONNECTED) {
			Serial.println("WiFi lost");
			WiFi.disconnect();
			WiFi.reconnect();
			if (WiFi.waitForConnectResult(WIFI_WAIT_TIME_MS) != WL_CONNECTED) {
				Serial.println("Unable to connect WiFi");
				return;
			}
		}

	int8_t rssiValue = WiFi.RSSI();

	mp.selectChannel(IN_TEMP_CH);
	float inTempVal = inTempSensor.readTemperature();

	mp.selectChannel(OUT_TEMP_CH);
	float outTempVal = outTempSensor.readTemperature();

	int chlorFlowVal = chlorFlowPulses / CHLOR_FLOW_PPL;
	int heatFlowVal = heatFlowPulses / HEAT_FLOW_PPL;

	chlorFlowPulses = 0;
	heatFlowPulses = 0;

	rssi.setValue(rssiValue);
	inTemp.setValue(inTempVal);
	outTemp.setValue(outTempVal);

	chlorFlow.setValue(chlorFlowVal);
	heatFlow.setValue(heatFlowVal);

	hvac.setCurrentTemperature(inTempVal);

	mp.selectChannel(DISP_CH);

	char str[10];
	display.setTextCursor(0, 32);
	display.write("In: ");
	sprintf( str, "%.1f", inTempVal );
	display.write( str );

	display.setTextCursor(0, 40);
	display.write("Out: ");
	sprintf( str, "%.1f", outTempVal );
	display.write( str );

	Serial.print("Set C: ");
	Serial.println();
	Serial.print("In C: ");
	Serial.println(inTempVal);
	Serial.print("Out C: ");
	Serial.println(outTempVal);
	Serial.print("Chlor flow: ");
	Serial.println(chlorFlowVal);
	Serial.print("Heat flow: ");
	Serial.println(heatFlowVal);
	Serial.println();

	}
	mqtt.loop();
}

# HA-Nightlight
NodeMCU with motion, LDR, temperature and humidity sensors published to MQTT. 

## Automation

[Home Assistant](https://www.home-assistant.io/) automation to turn on the light when motion is detected, if the sun is down and there is low light.

```yml
- alias: Ensuite LED auto ON motion
  initial_state: "on"
  trigger:
    - platform: state
      entity_id: binary_sensor.living_room_motion
      to: "on"
  condition:
    condition: or
    conditions:
      - condition: sun
        after: sunset
        after_offset: -00:10:00
      - condition: sun
        before: sunrise
        before_offset: -01:00:00
      - condition: and
        conditions:
          - condition: numeric_state
            entity_id: sensor.living_room_ldr
            below: 0.08
  action:
    service: light.turn_on
    entity_id: light.60412522bcddc251f567

- alias: Ensuite LED OFF after 1 min
  initial_state: "on"
  trigger:
    - platform: state
      entity_id: binary_sensor.living_room_motion
      to: "off"
      for: 00:01:00
  action:
    - service: homeassistant.turn_off
      entity_id: light.60412522bcddc251f567

- alias: Ensuite LED OFF when light
  initial_state: "on"
  trigger:
    platform: numeric_state
    entity_id: sensor.living_room_ldr
    above: 0.08
  action:
    - service: homeassistant.turn_off
      entity_id: light.60412522bcddc251f567
```

## Secrets

Create a secrets.h file in the same directory as `app.ino` and define the following secrets.

```c
#define wifi_ssid "" // Your WIFI SSD
#define wifi_password "" // Your WIFI Password
#define mqtt_server ""  // Your MQTT IP
#define mqtt_user "" // Your MQTT username
#define mqtt_pass "" // Your MQTT password
```
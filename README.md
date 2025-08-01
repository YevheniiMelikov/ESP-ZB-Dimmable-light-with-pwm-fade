# Zigbee CCT Dimmable Light (ESP32-C6)
This project is a custom Zigbee color-temperature (CCT) dimmable light firmware for Espressif silicon (e.g., ESP32-C6) based on the **esp-zigbee-sdk HA color dimmable light example**. It was improved with help from https://github.com/username-AL/ESP32H2-ZigbeeCCT, especially to resolve PWM-related issues.

## Key Features

- **CCT-only light**: Supports color temperature (mireds) and brightness; XY color is disabled.
- **Fade in / fade out** on on/off transitions using brightness.
- **Two-channel PWM**: Separate PWM outputs for warm and cold LEDs. Brightness and color temperature combine to drive both channels appropriately (blend between warm/cold based on temperature, scale with brightness).
- **Dedicated relay control pin**: Cuts power completely when off to eliminate ghosting/flickering at zero brightness (common with dimmable LEDs); relay is controlled before/after fades.
- **Zigbee HA-compatible**: Uses Espressif’s Zigbee stack with the HA Color Dimmable Light endpoint configured in CCT mode, including full support for `moveToColorTemperature` and reporting.
- **Zigbee2MQTT friendly**: Custom external converter support to integrate with Zigbee2MQTT installations.

## Hardware

- ESP32-C6 (or compatible with esp-zigbee-sdk)
- Warm and cold LED strings (or CCT fixture) driven via two PWM outputs.
- Relay on a dedicated GPIO to cut power to the LED fixture entirely when off.
- Zigbee radio (built into supported ESP32 chips) configured via esp-zigbee-sdk.

### GPIO Assignment (example)

```c
#define RELAY_GPIO        GPIO_NUM_19  // cuts power to light fixture
#define WARM_PWM_GPIO     GPIO_NUM_4  // warm LEDs PWM
#define COLD_PWM_GPIO     GPIO_NUM_5  // cold LEDs PWM
```

## Firmware Behavior

- `moveToColorTemperature` commands from Zigbee (e.g., via Zigbee2MQTT) adjust the blend between warm and cold LEDs by computing a cold/warm ratio from the desired mireds, then scaling by brightness.
- On/off uses a fade: when turning on, relay is asserted then PWM fades to the target level; when turning off, PWM fades to zero, then relay is de-asserted to remove residual power and prevent flicker.
- Reporting (on/off, brightness, color temperature and device measured temperature) is supported via the standard HA endpoint.

## Building

1. Install ESP-IDF (version matching the project; e.g., v5.x) and set up the environment per Espressif instructions.
2. Fetch and configure the `esp-zigbee-sdk` as a component (the example is using some of common helpers from `HA_color_dimmable_light` example).
3. Update CMakeLists path to point to your installation of `esp-zigbee-sdk` (`esp-zigbee-sdk/example/common` folder).
4. Build:

```sh
idf.py build
```

5. Flash to device:

```sh
idf.py -p <PORT> erase-flash
idf.py -p <PORT> flash
```

## Zigbee2MQTT Integration

To have Zigbee2MQTT recognize and handle the device properly, add an external converter for your custom model. Example converter (CommonJS style):

```js
const {
  light,
  temperature,
  forcePowerSource,
} = require("zigbee-herdsman-converters/lib/modernExtend");
const reporting = require("zigbee-herdsman-converters/lib/reporting");

/** @type {import('zigbee-herdsman-converters/lib/types').DefinitionWithExtend[]} */
module.exports = [
  {
    zigbeeModel: ["esp32c6_cct_light"],
    manufacturerName: "ESPRESSIF",
    model: "esp32c6_cct_light",
    vendor: "Espressif",
    description: "Custom ESP32-C6 CCT dimmable light (warm + cold)",

    extend: [
      light({
        colorTemp: {
          startup: false,
          range: [150, 500],
        },
        effect: false,
        powerOnBehavior: false,
      }),
      temperature(),
      forcePowerSource({ powerSource: "Mains (single phase)" }),
    ],

    endpoint: (device) => ({ default: 10 }),
    meta: { disableDefaultResponse: true },

    configure: async (device, coordinatorEndpoint, logger) => {
      const ep = device.getEndpoint(10);

      await reporting.bind(ep, coordinatorEndpoint, [
        "genOnOff",
        "genLevelCtrl",
        "lightingColorCtrl",
        "msTemperatureMeasurement",
      ]);

      await reporting.onOff(ep, {
        min: 0,
        max: 3600,
        change: 1,
      });

      await reporting.brightness(ep, {
        min: 0,
        max: 3600,
        change: 1,
      });

      await reporting.colorTemperature(ep, {
        min: 0,
        max: 3600,
        change: 1,
      });

      await reporting.temperature(ep, {
        min: 0,
        max: 10,
        change: 1,
      });
    },
  },
];
```

Ensure you’ve removed any conflicting YAML definitions (Zigbee2MQTT external converter files should not be double-defined in `.yaml` and JS simultaneously).
Converter needs to be added to `/config/zigbee2mqtt/external_converters/`. In that folder add `esp32c6_cct_light.js` with contents above and restart zigbee2mqtt. Then pair your flashed device.
From my experience it is better to do in this order:

- erase-flash on your device
- stop mqtt broker, wait till stopped
- stop z2m, wait till stopped
- start mqtt broker
- start z2m
- enable device pairing
- flash your device and it will connect as soon as it started

Check that is shows that device is supported and there are no errors appearing when device joins in z2m web interface. If there are errors check logs in `config/zigbee2mqtt/logs/`.

## Acknowledgments

- Based on the **esp-zigbee-sdk HA color dimmable light example** from Espressif.
- Thanks to [username-AL/ESP32H2-ZigbeeCCT](https://github.com/username-AL/ESP32H2-ZigbeeCCT) for invaluable help fixing PWM blending and fade issues.

## License

This project is released under the **MIT License**. See `LICENSE` for details.

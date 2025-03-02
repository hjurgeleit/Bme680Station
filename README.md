# ESP32 BME680 I2C Example

## Bosch BME68x SensorAPI Integration

This project requires the Bosch BME68x SensorAPI, which is not included in the repository.  
Please download it manually from [BME68x SensorAPI](https://github.com/boschsensortec/BME68x_SensorAPI/tree/v4.4.8)  
and place the files in the `components/bme68x_sensor_api` directory.

**Required Files:**  
- `bme68x.c`  
- `bme68x.h`  
- `bme68x_defs.h`  

## Prerequisites

- ESP-IDF v5.4
- BME68x_SensorAPI v4.4.8

## Build and Flash

```bash
idf.py build
idf.py flash
idf.py monitor
```

## License

This project is licensed under the MIT License.  
See the [LICENSE](./LICENSE) file for more information.

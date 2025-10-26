# Proyecto ESP32 - Energy Monitor (ESP-IDF)

## Requisitos previos
- ESP-IDF v5.4 o superior
- Visual Studio Code con extensión oficial de Espressif
- Python 3.11
- Git y CMake

## Cómo compilar
```bash
idf.py set-target esp32
idf.py build
idf.py -p <puerto> flash monitor

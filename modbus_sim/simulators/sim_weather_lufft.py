"""
A simulator for weather information from OpenWeatherMap.org that feeds into Modbus slave registers to be polled

Configuration of Modbus is done using the UMB Config Tool.  Default is 19200-8-E-1.  Fastest poll interval 1s.
Measurement to Modbus register mapping for Lufft AWS:

   * **Air Temperature and Humidity** also handles dewpoint, absolute humidity, mixing ratio

      * Air Temperature -50C to 60C in channels 100(act), 160(avg)
      * Dewpoint Temperature -50C to 60C in channels 110 (actual), 170 (average)
      * Wind Chill Temperature -60C to 70C in channel 111 (actual)
      * Relative Humidity %RH channel 200(act):10, 260(avg):13

   * **Air Pressure** absolute measured, relative is calculated based on user-configurable local altitude

      * Air Pressure absolute 300hPa to 1200hPa in channels 300 (actual), 360 (average)

   * **Precipitation** radar based (24GHz Doppler) for drop size and speed

      * Quantity Absolute (since last reset) mm ch620
      * Quantity Differential (since last read) mm ch625
      * Intensity 0-200.0 mm/h ch820
      * Type ch700 enumerated as:

         * 0 = None
         * 60 = Liquid/rain
         * 67 = Freezing rain (WS100-UMB only)
         * 69 = Sleet (WS100-UMB only)
         * 70 = Solid/snow
         * 90 = Hail (WS100-UMB only)

   * **Wet Bulb Temperature**
   * **Specific Enthalpy**
   * **Air Density** calculated from air temperature, humidity and air pressure
   * **Wind** uses 4 ultrasonic sensors for both wind speed and direction

      * Wind Speed 0-270km/h ch405 (act), ch460 (avg)
      * Wind Direction 0-359.9 degrees ch500 (act)

   * **Compass** used to calibrate ultrasonic wind direction, also used for compass-corrected wind direction
   * **Heating** is used in winter for precipitation sensor and wind meter
   * **Global Radiation** pyranometer mounted on top cover

      * radiation 0-2000.0 W/m^2 ch900(act), ch960(avg)

   * **Lightning** based on analysing radio wave emission of lightnings

      * Events 0-255 ch617(act)

   * **External Temperature** (optional)
   * **External Rain Gauge** (optional)

Configuration Registers:

   * **Local Altitude** hr 0, scale 1.0
   * **Averaging Interval TFF** hr 2, scale 1.0
   * **Averaging Interval Air Pressure** hr 3, scale 1.0
   * **Averaging Interval Wind** hr 4, scale 1.0
   * **Averaging Interval Global Radiation** hr 5, scale 1.0
   * **Reset absolute rain** hr 7
   * **Device reset** hr 8

"""

import requests
import json
import time
import headless

'''
Example calls:
api.openweathermap.org/data/2.5/weather?q={city name}[,{country code}][&units=metric][&callback={callback}]&APPID={key}
api.openweathermap.org/data/2.5/weather?lat=45.285352&lon=-75.848489&units=metric&APPID={key}

Max refresh rate is 10 minutes
'''

SIM_ID = ("Lufft", "WS501")
_logger = headless.get_wrapping_logger(name=__name__, debug=True)

DEFAULT_LOCATION = "lat={}&lon={}".format(45.285352, -75.848489)
DEFAULT_CITY = "Kanata,CA"
DEFAULT_KEY = "ea59750a8c2eba42d0cb26b9bf68b68d"
_USE_LIVE_API = True
MIN_REFRESH = 660 if _USE_LIVE_API else 10

sample_weather_resp = {
    "coord": {"lon": -75.9, "lat": 45.32},
    "weather": [{"id": 803, "main": "Clouds", "description": "broken clouds", "icon": "04n"}],
    "base": "stations",
    "main": {
        "temp": -16.0,          # default Kelvin, options metric=Celsius, imperial=Fahrenheit
        "pressure": 1023,       # hPa, at sea level, if no sea_level or grnd_level parameters
        "humidity": 59,         # %RH
        "temp_min": -16.0,      # same units as "temp"
        "temp_max": -16.0,      # same units as "temp"
        # "sea_level": 0,         # (optional) hPa
        # "grnd_level": 0         # (optional) hPa
    },
    "visibility": 24140,
    "wind": {
        "speed": 3.6,           # default m/s
        "deg": 60               # direction (meteorological)
    },
    "clouds": {"all": 75},      # cloudiness %
    # "rain": {                   # (optional)
    #     "1h": 0,                # rain volume last hour, mm
    #     "3h": 0                 # rain volume last 3 hours, mm
    # },
    # "snow": {                   # (optional)
    #     "1h": 0,                # snow volume last hour, mm
    #     "3h": 0                 # snow volume last 3 hours, mm
    # },
    "dt": 1547776800,           # time of day unix, UTC
    "sys": {
        "reg_type": 1,
        "id": 872,
        "message": 0.0032,
        "country": "CA",
        "sunrise": 1547815046,
        "sunset": 1547848261
    },
    "id": 6094817,              # City ID
    "name": "Kanata",           # City Name
    "cod": 200                  # internal parameter (unused)
}

sample_uv_resp = {
    "lat": 45.32,
    "lon": -75.9,
    "date_iso": "2018-01-17T02:00:00Z",
    "date": 1547776800,
    "value": 10.06
}

MODBUS_REGISTERS = [
    {"name": "Identification", "sparse": {0: 0}, "register_type": "ir", "factor": None, "enc": "uint16"},
    {"name": "Sensor Status 1", "sparse": {2: 0}, "register_type": "ir", "factor": None, "enc": "uint16"},
    {"name": "Sensor Status 2", "sparse": {3: 0}, "register_type": "ir", "factor": None, "enc": "uint16"},
    {"name": "Sensor Status 3", "sparse": {4: 0}, "register_type": "ir", "factor": None, "enc": "uint16"},
    {"name": "Sensor Status 4", "sparse": {5: 0}, "register_type": "ir", "factor": None, "enc": "uint16"},
    {"name": "Sensor Status 5", "sparse": {6: 0}, "register_type": "ir", "factor": None, "enc": "uint16"},
    {"name": "Sensor Status 6", "sparse": {7: 0}, "register_type": "ir", "factor": None, "enc": "uint16"},
    {"name": "Relative Humidity (act)", "sparse": {10: 0}, "register_type": "ir", "factor": 10, "enc": "int16"},
    {"name": "Relative Air Pressure (act)", "sparse": {14: 0}, "register_type": "ir", "factor": 10, "enc": "int16"},
    {"name": "Wind Direction (act)", "sparse": {18: 0}, "register_type": "ir", "factor": 10, "enc": "int16"},
    {"name": "Precipitation Type", "sparse": {25: 0}, "register_type": "ir", "factor": 1, "enc": "int16"},
    {"name": "Global Radiation (act)", "sparse": {27: 0}, "register_type": "ir", "factor": 10, "enc": "int16"},
    {"name": "Air Temperature C (act)", "sparse": {31: 0}, "register_type": "ir", "factor": 10, "enc": "int16"},
    {"name": "Dew Point C (act)", "sparse": {35: 0}, "register_type": "ir", "factor": 10, "enc": "int16"},
    # {"name": "Wind Chill C", "sparse": {39: 0}, "register_type": "ir", "factor": 10, "enc": "int16"},
    # {"name": "Wind Speed m/s (act)", "sparse": {42: 0}, "register_type": "ir", "factor": 10, "enc": "int16"},
    {"name": "Precipitation abs mm", "sparse": {48: 0}, "register_type": "ir", "factor": 100, "enc": "uint16"},
    # {"name": "Precipitation diff mm", "sparse": {49: 0}, "register_type": "ir", "factor": 100, "enc": "uint16"},
    {"name": "Precipitation Intensity mm/h", "sparse": {50: 0}, "register_type": "ir", "factor": 100, "enc": "uint16"},
    # {"name": "Wind Speed kph (act)", "sparse": {83: 0}, "register_type": "ir", "factor": 100, "enc": "uint16"},
    {"name": "Wind Speed kph (max)", "sparse": {85: 0}, "register_type": "ir", "factor": 100, "enc": "uint16"},
    {"name": "Wind Speed kph (avg)", "sparse": {86: 0}, "register_type": "ir", "factor": 100, "enc": "uint16"},
    {"name": "Local Altitude", "sparse": {0: 0}, "register_type": "hr", "factor": 1, "enc": "uint16"},
    {"name": "Averaging Interval TFF", "sparse": {2: 1}, "register_type": "hr", "factor": 1, "enc": "uint16"},
    {"name": "Averaging Interval Air Pressure", "sparse": {3: 1}, "register_type": "hr", "factor": 1, "enc": "uint16"},
    {"name": "Averaging Interval Wind", "sparse": {4: 1}, "register_type": "hr", "factor": 1, "enc": "uint16"},
    {"name": "Averaging Interval Radiation", "sparse": {5: 1}, "register_type": "hr", "factor": 1, "enc": "uint16"},
    {"name": "Reset Abs. Rain", "sparse": {7: 0}, "register_type": "hr", "factor": 1, "enc": "uint16"},
    {"name": "Device Reset", "sparse": {8: 0}, "register_type": "hr", "factor": 1, "enc": "uint16"}
]


def get_weatherstation_id(station_model="WS501-UMB", software_version=1):
    """
    Identification in Modbus register ir 0 (30000).  High byte: WS-Type, Low byte: Software version

    :param str station_model: the name of the weather station model being simulated
    :param int software_version: the version being simulated
    :return: the value of input register 0
    :rtype: int
    """
    smart_weather_sensor_type_codes = {
        "WS100-UMB": 1,
        "WS200-UMB": 2,
        "WS300-UMB": 3,
        "WS400-UMB": 4,
        "WS500-UMB": 5,
        "WS600-UMB": 6,
        "WS700-UMB": 7,
        "WS800-UMB": 8,
        "WS301-UMB": 13,
        "WS302-UMB": 23,
        "WS303-UMB": 33,
        "WS304-UMB": 43,
        "WS310-UMB": 93,
        "WS501-UMB": 15,
        "WS502-UMB": 25,
        "WS503-UMB": 35,
        "WS504-UMB": 45,
        "WS510-UMB": 95,
        "WS401-UMB": 14,
        "WS601-UMB": 16,
    }
    high_byte = 0
    for s in smart_weather_sensor_type_codes:
        if s == station_model:
            high_byte = smart_weather_sensor_type_codes[s]
            break
    low_byte = int(software_version) if int(software_version) in range(0, 2**8) else 0
    register_value = high_byte << 8 | low_byte
    return register_value


def get_weather(location, key, units="metric"):
    """
    Fetches weather and UV information from a subscription to OpenWeatherMap.org

    :param str location: format lat=Y&lon=X or city_name,country_code
    :param str key: the subscription key
    :param str units: from a selection of [metric, imperial] or using default (Kelvin/metric)
    :return: A dictionary with the following parameters:

       * ``temp_c`` (float) air temperature, Celsius
       * ``temp_f`` (float) air temperature, Fahrenheit
       * ``airpress_hpa`` (float) air pressure, hPa
       * ``rh_pct`` (int) Relative Humidity %
       * ``windspeed_kph`` (float) wind speed, km/h
       * ``windspeed_mph`` (float) wind speed, mph
       * ``winddirection`` (float) wind direction, degrees meteorological
       * ``precip_intensity`` (float) precipitation intensity, mm/h
       * ``precip_type`` (int) 60=rain, 70=snow/solid
       * ``global_radiation`` (float) derived from UV rating, W/m**2
    """
    TEMPERATURE_PRECISION = 1
    WINDSPEED_PRECISION = 1
    weather_data = None
    uv_data = None
    if _USE_LIVE_API:
        if units in ["metric", "imperial", "default"]:
            queries_ok = True
            weather_query = requests.get("http://api.openweathermap.org/data/2.5/"
                                         "weather?{location}{units}&APPID={appid}"
                                         .format(location=location, units="&units={}".format(units), appid=key),
                                         timeout=5)
            # TODO: handle failed queries (e.g. too many requests)
            if weather_query.status_code == requests.codes.ok:
                weather_data = json.loads(weather_query.content)
            else:
                _logger.error("Bad response from server: {}".format(weather_query.status_code))
                queries_ok = False
            uv_query = requests.get("http://api.openweathermap.org/data/2.5/uvi?{location}&appid={appid}"
                                    .format(location=location, appid=key), timeout=5)
            if uv_query.status_code == requests.codes.ok:
                uv_data = json.loads(uv_query.content)
            else:
                _logger.error("Bad response from server: {}".format(uv_query.status_code))
                queries_ok = False
        else:
            raise ValueError("Invalid units reg_type {} - must be metric, imperial or default".format(units))
    else:
        units = "metric"
        weather_data = sample_weather_resp
        uv_data = sample_uv_resp
        queries_ok = True
    weather = {}
    if queries_ok:
        for tag in weather_data:
            if tag == "main":
                for k in weather_data[tag]:
                    if k == "temp":
                        if units == "metric":
                            weather['temp_c'] = round(float(weather_data[tag][k]), TEMPERATURE_PRECISION)
                            weather['temp_f'] = round(weather['temp_c'] * 9.0/5.0 + 32.0, TEMPERATURE_PRECISION)
                        elif units == "imperial":
                            weather['temp_f'] = round(float(weather_data[tag][k]), TEMPERATURE_PRECISION)
                            weather['temp_c'] = round((weather['temp_f'] - 32.0) * 5.0/9.0, TEMPERATURE_PRECISION)
                        else:
                            weather['temp_c'] = round(weather_data[tag][k] - 273.15, TEMPERATURE_PRECISION)
                            weather['temp_f'] = round(weather['temp_c'] * 9.0/5.0 + 32.0, TEMPERATURE_PRECISION)
                    elif k == "pressure":
                        weather['airpress_hpa'] = float(weather_data[tag][k])
                    elif k == "humidity":
                        weather['rh_pct'] = int(weather_data[tag][k])
            elif tag == "wind":
                for k in weather_data[tag]:
                    if k == "speed":
                        weather['windspeed_kph'] = round(float(weather_data[tag][k]) * 3.6, WINDSPEED_PRECISION)
                        weather['windspeed_mph'] = round(float(weather_data[tag][k]) * 2.23694, WINDSPEED_PRECISION)
                    elif k == "deg":
                        weather['winddirection'] = float(weather_data[tag][k])
            elif tag in ["rain", "snow"]:
                for k in weather_data[tag]:
                    if k == "1h":
                        weather['precip_intensity'] = float(weather_data[tag][k])
                weather['precip_type'] = 60 if tag == "rain" else 70
        # http://strang.smhi.se/extraction/units-conversion.html
        weather['global_radiation'] = round(float(uv_data["value"]) / 40.0, 2)
        # print("Weather: {}".format(weather))
    return weather


def set_value(name, value):
    """
    Sets the abstracted value of a parameter (pre-factored, not necessarily the Modbus register value)

    :param str name: the parameter name of the Modbus register e.g. "Relative Humidity (act)"
    :param value: the value, a number could be int or float or raw data
    """
    for reg in MODBUS_REGISTERS:
        if reg['name'] == name:
            value = value * reg['factor'] if isinstance(reg['factor'], int) else value
            for addr in reg['sparse']:
                write_register(reg_type=reg['register_type'], address=addr, value=value)
            break
    if name in ["Reset Abs. Rain", "Device Reset"]:
        set_value("Precipitation abs mm", 0)


def write_register(reg_type, address, value):
    """
    Writes the raw register value in Modbus

    :param str reg_type: register type from the list ['hr', 'ir', 'di', 'co']
    :param int address: the native Modbus register address
    :param value: the value int or float or binary data
    """
    for reg in MODBUS_REGISTERS:
        if reg['register_type'] == reg_type and address in reg['sparse']:
            if "int" in reg['enc']:
                value = int(value)
            elif "float" in reg['enc']:
                value = float(value)
            reg['sparse'][address] = value
            break


def read_register(reg_type, address):
    """
    Reads the raw register value from Modbus

    :param str reg_type: register type from the list ['hr', 'ir', 'di', 'co']
    :param int address: the native Modbus register address
    :return: the register value
    :rtype: int or float or blob
    """
    value = 0
    for reg in MODBUS_REGISTERS:
        if reg['register_type'] == reg_type and address in reg['sparse']:
            value = reg['sparse'][address]
            break
    return value


def get_value(name):
    """
    Gets the converted parameter value in expected units.

    :param str name: the parameter name of the Modbus register e.g. "Relative Humidity (act)"
    :return: the data value
    :rtype: float or int depending on value type
    """
    value = 0
    for reg in MODBUS_REGISTERS:
        if reg['name'] == name:
            for addr in reg['sparse']:
                value = read_register(reg_type=reg['register_type'], address=addr)
            if isinstance(reg['factor'], int) and reg['factor'] != 1:
                value = float(value / reg['factor'])
            break
    return value


def simulate(location=DEFAULT_LOCATION, key=DEFAULT_KEY, log=_logger, refresh=MIN_REFRESH):
    """
    Starts a loop periodically updating weather data into Modbus registers

    :param str location: format lat=Y&lon=X or city_name,country_code
    :param str key: the OpenWeatherMap.org subscription key
    :param logging.Log log: (optional) logger to store debug messages
    :param int refresh: the refresh interval for weather data, in seconds (default 660)
    """
    if refresh < MIN_REFRESH:
        refresh = MIN_REFRESH
    time_ref = time.time()
    log.debug("Simulating with {interval} {units} refresh"
              .format(interval=refresh if refresh < 60 else int(refresh/60),
                      units='seconds' if refresh < 60 else 'minutes'))
    is_running = True
    first_run = True
    try:
        while True:
            if time.time() - time_ref >= refresh or first_run:
                if first_run:
                    log.debug("Initial query running via {}".format('Internet' if _USE_LIVE_API else 'static data'))
                    first_run = False
                else:
                    log.debug("Refreshing weather data via {}".format('Internet' if _USE_LIVE_API else 'static data'))
                time_ref = time.time()
                weather = get_weather(location, key)
                if len(weather) > 0:
                    set_value("Identification", get_weatherstation_id())
                    # TODO: set values / simulate sensor faults periodically in Sensor Status N
                    for tag in weather:
                        if tag == "temp_c":
                            set_value("Air Temperature C (act)", weather[tag])
                        elif tag == "temp_f":
                            set_value("Air Temperature F (act)", weather[tag])
                        elif tag == "rh_pct":
                            set_value("Relative Humidity (act)", weather[tag])
                        elif tag == "windspeed_kph":
                            set_value("Wind Speed kph (avg)", weather[tag])
                            set_value("Wind Speed kph (max)", weather[tag])
                        elif tag == "winddirection":
                            set_value("Wind Direction (act)", weather[tag])
                        elif tag == "airpress_hpa":
                            set_value("Relative Air Pressure (act)", weather[tag])
                        elif tag == "precip_intensity":
                            set_value("Precipitation Intensity mm/h", weather[tag])
                        elif tag == "global_radiation":
                            set_value("Global Radiation (act)", weather[tag])
                    for reg in MODBUS_REGISTERS:
                        if reg['name'] == "Dew Point C (act)":
                            # Tdp = T - (100 - RH)/5  ==> https://en.wikipedia.org/wiki/Dew_point (Simple approximation)
                            t = get_value("Air Temperature C (act)")
                            rh = get_value("Relative Humidity (act)")
                            set_value("Dew Point C (act)", int(t - (100 - rh) / 5))
                        elif reg['name'] == "Precipitation abs mm":
                            prev = get_value("Precipitation abs mm")
                            new = get_value("Precipitation Intensity mm/h")
                            set_value("Precipitation abs mm", prev + new)
                    # log.debug("Updating weather simulation {}".format(MODBUS_REGISTERS))
                else:
                    log.warning("No weather data returned by API call")
            else:
                if (time.time() - time_ref) % 60 == 0:
                    remaining_time = int((refresh - (time.time() - time_ref)) / 60)
                    log.debug("Waiting {} minutes for refresh".format(remaining_time))
                time.sleep(1)
    except Exception, e:
        log.error("Exception: {}".format(e))
        if "HTTPConnectionPool" in str(e):
            simulate()
        raise ValueError(e)
    finally:
        is_running = False


if __name__ == "__main__":
    simulate()

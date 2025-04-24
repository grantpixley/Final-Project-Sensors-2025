from machine import ADC, Pin, I2C, SPI
from utime import sleep, localtime
import math
import os
import sdcard
from dht20 import DHT20

# === Constants ===
VREF = 3.3
NUM_SAMPLES = 30
LOG_INTERVAL_SEC = 60
LOG_FILE = "/sd/sensor_log.csv"

# === SPI and SD Setup (unchanged) ===
spi = SPI(1, sck=Pin(14), mosi=Pin(15), miso=Pin(12))
cs = Pin(17, Pin.OUT)
sd = sdcard.SDCard(spi, cs)
vfs = os.VfsFat(sd)
os.mount(vfs, "/sd")

# === Write Header if File Doesn't Exist ===
try:
    with open(LOG_FILE, "x") as f:
        f.write("Time,Temperature_F,Humidity_RH,UV_Index\n")
except OSError:
    pass  # File exists

# === Sensor Setup ===
thermistor = ADC(26)      # Thermistor on GP26
uv_sensor = ADC(27)       # UV sensor on GP27
i2c0 = I2C(0, scl=Pin(9), sda=Pin(8))  # DHT20 I2C
dht20 = DHT20(0x38, i2c0)

# === Kalman Filter Class ===
class KalmanFilter:
    def __init__(self, process_variance, measurement_variance, estimated_error=1.0, initial_value=75.0):
        self.Q = process_variance
        self.R = measurement_variance
        self.P = estimated_error
        self.X = initial_value

    def update(self, measurement):
        self.P += self.Q
        K = self.P / (self.P + self.R)
        self.X += K * (measurement - self.X)
        self.P *= (1 - K)
        return self.X

# === Initialize Kalman Filter for Thermistor ===
kalman_temp = KalmanFilter(process_variance=0.1, measurement_variance=2.0, estimated_error=1.0, initial_value=75.0)

# === Main Loop ===
while True:
    # --- Thermistor Averaging ---
    valid_temp_sum = 0
    valid_temp_count = 0
    for _ in range(NUM_SAMPLES):
        raw = thermistor.read_u16()
        voltage = (raw / 65535) * VREF
        if voltage > 0:
            R_T = (250 * (-2 * voltage + 5)) / voltage
            if R_T > 0:
                try:
                    temp_K = 1084073.4 / (3636 + 298.15 * math.log(0.001 * R_T))
                    temp_C = temp_K - 273.15
                    temp_F = temp_C * 9/5 + 32
                    valid_temp_sum += temp_F
                    valid_temp_count += 1
                except:
                    pass
    if valid_temp_count > 0:
        avg_temp_F_raw = valid_temp_sum / valid_temp_count
        avg_temp_F = round(kalman_temp.update(avg_temp_F_raw))
    else:
        avg_temp_F = "undefined"

    # --- Humidity Averaging ---
    humidity_sum = 0
    for _ in range(NUM_SAMPLES):
        humidity_sum += dht20.measurements['rh']
    avg_humidity = round(humidity_sum / NUM_SAMPLES)

    # --- UV Index Averaging ---
    uv_sum = 0
    for _ in range(NUM_SAMPLES):
        raw = uv_sensor.read_u16()
        voltage = (raw / 65535) * VREF
        uv_index = voltage * 1000
        uv_sum += uv_index
    avg_uv_index = round(uv_sum / NUM_SAMPLES)

    # --- Timestamp ---
    t = localtime()
    timestamp = f"{t[0]:04d}-{t[1]:02d}-{t[2]:02d} {t[3]:02d}:{t[4]:02d}:{t[5]:02d}"

    # --- Print to Console ---
    print(f"{timestamp} | Temp: {avg_temp_F} °F | Humidity: {avg_humidity} %RH | UV: {avg_uv_index}")

    # --- Save to SD File ---
    with open(LOG_FILE, "a") as f:
        f.write(f"{timestamp},{avg_temp_F},{avg_humidity},{avg_uv_index}\n")

    sleep(LOG_INTERVAL_SEC)

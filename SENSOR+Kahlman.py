from machine import ADC, Pin, I2C, SPI
from utime import sleep, localtime
import math
import os
import sdcard
from dht20 import DHT20

# Constants
VREF = 3.3                  # ADC reference voltage
NUM_SAMPLES = 30            # Number of samples to average
LOG_INTERVAL_SEC = 60       # Time between logs (in seconds)
LOG_FILE = "/sd/sensor_log.csv"  # Path to log file on SD card

# Set up SPI and SD card
spi = SPI(1, sck=Pin(14), mosi=Pin(15), miso=Pin(12))  # SPI bus 1
cs = Pin(17, Pin.OUT)                                  # Chip select for SD card
sd = sdcard.SDCard(spi, cs)                            # Initialize SD card
vfs = os.VfsFat(sd)                                    # Mount filesystem
os.mount(vfs, "/sd")                                   # Mount at /sd

# Create log file with header if it doesn't already exist
try:
    with open(LOG_FILE, "x") as f:
        f.write("Time,Temperature_F,Humidity_RH,UV_Index\n")
except OSError:
    pass  # File already exists

# Set up sensors
thermistor = ADC(26)           # Thermistor on ADC pin GP26
uv_sensor = ADC(27)            # UV sensor on ADC pin GP27
i2c0 = I2C(0, scl=Pin(9), sda=Pin(8))  # I2C0 for DHT20
dht20 = DHT20(0x38, i2c0)      # Initialize DHT20 sensor

# Kalman filter for smoothing temperature readings
class KalmanFilter:
    def __init__(self, process_variance, measurement_variance, estimated_error=1.0, initial_value=75.0):
        self.Q = process_variance        # Process noise variance
        self.R = measurement_variance    # Measurement noise variance
        self.P = estimated_error         # Estimated error
        self.X = initial_value           # Filtered value (estimate)

    def update(self, measurement):
        self.P += self.Q
        K = self.P / (self.P + self.R)   # Kalman gain
        self.X += K * (measurement - self.X)  # Update estimate
        self.P *= (1 - K)                # Update error estimate
        return self.X

# Initialize Kalman filter for thermistor readings
kalman_temp = KalmanFilter(process_variance=0.1, measurement_variance=2.0, estimated_error=1.0, initial_value=75.0)

# Main loop
while True:
    #Thermistor temperature reading and averaging
    valid_temp_sum = 0
    valid_temp_count = 0
    for _ in range(NUM_SAMPLES):
        raw = thermistor.read_u16()                      # Read raw ADC value
        voltage = (raw / 65535) * VREF                   # Convert to voltage
        if voltage > 0:                                  # Skip invalid readings
            R_T = (250 * (-2 * voltage + 5)) / voltage   # Calculate thermistor resistance
            if R_T > 0:
                try:
                    temp_K = 1084073.4 / (3636 + 298.15 * math.log(0.001 * R_T))  # Temperature in Kelvin
                    temp_C = temp_K - 273.15                                     # Convert to Celsius
                    temp_F = temp_C * 9/5 + 32                                   # Convert to Fahrenheit
                    valid_temp_sum += temp_F                                     # Accumulate valid readings
                    valid_temp_count += 1
                except:
                    pass  # Skip math errors (e.g., log domain errors)

    # Apply Kalman filter if there are valid temperature readings
    if valid_temp_count > 0:
        avg_temp_F_raw = valid_temp_sum / valid_temp_count
        avg_temp_F = round(kalman_temp.update(avg_temp_F_raw))
    else:
        avg_temp_F = "undefined"

    #Humidity reading and averaging from DHT20
    humidity_sum = 0
    for _ in range(NUM_SAMPLES):
        humidity_sum += dht20.measurements['rh']  # Read humidity percentage
    avg_humidity = round(humidity_sum / NUM_SAMPLES)

    #UV index reading and averaging 
    uv_sum = 0
    for _ in range(NUM_SAMPLES):
        raw = uv_sensor.read_u16()               # Read raw ADC value
        voltage = (raw / 65535) * VREF           # Convert to voltage
        uv_index = voltage * 1000                # Scale voltage to UV index (per datasheet)
        uv_sum += uv_index
    avg_uv_index = round(uv_sum / NUM_SAMPLES)

    #Timestamp
    t = localtime()
    timestamp = f"{t[0]:04d}-{t[1]:02d}-{t[2]:02d} {t[3]:02d}:{t[4]:02d}:{t[5]:02d}"

    #Print sensor data to console
    print(f"{timestamp} | Temp: {avg_temp_F} °F | Humidity: {avg_humidity} %RH | UV: {avg_uv_index}")

    #Save data to SD card
    with open(LOG_FILE, "a") as f:
        f.write(f"{timestamp},{avg_temp_F},{avg_humidity},{avg_uv_index}\n")

    #Wait before next reading
    sleep(LOG_INTERVAL_SEC)


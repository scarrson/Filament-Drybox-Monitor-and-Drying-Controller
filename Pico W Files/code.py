'r_1.0 - Initial working build Sep. 1, 2023'
'Tested on a Raspberry Pi Pico W running Circuitpython firmware 8.2.3'
'For monitoring a drybox, with functions for controlling a heater, crash recovery and MQTT + OLED reporting'

import time
import board
import busio
import adafruit_htu31d
import countio
import digitalio
import pwmio
import rotaryio
import displayio
import terminalio
from adafruit_display_text import label
import adafruit_displayio_ssd1306
import os
import wifi
import ssl
import socketpool
import adafruit_minimqtt.adafruit_minimqtt as MQTT
from microcontroller import watchdog
from watchdog import WatchDogMode
import math
from secrets import secrets

print('Import Complete')

# Delay to allow all sensors to boot properly before starting
time.sleep(2) 

# Set watchdog timer and feed watchdog
watchdog.timeout = 8 # Must be fed every n seconds 
watchdog.mode = WatchDogMode.RESET # Mode is RESET - if wd expire, reset board 
watchdog.feed()

# Release displays prior to running
displayio.release_displays()

# Setup SPI bus
spi = busio.SPI(clock=board.GP2, MISO=board.GP4, MOSI=board.GP3)

# Setup SSD1306 and connect via SPI
WIDTH = 128
HEIGHT = 64
BORDER = 5
oled_cs = board.GP5
oled_dc = board.GP6
oled_reset = board.GP7
display_bus = displayio.FourWire(spi, command=oled_dc, chip_select=oled_cs, reset=oled_reset)
display = adafruit_displayio_ssd1306.SSD1306(display_bus, width=WIDTH, height=HEIGHT)

# Setup I2C bus
i2c = busio.I2C(board.GP9, board.GP8, frequency=100000)

# Setup HTU31 and connect via I2C
htu31 = adafruit_htu31d.HTU31D(i2c)

# Setup counter input for countio (must be Channel B PWM GPIO), enable rising edge and pull-up
fan_counter = countio.Counter(board.GP11, edge=countio.Edge.RISE, pull=digitalio.Pull.UP)

# Setup rotary encoder
encoder_pb = digitalio.DigitalInOut(board.GP12)
encoder_pb.direction = digitalio.Direction.INPUT
encoder_pb.pull = digitalio.Pull.UP
encoder = rotaryio.IncrementalEncoder(board.GP13, board.GP14)
position = 0
encoder.position = position
last_position = position #Pre-load last_position

# Setup digital I/O
mode_toggle = digitalio.DigitalInOut(board.GP15) # Sink to ground via toggle switch.
mode_toggle.direction = digitalio.Direction.INPUT
mode_toggle.pull = digitalio.Pull.UP

mode_led = digitalio.DigitalInOut(board.GP0)  # Source +3.3V to transistor for heat mode active 12V LED
mode_led.direction = digitalio.Direction.OUTPUT

heater = digitalio.DigitalInOut(board.GP1)  # Source +3.3V to transistor for heater relay 5V coil circuit
heater.direction = digitalio.Direction.OUTPUT

# Setup SSD1306 group and text lines
main_group = displayio.Group()
display.show(main_group)
text = ""
rows_1 = label.Label(terminalio.FONT, text=text, color=0xFFFFFF, x=0, y=4)
rows_2 = label.Label(terminalio.FONT, text=text, color=0xFFFFFF, x=0, y=20)
rows_3 = label.Label(terminalio.FONT, text=text, color=0xFFFFFF, x=72, y=20)
main_group.append(rows_1)
main_group.append(rows_2)
main_group.append(rows_3)


# Setup wifi connection - SSID and Password in secrets.txt file, not here
print("Connecting to WiFi...")
wifi.radio.connect(secrets["ssid"], secrets["wifi_password"])
pool = socketpool.SocketPool(wifi.radio)

# Setup MQTT functions
def connected(client, userdata, flags, rc):
    # gets called when client is connected successfully to the broker
    print("Connected to MQTT")

def disconnected(client, userdata, rc):
    # gets called when client is disconnected
    print("Disconnected from MQTT")

# Setup MQTT Client
mqtt_client = MQTT.MQTT(
    broker=secrets["broker"],
    port=secrets["port"],
    username=secrets["username"],
    password=secrets["password"],
    socket_pool=pool,
    ssl_context=ssl.create_default_context(),
)

mqtt_client.on_connect = connected # Run connected on connect
mqtt_client.on_disconnect = disconnected # Run disconnected on disconnect

# Connect to MQTT
print("Connecting to MQTT...")
mqtt_client.connect()

# Setup average function for MQTT logging of analogs
def average(measurements, key):
    sum = 0
    for i in measurements:
        sum = sum + i[key]
    return sum / len(measurements)

# Setup maximum function for MQTT logging of booleana (favours a "1" at any point during sample period)
def maximum(measurements, key):
    max = 0
    for i in measurements:
        if i[key] > max:
            max = i[key]
    return max

# Setup measurements list for average & maximum functions
measurements = []  

# Setup dewpoint function to calculate based on sensor RH & Temp
def calculate_dew_point(temperature, relative_humidity):
    rh_decimal = relative_humidity / 100.0 # Convert relative humidity to decimal
    es = 6.112 * math.exp((17.67 * temperature) / (temperature + 243.5)) # Calculate saturation vapor pressure (Es)
    e = es * rh_decimal # Calculate actual vapor pressure (E)
    td = (243.5 * math.log(e / 6.112)) / (17.67 - math.log(e / 6.112)) # Calculate dew point temperature (Td)
    return td

# Setup program variables
htu31_rh = 0
htu31_temp = 0
htu31_dewpoint = 0
htu31_errors = 0
fan_rpm = 0
current_time = 0
prev_time = 0
elapsed_time = 0
mode = "Monitor"
heatingmode = False # False = Monitor, True = Heating (for MQTT reporting)
firstscan = True # Set to 1 to indicate RPi was recently booted
fault = False
num_edit = False
pb_state = False

'******************* USER-DEFINED VALUES & SETPOINTS *******************'

# DEFINE SETPOINTS
trip_temp_low = 10 # trip below this temp (sensor error)
trip_temp_high = 85 # trip above this temp (sensor error)
min_temp = 20 # min allowable setpoint
max_temp = 75 # max allowable setpoint
set_temp = 20 # default setpoint
deadband = 2 # deadband for heater operation
sleep_delay = 600 # Number of idle seconds before sleeping OLED


# DEFINE MQTT FEEDS
htu31_temp_feed = "home/filament-monitor-2/htu31-temp-degc"
htu31_rh_feed = "home/filament-monitor-2/htu31-rh-pct"
htu31_dewpoint_feed = "home/filament-monitor-2/htu31-dewpoint-degc"
fan_rpm_feed = "home/filament-monitor-2/fan-rpm"
firstscan_feed = "home/filament-monitor-2/firstscan"
heatingmode_feed = "home/filament-monitor-2/heatingmode"
heater_feed = "home/filament-monitor-2/heater"
fault_feed = "home/filament-monitor-2/fault"
htu31_errors_feed = "home/filament-monitor-2/htu31_errors"
set_temp_feed = "home/filament-monitor-2/set_temp"

'************** CHECK IF CRASHED WHILE HEATING AND LOAD OLD SETPOINT **************'

# Read stored values
filename = "/saved_setpoint.txt"

# If crashed while heating, resume with original saved setpoint else start with default.
if mode_toggle.value == False: #Booting with heat switch in heat mode, load last setpoint.
    try:
        with open(filename, "r") as f:
            set_temp = int(f.read())
            print('Read last setpoint as ' + str(set_temp))
    except OSError as e:
        print('Error reading stored file, rebooting...')
        microcontroller.reset()        
if mode_toggle.value == True: # Booting with heat switch in monitor mode, write setpoint to default just in case.
    try: 
        with open(filename, "w") as f:
            f.write(str(set_temp)) # Store setpoint in offline file
            print('Saved last setpoint as ' + str(set_temp))
    except OSError as e:
        print('Error reading stored file, rebooting...')
        microcontroller.reset()

'******************* RESET AND PRELOAD VALUES BEFORE STARTING *******************'

position = set_temp # Preload encoder
encoder.position = position

prev_time = time.monotonic() # Mark start time
prev_pubtime = time.monotonic()
prev_sleeptime = time.monotonic()
htu31_timer = time.monotonic()
fan_counter.reset() # Reset counter to zero before starting

print('Starting sampling cycle')

'******************************* START MAIN LOOP *******************************'

while True:
    # Feed watchdog, mark start of routine
    watchdog.feed()
    
    # Mode toggle
    if mode_toggle.value == True: # Monitor selected
        mode = "Monitor"
        heatingmode = False
        mode_led.value = False
    if mode_toggle.value == False: # Heating selected
        mode = "Heating"
        heatingmode = True
        mode_led.value = True
        prev_sleeptime = time.monotonic() # Always awake if heating, sleep timer never expires
    
    # HTU31 sensor grab
    try:
        htu31_temp = htu31.temperature
        htu31_rh = htu31.relative_humidity
        htu31_timer = time.monotonic()
    except Exception: # Typically I2C error due to cable length
        print("HTU31 Error, continuing...")
        htu31_errors += 1
    
    # Run dew point calculation
    if htu31_temp != 0 and htu31_rh != 0: # Function errors if inputs are zero, let sensors provide values first
        htu31_dewpoint = calculate_dew_point(htu31_temp, htu31_rh)

    # Manage fan tach counter and convert to RPM
    if elapsed_time >=1:
        fan_rpm = (fan_counter.count/elapsed_time) / 2 * 60 # 2 pulses per revolution, x 60 sec = RPM
        fan_counter.reset() # reset to 0
        
    # Encoder routine
    if num_edit == False:
        encoder.position = position # keep encoder at same position as previous edit until num_edit is true
    if mode == "Monitor": # Reset num_edit to False if turning off heater
        num_edit = False
    if mode == "Heating": # Only activate num changes if in heating mode
        if num_edit == True:
            position = encoder.position        
        if position >= max_temp: # upper limit
            position = max_temp
            encoder.position = max_temp
        if position <= min_temp: # lower limit
            position = min_temp
            encoder.position = min_temp
        if last_position == None or position != last_position:
            print("encoder pos = " + str(position))
        set_temp = position
        last_position = position
        if not encoder_pb.value and pb_state == False:
            pb_state = True
        if encoder_pb.value and pb_state == True:      
            pb_state = False
            num_edit = not num_edit # toggle num edit
            print("Button released, num_edit = " + str(num_edit))
            try: # Store setpoint in offline file in case of crash
                with open(filename, "w") as f:
                    f.write(str(set_temp))
                    print('Saved set temp as ' + str(set_temp))
            except OSError as e:
                print('Error writing file, rebooting...')
                microcontroller.reset()

    # OLED activity timer
    elapsed_sleeptime = current_time - prev_sleeptime
    if encoder_pb.value == False: # Press to wakeup
        prev_sleeptime = time.monotonic()
    if elapsed_sleeptime >= sleep_delay and display.is_awake == True and mode == "Monitor":
        display.sleep()
        print("Sleeping OLED Display")
    if (elapsed_sleeptime <= sleep_delay and display.is_awake == False and mode == "Monitor") or (display.is_awake == False and mode == "Heating"):
        display.wake()
        print("Waking OLED Display")
            
    # OLED routine
    num = int(time.monotonic()) # for blink on odd # secs if setpoint modify is active
    if fault == False:
        text = "Mode: " + mode
    if fault == True:
        text = "ACTIVE FAULT"
    rows_1.text = text
    text = "Temp: " + "{:.1f}".format(htu31_temp) + "\nRH %: " + "{:.1f}".format(htu31_rh) + "\nDewP: " + "{:.1f}".format(htu31_dewpoint)
    rows_2.text = text
    if mode == "Monitor": # Monitoring, blank this text portion
        text = ""
    elif mode == "Heating": # Heating mode, display setpoint & dband
        if num % 2 == 0 or num_edit == False:
            text = "SetP: " + str(set_temp) + "\nBand: " + str(deadband)
        elif num % 2 != 0 and num_edit == True:
            text = "SetP: " + "__" + "\nBand: " + str(deadband)
    rows_3.text = text
        
    # Fault detection
    if (htu31_temp >= trip_temp_high) or (htu31_temp <= trip_temp_low):
        fault = True
    if current_time - htu31_timer >= 10: # If HTU31 comms bad for > delay, fault
        fault = True
    else:
        fault = False
        
    # Heater control routine
    if mode == "Heating" and htu31_temp <= (set_temp - deadband/2) and fault == False and num_edit == False:
        heater.value = True
    if mode == "Heating" and htu31_temp >= (set_temp + deadband/2) and fault == False and num_edit == False:
        heater.value = False
    if mode == "Monitor" or fault == True:
        heater.value = False
    
    # Upload measurements to MQTT (try) or keep sampling values (else)
    current_time = time.monotonic()
    elapsed_time = current_time - prev_time
    elapsed_pubtime = current_time - prev_pubtime
    try:
        if (mode == "Monitor" and elapsed_pubtime >= 60) or (mode == "Heating" and elapsed_pubtime >= 5): # Faster publishing if heating, monitor is slower
            print("Sending data to MQTT")
            mqtt_client.publish(htu31_temp_feed, average(
                measurements, "htu31_temp"))
            mqtt_client.publish(htu31_rh_feed, average(
                measurements, "htu31_rh"))
            mqtt_client.publish(htu31_dewpoint_feed, average(
                measurements, "htu31_dewpoint"))
            mqtt_client.publish(fan_rpm_feed, average(
                measurements, "fan_rpm"))
            mqtt_client.publish(firstscan_feed, maximum(
                measurements, "firstscan"))
            mqtt_client.publish(heatingmode_feed, maximum(
                measurements, "heatingmode"))
            mqtt_client.publish(heater_feed, maximum(
                measurements, "heater"))
            mqtt_client.publish(fault_feed, maximum(
                measurements, "fault"))
            mqtt_client.publish(htu31_errors_feed, htu31_errors)
            if heatingmode == True: # Only publish setpoint if in heating mode
                mqtt_client.publish(set_temp_feed, set_temp)
                
            firstscan = False # Clear flag after reporting that the pi rebooted
            measurements = []  # Reset list to 0
            prev_pubtime = time.monotonic() # Mark last published time
            htu31_errors = 0 # Reset counter
            
        elif elapsed_time >= 1:  # If not publishing, append a measurement every n seconds
            measurements.append(
                {
                    "htu31_temp": htu31_temp,
                    "htu31_rh": htu31_rh,
                    "htu31_dewpoint": htu31_dewpoint,
                    "fan_rpm": fan_rpm,
                    "firstscan": int(firstscan),
                    "heatingmode": int(heatingmode),
                    "heater": int(heater.value),
                    "fault": int(fault),
                }
            )
            prev_time = time.monotonic()
    except RuntimeError as e:
        print("Reading error: ", e.args)

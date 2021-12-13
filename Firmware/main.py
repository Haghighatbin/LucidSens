"""
LucidSens(2021)

This firmware has been written for the LucidSens device based on Micropython (Loboris port ESP32_LoBo_v3.2.24).

Licensed under XXX:

Firmware: https://github.com/haghighatbin/LucidSens_Firmware.git
GUI: https://github.com/haghighatbin/LucidSens_GUI.git

aminhb@tutanota.com
"""
# pylint: disable=no-name-in-module
# pylint: disable=no-member
# pylint: disable=eval-used

import sys, os, _thread, gc
from array import array
from display import TFT
from network import WLAN, STA_IF
from machine import Pin, ADC, DAC, PWM, RTC, DHT, stdin_get, stdout_put
from utime import sleep, strftime, localtime
import usocket as socket
import ujson as json
# from upysh import *

# Color Codes
GREEN = '\033[92m'
RED = '\033[91m'
CYAN = '\033[96m'
PINK  = '\033[95m'
YELLOW = '\033[93m'
WHITE = '\033[97m'

# ST7735_INVOFF = const(0x20)
# ST7735_INVON = const(0x21)

gc.enable()
print(GREEN + 'THREAD stack_size is: {}'.format(_thread.stack_size(6*4096)))
for p in [5, 13, 14, 15, 21]:
    Pin(p, Pin.OUT, value=0)
print('pins 5, 13, 14, 15, 21 were pulled down.' + WHITE)

class kill:
    def __repr__(self):
        return _thread.notify(0, _thread.EXIT)
    def __call__(self):
        return self.__repr__()

class Clear:
    def __repr__(self):
        return "\x1b[2J\x1b[H"
    def __call__(self):
        return self.__repr__()

class TemperatureSensor:
    # pylint: disable=too-few-public-methods
    def __init__(self):
        self.dht = DHT(Pin(27), DHT.DHT2X)
        self.temperature = str(25) # some random initial values, in case the sensor's disconnected
        self.humidity = str(55)

    def read(self):
        sensor = self.dht.read()
        if sensor[0]:
            self.temperature, self.humidity = sensor[1], sensor[2]
            return [self.temperature, self.humidity]
        else:
            return False

class CommandHandler:
    """Handles the commands received from the GUI and calls the appropriate modules."""
    def __init__(self):
        self.clear = Clear()

    def test_mod(self, iterations):
        """Checks the validity of USB-UART connection. Generates a list of an astroid's coordinates as a test sample."""
        n = iterations
        response = ({'header': 'test'})
        response.update({'body': [[(i, 0), (0, abs(abs(i) - n)), (0, -(abs(abs(i) - n)))] for i in range(-n, n + 1)]})
        jsnd_response = json.dumps(response)
        return jsnd_response

    def operator(self, command):
        """Processes the commands received and calls the required modules."""
        if command['header'] == 'test':
            astroid = self.test_mod(command['body']['it'])
            with open('resp.txt', 'w') as resp:
                resp.write(astroid)
            with open('resp.txt', 'r') as r:
                for line in r:
                    return line

        elif command['header'] == 'incubation':
            stpr = SamplingModule()
            _thread.start_new_thread('incubation_thrd', stpr.incubation_thrd, (300, 50, 0,
                float(command['body']['it']),
                float(command['body']['ip']),
                str(command['body']['bf'])))
            response = ({'header': 'incubation'})
            response.update(
                {'body':'Incubation has been initialised.\n\nNote: you may cancel the incubation by clicking on the Stop button.'})
            return json.dumps(response)

        elif command['header'] == 'sampling':
            stpr = SamplingModule()
            # This code-block in thread halts the CPU!
            # _thread.start_new_thread(...)
            stpr.sampling_thrd(0, 1,
                float(command['body']['sqt']),    # quiet time
                int(command['body']['sn']),       # number of samples
                float(command['body']['st']),     # total sampling time spent on each sample
                float(command['body']['si']),     # sampling intervals
                int(command['body']['r2avg']),    # raw-to-average
                str(command['body']['pmr']),      # photodetection mode
                float(command['body']['pv']),     # high voltage value
                float(command['body']['e']))      # applied potential

        elif command['header'] == 'kill':
            for thrd in _thread.list(False):
                if thrd[2] == 'incubation_thrd':
                    _thread.notify(thrd[0], _thread.EXIT)
            response = ({'header': 'kill'})
            response.update({'body': 'Incubation has been canceled.'})
            return json.dumps(response)

        elif command['header'] == 'wifi':
            wfcreds = "ip = {}\nport = {}\nsubnet = {}\ngateway = {}\ndns = {}\nessid = {}\npassword = {}".format(
                command['body']['ip'],
                command['body']['port'],
                command['body']['subnet'],
                command['body']['gateway'],
                command['body']['dns'],
                command['body']['essid'],
                command['body']['password'])
            response = ({'header': 'wifi'})
            response.update({'body': 'Wifi credentials were updated on the LucidSens. Please restart the device and re-establish your connection.'})
            with open('wfcreds.txt', 'w') as f:
                f.write(wfcreds)
            return json.dumps(response)
        else:
            pass

class DetectionModule:
    """Handles analog readings on pin (36: assigned to HV) and pins (34, 39: assigned to SiPM).
    Pin 25 was assigned to DAC
    """
    def __init__(self, adc_pin, dac1_pin=25, voltage=20):
        self.adc = ADC(adc_pin)
        if adc_pin == 36:
            self.mode = 'hv'
        elif adc_pin in [34, 39]:
            self.mode = 'sipm'
        else:
            raise RuntimeError('Invalid pin definition or wrong call.')
        self.voltage = voltage
        self.adc.atten(ADC.ATTN_11DB)
        self.dac1 = DAC(dac1_pin)

    def sampler(self, acquisition=3, intervals=0.1, raw_to_avrg=10):
        """
        acquisition: ADC total read-time in seconds
        intervals: sampling intervals in seconds
        raw_to_avrg: number of raw samples to be collected and averaged between the two intervals
        returns an array[list] of the ADC read
        """
        def dac1_calibration(val):
            """DAC calibration module for a linear voltage-to-response read."""
            return int((val + 5.15) / 0.39)

        def adc_calibration(data):
            """ADC calibration module for a linear signal-to-voltage read."""
            if self.mode == 'hv':
                return (data + 176.3)/79.3 # calibrated on 8052021
            elif self.mode == 'sipm':
                return (data + 168.6)/1288.2
            else:
                raise RuntimeError('Invalid pin definition or wrong call.')

        frq = int(raw_to_avrg / intervals)
        datapoint = array('H', raw_to_avrg)
        true_set = array('d')
        calib_set = array('d')

        self.dac1.write(dac1_calibration(self.voltage))

        for _ in range(int(acquisition/intervals)):
            self.adc.collect(freq=frq, data=datapoint)
            while True:
                try:
                    if not self.adc.progress()[0]:
                        true_val = self.adc.collected()[2]
                        # returns the average, 0 -> Min, 1-> Max, 3 -> rms
                        calib_val = adc_calibration(self.adc.collected()[2])
                        true_set.append(true_val)
                        calib_set.append(calib_val)
                        # print("True_point: {} ----- Caliberated_point: {}".format(true_val, calib_val))
                        break
                except KeyboardInterrupt:
                    self.adc.deinit()
                    return
                except Exception as e:
                    self.adc.deinit()
                    print(e)
                    return "deinitialised the ADC, pins were released, exiting."
        # print('true val: {}'.format(round(sum(true_set)/len(true_set),1)))
        # print('calib val: {}'.format(round(sum(calib_set)/len(calib_set),2)))

        # returning the list of calibrated acquired values
        return calib_set

    def deinit(self):
        self.adc.deinit()
        self.dac1.write(0)
        self.dac1.deinit()

class EJack:
    # pylint: disable=too-many-instance-attributes
    """ESP32 (PWM-based) Micropython class for DRV8825 stepper-motor driver.
    pylint: disable=too-many-instance-attributes
    """
    def __init__(self, pwr_pin=19, dir_pin=2, step_pin=12):
        """Initialization of the stepper driver associated with the side-stepper
        Side_Stepper involved pins: 19, 2, 12
        """
        self.pwr = Pin(pwr_pin, Pin.INOUT, value=0)
        self.dir = Pin(dir_pin, Pin.INOUT, value=0)
        self.stepper = Pin(step_pin, Pin.OUT, value=0)
        self.spr = 200  # 200 Steps per revolution: 360 / 1.8

    def lift_up(self, rounds=500, delay=0.002):
        """EJack lifts up."""
        self.dir.value(0)
        self.pwr.value(1)

        try:
            for _ in range(rounds):
                self.stepper.value(1)
                sleep(delay)
                self.stepper.value(0)
            self.pwr.value(0)
        except KeyboardInterrupt:
            self.pwr.value(0)
            return 'aborted!'
        except Exception as e:
            print(e)
            self.pwr.value(0)
            return 'something is not right! shutting down.'

    def pull_down(self, rounds=500, delay=0.002):
        """EJack pulls down."""
        self.dir.value(1)
        self.pwr.value(1)
        try:
            for _ in range(rounds):
                self.stepper.value(1)
                sleep(delay)
                self.stepper.value(0)
            self.pwr.value(0)
        except KeyboardInterrupt:
            self.pwr.value(0)
            return 'aborted!'
        except Exception as e:
            print(e)
            self.pwr.value(0)
            return 'something is not right! shutting down.'

class EChem:
    # pylint: disable=too-many-instance-attributes
    """Initialises DAC2 and controls the potentials at the working electrode pin.
    """
    def __init__(self, dac2_pin=26):
        self.dac2 = DAC(dac2_pin)

    def dac2_calibration(self, val):
        """DAC2 calibration module for a linear voltage-to-response read."""
        return int((val - 0.087) / 0.012)

    def applied_potential(self, e):
        self.dac2.write(self.dac2_calibration(e))

    def release(self):
        self.dac2.write(0)

    def deinit(self):
        self.dac2.write(0)
        self.dac2.deinit()

class SamplingModule:
    # pylint: disable=too-many-instance-attributes
    """ESP32 (PWM-based) Micropython class for DRV8825 stepper-motor driver.
    pylint: disable=too-many-instance-attributes
    """
    def __init__(self, pwr_pin=13, dir_pin=33, step_pin=32, intrptr_pin=35):
        """Initialization of the stepper driver and the peltier module."""
        self.pwr = Pin(pwr_pin, Pin.INOUT, value=0)
        self.dir = Pin(dir_pin, Pin.INOUT, value=0)
        self.intrptr = Pin(intrptr_pin, Pin.IN)
        self.step_pin = step_pin
        self.spr = 200
        self.delay = 0.01
        self._current_temp = TemperatureSensor()
        self.current_temp = float()
        self.target_temp = float()
        self.p_BF = Pin(14, Pin.INOUT, value=0)
        self.p_EN = Pin(21, Pin.INOUT, value=0)
        self.p_PH = Pin(22, Pin.INOUT, value=0)
        self.timer = 0
        self.blower = 'Off'
        self.collected_data = []
        self.ejack = EJack()
        self.echem = EChem()

    def incubation_thrd(self, freq=300, duty=50, direction=0, duration=1, target_temp=37, blower='On'):
        """Motor rotates for [duration]minutes."""
        _thread.allowsuspend(True)
        stepper = PWM(self.step_pin)
        duration = duration * 60
        self.timer = duration
        self.blower = blower
        def ramp_gen(freq):
            _a, _b, _base = 1, 7, 100
            return [int(freq * 1/(1+2**(_a*(x-_b))) * 1/(1+2**(-_a*(x+_b)))) + _base for x in range(-12,13)]

        ramp = ramp_gen(freq)
        self.target_temp = target_temp
        while True:
            notif = _thread.getnotification()
            if notif == _thread.EXIT:
                self.p_EN.value(0)
                self.p_BF.value(0)
                self.interrupter()
                return

            try:
                self.current_temp = self._current_temp.read()
                sleep(1)
                if self.current_temp is not False:
                    tft.ambient_sensor()
                    if self.current_temp[0] < int(self.target_temp - 1):
                        tft.temp_status('rising')
                        if self.pwr.value():
                            self.pwr.value(0)
                            sleep(1)

                        if self.blower == 'On':
                            if not self.p_BF.value():
                                self.p_BF.value(1)
                                sleep(1)

                        if not self.p_EN.value():
                            self.p_EN.value(1)

                        if self.p_PH.value():
                            self.p_PH.value(0)
                        sleep(15)

                    elif self.current_temp[0] > int(self.target_temp + 1):
                        tft.temp_status('falling')
                        if self.pwr.value():
                            self.pwr.value(0)
                            sleep(1)

                        if self.blower == 'On':
                            if not self.p_BF.value():
                                self.p_BF.value(1)

                        for pin in [self.p_EN, self.p_PH]:
                            if not pin.value():
                                pin.value(1)
                        sleep(15)

                    else:
                        for pin in [self.p_EN, self.p_BF, self.p_PH]:
                            pin.value(0)
                        sleep(3)
                        tft.temp_status('idle')
                        self.dir.value(direction if direction in [0, 1] else 0)
                        stepper.duty(duty)
                        self.pwr.value(1)
                        for idx, frequency in enumerate(ramp):
                            # print(idx, frequency)
                            if int(self.target_temp - 1) <= self.current_temp[0] <= int(self.target_temp + 1):
                                stepper.freq(frequency)
                                sleeping_time = round((duration / sum(ramp)) * frequency, 3)
                                # print('@Frequency: {}Hz --- Duration: {}s'.format(frequency, sleeping_time))
                                sleep(sleeping_time)
                                self.timer -= sleeping_time
                                self.current_temp = self._current_temp.read()
                                # print("current temperature: {}".format(self.current_temp[0]))
                                sleep(1)
                                tft.ambient_sensor()

                                if self.timer < 5:
                                    stepper.deinit()
                                    self.interrupter()
                                    return "incubation is done.\n"
                            else:
                                # Stepper will be stopped in 10 seconds
                                _ramp = ramp[idx+1:] if idx>len(ramp)/2 else ramp[idx-1::-1]
                                for _frequency in _ramp:
                                    sleep(round((10 / sum(_ramp)) * _frequency, 3))
                                break
            except Exception as e:
                for pin in [self.p_EN, self.p_BF, self.p_PH]:
                    pin.value(0)
                return e

    def sampling_thrd(self, direction=0, cycles=1, quiet_time=2.0, samples=3, acquisition_time=1, intervals=0.1, raw2avg=10, adc_read='Slow', pv=30, e=1.0):
        """Motor rotates clockwise for [revs]cycles;
        in each cycle rotates for 200/[samples] * 1.8 degrees;
        stops for [acquisition_time]seconds on each sample."""
        # _thread.allowsuspend(True)
        sleep(quiet_time)
        stepper = Pin(self.step_pin, Pin.OUT)
        self.collected_data = []
        if adc_read == 'Slow':
            sampler = DetectionModule(34, 25, pv)
        else:
            sampler = DetectionModule(39, 25, pv)
        self.interrupter()
        self.pwr.value(1)
        self.dir.value(direction if direction in [0, 1] else 0)
        for thrd in _thread.list(False):
            if thrd[2] == 'status_thrd':
                _thread.notify(thrd[0], _thread.EXIT)

        while True:
            try:
                for _ in range(cycles):
                    for i in range(samples):
                        for _ in range(int(self.spr/samples)):
                            stepper.value(1)
                            sleep(self.delay)
                            stepper.value(0)
                            sleep(self.delay)

                        self.ejack.pull_down(1200, 0.001)
                        self.echem.applied_potential(1) # testing

                        self.collected_data.append(('Sample {}:'.format(i+1), sampler.sampler(acquisition_time, intervals, raw2avg)))

                        self.echem.release()
                        self.ejack.lift_up(1200, 0.001)
                response = ({'header': 'sampling'})
                response.update({'body': self.collected_data})
                response.update({'notes': [samples, acquisition_time, intervals, raw2avg]})

                # self.echem.deinit()
                # sampler.deinit()
                self.interrupter()

                with open('resp.txt', 'w') as resp:
                    resp.write(json.dumps(response))

                with open('resp.txt', 'r') as r:
                    for line in r:
                        SerialConnection().sr_sender(line)
                _thread.start_new_thread('status_thrd', tft.status_thrd, ())
                print('sender is done')
                return 'Done sending data.'

            except KeyboardInterrupt:
                print(RED + 'stepper: aborted!' + WHITE)
                self.interrupter()
                return
            except Exception as e:
                print(e)
                self.interrupter()

    def interrupter(self, delay=0.01):
        """Motor rotates clockwise to finally be stopped by the opto-interrupter."""
        self.dir.value(0)
        self.pwr.value(1)
        stepper = Pin(self.step_pin, Pin.OUT)
        try:
            if self.intrptr.value():
                self.pwr.value(0)
                return
            while True:
                stepper.value(1)
                sleep(delay)
                stepper.value(0)
                if self.intrptr.value():
                    self.pwr.value(0)
                    break
            return
        except KeyboardInterrupt:
            self.pwr.value(0)
            return 'aborted!'
        except Exception as e:
            print(e)
            self.pwr.value(0)
            return 'shutting down.'

    def stpr_chk(self, delay=0.01, duration=10):
        """Motor rotates clockwise to finally be stopped by the opto-interrupter."""
        self.dir.value(0)
        self.pwr.value(1)
        stepper = Pin(self.step_pin, Pin.OUT)

        try:
            for _ in range(int(duration/delay)):
                stepper.value(1)
                sleep(delay)
                stepper.value(0)
            self.pwr.value(0)
            return
        except KeyboardInterrupt:
            self.pwr.value(0)
            return 'aborted!'
        except Exception as e:
            print(e)
            self.pwr.value(0)
            return 'shutting down.'

class DisplayModule:
    # pylint: disable=too-many-instance-attributes
    """handles the TFT (128x128 pixels) LCD module."""
    def __init__(self):
        self.tft = TFT()
        self.tft.init(self.tft.ST7735R, speed=10000000, spihost=self.tft.VSPI, mosi=23, miso=19, clk=18, cs=5, dc=15,rst_pin=26, hastouch=False, bgr=False, width=128, height=128, rot=2) # lcd flipped
        # self.tft.tft_writecmd(ST7735_INVOFF)
        # tft.init(tft.ST7735R, speed=10000000, spihost=tft.VSPI, mosi=23, miso=17, clk=18, cs=5, dc=15,rst_pin=16, hastouch=False, bgr=False, width=128, height=128, invrot=2)
        self.max_x, self.max_y = self.tft.screensize()
        self.init_x, self.init_y = 2, 10
        self.rtc = RTC()
        self.adc = DetectionModule(36)
        self.fan = Pin(14, Pin.OUT)
        self.tft.image(self.init_x + 100, self.init_y + 77, 'ICONS/humidity.jpg', 2, self.tft.JPG)
        self.rtc.ntp_sync(server='cn.pool.ntp.org', tz='CST-8')
        self.tmp = TemperatureSensor()

    def ntp(self):
        """Network Time Protocol (needs wifi access, currently set on China time-zone)."""
        if not self.rtc.synced():
            self.rtc.ntp_sync(server='cn.pool.ntp.org', tz='CST-8')
            sleep(2)

    def logo(self, signal):
        """Welcome image."""
        if signal:
            self.tft.image(12, 2, 'logo.jpg', 2)
            sleep(5)
        else:
            self.clear_panel(self.init_x + 2, self.init_y + 21, 120, 110)

    def frame(self):
        """Draws a frame arond the LCD."""
        self.tft.rect(self.init_x, self.init_y, self.max_x - self.init_x, self.max_y - self.init_y, color=self.tft.BLUE)

    def write(self, font, txt, color, x=2, y=3):
        self.tft.font(font)
        self.tft.text(x, y, txt, color)

    def clear(self):
        self.tft.clear(color=self.tft.BLACK)

    def clear_panel(self, a, b, i, j):
        self.tft.rect(self.init_x + a, self.init_y + b, i, j, color=self.tft.BLACK, fillcolor=self.tft.BLACK)

    def time_panel(self):
        self.clear_panel(self.init_x + 45, self.init_y + 100, 35, 5)  # TIME panel
        self.write('FONTS/font10B.fon', txt=strftime('%H:%M', localtime()), color=self.tft.GREEN, x=self.init_x + 50, y=self.init_y + 100)

    def date_panel(self):
        self.clear_panel(self.init_x + 26, self.init_y + 91, 72, 12)  # DATE panel
        # day = strftime('%a', localtime())
        # self.write(font=self.tft.FONT_Default, txt=day, x=5, y=5)
        date = strftime('%d-%b-%Y', localtime())
        self.write('FONTS/font10B.fon', txt=date, color=self.tft.GREEN, x=self.init_x + 32, y=self.init_y + 91)

    def ambient_sensor(self):
        sleep(1)
        self.clear_panel(self.init_x + 100, self.init_y + 55, 24, 12)  # HUMIDITY panel
        self.clear_panel(self.init_x + 5, self.init_y + 110, 93, 40)  # MAIN panel
        _sensor = self.tmp.read()
        if _sensor:
            temperature, humidity = _sensor[0], _sensor[1]

            self.write('FONTS/font10.fon', txt=str(humidity), color=self.tft.WHITE, x=self.init_x + 100, y=self.init_y + 65)
            self.write('FONTS/font48.fon', txt=str(temperature), color=self.tft.RED, x=self.init_x + 5, y=self.init_y + 31)
            self.write('FONTS/font10.fon', txt='o', color=self.tft.RED, x=self.init_x + 101, y=self.init_y + 37)
            self.write('FONTS/font20B.fon', txt='C', color=self.tft.RED, x=self.init_x + 105, y=self.init_y + 42)
            # self.frame()
        else:
            print('Temperature sensor is not connected.')

    def hv_panel(self, signal):
        self.clear_panel(self.init_x + 5, self.init_y + 75, 40, 20)
        if signal:
            collected_samples = self.adc.sampler(3, 1, 5)
            voltage = 'HV Output: {:.1f} V'.format(sum(collected_samples)/len(collected_samples))
            self.write('FONTS/font10.fon', txt=voltage, color=self.tft.BLUE, x=self.init_x + 5, y=self.init_y + 80)
            self.adc.deinit()
        else:
            self.clear_panel(self.init_x + 5, self.init_y + 76, 40, 20)

    def serial_status(self, signal):
        self.clear_panel(self.init_x + 100, self.init_y + 3, 22, 22)
        if signal:
            self.tft.image(self.init_x + 100, self.init_y + 3, 'ICONS/serial.jpg', 2, self.tft.JPG)
        else:
            self.tft.image(self.init_x + 100, self.init_y + 3, 'ICONS/noSerial.jpg', 2, self.tft.JPG)

    def wifi_status(self, signal):
        self.clear_panel(self.init_x + 78, self.init_y + 3, 22, 22)
        if signal:
            self.tft.image(self.init_x + 78, self.init_y + 3, 'ICONS/wifi.jpg', 2, self.tft.JPG)
        else:
            self.tft.image(self.init_x + 78, self.init_y + 3, 'ICONS/noWifi.jpg', 2, self.tft.JPG)

    def temp_status(self, signal='idle'):
        self.clear_panel(self.init_x + 56, self.init_y + 3, 22, 22)
        if signal == 'rising':
            self.tft.image(self.init_x + 56, self.init_y + 3, 'ICONS/tempRise.jpg', 2, self.tft.JPG)
        elif signal == 'falling':
            self.tft.image(self.init_x + 56, self.init_y + 3, 'ICONS/tempFall.jpg', 2, self.tft.JPG)
        else:
            self.tft.image(self.init_x + 56, self.init_y + 3, 'ICONS/temp.jpg', 2, self.tft.JPG)

    def opr_status(self, signal):
        self.clear_panel(self.init_x + 32, self.init_y + 3, 22, 22)
        if signal == 'hv':
            self.tft.image(self.init_x + 32, self.init_y + 3, 'ICONS/hv.jpg', 2, self.tft.JPG)
        elif signal == 'stepper':
            self.tft.image(self.init_x + 32, self.init_y + 3, 'ICONS/stepper.jpg', 2, self.tft.JPG)
        elif signal == 'detector':
            self.tft.image(self.init_x + 32, self.init_y + 3, 'ICONS/phDetector.jpg', 2, self.tft.JPG)
        elif signal == 'done':
            self.tft.rect(self.init_x + 32, self.init_y + 3, 22, 22, color=self.tft.BLACK, fillcolor=self.tft.BLACK)
        else:
            pass

    def connect_status(self):
        self.clear_panel(self.init_x + 8, self.init_y + 3, 22, 22)
        self.tft.image(self.init_x + 8, self.init_y + 3, 'ICONS/connect.jpg', 2, self.tft.JPG)

    def wait_status(self, signal):
        if signal:
            self.clear_panel(self.init_x + 47, self.init_y + 112, 35, 12)  # TIME panel
            self.clear_panel(self.init_x + 28, self.init_y + 99, 72, 12)  # DATE panel
            self.clear_panel(self.init_x + 7, self.init_y + 27, 93, 65)  # MAIN panel
            self.tft.image(self.init_x + 7, self.init_y + 27, 'ICONS/wait.jpg', 0, self.tft.JPG)

    def tft_close(self):
        self.clear()
        # self.frame()
        self.write(font='FONTS/font20B.fon', txt='deinitialized!', color=self.TFT.RED, x=20, y=60)
        self.tft.deinit()
        print(RED + 'TFT: module deinitialized.' + WHITE)

    def status_thrd(self):
        self.tft.image(self.init_x + 100, self.init_y + 77, 'ICONS/humidity.jpg', 2, self.tft.JPG)
        _thread.allowsuspend(True)
        while True:
            ntf = _thread.getnotification()
            if ntf == _thread.EXIT:
                return
            elif ntf == _thread.SUSPEND:
                for thrd in _thread.list(False):
                    if thrd[2] == 'status_thrd':
                        _thread.suspend(thrd[0])
                while _thread.wait() != _thread.RESUME:
                    pass
            try:
                self.ntp()
                self.ambient_sensor()
                self.date_panel()
                self.time_panel()
                sleep(30)
            except KeyboardInterrupt:
                print(RED + 'TFT: aborted!' + WHITE)
                break
            except Exception as e:
                print(e)
                break

class SerialConnection:
    """Handles the serial connection and data transfer over the serial connection."""
    def __init__(self):
        self.read, self.signal, self.content= '', '', ''
        self.opr = CommandHandler()
        self.clear = Clear()
        self.seg_size = 1024

    def read_until(self, ending, timeout=10000):
        self.read = stdin_get(1, timeout)
        if self.read is None:
            return '\n'

        timeout_count = 0
        while True:
            if self.read is not None:
                if self.read.endswith(ending):
                    break
                else:
                    new_data = stdin_get(1, timeout)
                    self.read += new_data
                    timeout_count = 0
            else:
                timeout_count += 1
                if timeout is not None and timeout_count >= timeout:
                    break
        return self.read

    def sr_sender(self, response):
        """Sends the response back to GUI over serial."""
        self.clear()
        def chopper(cmd):
            data = []
            segments = [cmd[i:i + self.seg_size] for i in range(0, len(cmd), self.seg_size)]
            for idx, segment in enumerate(segments, start=1):
                if segment == segments[-1]:
                    data.append(segment + '*#')
                else:
                    data.append(segment + '<{}/{}>_#'.format(idx, len(segments)))
            return data
        # print(response, type(response))
        if response:
            if len(response) > self.seg_size:
                for data in ([chunk for chunk in chopper(response)]):
                    for _ in range(3):
                        stdout_put(data)
                        sleep(0.125)
                        resp = self.read_until('#', 5000)
                        if 'EOF received.' in resp:
                            sleep(0.002)
                            return '\nresponse was sent, exiting the sender, chopper involved.'
                        elif 'got it.' in resp:
                            sleep(0.002)
                            break
                        else:
                            sleep(0.125)
            else:
                for _ in range(3):
                    stdout_put(response + "*#")
                    sleep(0.125)
                    resp = self.read_until('#', 5000)
                    if 'EOF received.' in resp:
                        sleep(0.002)
                        return '\nresponse was sent, exiting the sender, no chopper.'
                    else:
                        sleep(0.125)
        return '\nsender: done.'

    def sr_receiver(self):
        """Receives the commands over serial.
        type !# to exit.
        """
        self.clear()
        self.content, self.signal = '', ''

        while True:
            while 'go#' not in self.signal and '!#' not in self.signal:
                stdout_put('sr_receiver: READY\n')
                sleep(1)
                self.signal = self.read_until('#')
            if '!#' in self.signal:
                print('sr_receiver:' + RED + ' aborted!' + WHITE)
                sys.exit(0)
            else:
                stdout_put('got it.\n')
                self.clear()
                self.signal = ''
            while '*' not in self.content:
                try:
                    sleep(0.125)
                    data = self.read_until('#')
                    if '#' in data and data[:-2] not in self.content:
                        if data[-2] == '*':
                            self.content += data[:-1]
                            break
                        elif data[-2] == '_':
                            self.content += data[:-2]
                            stdout_put('got it.\n')
                            sleep(0.125)
                        else:
                            sleep(0.125)
                    else:
                        sleep(0.125)
                except Exception as e:
                    print(e)
                    break
            sleep(0.25)
            if '*' in self.content:
                stdout_put('EOF received.\n')
                sleep(1)
                with open('cmd.txt', 'w') as raw_cmd:
                    raw_cmd.write(self.content[:-1])
                self.content = ''
                with open('cmd.txt', 'r') as order:
                    for line in order:
                        self.sr_sender(self.opr.operator(eval(line)))
                print('returned back to receiver.')
            else:
                return "operator: invalid command."
            print('reruning the receiver')
            self.sr_receiver()
        return 'exiting the sr_receiver.'

class WifiConnection:
    # pylint: disable=too-many-instance-attributes
    """Handles Wifi connections.
    Reads the Wifi credentials from the wfcred.txt files and tries to establish a connection.
    """
    def __init__(self):
        self.p_led = Pin(2, Pin.OUT)
        self.clear = Clear()
        self.station = WLAN(STA_IF)
        self.station.active(True)

    def wf_handler(self):
        "Receives commands over Wifi. ---- This method needs re-consideration."
        opr = CommandHandler()
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.ip, self.port))
        while True:
            try:
                self.clear()
                sock.listen(5)
                conn, _ = sock.accept()
                while True:
                    try:
                        if conn:
                            cmd = conn.recv(4096).decode()
                            parsed_cmd = json.loads(cmd)
                            if parsed_cmd['header'] in ['run', 'tst']:
                                conn.sendall(opr.operator(parsed_cmd))
                                break
                                # sock.close()
                            else:
                                # sock.close()
                                break
                        else:
                            break
                    except KeyboardInterrupt:
                        print("\nwifi_thrd: aborted!")
                        break
                    except Exception as e:
                        print(e)
                        break

            except KeyboardInterrupt:
                print("\nwifi_thrd: aborted!")
                sock.close()
                break

            except Exception as e:
                print(e)
                sock.close()
                break

    def wf_connection(self):
        """Manages the Wifi connection."""
        if 'wfcreds.txt' not in os.listdir():
            return False
        try:
            with open("wfcreds.txt", 'r') as f:
                for lines in f:
                    for line in lines.split('\r'):
                        if "#" in line:
                            pass
                        elif "ip" in line:
                            _ip = line.strip().split(' ')[2]
                        elif "port" in line:
                            _port = int(line.strip().split(' ')[2])
                        elif "subnet" in line:
                            _subnet = line.strip().split(' ')[2]
                        elif "gateway" in line:
                            _gateway = line.strip().split(' ')[2]
                        elif "dns" in line:
                            _dns = line.strip().split(' ')[2]
                        elif "essid" in line:
                            essid = line.strip().split(' ')[2]
                        elif "password" in line:
                            password = line.strip().split(' ')[2]
                        else:
                            pass
        except Exception as e:
            print(e)
        # print("connecting to the pre-defined:\nessid ---> {} ||| password ---> {}".format(essid, password))
        self.station.connect(essid, password)
        sleep(3)
        if self.station.isconnected():
            return self.station.isconnected()
        else:
            if not self.station.active():
                self.station.active(True)
            for _ in range(2):
                self.station.connect(essid, password)
                sleep(2)
                if self.station.isconnected():
                    return self.station.isconnected()
            return self.station.isconnected()

    def wf_disconnect(self):
        """Manages Wifi disconnection."""
        self.station.disconnect()
        # print("wifi_thrd: shutting down the wifi.")
        self.station.active(False)
        self.p_led.value(0)

    def auth_mode(self, mod):
        lst = {0: 'open',
               1: 'WEP',
               2: 'WPA-PSK',
               3: 'WPA2-PSK',
               4: 'WPA/WPA2-PSK'
               }
        return lst.get(mod)

    def hidden(self, mod):
        lst = {False: "visible",
               True: "hidden"}
        return lst.get(mod)

    # def __str__(self):
    #     print("WiFiConn class is activated.")

def main():
    def gc_thrd():
        _thread.allowsuspend(True)
        while True:
            ntf = _thread.getnotification()
            if ntf == _thread.EXIT:
                return 'gc_thrd: EXIT command received.'
            gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())
            if gc.mem_free() < gc.threshold():
                gc.collect()
            sleep(10)

    try:
        wf = WifiConnection()

        global tft
        tft = DisplayModule()
        tft.clear()
        tft.logo(True)
        tft.logo(False)
        tft.clear()
        tft.connect_status()
        tft.serial_status(False)
        tft.temp_status()
        # tft.frame()
        tft.ambient_sensor()
        print('TFT module initialised.')

        print('establishing wifi connection.')
        if wf.wf_connection():
            tft.wifi_status(True)
            print(YELLOW + "wifi is connected." + GREEN)
            # wf_thrd = _thread.start_new_thread('wifi_thrd', wf.wf_handler, ())
        else:
            print(RED + 'wifi was not connected, shutting down the module.' + GREEN)
            wf.wf_disconnect()
            tft.wifi_status(False)

        tft.hv_panel(True)

        print('initialising the stepper.')
        tft.opr_status('stepper')
        stpr = SamplingModule()
        stpr.interrupter()
        print(YELLOW + 'stepper adjusted to point zero.' + GREEN)
        tft.opr_status('done')

        _thread.start_new_thread('gc_thrd', gc_thrd, ())
        print(CYAN + 'gc_thrd initialised.' + GREEN)
        sleep(1)

        if wf.station.isconnected():
            _thread.start_new_thread('status_thrd', tft.status_thrd, ())
            sleep(1)
            print(CYAN + 'status_thrd initialised.' + GREEN)
            sleep(1)

        print('establishing serial connection.')
        print('to exit the serial receiver type -> !# ')
        tft.serial_status(True)
        sr = SerialConnection()
        sleep(3)
        sr.sr_receiver()

    except KeyboardInterrupt:
        print(RED + 'main module aborted!' + WHITE)
        sys.exit(0)

    except Exception as e:
        print(e)
        sys.exit(1)
if __name__ == '__main__':
    main()

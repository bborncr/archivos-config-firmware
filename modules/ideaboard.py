from machine import Pin, PWM, ADC
from neopixel import NeoPixel

class IdeaBoard:
    def __init__(self, m1a=12, m1b=14, m2a=13, m2b=15):
        self._pixel_pin = Pin(2, Pin.OUT)
        self._brightness = 0.5
        self._np = NeoPixel(self._pixel_pin, 1)
        self._np[0] = (0,0,0)
        self._m1a = PWM(Pin(m1a))
        self._m1b = PWM(Pin(m1b))
        self._m2a = PWM(Pin(m2a))
        self._m2b = PWM(Pin(m2b))
    
    def pixel(self, color=(0,0,0)):
        r,g,b = color
        r = int(r * self._brightness)
        g = int(g * self._brightness)
        b = int(b * self._brightness)
        color = (r,g,b)
        #print('color',color)
        self._np[0] = color
        self._np.write()
    
    @property
    def brightness(self):
        return self._brightness
    
    @brightness.setter
    def brightness(self, new_brightness):
        if new_brightness <= 1 or new_brightness >= 0:
            self._brightness = new_brightness
    
    def arcoiris(self, n=0):
        color = self._wheel(n)
        self.pixel(color)
    
    def motor_1(self, vel=0):
        if vel < -1023 | vel > 1023:
            print(f"velocidad no valida: {vel}")
        if vel >= 0:
            self._m1a.duty(vel)
            self._m1b.duty(0)
        if vel < 0:
            self._m1a.duty(0)
            self._m1b.duty(vel * -1)
    
    def motor_2(self, vel=0):
        if vel < -1023 | vel > 1023:
            print(f"velocidad no valida: {vel}")
        if vel >= 0:
            self._m2a.duty(vel)
            self._m2b.duty(0)
        if vel < 0:
            self._m2a.duty(0)
            self._m2b.duty(vel * -1)
    
    def _wheel(self, pos):
        # Adafruit color wheel code
        # Input a value 0 to 255 return RGB tuple.
        if pos < 0:
            return (0, 0, 0)
        if pos > 255:
            return (0, 0, 0)
        if pos < 85:
            return (int(pos * 3), int(255 - (pos * 3)), 0)
        elif pos < 170:
            pos -= 85
            return (int(255 - pos * 3), 0, int(pos * 3))
        else:
            pos -= 170
            return (0, int(pos * 3), int(255 - pos * 3))
        
    def scale(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        
        
    class Servo:
        """
        A simple class for controlling hobby servos.

        Args:
            pin: The pin where servo is connected. Must support PWM.
            freq (int): The frequency of the signal, in hertz.
            min_us (int): The minimum signal length supported by the servo.
            max_us (int): The maximum signal length supported by the servo.
            max_angle (int): The angle between the minimum and maximum positions.

        """
        def __init__(self, pin, freq=50, min_us=600, max_us=2400, max_angle=180):
            self.min_us = min_us
            self.max_us = max_us
            self.us = 0
            self.freq = freq
            self.max_angle = max_angle
            self.pwm = PWM(Pin(pin), freq=freq, duty=0)

        def write_us(self, us):
            """Set the signal to be ``us`` microseconds long. Zero disables it."""
            if us == 0:
                self.pwm.duty(0)
                return
            us = min(self.max_us, max(self.min_us, us))
            duty = us * 1024 * self.freq // 1000000
            self.pwm.duty(duty)

        def angle(self, degrees):
            """Move to the specified angle in ``degrees``."""
            degrees = degrees % 360
            total_range = self.max_us - self.min_us
            us = self.min_us + total_range * degrees // self.max_angle
            self.write_us(us)
            
    class AnalogIn:
        def __init__(self, pin):
            self._pin = ADC(Pin(pin), atten=ADC.ATTN_11DB)
        
        def read(self):
            return self._pin.read()

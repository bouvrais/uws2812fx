import uasyncio
import machine
import utime
import random
import neopixel
import copy

gamma8 = [
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,
    1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,  2,  3,  3,  3,  3,
    3,  3,  4,  4,  4,  4,  5,  5,  5,  5,  5,  6,  6,  6,  6,  7,
    7,  7,  8,  8,  8,  9,  9,  9, 10, 10, 10, 11, 11, 11, 12, 12,
   13, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20,
   20, 21, 21, 22, 22, 23, 24, 24, 25, 25, 26, 27, 27, 28, 29, 29,
   30, 31, 31, 32, 33, 34, 34, 35, 36, 37, 38, 38, 39, 40, 41, 42,
   42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57,
   58, 59, 60, 61, 62, 63, 64, 65, 66, 68, 69, 70, 71, 72, 73, 75,
   76, 77, 78, 80, 81, 82, 84, 85, 86, 88, 89, 90, 92, 93, 94, 96,
   97, 99,100,102,103,105,106,108,109,111,112,114,115,117,119,120,
  122,124,125,127,129,130,132,134,136,137,139,141,143,145,146,148,
  150,152,154,156,158,160,162,164,166,168,170,172,174,176,178,180,
  182,184,186,188,191,193,195,197,199,202,204,206,209,211,213,215,
  218,220,223,225,227,230,232,235,237,240,242,245,247,250,252,255]

sine8 = [
  128,131,134,137,140,143,146,149,152,155,158,162,165,167,170,173,
  176,179,182,185,188,190,193,196,198,201,203,206,208,211,213,215,
  218,220,222,224,226,228,230,232,234,235,237,238,240,241,243,244,
  245,246,248,249,250,250,251,252,253,253,254,254,254,255,255,255,
  255,255,255,255,254,254,254,253,253,252,251,250,250,249,248,246,
  245,244,243,241,240,238,237,235,234,232,230,228,226,224,222,220,
  218,215,213,211,208,206,203,201,198,196,193,190,188,185,182,179,
  176,173,170,167,165,162,158,155,152,149,146,143,140,137,134,131,
  128,124,121,118,115,112,109,106,103,100, 97, 93, 90, 88, 85, 82,
   79, 76, 73, 70, 67, 65, 62, 59, 57, 54, 52, 49, 47, 44, 42, 40,
   37, 35, 33, 31, 29, 27, 25, 23, 21, 20, 18, 17, 15, 14, 12, 11,
   10,  9,  7,  6,  5,  5,  4,  3,  2,  2,  1,  1,  1,  0,  0,  0,
    0,  0,  0,  0,  1,  1,  1,  2,  2,  3,  4,  5,  5,  6,  7,  9,
   10, 11, 12, 14, 15, 17, 18, 20, 21, 23, 25, 27, 29, 31, 33, 35,
   37, 40, 42, 44, 47, 49, 52, 54, 57, 59, 62, 65, 67, 70, 73, 76,
   79, 82, 85, 88, 90, 93, 97,100,103,106,109,112,115,118,121,124]

class Segment:
    def __init__(self, start, stop, mode, colors, speed, options):
        self.mode = mode
        self.speed = speed
        self.start = start
        self.stop = stop 
        self.colors = colors
        self.len = self.stop - self.start + 1
        self.options = options
        self.active = True

    MAX_NUM_COLORS = 3

    # segment options
    # bit    7: reverse animation
    # bits 4-6: fade rate (0-7)
    # bit    3: gamma correction
    # bits 1-2: size
    # bits   0: TBD
    NO_OPTIONS   = 0b00000000
    REVERSE      = 0b10000000
    FADE_XFAST   = 0b00010000
    FADE_FAST    = 0b00100000
    FADE_MEDIUM  = 0b00110000
    FADE_SLOW    = 0b01000000
    FADE_XSLOW   = 0b01010000
    FADE_XXSLOW  = 0b01100000
    FADE_GLACIAL = 0b01110000
    
    GAMMA        = 0b00001000
    
    SIZE_SMALL   = 0b00000000
    SIZE_MEDIUM  = 0b00000010
    SIZE_LARGE   = 0b00000100
    SIZE_XLARGE  = 0b00000110
    def is_reverse(self):
        return self.options & Segment.REVERSE == Segment.REVERSE
    def fade_rate(self):
        return (self.options >> 4) & 7
    def is_gamma(self):
        return self.options & Segment.GAMMA == Segment.GAMMA
    def size_option(self):
        return (self.options >> 1) & 3

    def setMode(self, mode):
        self.mode = mode


# segment runtime parameters
class SegmentRuntime:
    def __init__(self):
        self.reset()

    def reset(self):
      self.next_time = 0
      self.counter_mode_step = 0
      self.counter_mode_call = 0
      self.aux_param = 0   # auxilary param (usually stores a color_wheel index)
      self.aux_param2 = 0  # auxilary param (usually stores bitwise options)
      self.aux_param3 = 0  # auxilary param (usually stores a segment index)

    FRAME = 0b10000000
    CYCLE = 0b01000000
    def set_frame(self):
        self.aux_param2 |=  SegmentRuntime.FRAME
    def clr_frame(self):
        self.aux_param2 &= ~SegmentRuntime.FRAME
    def set_cycle(self):
        self.aux_param2 |=  SegmentRuntime.CYCLE
    def clr_cycle(self):
        self.aux_param2 &= ~SegmentRuntime.CYCLE
    def clr_frame_cycle(self):
        self.aux_param2 &= ~(SegmentRuntime.CYCLE | SegmentRuntime.FRAME)




class WS2812FX:
    MAX_TIME = 60000
    SPEED_MIN = 2
    SPEED_MAX = 65535

    # some common colors
    RED        = (0xFF,0x00,0x00)
    GREEN      = (0x00,0xFF,0x00)
    BLUE       = (0x00,0x00,0xFF)
    WHITE      = (0xFF,0xFF,0xFF)
    BLACK      = (0x00,0x00,0x00)
    YELLOW     = (0xFF,0xFF,0x00)
    CYAN       = (0x00,0xFF,0xFF)
    MAGENTA    = (0xFF,0x00,0xFF)
    PURPLE     = (0x40,0x00,0x80)
    ORANGE     = (0xFF,0x30,0x00)
    PINK       = (0xFF,0x14,0x93)
    GRAY       = (0x10,0x10,0x10)
    ULTRAWHITE = (0xFF,0xFF,0xFF, 0xFF)
    @staticmethod
    def dim(c):
        return (c[0]//4, c[1]//4, c[2]//4) if len(c) == 3 else (c[0]//4, c[1]//4, c[2]/4, c[3]//4) # color at 25% intensity
    @staticmethod
    def dark(c):
        return (c[0]//16, c[1]//16, c[2]//16) if len(c) == 3 else (c[0]//16, c[1]//16, c[2]/16, c[3]//16) # color at 25% intensity

    def __init__(self, pin, nb_leds):
        self.neo = self.neo = neopixel.NeoPixel(machine.Pin(pin), nb_leds, bpp=3)
        self.segments = []
        self.segment_runtimes = []
        self.segments.append(Segment(0, nb_leds - 1, self.mode_static, [(255,255,255), (0,0,0), (255,0,0)], 1000, Segment.SIZE_MEDIUM | Segment.FADE_XXSLOW))
        self.active_segments = copy.copy(self.segments)
        self.segment_runtimes.append(SegmentRuntime())
        self.seg = self.segments[0]
        self.seg_rt = self.segment_runtimes[0]
        self.running = False
        self.triggered = False

        self.modes = {}
        self.modes['mode_static'] = self.mode_static
        self.modes['mode_blink'] = self.mode_blink
        self.modes['mode_blink_rainbow'] = self.mode_blink_rainbow
        self.modes['mode_strobe'] = self.mode_strobe
        self.modes['mode_strobe_rainbow'] = self.mode_strobe_rainbow
        self.modes['mode_color_wipe'] = self.mode_color_wipe
        self.modes['mode_color_wipe_inv'] = self.mode_color_wipe_inv
        self.modes['mode_color_wipe_rev_inv'] = self.mode_color_wipe_rev_inv
        self.modes['mode_color_wipe_random'] = self.mode_color_wipe_random
        self.modes['mode_color_sweep_random'] = self.mode_color_sweep_random
        self.modes['mode_random_color'] = self.mode_random_color
        self.modes['mode_single_dynamic'] = self.mode_single_dynamic
        self.modes['mode_multi_dynamic'] = self.mode_multi_dynamic
        self.modes['mode_breath'] = self.mode_breath
        self.modes['mode_fade'] = self.mode_fade
        self.modes['mode_scan'] = self.mode_scan
        self.modes['mode_dual_scan'] = self.mode_dual_scan
        self.modes['mode_rainbow'] = self.mode_rainbow
        self.modes['mode_tricolor_chase'] = self.mode_tricolor_chase
        self.modes['mode_circus_combustus'] = self.mode_circus_combustus
        self.modes['mode_theater_chase'] = self.mode_theater_chase
        self.modes['mode_theater_chase_rainbow'] = self.mode_theater_chase_rainbow
        self.modes['mode_running_lights'] = self.mode_running_lights
        self.modes['mode_twinkle'] = self.mode_twinkle
        self.modes['mode_twinkle_random'] = self.mode_twinkle_random
        self.modes['mode_twinkle_fade_random'] = self.mode_twinkle_fade_random
        self.modes['mode_sparkle'] = self.mode_sparkle
        self.modes['mode_flash_sparkle'] = self.mode_flash_sparkle
        self.modes['mode_hyper_sparkle'] = self.mode_hyper_sparkle
        self.modes['mode_multi_strobe'] = self.mode_multi_strobe
        self.modes['mode_bicolor_chase'] = self.mode_bicolor_chase
        self.modes['mode_chase_color'] = self.mode_chase_color
        self.modes['mode_chase_blackout'] = self.mode_chase_blackout
        self.modes['mode_chase_white'] = self.mode_chase_white
        self.modes['mode_chase_random'] = self.mode_chase_random
        self.modes['mode_chase_rainbow_white'] = self.mode_chase_rainbow_white
        self.modes['mode_chase_rainbow'] = self.mode_chase_rainbow
        self.modes['mode_chase_blackout_rainbow'] = self.mode_chase_blackout_rainbow


    async def service(self):
        # while True:
        #     start = utime.ticks_ms()
        #     speed = self.mode()
        #     end = utime.ticks_ms()
        #     wait_time = int(max(min(WS2812FX.MAX_TIME, speed - (end-start)), 0))
        #     self.neo.write()
        #     self.seg_rt.counter_mode_call += 1
        #     await uasyncio.sleep_ms(wait_time)
        doShow = False
        while True:
            if self.running or self.triggered: 
                now = utime.ticks_ms()
                doShow = False
                for i in range(0, len(self.segments)): 
                    if self.segments[i].active:
                        self.seg = self.segments[i]
                        self.seg_rt  = self.segment_runtimes[i]
                        self.seg_rt.clr_frame_cycle()
                        if now > self.seg_rt.next_time or self.triggered:
                            self.seg_rt.set_frame()
                            doShow = True
                            delay = self.seg.mode()
                            self.seg_rt.next_time = now + max(delay, WS2812FX.SPEED_MIN)
                            self.seg_rt.counter_mode_call += 1
            if doShow:
                self.neo.write()
            self.triggered = False
            await uasyncio.sleep_ms(1)

    def start(self):
        self.reset_segment_runtimes()
        self.running = True

    def stop(self):
        self.running = False
        self.strip_off()

    def pause(self):
        self.running = False

    def resume(self):
        self.running = True

    def trigger(self):
        self.triggered = True

    def reset_segment_runtimes(self):
        for seg_rt in self.segment_runtimes:
            seg_rt.reset()

    def set_mode_strip(self, m):
        self.set_mode(0, m)

    def set_mode(self, seg, m):
        self.segment_runtimes[seg].reset()
        if isinstance(m, int):          
            self.segments[seg].mode = min(max(0, m), len(self.modes) - 1)
        elif isinstance(m, str):
            self.segments[seg].mode = self.modes[m]
        else:
            self.segments[seg].mode = m

    def set_options(self, seg, o):
        self.segments[seg].options = o

    def set_speed_strip(self, s):
        self.set_speed(0, s)

    def set_speed(self, seg, s):
        self.segments[seg].speed = min(max(WS2812FX.SPEED_MIN, s), WS2812FX.SPEED_MAX)

    def increase_speed(self, s):
        new_speed = min(max(WS2812FX.SPEED_MIN, self.seg.speed + s), WS2812FX.SPEED_MAX)
        self.set_speed_strip(new_speed)

    def decrease_speed(self, s):
        new_speed = min(max(WS2812FX.SPEED_MIN, self.seg.speed - s), WS2812FX.SPEED_MAX)
        self.set_speed(new_speed)

    def set_color_strip(self, r, g, b, w = 0):
        self.set_color(0, r, g, b, w)

    def set_color(self, seg, r, g, b, w = 0):
        self.segments[seg].colors[0] = (r, g, b, w) if self.neo.bpp ==4 else (r, g, b)

    def set_colors(self, seg, c):
        self.segments[seg].colors = copy.deepcopy(c)

    def is_running(self):
        return self.running

    def is_triggered(self):
        return self.triggered

    def is_frame(self, seg = 0):
        if not self.segments[seg].active:
            return False # segment not active
        return self.segment_runtimes[seg].aux_param2 & SegmentRuntime.FRAME

    def is_cycle(self, seg = 0):
        if not self.segments[seg].active:
            return False # segment not active
        return self.segment_runtimes[seg].aux_param2 & SegmentRuntime.CYCLE

    def set_cycle(self):
        self.seg_rt.set_cycle()

    def get_mode(self, seg):
        for mode_name, mode in self.modes.items():
            if mode == self.segments[seg].mode:
                return mode_name

    def get_speed(self, seg = 0):
        return self.segments[seg].speed

    def get_options(self, seg = 0):
        return self.segments[seg].options

    # parameters: index, start, stop, mode, color, speed, reverse
    def add_segment(self, start, stop, mode, colors, speed, options):
        self.segments.append(Segment(start, stop, mode, colors, speed, options)) 
        self.segment_runtimes.append(SegmentRuntime()) 

    def set_segment(self, n, start, stop, mode, colors, speed, options):
        if n >= len(self.segments):
            raise IndexError('list index out of range')

        self.segments[n].start = start
        self.segments[n].stop = stop
        self.segments[n].mode = mode
        self.segments[n].speed = speed
        self.segments[n].options = options
        self.set_colors(n, colors)

    #
    # Turns everything off. Doh.
    #
    def strip_off(self):
        self.fill()
        self.neo.write()

    def fill(self, c = 0,first = 0,count = 0):
        if c == 0:
            c = (0, 0, 0, 0) if self.neo.bpp == 4 else (0, 0, 0)
        if count == 0:
            count = self.neo.n - first
        for i in range(first, first + count):
            self.neo[i] = c

    def fill_seg(self, c = 0):
        self.fill(c, self.seg.start, self.seg.stop - self.seg.start + 1)


    #
    # Put a value 0 to 255 in to get a color value.
    # The colours are a transition r -> g -> b -> back to r
    # Inspired by the Adafruit examples.
    #
    def color_wheel(self, pos):
        pos = 255 - pos
        if pos < 85:
            return (255 - pos*  3, 0, pos * 3)
        elif pos < 170:
            pos -= 85
            return (0, pos * 3, 255 - pos *  3)
        else:
            pos -= 170
            return (pos * 3, 255 - pos * 3, 0)

    #
    # Returns a new, random wheel index with a minimum distance of 42 from pos.
    #
    def get_random_wheel_index(self, pos):
        r = 0
        x = 0
        y = 0
        d = 0

        while(d < 42):
            r = random.randrange(0,256)
            x = abs(pos - r)
            y = 255 - x
            d = min(x, y)

        return r

    # overload setPixelColor() functions so we can use gamma correction
    # (see https://learn.adafruit.com/led-tricks-gamma-correction/the-issue)
    def setPixelColor(self, n, *argv):
        if len(argv) < 1 and len(argv) > 4:
            raise TypeError("Wrong number of arguments")
        r=0
        g=0
        b=0
        w=0
        if len(argv) == 1:
            r = argv[0][0]
            g = argv[0][1]
            b = argv[0][2]
            w = argv[0][3] if self.neo.bpp == 4 else 0
        elif len(argv) >= 3:
            if len(argv) == 4:
                w = argv[3]
            r = argv[0]
            g = argv[1]
            b = argv[2]

        if(self.seg.is_gamma()):
            self.neo[n] = (gamma8[r], gamma8[g], gamma8[b], gamma8[w]) if self.neo.bpp ==4 else (gamma8[r], gamma8[g], gamma8[b])
        else:
            self.neo[n] = (r, g, b, w) if self.neo.bpp ==4 else (r, g, b)

    #
    # color blend function
    #
    def color_blend(self, color1, color2, blend):
        if(blend == 0):
            return color1
        if(blend == 255):
            return color2
        
        r1 = color1[0]
        g1 = color1[1]
        b1 = color1[2]
        w1 = color1[3] if self.neo.bpp == 4 else 0

        r2 = color2[0]
        g2 = color2[1]
        b2 = color2[2]
        w2 = color2[3] if self.neo.bpp == 4 else 0

        r3 = int(((r2 * blend) + (r1 * (255 - blend))) / 256)
        g3 = int(((g2 * blend) + (g1 * (255 - blend))) / 256)
        b3 = int(((b2 * blend) + (b1 * (255 - blend))) / 256)
        w3 = int(((w2 * blend) + (w1 * (255 - blend))) / 256 if self.neo.bpp == 4 else 0)

        return (r3, g3, b3, w3) if self.neo.bpp == 4 else (r3, g3, b3)

    ######################
    #       Modes        #
    ######################
    
    #
    # No blinking. Just plain old static light.
    #
    def mode_static(self):
        self.fill_seg(self.seg.colors[0])
        return self.seg.speed


    # Blink/strobe function
    # Alternate between color1 and color2
    # if(strobe == true) then create a strobe effect
    def blink(self, color1, color2, strobe):
        if(self.seg_rt.counter_mode_call & 1):
            color = color1 if self.seg.is_reverse() else color2 # off
            self.fill_seg(color)
            self.seg_rt.set_cycle()
            return self.seg.speed - 20 if strobe else (self.seg.speed / 2)
        else:
            color = color2 if self.seg.is_reverse() else color1 # on
            self.fill_seg(color)
            return 20 if strobe else (self.seg.speed / 2)

    #
    # Normal blinking. 50% on/off time.
    #
    def mode_blink(self):
        return self.blink(self.seg.colors[0], self.seg.colors[1], False)

    #
    # Classic Blink effect. Cycling through the rainbow.
    #
    def mode_blink_rainbow(self):
        return self.blink(self.color_wheel(self.seg_rt.counter_mode_call & 0xFF), self.seg.colors[1], False)

    #
    # Classic Strobe effect.
    #
    def mode_strobe(self):
        return self.blink(self.seg.colors[0], self.seg.colors[1], True)


    #
    # Classic Strobe effect. Cycling through the rainbow.
    #
    def mode_strobe_rainbow(self):
        return self.blink(self.color_wheel(self.seg_rt.counter_mode_call & 0xFF), self.seg.colors[1], True)

    #
    # Color wipe function
    # LEDs are turned on (color1) in sequence, then turned off (color2) in sequence.
    #* if (bool rev == true) then LEDs are turned off in reverse order
    #
    def color_wipe(self, color1, color2, rev):
        if(self.seg_rt.counter_mode_step < self.seg.len):
            led_offset = self.seg_rt.counter_mode_step
            if(self.seg.is_reverse()):
                self.setPixelColor(self.seg.stop - led_offset, color1)
            else:
                self.setPixelColor(self.seg.start + led_offset, color1)
        else:
            led_offset = self.seg_rt.counter_mode_step - self.seg.len
            if((self.seg.is_reverse() and not rev) or (not self.seg.is_reverse() and rev)):
                self.setPixelColor(self.seg.stop - led_offset, color2)
            else:
                self.setPixelColor(self.seg.start + led_offset, color2)

        self.seg_rt.counter_mode_step = (self.seg_rt.counter_mode_step + 1) % (self.seg.len * 2)

        if(self.seg_rt.counter_mode_step == 0):
            self.seg_rt.set_cycle()

        return (self.seg.speed / (self.seg.len * 2))

    #
    # Lights all LEDs one after another.
    #
    def mode_color_wipe(self):
        return self.color_wipe(self.seg.colors[0], self.seg.colors[1], False)

    def mode_color_wipe_inv(self):
        return self.color_wipe(self.seg.colors[1], self.seg.colors[0], False)

    def mode_color_wipe_rev(self):
        return self.color_wipe(self.seg.colors[0], self.seg.colors[1], True)

    def mode_color_wipe_rev_inv(self):
        return self.color_wipe(self.seg.colors[1], self.seg.colors[0], True)

    #
    # Turns all LEDs after each other to a random color.
    # Then starts over with another color.
    #
    def mode_color_wipe_random(self):
        if(self.seg_rt.counter_mode_step % self.seg.len == 0): # aux_param will store our random color wheel index
            self.seg_rt.aux_param = self.get_random_wheel_index(self.seg_rt.aux_param)

        color = self.color_wheel(self.seg_rt.aux_param)
        return self.color_wipe(color, color, False) * 2


    #
    # Random color introduced alternating from start and end of strip.
    #
    def mode_color_sweep_random(self):
        if(self.seg_rt.counter_mode_step % self.seg.len == 0): # aux_param will store our random color wheel index
            self.seg_rt.aux_param = self.get_random_wheel_index(self.seg_rt.aux_param)

        color = self.color_wheel(self.seg_rt.aux_param)
        return self.color_wipe(color, color, True) * 2

    #
    # Lights all LEDs in one random color up. Then switches them
    # to the next random color.
    #
    def mode_random_color(self):
        self.seg_rt.aux_param = self.get_random_wheel_index(self.seg_rt.aux_param) # aux_param will store our random color wheel index
        color = self.color_wheel(self.seg_rt.aux_param)
        self.fill(color, self.seg.start, self.seg.len)
        self.seg_rt.set_cycle()
        return self.seg.speed

    #
    # Lights every LED in a random color. Changes one random LED after the other
    # to another random color.
    #
    def mode_single_dynamic(self):
        if(self.seg_rt.counter_mode_call == 0):
            for i in range(self.seg.start, self.seg.stop+1):
                self.setPixelColor(i, self.color_wheel(random.randrange(0,256)))

        self.setPixelColor(self.seg.start + random.randrange(self.seg.len), self.color_wheel(random.randrange(0,256)))
        self.seg_rt.set_cycle()
        return self.seg.speed


    #
    # Lights every LED in a random color. Changes all LED at the same time
    # to new random colors.
    #
    def mode_multi_dynamic(self):
        for i in range(self.seg.start, self.seg.stop+1):
            self.setPixelColor(i, self.color_wheel(random.randrange(0,256)))

        self.seg_rt.set_cycle()
        return self.seg.speed

    #
    # Does the "standby-breathing" of well known i-Devices. Fixed Speed.
    # Use mode "fade" if you like to have something similar with a different speed.
    #
    def mode_breath(self):
        lum = self.seg_rt.counter_mode_step
        if(lum > 255): 
            lum = 511 - lum # lum = 15 -> 255 -> 15

        delay = 0
        if(lum == 15):
            delay = 970 # 970 pause before each breath
        elif(lum <=  25):
            delay = 38 # 19
        elif(lum <=  50):
            delay = 36 # 18
        elif(lum <=  75):
            delay = 28 # 14
        elif(lum <= 100):
            delay = 20 # 10
        elif(lum <= 125):
            delay = 14 # 7
        elif(lum <= 150):
            delay = 11 # 5
        else:
            delay = 10 # 4

        color =  self.color_blend(self.seg.colors[1], self.seg.colors[0], lum)
        self.fill(color, self.seg.start, self.seg.len)

        self.seg_rt.counter_mode_step += 2
        if self.seg_rt.counter_mode_step > (512-15):
            self.seg_rt.counter_mode_step = 15
            self.seg_rt.set_cycle()
        return delay


    #
    # Fades the LEDs between two colors
    #
    def mode_fade(self):
        lum = self.seg_rt.counter_mode_step
        if(lum > 255):
            lum = 511 - lum # lum = 0 -> 255 -> 0

        color = self.color_blend(self.seg.colors[1], self.seg.colors[0], lum)
        self.fill(color, self.seg.start, self.seg.len)

        self.seg_rt.counter_mode_step += 4
        if self.seg_rt.counter_mode_step > 511:
            self.seg_rt.counter_mode_step = 0
            self.seg_rt.set_cycle()
        return self.seg.speed / 128

    #
    # scan function - runs a block of pixels back and forth.
    #
    def scan(self, color1, color2, dual):
        dir = -1 if self.seg_rt.aux_param else 1
        size = 1 << self.seg.size_option()

        self.fill(color2, self.seg.start, self.seg.len)

        for i in range(0, size):
            if self.seg.is_reverse() or dual:
                self.setPixelColor(self.seg.stop - self.seg_rt.counter_mode_step - i, color1)

            if not self.seg.is_reverse() or dual:
                self.setPixelColor(self.seg.start + self.seg_rt.counter_mode_step + i, color1)

        self.seg_rt.counter_mode_step += dir
        if self.seg_rt.counter_mode_step == 0:
            self.seg_rt.aux_param = 0
            self.seg_rt.set_cycle()

        if self.seg_rt.counter_mode_step >= (self.seg.len - size):
            self.seg_rt.aux_param = 1

        return int(self.seg.speed / (self.seg.len * 2))


    #
    # Runs a block of pixels back and forth.
    #
    def mode_scan(self):
        return self.scan(self.seg.colors[0], self.seg.colors[1], False)


    #
    # Runs two blocks of pixels back and forth in opposite directions.
    #
    def mode_dual_scan(self):
        return self.scan(self.seg.colors[0], self.seg.colors[1], True)

    #
    # Cycles all LEDs at once through a rainbow.
    #
    def mode_rainbow(self):
        color = self.color_wheel(self.seg_rt.counter_mode_step)
        self.fill(color, self.seg.start, self.seg.len)

        self.seg_rt.counter_mode_step = (self.seg_rt.counter_mode_step + 1) & 0xFF

        if self.seg_rt.counter_mode_step == 0:
            self.seg_rt.set_cycle()

        return int(self.seg.speed / 256)

    #
    # Cycles a rainbow over the entire string of LEDs.
    #
    def mode_rainbow_cycle(self):
        for i in range(0, self.seg.len):
            color = self.color_wheel(((i * 256 // self.seg.len) + self.seg_rt.counter_mode_step) & 0xFF)
            self.setPixelColor(self.seg.start + i, color)

        self.seg_rt.counter_mode_step = (self.seg_rt.counter_mode_step + 1) & 0xFF

        if(self.seg_rt.counter_mode_step == 0):
            self.seg_rt.set_cycle()

        return int(self.seg.speed / 256)

    #
    # Tricolor chase function
    #
    def tricolor_chase(self, color1, color2, color3):
        sizeCnt = 1 << self.seg.size_option()
        sizeCnt2 = sizeCnt + sizeCnt
        sizeCnt3 = sizeCnt2 + sizeCnt
        index = self.seg_rt.counter_mode_step % sizeCnt3
        for i in range(0, self.seg.len):
            index = index % sizeCnt3
            index += 1

            color = color3
            if index < sizeCnt:
                color = color1
            elif index < sizeCnt2: 
                color = color2

            if self.seg.is_reverse():
                self.setPixelColor(self.seg.start + i, color)
            else:
                self.setPixelColor(self.seg.stop - i, color)

        self.seg_rt.counter_mode_step += 1
        if(self.seg_rt.counter_mode_step % self.seg.len == 0):
            self.seg_rt.set_cycle()

        return int(self.seg.speed / self.seg.len)


    #
    # Tricolor chase mode
    #
    def mode_tricolor_chase(self):
        return self.tricolor_chase(self.seg.colors[0], self.seg.colors[1], self.seg.colors[2])


    #
    # Alternating white/red/black pixels running.
    #
    def mode_circus_combustus(self):
        return self.tricolor_chase((255, 0, 0), (255, 255, 255), (0, 0, 0))


    #
    # Theatre-style crawling lights.
    # Inspired by the Adafruit examples.
    #
    def mode_theater_chase(self):
        return self.tricolor_chase(self.seg.colors[0], self.seg.colors[1], self.seg.colors[1])


    #
    # Theatre-style crawling lights with rainbow effect.
    # Inspired by the Adafruit examples.
    #
    def mode_theater_chase_rainbow(self):
        self.seg_rt.counter_mode_step = (self.seg_rt.counter_mode_step + 1) & 0xFF
        color = self.color_wheel(self.seg_rt.counter_mode_step)
        return self.tricolor_chase(color, self.seg.colors[1], self.seg.colors[1])

    #
    # Running lights effect with smooth sine transition.
    #
    def mode_running_lights(self):
        size = 1 << self.seg.size_option()
        sineIncr = max(1, (256 / self.seg.len) * size)
        for i in range (0, self.seg.len):
            lum = sine8[int((i + self.seg_rt.counter_mode_step) * sineIncr) % 256]
            color = self.color_blend(self.seg.colors[0], self.seg.colors[1], lum)
            if self.seg.is_reverse():
                self.setPixelColor(self.seg.start + i, color)
            else:
                self.setPixelColor(self.seg.stop - i,  color)
            
        self.seg_rt.counter_mode_step = (self.seg_rt.counter_mode_step + 1) % 256
        if self.seg_rt.counter_mode_step == 0:
            self.seg_rt.set_cycle()
        return int(self.seg.speed / self.seg.len)


    #
    # twinkle function
    #
    def twinkle(self, color1, color2):
        if self.seg_rt.counter_mode_step == 0:
            self.fill(color2, self.seg.start, self.seg.len)
            min_leds = int(self.seg.len / 4) + 1 # make sure, at least one LED is on
            self.seg_rt.counter_mode_step = random.randrange(min_leds, min_leds * 2)
            self.seg_rt.set_cycle()

        self.setPixelColor(self.seg.start + random.randrange(self.seg.len), color1)

        self.seg_rt.counter_mode_step -= 1
        return int(self.seg.speed / self.seg.len)

    #
    # Blink several LEDs on, reset, repeat.
    # Inspired by www.tweaking4all.com/hardware/arduino/arduino-led-strip-effects/
    #
    def mode_twinkle(self):
        return self.twinkle(self.seg.colors[0], self.seg.colors[1])

    #
    # Blink several LEDs in random colors on, reset, repeat.
    # Inspired by www.tweaking4all.com/hardware/arduino/arduino-led-strip-effects/
    #
    def mode_twinkle_random(self):
        return self.twinkle(self.color_wheel(random.randrange(0, 256)), self.seg.colors[1])

    #
    # fade out functions
    #
    rateMapH = [0, 1, 1, 1, 2, 3, 4, 6]
    rateMapL = [0, 2, 3, 8, 8, 8, 8, 8]
    def fade_out(self, *argv):
        targetColor = self.seg.colors[1]
        if len(argv) > 0:
            targetColor = argv[0]

        rate  = self.seg.fade_rate()
        rateH = WS2812FX.rateMapH[rate]
        rateL = WS2812FX.rateMapL[rate]

        color = targetColor
        r2 = color[0]
        g2 = color[1]
        b2 = color[2]
        w2 = color[3] if self.neo.bpp == 4 else 0

        for i in range(self.seg.start, self.seg.stop + 1):
            color = self.neo[i] # current color
            if rate == 0: # old fade-to-black algorithm
                self.setPixelColor(i, (color[0]>>1 & 0x7F, color[1]>>1 & 0x7F, color[2]>>1 & 0x7F, color[3]>>1 & 0x7F if self.neo.bpp == 4 else 0))
            else: # new fade-to-color algorithm
                r1 = color[0]
                g1 = color[1]
                b1 = color[2]
                w1 = color[3] if self.neo.bpp == 4 else 0

                # calculate the color differences between the current and target colors
                rdelta = r2 - r1
                gdelta = g2 - g1
                bdelta = b2 - b1
                wdelta = w2 - w1

                # if the current and target colors are almost the same, jump right to the target
                # color, otherwise calculate an intermediate color. (fixes rounding issues)
                rdelta = rdelta if abs(rdelta) < 3 else (rdelta >> rateH) + (rdelta >> rateL)
                gdelta = gdelta if abs(gdelta) < 3 else (gdelta >> rateH) + (gdelta >> rateL)
                bdelta = bdelta if abs(bdelta) < 3 else (bdelta >> rateH) + (bdelta >> rateL)
                wdelta = (wdelta if abs(wdelta) < 3 else (wdelta >> rateH) + (wdelta >> rateL)) if self.neo.bpp == 4 else 0

                self.setPixelColor(i, r1 + rdelta, g1 + gdelta, b1 + bdelta, w1 + wdelta)

    #
    # twinkle_fade function
    #
    def twinkle_fade(self, color):
        self.fade_out()

        if random.randrange(3) == 0:
            size = 1 << self.seg.size_option()
            index = self.seg.start + random.randrange(self.seg.len - size)
            self.fill(color, index, size)
            self.seg_rt.set_cycle()

        return int(self.seg.speed / 8)

    #
    # Blink several LEDs on, fading out.
    #
    def mode_twinkle_fade(self):
        return self.twinkle_fade(self.seg.colors[0])

    #
    # Blink several LEDs in random colors on, fading out.
    #
    def mode_twinkle_fade_random(self):
        return self.twinkle_fade(self.color_wheel(random.randrange(0,256)))

    #
    # Sparkle function
    # color1 = background color
    # color2 = sparkle color
    #
    def sparkle(self, color1, color2):
        if self.seg_rt.counter_mode_step == 0:
            self.fill(color1, self.seg.start, self.seg.len)

        size = 1 << self.seg.size_option()
        self.fill(color1, self.seg.start + self.seg_rt.aux_param3, size)

        self.seg_rt.aux_param3 = random.randrange(self.seg.len - size) # aux_param3 stores the random led index
        self.fill(color2, self.seg.start + self.seg_rt.aux_param3, size)

        self.seg_rt.set_cycle()
        return int(self.seg.speed / 32)


    #
    # Blinks one LED at a time.
    # Inspired by www.tweaking4all.com/hardware/arduino/arduino-led-strip-effects/
    #
    def mode_sparkle(self):
        return self.sparkle(self.seg.colors[1], self.seg.colors[0])

    #
    # Lights all LEDs in the color. Flashes white pixels randomly.
    # Inspired by www.tweaking4all.com/hardware/arduino/arduino-led-strip-effects/
    #
    def mode_flash_sparkle(self):
        return self.sparkle(self.seg.colors[0], WS2812FX.WHITE)

    #
    # Like flash sparkle. With more flash.
    # Inspired by www.tweaking4all.com/hardware/arduino/arduino-led-strip-effects/
    #
    def mode_hyper_sparkle(self):
        self.fill(self.seg.colors[0], self.seg.start, self.seg.len)

        size = 1 << self.seg.size_option()
        for _ in range(0, 8):
            self.fill(WS2812FX.WHITE, self.seg.start + random.randrange(self.seg.len - size), size)

        self.seg_rt.set_cycle()
        return int(self.seg.speed / 32)

    #
    # Strobe effect with different strobe count and pause, controlled by speed.
    #
    def mode_multi_strobe(self):
        self.fill(self.seg.colors[1], self.seg.start, self.seg.len)

        delay = 200 + ((9 - (self.seg.speed % 10)) * 100)
        count = int(2 * ((self.seg.speed / 100) + 1))
        if self.seg_rt.counter_mode_step < count:
            if (self.seg_rt.counter_mode_step & 1) == 0:
                self.fill(self.seg.colors[0], self.seg.start, self.seg.len)
                delay = 20
            else:
                delay = 50

        self.seg_rt.counter_mode_step = (self.seg_rt.counter_mode_step + 1) % (count + 1)
        if self.seg_rt.counter_mode_step == 0:
            self.seg_rt.set_cycle()
        return delay

    #
    # color chase function.
    # color1 = background color
    # color2 and color3 = colors of two adjacent leds
    #
    def chase(self, color1, color2, color3):
        size = 1 << self.seg.size_option()
        for i in range(0, size):
            a = (self.seg_rt.counter_mode_step + i) % self.seg.len
            b = (a + size) % self.seg.len
            c = (b + size) % self.seg.len
            if self.seg.is_reverse():
                self.setPixelColor(self.seg.stop - a, color1)
                self.setPixelColor(self.seg.stop - b, color2)
                self.setPixelColor(self.seg.stop - c, color3)
            else:
                self.setPixelColor(self.seg.start + a, color1)
                self.setPixelColor(self.seg.start + b, color2)
                self.setPixelColor(self.seg.start + c, color3)

        if self.seg_rt.counter_mode_step + (size * 3) == self.seg.len:
            self.seg_rt.set_cycle()

        self.seg_rt.counter_mode_step = (self.seg_rt.counter_mode_step + 1) % self.seg.len
        return int(self.seg.speed / self.seg.len)


    #
    # Bicolor chase mode
    #
    def mode_bicolor_chase(self):
        return self.chase(self.seg.colors[0], self.seg.colors[1], self.seg.colors[2])

    #
    # White running on _color.
    #
    def mode_chase_color(self):
        return self.chase(self.seg.colors[0], WS2812FX.WHITE, WS2812FX.WHITE)


    #
    # Black running on _color.
    #
    def mode_chase_blackout(self):
        return self.chase(self.seg.colors[0], WS2812FX.BLACK, WS2812FX.BLACK)

    #
    # _color running on white.
    #
    def mode_chase_white(self):
        return self.chase(WS2812FX.WHITE, self.seg.colors[0], self.seg.colors[0])

    #
    # White running followed by random color.
    #
    def mode_chase_random(self):
        if self.seg_rt.counter_mode_step == 0:
            self.seg_rt.aux_param = self.get_random_wheel_index(self.seg_rt.aux_param)
        return self.chase(self.color_wheel(self.seg_rt.aux_param), WS2812FX.WHITE, WS2812FX.WHITE)


    #
    # Rainbow running on white.
    #
    def mode_chase_rainbow_white(self):
        n = self.seg_rt.counter_mode_step
        m = (self.seg_rt.counter_mode_step + 1) % self.seg.len
        color2 = self.color_wheel(((n * 256 // self.seg.len) + (self.seg_rt.counter_mode_call & 0xFF)) & 0xFF)
        color3 = self.color_wheel(((m * 256 // self.seg.len) + (self.seg_rt.counter_mode_call & 0xFF)) & 0xFF)

        return self.chase(WS2812FX.WHITE, color2, color3)


    #
    # White running on rainbow.
    #
    def mode_chase_rainbow(self):
        color_sep = 256 // self.seg.len
        color_index = self.seg_rt.counter_mode_call & 0xFF
        color = self.color_wheel(((self.seg_rt.counter_mode_step * color_sep) + color_index) & 0xFF)

        return self.chase(color, WS2812FX.WHITE, WS2812FX.WHITE)

    #
    # Black running on rainbow.
    #
    def mode_chase_blackout_rainbow(self):
        color_sep = 256 // self.seg.len
        color_index = self.seg_rt.counter_mode_call & 0xFF
        color = self.color_wheel(((self.seg_rt.counter_mode_step * color_sep) + color_index) & 0xFF)

        return self.chase(color, WS2812FX.BLACK, WS2812FX.BLACK)
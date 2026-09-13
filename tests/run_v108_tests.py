"""Compile/run the actual v1.08 sketch against deterministic host hardware fakes.

Usage: python tests/run_v108_tests.py --compiler /path/to/zig.exe
The compiler may also be g++ or clang++. No Arduino hardware is accessed.
Builds in the OS temporary directory. Requires a C++17 compiler.
"""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
FAKE = r'''
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <cmath>
#include <array>
#include <stdexcept>
#include <algorithm>
#include <vector>
#include <string>
#include <cstdio>
using std::isfinite;
template<class A,class B> auto max(A a,B b) -> decltype(a+b) { return a>b?a:b; }
using byte = uint8_t;
#define PROGMEM
#define pgm_read_byte(p) (*(p))
#define pgm_read_ptr(p) (*(p))
#define F(x) x
#define HIGH true
#define LOW false
#define INPUT_PULLUP 2
#define RISING 3
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define U8G2_R0 0
#define U8G2_16BIT
#define _BV(x) (1U << (x))
#define PD3 3
#define PB4 4
#define PC0 0
#define PCINT4 4
#define PCINT19 3
#define PCIF0 0
#define PCIF2 2
#define PCIE0 0
#define PCIE2 2
#define ATOMIC_RESTORESTATE 0
#define ATOMIC_BLOCK(x) for (bool once = true; once; once = false)
#define ISR(x) void x()
#define constrain(x,a,b) ((x)<(a)?(a):((x)>(b)?(b):(x)))
uint8_t PIND=0, PINB=0, PINC=1, PCMSK0=0, PCMSK2=0, PCIFR=0, PCICR=0;
uint32_t clockUs=0, clockMs=0;
std::array<bool, 20> pinLevels;
uint32_t micros() { return clockUs; }
uint32_t millis() { return clockMs; }
bool digitalRead(uint8_t pin) { return pinLevels[pin]; }
void pinMode(uint8_t, uint8_t) {}
uint8_t digitalPinToInterrupt(uint8_t pin) { return pin; }
void attachInterrupt(uint8_t, void (*)(), uint8_t) {}
char* ultoa(uint32_t value,char* text,int) { sprintf(text,"%lu",(unsigned long)value); return text; }
char* dtostrf(double value,signed char,unsigned char precision,char* text) {
  sprintf(text,"%.*f",(int)precision,value); return text;
}
// Representative metrics for control/render contract tests, not real bitmaps.
const uint8_t u8g2_font_5x7_tr[]={7,5}, u8g2_font_luBS12_tn[]={17,12};
const uint8_t u8g2_font_luBS08_tn[]={11,7}, u8g2_font_logisoso16_tn[]={16,10};
const uint8_t u8g2_font_logisoso18_tn[]={18,11}, u8g2_font_logisoso20_tn[]={20,12};
const uint8_t u8g2_font_logisoso22_tn[]={22,13}, u8g2_font_logisoso24_tn[]={24,14};
const uint8_t u8g2_font_logisoso26_tn[]={26,15}, u8g2_font_logisoso28_tn[]={28,16};
const uint8_t u8g2_font_logisoso32_tn[]={32,19}, u8g2_font_logisoso38_tn[]={38,22};
const uint8_t u8g2_font_logisoso42_tn[]={42,24};
struct FakeFault : std::runtime_error { FakeFault():runtime_error("FRAM ERROR"){} };
struct PowerCut {};
class U8G2_SH1122_256X64_1_4W_HW_SPI {
 public:
  struct Glyph { uint16_t x,y; char digit; const uint8_t* font; };
  std::vector<Glyph> glyphs;
  std::string labels;
  const uint8_t* selected=u8g2_font_5x7_tr;
  U8G2_SH1122_256X64_1_4W_HW_SPI(int,int,int,int) {}
  void begin() {}
  void firstPage() {}
  bool nextPage() { return false; }
  void setFont(const uint8_t* font) { selected=font; }
  void setFontPosBaseline() {}
  void setFontRefHeightAll() {}
  int8_t getAscent() { return selected[0]; }
  int8_t getDescent() { return 0; }
  uint16_t getStrWidth(const char* value) { return strlen(value)*selected[1]; }
  uint16_t drawGlyph(uint16_t x,uint16_t y,uint16_t ch) {
    glyphs.push_back({x,y,(char)ch,selected}); return selected[1];
  }
  void setCursor(int,int) {}
  template<class T> void print(T) {}
  void print(const char* value) {
    if (!strcmp(value,"FRAM ERROR")) throw FakeFault();
    labels+=value;
  }
  void print(float,int) {}
};
class SwitecX25 {
 public:
  bool stopped=true;
  uint16_t target=0;
  SwitecX25(int,int,int,int,int) {}
  void update() {}
  void setPosition(uint16_t value) { target=value; }
  void zero() {}
};
class Adafruit_FRAM_I2C {
 public:
  std::array<uint8_t,512> memory{};
  int writesBeforeCut=-1;
  bool begin(uint8_t) { return true; }
  uint8_t read(uint16_t address) { return memory.at(address); }
  void write(uint16_t address,uint8_t value) {
    if (writesBeforeCut == 0) throw PowerCut();
    if (writesBeforeCut > 0) --writesBeforeCut;
    memory.at(address)=value;
  }
};
'''

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='stepper-v108-tests-') as directory:
        build = Path(directory)
        (build / 'util').mkdir()
        (build / 'avr').mkdir()
        (build / 'Arduino.h').write_text(FAKE)
        for name in ['SPI.h', 'Wire.h', 'Adafruit_FRAM_I2C.h', 'SwitecX25.h',
                     'U8g2lib.h', 'util/atomic.h', 'avr/interrupt.h']:
            (build / name).write_text('#include "Arduino.h"\n')
        shutil.copyfile(ROOT / 'stepper_speedometer_v1_08.ino', build / 'firmware.ino')
        shutil.copyfile(ROOT / 'tests/v108_logic.cpp', build / 'test.cpp')
        executable = build / 'test.exe'
        command = [args.compiler]
        if Path(args.compiler).stem == 'zig':
            command.append('c++')
        command += ['-std=c++17', '-D__AVR_ATmega328P__', '-I', str(build),
                    str(build / 'test.cpp'), '-o', str(executable)]
        result = subprocess.run(command, text=True, capture_output=True)
        if result.returncode:
            print(result.stdout[-8000:])
            print(result.stderr[-8000:])
            result.check_returncode()
        subprocess.run([str(executable)], check=True)

if __name__ == '__main__':
    main()


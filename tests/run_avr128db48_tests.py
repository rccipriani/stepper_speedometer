"""Run the AVR DB application with fake GPIO/SPI/I2C and the real Switec driver.

python tests/run_avr128db48_tests.py --compiler PATH_TO_ZIG_OR_GXX --switec-source PATH
"""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile
import re
from run_v112_tests import FAKE

ROOT = Path(__file__).resolve().parents[1]

WIRE = r'''
#define TWI_TIMEOUT_ENABLE
struct FakeWire {
  std::array<uint8_t,512> memory{};
  std::vector<uint8_t> tx;
  uint16_t pointer=0;
  bool present=true, shortRead=false;
  int writesBeforeCut=-1, writesBeforeFailure=-1;
  unsigned transactions=0;
  bool pins(uint8_t,uint8_t) { return true; }
  void begin() {}
  void beginTransmission(uint8_t address) { assertAddress=address; tx.clear(); ++transactions; }
  uint8_t assertAddress=0;
  void write(uint8_t value) { tx.push_back(value); }
  uint8_t endTransmission(bool=true) {
    if (!present || assertAddress!=0x50) return 2;
    if (tx.size()>=2) pointer=(uint16_t(tx[0])<<8)|tx[1];
    if (tx.size()==3) {
      if (writesBeforeCut==0) throw 42;
      if (writesBeforeFailure==0) { present=false; return 3; }
      if (writesBeforeCut>0) --writesBeforeCut;
      if (writesBeforeFailure>0) --writesBeforeFailure;
      memory.at(pointer)=tx[2];
    }
    return 0;
  }
  uint8_t requestFrom(uint8_t,uint8_t n) { return present&&!shortRead?n:0; }
  int read() { return memory.at(pointer); }
} Wire;
struct FakeSPI { bool pins(uint8_t,uint8_t,uint8_t,uint8_t) { return true; } } SPI;
'''

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', required=True)
    parser.add_argument('--switec-source', type=Path, required=True)
    parser.add_argument('--debug-serial', action='store_true')
    args = parser.parse_args()
    # Compare the actual on-disk schemas and distance math to the legacy source.
    legacy = (ROOT/'stepper_speedometer_v1_12.ino').read_text()
    schemas = (ROOT/'firmware/avr128db48/StorageLayout.h').read_text()
    for name in ['StoredState', 'DisplayLayout']:
        pattern = rf'struct __attribute__\(\(packed\)\) {name} \{{.*?\n\}};'
        assert re.search(pattern, legacy, re.S).group() == re.search(pattern, schemas, re.S).group()
    distance = (ROOT/'firmware/avr128db48/Distance.h').read_text()
    pattern = r'void addDistance\(uint32_t pulses\) \{.*?\n\}'
    assert re.search(pattern, legacy, re.S).group() == re.search(pattern, distance, re.S).group()
    fake = FAKE.replace('std::array<bool, 20>', 'std::array<bool, 48>')
    fake = fake.replace('std::array<uint16_t,22>', 'std::array<uint16_t,48>')
    a, b = fake.index('struct FakeWire'), fake.index('char* ultoa')
    fake = fake[:a] + WIRE + fake[b:]
    fake = fake[:fake.index('class SwitecX25')]
    fake = fake.replace('    if (!strcmp(value,"FRAM ERROR")) throw FakeFault();', '')
    fake += '\nusing boolean = bool;\n#define OUTPUT 1\n#define VDD 0\n'
    fake += 'void digitalWrite(uint8_t p,bool v) { pinLevels.at(p)=v; }\n'
    fake += 'void delayMicroseconds(uint32_t) { throw std::runtime_error("blocking delay"); }\n'
    fake += 'uint8_t adcReference=255, adcResolution=0;\n'
    fake += 'void analogReference(uint8_t v) { adcReference=v; }\nvoid analogReadResolution(uint8_t v) { adcResolution=v; }\n'
    fake += 'struct FakeSerial { unsigned baud=0; void begin(unsigned b) { baud=b; } } Serial3;\n'
    for port, base, count in [('A',0,8),('B',8,6),('C',14,8),('D',22,8),('E',30,4),('F',34,7)]:
        for n in range(count):
            fake += f'#define PIN_P{port}{n} {base+n}\n'
    with tempfile.TemporaryDirectory(prefix='avrdb-tests-') as directory:
        build=Path(directory)
        for f in (ROOT/'firmware/avr128db48').iterdir():
            if f.suffix in ('.cpp','.h'): shutil.copyfile(f,build/f.name)
        (build/'Arduino.h').write_text(fake)
        for name in ['SPI.h','Wire.h','U8g2lib.h','util/atomic.h','avr/interrupt.h','avr/wdt.h','avr/cpufunc.h']:
            f=build/name; f.parent.mkdir(exist_ok=True); f.write_text('#include "Arduino.h"\n')
        shutil.copyfile(args.switec_source/'SwitecX25.h', build/'SwitecX25.h')
        # Compile as one TU because the shared fake deliberately owns its globals.
        (build/'SwitecImplementation.h').write_text((args.switec_source/'SwitecX25.cpp').read_text())
        shutil.copyfile(ROOT/'tests/avr128db48_logic.cpp',build/'test.cpp')
        command=[args.compiler]
        if Path(args.compiler).stem=='zig': command+=['c++']
        if args.debug_serial: command+=['-DDEBUG_SERIAL=1']
        executable=build/'test.exe'
        command+=['-std=c++17','-D__AVR_AVR128DB48__','-I',str(build),str(build/'test.cpp'),'-o',str(executable)]
        subprocess.run(command,check=True)
        subprocess.run([str(executable)],check=True)

if __name__=='__main__': main()

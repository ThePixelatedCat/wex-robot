# WeX Robot

## TODO

- defmt over usb for Pico
- build runner that auto-enters bootloader

## TODO

### Urgent

- Get Bluetooth control working (Pico)
  - ~~Dependency management (bump embassy versions)~~
  - ~~GATT server/ client~~
  - ~~Pairing~~
  - Fix bug where GATT disconnects after ~1 minute
- ~~Build Bluetooth controller (PC)~~
- Tidy into library
- Create template repository
- Test on WeX Laptop

### Still Urgent But Less So

- Mentor documentation (pre-reading)
- Student documentation (installation)
- Install dependencies on WeX laptops
  - Rustup (could get the students to do this)
  - VSCode/ Zed
- Pick up wheels

### Non-blockers

- Battery Level
- USB deferred formatting
- Enter boot-loader from USB (custom runner w/ defmt over USB)
- Document robot library
- Find a unique identifier to use during pairing?

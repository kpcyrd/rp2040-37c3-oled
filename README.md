# rp2040-37c3-oled

```
rustup target add thumbv6m-none-eabi
cargo build --release
elf2uf2-rs -d target/thumbv6m-none-eabi/release/rp2040-37c3-oled
```

## Bill of materials

- rp2040
- ssd1306
- some wire

## Pins

There are 4 pins that need to be connected:

- `GPIO14` - data (sda)
- `GPIO15` - clock (scl)
- `3V3` - VDD (power, 3.3V or 5V are both fine)
- `GND` - GND (ground)

![](https://www.waveshare.com/img/devkit/RP2040-Zero/RP2040-Zero-details-7.jpg)

[Archive](https://web.archive.org/web/20241228234716if_/https://www.waveshare.com/img/devkit/RP2040-Zero/RP2040-Zero-details-7.jpg)

## Regenerate frames

```
convert frame1.png -monochrome -negate frame1.pbm
# use a hex editor, search for start of null bytes
tail -c +11 frame1.pbm > frame1.raw
```

# rp2040-37c3-oled

```
rustup target add thumbv6m-none-eabi
cargo build --release
elf2uf2-rs -d
```

## Regenerate frames

```
convert frame1.png -monochrome -negate frame1.pbm
# use a hex editor, search for start of null bytes
tail -c +11 frame1.pbm > frame1.raw
```

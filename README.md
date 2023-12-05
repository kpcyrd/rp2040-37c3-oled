
```
convert frame1.png -monochrome -negate frame1.pbm
# use a hex editor, search for start of null bytes
tail -c +11 frame1.pbm > frame1.raw
```


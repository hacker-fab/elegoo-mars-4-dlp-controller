# Usage

how to copy files over the network
```
scp my_image.png hf@litho:elegoo-mars-4-dlp-controller/images
```

how to connect remotely to the pi
```
ssh hf@litho
```

how to run an exposure
```
PYTHONPATH=src python3 src/cli.py images/my_image.png --brightness 100 --exposure-frames 300 -x 100 -y 50
```
- first argument is the path to the image
- `--brightness` is the brightness to expose the image at, from 0% -> 100%
- `--exposure-frames` is the number of frames to display the image for, at 60fps. So to convert to seconds do # frames / 60.
    - example: 300 frames = 5 seconds
- `-x` and `-y` are the offsets of the top left corner of the image

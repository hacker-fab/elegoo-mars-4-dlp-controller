from PIL import Image
import numpy as np


def image_to_arr(path):
    print("-- Converting image to bytes --")
    im_frame = Image.open(path)
    # convert to 8bit grayscale
    if im_frame.mode != 'L':
          im_frame = im_frame.convert('L')
    pixeldata = np.transpose(np.array(im_frame))
    assert pixeldata.dtype == np.uint8, f"Expected uint8, got: {pixeldata.dtype}"

    return pixeldata

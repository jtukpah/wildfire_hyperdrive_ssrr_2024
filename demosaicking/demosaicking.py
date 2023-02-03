
import os
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt

def readraw(filename,bitdepth,width,height):
    """
    Read RAW frames into numpy array
    :param filename: filename of input image
    :param bitdepth: bitdepth of image (8, 16 or 32)
    :param width: width of input image
    :param height: height of input image
    :return: floating point image data
    """
    options_bitdepth = {8 : np.uint8, 16 : np.uint16, 32 : np.uint32}

    with open(filename, 'rb') as fid:
        if os.stat(filename).st_size < (height*width*bitdepth/8):
            data = np.zeros(shape = (height, width))
        else:
            data = np.fromfile(fid, options_bitdepth[bitdepth]).reshape((height,width)).astype(float)

    return data

def filter_box(im,radius):
    """
    Box filter with running average O(1)
    :param im: input image
    :param radius: radius of box kernel
    :return: box-filtered image
    """
    (rows, cols) = im.shape[:2]
    z = np.zeros_like(im)

    tile = [1] * im.ndim
    tile[0] = radius
    t = np.cumsum(im, 0)
    z[0:radius + 1, :, ...] = t[radius:2 * radius + 1, :, ...]
    z[radius + 1:rows - radius, :, ...] = t[2 * radius + 1:rows, :, ...] - t[0:rows - 2 * radius - 1, :, ...]
    z[rows - radius:rows, :, ...] = np.tile(t[rows - 1:rows, :, ...], tile) - t[rows - 2 * radius - 1:rows - radius - 1, :, ...]

    tile = [1] * im.ndim
    tile[1] = radius
    t = np.cumsum(z, 1)
    z[:, 0:radius + 1, ...] = t[:, radius:2 * radius + 1, ...]
    z[:, radius + 1:cols - radius, ...] = t[:, 2 * radius + 1: cols, ...] - t[:, 0: cols - 2 * radius - 1, ...]
    z[:, cols - radius: cols, ...] = np.tile(t[:, cols - 1:cols, ...], tile) - t[:, cols - 2 * radius - 1: cols - radius - 1, ...]

    return z

def filter_guided_gray(im,guide,radius,smooth):
    """
    Guided filter for grayscale images
    :param im: input image
    :param guide: guidance image
    :param radius: window radius parameter
    :param smooth: regularization or smooth parameter
    :return: guided-filtered image
    """

    (rows, cols) = guide.shape
    N = filter_box(np.ones([rows, cols]), radius)

    meanI = filter_box(guide, radius) / N
    meanP = filter_box(im, radius) / N
    corrI = filter_box(guide * guide, radius) / N
    corrIp = filter_box(guide * im, radius) / N
    varI = corrI - meanI * meanI
    covIp = corrIp - meanI * meanP

    a = covIp / (varI + smooth)
    b = meanP - a * meanI

    meanA = filter_box(a, radius) / N
    meanB = filter_box(b, radius) / N

    z = meanA * guide + meanB

    return z

def filter_guided(im,guide,radius,smooth):
    """
    Guided filter for multichannel images
    :param im: input image
    :param guide: guidance image
    :param radius: window radius parameter
    :param smooth: regularization or smooth parameter
    :return: guided-filtered image
    """

    (rows, cols) = guide.shape
    N = filter_box(np.ones([rows, cols]), radius)
    (rows, cols, channels) = im.shape
    N2 = filter_box(np.ones([rows, cols, channels]), radius)

    meanI = filter_box(guide, radius) / N
    meanI2 = np.repeat(meanI[:, :, np.newaxis], channels, axis=2)
    meanP = filter_box(im, radius) / N2
    corrI = filter_box(guide * guide, radius) / N
    guide2 = np.repeat(guide[:, :, np.newaxis], channels, axis=2)
    corrIp = filter_box(guide2 * im, radius) / N2
    varI = corrI - meanI * meanI
    covIp = corrIp - meanI2 * meanP

    varI2 = varI + smooth
    varI2 = np.repeat(varI2[:, :, np.newaxis], channels, axis=2)

    a = covIp / varI2
    b = meanP - a * meanI2

    meanA = filter_box(a, radius) / N2
    meanB = filter_box(b, radius) / N2

    z = meanA * guide2 + meanB

    return z

def filter_shift(im,offset):
    """
    Shift image with offset
    :param im: input image
    :param offset: translation shift
    :return: shifted image
    """

    rows, cols = im.shape
    M = np.float32([[1, 0, offset[1]], [0, 1, offset[0]]])
    z = cv.warpAffine(im, M, (cols, rows), cv.INTER_LANCZOS4)

    return z

def im2cube_sinc(im_raw, pattern, height, width):
    """
    Interpolated demosaicking (Lanczos windowed sinc)
    :param im_raw: raw input image
    :param pattern: integer dimension of the mosaic pattern (e.g. 4 or 5)
    :param height: height of valid image region
    :param width: width of valid image region
    :return:
    """
    cube = np.zeros([height, width, pattern ** 2])

    band = 0
    for i in range(0, pattern):
        for j in range(0, pattern):
            im_sub = im_raw[i:(height + 1):pattern, j:(width + 1):pattern]
            im_sub = cv.resize(im_sub, (width, height), interpolation = cv.INTER_LANCZOS4)

            offset = [i / pattern, j / pattern]
            im_sub = filter_shift(im_sub, offset)

            cube[:, :, band] = im_sub[:height, :width]
            band += 1

    im_guide = cube[:, :, 0]

    return cube

def im2cube_sinc_guided(im_raw, pattern, height, width):
    """
    Interpolated demosaicking with guided filter
    :param im_raw: raw input image
    :param pattern: integer dimension of the mosaic pattern (e.g. 4 or 5)
    :param height: height of valid image region
    :param width: width of valid image region
    :return:
    """
    cube = im2cube_sinc(im_raw, pattern, height, width)
    im_guide = cube[:, :, 0]
    cube = filter_guided(cube, im_guide, 3, 0.01)

    return cube

filename = r'.\data\image_001784.raw'

bitdepth = 16
width = 2048
height = 1088
pattern = 4

data = readraw(filename, bitdepth, width, height)

cube_sinc = im2cube_sinc(data, pattern, height, width)
cube_sinc_guided = im2cube_sinc_guided(data, pattern, height, width)

# visualization
band = 10 # band number

plt.figure(0)
plt.imshow(data)
plt.title("RAW data")

plt.figure(1)
plt.imshow(cube_sinc[:, :, band])
plt.title("Sinc interpolation")

plt.figure(2)
plt.imshow(cube_sinc_guided[:, :, band])
plt.title("Sinc interpolation + classical guided filter")


plt.show()
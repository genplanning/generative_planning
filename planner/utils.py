import cv2 as cv


# imshow with wait key implemented on 'esc'
def imshow_wait(img, name="img"):
    cv.imshow(name, img)
    while cv.waitKey(0) != 27:
        pass


# resize image
def resize_image(image, scale_percent=100):
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)
    dim = (width, height)
    image = cv.resize(image, dim, interpolation=cv.INTER_AREA)

    return image

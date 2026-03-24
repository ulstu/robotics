import cv2


def scale_image(cv_image, scale: int):
    width = int(cv_image.shape[1] * scale / 100)
    height = int(cv_image.shape[0] * scale / 100)
    dim = (width, height)

    return cv2.resize(cv_image, dim, interpolation=cv2.INTER_AREA)


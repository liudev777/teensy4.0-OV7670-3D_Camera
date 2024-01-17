import numpy as np
import cv2

with open("output/img.txt") as file:
    data = file.read()

data = bytes.fromhex(data)

with open("output/img.raw", "wb") as file:
    file.write(data)

# Convert to 3 channel uint8 numpy array representing the BGR image
img = cv2.cvtColor(
    np.frombuffer(data, dtype=np.uint8).reshape(240, 320, 2), cv2.COLOR_BGR5652RGB
)

cv2.imwrite("output/image.png", img)

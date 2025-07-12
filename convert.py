import numpy as np
import cv2

with open("output/img.txt") as file:
    data = file.read()

data = bytes.fromhex(data)

with open("output/img.raw", "wb") as file:
    file.write(data)

buffer = np.frombuffer(data, dtype=np.uint8).reshape(240, 320, 2).copy()

# Convert to 3 channel uint8 numpy array representing the BGR image
img = cv2.cvtColor(
    buffer, cv2.COLOR_BGR5652BGR
)


cv2.imwrite("output/image0.png", img)

# test - YBGR
# E0FF1F00E00700F8
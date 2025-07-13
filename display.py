import numpy as np
import cv2
import serial

ser = serial.Serial("COM6", 6000000)

correction = np.array([[1.36, -0.3, -0.06], 
                        [-0.20, 1.32, -0.12], 
                        [-0.04, -0.55, 1.59]])

while (True):
    data = ser.read_until(b"FRAME")
    data = ser.read(153600)
    print(len(data))

    # buf = np.frombuffer(data, dtype=np.uint8)
    # s = 153600 // 2
    # print(buf[s:s+8])
    # print(buf[s+8:s+16])
    # print(U.min(), U.max(), V.min(), V.max())

    img = cv2.cvtColor(
        np.frombuffer(data, dtype=np.uint8).reshape(240, 320, 2), cv2.COLOR_YUV2RGB_UYVY 
    )
    img = img @ correction.T

    cv2.imwrite("output/test.png", img)
    print("WRITE")
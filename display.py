import numpy as np
import cv2
import serial
import os
from dotenv import load_dotenv

load_dotenv()

SERIAL_NUM=os.getenv("SERIAL_NUM")
ser = serial.Serial(SERIAL_NUM, 115200)

while (True):
    data = ser.read_until(b"FRAME")
    data = ser.read(153600)
    img = cv2.cvtColor(
        np.frombuffer(data, dtype=np.uint8).reshape(240, 320, 2), cv2.COLOR_YUV2BGR_UYVY 
    )[:,:,[2,0,1]]

    cv2.imwrite("output/test.png", img)
    print("WRITE")
import time
import numpy as np
import cv2
import serial
import os
from dotenv import load_dotenv

load_dotenv()

SERIAL_NUM=os.getenv("SERIAL_NUM")
ser = serial.Serial(SERIAL_NUM, 6000000)

while (True):
    data = ser.read_until(b"FRAME")
    flag = ser.read(1)          # read shutter trigger
    data = ser.read(153600)     # read image bytes
    img = cv2.cvtColor(
        np.frombuffer(data, dtype=np.uint8).reshape(240, 320, 2), cv2.COLOR_YUV2BGR_UYVY 
    )[:,:,[2,0,1]]
    
    # save image to output folder
    if flag == b'\x01':
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"output/snapshot_{timestamp}.png"
        cv2.imwrite(filename, img)
        print(f"Screenshot saved: {filename}")

    cv2.imwrite("output/test.png", img)
    print(flag)
    print("WRITE")
    
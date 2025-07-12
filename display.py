import numpy as np
import cv2
import serial

ser = serial.Serial("COM6", 115200)

while (True):
    # data = ser.readline().strip().decode("utf-8")
    # if ser.in_waiting > 0:

    ser.reset_input_buffer()
    data = ser.read(307200).strip().decode("utf-8")
    print("RX")
    print([data[:16] + "..." + data[-16:]])
    print(len(data))

    if len(data) == 307200:
        try:
            data = bytes.fromhex(data)
            img = cv2.cvtColor(
                np.frombuffer(data, dtype=np.uint8).reshape(240, 320, 2), cv2.COLOR_YUV2BGR_UYVY 
            )[:,:,[2,0,1]]

            cv2.imwrite("output/test.png", img)
            print("WRITE")
        except ValueError:
            print(ValueError)
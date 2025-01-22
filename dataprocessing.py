from tkinter import W


data = ""

with open(r"C:\Users\aarti\Swirling pendulum LQR stabilisation -2022-05-07 15-47-37.531477.csv") as file:
     data = file.read()
     headers = data[0:51]
     print(headers)

print(type(data))
# with open(r"C:\Users\aarti\Swirling pendulum LQR stabilisation -2022-05-07 15-47-37.531477.csv") as file:
#      file.write(data)
# MLX90640.py-
A simple python driver for the MLX90640 loosely based on the original c++ driver from Melexis.

I wrote the driver for a school project with the raspberry Pi and did not implement everything, it is similar to the c++ driver. It uses smbus2  as i2c bus library but i think it is relatively easy to modify it to work with other librarys too. 

Example:
I included a demo program which captures "images" one rgb with the Pi camera and a Thermal image (a numpy array) and stores them in seperate folders. It is not very good commented but i think the general ideas come across.

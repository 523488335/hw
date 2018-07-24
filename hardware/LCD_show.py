from PIL import Image
import time
import Adafruit_ILI9341 as TFT
import Adafruit_GPIO.SPI as SPI
import RPi.GPIO as GPIO
from hardware import globalvar as gl


def LCD():
    DC = 18
    RST = 23
    SPI_PORT = 0
    SPI_DEVICE = 0
    GPIO.setmode(GPIO.BCM)
    # Create TFT LCD display class.
    disp = TFT.ILI9341(DC, rst=RST, spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE, max_speed_hz=64000000))

    # Initialize display.
    disp.begin()

    # Load an image.

    image = ['01.jpg', '02.jpg', '03.jpg', '04.jpg', '05.jpg', '06.jpg',
             '07.jpg', '08.jpg', '09.jpg', '10.jpg', '11.jpg', '12.jpg']

    # image_01 = Image.open(image[0]).rotate(90).resize((240, 320))

    image_show = [Image.open('pictures/'+image[i]).rotate(90).resize((240, 320)) for i in range(len(image))]

    while True:
        picture = gl.get_value('picture')
        if picture == 0:
            disp.display(image_show[0])
            time.sleep(3)
            disp.display(image_show[1])
            
        elif picture == 3:
            disp.display(image_show[2])
        elif picture == 4:
            disp.display(image_show[3])
        elif picture == 5:
            disp.display(image_show[4])
        elif picture == 6:
            disp.display(image_show[5])
        elif picture == 7:
            disp.display(image_show[6])
        elif picture == 8:
            disp.display(image_show[7])
        elif picture == 9:
            disp.display(image_show[8])
        elif picture == 10:
            disp.display(image_show[9])
        elif picture == 11:
            disp.display(image_show[10])
        elif picture == 12:
            disp.display(image_show[11])
        else:
            continue

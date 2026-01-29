from time import sleep

from digit_interface import Digit
from digit_interface.digit_handler import DigitHandler

print(DigitHandler().list_digits())
d = Digit("D21193", "test")
# d = Digit("D21182", "test")
d.connect()
# d.set_intensity(15)
# sleep(1)
# d.save_frame("D21193.png")

try:
    d.show_view()
except KeyboardInterrupt:
    pass

# d.save_frame("D21182.png")
f = d.get_frame()
d.show_view(f)

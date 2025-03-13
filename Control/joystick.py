from pyPS4Controller.controller import Controller


class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)

    def on_x_press(self):
       print("X pressed")

    def on_x_release(self):
       print("X released")

    def on_R3_up(self, val):
        print(val)

    def on_R3_down(self, val):
        print(val)

    def on_R3_right(self, val):
        print(val)

    def on_R3_left(self, val):
        print(val)

    def on_R3_x_at_rest(self):
        print("R3 x at rest")

    def on_R3_y_at_rest(self):
        print("R3 y at rest")

controller = MyController(interface="/dev/input/js1", connecting_using_ds4drv=False)
# you can start listening before controller is paired, as long as you pair it within the timeout window
controller.listen(timeout=60)
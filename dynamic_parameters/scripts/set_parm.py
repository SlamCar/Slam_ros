#!/usr/bin/env python3

import remi.gui as gui
from remi import start, App


class MyApp(App):

    def __init__(self, *args):
        App.__init__(self, *args)

    def main(self):
        self._container = gui.VBox()

        # returning the root widget
        return self._container

# starts the webserver
start(MyApp,
      username='slamCar',
      password='123',
      multiple_instance=False,
      start_browser=False,
      debug=False)

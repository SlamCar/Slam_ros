#!/usr/bin/env python3

import remi.gui as gui
from remi import start, App


class MyApp(App):

    def __init__(self, *args):
        App.__init__(self, *args)

    def main(self):
        container = gui.VBox(width = 120, height = 100)
        self.lbl = gui.Label('Hello world!')
        self.bt = gui.Button('Press me!')

        # setting the listener for the onclick event of the button
        self.bt.onclick.connect(self.on_button_pressed)
        
        # appending a widget to another, the first argument is a string key
        container.append(self.lbl)
        container.append(self.bt)
        
        # returning the root widget
        return self._container
    
    
    # listener function
    def on_button_pressed(self, widget):
        self.lbl.set_text('Button pressed!')
        self.bt.set_text('Hi!')

# starts the webserver
start(MyApp,
      username='slamCar',
      password='123',
      address='127.0.0.1', 
      port=8081,
      multiple_instance=False,
      start_browser=False,
      debug=False)

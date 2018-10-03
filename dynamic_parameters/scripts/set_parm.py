#!/usr/bin/env python3

import remi.gui as gui
from remi import start, App


class MyApp(App):

    def __init__(self, *args):
        App.__init__(self, *args)

    def main(self):
        container = gui.VBox(width = 120, height = 100)
        self.lbl = gui.Label('Hello world!')
        self.bt = gui.Button('Hello name!')
        self.bt2 = gui.Button('Hello name surname!')


        # setting the listener for the onclick event of the button
        self.bt.onclick.connect(self.on_button_pressed, "Name")
        self.bt2.onclick.connect(self.on_button_pressed, "Name", "Surname")
        
        # appending a widget to another, the first argument is a string key
        container.append(self.lbl)
        container.append(self.bt)
        container.append(self.bt2)
        
        # returning the root widget
        return container
    
    
    # listener function
    def on_button_pressed(self, widget, name='', surname=''):
        self.lbl.set_text('Button pressed!')
        widget.set_text('Hello ' + name + ' ' + surname)

# starts the webserver
start(MyApp,
    #   username='slamCar',
    #   password='123',
      address='127.0.0.1', 
      port=8081,
      multiple_instance=False,
      start_browser=True,
      debug=False)

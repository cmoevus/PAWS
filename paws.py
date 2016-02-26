#!/usr/bin/python2
# -*- coding: utf-8 -*-
"""
Programmable Alternating/Waiting Shutter with graphical interface.

@author: Corentin Moevus cjm2206@columbia.edu
"""
from __future__ import absolute_import, division, unicode_literals, print_function
import time
import os
import pickle
import threading
from Queue import PriorityQueue
from math import ceil
import numpy as np
from functools import partial
import Phidgets
from Phidgets.Devices.Stepper import Stepper
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GLib


class TimeKeeper(threading.Thread):
    """
    Do tasks at given time, with a time precision given by "precision".

    The precision is actually the time the loop will wait before looking again into its task list to see if it is time for the next event. Hence, the lower the precision, the lower the resource consumption, and vice versa.

    Running slow functions in TimeKeeper loop can cause delays in the precise execution of the following events, as the TimeKeeper will wait until the function is done before looking for things to do. Consider threading or multiprocessing heavy functions.

    Arguments:
        max_precision <float>: time (in seconds) to wait before checking for actions to do. Defaults to 0.5s.
    """

    def __init__(self, precision=0.5):
        """Start the object and the thread."""
        threading.Thread.__init__(self)
        self.precision = precision
        self.queue = PriorityQueue()
        self.daemon = True
        self.kill = threading.Event()
        self.start()

    def run(self):
        """Check for things to do."""
        while self.kill.wait(self.precision) != True:
            # Is there something to do?
            if self.queue.empty() != True:
                # Is that thing to be done now?
                get = True
                while get == True:
                    action = self.queue.get()
                    # Yes, do it.
                    if action[0] <= time.time():
                        if action[2] is None:
                            action[1]()
                        elif type(action[2]) == dict:
                            action[1](**action[2])
                        else:
                            action[1](*action[2])

                    # No, put it back in the list and wait for next round
                    else:
                        self.queue.put(action)
                        get = False

    def add(self, t, func, args=None):
        """
        Add a thing to do.

        Arguments:
            t <float>: time in seconds since the epoch (like the output of time.time()) at which to do the action. If time is lesser than time.time() - 10, it will be added to current time.time(), so that if you want something to be done in 10seconds, you can simply enter time=10.
            func <callable>: function to call when time.time() == time
            args <list, tuple, dict>: arguments to pass to the function when executing it.
        """
        if t < time.time() - 10:
            t += time.time()
        self.queue.put([t, func, args])

    def stop(self):
        """Kill the TimeKeeper."""
        self.kill.set()

    __del__ = stop


class Shutter(threading.Thread):
    """
    Control one physical motor as a shutter.

    Arguments:
        hardware: Opened and attached Stepper() object (for concision, a Hardware instance)
        id: ID of the motor as indicated on the Phidgets board (0 to 3)
        step <float>: angle, in degrees, of one of the motor step (given by motor manufacturer)
        angle <float>: the number of degrees to move when switching the motor's position
        direction <bool>: direction of the movement. True or False may mean up or down, depending on the motor's wiring and orientation. You need to test it to know it.
        state <bool>: Current state of the motor. True if it is closed (blocks the beam) and False if it is open (let the beam through). Defaults to True.
        gui <Gtk.Switch>: Associated GUI object to record state changes
    """

    def __init__(self, hardware, ID, step, angle, direction, state=True, gui=None):
        """Activate the hardware."""
        # A. Initiate the thread
        threading.Thread.__init__(self)

        # B. Configure the motor
        self.hardware, self.id, self.step, self.angle, self.direction, self.state, self.gui = hardware, ID, step, angle, direction, state, gui
        self.activate = threading.Event()

        # C. Set a kill switch
        self.kill = threading.Event()

        # D. Define the waiting times for the running loop.
        self.kill_wait = 0.5
        self.activate_wait = 2
        self.moving_wait = 0.1

    def run(self):
        """Wait for a switching event or death."""
        while self.kill.wait(self.kill_wait) != True:
            if self.activate.wait(self.activate_wait) is True:
                self.switch()

    def switch(self):
        """Switch the position of the motor."""
        # A. Define the target position
        target = int(round(self.angle / self.step * 2, 0))
        if self.state != self.direction:
            target *= -1
        p = self.hardware.getCurrentPosition(self.id)
        target = p + target

        # B. Go there
        self.hardware.setEngaged(self.id, True)
        self.hardware.setTargetPosition(self.id, target)
        while p != target:
            time.sleep(self.moving_wait)
            p = self.hardware.getCurrentPosition(self.id)
        self.hardware.setEngaged(self.id, False)

        # C. Confirm movement
        self.state = not self.state
        self.activate.clear()

        # D. Update GUI
        if self.gui is not None:
            GLib.idle_add(self.gui.set_active, self.state)

    def set_state(self, state):
        """Set the shutter in the given state (True = Closed / False = Opened)."""
        if (state == True or state == False) and self.state != state:
            self.switch()

    def get_settings(self):
        """Return all the shutter's pickleable setup parameters in one dict, for convenience."""
        return {'step': self.step,
                'angle': self.angle,
                'direction': self.direction,
                'state': self.state}

    def stop(self):
        """Stop the thread."""
        self.kill.set()


class Hardware(Stepper):
    """Activate the Phidgets controller board (1062_1)."""

    def __init__(self):
        """Activate the hardware."""
        Stepper.__init__(self)
        self.openPhidget()
        self.waitForAttach(1000)

    def __del__(self):
        """Stop the hardware."""
        self.closePhidget()

    stop = __del__


class GUI(Gtk.Builder):
    """Graphical User Interface for interacting with a PAWS."""

    #
    # GUI related functions
    #

    def __init__(self, paws, gui_file="GUI.glade", *args):
        """Initiate the parent objects and do basic definitions."""
        # Connect the GUI to handlers
        Gtk.Builder.__init__(self)
        self.add_from_file(gui_file)
        self.connect_signals(self)

        # Connect to PAWS
        self.paws = paws

        # Times per second the progress is updated
        self.update_frequency = 0.1

    def show_settings_if_need_be(self):
        """Decide whether to show the Settings panel at start."""
        if self.paws.shutters is None:
            self.get_object('settings_window').show_all()

    def start(self):
        """Start the GUI."""
        # No Hardware, no GUI.
        if self.paws.hardware is None:
            window = self.get_object('no_hardware')
            window.show_all()
        else:
            window = self.get_object('paws')
            window.show_all()
            self.update_gui_settings()
            self.show_settings_if_need_be()
            self.load_shuttering_parameters()

        Gtk.main()

    def close(self, *args):
        """Stop the GUI."""
        self.save_shuttering_parameters()
        self.paws.gui = None
        Gtk.main_quit(*args)

    #
    # Shuttering related methods
    #

    def get_shuttering_parameters(self):
        """Return the shuttering parameters from the GUI."""
        loops = self.get_object("loops").get_value()
        loops_units = self.get_object("loops_units").get_active()
        wait = self.get_object("wait").get_value()
        wait_units = self.get_object("wait_units").get_active()
        return loops, loops_units, wait, wait_units

    def get_converted_shuttering_parameters(self):
        """Get the shuttering parameters from the GUI and return them in loops and seconds for the PAWS.shutter() function."""
        loops, loops_u, wait, wait_u = self.get_shuttering_parameters()
        convert = {0: 1, 1: 60, 2: 60**2}
        wait = wait * convert[wait_u]
        if loops_u != 0:
            loops = ceil(loops * convert[loops_u - 1] / wait)
        loops = int(loops)
        return loops, wait

    def start_shutter(self, *args):
        """Start shuttering via GUI."""
        self.get_object("shutter_stop").show()
        self.get_object("shutter_back").hide()
        self.show_shutter_progress()
        self.loops, self.wait = self.get_converted_shuttering_parameters()
        self.paws.shutter(self.loops, self.wait)

    def stop_shutter(self, *args):
        """Stop shuttering via GUI."""
        self.paws.stop_shutter()
        self.loops = None
        self.wait = None
        self.get_object("shutter_stop").hide()
        self.get_object("shutter_back").show()

    def show_shutter_parameters(self, *args):
        """Go to the shuttering parameters panel."""
        self.get_object("shutter").set_visible_child_name('parameters')

    def show_shutter_progress(self, *args):
        """Go to the shuttering progress panel."""
        self.get_object("shutter").set_visible_child_name('progress')

    def update_shuttering_progress(self, loop, t):
        """Update the progress bars for shuttering."""
        # Adjust text.
        loops_text = "Alternation {0}/{1}".format(loop, self.loops)
        wait_text = "{0} seconds before next alternation".format(self.wait - t)

        # Shuttering is done?
        if t == self.wait and loop == self.loops:
            wait_text = "Done."
            loops_text = "Done. ({0}/{1})".format(loop, self.loops)
            GLib.idle_add(self.get_object("shutter_stop").hide)
            GLib.idle_add(self.get_object("shutter_back").show)

        # Send all of it to the GUI (we're it the TimeKeeper thread, here.)
        GLib.idle_add(self.get_object('progress_loops').set_fraction, loop / self.loops)
        GLib.idle_add(self.get_object('progress_loops').set_text, loops_text)
        GLib.idle_add(self.get_object('progress_wait').set_fraction, t / self.wait)
        GLib.idle_add(self.get_object('progress_wait').set_text, wait_text)

    def save_shuttering_parameters(self):
        """Save the shuttering parameters in the PAWS object."""
        if 'gui_settings' not in vars(self.paws):
            self.paws.gui_settings = dict()
        self.paws.gui_settings['shuttering'] = self.get_shuttering_parameters()

    def load_shuttering_parameters(self):
        """Load shuttering parameters saved in the PAWS object."""
        if 'gui_settings' in vars(self.paws) and 'shuttering' in self.paws.gui_settings.keys():
            loops, loops_u, wait, wait_u = self.paws.gui_settings['shuttering']
            self.get_object('loops').set_value(loops)
            self.get_object('loops_units').set_active(loops_u)
            self.get_object('wait').set_value(wait)
            self.get_object('wait_units').set_active(wait_u)

    #
    # Settings related methods
    #

    def show_settings(self, *args):
        """Open the settings window."""
        self.get_object('settings_window').show()

    def close_settings(self, *args):
        """Close the settings window."""
        self.get_object('settings_window').hide()

    def get_shutter_settings_widgets(self, i):
        """Return widgets for shutter <i>."""
        settings = self.get_object("settings_shutters")
        widgets = {
            'active': settings.get_child_at(i + 1, 1),
            'step': settings.get_child_at(i + 1, 2),
            'angle': settings.get_child_at(i + 1, 3),
            'direction': settings.get_child_at(i + 1, 4),
            'state': settings.get_child_at(i + 1, 5),
        }
        return widgets

    def switch_shutter_settings_widgets(self, *args):
        """Hide/Show the settings for the given shutter."""
        active = args[0].get_active()
        widgets = self.get_shutter_settings_widgets(self.get_object("settings_shutters").child_get_property(args[0], 'left-attach') - 1)
        for k, v in widgets.items():
            if k != 'active':
                if active:
                    v.show()
                else:
                    v.hide()

    def update_paws_settings(self, *args):
        """Update the settings in PAWS using the settings in the GUI."""
        # Update Shutters settings
        shutters = dict()
        if self.paws.hardware is not None:
            for i in range(self.paws.hardware.getMotorCount()):
                widgets = self.get_shutter_settings_widgets(i)
                if widgets['active'].get_active() == True:
                    shutters[i] = dict()
                    shutters[i]['step'] = widgets['step'].get_value()
                    shutters[i]['angle'] = widgets['angle'].get_value()
                    shutters[i]['direction'] = widgets['direction'].get_active()
                    shutters[i]['state'] = widgets['state'].get_active()
        if len(shutters) == 0:
            shutters = None
        self.paws.setup_shutters(shutters)

        # Update switches
        self.pair_shutter_switches()

        # Update general settings
        self.update_frequency = self.get_object('update_frequency').get_value()
        self.paws.todo.precision = self.get_object('precision').get_value()

        # Put the GUI-specific settings in the PAWS object for saving them.
        self.paws.gui_settings = {
            'update_frequency': self.update_frequency,
        }

    def update_gui_settings(self, *args):
        """Update the settings in the GUI using the settings from PAWS."""
        # Update Shutters settings
        if self.paws.shutters is not None:
            for i in range(self.paws.hardware.getMotorCount()):
                widgets = self.get_shutter_settings_widgets(i)
                if i in self.paws.shutters.keys():
                    shutter = self.paws.shutters[i]
                    widgets['step'].set_value(shutter.step)
                    widgets['angle'].set_value(shutter.angle)
                    widgets['direction'].set_active(shutter.direction)
                    widgets['state'].set_active(shutter.state)
                else:
                    widgets['active'].set_active(False)

        # Update switches
        self.pair_shutter_switches()

        # Load the GUI-specific settings from the PAWS object
        if 'gui_settings' in vars(self.paws):
            self.update_frequency = self.paws.gui_settings['update_frequency']

        # Update the general settings
        self.get_object('update_frequency').set_value(self.update_frequency)
        self.get_object('precision').set_value(self.paws.todo.precision)

    #
    # Shutter switches related methods
    #

    def get_shutter_switch(self, i):
        """Return widgets for shutter <i>."""
        return self.get_object("switches").get_child_at(i, 0).get_children()[1]

    def get_shutter_switchbox(self, i):
        """Return the Box containing Switch and Label of shutter <i>."""
        return self.get_object("switches").get_child_at(i, 0)

    def switch_shutter_state(self, *args):
        """Switch the physical state of a shutter from a GUI call."""
        for s in self.paws.shutters.values():
            if s.gui == args[0]:
                s.switch()

    def pair_shutter_switches(self):
        """Update the switches to match current settings and associate the GUI to the shutters."""
        if self.paws.shutters is not None:
            self.get_object('switches').show()
            self.get_object('no_shutters').hide()
            for i in range(self.paws.hardware.getMotorCount()):
                if i in self.paws.shutters.keys():
                    shutter = self.paws.shutters[i]
                    switch = self.get_shutter_switch(i)

                    # Switch visibility
                    self.get_shutter_switchbox(i).show()
                    switch.set_active(shutter.state)

                    # Add GUI to the shutter
                    if shutter.gui is None:
                        shutter.gui = switch

                    # Sync state
                    switch.set_state(shutter.state)
                else:
                    self.get_shutter_switchbox(i).hide()
        else:
            self.get_object('switches').hide()
            self.get_object('no_shutters').show()


class PAWS(object):
    """
    Programmable Alternating/Waiting Shutter.

    Arguments:
    config <string>: path to the configuration file. If False, creates a new configuration.
    shutters <dict>: configuration for the shutters:
        id: {step, angle, direction, state}
        See the doc from Shutter for explanation of each parameter.
    precision <float>: see TimeKeeper
    """

    def __init__(self, settings=None, shutters=None, precision=0.5):
        """Initiate a PAWS."""
        # Start the TimeKeeper
        self.todo = TimeKeeper(precision)

        # There are no shutters nor settings, yet.
        self.shutters = None
        self.settings = None

        # Try to start the hardware
        try:
            self.hardware = Hardware()
        except Phidgets.PhidgetException.PhidgetException:
            self.hardware = None

        # There is no GUI, yet
        self.gui = None

        # Load a configuration
        if settings is not None:
            self.settings = settings
            if os.path.isfile(settings):
                self.load_settings(settings)
        elif shutters is not None:
            self.setup_shutters(shutters)

    def start_gui(self):
        """Start the GUI for controlling PAWS."""
        self.gui = GUI(paws=self)
        self.gui.start()

    def setup_shutters(self, shutters):
        """Setup shutters."""
        # Kill old shutter threads
        if self.shutters is not None:
            for s in self.shutters.values():
                s.stop()

        # Setup new shutters threads
        if shutters is not None:
            s = dict()
            for k, i in shutters.items():
                s[k] = Shutter(self.hardware, k, **i)
                s[k].start()
        else:
            s = None
        self.shutters = s

    def shutter(self, loops, wait):
        """Alternate the shutters <loops> times, waiting <wait> seconds before each alternation."""
        t = time.time()
        for i in range(0, loops):
            # Switch shutters
            for shutter in self.shutters.values():
                self.todo.add(t + (i + 1) * wait, shutter.activate.set)

            # Update progress on GUI
            if self.gui is not None:
                for j in np.arange(0, wait, self.gui.update_frequency):
                    self.todo.add(t + i * wait + j, self.gui.update_shuttering_progress, (i, j))

        # Mark the end of shuttering on GUI
        if self.gui is not None:
            self.todo.add(t + loops * wait, self.gui.update_shuttering_progress, (loops, wait))

    def stop_shutter(self, *args):
        """Stop the shutter."""
        self.todo.stop()
        self.todo = TimeKeeper(self.todo.precision)

    def load_settings(self, f):
        """Load settings from a file."""
        # Open file
        with open(f, 'r') as load:
            settings = pickle.load(load)

        # Load settings
        self.setup_shutters(settings['shutters'])
        self.todo.precision = settings['precision']
        if 'gui' in settings.keys():
            self.gui_settings = settings['gui']

    def save_settings(self, f=None):
        """Save settings to a file, self.settings if the file is None."""
        # Get a proper file
        f = self.settings if f is None else f

        # Get the latest GUI settings
        if self.gui is not None:
            self.gui.update_paws_settings()

        # Gather the settings
        settings = dict()
        if self.shutters is not None:
            settings['shutters'] = dict([(i, s.get_settings()) for i, s in self.shutters.items()])
        else:
            settings['shutters'] = None
        settings['precision'] = self.todo.precision
        if 'gui_settings' in vars(self):
            settings['gui'] = self.gui_settings

        # Write them in a file
        with open(f, 'w') as save:
            pickle.dump(settings, save)

    def __del__(self):
        """Kill the object clean."""
        self.todo.stop()
        if self.shutters is not None:
            for s in self.shutters.values():
                s.stop()
        if self.hardware is not None:
            self.hardware.stop()
        if self.gui is not None:
            self.gui.close()
        if self.settings is not None:
            self.save_settings()

    close = __del__

if __name__ == "__main__":
    shutters = {
        0: {'step': 1.8, 'angle': 30, 'direction': False, 'state': True},
        1: {'step': 1.8, 'angle': 30, 'direction': False, 'state': True}
    }
    # P = PAWS(shutters=shutters, precision=0.1)
    # P.settings = 'paws.conf'
    P = PAWS(settings="paws.conf")
    P.start_gui()
    P.close()

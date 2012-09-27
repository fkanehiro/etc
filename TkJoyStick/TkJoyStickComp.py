#!/usr/bin/env python
# -*- Python -*-

import sys
import time
sys.path.append(".")

# Import RTM module
import OpenRTM_aist
import RTC

import math

# This module's spesification
# <rtc-template block="module_spec">
tkjoystick_spec = ["implementation_id", "TkJoyStick", 
		   "type_name",         "TkJoyStick", 
		   "description",       "joystick component", 
		   "version",           "1.0", 
		   "vendor",            "AIST", 
		   "category",          "example", 
		   "activity_type",     "DataFlowComponent", 
		   "max_instance",      "10", 
		   "language",          "Python", 
		   "lang_type",         "SCRIPT",
		   ""]
# </rtc-template>

import tkjoystick

class TkJoyStick(OpenRTM_aist.DataFlowComponentBase):
	def __init__(self, manager):
		OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

		self._d_axes = RTC.TimedFloatSeq(RTC.Time(0,0),[])
		self._axesOut = OpenRTM_aist.OutPort("Axes", self._d_axes)
		self._d_buttons = RTC.TimedBooleanSeq(RTC.Time(0,0),[])
		self._buttonsOut = OpenRTM_aist.OutPort("Buttons", self._d_buttons)


		self.x = 0.0
		self.y = 0.0
		self.buttons = [False, False, False, False]

		 
	def onInitialize(self):
		# Bind variables and configuration variable

		# Set OutPort buffers
		self.registerOutPort("Axes",self._axesOut)
		self.registerOutPort("Buttons",self._buttonsOut)

		
		return RTC.RTC_OK


	def onShutdown(self, ec_id):
		return RTC.RTC_OK


	def onExecute(self, ec_id):
		self._d_axes.data = [self.x, self.y]
		self._axesOut.write()
		self._d_buttons.data = self.buttons
		self._buttonsOut.write()
		
		return RTC.RTC_OK

	def set_pos(self, pos, buttons):
		self.x = pos[0]
		self.y = pos[1]
		self.buttons = buttons

#def MyModuleInit(manager):
#    profile = OpenRTM_aist.Properties(defaults_str=tkjoystick_spec)
#    manager.registerFactory(profile,
#                            TkJoyStick,
#                            OpenRTM_aist.Delete)
#
#    # Create a component
#    comp = manager.createComponent("TkJoyStick")



def main():
	tkJoyCanvas = tkjoystick.TkJoystick()
	tkJoyCanvas.master.title("TkJoystick")
	mgr = OpenRTM_aist.Manager.init(sys.argv)
	mgr.activateManager()

	# Register component
	profile = OpenRTM_aist.Properties(defaults_str=tkjoystick_spec)
	mgr.registerFactory(profile,
			    TkJoyStick,
			    OpenRTM_aist.Delete)
	# Create a component
	comp = mgr.createComponent("TkJoyStick?instance_name=Joystick0")

	tkJoyCanvas.set_on_update(comp.set_pos)
	mgr.runManager(True)
	tkJoyCanvas.mainloop()

if __name__ == "__main__":
	main()


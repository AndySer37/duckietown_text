#!/usr/bin/env python
# license removed for brevity
import rospy
#from transitions import Machine
import transitions
from text_navigation.srv import *
from transitions.extensions import GraphMachine as Machine
import pygraphviz
class topomap(object):
	states = ['1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13', '14', '15', '16', '17', '18', '19', '20', '21']
	actions = ['N', 'E', 'S', 'W'] 
	def __init__(self, init, initial_heading):
		self.machine = Machine(model=self, states=topomap.states, initial=init)
		self.last_state = init
		self.current_heading = initial_heading
		self.machine.add_transition(trigger=self.actions[0], source='1', dest = '4')

		self.machine.add_transition(trigger=self.actions[1], source='1', dest = '5')
		self.machine.add_transition(trigger=self.actions[2], source='1', dest = '2')


		self.machine.add_transition(trigger=self.actions[0], source='2', dest = '3')

		self.machine.add_transition(trigger=self.actions[1], source='2', dest = '1')
		self.machine.add_transition(trigger=self.actions[2], source='2', dest = '10')


		self.machine.add_transition(trigger=self.actions[0], source='3', dest = '8')
		self.machine.add_transition(trigger=self.actions[3], source='3', dest = '2')
		self.machine.add_transition(trigger=self.actions[1], source='3', dest = '4')
		self.machine.add_transition(trigger=self.actions[2], source='3', dest = '9')

		self.machine.add_transition(trigger=self.actions[0], source='4', dest = '7')
		self.machine.add_transition(trigger=self.actions[3], source='4', dest = '1')
		self.machine.add_transition(trigger=self.actions[1], source='4', dest = '5')
		self.machine.add_transition(trigger=self.actions[2], source='4', dest = '3')

		self.machine.add_transition(trigger=self.actions[0], source='5', dest = '6')
		self.machine.add_transition(trigger=self.actions[3], source='5', dest = '1')

		self.machine.add_transition(trigger=self.actions[2], source='5', dest = '4')

		self.machine.add_transition(trigger=self.actions[0], source='6', dest = '11')
		self.machine.add_transition(trigger=self.actions[3], source='6', dest = '5')

		self.machine.add_transition(trigger=self.actions[2], source='6', dest = '7')

		self.machine.add_transition(trigger=self.actions[0], source='7', dest = '12')
		self.machine.add_transition(trigger=self.actions[3], source='7', dest = '4')
		self.machine.add_transition(trigger=self.actions[1], source='7', dest = '6')
		self.machine.add_transition(trigger=self.actions[2], source='7', dest = '8')

		self.machine.add_transition(trigger=self.actions[3], source='8', dest = '3')
		self.machine.add_transition(trigger=self.actions[1], source='8', dest = '7')
		self.machine.add_transition(trigger=self.actions[2], source='8', dest = '9')

		self.machine.add_transition(trigger=self.actions[0], source='9', dest = '10')
		self.machine.add_transition(trigger=self.actions[3], source='9', dest = '3')
		self.machine.add_transition(trigger=self.actions[1], source='9', dest = '8')

		self.machine.add_transition(trigger=self.actions[0], source='10', dest = '13')
		self.machine.add_transition(trigger=self.actions[3], source='10', dest = '2')
		self.machine.add_transition(trigger=self.actions[1], source='10', dest = '9')

		self.machine.add_transition(trigger=self.actions[0], source='11', dest = '16')
		self.machine.add_transition(trigger=self.actions[3], source='11', dest = '6')

		self.machine.add_transition(trigger=self.actions[2], source='11', dest = '12')

		self.machine.add_transition(trigger=self.actions[3], source='12', dest = '7')
		self.machine.add_transition(trigger=self.actions[1], source='12', dest = '11')
		self.machine.add_transition(trigger=self.actions[2], source='12', dest = '14')

		self.machine.add_transition(trigger=self.actions[0], source='13', dest = '18')
		self.machine.add_transition(trigger=self.actions[3], source='13', dest = '10')
		self.machine.add_transition(trigger=self.actions[1], source='13', dest = '14')

		self.machine.add_transition(trigger=self.actions[0], source='14', dest = '18')
		self.machine.add_transition(trigger=self.actions[3], source='14', dest = '12')
		self.machine.add_transition(trigger=self.actions[1], source='14', dest = '17')
		self.machine.add_transition(trigger=self.actions[2], source='14', dest = '13')

		self.machine.add_transition(trigger=self.actions[0], source='15', dest = '19')
		self.machine.add_transition(trigger=self.actions[3], source='15', dest = '15')
		self.machine.add_transition(trigger=self.actions[1], source='15', dest = '15')

		self.machine.add_transition(trigger=self.actions[0], source='16', dest = '19')
		self.machine.add_transition(trigger=self.actions[3], source='16', dest = '11')

		self.machine.add_transition(trigger=self.actions[2], source='16', dest = '17')

		self.machine.add_transition(trigger=self.actions[3], source='17', dest = '14')
		self.machine.add_transition(trigger=self.actions[1], source='17', dest = '16')
		self.machine.add_transition(trigger=self.actions[2], source='17', dest = '18')

		self.machine.add_transition(trigger=self.actions[3], source='18', dest = '14')
		self.machine.add_transition(trigger=self.actions[1], source='18', dest = '17')
		self.machine.add_transition(trigger=self.actions[2], source='18', dest = '13')

		self.machine.add_transition(trigger=self.actions[0], source='19', dest = '20')
		self.machine.add_transition(trigger=self.actions[3], source='19', dest = '16')
		self.machine.add_transition(trigger=self.actions[1], source='19', dest = '15')
		self.machine.add_transition(trigger=self.actions[2], source='19', dest = '21')

		self.machine.add_transition(trigger=self.actions[0], source='20', dest = '21')
		self.machine.add_transition(trigger=self.actions[3], source='20', dest = '19')

		self.machine.add_transition(trigger=self.actions[2], source='20', dest = '21')

		self.machine.add_transition(trigger=self.actions[0], source='21', dest = '20')
		self.machine.add_transition(trigger=self.actions[3], source='21', dest = '19')
		self.machine.add_transition(trigger=self.actions[1], source='21', dest = '20')

	def service_callback(self, req):
		action_str = "INIT"
		action_str = req.action
		head = actions.index(self.current_heading)
		turn_arr = ['L', 'F', 'R']
		turn = turn_arr.index(action_str)-1
		absolute_dir = (head + turn)%4
		try:
			if absolute_dir == 0:
				self.N();
			elif absolute_dir == 1:
				self.E();
			elif absolute_dir == 2:
				self.S();
			elif absolute_dir == 3:
				self.W();
			print "current state: ", self.machine.get_state(self.state).name
			print "last_state: ", self.last_state
			self.last_state = self.machine.get_state(self.state).name
			#m.get_graph().draw('my_state_diagram.png', prog='dot')
			return self.machine.get_state(self.state).name
		except transitions.core.MachineError as e:
			print e


if __name__ == '__main__':
	rospy.init_node('topo_map_node')
	initial_node = '1'
	initial_heading = 'W'
	m = topomap(initial_node, initial_heading)
	s = rospy.Service('topo_map_action', actions, m.service_callback)

	print "initial state", initial_state
	rospy.spin()


 





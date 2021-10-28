from time import process_time_ns
import Manipulatorkinematics as MK

arm = MK.Manipulator()
arm.setDH_parameters(125,360,120,170,80,-67,-67,0)
pos, orient = arm.FWDKine(300, 0, 0, 0)
print(pos)
print(orient)
joint1, joint2, joint3, joint4 = arm.InvKine(pos, orient)
print(joint1)
print(joint2)
print(joint3)
print(joint4)

import ManipulatorKinematics as MK

arm = MK.Manipulator()
arm.setDH_parameters(10,10,10,10,0,-10,-10,-10)
Position, Orientation = arm.FWDKine(100, 50, 50, 50)
print(Position)
print(Orientation)

Joint1, Joint2, Joint3, Joint4 = arm.InvKine(Position, Orientation)
print(Joint1, Joint2, Joint3, Joint4)
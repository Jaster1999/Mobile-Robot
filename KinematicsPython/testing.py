import ManipulatorKinematics as MK

arm = MK.Manipulator()
# init with link length parameters
arm.setDH_parameters(10,10,10,10,0,-10,-10,-10)
# Obtain Position and oreintation from joint angles and distances
Position, Orientation = arm.FWDKine(100, 50, 50, 50)
print("Position[X,Y,Z]")
print(Position)
print("Orientation[[I],[J],[K]]")
print(Orientation)
# Pass a position vector [X, Y, Z] and orientation array [[I],[J],[K]]
Joint1, Joint2, Joint3, Joint4 = arm.InvKine(Position, Orientation)
print("Joint values")
print(Joint1, Joint2, Joint3, Joint4)

#convert invkine (-180º to 180º) angle to a servo angle (0º to 180º)
print("Servo 2 Move to: "+str(arm.AngleValidator(Joint2))+"º")
print("Servo 3 Move to: "+str(arm.AngleValidator(Joint3))+"º")
print("Servo 4 Move to: "+str(arm.AngleValidator(Joint4))+"º")

# Need to round to mm as the stepper firmware expects an int mm
print("Stepper 1 Move to: "+str(round(Joint1))+"mm")

print(arm.AngleToOrientation(360))
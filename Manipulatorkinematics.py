import math

from numpy.lib.twodim_base import diag

class Manipulator:
    def __init__(self):
        self.a1 = 0
        self.a2 = 0
        self.a3 = 0
        self.a4 = 0
        self.l1 = 0
        self.l2 = 0
        self.l3 = 0
        self.l4 = 0
    def setDH_parameters(self, a1, a2, a3, a4, l1, l2, l3, l4):
        # L1 is the distance of joint 1 when the stepper motor is homed.
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3
        self.a4 = a4
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4
        return
    def FWDKine(self, Joint1, Joint2, Joint3, Joint4):
        #joint 2,3, and 4 are in degrees but sin and cos need rad so converted here
        Joint2 = math.radians(Joint2)
        Joint3 = math.radians(Joint3)
        Joint4 = math.radians(Joint4)
        Position = [0, 0, 0] #[X, Y, Z]
        Orientation =[[1,0,0],[0,1,0],[0,0,1]] #[[i],[j],[k]]
        X = Y = Z = 0
        I = J = K = [0,0,0]
        d1 = self.l1 + Joint1
        s2 = math.sin(Joint2)
        s3 = math.sin(Joint3)
        s4 = math.sin(Joint4)
        c2 = math.cos(Joint2)
        c3 = math.cos(Joint3)
        c4 = math.cos(Joint4)
        I = [c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2), c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3), 0]
        J = [- c4*(c2*s3 + c3*s2) - s4*(c2*c3 - s2*s3), c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2), 0]
        K = [0,0,1]
        X = self.a1 + self.a2*c2 + self.a3*c2*c3 - self.a3*s2*s3 + self.a4*c4*(c2*c3 - s2*s3) - self.a4*s4*(c2*s3 + c3*s2)
        Y = self.a2*s2 + self.a3*c2*s3 + self.a3*c3*s2 + self.a4*c4*(c2*s3 + c3*s2) + self.a4*s4*(c2*c3 - s2*s3)
        Z = d1 + self.l2 + self.l3 + self.l4
        Position = [X,Y,Z]
        Orientation = [I,J,K]
        return Position, Orientation
    
    def InvKine(self, Position, Orientation):
        zerooffset = 0.0001
        Joint1 = Joint2 = Joint3 = Joint4 = 0
        X = Position[0]
        Y = Position[1]
        Z = Position[2]
        I = Orientation[0]
        J = Orientation[1]
        K = Orientation[2]
        Yc = Y - I[1]*self.a4
        Xc = X - I[0]*self.a4
        Joint1 = float(Z - self.l2 - self.l3 - self.l4 - self.l1)
        D = (-self.a2**2 -self.a3**2 + (Xc-self.a1)**2 + Yc**2)/(2*self.a2*self.a3)
        Joint3 = math.atan((math.sqrt(1-D**2))/(D+zerooffset))
        Joint2 = math.atan((Yc)/(Xc-self.a1 +zerooffset)) - math.atan((self.a3*math.sin(Joint3))/(zerooffset+self.a2 + self.a3*math.cos(Joint3)))
        s2 = math.sin(Joint2)
        s3 = math.sin(Joint3)
        c2 = math.cos(Joint2)
        c3 = math.cos(Joint3)
        #Orientation of frame 3
        O3 = [[c2*c3 - s2*s3, c2*s3 + c3*s2, 0],[- c2*s3 - c3*s2, c2*c3 - s2*s3, 0],[0,0,1]]
        Joint4 = math.atan((I[1])/(I[0]+zerooffset)) - math.atan((O3[0][1])/(O3[0][0]+zerooffset))
        #convert radians to degrees
        Joint2 = math.degrees(Joint2)
        Joint3 = math.degrees(Joint3)
        Joint4 = math.degrees(Joint4)
        return Joint1, Joint2, Joint3, Joint4
    
    def AngleValidator(self, angle):
        #current angle can be ±180º, need to convert to 0-180 for servo with 0º being 90º on the servo
        angle = angle/2
        angle += 90
        angle = 180-angle
        angle = round(angle)
        return angle

    def DistanceValidator(self, Distance):
        #Do some inversion cos stepper homes to top
        # and down is positive for the stepper
        Distance = 400-Distance
        Distance = round(Distance)
        return Distance

    def AngleToOrientation(self, angle):
        angle = math.radians(angle)
        C_angle = round(math.cos(angle), 5)
        S_angle = round(math.sin(angle), 5)
        I = [C_angle, S_angle, 0]
        J = [-S_angle, C_angle, 0]
        K = [0,0,1.0]
        Orientation = [I,J,K]
        return Orientation
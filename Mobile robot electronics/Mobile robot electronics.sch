EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Mobile Robot"
Date "2021-04-12"
Rev "V1.0"
Comp ""
Comment1 "Author: Stanley Jackson"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L power:GND #PWR0101
U 1 1 6073A39F
P 2300 2300
F 0 "#PWR0101" H 2300 2050 50  0001 C CNN
F 1 "GND" V 2305 2172 50  0000 R CNN
F 2 "" H 2300 2300 50  0001 C CNN
F 3 "" H 2300 2300 50  0001 C CNN
	1    2300 2300
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 6073BFC2
P 2300 2900
F 0 "#PWR0102" H 2300 2650 50  0001 C CNN
F 1 "GND" V 2305 2772 50  0000 R CNN
F 2 "" H 2300 2900 50  0001 C CNN
F 3 "" H 2300 2900 50  0001 C CNN
	1    2300 2900
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 6073C1A2
P 1350 1600
F 0 "#PWR0103" H 1350 1350 50  0001 C CNN
F 1 "GND" V 1355 1472 50  0000 R CNN
F 2 "" H 1350 1600 50  0001 C CNN
F 3 "" H 1350 1600 50  0001 C CNN
	1    1350 1600
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0104
U 1 1 6073C466
P 1350 1100
F 0 "#PWR0104" H 1350 950 50  0001 C CNN
F 1 "+5V" V 1365 1228 50  0000 L CNN
F 2 "" H 1350 1100 50  0001 C CNN
F 3 "" H 1350 1100 50  0001 C CNN
	1    1350 1100
	0    -1   -1   0   
$EndComp
$Comp
L Motor_driver:BD65496MUV U2
U 1 1 6073E679
P 4600 1100
F 0 "U2" H 4950 1265 50  0000 C CNN
F 1 "BD65496MUV" H 4950 1174 50  0000 C CNN
F 2 "Motor_driver:BD65496MUV_Carrier" H 4600 1100 50  0001 C CNN
F 3 "" H 4600 1100 50  0001 C CNN
	1    4600 1100
	1    0    0    -1  
$EndComp
$Comp
L Motor_driver:BD65496MUV U3
U 1 1 6073FBB9
P 4600 2200
F 0 "U3" H 4950 2365 50  0000 C CNN
F 1 "BD65496MUV" H 4950 2274 50  0000 C CNN
F 2 "Motor_driver:BD65496MUV_Carrier" H 4600 2200 50  0001 C CNN
F 3 "" H 4600 2200 50  0001 C CNN
	1    4600 2200
	1    0    0    -1  
$EndComp
$Comp
L Motor_driver:BD65496MUV U4
U 1 1 60740246
P 6700 1100
F 0 "U4" H 7050 1265 50  0000 C CNN
F 1 "BD65496MUV" H 7050 1174 50  0000 C CNN
F 2 "Motor_driver:BD65496MUV_Carrier" H 6700 1100 50  0001 C CNN
F 3 "" H 6700 1100 50  0001 C CNN
	1    6700 1100
	1    0    0    -1  
$EndComp
$Comp
L Motor_driver:BD65496MUV U5
U 1 1 60740E17
P 6700 2200
F 0 "U5" H 7050 2365 50  0000 C CNN
F 1 "BD65496MUV" H 7050 2274 50  0000 C CNN
F 2 "Motor_driver:BD65496MUV_Carrier" H 6700 2200 50  0001 C CNN
F 3 "" H 6700 2200 50  0001 C CNN
	1    6700 2200
	1    0    0    -1  
$EndComp
Text Notes 4550 650  0    50   ~ 0
Vcc is 3.3V since ESP32 signals are 3.3V
Text Notes 4550 750  0    50   ~ 0
Datasheet: https://www.pololu.com/file/0J886/bd65496muv-e.pdf
Text Notes 4550 850  0    50   ~ 0
Vin is Vmotor so it needs to be less than 6v since the motor has a max input of 6v
$Comp
L power:GND #PWR0105
U 1 1 60741F36
P 5450 2750
F 0 "#PWR0105" H 5450 2500 50  0001 C CNN
F 1 "GND" V 5455 2622 50  0000 R CNN
F 2 "" H 5450 2750 50  0001 C CNN
F 3 "" H 5450 2750 50  0001 C CNN
	1    5450 2750
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 60742B26
P 7550 2750
F 0 "#PWR0106" H 7550 2500 50  0001 C CNN
F 1 "GND" V 7555 2622 50  0000 R CNN
F 2 "" H 7550 2750 50  0001 C CNN
F 3 "" H 7550 2750 50  0001 C CNN
	1    7550 2750
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 60742CF8
P 5450 1650
F 0 "#PWR0107" H 5450 1400 50  0001 C CNN
F 1 "GND" V 5455 1522 50  0000 R CNN
F 2 "" H 5450 1650 50  0001 C CNN
F 3 "" H 5450 1650 50  0001 C CNN
	1    5450 1650
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 6074363B
P 7550 1650
F 0 "#PWR0108" H 7550 1400 50  0001 C CNN
F 1 "GND" V 7555 1522 50  0000 R CNN
F 2 "" H 7550 1650 50  0001 C CNN
F 3 "" H 7550 1650 50  0001 C CNN
	1    7550 1650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7500 1600 7500 1650
Wire Wire Line
	7550 1650 7500 1650
Connection ~ 7500 1650
Wire Wire Line
	7500 1650 7500 1700
Wire Wire Line
	7500 2700 7500 2750
Wire Wire Line
	7500 2750 7550 2750
Connection ~ 7500 2750
Wire Wire Line
	7500 2750 7500 2800
Wire Wire Line
	5400 2700 5400 2750
Wire Wire Line
	5400 2750 5450 2750
Connection ~ 5400 2750
Wire Wire Line
	5400 2750 5400 2800
Wire Wire Line
	5400 1600 5400 1650
Wire Wire Line
	5400 1650 5450 1650
Connection ~ 5400 1650
Wire Wire Line
	5400 1650 5400 1700
$Comp
L power:+3.3V #PWR0109
U 1 1 60747A00
P 1350 2900
F 0 "#PWR0109" H 1350 2750 50  0001 C CNN
F 1 "+3.3V" V 1365 3028 50  0000 L CNN
F 2 "" H 1350 2900 50  0001 C CNN
F 3 "" H 1350 2900 50  0001 C CNN
	1    1350 2900
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR0110
U 1 1 6074801F
P 5400 1200
F 0 "#PWR0110" H 5400 1050 50  0001 C CNN
F 1 "+3.3V" V 5415 1328 50  0000 L CNN
F 2 "" H 5400 1200 50  0001 C CNN
F 3 "" H 5400 1200 50  0001 C CNN
	1    5400 1200
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR0111
U 1 1 6074959B
P 5400 2300
F 0 "#PWR0111" H 5400 2150 50  0001 C CNN
F 1 "+3.3V" V 5415 2428 50  0000 L CNN
F 2 "" H 5400 2300 50  0001 C CNN
F 3 "" H 5400 2300 50  0001 C CNN
	1    5400 2300
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR0112
U 1 1 60749F5D
P 7500 2300
F 0 "#PWR0112" H 7500 2150 50  0001 C CNN
F 1 "+3.3V" V 7515 2428 50  0000 L CNN
F 2 "" H 7500 2300 50  0001 C CNN
F 3 "" H 7500 2300 50  0001 C CNN
	1    7500 2300
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR0113
U 1 1 6074A8E0
P 7500 1200
F 0 "#PWR0113" H 7500 1050 50  0001 C CNN
F 1 "+3.3V" V 7515 1328 50  0000 L CNN
F 2 "" H 7500 1200 50  0001 C CNN
F 3 "" H 7500 1200 50  0001 C CNN
	1    7500 1200
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0114
U 1 1 6074B829
P 5400 1300
F 0 "#PWR0114" H 5400 1150 50  0001 C CNN
F 1 "+5V" V 5415 1428 50  0000 L CNN
F 2 "" H 5400 1300 50  0001 C CNN
F 3 "" H 5400 1300 50  0001 C CNN
	1    5400 1300
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0115
U 1 1 6074C91F
P 5400 2400
F 0 "#PWR0115" H 5400 2250 50  0001 C CNN
F 1 "+5V" V 5415 2528 50  0000 L CNN
F 2 "" H 5400 2400 50  0001 C CNN
F 3 "" H 5400 2400 50  0001 C CNN
	1    5400 2400
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0116
U 1 1 6074D5A9
P 7500 2400
F 0 "#PWR0116" H 7500 2250 50  0001 C CNN
F 1 "+5V" V 7515 2528 50  0000 L CNN
F 2 "" H 7500 2400 50  0001 C CNN
F 3 "" H 7500 2400 50  0001 C CNN
	1    7500 2400
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0117
U 1 1 6074E112
P 7500 1300
F 0 "#PWR0117" H 7500 1150 50  0001 C CNN
F 1 "+5V" V 7515 1428 50  0000 L CNN
F 2 "" H 7500 1300 50  0001 C CNN
F 3 "" H 7500 1300 50  0001 C CNN
	1    7500 1300
	0    1    1    0   
$EndComp
Text GLabel 5400 1400 2    50   Input ~ 0
MotorFLA
Text GLabel 5400 1500 2    50   Input ~ 0
MotorFLB
Text GLabel 7500 1400 2    50   Input ~ 0
MotorFRA
Text GLabel 7500 1500 2    50   Input ~ 0
MotorFRB
Text GLabel 7500 2600 2    50   Input ~ 0
MotorBRB
Text GLabel 5400 2600 2    50   Input ~ 0
MotorBLB
Text GLabel 5400 2500 2    50   Input ~ 0
MotorBLA
Text GLabel 7500 2500 2    50   Input ~ 0
MotorBRA
$Comp
L Connector:Conn_01x06_Male J7
U 1 1 607523FA
P 8600 1300
F 0 "J7" H 8708 1681 50  0000 C CNN
F 1 "Conn_01x06_Male" H 8708 1590 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 8600 1300 50  0001 C CNN
F 3 "~" H 8600 1300 50  0001 C CNN
	1    8600 1300
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x06_Male J9
U 1 1 60757447
P 9900 1300
F 0 "J9" H 10008 1681 50  0000 C CNN
F 1 "Conn_01x06_Male" H 10008 1590 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 9900 1300 50  0001 C CNN
F 3 "~" H 9900 1300 50  0001 C CNN
	1    9900 1300
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x06_Male J8
U 1 1 60758072
P 8650 2900
F 0 "J8" H 8758 3281 50  0000 C CNN
F 1 "Conn_01x06_Male" H 8758 3190 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 8650 2900 50  0001 C CNN
F 3 "~" H 8650 2900 50  0001 C CNN
	1    8650 2900
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x06_Male J10
U 1 1 60758D41
P 9950 2900
F 0 "J10" H 10058 3281 50  0000 C CNN
F 1 "Conn_01x06_Male" H 10058 3190 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 9950 2900 50  0001 C CNN
F 3 "~" H 9950 2900 50  0001 C CNN
	1    9950 2900
	1    0    0    -1  
$EndComp
Text Notes 8800 950  0    50   ~ 0
motor FL
Text Notes 10100 950  0    50   ~ 0
motor FR\n
Text Notes 8850 2550 0    50   ~ 0
motor BL
Text Notes 10150 2550 0    50   ~ 0
motor BR
$Comp
L power:GND #PWR0118
U 1 1 6075BC4E
P 8850 2900
F 0 "#PWR0118" H 8850 2650 50  0001 C CNN
F 1 "GND" V 8855 2772 50  0000 R CNN
F 2 "" H 8850 2900 50  0001 C CNN
F 3 "" H 8850 2900 50  0001 C CNN
	1    8850 2900
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0119
U 1 1 6075C9CB
P 8800 1300
F 0 "#PWR0119" H 8800 1050 50  0001 C CNN
F 1 "GND" V 8805 1172 50  0000 R CNN
F 2 "" H 8800 1300 50  0001 C CNN
F 3 "" H 8800 1300 50  0001 C CNN
	1    8800 1300
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0120
U 1 1 6075CF89
P 10100 1300
F 0 "#PWR0120" H 10100 1050 50  0001 C CNN
F 1 "GND" V 10105 1172 50  0000 R CNN
F 2 "" H 10100 1300 50  0001 C CNN
F 3 "" H 10100 1300 50  0001 C CNN
	1    10100 1300
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0121
U 1 1 6075D6CA
P 10150 2900
F 0 "#PWR0121" H 10150 2650 50  0001 C CNN
F 1 "GND" V 10155 2772 50  0000 R CNN
F 2 "" H 10150 2900 50  0001 C CNN
F 3 "" H 10150 2900 50  0001 C CNN
	1    10150 2900
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR0122
U 1 1 6075DEC9
P 8850 3000
F 0 "#PWR0122" H 8850 2850 50  0001 C CNN
F 1 "+5V" V 8865 3128 50  0000 L CNN
F 2 "" H 8850 3000 50  0001 C CNN
F 3 "" H 8850 3000 50  0001 C CNN
	1    8850 3000
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0123
U 1 1 6075EF66
P 8800 1400
F 0 "#PWR0123" H 8800 1250 50  0001 C CNN
F 1 "+5V" V 8815 1528 50  0000 L CNN
F 2 "" H 8800 1400 50  0001 C CNN
F 3 "" H 8800 1400 50  0001 C CNN
	1    8800 1400
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0124
U 1 1 6075F68A
P 10100 1400
F 0 "#PWR0124" H 10100 1250 50  0001 C CNN
F 1 "+5V" V 10115 1528 50  0000 L CNN
F 2 "" H 10100 1400 50  0001 C CNN
F 3 "" H 10100 1400 50  0001 C CNN
	1    10100 1400
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0125
U 1 1 6075FE44
P 10150 3000
F 0 "#PWR0125" H 10150 2850 50  0001 C CNN
F 1 "+5V" V 10165 3128 50  0000 L CNN
F 2 "" H 10150 3000 50  0001 C CNN
F 3 "" H 10150 3000 50  0001 C CNN
	1    10150 3000
	0    1    1    0   
$EndComp
Text GLabel 9150 1800 2    50   Input ~ 0
Encoder_1_A
Text GLabel 8750 1900 0    50   Input ~ 0
Encoder_1_B
Text GLabel 9200 3400 2    50   Input ~ 0
Encoder_3_A
Text GLabel 8800 3500 0    50   Input ~ 0
Encoder_3_B
Text GLabel 10500 3400 2    50   Input ~ 0
Encoder_4_A
Text GLabel 10100 3500 0    50   Input ~ 0
Encoder_4_B
Text GLabel 10450 1800 2    50   Input ~ 0
Encoder_2_A
Text GLabel 10050 1900 0    50   Input ~ 0
Encoder_2_B
$Comp
L power:GND #PWR0126
U 1 1 6079888D
P 8850 3800
F 0 "#PWR0126" H 8850 3550 50  0001 C CNN
F 1 "GND" H 8855 3627 50  0000 C CNN
F 2 "" H 8850 3800 50  0001 C CNN
F 3 "" H 8850 3800 50  0001 C CNN
	1    8850 3800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0127
U 1 1 607994AD
P 10150 3800
F 0 "#PWR0127" H 10150 3550 50  0001 C CNN
F 1 "GND" H 10155 3627 50  0000 C CNN
F 2 "" H 10150 3800 50  0001 C CNN
F 3 "" H 10150 3800 50  0001 C CNN
	1    10150 3800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0128
U 1 1 60799E6F
P 10100 2200
F 0 "#PWR0128" H 10100 1950 50  0001 C CNN
F 1 "GND" H 10105 2027 50  0000 C CNN
F 2 "" H 10100 2200 50  0001 C CNN
F 3 "" H 10100 2200 50  0001 C CNN
	1    10100 2200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0129
U 1 1 6079A793
P 8800 2200
F 0 "#PWR0129" H 8800 1950 50  0001 C CNN
F 1 "GND" H 8805 2027 50  0000 C CNN
F 2 "" H 8800 2200 50  0001 C CNN
F 3 "" H 8800 2200 50  0001 C CNN
	1    8800 2200
	1    0    0    -1  
$EndComp
Text Notes 8000 750  0    50   ~ 0
voltage divders are needed since the encoders need a Vcc greater than 3.3v\nand the outputs from them are from 0 to Vcc.  The ESP32 pin are not 5v tolerant
Text GLabel 4500 1200 0    50   Input ~ 0
Motor_EN
Text GLabel 4500 2300 0    50   Input ~ 0
Motor_EN
Text GLabel 6600 2300 0    50   Input ~ 0
Motor_EN
Text GLabel 6600 1200 0    50   Input ~ 0
Motor_EN
NoConn ~ 6600 2400
NoConn ~ 6600 2500
NoConn ~ 4500 2400
NoConn ~ 4500 2500
NoConn ~ 4500 1300
NoConn ~ 4500 1400
NoConn ~ 6600 1300
NoConn ~ 6600 1400
$Comp
L power:+3.3V #PWR0130
U 1 1 607A335D
P 4500 2800
F 0 "#PWR0130" H 4500 2650 50  0001 C CNN
F 1 "+3.3V" V 4515 2928 50  0000 L CNN
F 2 "" H 4500 2800 50  0001 C CNN
F 3 "" H 4500 2800 50  0001 C CNN
	1    4500 2800
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR0131
U 1 1 607A422E
P 4500 1700
F 0 "#PWR0131" H 4500 1550 50  0001 C CNN
F 1 "+3.3V" V 4515 1828 50  0000 L CNN
F 2 "" H 4500 1700 50  0001 C CNN
F 3 "" H 4500 1700 50  0001 C CNN
	1    4500 1700
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR0132
U 1 1 607A4986
P 6600 1700
F 0 "#PWR0132" H 6600 1550 50  0001 C CNN
F 1 "+3.3V" V 6615 1828 50  0000 L CNN
F 2 "" H 6600 1700 50  0001 C CNN
F 3 "" H 6600 1700 50  0001 C CNN
	1    6600 1700
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR0133
U 1 1 607A545E
P 6600 2800
F 0 "#PWR0133" H 6600 2650 50  0001 C CNN
F 1 "+3.3V" V 6615 2928 50  0000 L CNN
F 2 "" H 6600 2800 50  0001 C CNN
F 3 "" H 6600 2800 50  0001 C CNN
	1    6600 2800
	0    -1   -1   0   
$EndComp
Text GLabel 4500 1500 0    50   Input ~ 0
Motor_FL_PWM
Text GLabel 4500 1600 0    50   Input ~ 0
Motor_FL_DIR
Text GLabel 4500 2600 0    50   Input ~ 0
Motor_BL_PWM
Text GLabel 4500 2700 0    50   Input ~ 0
Motor_BL_DIR
Text GLabel 6600 1500 0    50   Input ~ 0
Motor_FR_PWM
Text GLabel 1350 1800 0    50   Input ~ 0
Motor_EN
Text Notes 500  550  0    50   ~ 0
ESP32 MCU\n
Text Notes 3850 550  0    50   ~ 0
Motor Drivers\n
Text Notes 8000 550  0    50   ~ 0
Motor Connectors
Text GLabel 1350 2200 0    50   Input ~ 0
Encoder_3_B
Text GLabel 2300 2100 2    50   Input ~ 0
Encoder_1_A
Text GLabel 2300 2200 2    50   Input ~ 0
Encoder_1_B
Text GLabel 2300 1700 2    50   Input ~ 0
Encoder_2_A
Text GLabel 2300 2000 2    50   Input ~ 0
Encoder_2_B
Text GLabel 1350 2400 0    50   Input ~ 0
Encoder_4_A
Text GLabel 1350 2500 0    50   Input ~ 0
Encoder_4_B
$Comp
L Connector:Conn_01x04_Male J1
U 1 1 6074BB3F
P 1150 4950
F 0 "J1" H 1258 5231 50  0000 C CNN
F 1 "Front_Sens" H 1258 5140 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 1150 4950 50  0001 C CNN
F 3 "~" H 1150 4950 50  0001 C CNN
	1    1150 4950
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male J2
U 1 1 6074CDDE
P 1150 5850
F 0 "J2" H 1258 6131 50  0000 C CNN
F 1 "Back_Sens" H 1258 6040 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 1150 5850 50  0001 C CNN
F 3 "~" H 1150 5850 50  0001 C CNN
	1    1150 5850
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male J3
U 1 1 6074D5AF
P 2200 4950
F 0 "J3" H 2308 5231 50  0000 C CNN
F 1 "Left_Sens" H 2308 5140 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 2200 4950 50  0001 C CNN
F 3 "~" H 2200 4950 50  0001 C CNN
	1    2200 4950
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male J4
U 1 1 6074EC48
P 2200 5850
F 0 "J4" H 2308 6131 50  0000 C CNN
F 1 "Right_sens" H 2308 6040 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 2200 5850 50  0001 C CNN
F 3 "~" H 2200 5850 50  0001 C CNN
	1    2200 5850
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0134
U 1 1 6074F8CA
P 1350 4850
F 0 "#PWR0134" H 1350 4700 50  0001 C CNN
F 1 "+5V" V 1365 4978 50  0000 L CNN
F 2 "" H 1350 4850 50  0001 C CNN
F 3 "" H 1350 4850 50  0001 C CNN
	1    1350 4850
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0135
U 1 1 607502D2
P 1350 5750
F 0 "#PWR0135" H 1350 5600 50  0001 C CNN
F 1 "+5V" V 1365 5878 50  0000 L CNN
F 2 "" H 1350 5750 50  0001 C CNN
F 3 "" H 1350 5750 50  0001 C CNN
	1    1350 5750
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0136
U 1 1 60750B7D
P 2400 4850
F 0 "#PWR0136" H 2400 4700 50  0001 C CNN
F 1 "+5V" V 2415 4978 50  0000 L CNN
F 2 "" H 2400 4850 50  0001 C CNN
F 3 "" H 2400 4850 50  0001 C CNN
	1    2400 4850
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0137
U 1 1 6075120E
P 2400 5750
F 0 "#PWR0137" H 2400 5600 50  0001 C CNN
F 1 "+5V" V 2415 5878 50  0000 L CNN
F 2 "" H 2400 5750 50  0001 C CNN
F 3 "" H 2400 5750 50  0001 C CNN
	1    2400 5750
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0138
U 1 1 60751F12
P 1350 5150
F 0 "#PWR0138" H 1350 4900 50  0001 C CNN
F 1 "GND" V 1355 5022 50  0000 R CNN
F 2 "" H 1350 5150 50  0001 C CNN
F 3 "" H 1350 5150 50  0001 C CNN
	1    1350 5150
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0139
U 1 1 60752D67
P 1350 6050
F 0 "#PWR0139" H 1350 5800 50  0001 C CNN
F 1 "GND" V 1355 5922 50  0000 R CNN
F 2 "" H 1350 6050 50  0001 C CNN
F 3 "" H 1350 6050 50  0001 C CNN
	1    1350 6050
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0140
U 1 1 60753442
P 2400 5150
F 0 "#PWR0140" H 2400 4900 50  0001 C CNN
F 1 "GND" V 2405 5022 50  0000 R CNN
F 2 "" H 2400 5150 50  0001 C CNN
F 3 "" H 2400 5150 50  0001 C CNN
	1    2400 5150
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0141
U 1 1 60753976
P 2400 6050
F 0 "#PWR0141" H 2400 5800 50  0001 C CNN
F 1 "GND" V 2405 5922 50  0000 R CNN
F 2 "" H 2400 6050 50  0001 C CNN
F 3 "" H 2400 6050 50  0001 C CNN
	1    2400 6050
	0    -1   -1   0   
$EndComp
Text GLabel 1350 2300 0    50   Input ~ 0
Encoder_3_A
Text GLabel 6600 1600 0    50   Input ~ 0
Motor_FR_DIR
Text GLabel 6600 2600 0    50   Input ~ 0
Motor_BR_PWM
Text GLabel 6600 2700 0    50   Input ~ 0
Motor_BR_DIR
Text GLabel 8800 1100 2    50   Input ~ 0
MotorFLA
Text GLabel 8800 1200 2    50   Input ~ 0
MotorFLB
Text GLabel 8850 2800 2    50   Input ~ 0
MotorBLB
Text GLabel 8850 2700 2    50   Input ~ 0
MotorBLA
Text GLabel 10150 2800 2    50   Input ~ 0
MotorBRB
Text GLabel 10150 2700 2    50   Input ~ 0
MotorBRA
Text GLabel 10100 1200 2    50   Input ~ 0
MotorFRB
Text GLabel 10100 1100 2    50   Input ~ 0
MotorFRA
Text GLabel 1350 1500 0    50   Input ~ 0
Motor_FL_PWM
Text GLabel 1350 1700 0    50   Input ~ 0
Motor_FL_DIR
Text GLabel 2300 1500 2    50   Input ~ 0
Motor_FR_PWM
Text GLabel 2300 1600 2    50   Input ~ 0
Motor_FR_DIR
Text GLabel 1350 2000 0    50   Input ~ 0
Motor_BL_PWM
Text GLabel 1350 2100 0    50   Input ~ 0
Motor_BL_DIR
Text GLabel 2300 2700 2    50   Input ~ 0
Motor_BR_PWM
Text GLabel 2300 2800 2    50   Input ~ 0
Motor_BR_DIR
$Comp
L MCU_ESP:ESP32-DevKitc-V4-WROVER U1
U 1 1 608569C9
P 1800 900
F 0 "U1" H 1825 925 50  0000 C CNN
F 1 "ESP32-DevKitc-V4-WROVER" H 1825 834 50  0000 C CNN
F 2 "ESP32-DEVKITC-32D:ESP32-DevKitC-V4-WROVER" H 1800 900 50  0001 C CNN
F 3 "" H 1800 900 50  0001 C CNN
	1    1800 900 
	1    0    0    -1  
$EndComp
NoConn ~ 2300 1800
NoConn ~ 2300 1900
NoConn ~ 1350 1300
NoConn ~ 1350 1400
NoConn ~ 2300 1200
NoConn ~ 2300 1300
NoConn ~ 2300 1100
NoConn ~ 1350 1200
$Comp
L Device:R R11
U 1 1 608668F4
P 8800 1750
F 0 "R11" H 8870 1796 50  0000 L CNN
F 1 "1K" H 8870 1705 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 8730 1750 50  0001 C CNN
F 3 "~" H 8800 1750 50  0001 C CNN
	1    8800 1750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R15
U 1 1 608674EF
P 9100 1650
F 0 "R15" H 9170 1696 50  0000 L CNN
F 1 "1K" H 9170 1605 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9030 1650 50  0001 C CNN
F 3 "~" H 9100 1650 50  0001 C CNN
	1    9100 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R12
U 1 1 6086E286
P 8800 2050
F 0 "R12" H 8870 2096 50  0000 L CNN
F 1 "1K8" H 8870 2005 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 8730 2050 50  0001 C CNN
F 3 "~" H 8800 2050 50  0001 C CNN
	1    8800 2050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R16
U 1 1 6086E7EC
P 9100 2050
F 0 "R16" H 9170 2096 50  0000 L CNN
F 1 "1K8" H 9170 2005 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9030 2050 50  0001 C CNN
F 3 "~" H 9100 2050 50  0001 C CNN
	1    9100 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9100 1800 9100 1900
Wire Wire Line
	8800 2200 9100 2200
Connection ~ 8800 2200
Wire Wire Line
	8800 1500 9100 1500
Wire Wire Line
	9100 1800 9150 1800
Connection ~ 9100 1800
Wire Wire Line
	8750 1900 8800 1900
Connection ~ 8800 1900
$Comp
L Device:R R19
U 1 1 60878869
P 10100 1750
F 0 "R19" H 10170 1796 50  0000 L CNN
F 1 "1K" H 10170 1705 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 10030 1750 50  0001 C CNN
F 3 "~" H 10100 1750 50  0001 C CNN
	1    10100 1750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R23
U 1 1 6087886F
P 10400 1650
F 0 "R23" H 10470 1696 50  0000 L CNN
F 1 "1K" H 10470 1605 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 10330 1650 50  0001 C CNN
F 3 "~" H 10400 1650 50  0001 C CNN
	1    10400 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R20
U 1 1 60878875
P 10100 2050
F 0 "R20" H 10170 2096 50  0000 L CNN
F 1 "1K8" H 10170 2005 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 10030 2050 50  0001 C CNN
F 3 "~" H 10100 2050 50  0001 C CNN
	1    10100 2050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R24
U 1 1 6087887B
P 10400 2050
F 0 "R24" H 10470 2096 50  0000 L CNN
F 1 "1K8" H 10470 2005 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 10330 2050 50  0001 C CNN
F 3 "~" H 10400 2050 50  0001 C CNN
	1    10400 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	10400 1800 10400 1900
Wire Wire Line
	10400 1800 10450 1800
Connection ~ 10400 1800
Wire Wire Line
	10050 1900 10100 1900
Connection ~ 10100 1900
$Comp
L Device:R R13
U 1 1 6087CFBA
P 8850 3350
F 0 "R13" H 8920 3396 50  0000 L CNN
F 1 "1K" H 8920 3305 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 8780 3350 50  0001 C CNN
F 3 "~" H 8850 3350 50  0001 C CNN
	1    8850 3350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R17
U 1 1 6087CFC0
P 9150 3250
F 0 "R17" H 9220 3296 50  0000 L CNN
F 1 "1K" H 9220 3205 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9080 3250 50  0001 C CNN
F 3 "~" H 9150 3250 50  0001 C CNN
	1    9150 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R14
U 1 1 6087CFC6
P 8850 3650
F 0 "R14" H 8920 3696 50  0000 L CNN
F 1 "1K8" H 8920 3605 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 8780 3650 50  0001 C CNN
F 3 "~" H 8850 3650 50  0001 C CNN
	1    8850 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R18
U 1 1 6087CFCC
P 9150 3650
F 0 "R18" H 9220 3696 50  0000 L CNN
F 1 "1K8" H 9220 3605 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 9080 3650 50  0001 C CNN
F 3 "~" H 9150 3650 50  0001 C CNN
	1    9150 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	9150 3400 9150 3500
Wire Wire Line
	9150 3400 9200 3400
Connection ~ 9150 3400
Wire Wire Line
	8800 3500 8850 3500
Connection ~ 8850 3500
$Comp
L Device:R R21
U 1 1 6087F336
P 10150 3350
F 0 "R21" H 10220 3396 50  0000 L CNN
F 1 "1K" H 10220 3305 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 10080 3350 50  0001 C CNN
F 3 "~" H 10150 3350 50  0001 C CNN
	1    10150 3350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R25
U 1 1 6087F33C
P 10450 3250
F 0 "R25" H 10520 3296 50  0000 L CNN
F 1 "1K" H 10520 3205 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 10380 3250 50  0001 C CNN
F 3 "~" H 10450 3250 50  0001 C CNN
	1    10450 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R22
U 1 1 6087F342
P 10150 3650
F 0 "R22" H 10220 3696 50  0000 L CNN
F 1 "1K8" H 10220 3605 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 10080 3650 50  0001 C CNN
F 3 "~" H 10150 3650 50  0001 C CNN
	1    10150 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R26
U 1 1 6087F348
P 10450 3650
F 0 "R26" H 10520 3696 50  0000 L CNN
F 1 "1K8" H 10520 3605 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 10380 3650 50  0001 C CNN
F 3 "~" H 10450 3650 50  0001 C CNN
	1    10450 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	10450 3400 10450 3500
Wire Wire Line
	10450 3400 10500 3400
Connection ~ 10450 3400
Wire Wire Line
	10100 3500 10150 3500
Connection ~ 10150 3500
Wire Wire Line
	10100 1500 10400 1500
Wire Wire Line
	10100 2200 10400 2200
Connection ~ 10100 2200
Wire Wire Line
	8850 3100 9150 3100
Wire Wire Line
	8850 3800 9150 3800
Connection ~ 8850 3800
Wire Wire Line
	10150 3800 10450 3800
Connection ~ 10150 3800
Wire Wire Line
	10150 3100 10450 3100
Text GLabel 1350 4950 2    50   Input ~ 0
TRIG_US
Text GLabel 1350 5850 2    50   Input ~ 0
TRIG_US
Text GLabel 2400 4950 2    50   Input ~ 0
TRIG_US
Text GLabel 2400 5850 2    50   Input ~ 0
TRIG_US
Text GLabel 1350 1900 0    50   Input ~ 0
TRIG_US
Text Notes 1200 4600 0    50   ~ 0
Front sensor\n
Text Notes 1150 5500 0    50   ~ 0
Back Sensor
Text Notes 2200 4600 0    50   ~ 0
Left Sensor\n
Text Notes 2200 5500 0    50   ~ 0
Right Sensor
$Comp
L Device:R R2
U 1 1 6088C8DD
P 1750 5500
F 0 "R2" H 1820 5546 50  0000 L CNN
F 1 "1K8" H 1820 5455 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 1680 5500 50  0001 C CNN
F 3 "~" H 1750 5500 50  0001 C CNN
	1    1750 5500
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 6088C8D7
P 1750 5200
F 0 "R1" H 1820 5246 50  0000 L CNN
F 1 "1K" H 1820 5155 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 1680 5200 50  0001 C CNN
F 3 "~" H 1750 5200 50  0001 C CNN
	1    1750 5200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 6089FCDA
P 1750 6400
F 0 "R4" H 1820 6446 50  0000 L CNN
F 1 "1K8" H 1820 6355 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 1680 6400 50  0001 C CNN
F 3 "~" H 1750 6400 50  0001 C CNN
	1    1750 6400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 6089FCE0
P 1750 6100
F 0 "R3" H 1820 6146 50  0000 L CNN
F 1 "1K" H 1820 6055 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 1680 6100 50  0001 C CNN
F 3 "~" H 1750 6100 50  0001 C CNN
	1    1750 6100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R8
U 1 1 608A0CD2
P 2800 6400
F 0 "R8" H 2870 6446 50  0000 L CNN
F 1 "1K8" H 2870 6355 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2730 6400 50  0001 C CNN
F 3 "~" H 2800 6400 50  0001 C CNN
	1    2800 6400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 608A0CD8
P 2800 6100
F 0 "R7" H 2870 6146 50  0000 L CNN
F 1 "1K" H 2870 6055 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2730 6100 50  0001 C CNN
F 3 "~" H 2800 6100 50  0001 C CNN
	1    2800 6100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 608A15C5
P 2800 5500
F 0 "R6" H 2870 5546 50  0000 L CNN
F 1 "1K8" H 2870 5455 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2730 5500 50  0001 C CNN
F 3 "~" H 2800 5500 50  0001 C CNN
	1    2800 5500
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 608A15CB
P 2800 5200
F 0 "R5" H 2870 5246 50  0000 L CNN
F 1 "1K" H 2870 5155 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2730 5200 50  0001 C CNN
F 3 "~" H 2800 5200 50  0001 C CNN
	1    2800 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 5950 1750 5950
Wire Wire Line
	1350 5050 1750 5050
Wire Wire Line
	2400 5050 2800 5050
Wire Wire Line
	2400 5950 2800 5950
$Comp
L power:GND #PWR0142
U 1 1 608A47CD
P 1750 6550
F 0 "#PWR0142" H 1750 6300 50  0001 C CNN
F 1 "GND" V 1755 6422 50  0000 R CNN
F 2 "" H 1750 6550 50  0001 C CNN
F 3 "" H 1750 6550 50  0001 C CNN
	1    1750 6550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0143
U 1 1 608A51AC
P 2800 6550
F 0 "#PWR0143" H 2800 6300 50  0001 C CNN
F 1 "GND" V 2805 6422 50  0000 R CNN
F 2 "" H 2800 6550 50  0001 C CNN
F 3 "" H 2800 6550 50  0001 C CNN
	1    2800 6550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0144
U 1 1 608A58E5
P 2800 5650
F 0 "#PWR0144" H 2800 5400 50  0001 C CNN
F 1 "GND" V 2805 5522 50  0000 R CNN
F 2 "" H 2800 5650 50  0001 C CNN
F 3 "" H 2800 5650 50  0001 C CNN
	1    2800 5650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0145
U 1 1 608A5FAA
P 1750 5650
F 0 "#PWR0145" H 1750 5400 50  0001 C CNN
F 1 "GND" V 1755 5522 50  0000 R CNN
F 2 "" H 1750 5650 50  0001 C CNN
F 3 "" H 1750 5650 50  0001 C CNN
	1    1750 5650
	1    0    0    -1  
$EndComp
Text GLabel 1850 5350 2    50   Input ~ 0
Front_US
Text GLabel 1850 6250 2    50   Input ~ 0
Back_US
Text GLabel 2950 6250 2    50   Input ~ 0
Right_US
Text GLabel 2950 5350 2    50   Input ~ 0
Left_US
Wire Wire Line
	1750 6250 1850 6250
Connection ~ 1750 6250
Wire Wire Line
	1750 5350 1850 5350
Connection ~ 1750 5350
Wire Wire Line
	2800 5350 2950 5350
Connection ~ 2800 5350
Wire Wire Line
	2800 6250 2950 6250
Connection ~ 2800 6250
Text GLabel 2300 1400 2    50   Input ~ 0
Front_US
Text GLabel 1350 2700 0    50   Input ~ 0
Back_US
Text GLabel 1350 2600 0    50   Input ~ 0
Left_US
Text GLabel 2300 2400 2    50   Input ~ 0
Right_US
Text Notes 700  4500 0    50   ~ 0
Voltage dividers to ensure the 5V signal from US sensor is stepped down\nto 3.2V for ESP
Wire Notes Line
	450  4250 11250 4250
Wire Notes Line
	7950 450  7950 4250
Wire Notes Line
	3800 450  3800 7800
$Comp
L Connector:Conn_01x02_Male J5
U 1 1 608C2658
P 4850 4700
F 0 "J5" H 4958 4881 50  0000 C CNN
F 1 "Conn_01x02_Male" H 4958 4790 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 4850 4700 50  0001 C CNN
F 3 "~" H 4850 4700 50  0001 C CNN
	1    4850 4700
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Male J6
U 1 1 608C3083
P 5650 4700
F 0 "J6" H 5758 4881 50  0000 C CNN
F 1 "Conn_01x02_Male" H 5758 4790 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 5650 4700 50  0001 C CNN
F 3 "~" H 5650 4700 50  0001 C CNN
	1    5650 4700
	1    0    0    -1  
$EndComp
Text Notes 4250 4450 0    50   ~ 0
Connectors for the limit switches for load detection\n
$Comp
L Device:R R10
U 1 1 608C43FB
P 5850 4950
F 0 "R10" H 5920 4996 50  0000 L CNN
F 1 "10K" H 5920 4905 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 5780 4950 50  0001 C CNN
F 3 "~" H 5850 4950 50  0001 C CNN
	1    5850 4950
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0146
U 1 1 608C51E9
P 5050 4700
F 0 "#PWR0146" H 5050 4550 50  0001 C CNN
F 1 "+3.3V" V 5065 4828 50  0000 L CNN
F 2 "" H 5050 4700 50  0001 C CNN
F 3 "" H 5050 4700 50  0001 C CNN
	1    5050 4700
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR0147
U 1 1 608C753B
P 5850 4700
F 0 "#PWR0147" H 5850 4550 50  0001 C CNN
F 1 "+3.3V" V 5865 4828 50  0000 L CNN
F 2 "" H 5850 4700 50  0001 C CNN
F 3 "" H 5850 4700 50  0001 C CNN
	1    5850 4700
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0148
U 1 1 608C7AB9
P 5850 5100
F 0 "#PWR0148" H 5850 4850 50  0001 C CNN
F 1 "GND" V 5855 4972 50  0000 R CNN
F 2 "" H 5850 5100 50  0001 C CNN
F 3 "" H 5850 5100 50  0001 C CNN
	1    5850 5100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 608CB43D
P 5050 4950
F 0 "R9" H 5120 4996 50  0000 L CNN
F 1 "10K" H 5120 4905 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4980 4950 50  0001 C CNN
F 3 "~" H 5050 4950 50  0001 C CNN
	1    5050 4950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0149
U 1 1 608CB443
P 5050 5100
F 0 "#PWR0149" H 5050 4850 50  0001 C CNN
F 1 "GND" V 5055 4972 50  0000 R CNN
F 2 "" H 5050 5100 50  0001 C CNN
F 3 "" H 5050 5100 50  0001 C CNN
	1    5050 5100
	1    0    0    -1  
$EndComp
Text GLabel 5950 4800 2    50   Input ~ 0
Load_SW1
Text GLabel 5150 4800 2    50   Input ~ 0
Load_SW2
Wire Wire Line
	5050 4800 5150 4800
Connection ~ 5050 4800
Wire Wire Line
	5850 4800 5950 4800
Connection ~ 5850 4800
Text GLabel 2300 2600 2    50   Input ~ 0
Load_SW2
Text GLabel 2300 2500 2    50   Input ~ 0
Load_SW1
Text Notes 800  3400 0    50   ~ 0
Use the onboard regulator of the esp32 devkitc to create thge 3v3 rail.
Text Notes 800  3550 0    50   ~ 0
External 5v power supply\n
Wire Notes Line
	450  3200 3800 3200
Text Notes 500  3300 0    50   ~ 0
Power\n
$Comp
L power:GND #PWR0150
U 1 1 608EA124
P 1150 3950
F 0 "#PWR0150" H 1150 3700 50  0001 C CNN
F 1 "GND" V 1155 3822 50  0000 R CNN
F 2 "" H 1150 3950 50  0001 C CNN
F 3 "" H 1150 3950 50  0001 C CNN
	1    1150 3950
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR0151
U 1 1 608EAA30
P 1150 4050
F 0 "#PWR0151" H 1150 3900 50  0001 C CNN
F 1 "+5V" V 1165 4178 50  0000 L CNN
F 2 "" H 1150 4050 50  0001 C CNN
F 3 "" H 1150 4050 50  0001 C CNN
	1    1150 4050
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J11
U 1 1 608EC567
P 950 3950
F 0 "J11" H 868 3625 50  0000 C CNN
F 1 "Conn_01x03" H 868 3716 50  0000 C CNN
F 2 "Connector_BarrelJack:BarrelJack_Horizontal" H 950 3950 50  0001 C CNN
F 3 "~" H 950 3950 50  0001 C CNN
	1    950  3950
	-1   0    0    1   
$EndComp
Wire Wire Line
	1150 3850 1150 3950
Connection ~ 1150 3950
$Comp
L Mechanical:MountingHole H1
U 1 1 6088D218
P 8850 4850
F 0 "H1" H 8950 4896 50  0000 L CNN
F 1 "MountingHole" H 8950 4805 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4" H 8850 4850 50  0001 C CNN
F 3 "~" H 8850 4850 50  0001 C CNN
	1    8850 4850
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H2
U 1 1 6088D7F9
P 8850 5050
F 0 "H2" H 8950 5096 50  0000 L CNN
F 1 "MountingHole" H 8950 5005 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4" H 8850 5050 50  0001 C CNN
F 3 "~" H 8850 5050 50  0001 C CNN
	1    8850 5050
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H3
U 1 1 6088E0B9
P 8850 5250
F 0 "H3" H 8950 5296 50  0000 L CNN
F 1 "MountingHole" H 8950 5205 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4" H 8850 5250 50  0001 C CNN
F 3 "~" H 8850 5250 50  0001 C CNN
	1    8850 5250
	1    0    0    -1  
$EndComp
$EndSCHEMATC

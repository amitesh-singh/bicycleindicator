EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "t13a based bicycle indicator"
Date "2020-11-14"
Rev "v1.0"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MCU_Microchip_ATtiny:ATtiny13A-SSU U?
U 1 1 5FAFE2BF
P 4750 2450
F 0 "U?" H 4220 2496 50  0000 R CNN
F 1 "ATtiny13A-SSU" H 4220 2405 50  0000 R CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 4750 2450 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/doc8126.pdf" H 4750 2450 50  0001 C CNN
	1    4750 2450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FB2828B
P 2950 1550
F 0 "#PWR?" H 2950 1300 50  0001 C CNN
F 1 "GND" H 2955 1377 50  0000 C CNN
F 2 "" H 2950 1550 50  0001 C CNN
F 3 "" H 2950 1550 50  0001 C CNN
	1    2950 1550
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG?
U 1 1 5FB294ED
P 2950 1550
F 0 "#FLG?" H 2950 1625 50  0001 C CNN
F 1 "PWR_FLAG" H 2950 1723 50  0000 C CNN
F 2 "" H 2950 1550 50  0001 C CNN
F 3 "~" H 2950 1550 50  0001 C CNN
	1    2950 1550
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 5FB29C19
P 3200 1550
F 0 "#PWR?" H 3200 1400 50  0001 C CNN
F 1 "VCC" H 3215 1723 50  0000 C CNN
F 2 "" H 3200 1550 50  0001 C CNN
F 3 "" H 3200 1550 50  0001 C CNN
	1    3200 1550
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG?
U 1 1 5FB2A332
P 3200 1550
F 0 "#FLG?" H 3200 1625 50  0001 C CNN
F 1 "PWR_FLAG" H 3200 1723 50  0000 C CNN
F 2 "" H 3200 1550 50  0001 C CNN
F 3 "~" H 3200 1550 50  0001 C CNN
	1    3200 1550
	-1   0    0    1   
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 5FB2B995
P 4750 1850
F 0 "#PWR?" H 4750 1700 50  0001 C CNN
F 1 "VCC" H 4765 2023 50  0000 C CNN
F 2 "" H 4750 1850 50  0001 C CNN
F 3 "" H 4750 1850 50  0001 C CNN
	1    4750 1850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FB2BE3D
P 4750 3050
F 0 "#PWR?" H 4750 2800 50  0001 C CNN
F 1 "GND" H 4755 2877 50  0000 C CNN
F 2 "" H 4750 3050 50  0001 C CNN
F 3 "" H 4750 3050 50  0001 C CNN
	1    4750 3050
	1    0    0    -1  
$EndComp
$EndSCHEMATC

EESchema Schematic File Version 2  date nie, 30 sty 2011, 13:31:02
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:special
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:cs5343_4
EELAYER 24  0
EELAYER END
$Descr A4 11700 8267
Sheet 1 1
Title ""
Date "21 jan 2011"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Connection ~ 6250 1550
Wire Wire Line
	6250 2050 6250 1550
Connection ~ 5650 1850
Wire Wire Line
	5650 2050 5650 1850
Wire Wire Line
	6250 2450 6250 2500
Wire Wire Line
	5650 2450 5650 2500
Wire Wire Line
	3100 2450 3100 2500
Connection ~ 2800 1950
Wire Wire Line
	2800 2050 2800 1950
Connection ~ 4750 1750
Wire Wire Line
	4750 2050 4750 1750
Wire Wire Line
	5050 2450 5050 2500
Connection ~ 4450 1650
Wire Wire Line
	4450 1650 4450 2350
Wire Wire Line
	3200 1850 2750 1850
Wire Wire Line
	3200 1650 2750 1650
Wire Wire Line
	6850 1850 4350 1850
Wire Wire Line
	4350 1750 6850 1750
Wire Wire Line
	6850 1550 4350 1550
Wire Wire Line
	6850 1650 4350 1650
Wire Wire Line
	4350 1950 6850 1950
Wire Wire Line
	3200 1550 2750 1550
Wire Wire Line
	3200 1750 2750 1750
Wire Wire Line
	2750 1950 3200 1950
Wire Wire Line
	4750 2450 4750 2500
Wire Wire Line
	5050 2050 5050 1950
Connection ~ 5050 1950
Wire Wire Line
	3100 1950 3100 2050
Connection ~ 3100 1950
Wire Wire Line
	2800 2450 2800 2500
Wire Wire Line
	5350 2450 5350 2500
Wire Wire Line
	5950 2450 5950 2500
Wire Wire Line
	5350 2050 5350 1850
Connection ~ 5350 1850
Wire Wire Line
	5950 2050 5950 1550
Connection ~ 5950 1550
$Comp
L GND #PWR01
U 1 1 4D39EC64
P 6250 2500
F 0 "#PWR01" H 6250 2500 30  0001 C CNN
F 1 "GND" H 6250 2430 30  0001 C CNN
	1    6250 2500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 4D39EC63
P 5950 2500
F 0 "#PWR02" H 5950 2500 30  0001 C CNN
F 1 "GND" H 5950 2430 30  0001 C CNN
	1    5950 2500
	1    0    0    -1  
$EndComp
$Comp
L C C8
U 1 1 4D39EC54
P 6250 2250
F 0 "C8" H 6300 2350 50  0000 L CNN
F 1 "1u" H 6300 2150 50  0000 L CNN
	1    6250 2250
	1    0    0    -1  
$EndComp
$Comp
L C C7
U 1 1 4D39EC4F
P 5950 2250
F 0 "C7" H 6000 2350 50  0000 L CNN
F 1 "100n" H 6000 2150 50  0000 L CNN
	1    5950 2250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 4D39EBDF
P 5350 2500
F 0 "#PWR03" H 5350 2500 30  0001 C CNN
F 1 "GND" H 5350 2430 30  0001 C CNN
	1    5350 2500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR04
U 1 1 4D39EBDA
P 5650 2500
F 0 "#PWR04" H 5650 2500 30  0001 C CNN
F 1 "GND" H 5650 2430 30  0001 C CNN
	1    5650 2500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 4D39EBD6
P 2800 2500
F 0 "#PWR05" H 2800 2500 30  0001 C CNN
F 1 "GND" H 2800 2430 30  0001 C CNN
	1    2800 2500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 4D39EBD2
P 3100 2500
F 0 "#PWR06" H 3100 2500 30  0001 C CNN
F 1 "GND" H 3100 2430 30  0001 C CNN
	1    3100 2500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 4D39EBCE
P 5050 2500
F 0 "#PWR07" H 5050 2500 30  0001 C CNN
F 1 "GND" H 5050 2430 30  0001 C CNN
	1    5050 2500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 4D39EBCB
P 4750 2500
F 0 "#PWR08" H 4750 2500 30  0001 C CNN
F 1 "GND" H 4750 2430 30  0001 C CNN
	1    4750 2500
	1    0    0    -1  
$EndComp
$Comp
L C C6
U 1 1 4D39EBBD
P 5650 2250
F 0 "C6" H 5700 2350 50  0000 L CNN
F 1 "1u" H 5700 2150 50  0000 L CNN
	1    5650 2250
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 4D39EBB6
P 5350 2250
F 0 "C5" H 5400 2350 50  0000 L CNN
F 1 "100n" H 5400 2150 50  0000 L CNN
	1    5350 2250
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 4D39EBB0
P 2800 2250
F 0 "C1" H 2850 2350 50  0000 L CNN
F 1 "100n" H 2850 2150 50  0000 L CNN
	1    2800 2250
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 4D39EBA9
P 3100 2250
F 0 "C2" H 3150 2350 50  0000 L CNN
F 1 "1u" H 3150 2150 50  0000 L CNN
	1    3100 2250
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 4D39EBA0
P 5050 2250
F 0 "C4" H 5100 2350 50  0000 L CNN
F 1 "180p" H 5100 2150 50  0000 L CNN
	1    5050 2250
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 4D39EB9A
P 4750 2250
F 0 "C3" H 4800 2350 50  0000 L CNN
F 1 "180p" H 4800 2150 50  0000 L CNN
	1    4750 2250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 4D39EB78
P 4450 2350
F 0 "#PWR09" H 4450 2350 30  0001 C CNN
F 1 "GND" H 4450 2280 30  0001 C CNN
	1    4450 2350
	1    0    0    -1  
$EndComp
$Comp
L CONN_5 P1
U 1 1 4D11FCE8
P 2350 1750
F 0 "P1" V 2300 1750 50  0000 C CNN
F 1 "CONN_5" V 2400 1750 50  0000 C CNN
	1    2350 1750
	-1   0    0    -1  
$EndComp
$Comp
L CONN_5 P2
U 1 1 4D11FCE2
P 7250 1750
F 0 "P2" V 7200 1750 50  0000 C CNN
F 1 "CONN_5" V 7300 1750 50  0000 C CNN
	1    7250 1750
	1    0    0    -1  
$EndComp
$Comp
L CS5343 U1
U 1 1 4D11FCCB
P 3750 1750
F 0 "U1" H 3750 1400 60  0000 C CNN
F 1 "CS5343" H 3750 2100 60  0000 C CNN
F 2 "TSSOP-10" H 3650 1300 60  0001 C CNN
F 3 "www.cirrus.com/en/pubs/proDatasheet/CS5343_F3.pdf" H 3800 1200 60  0001 C CNN
	1    3750 1750
	1    0    0    -1  
$EndComp
$EndSCHEMATC

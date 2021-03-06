EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 1
Title "LucidSens"
Date "2020-11-02"
Rev "1.05v"
Comp "M.Amin Haghighatbin"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L power:GND #PWR0101
U 1 1 5F5AB4E7
P 4255 1595
F 0 "#PWR0101" H 4255 1345 50  0001 C CNN
F 1 "GND" H 4255 1445 50  0000 C CNN
F 2 "" H 4255 1595 50  0001 C CNN
F 3 "" H 4255 1595 50  0001 C CNN
	1    4255 1595
	-1   0    0    1   
$EndComp
Wire Wire Line
	4180 1600 4180 1700
$Comp
L Device:C C12
U 1 1 5F5BCC23
P 1890 1255
F 0 "C12" V 1945 1050 50  0000 L CNN
F 1 "0.1uF" V 2025 1150 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 1928 1105 50  0001 C CNN
F 3 "~" H 1890 1255 50  0001 C CNN
	1    1890 1255
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5F5BD81C
P 2140 1255
F 0 "#PWR0103" H 2140 1005 50  0001 C CNN
F 1 "GND" H 2145 1082 50  0000 C CNN
F 2 "" H 2140 1255 50  0001 C CNN
F 3 "" H 2140 1255 50  0001 C CNN
	1    2140 1255
	1    0    0    -1  
$EndComp
Wire Wire Line
	2040 1255 2140 1255
$Comp
L power:GND #PWR0104
U 1 1 5F5C6CA5
P 2325 1600
F 0 "#PWR0104" H 2325 1350 50  0001 C CNN
F 1 "GND" H 2320 1450 50  0000 C CNN
F 2 "" H 2325 1600 50  0001 C CNN
F 3 "" H 2325 1600 50  0001 C CNN
	1    2325 1600
	0    1    1    0   
$EndComp
Wire Wire Line
	1540 1255 1640 1255
Text GLabel 2325 1800 0    50   Output ~ 0
EN
Text GLabel 1640 1105 1    50   Input ~ 0
EN
Wire Wire Line
	1640 1105 1640 1255
Connection ~ 1640 1255
Wire Wire Line
	1640 1255 1740 1255
Text GLabel 4075 3000 2    50   Output ~ 0
IO0
Text GLabel 4680 2000 2    50   BiDi ~ 0
RXD
Text GLabel 4680 2100 2    50   BiDi ~ 0
TXD
Text GLabel 2330 2200 0    50   Input ~ 0
IO35
Text GLabel 4075 2900 2    50   Output ~ 0
IO4
Text GLabel 4075 2500 2    50   BiDi ~ 0
IO18
Text GLabel 4075 1800 2    50   BiDi ~ 0
IO23
Text GLabel 4075 3200 2    50   Output ~ 0
IO15
Text GLabel 4075 2600 2    50   Output ~ 0
IO5
Text GLabel 2325 3100 0    50   Output ~ 0
IO13
Text GLabel 2325 2700 0    50   Input ~ 0
IO27
Text GLabel 2325 2800 0    50   Input ~ 0
IO14
Wire Wire Line
	2325 2200 2330 2200
Wire Wire Line
	4075 1600 4180 1600
Wire Wire Line
	4075 1700 4180 1700
Connection ~ 4180 1700
Wire Wire Line
	4180 1700 4255 1700
Wire Wire Line
	4255 1595 4255 1700
Text GLabel 4075 2200 2    50   Output ~ 0
IO21
Text GLabel 4075 1900 2    50   Output ~ 0
IO22
$Comp
L power:GND #PWR0134
U 1 1 652786D1
P 2325 3000
F 0 "#PWR0134" H 2325 2750 50  0001 C CNN
F 1 "GND" V 2330 2872 50  0000 R CNN
F 2 "" H 2325 3000 50  0001 C CNN
F 3 "" H 2325 3000 50  0001 C CNN
	1    2325 3000
	0    1    1    0   
$EndComp
Text GLabel 2325 2300 0    50   Output ~ 0
IO32
Text GLabel 2325 2400 0    50   Output ~ 0
IO33
Text GLabel 2325 1900 0    50   Input ~ 0
IO36
NoConn ~ 2325 3200
NoConn ~ 2325 3300
NoConn ~ 2325 3400
NoConn ~ 4075 3300
NoConn ~ 4075 3400
NoConn ~ 4075 3500
Text GLabel 2325 2000 0    50   Input ~ 0
IO39
Text GLabel 2325 2100 0    50   Input ~ 0
IO34
Text GLabel 2325 2500 0    50   Output ~ 0
IO25
Text Notes 4215 890  2    100  ~ 20
ESP32 WROVER-I Module
$Comp
L power:+3.3V #PWR0177
U 1 1 601A072A
P 1240 1255
F 0 "#PWR0177" H 1240 1105 50  0001 C CNN
F 1 "+3.3V" V 1245 1370 50  0000 L CNN
F 2 "" H 1240 1255 50  0001 C CNN
F 3 "" H 1240 1255 50  0001 C CNN
	1    1240 1255
	0    -1   -1   0   
$EndComp
Text Notes 1595 2530 0    50   ~ 10
DAC(HV-DC)
Text Notes 1345 2430 0    50   ~ 10
DIR_MAIN_STEPPER
Text Notes 1335 2340 0    50   ~ 10
STP_MAIN_STEPPER
Text Notes 1580 2240 0    50   ~ 10
OPTOSWITCH
Text Notes 1435 2140 0    50   ~ 10
SiPM(SLOW_OUT)
Text Notes 1450 2035 0    50   ~ 10
SiPM(FAST_OUT)
Text Notes 1665 1940 0    50   ~ 10
HV_CHECK
Text Notes 1335 3145 0    50   ~ 10
PWR_MAIN_STEPPER
Text Notes 4340 2940 0    50   ~ 10
BUZZER
Text Notes 4345 3245 0    50   ~ 10
DC(TFT)
Text Notes 4360 1940 0    50   ~ 10
PELTIER_PH
Text Notes 4355 2245 0    50   ~ 10
PELTIER_EN
$Comp
L Device:R R7
U 1 1 5F5BBD56
P 1390 1255
F 0 "R7" V 1490 1205 50  0000 L CNN
F 1 "10k" V 1290 1205 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 1320 1255 50  0001 C CNN
F 3 "~" H 1390 1255 50  0001 C CNN
	1    1390 1255
	0    -1   -1   0   
$EndComp
Text Notes 1780 1065 0    31   ~ 6
50V(10%)
$Comp
L Device:R R51
U 1 1 5FA3F25E
P 4225 2000
F 0 "R51" V 4295 2005 39  0000 C CNN
F 1 "0" V 4225 1940 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 4155 2000 50  0001 C CNN
F 3 "~" H 4225 2000 50  0001 C CNN
	1    4225 2000
	0    1    1    0   
$EndComp
$Comp
L Device:R R52
U 1 1 5FA3FAD2
P 4440 2100
F 0 "R52" V 4370 2105 39  0000 C CNN
F 1 "0" V 4440 2045 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 4370 2100 50  0001 C CNN
F 3 "~" H 4440 2100 50  0001 C CNN
	1    4440 2100
	0    1    1    0   
$EndComp
Wire Wire Line
	4590 2100 4680 2100
Wire Wire Line
	4075 2100 4290 2100
Text Notes 4210 2025 0    31   ~ 6
(5%)
Text Notes 4425 2125 0    31   ~ 6
(5%)
Wire Wire Line
	4375 2000 4680 2000
$Comp
L LucidSens_2.01v-rescue:ESP32-WROVER-ESP32-WROVER-I-MinimalSens_0.04v-rescue-LucidSens_1.05v-rescue U1
U 1 1 5FA55641
P 3225 2350
F 0 "U1" H 3200 3687 60  0000 C CNN
F 1 "ESP32-WROVER" H 3200 3581 60  0000 C CNN
F 2 "ESP32-WROVER-I:ESP32-WROVER" H 3675 2050 60  0001 C CNN
F 3 "" H 3675 2050 60  0001 C CNN
	1    3225 2350
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x05_Male J?
U 1 1 5FCAA0D2
P 15490 1230
AR Path="/5FC3D9D6/5FCAA0D2" Ref="J?"  Part="1" 
AR Path="/5FCAA0D2" Ref="J11"  Part="1" 
F 0 "J11" V 15505 1465 50  0000 L CNN
F 1 "Conn_01x05_Male" V 15315 915 50  0000 L CNN
F 2 "Connector_JST:JST_EH_B5B-EH-A_1x05_P2.50mm_Vertical" H 15490 1230 50  0001 C CNN
F 3 "~" H 15490 1230 50  0001 C CNN
	1    15490 1230
	0    -1   -1   0   
$EndComp
Text Notes 15415 1250 3    31   ~ 6
GND
Text Notes 15315 1245 3    31   ~ 6
3.3V
Text Notes 15715 1250 3    31   ~ 6
HV
Text Notes 15510 1240 3    31   ~ 6
IO34
Text Notes 15615 1240 3    31   ~ 6
IO39
$Comp
L power:+3.3V #PWR06
U 1 1 5FCAC44F
P 15290 1030
F 0 "#PWR06" H 15290 880 50  0001 C CNN
F 1 "+3.3V" V 15285 1245 50  0000 C CNN
F 2 "" H 15290 1030 50  0001 C CNN
F 3 "" H 15290 1030 50  0001 C CNN
	1    15290 1030
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5FCAD0F2
P 15390 1030
F 0 "#PWR07" H 15390 780 50  0001 C CNN
F 1 "GND" V 15390 840 50  0000 C CNN
F 2 "" H 15390 1030 50  0001 C CNN
F 3 "" H 15390 1030 50  0001 C CNN
	1    15390 1030
	-1   0    0    1   
$EndComp
Text GLabel 15490 980  1    50   Input ~ 0
IO34
Wire Wire Line
	15490 1030 15490 980 
Text GLabel 15590 980  1    50   Input ~ 0
IO39
Wire Wire Line
	15590 1030 15590 980 
$Comp
L Connector:Conn_01x03_Male J?
U 1 1 5FCE2DB8
P 15330 5825
AR Path="/5FC2C912/5FCE2DB8" Ref="J?"  Part="1" 
AR Path="/5FCE2DB8" Ref="J13"  Part="1" 
F 0 "J13" V 15360 5990 50  0000 L CNN
F 1 "Conn_01x03_Male" V 15150 5545 50  0000 L CNN
F 2 "Connector_JST:JST_EH_B3B-EH-A_1x03_P2.50mm_Vertical" H 15330 5825 50  0001 C CNN
F 3 "~" H 15330 5825 50  0001 C CNN
	1    15330 5825
	0    -1   -1   0   
$EndComp
Text Notes 15255 5845 3    31   ~ 6
3.3V
Text Notes 15445 5845 3    31   ~ 6
GND
Text Notes 15355 5840 3    31   ~ 6
IO35
Text GLabel 15330 5625 1    50   Input ~ 0
IO35
$Comp
L power:+3.3V #PWR011
U 1 1 5FCE43C0
P 15230 5625
F 0 "#PWR011" H 15230 5475 50  0001 C CNN
F 1 "+3.3V" V 15225 5835 50  0000 C CNN
F 2 "" H 15230 5625 50  0001 C CNN
F 3 "" H 15230 5625 50  0001 C CNN
	1    15230 5625
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5FCE43C6
P 15430 5625
F 0 "#PWR010" H 15430 5375 50  0001 C CNN
F 1 "GND" V 15420 5435 50  0000 C CNN
F 2 "" H 15430 5625 50  0001 C CNN
F 3 "" H 15430 5625 50  0001 C CNN
	1    15430 5625
	-1   0    0    1   
$EndComp
$Comp
L Device:C C7
U 1 1 6045A388
P 995 1850
F 0 "C7" H 1205 1850 50  0000 R CNN
F 1 "22uF" H 1265 1940 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 1033 1700 50  0001 C CNN
F 3 "~" H 995 1850 50  0001 C CNN
	1    995  1850
	-1   0    0    1   
$EndComp
$Comp
L Device:C C10
U 1 1 6045B510
P 1405 1850
F 0 "C10" H 1175 1870 50  0000 L CNN
F 1 "0.1uF" H 1185 1760 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 1443 1700 50  0001 C CNN
F 3 "~" H 1405 1850 50  0001 C CNN
	1    1405 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2325 1700 1405 1700
Wire Wire Line
	995  1700 1405 1700
Connection ~ 1405 1700
Wire Wire Line
	1405 2000 995  2000
$Comp
L power:GND #PWR013
U 1 1 6046FAD4
P 995 2070
F 0 "#PWR013" H 995 1820 50  0001 C CNN
F 1 "GND" H 1000 1897 50  0000 C CNN
F 2 "" H 995 2070 50  0001 C CNN
F 3 "" H 995 2070 50  0001 C CNN
	1    995  2070
	1    0    0    -1  
$EndComp
Wire Wire Line
	995  2000 995  2070
Connection ~ 995  2000
$Comp
L power:+3.3V #PWR012
U 1 1 60478DE1
P 925 1700
F 0 "#PWR012" H 925 1550 50  0001 C CNN
F 1 "+3.3V" V 930 1815 50  0000 L CNN
F 2 "" H 925 1700 50  0001 C CNN
F 3 "" H 925 1700 50  0001 C CNN
	1    925  1700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	995  1700 925  1700
Connection ~ 995  1700
Text Notes 1285 1790 0    31   ~ 0
50V(10%)
Text Notes 915  1790 0    24   ~ 0
10V(20%)
Text GLabel 15690 1030 1    50   Input ~ 0
HV_OUT
Text Notes 1565 2850 0    50   ~ 10
COOLING_FAN
Text Notes 4355 2545 0    50   ~ 10
SCL(TFT)
Text Notes 4355 1840 0    50   ~ 10
SDA(TFT)
Text Notes 4350 2645 0    50   ~ 10
CS(TFT)
Text Notes 4345 3045 0    50   ~ 10
AutoProgramming
Text Notes 1535 2740 0    50   ~ 10
TEMP_SENSOR
$Comp
L Connector:TestPoint TP1
U 1 1 60AF2062
P 995 1700
F 0 "TP1" H 1053 1818 50  0000 L CNN
F 1 "TestPoint" H 875 1910 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 1195 1700 50  0001 C CNN
F 3 "~" H 1195 1700 50  0001 C CNN
	1    995  1700
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR09
U 1 1 5FCD88F1
P 12045 7090
F 0 "#PWR09" H 12045 6940 50  0001 C CNN
F 1 "+3.3V" V 12040 7300 50  0000 C CNN
F 2 "" H 12045 7090 50  0001 C CNN
F 3 "" H 12045 7090 50  0001 C CNN
	1    12045 7090
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 5FCD7180
P 12245 7090
F 0 "#PWR08" H 12245 6840 50  0001 C CNN
F 1 "GND" V 12235 6900 50  0000 C CNN
F 2 "" H 12245 7090 50  0001 C CNN
F 3 "" H 12245 7090 50  0001 C CNN
	1    12245 7090
	-1   0    0    1   
$EndComp
Text GLabel 12145 7090 1    50   Input ~ 0
IO27
Text Notes 12275 7305 3    31   ~ 6
GND
Text Notes 12075 7310 3    31   ~ 6
3.3V
Text Notes 12175 7305 3    31   ~ 6
IO27
$Comp
L Connector:Conn_01x03_Male J?
U 1 1 5FCD695B
P 12145 7290
AR Path="/5FC33348/5FCD695B" Ref="J?"  Part="1" 
AR Path="/5FCD695B" Ref="J12"  Part="1" 
F 0 "J12" V 12155 7420 50  0000 L CNN
F 1 "Conn_01x03_Male" V 11930 7045 50  0000 L CNN
F 2 "Connector_JST:JST_EH_B3B-EH-A_1x03_P2.50mm_Vertical" H 12145 7290 50  0001 C CNN
F 3 "~" H 12145 7290 50  0001 C CNN
	1    12145 7290
	0    -1   -1   0   
$EndComp
$Comp
L Connector:TestPoint TP2
U 1 1 60AF28FF
P 1640 1255
F 0 "TP2" H 1698 1373 50  0000 L CNN
F 1 "TestPoint" H 1520 1465 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 1840 1255 50  0001 C CNN
F 3 "~" H 1840 1255 50  0001 C CNN
	1    1640 1255
	-1   0    0    1   
$EndComp
Text Notes 13880 4815 0    31   ~ 0
#?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????-???????????????????????????-??????-#\n\nWith OPA354 as the op-amp (inverting TIA) the circuit requirements will be:\nFor the V_output range of:                                              0-3 V\nand the maximum Current of:                                           10.0 uA\nRF for MicroFC-SMTPA-60035 should be:                               =<300 kohm\nthe f-3dB is equal to:                                                  0.20 MHz\nCF value at f_3dB=0.2 MHz will be:                                     3 pF\nFor an ideal BW of 100.0 kHz the  CF will be:                          5 pF\n\n#?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????-???????????????????????????#\n\nWith OPA354 as the op-amp (inverting TIA) the circuit requirements will be:\nFor the V_output range of:                                              0-3 V\nand the maximum Current of:                                           20.0 uA\nRF for MicroFC-SMTPA-60035 should be:                               =<150 kohm\nthe f-3dB is equal to:                                                  0.28 MHz\nCF value at f_3dB=0.28 MHz will be:                                    4 pF\nFor an ideal BW of 100.0 kHz the  CF will be:                          11 pF\n\n#????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????-????????????-??????????????????-#\n\n* Datasheet suggests 470 ohm for 6mm SiPM.
Connection ~ 14460 2070
Wire Wire Line
	14460 1970 14460 2070
$Comp
L power:PWR_FLAG #FLG0111
U 1 1 60B04D5D
P 14460 1970
F 0 "#FLG0111" H 14460 2045 50  0001 C CNN
F 1 "PWR_FLAG" H 14460 2110 50  0000 C CNN
F 2 "" H 14460 1970 50  0001 C CNN
F 3 "~" H 14460 1970 50  0001 C CNN
	1    14460 1970
	1    0    0    -1  
$EndComp
Wire Wire Line
	14170 1775 14770 1775
$Comp
L power:PWR_FLAG #FLG0110
U 1 1 5FCEC4F3
P 14170 1775
F 0 "#FLG0110" H 14170 1850 50  0001 C CNN
F 1 "PWR_FLAG" H 14170 1948 50  0000 C CNN
F 2 "" H 14170 1775 50  0001 C CNN
F 3 "~" H 14170 1775 50  0001 C CNN
	1    14170 1775
	1    0    0    -1  
$EndComp
Text Notes 12300 1015 0    100  ~ 20
SiPM Module / Signal Amplification 
Wire Wire Line
	12725 2505 12435 2505
Wire Wire Line
	13025 2505 13460 2505
Text Notes 13815 1760 0    50   ~ 10
3.3V
Text Notes 13620 2190 0    50   ~ 10
GND
Connection ~ 13570 2070
Wire Wire Line
	13570 2875 13570 2070
Text Notes 12690 1810 0    50   ~ 10
HV_OUT
Wire Wire Line
	13400 1590 13400 1840
Text Notes 13525 1380 1    31   ~ 6
IO39
Text Notes 13625 1380 1    31   ~ 6
IO34
Text Notes 13425 1380 1    31   ~ 6
HV
Text Notes 13825 1380 1    31   ~ 6
3.3V
Text Notes 13725 1380 1    31   ~ 6
GND
Wire Wire Line
	13600 1590 13600 1990
Wire Wire Line
	15485 1990 15485 3275
Wire Wire Line
	15485 1990 13600 1990
Wire Wire Line
	12065 1985 13500 1985
Wire Wire Line
	13500 1985 13500 1590
Wire Wire Line
	12065 3350 12065 1985
Connection ~ 14170 1775
Wire Wire Line
	13800 1775 14170 1775
Wire Wire Line
	14455 2975 14770 2975
Wire Wire Line
	14770 2975 14770 1775
Wire Wire Line
	13800 1775 13800 1590
$Comp
L Connector:Conn_01x05_Male J16
U 1 1 60B04D5B
P 13600 1390
F 0 "J16" V 13615 1625 50  0000 L CNN
F 1 "Conn_01x05_Male" V 13820 1070 50  0000 L CNN
F 2 "Connector_JST:JST_EH_B5B-EH-A_1x05_P2.50mm_Vertical" H 13600 1390 50  0001 C CNN
F 3 "~" H 13600 1390 50  0001 C CNN
	1    13600 1390
	0    1    1    0   
$EndComp
Wire Wire Line
	13950 2075 13950 2070
Connection ~ 13950 2070
Wire Wire Line
	13945 3575 13950 2075
Wire Wire Line
	14455 3575 13945 3575
Wire Wire Line
	12370 1840 13400 1840
Wire Wire Line
	13365 2070 13570 2070
Wire Wire Line
	13365 2070 12220 2070
Connection ~ 13365 2070
Wire Wire Line
	13365 2070 13365 2975
Wire Wire Line
	12220 2070 12220 2765
Wire Wire Line
	12370 1840 12370 3075
Wire Wire Line
	12220 3350 12065 3350
Wire Wire Line
	12220 3350 12405 3350
Connection ~ 12220 3350
Wire Wire Line
	12220 3065 12220 3350
Wire Wire Line
	13825 2070 13950 2070
Connection ~ 13825 2070
Wire Wire Line
	13825 2665 13825 2070
Wire Wire Line
	14070 2070 14460 2070
Wire Wire Line
	13950 2070 14070 2070
Connection ~ 14070 2070
Wire Wire Line
	14070 2875 14070 2070
Wire Wire Line
	15380 2070 15380 2765
Wire Wire Line
	14460 2070 14945 2070
Wire Wire Line
	15380 2070 14945 2070
Connection ~ 14945 2070
Wire Wire Line
	14945 2070 14945 2095
Wire Wire Line
	13570 2070 13700 2070
Wire Wire Line
	13700 2070 13825 2070
Connection ~ 13700 2070
Wire Wire Line
	13700 1590 13700 2070
Wire Wire Line
	12435 2505 12435 3075
Wire Wire Line
	13460 2505 13460 3175
$Comp
L Device:C C?
U 1 1 60B04D5A
P 12875 2505
AR Path="/5FBD9187/60B04D5A" Ref="C?"  Part="1" 
AR Path="/60B04D5A" Ref="C26"  Part="1" 
AR Path="/5FC3D9D6/60B04D5A" Ref="C26"  Part="1" 
F 0 "C26" V 12623 2505 50  0000 C CNN
F 1 "10nF" V 12714 2505 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 12913 2355 50  0001 C CNN
F 3 "~" H 12875 2505 50  0001 C CNN
	1    12875 2505
	0    1    1    0   
$EndComp
Wire Wire Line
	12370 3075 12435 3075
Wire Wire Line
	12435 3075 12440 3075
Connection ~ 12435 3075
$Comp
L LucidSens_2.01v-rescue:uFC-SMTPA-60035-MicroFC-SMTPA-6mm-MinimalSens_0.04v-rescue-LucidSens_1.05v-rescue U?
U 1 1 5FC43685
P 13290 2875
AR Path="/5FBD9187/5FC43685" Ref="U?"  Part="1" 
AR Path="/5FC43685" Ref="U7"  Part="1" 
AR Path="/5FC3D9D6/5FC43685" Ref="U7"  Part="1" 
F 0 "U7" V 13475 2450 100 0000 C CNN
F 1 "uFC-SMTPA-60035" V 13355 2450 39  0000 C CNN
F 2 "uFC-SMTPA-6mm:uFC-SMTPA-6mm" H 13290 2875 100 0001 C CNN
F 3 "" H 13290 2875 100 0001 C CNN
	1    13290 2875
	0    1    -1   0   
$EndComp
Text Notes 15540 2810 2    31   ~ 6
6.6-10V(10%)
Text Notes 15050 2100 3    31   ~ 6
(5%)
Text Notes 14690 2595 2    31   ~ 6
(5%)
Text Notes 13925 2705 3    31   ~ 6
(5%)
Text Notes 13975 2900 0    31   ~ 6
10V(10%)
Text Notes 13700 2895 2    31   ~ 6
10V(10%)
Text Notes 12125 2725 3    31   ~ 6
6.6-10V(10%)
Text Notes 15125 3455 0    31   ~ 6
(5%)
Text Notes 12400 1975 0    50   ~ 10
ADC/FastO
Text Notes 14805 1960 0    50   ~ 10
ADC-Slow/O
Connection ~ 13825 3175
Wire Wire Line
	13825 2965 13825 3175
Wire Wire Line
	12405 3175 12440 3175
Wire Wire Line
	12405 3350 12405 3175
$Comp
L Device:C C25
U 1 1 5FC43662
P 12220 2915
AR Path="/5FC43662" Ref="C25"  Part="1" 
AR Path="/5FBD9187/5FC43662" Ref="C?"  Part="1" 
AR Path="/5FC3D9D6/5FC43662" Ref="C25"  Part="1" 
F 0 "C25" H 12235 3015 50  0000 L CNN
F 1 "10nF" H 12130 2845 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 12258 2765 50  0001 C CNN
F 3 "~" H 12220 2915 50  0001 C CNN
	1    12220 2915
	1    0    0    1   
$EndComp
Text Notes 12405 3450 2    50   ~ 10
Fast_out\n
Connection ~ 13460 3175
Wire Wire Line
	13290 3175 13460 3175
Text Notes 13270 3275 0    50   ~ 10
Slow_out
Wire Wire Line
	15380 3275 15485 3275
Wire Wire Line
	15335 3275 15380 3275
Connection ~ 15380 3275
Wire Wire Line
	15380 3275 15380 3065
$Comp
L Device:C C36
U 1 1 5FC4364B
P 15380 2915
AR Path="/5FC4364B" Ref="C36"  Part="1" 
AR Path="/5FBD9187/5FC4364B" Ref="C?"  Part="1" 
AR Path="/5FC3D9D6/5FC4364B" Ref="C36"  Part="1" 
F 0 "C36" H 15375 3000 50  0000 L CNN
F 1 "0.1uF" H 15285 2845 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 15418 2765 50  0001 C CNN
F 3 "~" H 15380 2915 50  0001 C CNN
	1    15380 2915
	-1   0    0    1   
$EndComp
Wire Wire Line
	14070 3175 14255 3175
Wire Wire Line
	13825 3175 14070 3175
Connection ~ 14070 3175
$Comp
L Device:C C31
U 1 1 5FC4363F
P 14070 3025
AR Path="/5FC4363F" Ref="C31"  Part="1" 
AR Path="/5FBD9187/5FC4363F" Ref="C?"  Part="1" 
AR Path="/5FC3D9D6/5FC4363F" Ref="C31"  Part="1" 
F 0 "C31" H 14080 2930 50  0000 L CNN
F 1 "40pF" H 13995 3110 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 14108 2875 50  0001 C CNN
F 3 "~" H 14070 3025 50  0001 C CNN
	1    14070 3025
	1    0    0    -1  
$EndComp
Wire Wire Line
	14945 3275 15035 3275
$Comp
L Device:R R38
U 1 1 60B04D55
P 15185 3275
AR Path="/60B04D55" Ref="R38"  Part="1" 
AR Path="/5FBD9187/60B04D55" Ref="R?"  Part="1" 
AR Path="/5FC3D9D6/60B04D55" Ref="R38"  Part="1" 
F 0 "R38" V 15105 3275 50  0000 C CNN
F 1 "50" V 15275 3275 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 15115 3275 50  0001 C CNN
F 3 "~" H 15185 3275 50  0001 C CNN
	1    15185 3275
	0    1    1    0   
$EndComp
$Comp
L Device:R R35
U 1 1 60B04D54
P 14945 2245
AR Path="/60B04D54" Ref="R35"  Part="1" 
AR Path="/5FBD9187/60B04D54" Ref="R?"  Part="1" 
AR Path="/5FC3D9D6/60B04D54" Ref="R35"  Part="1" 
F 0 "R35" V 14860 2165 50  0000 L CNN
F 1 "100" V 15025 2135 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 14875 2245 50  0001 C CNN
F 3 "~" H 14945 2245 50  0001 C CNN
	1    14945 2245
	1    0    0    -1  
$EndComp
Wire Wire Line
	14390 2425 14185 2425
Wire Wire Line
	14945 2395 14945 2425
Connection ~ 14945 2425
Wire Wire Line
	14690 2425 14945 2425
$Comp
L Device:C C35
U 1 1 5FC4361E
P 14540 2425
AR Path="/5FC4361E" Ref="C35"  Part="1" 
AR Path="/5FBD9187/5FC4361E" Ref="C?"  Part="1" 
AR Path="/5FC3D9D6/5FC4361E" Ref="C35"  Part="1" 
F 0 "C35" V 14490 2540 50  0000 C CNN
F 1 "4pF" V 14675 2425 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 14578 2275 50  0001 C CNN
F 3 "~" H 14540 2425 50  0001 C CNN
	1    14540 2425
	0    -1   -1   0   
$EndComp
Wire Wire Line
	14185 2665 14185 3375
Wire Wire Line
	14185 2425 14185 2665
Connection ~ 14185 2665
Wire Wire Line
	14185 2665 14405 2665
Connection ~ 14945 3275
Wire Wire Line
	14855 3275 14945 3275
Wire Wire Line
	14945 2665 14945 3275
Wire Wire Line
	14945 2425 14945 2665
Connection ~ 14945 2665
Wire Wire Line
	14945 2665 14705 2665
Wire Wire Line
	14255 3375 14185 3375
$Comp
L Device:R R32
U 1 1 5FC43612
P 14555 2665
AR Path="/5FC43612" Ref="R32"  Part="1" 
AR Path="/5FBD9187/5FC43612" Ref="R?"  Part="1" 
AR Path="/5FC3D9D6/5FC43612" Ref="R32"  Part="1" 
F 0 "R32" V 14635 2655 50  0000 C CNN
F 1 "157k" V 14465 2600 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 14485 2665 50  0001 C CNN
F 3 "~" H 14555 2665 50  0001 C CNN
	1    14555 2665
	0    1    1    0   
$EndComp
$Comp
L Device:R R31
U 1 1 60B04D51
P 13825 2815
AR Path="/60B04D51" Ref="R31"  Part="1" 
AR Path="/5FBD9187/60B04D51" Ref="R?"  Part="1" 
AR Path="/5FC3D9D6/60B04D51" Ref="R31"  Part="1" 
F 0 "R31" V 13745 2830 50  0000 L CNN
F 1 "100" V 13820 2755 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 13755 2815 50  0001 C CNN
F 3 "~" H 13825 2815 50  0001 C CNN
	1    13825 2815
	-1   0    0    1   
$EndComp
Wire Wire Line
	13460 3175 13570 3175
Wire Wire Line
	13570 3175 13825 3175
Connection ~ 13570 3175
$Comp
L Device:C C27
U 1 1 5FC435EE
P 13570 3025
AR Path="/5FC435EE" Ref="C27"  Part="1" 
AR Path="/5FBD9187/5FC435EE" Ref="C?"  Part="1" 
AR Path="/5FC3D9D6/5FC435EE" Ref="C27"  Part="1" 
F 0 "C27" H 13540 3125 50  0000 R CNN
F 1 "100pF" H 13670 2945 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 13608 2875 50  0001 C CNN
F 3 "~" H 13570 3025 50  0001 C CNN
	1    13570 3025
	-1   0    0    1   
$EndComp
$Comp
L Amplifier_Operational:OPA356xxDBV U9
U 1 1 5FC435E8
P 14555 3275
AR Path="/5FC435E8" Ref="U9"  Part="1" 
AR Path="/5FBD9187/5FC435E8" Ref="U?"  Part="1" 
AR Path="/5FC3D9D6/5FC435E8" Ref="U9"  Part="1" 
F 0 "U9" H 14630 3385 50  0000 L CNN
F 1 "OPA354" H 14580 3160 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 14455 3075 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/opa356.pdf" H 14555 3475 50  0001 C CNN
	1    14555 3275
	1    0    0    -1  
$EndComp
Wire Wire Line
	13365 2975 13290 2975
Text Notes 9875 2770 0    31   ~ 6
(10%)
Text Notes 6945 3355 1    31   ~ 6
(5%)
$Comp
L LucidSens_2.01v-rescue:ZM4733A-GS18-ZM4733A-GS18-MinimalSens_0.04v-rescue-LucidSens_1.05v-rescue D2
U 1 1 5FAFB85A
P 10635 2525
F 0 "D2" H 10780 2475 50  0000 C CNN
F 1 "ZM4733A-GS18" H 10925 2660 50  0000 C CNN
F 2 "ZM4733A-GS18:MELF_DO-213AB" H 11085 2525 50  0001 L CNN
F 3 "https://www.vishay.com/docs/85786/zm4728a.pdf" H 11085 2425 50  0001 L CNN
F 4 "VISHAY - ZM4733A-GS18 - ZENER DIODE, 1W, 5.1V, DO-213" H 11085 2325 50  0001 L CNN "Description"
F 5 "" H 11085 2225 50  0001 L CNN "Height"
F 6 "Vishay" H 11085 2125 50  0001 L CNN "Manufacturer_Name"
F 7 "ZM4733A-GS18" H 11085 2025 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "ZM4733A-GS18" H 11085 1925 50  0001 L CNN "Arrow Part Number"
F 9 "https://www.arrow.com/en/products/zm4733a-gs18/vishay" H 11085 1825 50  0001 L CNN "Arrow Price/Stock"
F 10 "625-ZM4733A-GS18" H 11085 1725 50  0001 L CNN "Mouser Part Number"
F 11 "https://www.mouser.co.uk/ProductDetail/Vishay-Semiconductors/ZM4733A-GS18?qs=6Fe8ASSgjNulBGqz5bPVMQ%3D%3D" H 11085 1625 50  0001 L CNN "Mouser Price/Stock"
	1    10635 2525
	1    0    0    -1  
$EndComp
Wire Wire Line
	11235 2525 11300 2525
Connection ~ 11235 2525
Wire Wire Line
	11235 2270 11235 2525
Wire Wire Line
	10945 2270 11235 2270
Wire Wire Line
	10845 1980 11235 1980
Connection ~ 11235 2270
Wire Wire Line
	11235 1980 11235 2270
Text Notes 6130 3525 1    31   ~ 6
(5%)
Text Notes 6870 2965 0    31   ~ 6
(10%)
Text Notes 7200 3090 0    31   ~ 6
(10%)
Text Notes 7405 3395 0    31   ~ 6
(10%)
Text Notes 7065 3860 0    31   ~ 6
(10%)
Text Notes 7485 3795 0    31   ~ 6
(10%)
Text Notes 10790 2975 0    31   ~ 6
(10%)
Text Notes 11565 3300 0    31   ~ 6
(5%)
Text Notes 10870 2675 0    31   ~ 6
5.1V
Text Notes 10825 1825 0    31   ~ 6
(5%)
Text Notes 10895 2335 0    31   ~ 6
(5%)
Text Notes 11685 2820 0    31   ~ 6
(5%)
Text Notes 9205 2775 0    31   ~ 6
(1%)
Text Notes 9415 2175 2    31   ~ 6
(1%)
Text Notes 7410 2130 0    31   ~ 6
(5%)
Text Notes 7410 2030 0    31   ~ 6
(5%)
Text Notes 7420 1730 0    31   ~ 6
(5%)
Text Notes 7425 1830 0    31   ~ 6
(5%)
Text Notes 7395 2330 0    31   ~ 6
(5%)
$Comp
L Device:C C8
U 1 1 5FB31702
P 7350 2990
F 0 "C8" H 7080 2995 50  0000 L CNN
F 1 "10nF" H 7015 2920 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 7388 2840 50  0001 C CNN
F 3 "~" H 7350 2990 50  0001 C CNN
	1    7350 2990
	1    0    0    -1  
$EndComp
Wire Wire Line
	7480 2840 7480 2940
Wire Wire Line
	7480 2940 7580 2940
Wire Wire Line
	7480 3140 7480 3040
Wire Wire Line
	7480 3040 7580 3040
$Comp
L power:GND #PWR0121
U 1 1 5FB316F8
P 8950 3540
F 0 "#PWR0121" H 8950 3290 50  0001 C CNN
F 1 "GND" H 8815 3455 50  0000 C CNN
F 2 "" H 8950 3540 50  0001 C CNN
F 3 "" H 8950 3540 50  0001 C CNN
	1    8950 3540
	1    0    0    -1  
$EndComp
Wire Wire Line
	8780 3440 8880 3440
Wire Wire Line
	8880 3440 8880 3540
Wire Wire Line
	8880 3340 8880 3440
Connection ~ 8880 3440
NoConn ~ 8780 3140
NoConn ~ 7580 2240
$Comp
L Device:R_POT_US RV1
U 1 1 5FB316E9
P 9830 2640
F 0 "RV1" H 9785 2595 50  0000 R CNN
F 1 "10k" H 9785 2685 50  0000 R CNN
F 2 "SM-43TW103:SM43TW103" H 9830 2640 50  0001 C CNN
F 3 "~" H 9830 2640 50  0001 C CNN
	1    9830 2640
	-1   0    0    1   
$EndComp
Wire Wire Line
	8780 2740 8930 2740
Wire Wire Line
	8930 2640 8930 2740
Text GLabel 9830 2290 1    50   Input ~ 0
DRV3V3
Wire Wire Line
	9830 2290 9830 2490
Text GLabel 9130 2790 1    50   Output ~ 0
DRV3V3
Wire Wire Line
	8780 2940 8920 2940
$Comp
L Device:C C9
U 1 1 5FB316DB
P 9130 3140
F 0 "C9" H 8850 3220 50  0000 L CNN
F 1 "0.47uF" H 8850 3055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 9168 2990 50  0001 C CNN
F 3 "~" H 9130 3140 50  0001 C CNN
	1    9130 3140
	1    0    0    -1  
$EndComp
$Comp
L Device:R R12
U 1 1 60B14D54
P 9330 2890
F 0 "R12" V 9245 2810 50  0000 L CNN
F 1 "0.1" V 9330 2830 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 9260 2890 50  0001 C CNN
F 3 "~" H 9330 2890 50  0001 C CNN
	1    9330 2890
	1    0    0    -1  
$EndComp
$Comp
L Device:R R13
U 1 1 60B14D53
P 9430 2290
F 0 "R13" V 9345 2210 50  0000 L CNN
F 1 "0.1" V 9430 2230 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 9360 2290 50  0001 C CNN
F 3 "~" H 9430 2290 50  0001 C CNN
	1    9430 2290
	1    0    0    -1  
$EndComp
Wire Wire Line
	9430 1940 9430 2140
Wire Wire Line
	8780 2440 9225 2440
Wire Wire Line
	9330 2440 9330 2740
Wire Wire Line
	8780 2640 8930 2640
Wire Wire Line
	8930 2640 9515 2640
Connection ~ 8930 2640
Wire Wire Line
	9830 2790 9830 3340
$Comp
L Connector:Conn_01x04_Male J2
U 1 1 60B14D52
P 9275 1415
F 0 "J2" H 9265 1165 50  0000 L CNN
F 1 "Conn_01x04_Male" H 9150 1620 50  0000 L CNN
F 2 "Connector_Phoenix_MSTB:PhoenixContact_MSTBVA_2,5_4-G-5,08_1x04_P5.08mm_Vertical" H 9275 1415 50  0001 C CNN
F 3 "~" H 9275 1415 50  0001 C CNN
	1    9275 1415
	1    0    0    1   
$EndComp
$Comp
L Device:C C6
U 1 1 60B14D51
P 7265 3690
F 0 "C6" H 7265 3765 50  0000 L CNN
F 1 "0.1uF" H 7275 3610 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 7303 3540 50  0001 C CNN
F 3 "~" H 7265 3690 50  0001 C CNN
	1    7265 3690
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 60B14D50
P 6850 3755
F 0 "C4" H 6855 3830 50  0000 L CNN
F 1 "0.1uF" H 6855 3675 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 6888 3605 50  0001 C CNN
F 3 "~" H 6850 3755 50  0001 C CNN
	1    6850 3755
	1    0    0    -1  
$EndComp
NoConn ~ 7580 1640
Text GLabel 6965 1740 0    50   Input ~ 0
IO32
Text GLabel 6965 1840 0    50   Input ~ 0
IO33
$Comp
L Device:R R4
U 1 1 60B14D4E
P 7120 2140
F 0 "R4" V 7115 2085 50  0000 L CNN
F 1 "1.5k" V 7150 1850 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 7050 2140 50  0001 C CNN
F 3 "~" H 7120 2140 50  0001 C CNN
	1    7120 2140
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7280 2340 7580 2340
$Comp
L Device:R R8
U 1 1 5FB31663
P 7130 2340
F 0 "R8" V 7125 2285 50  0000 L CNN
F 1 "10k" V 7160 2075 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 7060 2340 50  0001 C CNN
F 3 "~" H 7130 2340 50  0001 C CNN
	1    7130 2340
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0119
U 1 1 5FB3165D
P 9130 3490
F 0 "#PWR0119" H 9130 3240 50  0001 C CNN
F 1 "GND" H 9275 3415 50  0000 C CNN
F 2 "" H 9130 3490 50  0001 C CNN
F 3 "" H 9130 3490 50  0001 C CNN
	1    9130 3490
	1    0    0    -1  
$EndComp
Wire Wire Line
	8780 3340 8880 3340
Wire Wire Line
	8880 3340 9830 3340
Connection ~ 8880 3340
Wire Wire Line
	9130 3290 9130 3440
Wire Wire Line
	9330 3040 9330 3440
Connection ~ 9130 3440
Wire Wire Line
	9130 3440 9130 3490
Wire Wire Line
	9430 2440 9430 3440
Wire Wire Line
	9330 3440 9130 3440
Wire Wire Line
	9430 3440 9330 3440
Connection ~ 9330 3440
$Comp
L power:GND #PWR0116
U 1 1 5FB3161F
P 6630 2890
F 0 "#PWR0116" H 6630 2640 50  0001 C CNN
F 1 "GND" H 6505 2820 50  0000 C CNN
F 2 "" H 6630 2890 50  0001 C CNN
F 3 "" H 6630 2890 50  0001 C CNN
	1    6630 2890
	-1   0    0    1   
$EndComp
$Comp
L Device:CP C3
U 1 1 5FB31618
P 6630 3040
F 0 "C3" H 6450 3055 50  0000 C CNN
F 1 "100uF" H 6500 2945 50  0000 C CNN
F 2 "EEE-FTH101XAL:CAPAE660X800N" H 6668 2890 50  0001 C CNN
F 3 "~" H 6630 3040 50  0001 C CNN
	1    6630 3040
	-1   0    0    1   
$EndComp
Text GLabel 6630 3590 3    50   Input ~ 0
12VMOT
Wire Wire Line
	8780 1940 9430 1940
Text Notes 7490 1095 0    100  ~ 20
MAIN Stepper driver
$Comp
L Transistor_FET:AO3401A Q3
U 1 1 60B14D49
P 10645 1980
F 0 "Q3" H 10810 2075 50  0000 L CNN
F 1 "DMP2160U" H 10200 2070 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 10845 1905 50  0001 L CIN
F 3 "http://www.aosmd.com/pdfs/datasheet/AO3401A.pdf" H 10645 1980 50  0001 L CNN
	1    10645 1980
	-1   0    0    -1  
$EndComp
Text GLabel 10545 1515 1    50   Output ~ 0
12VMOT
$Comp
L Transistor_FET:2N7002K Q4
U 1 1 60B14D48
P 11500 2625
F 0 "Q4" V 11535 2750 50  0000 L CNN
F 1 "NTR4501" V 11725 2455 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 11700 2550 50  0001 L CIN
F 3 "https://www.diodes.com/assets/Datasheets/ds30896.pdf" H 11500 2625 50  0001 L CNN
	1    11500 2625
	0    -1   -1   0   
$EndComp
Text GLabel 11610 2965 2    50   Input ~ 0
IO13
$Comp
L Device:R R17
U 1 1 5FB315FB
P 11500 3250
F 0 "R17" H 11380 3215 50  0000 C CNN
F 1 "10k" V 11495 3245 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 11430 3250 50  0001 C CNN
F 3 "~" H 11500 3250 50  0001 C CNN
	1    11500 3250
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 60B14D46
P 11860 3525
F 0 "#PWR0115" H 11860 3275 50  0001 C CNN
F 1 "GND" H 11855 3370 50  0000 C CNN
F 2 "" H 11860 3525 50  0001 C CNN
F 3 "" H 11860 3525 50  0001 C CNN
	1    11860 3525
	1    0    0    -1  
$EndComp
Wire Wire Line
	11500 3430 11500 3400
Wire Wire Line
	11700 2525 11860 2525
$Comp
L Device:R R18
U 1 1 60B14D45
P 11860 2780
F 0 "R18" H 11980 2735 50  0000 C CNN
F 1 "120" V 11855 2780 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 11790 2780 50  0001 C CNN
F 3 "~" H 11860 2780 50  0001 C CNN
	1    11860 2780
	-1   0    0    1   
$EndComp
Wire Wire Line
	11860 2525 11860 2630
Wire Wire Line
	11860 2930 11860 3430
$Comp
L Device:R R14
U 1 1 60B14D44
P 10795 2270
F 0 "R14" V 10715 2275 50  0000 C CNN
F 1 "120" V 10795 2275 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 10725 2270 50  0001 C CNN
F 3 "~" H 10795 2270 50  0001 C CNN
	1    10795 2270
	0    1    1    0   
$EndComp
Wire Wire Line
	10645 2270 10545 2270
Wire Wire Line
	10545 2270 10545 2180
$Comp
L Device:C C11
U 1 1 60B14D43
P 10695 2825
F 0 "C11" V 10730 2635 50  0000 L CNN
F 1 "0.1uF" V 10565 2720 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 10733 2675 50  0001 C CNN
F 3 "~" H 10695 2825 50  0001 C CNN
	1    10695 2825
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0114
U 1 1 5FB315D1
P 10845 2825
F 0 "#PWR0114" H 10845 2575 50  0001 C CNN
F 1 "GND" H 10845 2675 50  0000 C CNN
F 2 "" H 10845 2825 50  0001 C CNN
F 3 "" H 10845 2825 50  0001 C CNN
	1    10845 2825
	0    -1   -1   0   
$EndComp
Wire Wire Line
	10545 2825 10545 2955
Connection ~ 10545 2825
Wire Wire Line
	10545 2825 10545 2525
Wire Wire Line
	10635 2525 10545 2525
Connection ~ 10545 2270
Connection ~ 10545 2525
Wire Wire Line
	10545 2525 10545 2270
Wire Wire Line
	10545 1780 10545 1725
$Comp
L Device:R R15
U 1 1 60B14D41
P 10875 1725
F 0 "R15" V 10950 1650 50  0000 L CNN
F 1 "47k" V 10870 1655 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 10805 1725 50  0001 C CNN
F 3 "~" H 10875 1725 50  0001 C CNN
	1    10875 1725
	0    -1   -1   0   
$EndComp
Wire Wire Line
	10545 1725 10725 1725
$Comp
L power:GND #PWR0113
U 1 1 60B14D40
P 11025 1725
F 0 "#PWR0113" H 11025 1475 50  0001 C CNN
F 1 "GND" H 11030 1552 50  0000 C CNN
F 2 "" H 11025 1725 50  0001 C CNN
F 3 "" H 11025 1725 50  0001 C CNN
	1    11025 1725
	0    -1   -1   0   
$EndComp
Wire Wire Line
	11500 3100 11500 2965
Wire Wire Line
	11610 2965 11500 2965
Connection ~ 11500 2965
Wire Wire Line
	11500 2965 11500 2825
$Comp
L power:+12V #PWR0110
U 1 1 60B14D3F
P 10545 2955
F 0 "#PWR0110" H 10545 2805 50  0001 C CNN
F 1 "+12V" H 10560 3128 50  0000 C CNN
F 2 "" H 10545 2955 50  0001 C CNN
F 3 "" H 10545 2955 50  0001 C CNN
	1    10545 2955
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 5FB31596
P 6705 3240
F 0 "#FLG0102" H 6705 3315 50  0001 C CNN
F 1 "PWR_FLAG" H 6700 3375 50  0000 C CNN
F 2 "" H 6705 3240 50  0001 C CNN
F 3 "~" H 6705 3240 50  0001 C CNN
	1    6705 3240
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5FB31590
P 8920 2940
F 0 "#FLG0101" H 8920 3015 50  0001 C CNN
F 1 "PWR_FLAG" H 8900 3080 50  0000 C CNN
F 2 "" H 8920 2940 50  0001 C CNN
F 3 "~" H 8920 2940 50  0001 C CNN
	1    8920 2940
	1    0    0    -1  
$EndComp
Wire Wire Line
	9130 2940 9130 2790
Wire Wire Line
	9130 2990 9130 2940
Connection ~ 9130 2940
Connection ~ 8920 2940
Wire Wire Line
	8920 2940 9130 2940
Text GLabel 8780 1640 2    50   BiDi ~ 0
A1
Text GLabel 8780 1740 2    50   BiDi ~ 0
A2
Text GLabel 8780 2140 2    50   BiDi ~ 0
B1
Text GLabel 8780 2240 2    50   BiDi ~ 0
B2
Text GLabel 9475 1515 2    50   BiDi ~ 0
A1
Text GLabel 9475 1415 2    50   BiDi ~ 0
A2
Text GLabel 9475 1315 2    50   BiDi ~ 0
B1
Text GLabel 9475 1215 2    50   BiDi ~ 0
B2
$Comp
L power:GND #PWR0109
U 1 1 5FB3157F
P 7265 3840
F 0 "#PWR0109" H 7265 3590 50  0001 C CNN
F 1 "GND" H 7270 3690 50  0000 C CNN
F 2 "" H 7265 3840 50  0001 C CNN
F 3 "" H 7265 3840 50  0001 C CNN
	1    7265 3840
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 5FB31579
P 6850 3905
F 0 "#PWR0108" H 6850 3655 50  0001 C CNN
F 1 "GND" H 6855 3755 50  0000 C CNN
F 2 "" H 6850 3905 50  0001 C CNN
F 3 "" H 6850 3905 50  0001 C CNN
	1    6850 3905
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 60B14D3A
P 7115 1740
F 0 "R9" V 7115 1740 50  0000 C CNN
F 1 "1.2k" V 7080 1955 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 7045 1740 50  0001 C CNN
F 3 "~" H 7115 1740 50  0001 C CNN
	1    7115 1740
	0    1    1    0   
$EndComp
$Comp
L Device:R R10
U 1 1 60B14D39
P 7115 1840
F 0 "R10" V 7115 1845 50  0000 C CNN
F 1 "1.2k" V 7080 2055 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 7045 1840 50  0001 C CNN
F 3 "~" H 7115 1840 50  0001 C CNN
	1    7115 1840
	0    1    1    0   
$EndComp
Wire Wire Line
	6030 4015 6030 3915
$Comp
L power:GND #PWR0107
U 1 1 60B14D38
P 6030 4015
F 0 "#PWR0107" H 6030 3765 50  0001 C CNN
F 1 "GND" H 6030 3870 50  0000 C CNN
F 2 "" H 6030 4015 50  0001 C CNN
F 3 "" H 6030 4015 50  0001 C CNN
	1    6030 4015
	-1   0    0    -1  
$EndComp
$Comp
L Device:LED D1
U 1 1 5FB3155E
P 6030 3765
F 0 "D1" V 6030 3665 50  0000 C CNN
F 1 "LED" V 6005 3930 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 6030 3765 50  0001 C CNN
F 3 "~" H 6030 3765 50  0001 C CNN
	1    6030 3765
	0    1    -1   0   
$EndComp
$Comp
L Device:R R3
U 1 1 5FB31558
P 6030 3465
F 0 "R3" H 5930 3480 50  0000 C CNN
F 1 "750" V 6030 3470 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 5960 3465 50  0001 C CNN
F 3 "~" H 6030 3465 50  0001 C CNN
	1    6030 3465
	1    0    0    1   
$EndComp
$Comp
L Switch:SW_DIP_x01 SW2
U 1 1 60B14D35
P 6330 3310
F 0 "SW2" H 6235 3165 50  0000 L CNN
F 1 "SW_DIP_x01" H 6110 3460 50  0000 L CNN
F 2 "Button_Switch_SMD:SW_DIP_SPSTx01_Slide_Omron_A6S-110x_W8.9mm_P2.54mm" H 6330 3310 50  0001 C CNN
F 3 "~" H 6330 3310 50  0001 C CNN
	1    6330 3310
	-1   0    0    -1  
$EndComp
Text Notes 6310 3750 2    50   ~ 10
Green
Wire Wire Line
	6030 3315 6030 3310
Wire Wire Line
	8780 3540 8880 3540
Wire Wire Line
	8880 3540 8950 3540
Connection ~ 8880 3540
Wire Wire Line
	11500 3430 11860 3430
Connection ~ 11860 3430
Wire Wire Line
	11860 3430 11860 3525
Connection ~ 10545 1725
Wire Wire Line
	10545 1515 10545 1725
Text Notes 7080 2945 0    50   ~ 10
50V
Wire Wire Line
	7580 3540 7265 3540
Wire Wire Line
	6775 3540 6775 3240
Wire Wire Line
	6775 3240 6850 3240
Connection ~ 7265 3540
Wire Wire Line
	7265 3540 6775 3540
Wire Wire Line
	6850 3240 6850 3440
Wire Wire Line
	7580 3440 7170 3440
$Comp
L Device:R R6
U 1 1 60B14D34
P 7000 3290
F 0 "R6" H 6890 3435 50  0000 L CNN
F 1 "1M" V 6995 3230 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 6930 3290 50  0001 C CNN
F 3 "~" H 7000 3290 50  0001 C CNN
	1    7000 3290
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 60B14D33
P 7170 3290
F 0 "C5" H 7070 3375 50  0000 L CNN
F 1 "0.1uF" H 7200 3210 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 7208 3140 50  0001 C CNN
F 3 "~" H 7170 3290 50  0001 C CNN
	1    7170 3290
	1    0    0    -1  
$EndComp
Connection ~ 7170 3440
Wire Wire Line
	7170 3440 7000 3440
Wire Wire Line
	7350 2840 7480 2840
Wire Wire Line
	7350 3140 7480 3140
Wire Wire Line
	7290 3140 7290 3240
Wire Wire Line
	7290 3240 7580 3240
Wire Wire Line
	7000 3140 7170 3140
Wire Wire Line
	7170 3140 7290 3140
Connection ~ 7170 3140
Connection ~ 6630 3310
Wire Wire Line
	6630 3310 6630 3530
Connection ~ 7000 3440
Wire Wire Line
	6850 3440 7000 3440
Connection ~ 6850 3440
Wire Wire Line
	6850 3440 6850 3605
Wire Wire Line
	6630 3190 6630 3240
Wire Wire Line
	6630 3240 6630 3310
Connection ~ 6630 3240
Wire Wire Line
	6630 3240 6705 3240
Connection ~ 6775 3240
Connection ~ 6705 3240
Wire Wire Line
	6705 3240 6775 3240
Text Notes 7265 3325 0    50   ~ 10
16V
Text Notes 8850 3180 0    50   ~ 10
6.3V
Text Notes 6460 2985 0    50   ~ 10
50V
Text Notes 7270 3885 0    50   ~ 10
16V
Text Notes 6855 3950 0    50   ~ 10
16V
$Comp
L Connector:TestPoint TP11
U 1 1 60B14D59
P 9515 2640
F 0 "TP11" H 9490 2710 50  0000 R CNN
F 1 "TestPoint" H 9590 2840 50  0000 R CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 9715 2640 50  0001 C CNN
F 3 "~" H 9715 2640 50  0001 C CNN
	1    9515 2640
	-1   0    0    1   
$EndComp
Connection ~ 9515 2640
Wire Wire Line
	9515 2640 9680 2640
$Comp
L Connector:TestPoint TP12
U 1 1 60B0CA6B
P 9530 3045
F 0 "TP12" H 9505 3115 50  0000 R CNN
F 1 "TestPoint" H 9605 3245 50  0000 R CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 9730 3045 50  0001 C CNN
F 3 "~" H 9730 3045 50  0001 C CNN
	1    9530 3045
	-1   0    0    1   
$EndComp
Wire Wire Line
	9130 2940 9190 2940
Wire Wire Line
	9190 2940 9190 3045
Wire Wire Line
	9190 3045 9530 3045
$Comp
L Connector:TestPoint TP9
U 1 1 60B14D5B
P 9225 2440
F 0 "TP9" H 9200 2510 50  0000 R CNN
F 1 "TestPoint" H 9300 2640 50  0000 R CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 9425 2440 50  0001 C CNN
F 3 "~" H 9425 2440 50  0001 C CNN
	1    9225 2440
	1    0    0    -1  
$EndComp
Connection ~ 9225 2440
Wire Wire Line
	9225 2440 9330 2440
$Comp
L Connector:TestPoint TP10
U 1 1 60B10888
P 9430 1940
F 0 "TP10" H 9405 2010 50  0000 R CNN
F 1 "TestPoint" H 9505 2140 50  0000 R CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 9630 1940 50  0001 C CNN
F 3 "~" H 9630 1940 50  0001 C CNN
	1    9430 1940
	1    0    0    -1  
$EndComp
Connection ~ 9430 1940
$Comp
L Connector:TestPoint TP8
U 1 1 60B14D5D
P 6630 3530
F 0 "TP8" V 6550 3705 50  0000 R CNN
F 1 "TestPoint" H 6660 3755 50  0000 R CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 6830 3530 50  0001 C CNN
F 3 "~" H 6830 3530 50  0001 C CNN
	1    6630 3530
	0    -1   -1   0   
$EndComp
Connection ~ 6630 3530
Wire Wire Line
	6630 3530 6630 3590
$Comp
L power:+3.3V #PWR026
U 1 1 60B18524
P 6970 2040
F 0 "#PWR026" H 6970 1890 50  0001 C CNN
F 1 "+3.3V" V 6985 2168 50  0000 L CNN
F 2 "" H 6970 2040 50  0001 C CNN
F 3 "" H 6970 2040 50  0001 C CNN
	1    6970 2040
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7270 2040 7580 2040
$Comp
L power:+3.3V #PWR027
U 1 1 60B14D5F
P 6970 2140
F 0 "#PWR027" H 6970 1990 50  0001 C CNN
F 1 "+3.3V" V 6975 2270 50  0000 L CNN
F 2 "" H 6970 2140 50  0001 C CNN
F 3 "" H 6970 2140 50  0001 C CNN
	1    6970 2140
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7580 2140 7270 2140
$Comp
L Device:R R5
U 1 1 5FB316A0
P 7120 2040
F 0 "R5" V 7120 1990 50  0000 L CNN
F 1 "1.5k" V 7085 2160 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 7050 2040 50  0001 C CNN
F 3 "~" H 7120 2040 50  0001 C CNN
	1    7120 2040
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR028
U 1 1 60B14D60
P 6980 2340
F 0 "#PWR028" H 6980 2190 50  0001 C CNN
F 1 "+3.3V" V 6985 2470 50  0000 L CNN
F 2 "" H 6980 2340 50  0001 C CNN
F 3 "" H 6980 2340 50  0001 C CNN
	1    6980 2340
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7580 1740 7265 1740
Wire Wire Line
	7580 1840 7265 1840
$Comp
L LucidSens_2.01v-rescue:DRV8825PWP-DRV8825PWPR-MinimalSens_0.04v-rescue-LucidSens_1.05v-rescue U2
U 1 1 60B14D31
P 7480 1640
F 0 "U2" H 8180 1905 50  0000 C CNN
F 1 "DRV8825PWP" H 8180 1814 50  0000 C CNN
F 2 "DRV8825PWPR:Texas_Instruments-DRV8825PWP-Level_A" H 7480 2040 50  0001 L CNN
F 3 "http://www.ti.com/general/docs/lit/getliterature.tsp?genericPartNumber=DRV8825&fileType=pdf" H 7480 2140 50  0001 L CNN
F 4 "Radial" H 7480 2240 50  0001 L CNN "Case Package"
F 5 "Manufacturer URL" H 7480 2340 50  0001 L CNN "Component Link 1 Description"
F 6 "http://www.ti.com/" H 7480 2440 50  0001 L CNN "Component Link 1 URL"
F 7 "SLVSA73F, 07/2014" H 7480 2540 50  0001 L CNN "Datasheet Version"
F 8 "8 mm" H 7480 2640 50  0001 L CNN "Diameter"
F 9 "11.5 mm" H 7480 2740 50  0001 L CNN "Height"
F 10 "3.5 mm" H 7480 2840 50  0001 L CNN "Lead Pitch"
F 11 "11.5 mm" H 7480 2940 50  0001 L CNN "Length"
F 12 "85 degC" H 7480 3040 50  0001 L CNN "Max Operating Temperature"
F 13 "-40 degC" H 7480 3140 50  0001 L CNN "Min Operating Temperature"
F 14 "Through Hole" H 7480 3240 50  0001 L CNN "Mount"
F 15 "28-Pin Small Outline Package Integrated Circuit, Body 9.7 x 4.4 mm, Pitch 0.65 mm" H 7480 3340 50  0001 L CNN "Package Description"
F 16 "200" H 7480 3440 50  0001 L CNN "Package Quantity"
F 17 "Bulk" H 7480 3540 50  0001 L CNN "Packaging"
F 18 "2" H 7480 3640 50  0001 L CNN "Pins"
F 19 "No SVHC" H 7480 3740 50  0001 L CNN "REACH SVHC"
F 20 "No" H 7480 3840 50  0001 L CNN "Radiation Hardening"
F 21 "true" H 7480 3940 50  0001 L CNN "Ro HSCompliant"
F 22 "20%" H 7480 4040 50  0001 L CNN "Tolerance"
F 23 "100 V" H 7480 4140 50  0001 L CNN "Voltage Rating"
F 24 "100 V" H 7480 4240 50  0001 L CNN "Voltage Rating DC"
F 25 "8 mm" H 7480 4340 50  0001 L CNN "Width"
F 26 "47 uF" H 7480 4440 50  0001 L CNN "capacitance"
F 27 "IC" H 7480 4540 50  0001 L CNN "category"
F 28 "972973" H 7480 4640 50  0001 L CNN "ciiva ids"
F 29 "8657dfec07a272e1" H 7480 4740 50  0001 L CNN "library id"
F 30 "Texas Instruments" H 7480 4840 50  0001 L CNN "manufacturer"
F 31 "PWP0028C" H 7480 4940 50  0001 L CNN "package"
F 32 "1475067547" H 7480 5040 50  0001 L CNN "release date"
F 33 "11C84FA6-0B99-4DFE-81B3-04F3FFCCD07D" H 7480 5140 50  0001 L CNN "vault revision"
F 34 "yes" H 7480 5240 50  0001 L CNN "imported"
	1    7480 1640
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x03_Male J18
U 1 1 60B34C92
P 5760 1940
F 0 "J18" H 5880 2160 50  0000 C CNN
F 1 "Conn_01x03_Male" V 5700 2045 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 5760 1940 50  0001 C CNN
F 3 "~" H 5760 1940 50  0001 C CNN
	1    5760 1940
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR024
U 1 1 60B14D62
P 6260 1840
F 0 "#PWR024" H 6260 1690 50  0001 C CNN
F 1 "+3.3V" V 6255 1945 50  0000 L CNN
F 2 "" H 6260 1840 50  0001 C CNN
F 3 "" H 6260 1840 50  0001 C CNN
	1    6260 1840
	0    1    1    0   
$EndComp
$Comp
L Device:R R56
U 1 1 60B14D63
P 6110 1840
F 0 "R56" V 6180 1905 50  0000 C CNN
F 1 "1.2k" V 6105 1845 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 6040 1840 50  0001 C CNN
F 3 "~" H 6110 1840 50  0001 C CNN
	1    6110 1840
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR022
U 1 1 60B4EB31
P 5960 2040
F 0 "#PWR022" H 5960 1790 50  0001 C CNN
F 1 "GND" V 5965 1912 50  0000 R CNN
F 2 "" H 5960 2040 50  0001 C CNN
F 3 "" H 5960 2040 50  0001 C CNN
	1    5960 2040
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7580 1940 5960 1940
Text Notes 6100 1785 0    31   ~ 6
(5%)
$Comp
L Connector:Conn_01x03_Male J19
U 1 1 60B692E5
P 5765 2640
F 0 "J19" H 5885 2860 50  0000 C CNN
F 1 "Conn_01x03_Male" V 5715 2650 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 5765 2640 50  0001 C CNN
F 3 "~" H 5765 2640 50  0001 C CNN
	1    5765 2640
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR025
U 1 1 60B692EB
P 6265 2540
F 0 "#PWR025" H 6265 2390 50  0001 C CNN
F 1 "+3.3V" V 6260 2645 50  0000 L CNN
F 2 "" H 6265 2540 50  0001 C CNN
F 3 "" H 6265 2540 50  0001 C CNN
	1    6265 2540
	0    1    1    0   
$EndComp
$Comp
L Device:R R57
U 1 1 60B692F1
P 6115 2540
F 0 "R57" V 6185 2605 50  0000 C CNN
F 1 "1.2k" V 6110 2545 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 6045 2540 50  0001 C CNN
F 3 "~" H 6115 2540 50  0001 C CNN
	1    6115 2540
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR023
U 1 1 60B692F7
P 5965 2740
F 0 "#PWR023" H 5965 2490 50  0001 C CNN
F 1 "GND" V 5970 2612 50  0000 R CNN
F 2 "" H 5965 2740 50  0001 C CNN
F 3 "" H 5965 2740 50  0001 C CNN
	1    5965 2740
	0    -1   -1   0   
$EndComp
Text Notes 6105 2485 0    31   ~ 6
(5%)
Wire Wire Line
	7580 2640 5965 2640
$Comp
L Connector:Conn_01x03_Male J15
U 1 1 60B73935
P 5205 2300
F 0 "J15" H 5325 2520 50  0000 C CNN
F 1 "Conn_01x03_Male" V 5140 2360 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 5205 2300 50  0001 C CNN
F 3 "~" H 5205 2300 50  0001 C CNN
	1    5205 2300
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR020
U 1 1 60B14D6A
P 5705 2200
F 0 "#PWR020" H 5705 2050 50  0001 C CNN
F 1 "+3.3V" V 5700 2305 50  0000 L CNN
F 2 "" H 5705 2200 50  0001 C CNN
F 3 "" H 5705 2200 50  0001 C CNN
	1    5705 2200
	0    1    1    0   
$EndComp
$Comp
L Device:R R54
U 1 1 60B14D6B
P 5555 2200
F 0 "R54" V 5625 2265 50  0000 C CNN
F 1 "1.2k" V 5550 2205 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 5485 2200 50  0001 C CNN
F 3 "~" H 5555 2200 50  0001 C CNN
	1    5555 2200
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR018
U 1 1 60B14D6C
P 5405 2400
F 0 "#PWR018" H 5405 2150 50  0001 C CNN
F 1 "GND" V 5410 2272 50  0000 R CNN
F 2 "" H 5405 2400 50  0001 C CNN
F 3 "" H 5405 2400 50  0001 C CNN
	1    5405 2400
	0    -1   -1   0   
$EndComp
Text Notes 5545 2145 0    31   ~ 6
(5%)
$Comp
L Connector:Conn_01x03_Male J17
U 1 1 60B14D6D
P 5215 3070
F 0 "J17" H 5335 3290 50  0000 C CNN
F 1 "Conn_01x03_Male" V 5155 3085 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 5215 3070 50  0001 C CNN
F 3 "~" H 5215 3070 50  0001 C CNN
	1    5215 3070
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR021
U 1 1 60B7D7B8
P 5715 2970
F 0 "#PWR021" H 5715 2820 50  0001 C CNN
F 1 "+3.3V" V 5710 3075 50  0000 L CNN
F 2 "" H 5715 2970 50  0001 C CNN
F 3 "" H 5715 2970 50  0001 C CNN
	1    5715 2970
	0    1    1    0   
$EndComp
$Comp
L Device:R R55
U 1 1 60B7D7BE
P 5565 2970
F 0 "R55" V 5635 3035 50  0000 C CNN
F 1 "1.2k" V 5560 2975 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 5495 2970 50  0001 C CNN
F 3 "~" H 5565 2970 50  0001 C CNN
	1    5565 2970
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR019
U 1 1 60B7D7C4
P 5415 3170
F 0 "#PWR019" H 5415 2920 50  0001 C CNN
F 1 "GND" V 5420 3042 50  0000 R CNN
F 2 "" H 5415 3170 50  0001 C CNN
F 3 "" H 5415 3170 50  0001 C CNN
	1    5415 3170
	0    -1   -1   0   
$EndComp
Text Notes 5555 2915 0    31   ~ 6
(5%)
Wire Wire Line
	5405 2300 6620 2300
Wire Wire Line
	6620 2300 6620 2540
Wire Wire Line
	6620 2540 7580 2540
Wire Wire Line
	5415 3070 6415 3070
Wire Wire Line
	6415 3070 6415 2740
Wire Wire Line
	6415 2740 7580 2740
$Comp
L LucidSens_2.01v-rescue:1812L150_24MR-1812L150_24MR-MinimalSens_0.04v-rescue-LucidSens_1.05v-rescue F1
U 1 1 5FB018DD
P 1970 4685
F 0 "F1" H 2320 4910 50  0000 C CNN
F 1 "1812L200_16V" H 2320 4819 50  0000 C CNN
F 2 "1812L150_24MR:FUSC4632X180N" H 2520 4735 50  0001 L CNN
F 3 "https://es.rs-online.com/web/p/products/174-0825/?cm_mmc=es-DS-APP-_-PCB-_-VIEW-_-174-0825&utm_source=es-DS-APP&utm_medium=PCB&utm_campaign=174-0825" H 2520 4635 50  0001 L CNN
F 4 "PTC Resettable Fuse 1.5A(hold) 3A(trip) 24VDC 20A 0.8W 1.5s 0.04Ohm SMD Solder Pad 1812 T/R" H 2520 4535 50  0001 L CNN "Description"
F 5 "1.8" H 2520 4435 50  0001 L CNN "Height"
F 6 "LITTELFUSE" H 2520 4335 50  0001 L CNN "Manufacturer_Name"
F 7 "1812L150/24MR" H 2520 4235 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "1812L150/24MR" H 2520 4135 50  0001 L CNN "Arrow Part Number"
F 9 "https://www.arrow.com/en/products/1812l15024mr/littelfuse" H 2520 4035 50  0001 L CNN "Arrow Price/Stock"
F 10 "576-1812L150/24MR" H 2520 3935 50  0001 L CNN "Mouser Part Number"
F 11 "https://www.mouser.co.uk/ProductDetail/Littelfuse/1812L150-24MR?qs=PWhpLWeW8wfTVRohZjjbDw%3D%3D" H 2520 3835 50  0001 L CNN "Mouser Price/Stock"
	1    1970 4685
	1    0    0    -1  
$EndComp
Wire Wire Line
	2670 4685 2720 4685
Text Notes 1430 6490 0    31   ~ 6
(5%)
Text Notes 2540 5825 0    31   ~ 6
10V(20%)
Text Notes 2140 5835 0    31   ~ 6
10V(20%)
Text Notes 2565 7135 0    31   ~ 6
10V(20%)
Text Notes 2130 7130 0    31   ~ 6
10V(20%)
Text Notes 1705 4620 0    31   ~ 6
4A-Rated
Text Notes 2185 4850 0    31   ~ 6
I_Hold: 2.0A\nI_Trip: 3.5A
$Comp
L Device:CP1 C20
U 1 1 60B9B11E
P 2035 6985
AR Path="/60B9B11E" Ref="C20"  Part="1" 
AR Path="/5FBA3F04/60B9B11E" Ref="C20"  Part="1" 
F 0 "C20" H 2150 7031 50  0000 L CNN
F 1 "22uF" H 2150 6940 50  0000 L CNN
F 2 "EEEFP1C220AR:CAPAE530X610N" H 2035 6985 50  0001 C CNN
F 3 "~" H 2035 6985 50  0001 C CNN
	1    2035 6985
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 6835 2535 6835
Connection ~ 2450 6835
$Comp
L Device:C C18
U 1 1 5FA5513C
P 2450 6985
F 0 "C18" H 2565 7031 50  0000 L CNN
F 1 "0.22uF" H 2565 6940 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 2488 6835 50  0001 C CNN
F 3 "~" H 2450 6985 50  0001 C CNN
	1    2450 6985
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 5FA55142
P 2450 7135
F 0 "#PWR02" H 2450 6885 50  0001 C CNN
F 1 "GND" H 2455 6962 50  0000 C CNN
F 2 "" H 2450 7135 50  0001 C CNN
F 3 "" H 2450 7135 50  0001 C CNN
	1    2450 7135
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5FA51158
P 2430 5835
F 0 "#PWR01" H 2430 5585 50  0001 C CNN
F 1 "GND" H 2435 5662 50  0000 C CNN
F 2 "" H 2430 5835 50  0001 C CNN
F 3 "" H 2430 5835 50  0001 C CNN
	1    2430 5835
	1    0    0    -1  
$EndComp
Wire Wire Line
	2430 5535 2535 5535
Connection ~ 2430 5535
$Comp
L Device:C C17
U 1 1 5FA50403
P 2430 5685
F 0 "C17" H 2545 5731 50  0000 L CNN
F 1 "0.22uF" H 2545 5640 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 2468 5535 50  0001 C CNN
F 3 "~" H 2430 5685 50  0001 C CNN
	1    2430 5685
	1    0    0    -1  
$EndComp
Text Notes 2405 5040 0    31   ~ 6
25V(20%)
Wire Wire Line
	1120 6835 1285 6835
Wire Wire Line
	1145 5535 1285 5535
Wire Wire Line
	2910 4685 3020 4685
Wire Wire Line
	1070 4685 1220 4685
$Comp
L power:PWR_FLAG #FLG0107
U 1 1 5FBD5943
P 2910 4685
AR Path="/5FBD5943" Ref="#FLG0107"  Part="1" 
AR Path="/5FBA3F04/5FBD5943" Ref="#FLG0107"  Part="1" 
F 0 "#FLG0107" H 2910 4760 50  0001 C CNN
F 1 "PWR_FLAG" H 2925 4850 50  0000 C CNN
F 2 "" H 2910 4685 50  0001 C CNN
F 3 "~" H 2910 4685 50  0001 C CNN
	1    2910 4685
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0102
U 1 1 5FBD593D
P 2535 6835
AR Path="/5FBD593D" Ref="#PWR0102"  Part="1" 
AR Path="/5FBA3F04/5FBD593D" Ref="#PWR0149"  Part="1" 
F 0 "#PWR0102" H 2535 6685 50  0001 C CNN
F 1 "+3.3V" V 2530 7065 50  0000 C CNN
F 2 "" H 2535 6835 50  0001 C CNN
F 3 "" H 2535 6835 50  0001 C CNN
	1    2535 6835
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0105
U 1 1 5FBD5937
P 1120 6835
AR Path="/5FBD5937" Ref="#PWR0105"  Part="1" 
AR Path="/5FBA3F04/5FBD5937" Ref="#PWR0148"  Part="1" 
F 0 "#PWR0105" H 1120 6685 50  0001 C CNN
F 1 "+5V" V 1115 7045 50  0000 C CNN
F 2 "" H 1120 6835 50  0001 C CNN
F 3 "" H 1120 6835 50  0001 C CNN
	1    1120 6835
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR0106
U 1 1 5FBD5931
P 2535 5535
AR Path="/5FBD5931" Ref="#PWR0106"  Part="1" 
AR Path="/5FBA3F04/5FBD5931" Ref="#PWR0147"  Part="1" 
F 0 "#PWR0106" H 2535 5385 50  0001 C CNN
F 1 "+5V" V 2530 5650 50  0000 L CNN
F 2 "" H 2535 5535 50  0001 C CNN
F 3 "" H 2535 5535 50  0001 C CNN
	1    2535 5535
	0    1    1    0   
$EndComp
$Comp
L power:+12V #PWR0111
U 1 1 5FBD592B
P 1145 5535
AR Path="/5FBD592B" Ref="#PWR0111"  Part="1" 
AR Path="/5FBA3F04/5FBD592B" Ref="#PWR0146"  Part="1" 
F 0 "#PWR0111" H 1145 5385 50  0001 C CNN
F 1 "+12V" V 1140 5655 50  0000 L CNN
F 2 "" H 1145 5535 50  0001 C CNN
F 3 "" H 1145 5535 50  0001 C CNN
	1    1145 5535
	0    -1   -1   0   
$EndComp
$Comp
L power:+12V #PWR0112
U 1 1 5FBD5925
P 3020 4685
AR Path="/5FBD5925" Ref="#PWR0112"  Part="1" 
AR Path="/5FBA3F04/5FBD5925" Ref="#PWR0145"  Part="1" 
F 0 "#PWR0112" H 3020 4535 50  0001 C CNN
F 1 "+12V" V 3015 4785 50  0000 L CNN
F 2 "" H 3020 4685 50  0001 C CNN
F 3 "" H 3020 4685 50  0001 C CNN
	1    3020 4685
	0    1    1    0   
$EndComp
Text Notes 2735 4785 0    50   ~ 10
12VOUT
Text Notes 2620 5445 0    50   ~ 10
5VOUT
Text Notes 2635 6765 0    50   ~ 10
3V3OUT
Text Notes 1100 6530 0    50   ~ 10
RED
Text Notes 2605 3975 2    100  ~ 20
Power management
$Comp
L Connector:Barrel_Jack J4
U 1 1 5FBD5913
P 770 4785
AR Path="/5FBD5913" Ref="J4"  Part="1" 
AR Path="/5FBA3F04/5FBD5913" Ref="J4"  Part="1" 
F 0 "J4" H 770 5135 50  0000 C CNN
F 1 "Barrel_Jack" H 770 5035 50  0000 C CNN
F 2 "Connector_BarrelJack:BarrelJack_CUI_PJ-063AH_Horizontal" H 820 4745 50  0001 C CNN
F 3 "~" H 820 4745 50  0001 C CNN
	1    770  4785
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0117
U 1 1 5FBD590D
P 1570 4985
AR Path="/5FBD590D" Ref="#PWR0117"  Part="1" 
AR Path="/5FBA3F04/5FBD590D" Ref="#PWR0144"  Part="1" 
F 0 "#PWR0117" H 1570 4735 50  0001 C CNN
F 1 "GND" H 1575 4812 50  0000 C CNN
F 2 "" H 1570 4985 50  0001 C CNN
F 3 "" H 1570 4985 50  0001 C CNN
	1    1570 4985
	1    0    0    -1  
$EndComp
$Comp
L Diode:B350 D5
U 1 1 5FBD5907
P 1820 4685
AR Path="/5FBD5907" Ref="D5"  Part="1" 
AR Path="/5FBA3F04/5FBD5907" Ref="D5"  Part="1" 
F 0 "D5" H 1820 4902 50  0000 C CNN
F 1 "SK44L-TP" H 1820 4811 50  0000 C CNN
F 2 "Diode_SMD:D_SMC" H 1820 4510 50  0001 C CNN
F 3 "http://www.jameco.com/Jameco/Products/ProdDS/1538777.pdf" H 1820 4685 50  0001 C CNN
	1    1820 4685
	-1   0    0    1   
$EndComp
$Comp
L Device:CP1 C21
U 1 1 60B9B121
P 2720 4835
AR Path="/60B9B121" Ref="C21"  Part="1" 
AR Path="/5FBA3F04/60B9B121" Ref="C21"  Part="1" 
F 0 "C21" H 2450 4805 50  0000 L CNN
F 1 "270uF" H 2405 4735 50  0000 L CNN
F 2 "EEE-FK1V271SL:CAPAE830X1050N" H 2720 4835 50  0001 C CNN
F 3 "~" H 2720 4835 50  0001 C CNN
	1    2720 4835
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0118
U 1 1 60B9B120
P 2720 4985
AR Path="/60B9B120" Ref="#PWR0118"  Part="1" 
AR Path="/5FBA3F04/60B9B120" Ref="#PWR0143"  Part="1" 
F 0 "#PWR0118" H 2720 4735 50  0001 C CNN
F 1 "GND" H 2720 4820 50  0000 C CNN
F 2 "" H 2720 4985 50  0001 C CNN
F 3 "" H 2720 4985 50  0001 C CNN
	1    2720 4985
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male J5
U 1 1 60B9B11F
P 1320 4185
AR Path="/60B9B11F" Ref="J5"  Part="1" 
AR Path="/5FBA3F04/60B9B11F" Ref="J5"  Part="1" 
F 0 "J5" V 1382 4329 50  0000 L CNN
F 1 "Conn_01x04_Male" V 1220 3830 50  0000 L CNN
F 2 "Connector_Phoenix_MSTB:PhoenixContact_MSTBVA_2,5_4-G-5,08_1x04_P5.08mm_Vertical" H 1320 4185 50  0001 C CNN
F 3 "~" H 1320 4185 50  0001 C CNN
	1    1320 4185
	0    -1   1    0   
$EndComp
Wire Wire Line
	1070 4885 1320 4885
Wire Wire Line
	1220 4685 1220 4385
Wire Wire Line
	1320 4385 1320 4885
Connection ~ 1320 4885
Wire Wire Line
	1320 4885 1570 4885
Wire Wire Line
	1420 4385 1420 4685
Wire Wire Line
	1420 4685 1520 4685
Wire Wire Line
	1520 4385 1520 4685
Connection ~ 1520 4685
Wire Wire Line
	1520 4685 1670 4685
$Comp
L Regulator_Linear:AMS1117-5.0 U4
U 1 1 5FBD58C0
P 1585 5535
AR Path="/5FBD58C0" Ref="U4"  Part="1" 
AR Path="/5FBA3F04/5FBD58C0" Ref="U4"  Part="1" 
F 0 "U4" H 1585 5777 50  0000 C CNN
F 1 "AMS1117-5.0" H 1585 5686 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 1585 5735 50  0001 C CNN
F 3 "http://www.advanced-monolithic.com/pdf/ds1117.pdf" H 1685 5285 50  0001 C CNN
	1    1585 5535
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:AMS1117-3.3 U5
U 1 1 5FBD58BA
P 1585 6835
AR Path="/5FBD58BA" Ref="U5"  Part="1" 
AR Path="/5FBA3F04/5FBD58BA" Ref="U5"  Part="1" 
F 0 "U5" H 1585 7077 50  0000 C CNN
F 1 "AMS1117-3.3" H 1585 6986 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 1585 7035 50  0001 C CNN
F 3 "http://www.advanced-monolithic.com/pdf/ds1117.pdf" H 1685 6585 50  0001 C CNN
	1    1585 6835
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0120
U 1 1 5FBD58B2
P 1585 5835
AR Path="/5FBD58B2" Ref="#PWR0120"  Part="1" 
AR Path="/5FBA3F04/5FBD58B2" Ref="#PWR0141"  Part="1" 
F 0 "#PWR0120" H 1585 5585 50  0001 C CNN
F 1 "GND" H 1590 5662 50  0000 C CNN
F 2 "" H 1585 5835 50  0001 C CNN
F 3 "" H 1585 5835 50  0001 C CNN
	1    1585 5835
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C19
U 1 1 5FBD58AC
P 2035 5685
AR Path="/5FBD58AC" Ref="C19"  Part="1" 
AR Path="/5FBA3F04/5FBD58AC" Ref="C19"  Part="1" 
F 0 "C19" H 2150 5731 50  0000 L CNN
F 1 "22uF" H 2150 5640 50  0000 L CNN
F 2 "EEEFP1C220AR:CAPAE530X610N" H 2035 5685 50  0001 C CNN
F 3 "~" H 2035 5685 50  0001 C CNN
	1    2035 5685
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0133
U 1 1 5FBD58A6
P 2035 5835
AR Path="/5FBD58A6" Ref="#PWR0133"  Part="1" 
AR Path="/5FBA3F04/5FBD58A6" Ref="#PWR0140"  Part="1" 
F 0 "#PWR0133" H 2035 5585 50  0001 C CNN
F 1 "GND" H 2040 5662 50  0000 C CNN
F 2 "" H 2035 5835 50  0001 C CNN
F 3 "" H 2035 5835 50  0001 C CNN
	1    2035 5835
	1    0    0    -1  
$EndComp
Wire Wire Line
	2035 5535 2430 5535
Wire Wire Line
	1885 5535 2035 5535
Connection ~ 2035 5535
$Comp
L power:GND #PWR0135
U 1 1 5FBD5897
P 2035 7135
AR Path="/5FBD5897" Ref="#PWR0135"  Part="1" 
AR Path="/5FBA3F04/5FBD5897" Ref="#PWR0139"  Part="1" 
F 0 "#PWR0135" H 2035 6885 50  0001 C CNN
F 1 "GND" H 2040 6962 50  0000 C CNN
F 2 "" H 2035 7135 50  0001 C CNN
F 3 "" H 2035 7135 50  0001 C CNN
	1    2035 7135
	1    0    0    -1  
$EndComp
Wire Wire Line
	1885 6835 2035 6835
Connection ~ 2035 6835
$Comp
L power:GND #PWR0136
U 1 1 5FBD588E
P 1585 7135
AR Path="/5FBD588E" Ref="#PWR0136"  Part="1" 
AR Path="/5FBA3F04/5FBD588E" Ref="#PWR0138"  Part="1" 
F 0 "#PWR0136" H 1585 6885 50  0001 C CNN
F 1 "GND" H 1590 6962 50  0000 C CNN
F 2 "" H 1585 7135 50  0001 C CNN
F 3 "" H 1585 7135 50  0001 C CNN
	1    1585 7135
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0137
U 1 1 5FBD5872
P 675 6485
AR Path="/5FBD5872" Ref="#PWR0137"  Part="1" 
AR Path="/5FBA3F04/5FBD5872" Ref="#PWR0136"  Part="1" 
F 0 "#PWR0137" H 675 6235 50  0001 C CNN
F 1 "GND" H 680 6312 50  0000 C CNN
F 2 "" H 675 6485 50  0001 C CNN
F 3 "" H 675 6485 50  0001 C CNN
	1    675  6485
	-1   0    0    -1  
$EndComp
$Comp
L Device:LED D4
U 1 1 5FBD586C
P 1175 6385
AR Path="/5FBD586C" Ref="D4"  Part="1" 
AR Path="/5FBA3F04/5FBD586C" Ref="D4"  Part="1" 
F 0 "D4" H 1168 6130 50  0000 C CNN
F 1 "LED" H 1168 6221 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 1175 6385 50  0001 C CNN
F 3 "~" H 1175 6385 50  0001 C CNN
	1    1175 6385
	1    0    0    1   
$EndComp
$Comp
L Device:R R29
U 1 1 5FBD5866
P 1475 6385
AR Path="/5FBD5866" Ref="R29"  Part="1" 
AR Path="/5FBA3F04/5FBD5866" Ref="R29"  Part="1" 
F 0 "R29" V 1395 6380 50  0000 C CNN
F 1 "100" V 1475 6385 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 1405 6385 50  0001 C CNN
F 3 "~" H 1475 6385 50  0001 C CNN
	1    1475 6385
	0    -1   1    0   
$EndComp
Wire Wire Line
	675  6385 675  6485
Wire Wire Line
	1025 6385 675  6385
$Comp
L Switch:SW_DIP_x01 SW4
U 1 1 5FBD585B
P 1925 6385
AR Path="/5FBD585B" Ref="SW4"  Part="1" 
AR Path="/5FBA3F04/5FBD585B" Ref="SW4"  Part="1" 
F 0 "SW4" H 1925 6652 50  0000 C CNN
F 1 "SW_DIP_x01" H 1925 6561 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_DIP_SPSTx01_Slide_Omron_A6S-110x_W8.9mm_P2.54mm" H 1925 6385 50  0001 C CNN
F 3 "~" H 1925 6385 50  0001 C CNN
	1    1925 6385
	-1   0    0    -1  
$EndComp
Connection ~ 2910 4685
Connection ~ 2720 4685
Wire Wire Line
	2720 4685 2910 4685
Wire Wire Line
	2225 6835 2450 6835
Wire Wire Line
	2035 6835 2225 6835
Connection ~ 2225 6835
Wire Wire Line
	2225 6385 2225 6835
Wire Wire Line
	1570 4885 1570 4985
$Comp
L Connector:TestPoint TP7
U 1 1 60B9B128
P 2720 4685
F 0 "TP7" H 2600 4890 50  0000 L CNN
F 1 "TestPoint" H 2545 4735 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 2920 4685 50  0001 C CNN
F 3 "~" H 2920 4685 50  0001 C CNN
	1    2720 4685
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP5
U 1 1 60B9B129
P 2430 5535
F 0 "TP5" H 2310 5740 50  0000 L CNN
F 1 "TestPoint" H 2040 5655 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 2630 5535 50  0001 C CNN
F 3 "~" H 2630 5535 50  0001 C CNN
	1    2430 5535
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP6
U 1 1 60B9D70B
P 2450 6835
F 0 "TP6" H 2330 7040 50  0000 L CNN
F 1 "TestPoint" H 2260 6895 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 2650 6835 50  0001 C CNN
F 3 "~" H 2650 6835 50  0001 C CNN
	1    2450 6835
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP21
U 1 1 60BBF4A7
P 4695 8885
F 0 "TP21" H 4835 8955 50  0000 R CNN
F 1 "TestPoint" H 4850 9095 50  0000 R CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 4895 8885 50  0001 C CNN
F 3 "~" H 4895 8885 50  0001 C CNN
	1    4695 8885
	-1   0    0    1   
$EndComp
$Comp
L Connector:TestPoint TP20
U 1 1 60BBF4A6
P 4695 8285
F 0 "TP20" H 4645 8485 50  0000 L CNN
F 1 "TestPoint" H 4735 8415 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 4895 8285 50  0001 C CNN
F 3 "~" H 4895 8285 50  0001 C CNN
	1    4695 8285
	1    0    0    -1  
$EndComp
Wire Wire Line
	2370 8985 2465 8985
Wire Wire Line
	2370 8845 2370 8985
Wire Wire Line
	1270 8845 2370 8845
Connection ~ 2370 8985
Wire Wire Line
	2270 8985 2370 8985
Wire Wire Line
	1985 9785 1770 9785
Wire Wire Line
	1985 9725 1985 9785
Wire Wire Line
	1260 9725 1985 9725
Wire Wire Line
	1260 9685 1260 9725
Wire Wire Line
	2660 9985 2765 9985
Wire Wire Line
	2660 9865 2660 9985
Wire Wire Line
	1070 9865 2660 9865
$Comp
L Connector:TestPoint TP18
U 1 1 60B908EC
P 1070 9865
F 0 "TP18" H 1030 10065 50  0000 L CNN
F 1 "TestPoint" H 1080 9930 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 1270 9865 50  0001 C CNN
F 3 "~" H 1270 9865 50  0001 C CNN
	1    1070 9865
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP19
U 1 1 60BBF4A4
P 1260 9685
F 0 "TP19" H 1200 9885 50  0000 L CNN
F 1 "TestPoint" H 1275 9755 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 1460 9685 50  0001 C CNN
F 3 "~" H 1460 9685 50  0001 C CNN
	1    1260 9685
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0103
U 1 1 5FB83904
P 4365 9485
AR Path="/5FB83904" Ref="#FLG0103"  Part="1" 
AR Path="/5FB4F567/5FB83904" Ref="#FLG0106"  Part="1" 
F 0 "#FLG0103" H 4365 9560 50  0001 C CNN
F 1 "PWR_FLAG" H 4360 9640 50  0000 C CNN
F 2 "" H 4365 9485 50  0001 C CNN
F 3 "~" H 4365 9485 50  0001 C CNN
	1    4365 9485
	1    0    0    -1  
$EndComp
Wire Wire Line
	4365 9485 4450 9485
$Comp
L power:PWR_FLAG #FLG0104
U 1 1 5FB838FC
P 4365 9785
AR Path="/5FB838FC" Ref="#FLG0104"  Part="1" 
AR Path="/5FB4F567/5FB838FC" Ref="#FLG0105"  Part="1" 
F 0 "#FLG0104" H 4365 9860 50  0001 C CNN
F 1 "PWR_FLAG" H 4365 9958 50  0000 C CNN
F 2 "" H 4365 9785 50  0001 C CNN
F 3 "~" H 4365 9785 50  0001 C CNN
	1    4365 9785
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG0105
U 1 1 5FB838F6
P 2720 9785
AR Path="/5FB838F6" Ref="#FLG0105"  Part="1" 
AR Path="/5FB4F567/5FB838F6" Ref="#FLG0104"  Part="1" 
F 0 "#FLG0105" H 2720 9860 50  0001 C CNN
F 1 "PWR_FLAG" H 2715 9935 50  0000 C CNN
F 2 "" H 2720 9785 50  0001 C CNN
F 3 "~" H 2720 9785 50  0001 C CNN
	1    2720 9785
	1    0    0    -1  
$EndComp
$Comp
L Device:R R22
U 1 1 5FB838F0
P 2400 9585
AR Path="/5FB838F0" Ref="R22"  Part="1" 
AR Path="/5FB4F567/5FB838F0" Ref="R22"  Part="1" 
F 0 "R22" V 2315 9580 50  0000 C CNN
F 1 "10k" V 2400 9595 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 2330 9585 50  0001 C CNN
F 3 "~" H 2400 9585 50  0001 C CNN
	1    2400 9585
	0    1    1    0   
$EndComp
Connection ~ 1985 9785
Wire Wire Line
	2720 9785 1985 9785
Wire Wire Line
	2680 8585 2765 8585
Wire Wire Line
	2680 8670 2680 8585
$Comp
L power:PWR_FLAG #FLG0106
U 1 1 60BBF49F
P 2680 8670
AR Path="/60BBF49F" Ref="#FLG0106"  Part="1" 
AR Path="/5FB4F567/60BBF49F" Ref="#FLG0103"  Part="1" 
F 0 "#FLG0106" H 2680 8745 50  0001 C CNN
F 1 "PWR_FLAG" H 2675 8800 50  0000 C CNN
F 2 "" H 2680 8670 50  0001 C CNN
F 3 "~" H 2680 8670 50  0001 C CNN
	1    2680 8670
	-1   0    0    1   
$EndComp
Wire Wire Line
	1770 9785 1770 9955
Connection ~ 2720 9785
Wire Wire Line
	2765 9785 2720 9785
Connection ~ 1770 9785
Wire Wire Line
	1770 9635 1770 9785
Wire Wire Line
	4365 9785 4450 9785
$Comp
L Device:C C16
U 1 1 60BBF49E
P 4450 9635
AR Path="/60BBF49E" Ref="C16"  Part="1" 
AR Path="/5FB4F567/60BBF49E" Ref="C16"  Part="1" 
F 0 "C16" H 4565 9681 50  0000 L CNN
F 1 "0.022uF" H 4565 9590 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 4488 9485 50  0001 C CNN
F 3 "~" H 4450 9635 50  0001 C CNN
	1    4450 9635
	1    0    0    -1  
$EndComp
Text Notes 2400 8680 0    50   ~ 10
X5R or X7R
Text Notes 4430 9875 0    50   ~ 10
X5R or X7R
Text Notes 1925 7820 0    100  ~ 20
Peltier Heating/Cooling Module
Text Notes 2190 10550 0    50   ~ 10
PH/EN Mode
Text Notes 2625 8475 0    50   ~ 10
50V
Text Notes 2175 8620 0    50   ~ 10
50V
Text Notes 1815 8625 0    50   ~ 10
50V
Text Notes 4565 9795 0    50   ~ 10
50V
Text Notes 1805 9980 0    50   ~ 10
I_Trip=3.01A
Text Notes 2100 9775 0    50   ~ 10
VREF=2.5V
$Comp
L Device:R R27
U 1 1 60BBF49D
P 2615 9185
AR Path="/60BBF49D" Ref="R27"  Part="1" 
AR Path="/5FB4F567/60BBF49D" Ref="R27"  Part="1" 
F 0 "R27" V 2540 9145 50  0000 C CNN
F 1 "1.2k" V 2615 9185 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 2545 9185 50  0001 C CNN
F 3 "~" H 2615 9185 50  0001 C CNN
	1    2615 9185
	0    1    1    0   
$EndComp
$Comp
L Device:R R26
U 1 1 60BBF49C
P 2615 8985
AR Path="/60BBF49C" Ref="R26"  Part="1" 
AR Path="/5FB4F567/60BBF49C" Ref="R26"  Part="1" 
F 0 "R26" V 2535 8950 50  0000 C CNN
F 1 "1.2k" V 2615 8985 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 2545 8985 50  0001 C CNN
F 3 "~" H 2615 8985 50  0001 C CNN
	1    2615 8985
	0    1    1    0   
$EndComp
Text GLabel 2465 9185 0    50   Input ~ 0
IO22
Text GLabel 2270 8985 0    50   Input ~ 0
IO21
Wire Wire Line
	4365 8285 4695 8285
Connection ~ 4695 8285
Wire Wire Line
	4695 8285 4695 8510
Wire Wire Line
	4695 8885 4695 8610
Connection ~ 4695 8885
Wire Wire Line
	4365 8885 4695 8885
$Comp
L Connector:Conn_01x02_Male J3
U 1 1 60BBF49B
P 4895 8610
AR Path="/60BBF49B" Ref="J3"  Part="1" 
AR Path="/5FB4F567/60BBF49B" Ref="J3"  Part="1" 
F 0 "J3" H 5080 8670 50  0000 R CNN
F 1 "Conn_01x02_Male" V 4830 8905 50  0000 R CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal" H 4895 8610 50  0001 C CNN
F 3 "~" H 4895 8610 50  0001 C CNN
	1    4895 8610
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0122
U 1 1 60BBF49A
P 1770 10255
AR Path="/60BBF49A" Ref="#PWR0122"  Part="1" 
AR Path="/5FB4F567/60BBF49A" Ref="#PWR0132"  Part="1" 
F 0 "#PWR0122" H 1770 10005 50  0001 C CNN
F 1 "GND" H 1775 10082 50  0000 C CNN
F 2 "" H 1770 10255 50  0001 C CNN
F 3 "" H 1770 10255 50  0001 C CNN
	1    1770 10255
	1    0    0    -1  
$EndComp
$Comp
L Device:R R20
U 1 1 60BBF499
P 1770 10105
AR Path="/60BBF499" Ref="R20"  Part="1" 
AR Path="/5FB4F567/60BBF499" Ref="R20"  Part="1" 
F 0 "R20" H 1575 10120 50  0000 L CNN
F 1 "3.3k" V 1765 10025 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 1700 10105 50  0001 C CNN
F 3 "~" H 1770 10105 50  0001 C CNN
	1    1770 10105
	1    0    0    -1  
$EndComp
Wire Wire Line
	1770 9205 1770 9335
$Comp
L Device:R R19
U 1 1 5FB8389E
P 1770 9485
AR Path="/5FB8389E" Ref="R19"  Part="1" 
AR Path="/5FB4F567/5FB8389E" Ref="R19"  Part="1" 
F 0 "R19" H 1815 9525 50  0000 L CNN
F 1 "1k" V 1765 9435 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 1700 9485 50  0001 C CNN
F 3 "~" H 1770 9485 50  0001 C CNN
	1    1770 9485
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0123
U 1 1 5FB83898
P 1770 9205
AR Path="/5FB83898" Ref="#PWR0123"  Part="1" 
AR Path="/5FB4F567/5FB83898" Ref="#PWR0131"  Part="1" 
F 0 "#PWR0123" H 1770 9055 50  0001 C CNN
F 1 "+3.3V" H 1750 9345 50  0000 C CNN
F 2 "" H 1770 9205 50  0001 C CNN
F 3 "" H 1770 9205 50  0001 C CNN
	1    1770 9205
	1    0    0    -1  
$EndComp
Wire Wire Line
	2010 9385 2010 9585
Wire Wire Line
	2550 9385 2765 9385
Wire Wire Line
	2010 9385 2250 9385
$Comp
L Device:R R21
U 1 1 60BBF496
P 2400 9385
AR Path="/60BBF496" Ref="R21"  Part="1" 
AR Path="/5FB4F567/60BBF496" Ref="R21"  Part="1" 
F 0 "R21" V 2315 9375 50  0000 C CNN
F 1 "10k" V 2400 9380 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 2330 9385 50  0001 C CNN
F 3 "~" H 2400 9385 50  0001 C CNN
	1    2400 9385
	0    1    1    0   
$EndComp
Connection ~ 2010 9385
Wire Wire Line
	2010 9385 2010 9325
$Comp
L power:+3.3V #PWR0124
U 1 1 60BBF495
P 2010 9325
AR Path="/60BBF495" Ref="#PWR0124"  Part="1" 
AR Path="/5FB4F567/60BBF495" Ref="#PWR0130"  Part="1" 
F 0 "#PWR0124" H 2010 9175 50  0001 C CNN
F 1 "+3.3V" H 1990 9465 50  0000 C CNN
F 2 "" H 2010 9325 50  0001 C CNN
F 3 "" H 2010 9325 50  0001 C CNN
	1    2010 9325
	1    0    0    -1  
$EndComp
Wire Wire Line
	1995 10385 1995 10390
Wire Wire Line
	2250 10385 1995 10385
$Comp
L power:GND #PWR0125
U 1 1 60BBF494
P 1995 10390
AR Path="/60BBF494" Ref="#PWR0125"  Part="1" 
AR Path="/5FB4F567/60BBF494" Ref="#PWR0129"  Part="1" 
F 0 "#PWR0125" H 1995 10140 50  0001 C CNN
F 1 "GND" H 2005 10245 50  0000 C CNN
F 2 "" H 1995 10390 50  0001 C CNN
F 3 "" H 1995 10390 50  0001 C CNN
	1    1995 10390
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 10385 2765 10385
$Comp
L Device:R R25
U 1 1 60BBF493
P 2400 10385
AR Path="/60BBF493" Ref="R25"  Part="1" 
AR Path="/5FB4F567/60BBF493" Ref="R25"  Part="1" 
F 0 "R25" V 2320 10375 50  0000 C CNN
F 1 "0" V 2400 10385 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 2330 10385 50  0001 C CNN
F 3 "~" H 2400 10385 50  0001 C CNN
	1    2400 10385
	0    1    1    0   
$EndComp
Wire Wire Line
	1995 10185 1995 10190
Wire Wire Line
	2250 10185 1995 10185
$Comp
L power:GND #PWR0126
U 1 1 5FB83870
P 1995 10190
AR Path="/5FB83870" Ref="#PWR0126"  Part="1" 
AR Path="/5FB4F567/5FB83870" Ref="#PWR0128"  Part="1" 
F 0 "#PWR0126" H 1995 9940 50  0001 C CNN
F 1 "GND" H 2005 10045 50  0000 C CNN
F 2 "" H 1995 10190 50  0001 C CNN
F 3 "" H 1995 10190 50  0001 C CNN
	1    1995 10190
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 10185 2765 10185
$Comp
L Device:R R24
U 1 1 60BBF491
P 2400 10185
AR Path="/60BBF491" Ref="R24"  Part="1" 
AR Path="/5FB4F567/60BBF491" Ref="R24"  Part="1" 
F 0 "R24" V 2320 10185 50  0000 C CNN
F 1 "0" V 2400 10185 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 2330 10185 50  0001 C CNN
F 3 "~" H 2400 10185 50  0001 C CNN
	1    2400 10185
	0    1    1    0   
$EndComp
Wire Wire Line
	1995 9985 1995 9990
Wire Wire Line
	2250 9985 1995 9985
$Comp
L power:GND #PWR0127
U 1 1 5FB83861
P 1995 9990
AR Path="/5FB83861" Ref="#PWR0127"  Part="1" 
AR Path="/5FB4F567/5FB83861" Ref="#PWR0127"  Part="1" 
F 0 "#PWR0127" H 1995 9740 50  0001 C CNN
F 1 "GND" H 2000 9850 50  0000 C CNN
F 2 "" H 1995 9990 50  0001 C CNN
F 3 "" H 1995 9990 50  0001 C CNN
	1    1995 9990
	1    0    0    -1  
$EndComp
Connection ~ 2660 9985
Wire Wire Line
	2550 9985 2660 9985
$Comp
L Device:R R23
U 1 1 5FB8385A
P 2400 9985
AR Path="/5FB8385A" Ref="R23"  Part="1" 
AR Path="/5FB4F567/5FB8385A" Ref="R23"  Part="1" 
F 0 "R23" V 2400 9980 50  0000 C CNN
F 1 "1.83k" V 2320 9965 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 2330 9985 50  0001 C CNN
F 3 "~" H 2400 9985 50  0001 C CNN
	1    2400 9985
	0    1    1    0   
$EndComp
Wire Wire Line
	2550 9585 2765 9585
Wire Wire Line
	2010 9585 2250 9585
$Comp
L power:+12V #PWR0128
U 1 1 5FB83852
P 1805 8175
AR Path="/5FB83852" Ref="#PWR0128"  Part="1" 
AR Path="/5FB4F567/5FB83852" Ref="#PWR0126"  Part="1" 
F 0 "#PWR0128" H 1805 8025 50  0001 C CNN
F 1 "+12V" H 1795 8320 50  0000 C CNN
F 2 "" H 1805 8175 50  0001 C CNN
F 3 "" H 1805 8175 50  0001 C CNN
	1    1805 8175
	1    0    0    -1  
$EndComp
Connection ~ 2680 8585
Wire Wire Line
	2515 8585 2680 8585
Wire Wire Line
	2165 8285 2515 8285
Wire Wire Line
	2515 8285 2765 8285
Connection ~ 2515 8285
$Comp
L Device:C C15
U 1 1 5FB83849
P 2515 8435
AR Path="/5FB83849" Ref="C15"  Part="1" 
AR Path="/5FB4F567/5FB83849" Ref="C15"  Part="1" 
F 0 "C15" H 2535 8520 50  0000 L CNN
F 1 "100nF" H 2520 8355 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 2553 8285 50  0001 C CNN
F 3 "~" H 2515 8435 50  0001 C CNN
	1    2515 8435
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0129
U 1 1 5FB83843
P 2165 8585
AR Path="/5FB83843" Ref="#PWR0129"  Part="1" 
AR Path="/5FB4F567/5FB83843" Ref="#PWR0125"  Part="1" 
F 0 "#PWR0129" H 2165 8335 50  0001 C CNN
F 1 "GND" H 2165 8445 50  0000 C CNN
F 2 "" H 2165 8585 50  0001 C CNN
F 3 "" H 2165 8585 50  0001 C CNN
	1    2165 8585
	1    0    0    -1  
$EndComp
$Comp
L Device:C C14
U 1 1 60BBF48B
P 2165 8435
AR Path="/60BBF48B" Ref="C14"  Part="1" 
AR Path="/5FB4F567/60BBF48B" Ref="C14"  Part="1" 
F 0 "C14" H 2180 8520 50  0000 L CNN
F 1 "100nF" H 2170 8355 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 2203 8285 50  0001 C CNN
F 3 "~" H 2165 8435 50  0001 C CNN
	1    2165 8435
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0130
U 1 1 60BBF48A
P 1805 8585
AR Path="/60BBF48A" Ref="#PWR0130"  Part="1" 
AR Path="/5FB4F567/60BBF48A" Ref="#PWR0124"  Part="1" 
F 0 "#PWR0130" H 1805 8335 50  0001 C CNN
F 1 "GND" H 1810 8412 50  0000 C CNN
F 2 "" H 1805 8585 50  0001 C CNN
F 3 "" H 1805 8585 50  0001 C CNN
	1    1805 8585
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C13
U 1 1 60BBF489
P 1805 8435
AR Path="/60BBF489" Ref="C13"  Part="1" 
AR Path="/5FB4F567/60BBF489" Ref="C13"  Part="1" 
F 0 "C13" H 1895 8520 50  0000 C CNN
F 1 "100uF" H 1925 8350 50  0000 C CNN
F 2 "EEE-FTH101XAL:CAPAE660X800N" H 1843 8285 50  0001 C CNN
F 3 "~" H 1805 8435 50  0001 C CNN
	1    1805 8435
	1    0    0    -1  
$EndComp
Connection ~ 2165 8285
Wire Wire Line
	1805 8285 2165 8285
Connection ~ 1805 8285
Wire Wire Line
	1805 8175 1805 8285
Wire Wire Line
	4365 10385 4550 10385
Wire Wire Line
	4550 10385 4550 10485
Wire Wire Line
	4550 10485 4550 10545
Connection ~ 4550 10485
Wire Wire Line
	4365 10485 4550 10485
$Comp
L power:GND #PWR0131
U 1 1 60BBF488
P 4550 10545
AR Path="/60BBF488" Ref="#PWR0131"  Part="1" 
AR Path="/5FB4F567/60BBF488" Ref="#PWR0123"  Part="1" 
F 0 "#PWR0131" H 4550 10295 50  0001 C CNN
F 1 "GND" H 4550 10400 50  0000 C CNN
F 2 "" H 4550 10545 50  0001 C CNN
F 3 "" H 4550 10545 50  0001 C CNN
	1    4550 10545
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 8585 4550 8635
Wire Wire Line
	4365 8585 4550 8585
$Comp
L power:GND #PWR0132
U 1 1 5FB8381A
P 4550 8635
AR Path="/5FB8381A" Ref="#PWR0132"  Part="1" 
AR Path="/5FB4F567/5FB8381A" Ref="#PWR0122"  Part="1" 
F 0 "#PWR0132" H 4550 8385 50  0001 C CNN
F 1 "GND" H 4550 8490 50  0000 C CNN
F 2 "" H 4550 8635 50  0001 C CNN
F 3 "" H 4550 8635 50  0001 C CNN
	1    4550 8635
	1    0    0    -1  
$EndComp
Text Notes 1610 10730 0    31   ~ 6
ITrip(A) x A_IPROPI(uA/A) = VRef(V) / R_IPROPI
Text Notes 2510 10370 0    31   ~ 6
(5%)
Text Notes 2510 10170 0    31   ~ 6
(5%)
Text Notes 2505 9970 0    31   ~ 6
(1%)
Text Notes 1595 10180 0    31   ~ 6
(5%)
Text Notes 1825 9530 0    31   ~ 6
(5%)
Text Notes 2505 9570 0    31   ~ 6
(5%)
Text Notes 2500 9375 0    31   ~ 6
(5%)
Text Notes 2655 8930 0    31   ~ 6
(5%)
Text Notes 2650 9135 0    31   ~ 6
(5%)
Text Notes 955  9275 2    50   ~ 10
Green
$Comp
L Switch:SW_DIP_x01 SW?
U 1 1 60BBF482
P 970 8845
AR Path="/5FAE0F92/60BBF482" Ref="SW?"  Part="1" 
AR Path="/5FB4F567/60BBF482" Ref="SW3"  Part="1" 
AR Path="/60BBF482" Ref="SW3"  Part="1" 
F 0 "SW3" H 875 8700 50  0000 L CNN
F 1 "SW_DIP_x01" H 750 8995 50  0000 L CNN
F 2 "Button_Switch_SMD:SW_DIP_SPSTx01_Slide_Omron_A6S-110x_W8.9mm_P2.54mm" H 970 8845 50  0001 C CNN
F 3 "~" H 970 8845 50  0001 C CNN
	1    970  8845
	-1   0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5FA536E4
P 670 8995
AR Path="/5FAE0F92/5FA536E4" Ref="R?"  Part="1" 
AR Path="/5FB4F567/5FA536E4" Ref="R28"  Part="1" 
AR Path="/5FA536E4" Ref="R28"  Part="1" 
F 0 "R28" V 670 9000 50  0000 C CNN
F 1 "100" V 585 9050 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 600 8995 50  0001 C CNN
F 3 "~" H 670 8995 50  0001 C CNN
	1    670  8995
	1    0    0    1   
$EndComp
$Comp
L Device:LED D?
U 1 1 5FA536EA
P 670 9295
AR Path="/5FAE0F92/5FA536EA" Ref="D?"  Part="1" 
AR Path="/5FB4F567/5FA536EA" Ref="D8"  Part="1" 
AR Path="/5FA536EA" Ref="D8"  Part="1" 
F 0 "D8" V 670 9195 50  0000 C CNN
F 1 "LED" V 645 9460 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 670 9295 50  0001 C CNN
F 3 "~" H 670 9295 50  0001 C CNN
	1    670  9295
	0    1    -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FA536F0
P 670 9445
AR Path="/5FAE0F92/5FA536F0" Ref="#PWR?"  Part="1" 
AR Path="/5FB4F567/5FA536F0" Ref="#PWR05"  Part="1" 
AR Path="/5FA536F0" Ref="#PWR05"  Part="1" 
F 0 "#PWR05" H 670 9195 50  0001 C CNN
F 1 "GND" H 670 9300 50  0000 C CNN
F 2 "" H 670 9445 50  0001 C CNN
F 3 "" H 670 9445 50  0001 C CNN
	1    670  9445
	-1   0    0    -1  
$EndComp
Text Notes 610  8985 1    31   ~ 6
(5%)
$Comp
L LucidSens_2.01v-rescue:DRV8874PWPR-DRV8874PWPR-MinimalSens_0.04v-rescue-LucidSens_1.05v-rescue U3
U 1 1 5FA6C7D7
P 3565 9385
F 0 "U3" H 3565 10873 60  0000 C CNN
F 1 "DRV8874PWPR" H 3565 10767 60  0000 C CNN
F 2 "DRV8874PWPR:DRV8874PWPR" H 3565 9325 60  0001 C CNN
F 3 "" H 3565 9385 60  0000 C CNN
	1    3565 9385
	1    0    0    -1  
$EndComp
Connection ~ 4365 9485
Connection ~ 4365 9785
$Comp
L Switch:SW_SPST SW1
U 1 1 611F9F9B
P 5740 7930
F 0 "SW1" H 5735 7750 50  0000 C CNN
F 1 "SW_SPST" H 5735 7820 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_SPST_TL3342" H 5740 7930 50  0001 C CNN
F 3 "~" H 5740 7930 50  0001 C CNN
	1    5740 7930
	1    0    0    -1  
$EndComp
Wire Wire Line
	6265 9700 6465 9700
Wire Wire Line
	5400 8700 5515 8700
Wire Wire Line
	5515 9100 5515 8700
Connection ~ 5515 8700
Wire Wire Line
	5515 8700 5565 8700
Wire Wire Line
	5465 9400 5465 9000
$Comp
L LucidSens_2.01v-rescue:SS8050-G-SS8050-G-MinimalSens_0.04v-rescue-LucidSens_1.05v-rescue Q1
U 1 1 611F9F9A
P 5865 8700
F 0 "Q1" H 6403 8746 50  0000 L CNN
F 1 "SS8050-G" H 6403 8655 50  0000 L CNN
F 2 "SS8050-G:SOT95P240X115-3N" H 6415 8550 50  0001 L CNN
F 3 "http://www.comchiptech.com/admin/files/product/SS8050-G%20RevA181526.pdf" H 6415 8450 50  0001 L CNN
F 4 "Bipolar Transistors - BJT NPN TRANSISTOR 1.5A 40V" H 6415 8350 50  0001 L CNN "Description"
F 5 "1.15" H 6415 8250 50  0001 L CNN "Height"
F 6 "Comchip Technology" H 6415 8150 50  0001 L CNN "Manufacturer_Name"
F 7 "SS8050-G" H 6415 8050 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "SS8050-G" H 6415 7950 50  0001 L CNN "Arrow Part Number"
F 9 "https://www.arrow.com/en/products/ss8050-g/comchip-technology" H 6415 7850 50  0001 L CNN "Arrow Price/Stock"
F 10 "750-SS8050-G" H 6415 7750 50  0001 L CNN "Mouser Part Number"
F 11 "https://www.mouser.co.uk/ProductDetail/Comchip-Technology/SS8050-G?qs=LLUE9lz1YbcHg%252BWLMAtcrQ%3D%3D" H 6415 7650 50  0001 L CNN "Mouser Price/Stock"
	1    5865 8700
	1    0    0    -1  
$EndComp
$Comp
L LucidSens_2.01v-rescue:SS8050-G-SS8050-G-MinimalSens_0.04v-rescue-LucidSens_1.05v-rescue Q2
U 1 1 611F9F99
P 5865 9400
F 0 "Q2" H 6403 9446 50  0000 L CNN
F 1 "SS8050-G" H 6403 9355 50  0000 L CNN
F 2 "SS8050-G:SOT95P240X115-3N" H 6415 9250 50  0001 L CNN
F 3 "http://www.comchiptech.com/admin/files/product/SS8050-G%20RevA181526.pdf" H 6415 9150 50  0001 L CNN
F 4 "Bipolar Transistors - BJT NPN TRANSISTOR 1.5A 40V" H 6415 9050 50  0001 L CNN "Description"
F 5 "1.15" H 6415 8950 50  0001 L CNN "Height"
F 6 "Comchip Technology" H 6415 8850 50  0001 L CNN "Manufacturer_Name"
F 7 "SS8050-G" H 6415 8750 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "SS8050-G" H 6415 8650 50  0001 L CNN "Arrow Part Number"
F 9 "https://www.arrow.com/en/products/ss8050-g/comchip-technology" H 6415 8550 50  0001 L CNN "Arrow Price/Stock"
F 10 "750-SS8050-G" H 6415 8450 50  0001 L CNN "Mouser Part Number"
F 11 "https://www.mouser.co.uk/ProductDetail/Comchip-Technology/SS8050-G?qs=LLUE9lz1YbcHg%252BWLMAtcrQ%3D%3D" H 6415 8350 50  0001 L CNN "Mouser Price/Stock"
	1    5865 9400
	1    0    0    1   
$EndComp
$Comp
L LucidSens_2.01v-rescue:LESD5D5.0CT1G-LESD5D5.0CT1G-MinimalSens_0.04v-rescue-LucidSens_1.05v-rescue D6
U 1 1 5FA8CC48
P 8255 9315
F 0 "D6" V 8520 9270 50  0000 L CNN
F 1 "LESD5D5.0CT1G" H 8375 9190 50  0000 L CNN
F 2 "LESD5D5.0CT1G:LESD5D50CT1G" H 8855 9465 50  0001 L BNN
F 3 "http://www.lrcls.com/data/pdf/LESD5D5.0CT1G.pdf" H 8855 9365 50  0001 L BNN
F 4 "Transient Voltage Suppressors for ESD Protection" H 8855 9265 50  0001 L BNN "Description"
F 5 "0.7" H 8855 9165 50  0001 L BNN "Height"
F 6 "LRC" H 8855 9065 50  0001 L BNN "Manufacturer_Name"
F 7 "LESD5D5.0CT1G" H 8855 8965 50  0001 L BNN "Manufacturer_Part_Number"
F 8 "" H 8855 8865 50  0001 L BNN "Arrow Part Number"
F 9 "" H 8855 8765 50  0001 L BNN "Arrow Price/Stock"
F 10 "" H 8855 8665 50  0001 L BNN "Mouser Part Number"
F 11 "" H 8855 8565 50  0001 L BNN "Mouser Price/Stock"
	1    8255 9315
	0    1    1    0   
$EndComp
$Comp
L LucidSens_2.01v-rescue:LESD5D5.0CT1G-LESD5D5.0CT1G-MinimalSens_0.04v-rescue-LucidSens_1.05v-rescue D7
U 1 1 5FA8AABB
P 8505 9315
F 0 "D7" V 8770 9270 50  0000 L CNN
F 1 "LESD5D5.0CT1G" H 8620 9190 50  0000 L CNN
F 2 "LESD5D5.0CT1G:LESD5D50CT1G" H 9105 9465 50  0001 L BNN
F 3 "http://www.lrcls.com/data/pdf/LESD5D5.0CT1G.pdf" H 9105 9365 50  0001 L BNN
F 4 "Transient Voltage Suppressors for ESD Protection" H 9105 9265 50  0001 L BNN "Description"
F 5 "0.7" H 9105 9165 50  0001 L BNN "Height"
F 6 "LRC" H 9105 9065 50  0001 L BNN "Manufacturer_Name"
F 7 "LESD5D5.0CT1G" H 9105 8965 50  0001 L BNN "Manufacturer_Part_Number"
F 8 "" H 9105 8865 50  0001 L BNN "Arrow Part Number"
F 9 "" H 9105 8765 50  0001 L BNN "Arrow Price/Stock"
F 10 "" H 9105 8665 50  0001 L BNN "Mouser Part Number"
F 11 "" H 9105 8565 50  0001 L BNN "Mouser Price/Stock"
	1    8505 9315
	0    1    1    0   
$EndComp
$Comp
L LucidSens_2.01v-rescue:LESD5D5.0CT1G-LESD5D5.0CT1G-MinimalSens_0.04v-rescue-LucidSens_1.05v-rescue D3
U 1 1 5FA86DCA
P 8005 9315
F 0 "D3" V 8270 9270 50  0000 L CNN
F 1 "LESD5D5.0CT1G" H 8125 9185 50  0000 L CNN
F 2 "LESD5D5.0CT1G:LESD5D50CT1G" H 8605 9465 50  0001 L BNN
F 3 "http://www.lrcls.com/data/pdf/LESD5D5.0CT1G.pdf" H 8605 9365 50  0001 L BNN
F 4 "Transient Voltage Suppressors for ESD Protection" H 8605 9265 50  0001 L BNN "Description"
F 5 "0.7" H 8605 9165 50  0001 L BNN "Height"
F 6 "LRC" H 8605 9065 50  0001 L BNN "Manufacturer_Name"
F 7 "LESD5D5.0CT1G" H 8605 8965 50  0001 L BNN "Manufacturer_Part_Number"
F 8 "" H 8605 8865 50  0001 L BNN "Arrow Part Number"
F 9 "" H 8605 8765 50  0001 L BNN "Arrow Price/Stock"
F 10 "" H 8605 8665 50  0001 L BNN "Mouser Part Number"
F 11 "" H 8605 8565 50  0001 L BNN "Mouser Price/Stock"
	1    8005 9315
	0    1    1    0   
$EndComp
Text Notes 5935 7995 0    31   ~ 6
50V(10%)
Text Notes 5905 9950 0    31   ~ 6
50V(10%)
Text Notes 5965 11120 0    50   ~ 10
RESET BUTTON
$Comp
L power:GND #PWR0138
U 1 1 5FA87D9C
P 5390 8180
AR Path="/5FA87D9C" Ref="#PWR0138"  Part="1" 
AR Path="/5FC2D7E3/5FA87D9C" Ref="#PWR?"  Part="1" 
AR Path="/5FC1D29D/5FA87D9C" Ref="#PWR03"  Part="1" 
F 0 "#PWR0138" H 5390 7930 50  0001 C CNN
F 1 "GND" H 5385 8025 50  0000 C CNN
F 2 "" H 5390 8180 50  0001 C CNN
F 3 "" H 5390 8180 50  0001 C CNN
	1    5390 8180
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0139
U 1 1 611F9F95
P 6615 9980
AR Path="/611F9F95" Ref="#PWR0139"  Part="1" 
AR Path="/5FC2D7E3/611F9F95" Ref="#PWR?"  Part="1" 
AR Path="/5FC1D29D/611F9F95" Ref="#PWR04"  Part="1" 
F 0 "#PWR0139" H 6615 9730 50  0001 C CNN
F 1 "GND" H 6615 9830 50  0000 C CNN
F 2 "" H 6615 9980 50  0001 C CNN
F 3 "" H 6615 9980 50  0001 C CNN
	1    6615 9980
	-1   0    0    1   
$EndComp
Text GLabel 5415 10280 0    50   Input ~ 0
EN
Text GLabel 6540 7930 2    50   Input ~ 0
IO0
Wire Wire Line
	5415 10280 5715 10280
Wire Wire Line
	6540 7930 6290 7930
Wire Wire Line
	5390 7930 5390 8180
$Comp
L Device:C C1
U 1 1 5FA87D8B
P 6015 10130
AR Path="/5FA87D8B" Ref="C1"  Part="1" 
AR Path="/5FC2D7E3/5FA87D8B" Ref="C?"  Part="1" 
AR Path="/5FC1D29D/5FA87D8B" Ref="C1"  Part="1" 
F 0 "C1" V 5985 10030 50  0000 C BNN
F 1 "0.1uF" V 6170 10130 50  0000 C BNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 6053 9980 50  0001 C CNN
F 3 "~" H 6015 10130 50  0001 C CNN
	1    6015 10130
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5715 10280 5715 10130
Wire Wire Line
	5715 10130 5865 10130
Wire Wire Line
	6165 10130 6615 10130
$Comp
L Connector:Conn_01x04_Male J1
U 1 1 5FA87D82
P 6315 10730
AR Path="/5FA87D82" Ref="J1"  Part="1" 
AR Path="/5FC2D7E3/5FA87D82" Ref="J?"  Part="1" 
AR Path="/5FC1D29D/5FA87D82" Ref="J1"  Part="1" 
F 0 "J1" H 6465 10430 50  0000 L CNN
F 1 "Conn_01x04_Male" V 6375 10365 50  0000 L CNN
F 2 "Connector_Phoenix_MSTB:PhoenixContact_MSTBVA_2,5_4-G-5,08_1x04_P5.08mm_Vertical" H 6315 10730 50  0001 C CNN
F 3 "~" H 6315 10730 50  0001 C CNN
	1    6315 10730
	0    1    -1   0   
$EndComp
Text Notes 6415 10730 3    50   ~ 0
GND_led\n
Text Notes 6315 10730 3    50   ~ 0
GND
Text Notes 6215 10730 3    50   ~ 0
En_led
Text Notes 6115 10730 3    50   ~ 0
En
Wire Wire Line
	6415 10530 6415 10430
Wire Wire Line
	6415 10280 6615 10280
Wire Wire Line
	6615 10280 6615 10130
Wire Wire Line
	6315 10530 6315 10430
Wire Wire Line
	6415 10430 6315 10430
Connection ~ 6415 10430
Wire Wire Line
	6415 10430 6415 10280
Wire Wire Line
	6115 10530 6115 10430
Wire Wire Line
	6215 10530 6215 10430
Wire Wire Line
	6215 10430 6115 10430
Connection ~ 6115 10430
Wire Wire Line
	6115 10430 6115 10280
$Comp
L Device:C C2
U 1 1 5FA87D6A
P 6040 8180
AR Path="/5FA87D6A" Ref="C2"  Part="1" 
AR Path="/5FC2D7E3/5FA87D6A" Ref="C?"  Part="1" 
AR Path="/5FC1D29D/5FA87D6A" Ref="C2"  Part="1" 
F 0 "C2" V 5895 8185 50  0000 C CNN
F 1 "0.1uF" V 6180 8170 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 6078 8030 50  0001 C CNN
F 3 "~" H 6040 8180 50  0001 C CNN
	1    6040 8180
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6190 8180 6290 8180
Wire Wire Line
	6290 8180 6290 7930
Connection ~ 6290 7930
Wire Wire Line
	6290 7930 5940 7930
Connection ~ 5390 8180
Wire Wire Line
	5890 8180 5390 8180
Wire Wire Line
	5540 7930 5390 7930
Connection ~ 5715 10280
Wire Wire Line
	6115 10280 5715 10280
Connection ~ 6615 10130
Wire Wire Line
	6615 9980 6615 10130
Text Notes 5390 7690 0    100  ~ 20
Auto-Programming
Text Notes 5815 9475 0    31   ~ 6
(5%)
Text Notes 5815 8780 0    31   ~ 6
(5%)
Text GLabel 5400 8700 0    50   BiDi ~ 0
DTR
Text GLabel 5415 9400 0    50   BiDi ~ 0
RTS
Text GLabel 6465 9700 2    50   Input ~ 0
IO0
Text GLabel 6465 8400 2    50   Input ~ 0
EN
$Comp
L Device:R R1
U 1 1 5FA7973F
P 5715 8700
AR Path="/5FA7973F" Ref="R1"  Part="1" 
AR Path="/5FC2D7E3/5FA7973F" Ref="R?"  Part="1" 
AR Path="/5FC1D29D/5FA7973F" Ref="R2"  Part="1" 
F 0 "R1" V 5795 8705 50  0000 C CNN
F 1 "10k" V 5710 8695 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 5645 8700 50  0001 C CNN
F 3 "~" H 5715 8700 50  0001 C CNN
	1    5715 8700
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5FA79739
P 5715 9400
AR Path="/5FA79739" Ref="R2"  Part="1" 
AR Path="/5FC2D7E3/5FA79739" Ref="R?"  Part="1" 
AR Path="/5FC1D29D/5FA79739" Ref="R1"  Part="1" 
F 0 "R2" V 5800 9405 50  0000 C CNN
F 1 "10k" V 5715 9400 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 5645 9400 50  0001 C CNN
F 3 "~" H 5715 9400 50  0001 C CNN
	1    5715 9400
	0    1    1    0   
$EndComp
Wire Wire Line
	5415 9400 5465 9400
Wire Wire Line
	6265 9000 5465 9000
Connection ~ 5465 9400
Wire Wire Line
	5465 9400 5565 9400
Wire Wire Line
	6265 9100 5515 9100
Wire Wire Line
	6265 8400 6465 8400
Text Notes 10075 9575 0    31   ~ 6
(5%)
Text Notes 8840 8160 0    31   ~ 6
(5%)
Text Notes 8325 8505 0    31   ~ 6
(5%)
Text Notes 8410 8640 0    31   ~ 6
(5%)
Text Notes 9710 7845 0    31   ~ 6
50V(10%)
Text Notes 9695 7450 0    31   ~ 6
6.3V(10%)
NoConn ~ 9855 9115
NoConn ~ 9855 9015
NoConn ~ 9855 8815
NoConn ~ 9855 8415
$Comp
L power:+3.3V #PWR0140
U 1 1 5FC257C9
P 9205 7565
AR Path="/5FC257C9" Ref="#PWR0140"  Part="1" 
AR Path="/5FC1D29D/5FC257C9" Ref="#PWR0174"  Part="1" 
F 0 "#PWR0140" H 9205 7415 50  0001 C CNN
F 1 "+3.3V" V 9215 7685 50  0000 L CNN
F 2 "" H 9205 7565 50  0001 C CNN
F 3 "" H 9205 7565 50  0001 C CNN
	1    9205 7565
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR0141
U 1 1 5FC257C3
P 8755 7865
AR Path="/5FC257C3" Ref="#PWR0141"  Part="1" 
AR Path="/5FC1D29D/5FC257C3" Ref="#PWR0173"  Part="1" 
F 0 "#PWR0141" H 8755 7715 50  0001 C CNN
F 1 "+3.3V" V 8750 7985 50  0000 L CNN
F 2 "" H 8755 7865 50  0001 C CNN
F 3 "" H 8755 7865 50  0001 C CNN
	1    8755 7865
	1    0    0    -1  
$EndComp
Wire Wire Line
	10005 9615 10005 9715
$Comp
L Device:R R42
U 1 1 5FC257B1
P 10005 9465
AR Path="/5FC257B1" Ref="R42"  Part="1" 
AR Path="/5FC1D29D/5FC257B1" Ref="R42"  Part="1" 
F 0 "R42" H 9955 9415 50  0000 R CNN
F 1 "10k" H 9950 9490 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 9935 9465 50  0001 C CNN
F 3 "~" H 10005 9465 50  0001 C CNN
	1    10005 9465
	-1   0    0    1   
$EndComp
Wire Wire Line
	9855 9315 10005 9315
Text Notes 10325 9045 0    100  ~ 20
MicroUSB\nUSB-UART Serial
$Comp
L Device:C C33
U 1 1 611F9FAA
P 9655 7965
AR Path="/611F9FAA" Ref="C33"  Part="1" 
AR Path="/5FC1D29D/611F9FAA" Ref="C33"  Part="1" 
F 0 "C33" V 9520 7970 50  0000 C CNN
F 1 "0.1uF" V 9795 8035 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 9693 7815 50  0001 C CNN
F 3 "~" H 9655 7965 50  0001 C CNN
	1    9655 7965
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9355 7565 9205 7565
NoConn ~ 9855 10615
NoConn ~ 9855 10515
NoConn ~ 9855 10415
NoConn ~ 9855 10315
NoConn ~ 9855 10215
NoConn ~ 9855 10115
NoConn ~ 9855 10015
NoConn ~ 9855 9815
NoConn ~ 9855 9715
NoConn ~ 9855 9615
$Comp
L power:GND #PWR0142
U 1 1 5FC25797
P 9355 10815
AR Path="/5FC25797" Ref="#PWR0142"  Part="1" 
AR Path="/5FC1D29D/5FC25797" Ref="#PWR0172"  Part="1" 
F 0 "#PWR0142" H 9355 10565 50  0001 C CNN
F 1 "GND" H 9360 10642 50  0000 C CNN
F 2 "" H 9355 10815 50  0001 C CNN
F 3 "" H 9355 10815 50  0001 C CNN
	1    9355 10815
	1    0    0    -1  
$EndComp
Wire Wire Line
	9855 8915 9955 8915
Text GLabel 9955 8915 2    50   BiDi ~ 0
DTR
Wire Wire Line
	9855 8715 9955 8715
Wire Wire Line
	9855 8615 9955 8615
Wire Wire Line
	9855 8515 9955 8515
Text GLabel 9955 8715 2    50   BiDi ~ 0
TXD
Text GLabel 9955 8615 2    50   BiDi ~ 0
RXD
Text GLabel 9955 8515 2    50   BiDi ~ 0
RTS
Wire Wire Line
	9805 7565 9855 7565
Connection ~ 9355 7565
Wire Wire Line
	9505 7565 9355 7565
Wire Wire Line
	9855 7965 9855 8065
Wire Wire Line
	9855 7965 9855 7565
Connection ~ 9855 7965
Wire Wire Line
	9805 7965 9855 7965
Wire Wire Line
	9355 7565 9355 7965
Connection ~ 9355 7965
Wire Wire Line
	9505 7965 9355 7965
$Comp
L power:GND #PWR0143
U 1 1 611F9FA8
P 9855 8065
AR Path="/611F9FA8" Ref="#PWR0143"  Part="1" 
AR Path="/5FC1D29D/611F9FA8" Ref="#PWR0171"  Part="1" 
F 0 "#PWR0143" H 9855 7815 50  0001 C CNN
F 1 "GND" H 9860 7892 50  0000 C CNN
F 2 "" H 9855 8065 50  0001 C CNN
F 3 "" H 9855 8065 50  0001 C CNN
	1    9855 8065
	1    0    0    -1  
$EndComp
$Comp
L Device:C C32
U 1 1 611F9FA7
P 9655 7565
AR Path="/611F9FA7" Ref="C32"  Part="1" 
AR Path="/5FC1D29D/611F9FA7" Ref="C32"  Part="1" 
F 0 "C32" V 9520 7575 50  0000 C CNN
F 1 "4.7uF" V 9820 7650 50  0000 C BNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 9693 7415 50  0001 C CNN
F 3 "~" H 9655 7565 50  0001 C CNN
	1    9655 7565
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9355 7965 9355 8115
Wire Wire Line
	9355 8115 9355 8215
Connection ~ 9355 8115
Wire Wire Line
	9255 8115 9355 8115
Wire Wire Line
	9255 8215 9255 8115
NoConn ~ 9855 9415
$Comp
L power:GND #PWR0144
U 1 1 611F9FA6
P 10005 9715
AR Path="/611F9FA6" Ref="#PWR0144"  Part="1" 
AR Path="/5FC1D29D/611F9FA6" Ref="#PWR0170"  Part="1" 
F 0 "#PWR0144" H 10005 9465 50  0001 C CNN
F 1 "GND" H 10010 9542 50  0000 C CNN
F 2 "" H 10005 9715 50  0001 C CNN
F 3 "" H 10005 9715 50  0001 C CNN
	1    10005 9715
	1    0    0    -1  
$EndComp
Wire Wire Line
	8755 8615 8855 8615
Wire Wire Line
	8755 8265 8755 8615
Wire Wire Line
	8755 7865 8755 7965
$Comp
L Interface_USB:CP2102N-A01-GQFN28 U8
U 1 1 5FC25765
P 9355 9515
AR Path="/5FC25765" Ref="U8"  Part="1" 
AR Path="/5FC1D29D/5FC25765" Ref="U8"  Part="1" 
F 0 "U8" H 9005 8265 50  0000 C CNN
F 1 "CP2102N-A01-GQFN28" V 8905 9515 50  0000 C CNN
F 2 "Package_DFN_QFN:QFN-28-1EP_5x5mm_P0.5mm_EP3.35x3.35mm" H 9805 8315 50  0001 L CNN
F 3 "https://www.silabs.com/documents/public/data-sheets/cp2102n-datasheet.pdf" H 9405 8765 50  0001 C CNN
	1    9355 9515
	1    0    0    -1  
$EndComp
Wire Wire Line
	8555 8715 8855 8715
Wire Wire Line
	8255 8915 8855 8915
Wire Wire Line
	8505 9015 8855 9015
$Comp
L Device:R R41
U 1 1 5FC2575C
P 8755 8115
AR Path="/5FC2575C" Ref="R41"  Part="1" 
AR Path="/5FC1D29D/5FC2575C" Ref="R41"  Part="1" 
F 0 "R41" H 8825 8161 50  0000 L CNN
F 1 "2k" V 8755 8070 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 8685 8115 50  0001 C CNN
F 3 "~" H 8755 8115 50  0001 C CNN
	1    8755 8115
	1    0    0    -1  
$EndComp
Wire Wire Line
	8855 8715 8855 8815
Wire Wire Line
	8555 8515 8555 8715
Connection ~ 8555 8715
Wire Wire Line
	8455 8715 8555 8715
$Comp
L Device:R R39
U 1 1 5FC25752
P 8305 8715
AR Path="/5FC25752" Ref="R39"  Part="1" 
AR Path="/5FC1D29D/5FC25752" Ref="R39"  Part="1" 
F 0 "R39" V 8205 8715 50  0000 C CNN
F 1 "22.1k" V 8405 8715 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 8235 8715 50  0001 C CNN
F 3 "~" H 8305 8715 50  0001 C CNN
	1    8305 8715
	0    1    -1   0   
$EndComp
$Comp
L Device:R R40
U 1 1 5FC2574C
P 8555 8365
AR Path="/5FC2574C" Ref="R40"  Part="1" 
AR Path="/5FC1D29D/5FC2574C" Ref="R40"  Part="1" 
F 0 "R40" H 8475 8320 50  0000 R CNN
F 1 "47.5k" H 8486 8410 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 8485 8365 50  0001 C CNN
F 3 "~" H 8555 8365 50  0001 C CNN
	1    8555 8365
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR0145
U 1 1 5FC25746
P 8555 8215
AR Path="/5FC25746" Ref="#PWR0145"  Part="1" 
AR Path="/5FC1D29D/5FC25746" Ref="#PWR0169"  Part="1" 
F 0 "#PWR0145" H 8555 7965 50  0001 C CNN
F 1 "GND" H 8560 8042 50  0000 C CNN
F 2 "" H 8555 8215 50  0001 C CNN
F 3 "" H 8555 8215 50  0001 C CNN
	1    8555 8215
	1    0    0    1   
$EndComp
Wire Wire Line
	8005 8715 8155 8715
Wire Wire Line
	7605 9415 7905 9415
Wire Wire Line
	7605 9415 7505 9415
Connection ~ 7605 9415
Wire Wire Line
	7605 9315 7605 9415
Wire Wire Line
	8005 8715 8005 9315
Wire Wire Line
	8255 10115 8255 10265
Connection ~ 8005 8715
Wire Wire Line
	7905 8715 8005 8715
Wire Wire Line
	8505 9315 8505 9015
Wire Wire Line
	8255 9315 8255 8915
Connection ~ 8505 9015
Wire Wire Line
	7905 9015 8505 9015
Connection ~ 8255 8915
Wire Wire Line
	7905 8915 8255 8915
Wire Wire Line
	7505 9315 7505 9415
Wire Wire Line
	7505 9415 7505 9515
Connection ~ 7505 9415
Wire Wire Line
	7905 9115 7905 9415
$Comp
L power:GND #PWR0146
U 1 1 611F9FA0
P 7505 9515
AR Path="/611F9FA0" Ref="#PWR0146"  Part="1" 
AR Path="/5FC1D29D/611F9FA0" Ref="#PWR0168"  Part="1" 
F 0 "#PWR0146" H 7505 9265 50  0001 C CNN
F 1 "GND" H 7510 9342 50  0000 C CNN
F 2 "" H 7505 9515 50  0001 C CNN
F 3 "" H 7505 9515 50  0001 C CNN
	1    7505 9515
	1    0    0    -1  
$EndComp
Wire Wire Line
	8505 10115 8505 10265
Wire Wire Line
	8005 10115 8005 10265
$Comp
L power:GND #PWR0147
U 1 1 611F9F9F
P 8505 10265
AR Path="/611F9F9F" Ref="#PWR0147"  Part="1" 
AR Path="/5FC1D29D/611F9F9F" Ref="#PWR0167"  Part="1" 
F 0 "#PWR0147" H 8505 10015 50  0001 C CNN
F 1 "GND" H 8510 10092 50  0000 C CNN
F 2 "" H 8505 10265 50  0001 C CNN
F 3 "" H 8505 10265 50  0001 C CNN
	1    8505 10265
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0148
U 1 1 611F9F9E
P 8255 10265
AR Path="/611F9F9E" Ref="#PWR0148"  Part="1" 
AR Path="/5FC1D29D/611F9F9E" Ref="#PWR0166"  Part="1" 
F 0 "#PWR0148" H 8255 10015 50  0001 C CNN
F 1 "GND" H 8260 10092 50  0000 C CNN
F 2 "" H 8255 10265 50  0001 C CNN
F 3 "" H 8255 10265 50  0001 C CNN
	1    8255 10265
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0149
U 1 1 5FC2570E
P 8005 10265
AR Path="/5FC2570E" Ref="#PWR0149"  Part="1" 
AR Path="/5FC1D29D/5FC2570E" Ref="#PWR0165"  Part="1" 
F 0 "#PWR0149" H 8005 10015 50  0001 C CNN
F 1 "GND" H 8010 10092 50  0000 C CNN
F 2 "" H 8005 10265 50  0001 C CNN
F 3 "" H 8005 10265 50  0001 C CNN
	1    8005 10265
	1    0    0    -1  
$EndComp
$Comp
L LucidSens_2.01v-rescue:USB_B_Micro-Connector-MinimalSens_0.04v-rescue-LucidSens_1.05v-rescue J6
U 1 1 5FC25702
P 7605 8915
AR Path="/5FC25702" Ref="J6"  Part="1" 
AR Path="/5FC1D29D/5FC25702" Ref="J6"  Part="1" 
F 0 "J6" H 7605 9365 50  0000 C CNN
F 1 "USB_B_Micro" H 7605 9265 50  0000 C CNN
F 2 "Connector_USB:USB_Micro-B_Molex_47346-0001" H 7755 8865 50  0001 C CNN
F 3 "~" H 7755 8865 50  0001 C CNN
	1    7605 8915
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP13
U 1 1 60AF5776
P 8555 8715
F 0 "TP13" H 8515 8830 50  0000 R CNN
F 1 "TestPoint" H 8690 8920 50  0000 R CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 8755 8715 50  0001 C CNN
F 3 "~" H 8755 8715 50  0001 C CNN
	1    8555 8715
	-1   0    0    1   
$EndComp
$Comp
L Connector:TestPoint TP14
U 1 1 60AF5E5B
P 9355 7965
F 0 "TP14" V 9275 8115 50  0000 R CNN
F 1 "TestPoint" H 9490 8170 50  0000 R CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 9555 7965 50  0001 C CNN
F 3 "~" H 9555 7965 50  0001 C CNN
	1    9355 7965
	0    -1   -1   0   
$EndComp
$Comp
L Connector:TestPoint TP15
U 1 1 60AF6B03
P 5715 10280
F 0 "TP15" H 5700 10355 50  0000 R CNN
F 1 "TestPoint" H 5865 10485 50  0000 R CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 5915 10280 50  0001 C CNN
F 3 "~" H 5915 10280 50  0001 C CNN
	1    5715 10280
	-1   0    0    1   
$EndComp
Text Notes 10540 4445 0    100  ~ 20
HV_Power Supply and HV_CHK\n
Wire Wire Line
	13130 4770 13295 4770
$Comp
L Device:C C29
U 1 1 61549B28
P 13130 4920
F 0 "C29" H 13245 4966 50  0000 L CNN
F 1 "10nF" H 13245 4875 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 13168 4770 50  0001 C CNN
F 3 "~" H 13130 4920 50  0001 C CNN
	1    13130 4920
	1    0    0    -1  
$EndComp
Wire Wire Line
	12655 5070 13130 5070
Wire Wire Line
	12655 4770 12765 4770
$Comp
L Device:C C28
U 1 1 61389FDD
P 12655 4920
F 0 "C28" H 12770 4966 50  0000 L CNN
F 1 "10nF" H 12770 4875 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 12693 4770 50  0001 C CNN
F 3 "~" H 12655 4920 50  0001 C CNN
	1    12655 4920
	1    0    0    -1  
$EndComp
Connection ~ 13130 4770
Wire Wire Line
	13065 4770 13130 4770
Connection ~ 12655 5070
Wire Wire Line
	12180 5070 12655 5070
Connection ~ 12655 4770
Wire Wire Line
	12585 4770 12655 4770
$Comp
L LucidSens_2.01v-rescue:Q01-12CTR-Q01-12CTR-MinimalSens_0.04v-rescue-LucidSens_1.05v-rescue PS1
U 1 1 61389FDC
P 10180 4870
F 0 "PS1" H 11130 5135 50  0000 C CNN
F 1 "Q01-12CTR" H 11130 5044 50  0000 C CNN
F 2 "Q01-12CTR:Q0112CTR" H 11930 4970 50  0001 L CNN
F 3 "http://uk.rs-online.com/web/p/products/1238412" H 11930 4870 50  0001 L CNN
F 4 "XP Power Q01-12CTR DC to High Voltage DC Converter 0  12 V dc 5mA 100V dc" H 11930 4770 50  0001 L CNN "Description"
F 5 "12.7" H 11930 4670 50  0001 L CNN "Height"
F 6 "XP POWER" H 11930 4570 50  0001 L CNN "Manufacturer_Name"
F 7 "Q01-12CTR" H 11930 4470 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "Q01-12CTR" H 11930 4370 50  0001 L CNN "Arrow Part Number"
F 9 "" H 11930 4270 50  0001 L CNN "Arrow Price/Stock"
F 10 "" H 11930 4170 50  0001 L CNN "Mouser Part Number"
F 11 "" H 11930 4070 50  0001 L CNN "Mouser Price/Stock"
	1    10180 4870
	1    0    0    -1  
$EndComp
Text Notes 10120 5465 0    31   ~ 6
6.6-10V(10%)
Text Notes 12255 5870 0    31   ~ 6
6.6-10V(10%)
Text Notes 11825 5825 0    31   ~ 6
(5%)\n
Text Notes 11610 5585 0    31   ~ 6
(5%)\n
Wire Wire Line
	11110 5505 11360 5505
Text Notes 13175 5060 0    31   ~ 6
60-100V(10%)
Text Notes 12720 5060 0    31   ~ 6
60-100V(10%)
Text Notes 13015 4850 0    31   ~ 6
(5%)
Text Notes 12530 4855 0    31   ~ 6
(5%)
Wire Wire Line
	11765 5595 11765 5505
Connection ~ 11765 5505
Wire Wire Line
	11660 5505 11765 5505
Text Notes 12480 5650 0    50   ~ 10
HV CHK
Text GLabel 11110 5505 0    50   Input ~ 0
HV_OUT
$Comp
L power:GND #PWR0150
U 1 1 61549B2F
P 11765 5895
AR Path="/61549B2F" Ref="#PWR0150"  Part="1" 
AR Path="/5FBD9187/61549B2F" Ref="#PWR0163"  Part="1" 
F 0 "#PWR0150" H 11765 5645 50  0001 C CNN
F 1 "GND" H 11775 5740 50  0000 C CNN
F 2 "" H 11765 5895 50  0001 C CNN
F 3 "" H 11765 5895 50  0001 C CNN
	1    11765 5895
	1    0    0    -1  
$EndComp
$Comp
L Device:R R36
U 1 1 61389FE8
P 11765 5745
AR Path="/61389FE8" Ref="R36"  Part="1" 
AR Path="/5FBD9187/61389FE8" Ref="R36"  Part="1" 
F 0 "R36" H 11825 5825 50  0000 L CNN
F 1 "330k" H 11825 5755 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 11695 5745 50  0001 C CNN
F 3 "~" H 11765 5745 50  0001 C CNN
	1    11765 5745
	1    0    0    -1  
$EndComp
$Comp
L Device:R R34
U 1 1 61389FE7
P 11510 5505
AR Path="/61389FE7" Ref="R34"  Part="1" 
AR Path="/5FBD9187/61389FE7" Ref="R34"  Part="1" 
F 0 "R34" V 11590 5425 50  0000 L CNN
F 1 "5.1M" V 11505 5410 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 11440 5505 50  0001 C CNN
F 3 "~" H 11510 5505 50  0001 C CNN
	1    11510 5505
	0    -1   -1   0   
$EndComp
Wire Wire Line
	10080 4970 10180 4970
Wire Wire Line
	10080 4970 9990 4970
Connection ~ 10080 4970
Wire Wire Line
	10080 5165 10080 4970
$Comp
L power:GND #PWR0151
U 1 1 61549B2E
P 10080 5465
AR Path="/61549B2E" Ref="#PWR0151"  Part="1" 
AR Path="/5FBD9187/61549B2E" Ref="#PWR0162"  Part="1" 
F 0 "#PWR0151" H 10080 5215 50  0001 C CNN
F 1 "GND" H 10085 5292 50  0000 C CNN
F 2 "" H 10080 5465 50  0001 C CNN
F 3 "" H 10080 5465 50  0001 C CNN
	1    10080 5465
	1    0    0    -1  
$EndComp
$Comp
L Device:C C22
U 1 1 61389FE5
P 10080 5315
AR Path="/61389FE5" Ref="C22"  Part="1" 
AR Path="/5FBD9187/61389FE5" Ref="C22"  Part="1" 
F 0 "C22" H 10195 5361 50  0000 L CNN
F 1 "10nF" H 10195 5270 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 10118 5165 50  0001 C CNN
F 3 "~" H 10080 5315 50  0001 C CNN
	1    10080 5315
	1    0    0    -1  
$EndComp
Text Notes 12545 5455 0    50   ~ 10
ADC
Text Notes 10810 5275 0    50   ~ 10
High Votlage DC
Text Notes 13370 4705 0    50   ~ 10
~~30V
Text Notes 9740 5110 0    50   ~ 10
DAC_25\n
$Comp
L power:GND #PWR0152
U 1 1 61389FE4
P 12220 5870
AR Path="/61389FE4" Ref="#PWR0152"  Part="1" 
AR Path="/5FBD9187/61389FE4" Ref="#PWR0159"  Part="1" 
F 0 "#PWR0152" H 12220 5620 50  0001 C CNN
F 1 "GND" H 12225 5715 50  0000 C CNN
F 2 "" H 12220 5870 50  0001 C CNN
F 3 "" H 12220 5870 50  0001 C CNN
	1    12220 5870
	1    0    0    -1  
$EndComp
Text GLabel 13295 4770 2    50   Output ~ 0
HV_OUT
Wire Wire Line
	12080 4770 12285 4770
Wire Wire Line
	12080 4870 12080 4770
Wire Wire Line
	12180 5150 12180 5070
Connection ~ 12180 5070
Wire Wire Line
	12080 5070 12180 5070
$Comp
L Device:R R37
U 1 1 61549B2D
P 12915 4770
AR Path="/61549B2D" Ref="R37"  Part="1" 
AR Path="/5FBD9187/61549B2D" Ref="R37"  Part="1" 
F 0 "R37" V 12990 4780 50  0000 C CNN
F 1 "50" V 12910 4770 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 12845 4770 50  0001 C CNN
F 3 "~" H 12915 4770 50  0001 C CNN
	1    12915 4770
	0    1    -1   0   
$EndComp
$Comp
L Device:R R33
U 1 1 61549B2C
P 12435 4770
AR Path="/61549B2C" Ref="R33"  Part="1" 
AR Path="/5FBD9187/61549B2C" Ref="R33"  Part="1" 
F 0 "R33" V 12510 4775 50  0000 C CNN
F 1 "50" V 12430 4770 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 12365 4770 50  0001 C CNN
F 3 "~" H 12435 4770 50  0001 C CNN
	1    12435 4770
	0    1    -1   0   
$EndComp
Text GLabel 9990 4970 0    50   Input ~ 0
IO25
Wire Wire Line
	10030 4870 10180 4870
$Comp
L power:GND #PWR0153
U 1 1 61549B2B
P 10030 4870
AR Path="/61549B2B" Ref="#PWR0153"  Part="1" 
AR Path="/5FBD9187/61549B2B" Ref="#PWR0151"  Part="1" 
F 0 "#PWR0153" H 10030 4620 50  0001 C CNN
F 1 "GND" V 10035 4742 50  0000 R CNN
F 2 "" H 10030 4870 50  0001 C CNN
F 3 "" H 10030 4870 50  0001 C CNN
	1    10030 4870
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0154
U 1 1 61549B2A
P 12180 5150
AR Path="/61549B2A" Ref="#PWR0154"  Part="1" 
AR Path="/5FBD9187/61549B2A" Ref="#PWR0150"  Part="1" 
F 0 "#PWR0154" H 12180 4900 50  0001 C CNN
F 1 "GND" H 12045 5075 50  0000 C CNN
F 2 "" H 12180 5150 50  0001 C CNN
F 3 "" H 12180 5150 50  0001 C CNN
	1    12180 5150
	1    0    0    -1  
$EndComp
NoConn ~ 10180 5070
NoConn ~ 12080 4970
Wire Wire Line
	12220 5870 12220 5865
Wire Wire Line
	12220 5505 12520 5505
Wire Wire Line
	11765 5505 12220 5505
Connection ~ 12220 5505
Wire Wire Line
	12220 5505 12220 5565
$Comp
L Device:C C30
U 1 1 61549B29
P 12220 5715
AR Path="/61549B29" Ref="C30"  Part="1" 
AR Path="/5FBD9187/61549B29" Ref="C30"  Part="1" 
F 0 "C30" H 12310 5745 50  0000 L CNN
F 1 "0.1uF" H 12310 5665 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 12258 5565 50  0001 C CNN
F 3 "~" H 12220 5715 50  0001 C CNN
	1    12220 5715
	1    0    0    -1  
$EndComp
Text GLabel 12520 5505 2    50   Output ~ 0
IO36
$Comp
L Connector:TestPoint TP16
U 1 1 60B07A92
P 12220 5505
F 0 "TP16" H 12060 5690 50  0000 L CNN
F 1 "TestPoint" H 11860 5570 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 12420 5505 50  0001 C CNN
F 3 "~" H 12420 5505 50  0001 C CNN
	1    12220 5505
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP17
U 1 1 61549B31
P 13130 4770
F 0 "TP17" H 13188 4888 50  0000 L CNN
F 1 "TestPoint" H 12980 4985 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 13330 4770 50  0001 C CNN
F 3 "~" H 13330 4770 50  0001 C CNN
	1    13130 4770
	1    0    0    -1  
$EndComp
Text Notes 14535 9120 2    50   ~ 0
CS
Text Notes 14540 9225 2    50   ~ 0
DC\n
Text Notes 14550 9320 2    50   ~ 0
SDA/SDI
Text Notes 14550 9420 2    50   ~ 0
SCL/SCK
Text GLabel 14805 9085 0    50   Input ~ 0
IO5
Text GLabel 14805 9185 0    50   Input ~ 0
IO15
Text GLabel 14805 9285 0    50   BiDi ~ 0
IO23
Text GLabel 14805 9385 0    50   BiDi ~ 0
IO18
$Comp
L LucidSens_2.01v-rescue:ZM4733A-GS18-ZM4733A-GS18-MinimalSens_0.04v-rescue-LucidSens_1.05v-rescue D9
U 1 1 5FAF5C32
P 12890 8310
F 0 "D9" V 13140 8370 50  0000 L CNN
F 1 "ZM4733A-GS18" H 12865 8170 50  0000 L CNN
F 2 "ZM4733A-GS18:MELF_DO-213AB" H 13340 8310 50  0001 L CNN
F 3 "https://www.vishay.com/docs/85786/zm4728a.pdf" H 13340 8210 50  0001 L CNN
F 4 "VISHAY - ZM4733A-GS18 - ZENER DIODE, 1W, 5.1V, DO-213" H 13340 8110 50  0001 L CNN "Description"
F 5 "" H 13340 8010 50  0001 L CNN "Height"
F 6 "Vishay" H 13340 7910 50  0001 L CNN "Manufacturer_Name"
F 7 "ZM4733A-GS18" H 13340 7810 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "ZM4733A-GS18" H 13340 7710 50  0001 L CNN "Arrow Part Number"
F 9 "https://www.arrow.com/en/products/zm4733a-gs18/vishay" H 13340 7610 50  0001 L CNN "Arrow Price/Stock"
F 10 "625-ZM4733A-GS18" H 13340 7510 50  0001 L CNN "Mouser Part Number"
F 11 "https://www.mouser.co.uk/ProductDetail/Vishay-Semiconductors/ZM4733A-GS18?qs=6Fe8ASSgjNulBGqz5bPVMQ%3D%3D" H 13340 7410 50  0001 L CNN "Mouser Price/Stock"
	1    12890 8310
	0    1    1    0   
$EndComp
Wire Wire Line
	13435 8520 13435 8910
Wire Wire Line
	13145 8910 13435 8910
Wire Wire Line
	13145 8620 13145 8910
Connection ~ 13145 8910
Wire Wire Line
	12890 8910 13145 8910
Text Notes 13735 8150 0    39   ~ 8
Peltier_Fan
Text Notes 14955 8680 1    31   ~ 6
(5%)
Text Notes 12110 9330 0    31   ~ 6
(5%)
Text Notes 12575 9765 0    31   ~ 6
(5%)
Text Notes 12895 8785 0    31   ~ 6
(5.1V)
Text Notes 13200 8525 0    31   ~ 6
(5%)
Text Notes 13745 8600 0    31   ~ 6
(5%)
Text Notes 12320 8560 0    31   ~ 6
(10%)
$Comp
L power:+3.3V #PWR038
U 1 1 5FC414AA
P 15435 8000
AR Path="/5FC414AA" Ref="#PWR038"  Part="1" 
AR Path="/5FC2D7E3/5FC414AA" Ref="#PWR0188"  Part="1" 
F 0 "#PWR038" H 15435 7850 50  0001 C CNN
F 1 "+3.3V" H 15450 8173 50  0000 C CNN
F 2 "" H 15435 8000 50  0001 C CNN
F 3 "" H 15435 8000 50  0001 C CNN
	1    15435 8000
	0    1    1    0   
$EndComp
Wire Wire Line
	14695 8000 14835 8000
Wire Wire Line
	14695 8000 14695 7920
$Comp
L Switch:SW_DIP_x01 SW6
U 1 1 5FC414A0
P 15135 8000
AR Path="/5FC414A0" Ref="SW6"  Part="1" 
AR Path="/5FC2D7E3/5FC414A0" Ref="SW7"  Part="1" 
F 0 "SW6" H 15085 8145 50  0000 C CNN
F 1 "SW_DIP_x01" H 15135 7835 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_DIP_SPSTx01_Slide_Omron_A6S-110x_W8.9mm_P2.54mm" H 15135 8000 50  0001 C CNN
F 3 "~" H 15135 8000 50  0001 C CNN
	1    15135 8000
	-1   0    0    1   
$EndComp
$Comp
L LucidSens_2.01v-rescue:MMBT2222A-7-F-dk_Transistors-Bipolar-BJT-Single-MinimalSens_0.04v-rescue-LucidSens_1.05v-rescue Q7
U 1 1 5FC41496
P 14695 8220
AR Path="/5FC41496" Ref="Q7"  Part="1" 
AR Path="/5FC2D7E3/5FC41496" Ref="Q7"  Part="1" 
F 0 "Q7" H 14550 8380 60  0000 L CNN
F 1 "MMBT2222A" H 14445 8020 39  0000 L CNN
F 2 "MMBT2222A:SOT95P240X120-3N" H 14895 8420 60  0001 L CNN
F 3 "https://www.diodes.com/assets/Datasheets/ds30041.pdf" H 14895 8520 60  0001 L CNN
F 4 "MMBT2222A-FDICT-ND" H 14895 8620 60  0001 L CNN "Digi-Key_PN"
F 5 "MMBT2222A-7-F" H 14895 8720 60  0001 L CNN "MPN"
F 6 "Discrete Semiconductor Products" H 14895 8820 60  0001 L CNN "Category"
F 7 "Transistors - Bipolar (BJT) - Single" H 14895 8920 60  0001 L CNN "Family"
F 8 "https://www.diodes.com/assets/Datasheets/ds30041.pdf" H 14895 9020 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/diodes-incorporated/MMBT2222A-7-F/MMBT2222A-FDICT-ND/815723" H 14895 9120 60  0001 L CNN "DK_Detail_Page"
F 10 "TRANS NPN 40V 0.6A SMD SOT23-3" H 14895 9220 60  0001 L CNN "Description"
F 11 "Diodes Incorporated" H 14895 9320 60  0001 L CNN "Manufacturer"
F 12 "Active" H 14895 9420 60  0001 L CNN "Status"
	1    14695 8220
	0    -1   -1   0   
$EndComp
Text Notes 15230 7625 2    100  ~ 20
Buzzer
Wire Wire Line
	14620 8520 14695 8520
Wire Wire Line
	14495 8120 14495 7920
Wire Wire Line
	15095 8520 15120 8520
Wire Wire Line
	14895 8120 15095 8120
Wire Wire Line
	15095 8520 15095 8120
Connection ~ 15095 8520
Wire Wire Line
	15045 8520 15095 8520
Wire Wire Line
	14695 8520 14695 8420
Connection ~ 14695 8520
Wire Wire Line
	14745 8520 14695 8520
$Comp
L power:GND #PWR037
U 1 1 5FC4147C
P 15120 8520
AR Path="/5FC4147C" Ref="#PWR037"  Part="1" 
AR Path="/5FC2D7E3/5FC4147C" Ref="#PWR0187"  Part="1" 
F 0 "#PWR037" H 15120 8270 50  0001 C CNN
F 1 "GND" H 15125 8347 50  0000 C CNN
F 2 "" H 15120 8520 50  0001 C CNN
F 3 "" H 15120 8520 50  0001 C CNN
	1    15120 8520
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R49
U 1 1 5FC41476
P 14895 8520
AR Path="/5FC41476" Ref="R49"  Part="1" 
AR Path="/5FC2D7E3/5FC41476" Ref="R49"  Part="1" 
F 0 "R49" V 14895 8440 50  0000 L CNN
F 1 "2k" H 14750 8555 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 14825 8520 50  0001 C CNN
F 3 "~" H 14895 8520 50  0001 C CNN
	1    14895 8520
	0    -1   -1   0   
$EndComp
Text GLabel 14620 8520 0    50   Input ~ 0
IO4
$Comp
L Device:Buzzer BZ1
U 1 1 5FC4146F
P 14595 7820
AR Path="/5FC4146F" Ref="BZ1"  Part="1" 
AR Path="/5FC2D7E3/5FC4146F" Ref="BZ1"  Part="1" 
F 0 "BZ1" H 14445 7780 50  0000 L CNN
F 1 "Buzzer" H 14470 7995 50  0000 L CNN
F 2 "Buzzer_Beeper:Buzzer_12x9.5RM7.6" V 14570 7920 50  0001 C CNN
F 3 "~" V 14570 7920 50  0001 C CNN
	1    14595 7820
	0    1    -1   0   
$EndComp
Text GLabel 12450 9335 3    50   Input ~ 0
IO14
$Comp
L Device:R R43
U 1 1 5FC3C3AA
P 12165 9225
AR Path="/5FC3C3AA" Ref="R43"  Part="1" 
AR Path="/5FC2D7E3/5FC3C3AA" Ref="R43"  Part="1" 
F 0 "R43" V 12245 9230 50  0000 C CNN
F 1 "10k" V 12160 9220 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 12095 9225 50  0001 C CNN
F 3 "~" H 12165 9225 50  0001 C CNN
	1    12165 9225
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR03
U 1 1 5FC3C3A4
P 11985 9605
AR Path="/5FC3C3A4" Ref="#PWR03"  Part="1" 
AR Path="/5FC2D7E3/5FC3C3A4" Ref="#PWR0186"  Part="1" 
F 0 "#PWR03" H 11985 9355 50  0001 C CNN
F 1 "GND" H 11985 9455 50  0000 C CNN
F 2 "" H 11985 9605 50  0001 C CNN
F 3 "" H 11985 9605 50  0001 C CNN
	1    11985 9605
	1    0    0    -1  
$EndComp
Wire Wire Line
	11985 9225 12015 9225
Wire Wire Line
	12890 9425 12890 9585
$Comp
L Device:R R46
U 1 1 5FC3C39C
P 12635 9585
AR Path="/5FC3C39C" Ref="R46"  Part="1" 
AR Path="/5FC2D7E3/5FC3C39C" Ref="R46"  Part="1" 
F 0 "R46" V 12710 9585 50  0000 C CNN
F 1 "120" V 12540 9585 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 12565 9585 50  0001 C CNN
F 3 "~" H 12635 9585 50  0001 C CNN
	1    12635 9585
	0    -1   -1   0   
$EndComp
Wire Wire Line
	11985 9225 11985 9585
Wire Wire Line
	12890 9585 12785 9585
Wire Wire Line
	12485 9585 11985 9585
Connection ~ 11985 9585
Wire Wire Line
	11985 9585 11985 9605
Connection ~ 12890 8910
Wire Wire Line
	12890 9025 12890 8910
$Comp
L Device:R R47
U 1 1 5FC3C38E
P 13145 8470
AR Path="/5FC3C38E" Ref="R47"  Part="1" 
AR Path="/5FC2D7E3/5FC3C38E" Ref="R47"  Part="1" 
F 0 "R47" H 13030 8445 50  0000 C CNN
F 1 "120" V 13140 8475 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 13075 8470 50  0001 C CNN
F 3 "~" H 13145 8470 50  0001 C CNN
	1    13145 8470
	-1   0    0    1   
$EndComp
Wire Wire Line
	13145 8320 13145 8220
Wire Wire Line
	13145 8220 13235 8220
$Comp
L Device:C C34
U 1 1 5FC3C383
P 12280 8370
AR Path="/5FC3C383" Ref="C34"  Part="1" 
AR Path="/5FC2D7E3/5FC3C383" Ref="C34"  Part="1" 
F 0 "C34" H 12110 8275 50  0000 L CNN
F 1 "0.1uF" H 12295 8275 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 12318 8220 50  0001 C CNN
F 3 "~" H 12280 8370 50  0001 C CNN
	1    12280 8370
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR033
U 1 1 5FC3C37D
P 12280 8520
AR Path="/5FC3C37D" Ref="#PWR033"  Part="1" 
AR Path="/5FC2D7E3/5FC3C37D" Ref="#PWR0185"  Part="1" 
F 0 "#PWR033" H 12280 8270 50  0001 C CNN
F 1 "GND" H 12280 8370 50  0000 C CNN
F 2 "" H 12280 8520 50  0001 C CNN
F 3 "" H 12280 8520 50  0001 C CNN
	1    12280 8520
	1    0    0    -1  
$EndComp
Wire Wire Line
	12890 8310 12890 8220
Connection ~ 13145 8220
Wire Wire Line
	12890 8220 13145 8220
Wire Wire Line
	13635 8220 13690 8220
$Comp
L Device:R R48
U 1 1 5FC3C36A
P 13690 8550
AR Path="/5FC3C36A" Ref="R48"  Part="1" 
AR Path="/5FC2D7E3/5FC3C36A" Ref="R48"  Part="1" 
F 0 "R48" H 13725 8580 50  0000 L CNN
F 1 "47k" V 13690 8480 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 13620 8550 50  0001 C CNN
F 3 "~" H 13690 8550 50  0001 C CNN
	1    13690 8550
	1    0    0    -1  
$EndComp
Wire Wire Line
	13690 8220 13690 8400
$Comp
L power:GND #PWR036
U 1 1 5FC3C362
P 13690 8700
AR Path="/5FC3C362" Ref="#PWR036"  Part="1" 
AR Path="/5FC2D7E3/5FC3C362" Ref="#PWR0184"  Part="1" 
F 0 "#PWR036" H 13690 8450 50  0001 C CNN
F 1 "GND" H 13695 8527 50  0000 C CNN
F 2 "" H 13690 8700 50  0001 C CNN
F 3 "" H 13690 8700 50  0001 C CNN
	1    13690 8700
	1    0    0    -1  
$EndComp
Wire Wire Line
	12315 9225 12450 9225
Wire Wire Line
	12450 9335 12450 9225
Connection ~ 12450 9225
Wire Wire Line
	12450 9225 12590 9225
$Comp
L Connector:Conn_01x02_Male J10
U 1 1 5FC3C358
P 14050 8220
AR Path="/5FC3C358" Ref="J10"  Part="1" 
AR Path="/5FC2D7E3/5FC3C358" Ref="J10"  Part="1" 
F 0 "J10" H 14075 8055 50  0000 L CNN
F 1 "Conn_01x02_Male" V 14000 7870 50  0000 L CNN
F 2 "Connector_JST:JST_EH_B2B-EH-A_1x02_P2.50mm_Vertical" H 14050 8220 50  0001 C CNN
F 3 "~" H 14050 8220 50  0001 C CNN
	1    14050 8220
	-1   0    0    -1  
$EndComp
Connection ~ 13690 8220
Wire Wire Line
	13690 8220 13850 8220
Wire Wire Line
	13850 8320 13850 8700
Wire Wire Line
	13850 8700 13690 8700
Connection ~ 13690 8700
Text Notes 13000 9795 0    100  ~ 20
Fan controller
$Comp
L power:+12V #PWR04
U 1 1 5FC3C34A
P 12215 8220
AR Path="/5FC3C34A" Ref="#PWR04"  Part="1" 
AR Path="/5FC2D7E3/5FC3C34A" Ref="#PWR0183"  Part="1" 
F 0 "#PWR04" H 12215 8070 50  0001 C CNN
F 1 "+12V" V 12230 8350 50  0000 L CNN
F 2 "" H 12215 8220 50  0001 C CNN
F 3 "" H 12215 8220 50  0001 C CNN
	1    12215 8220
	0    -1   -1   0   
$EndComp
Wire Wire Line
	12215 8220 12280 8220
Connection ~ 12280 8220
Wire Wire Line
	12280 8220 12340 8220
Connection ~ 12890 8220
Connection ~ 12340 8220
Wire Wire Line
	12340 8220 12890 8220
$Comp
L Switch:SW_DIP_x02 SW5
U 1 1 5FC3C33C
P 12640 7965
AR Path="/5FC3C33C" Ref="SW5"  Part="1" 
AR Path="/5FC2D7E3/5FC3C33C" Ref="SW6"  Part="1" 
F 0 "SW5" H 12630 7815 50  0000 C CNN
F 1 "SW_DIP_x02" H 12635 8220 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_DIP_SPSTx02_Slide_Omron_A6S-210x_W8.9mm_P2.54mm" H 12640 7965 50  0001 C CNN
F 3 "~" H 12640 7965 50  0001 C CNN
	1    12640 7965
	1    0    0    -1  
$EndComp
Wire Wire Line
	12340 7965 12340 8220
Wire Wire Line
	12340 7965 12340 7865
Connection ~ 12340 7965
$Comp
L Connector:Conn_01x02_Male J8
U 1 1 5FC3C333
P 12940 8165
AR Path="/5FC3C333" Ref="J8"  Part="1" 
AR Path="/5FC2D7E3/5FC3C333" Ref="J8"  Part="1" 
F 0 "J8" V 12965 8270 50  0000 R CNN
F 1 "Conn_01x02_Male" V 13055 8045 50  0000 R CNN
F 2 "Connector_JST:JST_EH_B2B-EH-A_1x02_P2.50mm_Vertical" H 12940 8165 50  0001 C CNN
F 3 "~" H 12940 8165 50  0001 C CNN
	1    12940 8165
	0    -1   -1   0   
$EndComp
Wire Wire Line
	13040 7965 13160 7965
Text Notes 11925 7890 0    31   ~ 6
Fan_Default_ON
Text Notes 11925 7990 0    31   ~ 6
Fan_Backup_OFF
$Comp
L power:GND #PWR034
U 1 1 5FC3C327
P 13160 7965
AR Path="/5FC3C327" Ref="#PWR034"  Part="1" 
AR Path="/5FC2D7E3/5FC3C327" Ref="#PWR0182"  Part="1" 
F 0 "#PWR034" H 13160 7715 50  0001 C CNN
F 1 "GND" V 13160 7855 50  0000 R CNN
F 2 "" H 13160 7965 50  0001 C CNN
F 3 "" H 13160 7965 50  0001 C CNN
	1    13160 7965
	0    -1   -1   0   
$EndComp
$Comp
L Connector:Conn_01x02_Male J7
U 1 1 5FC3C321
P 12940 7665
AR Path="/5FC3C321" Ref="J7"  Part="1" 
AR Path="/5FC2D7E3/5FC3C321" Ref="J7"  Part="1" 
F 0 "J7" V 13090 7550 50  0000 R CNN
F 1 "Conn_01x02_Male" V 12980 7550 50  0000 R CNN
F 2 "Connector_JST:JST_EH_B2B-EH-A_1x02_P2.50mm_Vertical" H 12940 7665 50  0001 C CNN
F 3 "~" H 12940 7665 50  0001 C CNN
	1    12940 7665
	0    -1   1    0   
$EndComp
$Comp
L power:GND #PWR035
U 1 1 5FC3C31B
P 13175 7865
AR Path="/5FC3C31B" Ref="#PWR035"  Part="1" 
AR Path="/5FC2D7E3/5FC3C31B" Ref="#PWR0181"  Part="1" 
F 0 "#PWR035" H 13175 7615 50  0001 C CNN
F 1 "GND" V 13175 7760 50  0000 R CNN
F 2 "" H 13175 7865 50  0001 C CNN
F 3 "" H 13175 7865 50  0001 C CNN
	1    13175 7865
	0    -1   -1   0   
$EndComp
Wire Wire Line
	13040 7865 13175 7865
$Comp
L Connector_Generic:Conn_02x04_Counter_Clockwise J9
U 1 1 60AFB4BC
P 15305 9185
F 0 "J9" H 15360 9410 50  0000 C CNN
F 1 "Conn_02x04_Counter_Clockwise" H 15360 8840 50  0000 C CNN
F 2 "Connector_JST:JST_PHD_B8B-PHDSS_2x04_P2.00mm_Vertical" H 15305 9185 50  0001 C CNN
F 3 "~" H 15305 9185 50  0001 C CNN
	1    15305 9185
	1    0    0    -1  
$EndComp
$Comp
L Device:R R44
U 1 1 60AFBF5B
P 14955 9085
F 0 "R44" V 14915 9240 50  0000 C CNN
F 1 "1.2k" V 14955 9090 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 14885 9085 50  0001 C CNN
F 3 "~" H 14955 9085 50  0001 C CNN
	1    14955 9085
	0    1    1    0   
$EndComp
$Comp
L Device:R R45
U 1 1 60AFC994
P 14955 9185
F 0 "R45" V 14915 9340 50  0000 C CNN
F 1 "1.2k" V 14955 9190 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 14885 9185 50  0001 C CNN
F 3 "~" H 14955 9185 50  0001 C CNN
	1    14955 9185
	0    1    1    0   
$EndComp
$Comp
L Device:R R50
U 1 1 60AFCD43
P 14955 9285
F 0 "R50" V 14915 9440 50  0000 C CNN
F 1 "1.2k" V 14955 9290 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 14885 9285 50  0001 C CNN
F 3 "~" H 14955 9285 50  0001 C CNN
	1    14955 9285
	0    1    1    0   
$EndComp
$Comp
L Device:R R53
U 1 1 60AFD012
P 14955 9385
F 0 "R53" V 14915 9540 50  0000 C CNN
F 1 "1.2k" V 14955 9390 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 14885 9385 50  0001 C CNN
F 3 "~" H 14955 9385 50  0001 C CNN
	1    14955 9385
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR014
U 1 1 60AFD6EC
P 15605 9085
F 0 "#PWR014" H 15605 8935 50  0001 C CNN
F 1 "+3.3V" V 15600 9200 50  0000 L CNN
F 2 "" H 15605 9085 50  0001 C CNN
F 3 "" H 15605 9085 50  0001 C CNN
	1    15605 9085
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR015
U 1 1 60AFE6BF
P 15605 9185
F 0 "#PWR015" H 15605 9035 50  0001 C CNN
F 1 "+3.3V" V 15600 9300 50  0000 L CNN
F 2 "" H 15605 9185 50  0001 C CNN
F 3 "" H 15605 9185 50  0001 C CNN
	1    15605 9185
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR016
U 1 1 60AFE9D5
P 15605 9285
F 0 "#PWR016" H 15605 9135 50  0001 C CNN
F 1 "+3.3V" V 15600 9400 50  0000 L CNN
F 2 "" H 15605 9285 50  0001 C CNN
F 3 "" H 15605 9285 50  0001 C CNN
	1    15605 9285
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR017
U 1 1 60AFEE5F
P 15605 9385
F 0 "#PWR017" H 15605 9135 50  0001 C CNN
F 1 "GND" V 15610 9257 50  0000 R CNN
F 2 "" H 15605 9385 50  0001 C CNN
F 3 "" H 15605 9385 50  0001 C CNN
	1    15605 9385
	0    -1   -1   0   
$EndComp
Text Notes 15215 9810 2    100  ~ 20
TFT
$Comp
L Connector:TestPoint TP3
U 1 1 60B06052
P 12280 8220
F 0 "TP3" H 12115 8405 50  0000 L CNN
F 1 "TestPoint" H 11890 8345 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 12480 8220 50  0001 C CNN
F 3 "~" H 12480 8220 50  0001 C CNN
	1    12280 8220
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP4
U 1 1 60B06A8C
P 13690 8220
F 0 "TP4" H 13710 8170 50  0000 L CNN
F 1 "TestPoint" H 13505 8450 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 13890 8220 50  0001 C CNN
F 3 "~" H 13890 8220 50  0001 C CNN
	1    13690 8220
	1    0    0    -1  
$EndComp
$Comp
L Sensor:DHT11 U6
U 1 1 6173D48A
P 11100 7425
AR Path="/6173D48A" Ref="U6"  Part="1" 
AR Path="/5FC2D7E3/6173D48A" Ref="U?"  Part="1" 
AR Path="/5FC33348/6173D48A" Ref="U6"  Part="1" 
F 0 "U6" V 10925 7210 50  0000 R CNN
F 1 "DHT22" V 11340 7400 50  0000 R CNN
F 2 "Sensor:Aosong_DHT11_5.5x12.0_P2.54mm" H 11100 7025 50  0001 C CNN
F 3 "http://akizukidenshi.com/download/ds/aosong/DHT11.pdf" H 11250 7675 50  0001 C CNN
	1    11100 7425
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R30
U 1 1 6173D48B
P 11100 6775
AR Path="/6173D48B" Ref="R30"  Part="1" 
AR Path="/5FC2D7E3/6173D48B" Ref="R?"  Part="1" 
AR Path="/5FC33348/6173D48B" Ref="R30"  Part="1" 
F 0 "R30" V 11100 6775 50  0000 C CNN
F 1 "10k" H 10975 6775 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 11030 6775 50  0001 C CNN
F 3 "~" H 11100 6775 50  0001 C CNN
	1    11100 6775
	1    0    0    -1  
$EndComp
Wire Wire Line
	11100 6525 10750 6525
Wire Wire Line
	10800 7425 10750 7425
Connection ~ 10750 7425
Wire Wire Line
	10750 7425 10750 6525
Wire Wire Line
	11100 6625 11100 6525
Text Notes 12025 6475 2    100  ~ 20
Temperature Sensor
Text Notes 10915 6870 0    31   ~ 6
(5%)
Wire Wire Line
	11100 7125 11100 7025
Wire Wire Line
	11100 7025 11100 6925
Connection ~ 11100 7025
Text Notes 11580 6995 2    31   ~ 6
(10%)
Text Notes 10545 7145 2    31   ~ 6
(10%)
Wire Wire Line
	10750 7425 10595 7425
Wire Wire Line
	10375 7425 10595 7425
Connection ~ 10595 7425
$Comp
L Device:C C?
U 1 1 6173D48C
P 10595 7275
AR Path="/5FC2D7E3/6173D48C" Ref="C?"  Part="1" 
AR Path="/6173D48C" Ref="C23"  Part="1" 
AR Path="/5FC33348/6173D48C" Ref="C23"  Part="1" 
F 0 "C23" H 10610 7370 50  0000 L CNN
F 1 "0.1uF" H 10615 7195 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 10633 7125 50  0001 C CNN
F 3 "~" H 10595 7275 50  0001 C CNN
	1    10595 7275
	-1   0    0    1   
$EndComp
$Comp
L power:+3.3V #PWR029
U 1 1 60BA8A0E
P 10375 7425
F 0 "#PWR029" H 10375 7275 50  0001 C CNN
F 1 "+3.3V" V 10390 7553 50  0000 L CNN
F 2 "" H 10375 7425 50  0001 C CNN
F 3 "" H 10375 7425 50  0001 C CNN
	1    10375 7425
	0    -1   -1   0   
$EndComp
$Comp
L Connector:TestPoint TP22
U 1 1 60BA9499
P 10595 7425
F 0 "TP22" H 10745 7510 50  0000 R CNN
F 1 "TestPoint" H 10740 7635 50  0000 R CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 10795 7425 50  0001 C CNN
F 3 "~" H 10795 7425 50  0001 C CNN
	1    10595 7425
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR030
U 1 1 6173D490
P 10595 7125
F 0 "#PWR030" H 10595 6875 50  0001 C CNN
F 1 "GND" H 10600 6952 50  0000 C CNN
F 2 "" H 10595 7125 50  0001 C CNN
F 3 "" H 10595 7125 50  0001 C CNN
	1    10595 7125
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR031
U 1 1 6173D491
P 11270 6725
F 0 "#PWR031" H 11270 6475 50  0001 C CNN
F 1 "GND" H 11270 6580 50  0000 C CNN
F 2 "" H 11270 6725 50  0001 C CNN
F 3 "" H 11270 6725 50  0001 C CNN
	1    11270 6725
	-1   0    0    1   
$EndComp
Text GLabel 11585 7025 2    50   Input ~ 0
IO27
$Comp
L power:GND #PWR032
U 1 1 6173D492
P 11400 7425
F 0 "#PWR032" H 11400 7175 50  0001 C CNN
F 1 "GND" V 11405 7310 50  0000 R CNN
F 2 "" H 11400 7425 50  0001 C CNN
F 3 "" H 11400 7425 50  0001 C CNN
	1    11400 7425
	0    -1   -1   0   
$EndComp
Wire Wire Line
	11270 7025 11585 7025
Wire Wire Line
	11270 7025 11100 7025
Connection ~ 11270 7025
$Comp
L Device:C C?
U 1 1 6173D48D
P 11270 6875
AR Path="/5FC2D7E3/6173D48D" Ref="C?"  Part="1" 
AR Path="/6173D48D" Ref="C24"  Part="1" 
AR Path="/5FC33348/6173D48D" Ref="C24"  Part="1" 
F 0 "C24" H 11055 6725 50  0000 L CNN
F 1 "0.01uF" H 11000 6800 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 11308 6725 50  0001 C CNN
F 3 "~" H 11270 6875 50  0001 C CNN
	1    11270 6875
	-1   0    0    1   
$EndComp
Wire Wire Line
	14485 6195 14485 5995
Wire Wire Line
	13535 6195 14485 6195
Wire Wire Line
	13735 6730 13535 6730
Wire Wire Line
	13235 6260 13235 6630
Wire Wire Line
	15235 6260 15235 6630
Wire Wire Line
	14385 6260 13235 6260
Wire Wire Line
	14385 6260 15235 6260
Connection ~ 14385 6260
Wire Wire Line
	14385 5995 14385 6260
Wire Wire Line
	13730 6630 13735 6630
Wire Wire Line
	13730 6145 13730 6630
Wire Wire Line
	15535 6145 15535 6730
Wire Wire Line
	14585 6145 13730 6145
Wire Wire Line
	14585 6145 15535 6145
Connection ~ 14585 6145
Wire Wire Line
	14585 5995 14585 6145
$Comp
L LucidSens_2.01v-rescue:ITR-9608-ITR-9608-MinimalSens_0.04v-rescue-LucidSens_1.05v-rescue S1
U 1 1 60937B77
P 15235 6730
F 0 "S1" H 15985 6365 50  0000 C CNN
F 1 "ITR-9608-ITR-9608" H 15985 6456 50  0000 C CNN
F 2 "ITR-9608:ITR9608" H 16585 6830 50  0001 L CNN
F 3 "https://media.digikey.com/pdf/Data%20Sheets/Everlight%20PDFs/ITR9608-F.pdf" H 16585 6730 50  0001 L CNN
F 4 "Optical Switches, Reflective, Phototransistor Output Opto Interrupter" H 16585 6630 50  0001 L CNN "Description"
F 5 "10.45" H 16585 6530 50  0001 L CNN "Height"
F 6 "Everlight" H 16585 6430 50  0001 L CNN "Manufacturer_Name"
F 7 "ITR-9608" H 16585 6330 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "ITR-9608" H 16585 6230 50  0001 L CNN "Arrow Part Number"
F 9 "https://www.arrow.com/en/products/itr-9608/everlight-electronics" H 16585 6130 50  0001 L CNN "Arrow Price/Stock"
F 10 "638-ITR9608" H 16585 6030 50  0001 L CNN "Mouser Part Number"
F 11 "https://www.mouser.co.uk/ProductDetail/Everlight/ITR-9608?qs=8cKuZ6Ok2lY35YbtvOHkMw%3D%3D" H 16585 5930 50  0001 L CNN "Mouser Price/Stock"
	1    15235 6730
	-1   0    0    1   
$EndComp
Wire Wire Line
	13535 6630 13535 6195
Wire Wire Line
	13535 6730 13535 6630
Connection ~ 13535 6630
$Comp
L Device:R R?
U 1 1 618CF9E6
P 13385 6630
AR Path="/5FAE0F92/618CF9E6" Ref="R?"  Part="1" 
AR Path="/618CF9E6" Ref="R11"  Part="1" 
AR Path="/5FC2C912/618CF9E6" Ref="R11"  Part="1" 
F 0 "R11" V 13305 6635 50  0000 C CNN
F 1 "47k" V 13390 6625 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 13315 6630 50  0001 C CNN
F 3 "~" H 13385 6630 50  0001 C CNN
	1    13385 6630
	0    1    1    0   
$EndComp
$Comp
L Device:R R?
U 1 1 5FC32D89
P 15385 6730
AR Path="/5FAE0F92/5FC32D89" Ref="R?"  Part="1" 
AR Path="/5FC32D89" Ref="R16"  Part="1" 
AR Path="/5FC2C912/5FC32D89" Ref="R16"  Part="1" 
F 0 "R16" V 15455 6720 50  0000 C CNN
F 1 "750" V 15380 6725 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 15315 6730 50  0001 C CNN
F 3 "~" H 15385 6730 50  0001 C CNN
	1    15385 6730
	0    -1   -1   0   
$EndComp
Text Notes 13330 6735 0    31   ~ 6
(5%)
Text Notes 15335 6830 0    31   ~ 6
(5%)
$Comp
L Connector:Conn_01x03_Male J14
U 1 1 5FCDA2F0
P 14485 5795
F 0 "J14" V 14645 5965 50  0000 L CNN
F 1 "Conn_01x03_Male" V 14555 5480 50  0000 L CNN
F 2 "Connector_JST:JST_EH_B3B-EH-A_1x03_P2.50mm_Vertical" H 14485 5795 50  0001 C CNN
F 3 "~" H 14485 5795 50  0001 C CNN
	1    14485 5795
	0    1    1    0   
$EndComp
Text Notes 14540 5990 3    31   ~ 6
IO35
Text Notes 14645 5995 3    31   ~ 6
3.3V
Text Notes 14440 6095 1    31   ~ 6
GND
Text Notes 14610 5775 1    31   ~ 6
3.3V
Text Notes 14410 5775 1    31   ~ 6
GND
Text Notes 14510 5780 1    31   ~ 6
IO35
Text Notes 13260 5950 0    100  ~ 20
OptoSwitch
Text Notes 1915 10795 0    31   ~ 0
455 uA/A
$Comp
L Transistor_FET:2N7002K Q5
U 1 1 60D1DC03
P 12790 9225
F 0 "Q5" H 12710 9390 50  0000 L CNN
F 1 "NTR4501" V 13015 9055 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 12990 9150 50  0001 L CIN
F 3 "https://www.diodes.com/assets/Datasheets/ds30896.pdf" H 12790 9225 50  0001 L CNN
	1    12790 9225
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:AO3401A Q6
U 1 1 60D2BED9
P 13435 8320
F 0 "Q6" V 13470 8440 50  0000 L CNN
F 1 "DMP2160U" H 12990 8410 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 13635 8245 50  0001 L CIN
F 3 "http://www.aosmd.com/pdfs/datasheet/AO3401A.pdf" H 13435 8320 50  0001 L CNN
	1    13435 8320
	0    1    -1   0   
$EndComp
Text GLabel 2325 2600 0    50   Output ~ 0
IO26
Text Notes 1595 2635 0    50   ~ 10
DAC(WE.)
Text Notes 8100 6080 0    31   ~ 6
(10%)
Text Notes 5170 6665 1    31   ~ 6
(5%)
$Comp
L LucidSens_2.01v-rescue:ZM4733A-GS18-ZM4733A-GS18-MinimalSens_0.04v-rescue-LucidSens_1.05v-rescue D11
U 1 1 612D1F41
P 8435 6030
F 0 "D11" H 8580 5980 50  0000 C CNN
F 1 "ZM4733A-GS18" H 8725 6165 50  0000 C CNN
F 2 "ZM4733A-GS18:MELF_DO-213AB" H 8885 6030 50  0001 L CNN
F 3 "https://www.vishay.com/docs/85786/zm4728a.pdf" H 8885 5930 50  0001 L CNN
F 4 "VISHAY - ZM4733A-GS18 - ZENER DIODE, 1W, 5.1V, DO-213" H 8885 5830 50  0001 L CNN "Description"
F 5 "" H 8885 5730 50  0001 L CNN "Height"
F 6 "Vishay" H 8885 5630 50  0001 L CNN "Manufacturer_Name"
F 7 "ZM4733A-GS18" H 8885 5530 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "ZM4733A-GS18" H 8885 5430 50  0001 L CNN "Arrow Part Number"
F 9 "https://www.arrow.com/en/products/zm4733a-gs18/vishay" H 8885 5330 50  0001 L CNN "Arrow Price/Stock"
F 10 "625-ZM4733A-GS18" H 8885 5230 50  0001 L CNN "Mouser Part Number"
F 11 "https://www.mouser.co.uk/ProductDetail/Vishay-Semiconductors/ZM4733A-GS18?qs=6Fe8ASSgjNulBGqz5bPVMQ%3D%3D" H 8885 5130 50  0001 L CNN "Mouser Price/Stock"
	1    8435 6030
	1    0    0    -1  
$EndComp
Wire Wire Line
	9035 6030 9100 6030
Connection ~ 9035 6030
Wire Wire Line
	9035 5775 9035 6030
Wire Wire Line
	8745 5775 9035 5775
Wire Wire Line
	8645 5485 9035 5485
Connection ~ 9035 5775
Wire Wire Line
	9035 5485 9035 5775
Text Notes 4355 6835 1    31   ~ 6
(5%)
Text Notes 5095 6275 0    31   ~ 6
(10%)
Text Notes 5425 6400 0    31   ~ 6
(10%)
Text Notes 5630 6705 0    31   ~ 6
(10%)
Text Notes 5290 7170 0    31   ~ 6
(10%)
Text Notes 5710 7105 0    31   ~ 6
(10%)
Text Notes 8590 6480 0    31   ~ 6
(10%)
Text Notes 9365 6805 0    31   ~ 6
(5%)
Text Notes 8670 6180 0    31   ~ 6
5.1V
Text Notes 8625 5330 0    31   ~ 6
(5%)
Text Notes 8695 5840 0    31   ~ 6
(5%)
Text Notes 9485 6325 0    31   ~ 6
(5%)
Text Notes 7430 6085 0    31   ~ 6
(1%)
Text Notes 7640 5485 2    31   ~ 6
(1%)
Text Notes 5635 5440 0    31   ~ 6
(5%)
Text Notes 5635 5340 0    31   ~ 6
(5%)
Text Notes 5645 5040 0    31   ~ 6
(5%)
Text Notes 5650 5140 0    31   ~ 6
(5%)
Text Notes 5620 5640 0    31   ~ 6
(5%)
$Comp
L Device:C C41
U 1 1 612D1F65
P 5575 6300
F 0 "C41" H 5305 6305 50  0000 L CNN
F 1 "10nF" H 5240 6230 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 5613 6150 50  0001 C CNN
F 3 "~" H 5575 6300 50  0001 C CNN
	1    5575 6300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5705 6150 5705 6250
Wire Wire Line
	5705 6250 5805 6250
Wire Wire Line
	5705 6450 5705 6350
Wire Wire Line
	5705 6350 5805 6350
$Comp
L power:GND #PWR054
U 1 1 612D1F73
P 7175 6850
F 0 "#PWR054" H 7175 6600 50  0001 C CNN
F 1 "GND" H 7040 6765 50  0000 C CNN
F 2 "" H 7175 6850 50  0001 C CNN
F 3 "" H 7175 6850 50  0001 C CNN
	1    7175 6850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7005 6750 7105 6750
Wire Wire Line
	7105 6750 7105 6850
Wire Wire Line
	7105 6650 7105 6750
Connection ~ 7105 6750
NoConn ~ 7005 6450
NoConn ~ 5805 5550
$Comp
L Device:R_POT_US RV2
U 1 1 612D1F83
P 8055 5950
F 0 "RV2" H 8010 5905 50  0000 R CNN
F 1 "10k" H 8010 5995 50  0000 R CNN
F 2 "SM-43TW103:SM43TW103" H 8055 5950 50  0001 C CNN
F 3 "~" H 8055 5950 50  0001 C CNN
	1    8055 5950
	-1   0    0    1   
$EndComp
Wire Wire Line
	7005 6050 7155 6050
Wire Wire Line
	7155 5950 7155 6050
Text GLabel 8055 5600 1    50   Input ~ 0
DRV3V3-S
Wire Wire Line
	8055 5600 8055 5800
Text GLabel 7355 6100 1    50   Output ~ 0
DRV3V3-S
Wire Wire Line
	7005 6250 7145 6250
$Comp
L Device:C C42
U 1 1 612D1F93
P 7355 6450
F 0 "C42" H 7075 6530 50  0000 L CNN
F 1 "0.47uF" H 7075 6365 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 7393 6300 50  0001 C CNN
F 3 "~" H 7355 6450 50  0001 C CNN
	1    7355 6450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R69
U 1 1 612D1F9D
P 7555 6200
F 0 "R69" V 7470 6120 50  0000 L CNN
F 1 "0.1" V 7555 6140 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 7485 6200 50  0001 C CNN
F 3 "~" H 7555 6200 50  0001 C CNN
	1    7555 6200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R70
U 1 1 612D1FA7
P 7655 5600
F 0 "R70" V 7570 5520 50  0000 L CNN
F 1 "0.1" V 7655 5540 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 7585 5600 50  0001 C CNN
F 3 "~" H 7655 5600 50  0001 C CNN
	1    7655 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7655 5250 7655 5450
Wire Wire Line
	7005 5750 7450 5750
Wire Wire Line
	7555 5750 7555 6050
Wire Wire Line
	7005 5950 7155 5950
Wire Wire Line
	7155 5950 7740 5950
Connection ~ 7155 5950
Wire Wire Line
	8055 6100 8055 6650
$Comp
L Connector:Conn_01x04_Male J24
U 1 1 612D1FB8
P 7500 4725
F 0 "J24" H 7490 4475 50  0000 L CNN
F 1 "Conn_01x04_Male" H 7375 4930 50  0000 L CNN
F 2 "Connector_Phoenix_MSTB:PhoenixContact_MSTBVA_2,5_4-G-5,08_1x04_P5.08mm_Vertical" H 7500 4725 50  0001 C CNN
F 3 "~" H 7500 4725 50  0001 C CNN
	1    7500 4725
	1    0    0    1   
$EndComp
$Comp
L Device:C C40
U 1 1 612D1FC2
P 5490 7000
F 0 "C40" H 5490 7075 50  0000 L CNN
F 1 "0.1uF" H 5500 6920 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 5528 6850 50  0001 C CNN
F 3 "~" H 5490 7000 50  0001 C CNN
	1    5490 7000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C38
U 1 1 612D1FCC
P 5075 7065
F 0 "C38" H 5080 7140 50  0000 L CNN
F 1 "0.1uF" H 5080 6985 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 5113 6915 50  0001 C CNN
F 3 "~" H 5075 7065 50  0001 C CNN
	1    5075 7065
	1    0    0    -1  
$EndComp
NoConn ~ 5805 4950
Text GLabel 5190 5050 0    50   Input ~ 0
IO12
Text GLabel 5190 5150 0    50   Input ~ 0
IO2
$Comp
L Device:R R67
U 1 1 612D1FD9
P 5345 5450
F 0 "R67" V 5340 5395 50  0000 L CNN
F 1 "1.5k" V 5375 5160 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 5275 5450 50  0001 C CNN
F 3 "~" H 5345 5450 50  0001 C CNN
	1    5345 5450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5505 5650 5805 5650
$Comp
L Device:R R68
U 1 1 612D1FE4
P 5355 5650
F 0 "R68" V 5350 5595 50  0000 L CNN
F 1 "10k" V 5385 5385 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 5285 5650 50  0001 C CNN
F 3 "~" H 5355 5650 50  0001 C CNN
	1    5355 5650
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR055
U 1 1 612D1FEE
P 7355 6800
F 0 "#PWR055" H 7355 6550 50  0001 C CNN
F 1 "GND" H 7500 6725 50  0000 C CNN
F 2 "" H 7355 6800 50  0001 C CNN
F 3 "" H 7355 6800 50  0001 C CNN
	1    7355 6800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7005 6650 7105 6650
Wire Wire Line
	7105 6650 8055 6650
Connection ~ 7105 6650
Wire Wire Line
	7355 6600 7355 6750
Wire Wire Line
	7555 6350 7555 6750
Connection ~ 7355 6750
Wire Wire Line
	7355 6750 7355 6800
Wire Wire Line
	7655 5750 7655 6750
Wire Wire Line
	7555 6750 7355 6750
Wire Wire Line
	7655 6750 7555 6750
Connection ~ 7555 6750
$Comp
L power:GND #PWR048
U 1 1 612D2003
P 4855 6200
F 0 "#PWR048" H 4855 5950 50  0001 C CNN
F 1 "GND" H 4730 6130 50  0000 C CNN
F 2 "" H 4855 6200 50  0001 C CNN
F 3 "" H 4855 6200 50  0001 C CNN
	1    4855 6200
	-1   0    0    1   
$EndComp
$Comp
L Device:CP C37
U 1 1 612D200D
P 4855 6350
F 0 "C37" H 4675 6365 50  0000 C CNN
F 1 "100uF" H 4725 6255 50  0000 C CNN
F 2 "EEE-FTH101XAL:CAPAE660X800N" H 4893 6200 50  0001 C CNN
F 3 "~" H 4855 6350 50  0001 C CNN
	1    4855 6350
	-1   0    0    1   
$EndComp
Text GLabel 4855 6900 3    50   Input ~ 0
12VMOT-S
Wire Wire Line
	7005 5250 7655 5250
Text Notes 5740 4540 0    100  ~ 20
SIDE Stepper driver
$Comp
L Transistor_FET:AO3401A Q8
U 1 1 612D201A
P 8445 5485
F 0 "Q8" H 8610 5580 50  0000 L CNN
F 1 "DMP2160U" H 8000 5575 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 8645 5410 50  0001 L CIN
F 3 "http://www.aosmd.com/pdfs/datasheet/AO3401A.pdf" H 8445 5485 50  0001 L CNN
	1    8445 5485
	-1   0    0    -1  
$EndComp
Text GLabel 8345 5020 1    50   Output ~ 0
12VMOT-S
$Comp
L Transistor_FET:2N7002K Q9
U 1 1 612D2025
P 9300 6130
F 0 "Q9" V 9335 6255 50  0000 L CNN
F 1 "NTR4501" V 9525 5960 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 9500 6055 50  0001 L CIN
F 3 "https://www.diodes.com/assets/Datasheets/ds30896.pdf" H 9300 6130 50  0001 L CNN
	1    9300 6130
	0    -1   -1   0   
$EndComp
Text GLabel 9410 6470 2    50   Input ~ 0
IO19
$Comp
L Device:R R73
U 1 1 612D2030
P 9300 6755
F 0 "R73" H 9180 6720 50  0000 C CNN
F 1 "10k" V 9295 6750 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 9230 6755 50  0001 C CNN
F 3 "~" H 9300 6755 50  0001 C CNN
	1    9300 6755
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR059
U 1 1 612D203A
P 9660 7030
F 0 "#PWR059" H 9660 6780 50  0001 C CNN
F 1 "GND" H 9655 6875 50  0000 C CNN
F 2 "" H 9660 7030 50  0001 C CNN
F 3 "" H 9660 7030 50  0001 C CNN
	1    9660 7030
	1    0    0    -1  
$EndComp
Wire Wire Line
	9300 6935 9300 6905
Wire Wire Line
	9500 6030 9660 6030
$Comp
L Device:R R74
U 1 1 612D2046
P 9660 6285
F 0 "R74" H 9780 6240 50  0000 C CNN
F 1 "120" V 9655 6285 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 9590 6285 50  0001 C CNN
F 3 "~" H 9660 6285 50  0001 C CNN
	1    9660 6285
	-1   0    0    1   
$EndComp
Wire Wire Line
	9660 6030 9660 6135
Wire Wire Line
	9660 6435 9660 6935
$Comp
L Device:R R71
U 1 1 612D2052
P 8595 5775
F 0 "R71" V 8515 5780 50  0000 C CNN
F 1 "120" V 8595 5780 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 8525 5775 50  0001 C CNN
F 3 "~" H 8595 5775 50  0001 C CNN
	1    8595 5775
	0    1    1    0   
$EndComp
Wire Wire Line
	8445 5775 8345 5775
Wire Wire Line
	8345 5775 8345 5685
$Comp
L Device:C C43
U 1 1 612D205E
P 8495 6330
F 0 "C43" V 8530 6140 50  0000 L CNN
F 1 "0.1uF" V 8365 6225 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 8533 6180 50  0001 C CNN
F 3 "~" H 8495 6330 50  0001 C CNN
	1    8495 6330
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR057
U 1 1 612D2068
P 8645 6330
F 0 "#PWR057" H 8645 6080 50  0001 C CNN
F 1 "GND" H 8645 6180 50  0000 C CNN
F 2 "" H 8645 6330 50  0001 C CNN
F 3 "" H 8645 6330 50  0001 C CNN
	1    8645 6330
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8345 6330 8345 6460
Connection ~ 8345 6330
Wire Wire Line
	8345 6330 8345 6030
Wire Wire Line
	8435 6030 8345 6030
Connection ~ 8345 5775
Connection ~ 8345 6030
Wire Wire Line
	8345 6030 8345 5775
Wire Wire Line
	8345 5285 8345 5230
$Comp
L Device:R R72
U 1 1 612D207A
P 8675 5230
F 0 "R72" V 8750 5155 50  0000 L CNN
F 1 "47k" V 8670 5160 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 8605 5230 50  0001 C CNN
F 3 "~" H 8675 5230 50  0001 C CNN
	1    8675 5230
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8345 5230 8525 5230
$Comp
L power:GND #PWR058
U 1 1 612D2085
P 8825 5230
F 0 "#PWR058" H 8825 4980 50  0001 C CNN
F 1 "GND" H 8830 5057 50  0000 C CNN
F 2 "" H 8825 5230 50  0001 C CNN
F 3 "" H 8825 5230 50  0001 C CNN
	1    8825 5230
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9300 6605 9300 6470
Wire Wire Line
	9410 6470 9300 6470
Connection ~ 9300 6470
Wire Wire Line
	9300 6470 9300 6330
$Comp
L power:+12V #PWR056
U 1 1 612D2093
P 8345 6460
F 0 "#PWR056" H 8345 6310 50  0001 C CNN
F 1 "+12V" H 8360 6633 50  0000 C CNN
F 2 "" H 8345 6460 50  0001 C CNN
F 3 "" H 8345 6460 50  0001 C CNN
	1    8345 6460
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG01
U 1 1 612D209D
P 4930 6550
F 0 "#FLG01" H 4930 6625 50  0001 C CNN
F 1 "PWR_FLAG" H 4925 6685 50  0000 C CNN
F 2 "" H 4930 6550 50  0001 C CNN
F 3 "~" H 4930 6550 50  0001 C CNN
	1    4930 6550
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG02
U 1 1 612D20A7
P 7145 6250
F 0 "#FLG02" H 7145 6325 50  0001 C CNN
F 1 "PWR_FLAG" H 7125 6390 50  0000 C CNN
F 2 "" H 7145 6250 50  0001 C CNN
F 3 "~" H 7145 6250 50  0001 C CNN
	1    7145 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7355 6250 7355 6100
Wire Wire Line
	7355 6300 7355 6250
Connection ~ 7355 6250
Connection ~ 7145 6250
Wire Wire Line
	7145 6250 7355 6250
Text GLabel 7005 4950 2    50   BiDi ~ 0
A1-S
Text GLabel 7005 5050 2    50   BiDi ~ 0
A2-S
Text GLabel 7005 5450 2    50   BiDi ~ 0
B1-S
Text GLabel 7005 5550 2    50   BiDi ~ 0
B2-S
Text GLabel 7700 4825 2    50   BiDi ~ 0
A1-S
Text GLabel 7700 4725 2    50   BiDi ~ 0
A2-S
Text GLabel 7700 4625 2    50   BiDi ~ 0
B1-S
Text GLabel 7700 4525 2    50   BiDi ~ 0
B2-S
$Comp
L power:GND #PWR053
U 1 1 612D20BE
P 5490 7150
F 0 "#PWR053" H 5490 6900 50  0001 C CNN
F 1 "GND" H 5495 7000 50  0000 C CNN
F 2 "" H 5490 7150 50  0001 C CNN
F 3 "" H 5490 7150 50  0001 C CNN
	1    5490 7150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR049
U 1 1 612D20C8
P 5075 7215
F 0 "#PWR049" H 5075 6965 50  0001 C CNN
F 1 "GND" H 5080 7065 50  0000 C CNN
F 2 "" H 5075 7215 50  0001 C CNN
F 3 "" H 5075 7215 50  0001 C CNN
	1    5075 7215
	1    0    0    -1  
$EndComp
$Comp
L Device:R R64
U 1 1 612D20D2
P 5340 5050
F 0 "R64" V 5340 5050 50  0000 C CNN
F 1 "1.2k" V 5305 5265 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 5270 5050 50  0001 C CNN
F 3 "~" H 5340 5050 50  0001 C CNN
	1    5340 5050
	0    1    1    0   
$EndComp
$Comp
L Device:R R65
U 1 1 612D20DC
P 5340 5150
F 0 "R65" V 5340 5155 50  0000 C CNN
F 1 "1.2k" V 5305 5365 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 5270 5150 50  0001 C CNN
F 3 "~" H 5340 5150 50  0001 C CNN
	1    5340 5150
	0    1    1    0   
$EndComp
Wire Wire Line
	4255 7325 4255 7225
$Comp
L power:GND #PWR045
U 1 1 612D20E7
P 4255 7325
F 0 "#PWR045" H 4255 7075 50  0001 C CNN
F 1 "GND" H 4255 7180 50  0000 C CNN
F 2 "" H 4255 7325 50  0001 C CNN
F 3 "" H 4255 7325 50  0001 C CNN
	1    4255 7325
	-1   0    0    -1  
$EndComp
$Comp
L Device:LED D10
U 1 1 612D20F1
P 4255 7075
F 0 "D10" V 4255 6975 50  0000 C CNN
F 1 "LED" V 4230 7240 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 4255 7075 50  0001 C CNN
F 3 "~" H 4255 7075 50  0001 C CNN
	1    4255 7075
	0    1    -1   0   
$EndComp
$Comp
L Device:R R60
U 1 1 612D20FB
P 4255 6775
F 0 "R60" H 4155 6790 50  0000 C CNN
F 1 "750" V 4255 6780 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 4185 6775 50  0001 C CNN
F 3 "~" H 4255 6775 50  0001 C CNN
	1    4255 6775
	1    0    0    1   
$EndComp
$Comp
L Switch:SW_DIP_x01 SW7
U 1 1 612D2105
P 4555 6620
F 0 "SW7" H 4460 6475 50  0000 L CNN
F 1 "SW_DIP_x01" H 4335 6770 50  0000 L CNN
F 2 "Button_Switch_SMD:SW_DIP_SPSTx01_Slide_Omron_A6S-110x_W8.9mm_P2.54mm" H 4555 6620 50  0001 C CNN
F 3 "~" H 4555 6620 50  0001 C CNN
	1    4555 6620
	-1   0    0    -1  
$EndComp
Text Notes 4535 7060 2    50   ~ 10
Green
Wire Wire Line
	4255 6625 4255 6620
Wire Wire Line
	7005 6850 7105 6850
Wire Wire Line
	7105 6850 7175 6850
Connection ~ 7105 6850
Wire Wire Line
	9300 6935 9660 6935
Connection ~ 9660 6935
Wire Wire Line
	9660 6935 9660 7030
Connection ~ 8345 5230
Wire Wire Line
	8345 5020 8345 5230
Text Notes 5305 6255 0    50   ~ 10
50V
Wire Wire Line
	5805 6850 5490 6850
Wire Wire Line
	5000 6850 5000 6550
Wire Wire Line
	5000 6550 5075 6550
Connection ~ 5490 6850
Wire Wire Line
	5490 6850 5000 6850
Wire Wire Line
	5075 6550 5075 6750
Wire Wire Line
	5805 6750 5395 6750
$Comp
L Device:R R63
U 1 1 612D2121
P 5225 6600
F 0 "R63" H 5115 6745 50  0000 L CNN
F 1 "1M" V 5220 6540 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 5155 6600 50  0001 C CNN
F 3 "~" H 5225 6600 50  0001 C CNN
	1    5225 6600
	1    0    0    -1  
$EndComp
$Comp
L Device:C C39
U 1 1 612D212B
P 5395 6600
F 0 "C39" H 5295 6685 50  0000 L CNN
F 1 "0.1uF" H 5425 6520 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 5433 6450 50  0001 C CNN
F 3 "~" H 5395 6600 50  0001 C CNN
	1    5395 6600
	1    0    0    -1  
$EndComp
Connection ~ 5395 6750
Wire Wire Line
	5395 6750 5225 6750
Wire Wire Line
	5575 6150 5705 6150
Wire Wire Line
	5575 6450 5705 6450
Wire Wire Line
	5515 6450 5515 6550
Wire Wire Line
	5515 6550 5805 6550
Wire Wire Line
	5225 6450 5395 6450
Wire Wire Line
	5395 6450 5515 6450
Connection ~ 5395 6450
Connection ~ 4855 6620
Wire Wire Line
	4855 6620 4855 6840
Connection ~ 5225 6750
Wire Wire Line
	5075 6750 5225 6750
Connection ~ 5075 6750
Wire Wire Line
	5075 6750 5075 6915
Wire Wire Line
	4855 6500 4855 6550
Wire Wire Line
	4855 6550 4855 6620
Connection ~ 4855 6550
Wire Wire Line
	4855 6550 4930 6550
Connection ~ 5000 6550
Connection ~ 4930 6550
Wire Wire Line
	4930 6550 5000 6550
Text Notes 5490 6635 0    50   ~ 10
16V
Text Notes 7075 6490 0    50   ~ 10
6.3V
Text Notes 4685 6295 0    50   ~ 10
50V
Text Notes 5495 7195 0    50   ~ 10
16V
Text Notes 5080 7260 0    50   ~ 10
16V
$Comp
L Connector:TestPoint TP26
U 1 1 612D2150
P 7740 5950
F 0 "TP26" H 7715 6020 50  0000 R CNN
F 1 "TestPoint" H 7815 6150 50  0000 R CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 7940 5950 50  0001 C CNN
F 3 "~" H 7940 5950 50  0001 C CNN
	1    7740 5950
	-1   0    0    1   
$EndComp
Connection ~ 7740 5950
Wire Wire Line
	7740 5950 7905 5950
$Comp
L Connector:TestPoint TP27
U 1 1 612D215C
P 7755 6355
F 0 "TP27" H 7730 6425 50  0000 R CNN
F 1 "TestPoint" H 7830 6555 50  0000 R CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 7955 6355 50  0001 C CNN
F 3 "~" H 7955 6355 50  0001 C CNN
	1    7755 6355
	-1   0    0    1   
$EndComp
Wire Wire Line
	7355 6250 7415 6250
Wire Wire Line
	7415 6250 7415 6355
Wire Wire Line
	7415 6355 7755 6355
$Comp
L Connector:TestPoint TP24
U 1 1 612D2169
P 7450 5750
F 0 "TP24" H 7530 5945 50  0000 R CNN
F 1 "TestPoint" H 7605 5985 50  0000 R CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 7650 5750 50  0001 C CNN
F 3 "~" H 7650 5750 50  0001 C CNN
	1    7450 5750
	1    0    0    -1  
$EndComp
Connection ~ 7450 5750
Wire Wire Line
	7450 5750 7555 5750
$Comp
L Connector:TestPoint TP25
U 1 1 612D2175
P 7655 5250
F 0 "TP25" H 7630 5320 50  0000 R CNN
F 1 "TestPoint" H 7730 5450 50  0000 R CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 7855 5250 50  0001 C CNN
F 3 "~" H 7855 5250 50  0001 C CNN
	1    7655 5250
	1    0    0    -1  
$EndComp
Connection ~ 7655 5250
$Comp
L Connector:TestPoint TP23
U 1 1 612D2180
P 4855 6840
F 0 "TP23" V 4770 7095 50  0000 R CNN
F 1 "TestPoint" H 4885 7065 50  0000 R CNN
F 2 "TestPoint:TestPoint_Pad_D1.5mm" H 5055 6840 50  0001 C CNN
F 3 "~" H 5055 6840 50  0001 C CNN
	1    4855 6840
	0    -1   -1   0   
$EndComp
Connection ~ 4855 6840
Wire Wire Line
	4855 6840 4855 6900
$Comp
L power:+3.3V #PWR050
U 1 1 612D218C
P 5195 5350
F 0 "#PWR050" H 5195 5200 50  0001 C CNN
F 1 "+3.3V" V 5210 5478 50  0000 L CNN
F 2 "" H 5195 5350 50  0001 C CNN
F 3 "" H 5195 5350 50  0001 C CNN
	1    5195 5350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5495 5350 5805 5350
$Comp
L power:+3.3V #PWR051
U 1 1 612D2197
P 5195 5450
F 0 "#PWR051" H 5195 5300 50  0001 C CNN
F 1 "+3.3V" V 5200 5580 50  0000 L CNN
F 2 "" H 5195 5450 50  0001 C CNN
F 3 "" H 5195 5450 50  0001 C CNN
	1    5195 5450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5805 5450 5495 5450
$Comp
L Device:R R66
U 1 1 612D21A2
P 5345 5350
F 0 "R66" V 5345 5300 50  0000 L CNN
F 1 "1.5k" V 5310 5470 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 5275 5350 50  0001 C CNN
F 3 "~" H 5345 5350 50  0001 C CNN
	1    5345 5350
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR052
U 1 1 612D21AC
P 5205 5650
F 0 "#PWR052" H 5205 5500 50  0001 C CNN
F 1 "+3.3V" V 5210 5780 50  0000 L CNN
F 2 "" H 5205 5650 50  0001 C CNN
F 3 "" H 5205 5650 50  0001 C CNN
	1    5205 5650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5805 5050 5490 5050
Wire Wire Line
	5805 5150 5490 5150
$Comp
L LucidSens_2.01v-rescue:DRV8825PWP-DRV8825PWPR-MinimalSens_0.04v-rescue-LucidSens_1.05v-rescue U10
U 1 1 612D21D7
P 5705 4950
F 0 "U10" H 6405 5215 50  0000 C CNN
F 1 "DRV8825PWP" H 6405 5124 50  0000 C CNN
F 2 "DRV8825PWPR:Texas_Instruments-DRV8825PWP-Level_A" H 5705 5350 50  0001 L CNN
F 3 "http://www.ti.com/general/docs/lit/getliterature.tsp?genericPartNumber=DRV8825&fileType=pdf" H 5705 5450 50  0001 L CNN
F 4 "Radial" H 5705 5550 50  0001 L CNN "Case Package"
F 5 "Manufacturer URL" H 5705 5650 50  0001 L CNN "Component Link 1 Description"
F 6 "http://www.ti.com/" H 5705 5750 50  0001 L CNN "Component Link 1 URL"
F 7 "SLVSA73F, 07/2014" H 5705 5850 50  0001 L CNN "Datasheet Version"
F 8 "8 mm" H 5705 5950 50  0001 L CNN "Diameter"
F 9 "11.5 mm" H 5705 6050 50  0001 L CNN "Height"
F 10 "3.5 mm" H 5705 6150 50  0001 L CNN "Lead Pitch"
F 11 "11.5 mm" H 5705 6250 50  0001 L CNN "Length"
F 12 "85 degC" H 5705 6350 50  0001 L CNN "Max Operating Temperature"
F 13 "-40 degC" H 5705 6450 50  0001 L CNN "Min Operating Temperature"
F 14 "Through Hole" H 5705 6550 50  0001 L CNN "Mount"
F 15 "28-Pin Small Outline Package Integrated Circuit, Body 9.7 x 4.4 mm, Pitch 0.65 mm" H 5705 6650 50  0001 L CNN "Package Description"
F 16 "200" H 5705 6750 50  0001 L CNN "Package Quantity"
F 17 "Bulk" H 5705 6850 50  0001 L CNN "Packaging"
F 18 "2" H 5705 6950 50  0001 L CNN "Pins"
F 19 "No SVHC" H 5705 7050 50  0001 L CNN "REACH SVHC"
F 20 "No" H 5705 7150 50  0001 L CNN "Radiation Hardening"
F 21 "true" H 5705 7250 50  0001 L CNN "Ro HSCompliant"
F 22 "20%" H 5705 7350 50  0001 L CNN "Tolerance"
F 23 "100 V" H 5705 7450 50  0001 L CNN "Voltage Rating"
F 24 "100 V" H 5705 7550 50  0001 L CNN "Voltage Rating DC"
F 25 "8 mm" H 5705 7650 50  0001 L CNN "Width"
F 26 "47 uF" H 5705 7750 50  0001 L CNN "capacitance"
F 27 "IC" H 5705 7850 50  0001 L CNN "category"
F 28 "972973" H 5705 7950 50  0001 L CNN "ciiva ids"
F 29 "8657dfec07a272e1" H 5705 8050 50  0001 L CNN "library id"
F 30 "Texas Instruments" H 5705 8150 50  0001 L CNN "manufacturer"
F 31 "PWP0028C" H 5705 8250 50  0001 L CNN "package"
F 32 "1475067547" H 5705 8350 50  0001 L CNN "release date"
F 33 "11C84FA6-0B99-4DFE-81B3-04F3FFCCD07D" H 5705 8450 50  0001 L CNN "vault revision"
F 34 "yes" H 5705 8550 50  0001 L CNN "imported"
	1    5705 4950
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x03_Male J22
U 1 1 612D21E1
P 3985 5250
F 0 "J22" H 4105 5470 50  0000 C CNN
F 1 "Conn_01x03_Male" V 3925 5355 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 3985 5250 50  0001 C CNN
F 3 "~" H 3985 5250 50  0001 C CNN
	1    3985 5250
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR046
U 1 1 612D21EB
P 4485 5150
F 0 "#PWR046" H 4485 5000 50  0001 C CNN
F 1 "+3.3V" V 4480 5255 50  0000 L CNN
F 2 "" H 4485 5150 50  0001 C CNN
F 3 "" H 4485 5150 50  0001 C CNN
	1    4485 5150
	0    1    1    0   
$EndComp
$Comp
L Device:R R61
U 1 1 612D21F5
P 4335 5150
F 0 "R61" V 4405 5215 50  0000 C CNN
F 1 "1.2k" V 4330 5155 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 4265 5150 50  0001 C CNN
F 3 "~" H 4335 5150 50  0001 C CNN
	1    4335 5150
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR043
U 1 1 612D21FF
P 4185 5350
F 0 "#PWR043" H 4185 5100 50  0001 C CNN
F 1 "GND" V 4190 5222 50  0000 R CNN
F 2 "" H 4185 5350 50  0001 C CNN
F 3 "" H 4185 5350 50  0001 C CNN
	1    4185 5350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5805 5250 4185 5250
Text Notes 4325 5095 0    31   ~ 6
(5%)
$Comp
L Connector:Conn_01x03_Male J23
U 1 1 612D220B
P 3990 5950
F 0 "J23" H 4110 6170 50  0000 C CNN
F 1 "Conn_01x03_Male" V 3940 5960 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 3990 5950 50  0001 C CNN
F 3 "~" H 3990 5950 50  0001 C CNN
	1    3990 5950
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR047
U 1 1 612D2215
P 4490 5850
F 0 "#PWR047" H 4490 5700 50  0001 C CNN
F 1 "+3.3V" V 4485 5955 50  0000 L CNN
F 2 "" H 4490 5850 50  0001 C CNN
F 3 "" H 4490 5850 50  0001 C CNN
	1    4490 5850
	0    1    1    0   
$EndComp
$Comp
L Device:R R62
U 1 1 612D221F
P 4340 5850
F 0 "R62" V 4410 5915 50  0000 C CNN
F 1 "1.2k" V 4335 5855 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 4270 5850 50  0001 C CNN
F 3 "~" H 4340 5850 50  0001 C CNN
	1    4340 5850
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR044
U 1 1 612D2229
P 4190 6050
F 0 "#PWR044" H 4190 5800 50  0001 C CNN
F 1 "GND" V 4195 5922 50  0000 R CNN
F 2 "" H 4190 6050 50  0001 C CNN
F 3 "" H 4190 6050 50  0001 C CNN
	1    4190 6050
	0    -1   -1   0   
$EndComp
Text Notes 4330 5795 0    31   ~ 6
(5%)
Wire Wire Line
	5805 5950 4190 5950
$Comp
L Connector:Conn_01x03_Male J20
U 1 1 612D2235
P 3430 5610
F 0 "J20" H 3550 5830 50  0000 C CNN
F 1 "Conn_01x03_Male" V 3365 5670 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 3430 5610 50  0001 C CNN
F 3 "~" H 3430 5610 50  0001 C CNN
	1    3430 5610
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR041
U 1 1 612D223F
P 3930 5510
F 0 "#PWR041" H 3930 5360 50  0001 C CNN
F 1 "+3.3V" V 3925 5615 50  0000 L CNN
F 2 "" H 3930 5510 50  0001 C CNN
F 3 "" H 3930 5510 50  0001 C CNN
	1    3930 5510
	0    1    1    0   
$EndComp
$Comp
L Device:R R58
U 1 1 612D2249
P 3780 5510
F 0 "R58" V 3850 5575 50  0000 C CNN
F 1 "1.2k" V 3775 5515 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 3710 5510 50  0001 C CNN
F 3 "~" H 3780 5510 50  0001 C CNN
	1    3780 5510
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR039
U 1 1 612D2253
P 3630 5710
F 0 "#PWR039" H 3630 5460 50  0001 C CNN
F 1 "GND" V 3635 5582 50  0000 R CNN
F 2 "" H 3630 5710 50  0001 C CNN
F 3 "" H 3630 5710 50  0001 C CNN
	1    3630 5710
	0    -1   -1   0   
$EndComp
Text Notes 3770 5455 0    31   ~ 6
(5%)
$Comp
L Connector:Conn_01x03_Male J21
U 1 1 612D225E
P 3440 6380
F 0 "J21" H 3560 6600 50  0000 C CNN
F 1 "Conn_01x03_Male" V 3380 6395 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 3440 6380 50  0001 C CNN
F 3 "~" H 3440 6380 50  0001 C CNN
	1    3440 6380
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR042
U 1 1 612D2268
P 3940 6280
F 0 "#PWR042" H 3940 6130 50  0001 C CNN
F 1 "+3.3V" V 3935 6385 50  0000 L CNN
F 2 "" H 3940 6280 50  0001 C CNN
F 3 "" H 3940 6280 50  0001 C CNN
	1    3940 6280
	0    1    1    0   
$EndComp
$Comp
L Device:R R59
U 1 1 612D2272
P 3790 6280
F 0 "R59" V 3860 6345 50  0000 C CNN
F 1 "1.2k" V 3785 6285 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 3720 6280 50  0001 C CNN
F 3 "~" H 3790 6280 50  0001 C CNN
	1    3790 6280
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR040
U 1 1 612D227C
P 3640 6480
F 0 "#PWR040" H 3640 6230 50  0001 C CNN
F 1 "GND" V 3645 6352 50  0000 R CNN
F 2 "" H 3640 6480 50  0001 C CNN
F 3 "" H 3640 6480 50  0001 C CNN
	1    3640 6480
	0    -1   -1   0   
$EndComp
Text Notes 3780 6225 0    31   ~ 6
(5%)
Wire Wire Line
	3630 5610 4845 5610
Wire Wire Line
	4845 5610 4845 5850
Wire Wire Line
	4845 5850 5805 5850
Wire Wire Line
	3640 6380 4640 6380
Wire Wire Line
	4640 6380 4640 6050
Wire Wire Line
	4640 6050 5805 6050
Text GLabel 2325 2900 0    50   Input ~ 0
IO12
Text Notes 1365 2935 0    50   ~ 10
STP_SIDE_STEPPER
Text GLabel 4075 2400 2    50   BiDi ~ 0
IO19
Text Notes 4355 2430 0    50   ~ 10
PWR_SIDE_STEPPER
Text Notes 4340 3140 0    50   ~ 10
DIR_SIDE_STEPPER
Text GLabel 4075 3100 2    50   Output ~ 0
IO2
$Comp
L Connector:Conn_01x02_Male J25
U 1 1 61299E1A
P 715 2620
F 0 "J25" H 910 2805 50  0000 C CNN
F 1 "Conn_01x02_Male" H 885 2730 50  0000 C CNN
F 2 "Connector_JST:JST_EH_B2B-EH-A_1x02_P2.50mm_Vertical" H 715 2620 50  0001 C CNN
F 3 "~" H 715 2620 50  0001 C CNN
	1    715  2620
	1    0    0    -1  
$EndComp
Text GLabel 915  2620 2    50   Output ~ 0
IO26
$Comp
L power:GND #PWR0155
U 1 1 6129B348
P 915 2720
F 0 "#PWR0155" H 915 2470 50  0001 C CNN
F 1 "GND" V 915 2530 50  0000 C CNN
F 2 "" H 915 2720 50  0001 C CNN
F 3 "" H 915 2720 50  0001 C CNN
	1    915  2720
	0    -1   -1   0   
$EndComp
Text Notes 12770 10285 0    100  ~ 20
ELECTROCHEMILUMINESCENCE WING
$EndSCHEMATC

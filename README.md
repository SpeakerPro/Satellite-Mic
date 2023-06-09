# Good Way - DUD9027G1 Project

## 聯想 Speak Pro main board MCU - M258SD3AE

- NuMicro_ICP_Programming_Tool_V3.08.7313r.zip : 
	透過 ICE (Nu-Link2-Me) 對 MCU 進行燒錄, 可燒錄 LDROM , APROM , config 等等
- NuMicro_ISP_Programming_Tool_V4.07.zip : 
	在 MCU LDROM 燒錄 ISP code 後, 可透過此 tool 對 MCU 更新 APROM
- NuTool-PinConfigure_V1.23.0014.zip :
	MCU pin config tool , 可以了解 MCU 每根 pin 的作用

==============================================================================================================================================
# M251/M252/M254/M256/M258 Series CMSIS BSP

This BSP folder


## .\Document\

- CMSIS.html<br>
	Document of CMSIS version 5.1.1.

- NuMicro M251_252_254_256_258 Series CMSIS BSP Driver Reference Guide.chm<br>
	This document describes the usage of drivers in M251/M252/M254/M256/M258 BSP.

- NuMicro M251_252_254_256_258 Series CMSIS BSP Revision History.pdf<br>
	This document shows the revision history of M251/M252/M254/M256/M258 BSP.


## .\Library\

- CMSIS<br>
	Cortex® Microcontroller Software Interface Standard (CMSIS) V5.1.1 definitions by Arm® Corp.

- Device<br>
	CMSIS compliant device header file.

- LCDLib<br>
	Library for controlling LCD module.

- SmartcardLib<br>
	Smart card library binary and header file.

- StdDriver<br>
	All peripheral driver header and source files.

- TKLib<br>
	Library for controlling touch key module.


## .\Sample Code\

- CardReader<br>
	USB CCID smart card reader sample code.

- CortexM23<br>
	Cortex®-M23 sample code.

- Hard\_Fault\_Sample<br>
	Show hard fault information when hard fault happened.<p>
	The hard fault handler shows some information including program counter, which is the address where the processor is executed when the hard fault occurs. The listing file (or map file) can show what function and instruction that is.<p>
	It also shows the Link Register (LR), which contains the return address of the last function call. It can show the status where CPU comes from to get to this point.

- ISP<br>
	Sample codes for In-System-Programming.

- Level1_Training<br>
	Level 1 training sample code.

- NuMaker-M258KE<br>
	Sample codes for NuMaker-M258KE board.

- NuMaker-M258KG<br>
	Sample codes for NuMaker-M258KG board.

- PowerManagement<br>
	Power management sample code.

- Semihost<br>
	Show how to print and get character through IDE console window.

- StdDriver<br>
	Sample code to demonstrate the usage of M251/M252/M254/M256/M258 series MCU peripheral driver APIs.

- Template<br>
	A project template for M251/M252/M254/M256/M258 series MCU.

- XOM<br>
	Demonstrate how to create XOM library and use it.


## .\Tool\

- TK<br>
	Touch key tool.


# License

**SPDX-License-Identifier: Apache-2.0**

Copyright in some of the content available in this BSP belongs to third parties.
Third parties license is specified in a file header or license file.<p>
M251/M252/M254/M256/M258 BSP files are provided under the Apache-2.0 license.

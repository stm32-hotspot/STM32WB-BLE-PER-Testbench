# STM32-Hotspot/STM32WB-BLE-PER-Testbench MCU Firmware Package, based on STM32CubeWB Release v1.13.1

![latest tag](https://img.shields.io/github/v/tag/STMicroelectronics/STM32CubeWB.svg?color=brightgreen)

## Example

This Hotspot FW package includes:
* Application example under "Projects\P-NUCLEO-WB55.Nucleo\Applications\BLE" called BLE_PER_Testbench.     
   * This example aims to demonstrates how to enable BLE Direct Test Mode (DTM) in the STM32WB from a firmware user application using HCI testing commands directly to perform a Packet Errror Rate (PER) measurement without the need of an external software tool like STM32CubeMonitor-RF or physical cable connections to the DUT.  
   * The Transmitter (packet generator) is set up to transmit test packets (i.e. PRBS9) continuously. 
   * The Receiver DUT is set up to calculate PER on an interval expected to have received a set number of packets. We can predetermine that it takes 938 ms for the packet generator (Tx) to transmit 1500 packets of 37bytes, PRBS9, in 1M PHY, therefore, this time can then be applied on the receiver as a receive window and read the number of packets actually received, then compare it against the expected number of packets (1500) for this time window and calculate the PER from that.  The receiver will also measure and report the RSSI level for that time window. 
   * Two NUCLEO-WB55RG boards can be used for the Transmitter and Receiver, however, the firmware can be tailored to run on any other platform based on STM32WB. the firmware .hex image file is provided in this package. 
   * A custom Android Smartphone app called "ST PER Testbench" is used to configure the radio mode and corresponding test parameters for the Transmitter and Reciver devices, respectively. 
   * The PER and RSSI measurement results can be viewed from a PC's serial terminal connected to the NUCLEO-WB55RG/DTM Rx device or from the "ST PER Testbench" Android Smartphone since the DTM-Rx device also advertises its results for 1s after every Rx window. 
   * Development tools, toolchains/compilers: IAR EWARM V9.20.x, STM32CubeIDE v1.8.0
   * Supported Devices and hardware boards: NUCLEO-WB55RG
   * Known limitations: None

## Hardware Needed

  * Two NUCLEO-WB55RG
    * [NUCLEO-WB55RG](https://www.st.com/en/evaluation-tools/nucleo-wb55rg.html)
	
  * Android Smartphone with Bluetooth LE 4.2 support
  
## Software Needed

  * Prebuilt firmware image, BLE_PER_Testbench.hex, provided under "STM32WB-BLE-PER-Testbench\Projects\P-NUCLEO-WB55.Nucleo\Applications\BLE\BLE_PER_Testbench\Binaries"
	
  * Android Smartphone app provided under "STM32WB-BLE-PER-Testbench\Utilities\Android_Software"
  
## User's Guide
1. Install the .apk app on an Android Smarphone 
2. Install the .hex firmware on both NUCLEO-WB55RG boards 
![image](Utilities\Media\Images\Users_Guide\UG_image_0.jpg)



## Troubleshooting

**Caution** : Issues and the pull-requests are **not supported** to submit problems or suggestions related to the software delivered in this repository. The BLE_Basic_DataThroughput_Server example is being delivered as-is, and not necessarily supported by ST.

**For any other question** related to the product, the hardware performance or characteristics, the tools, the environment, you can submit it to the **ST Community** on the STM32 MCUs related [page](https://community.st.com/s/topic/0TO0X000000BSqSWAW/stm32-mcus).

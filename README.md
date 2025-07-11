# RAB5-OSIRE SAID Code Example

<img src="images/said.jpg" style="zoom:90%;" />

**NOTE:**  The current up to 100 mA may flow from the System Basis Chip [U1] TLE9262-3BQXV33 VCC2 LDO Output. In case more current is needed, please use RDK4 BATTERY supply terminals J2 and J4 and switch to RECOM RPX-1.5Q power supply on RAB5-OSIRE board [Switch the jumper from pins 2 and 3 to pins 1 and 2 on the P2 port]. This would increase the current limit to 1.5A.

## Requirements

- [ModusToolbox™ software](https://www.cypress.com/products/modustoolbox-software-environment) v2025.4.0
- [RDK4 Rev2.](https://www.rutronik24.com/product/rutronik/rdk4/20820197.html)

## Supported toolchains (make variable 'TOOLCHAIN')

- GNU Arm&reg; Embedded Compiler v11.3.1 (`GCC_ARM`) - Default value of `TOOLCHAIN`

## Software and hardware on request

For any software or hardware that is listed below please get in touch with solutions@rutronik.com

- The ams OSIRE® E3731i LED stripe with 20 LEDs on it. RUTRONIK Part No.: [LEDATV3196](https://www.rutronik24.com/product/ams_osram/osire_e3731i_ledstripe/22814309.html)
- The source code for the colour correction algorithms.

## Using the code example

Create the project and open it using one of the following:

<details><summary><b>In Eclipse IDE for ModusToolbox&trade; software</b></summary>



1. Click the **New Application** link in the **Quick Panel** (or, use **File** > **New** > **ModusToolbox&trade; Application**). This launches the [Project Creator](https://www.infineon.com/ModusToolboxProjectCreator) tool.

2. Pick a RDK4 kit supported by the code example from the PSoC&trade; 4 BSPs list shown in the **Project Creator - Choose Board Support Package (BSP)** dialogue.

   When you select a supported kit, the example is reconfigured automatically to work with the kit. To work with a different supported kit later, use the [Library Manager](https://www.infineon.com/ModusToolboxLibraryManager) to choose the BSP for the supported kit. You can use the Library Manager to select or update the BSP and firmware libraries used in this application. To access the Library Manager, click the link from the **Quick Panel**.

   You can also just start the application creation process again and select a different kit.

   If you want to use the application for a kit not listed here, you may need to update the source files. If the kit does not have the required resources, the application may not work.

3. In the **Project Creator - Select Application** dialogue, choose the RDK4_RAB5-OSIRE_SAID example in the Peripherals category by enabling the checkbox.

4. (Optional) Change the suggested **New Application Name**.

5. The **Application(s) Root Path** defaults to the Eclipse workspace which is usually the desired location for the application. If you want to store the application in a different location, you can change the *Application(s) Root Path* value. Applications that share libraries should be in the same root path.

6. Click **Create** to complete the application creation process.

For more details, see the [Eclipse IDE for ModusToolbox&trade; software user guide](https://www.infineon.com/MTBEclipseIDEUserGuide) (locally available at *{ModusToolbox&trade; software install directory}/docs_{version}/mt_ide_user_guide.pdf*).

</details>

### Operation

The firmware example uses KitProg3 UART for debugging information output. The project environment is prepared for developing and testing the RAB5-OSIRE AS1163 SAID.

The RDK4 Arduino I2C is configured as a slave device and responds to the SAID I2C master with test data.

After all SAID tests are complete, the E5515 side-looker LEDs LED3 and LED4 will start blinking.

The RDK4 System Basis Chip TLE9262-3BQXV33 is kept online in a main loop by periodically feeding the internal watchdog timer. This assures that the VCC2 LDO (+5V 100mA) power supply stays on.

### Debugging

If you successfully imported the example, the debug configurations are already prepared to use with the onboard KitProg3 debugger. Open the ModusToolbox™ perspective and find the Quick Panel. Click on the debug launch configuration and wait for the programming to complete and the debugging process to start.

<img src="images/debug_start.jpg" style="zoom:100%;" />

#### System Basis Chip Development Mode

A special mode, called SBC Development Mode, is available during software development or debugging of the system. The watchdog counter is stopped and does not need to be triggered. This mode can be accessed by setting the TEST [**FO3**] pin to GND during SBC Init Mode.

## Legal Disclaimer

The evaluation board including the software is for testing purposes only and, because it has limited functions and limited resilience, is not suitable for permanent use under real conditions. If the evaluation board is nevertheless used under real conditions, this is done at one’s responsibility; any liability of Rutronik is insofar excluded. 

<img src="images/rutronik_origin_kaunas.png" style="zoom:50%;" />




Installation
============

Guide microcontroller setup develop
------------------------------------ 
..  rst-class:: bignums

1. Use your favourite Programing Environment a) CLion b) VSCode

2.a (for CLion)
    Install Extension `OpenOCD <https://gnutoolchains.com/arm-eabi/openocd/>_`
      Open the Plugin Manager in the Serrings in the Menue under File->Settings/Plugin Manager
      Search for  **OpenOCD** and install. A restart of windows will be required.
	  
	  Restart CLion and Select the Board at new start.
	  Now in the configurations select OpenOCD.	  

2.b (for VSCode)
    1) Install Extension `PlatformIO <https://platformio.org/>_`
	
      Open the Extension Manager on the left side bar. 
      Search for  **PlatformIO** and install. A restart of the window will be required.

       .. image:: ../images/extPIO.png
           :width: 399
           :alt: ExtensionManager 
 
    2) Open Project Folder containing **platformio.ini** 
    This assures that PlatformIO is recognizing the Projectstructure. If it is loaded correctly, it should open up the *PlatformIO Home* Page along with the loaded **platformio.ini**

         .. image:: ../images/initPIO.png
              :width: 399
              :alt: inital Project Page 
 

3. Install `STM32CubeMX <https://www.st.com/content/st_com/en/stm32cubemx.license=1696990725151.product=STM32CubeMX-Win.version=6.9.2.html?logged#>`_

    Windows: download the installer and install, assure the Drivers of the uController are installed

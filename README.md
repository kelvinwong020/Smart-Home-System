# Smart-Home-System
HKUST ELEC 3300 Project
![Smart Home System](/assets/img/smart_home.jpg)
![System Diagram](/assets/img/smart_home_overview.png)

# <a href="https://youtu.be/idSp3R__D4U">YouTube Demo</a>

# Description
In this project, we built an IOT(Internet Of Things) system that utilizes STM32F103 development boards to control an array of sensors and devices. This systems contains 4 major components: room monitoring, automatic brightness control, automatic temperature control, and control over WLAN. The project aims to provide a suite of centralized electronic devices that can improve the quality of life within a household. Originally, the system was intended to have a door lock control system and a mobile application for remote control. But due to budget and time constraints, these components was reduced to room monitoring and web server instead.

The technology used for this project are: C, I2C, UART, SPI, GPIO, STLink, STM32

IDE Utilized: Keli V5, STM32CubeMX

# Room Monitoring
For security/monitoring purposes, we included an OV7725 camera that gives live feed of the environment being monitored. The OV7725 camera contains a CMOS that captures the image data and feeds it to a FIFO buffer. The STM32 borad than reads from the FIFO and displays the data on a resistive touch screen(XPT2046).
<br>![OV7725](/assets/img/ov7725.png)
![monitor](/assets/img/room_monitoring.png)

To facilitate an easy-to-use interface, we have included an touch screen that allows users to control the functionality of the board. XPT2046 is controlled using SPI. The X and Y coordinates of each touch is obtained by detecting the voltage of the ITO layer and translating the information with ADC. The X and Y coordiantes of touch are then used to determine which subroutines to trigger.
<br>![Touch1](/assets/img/touch_screen1.png)
![Touch2](/assets/img/touch_screen2.png)


# Automatic Brightness Control

# Automatic Temperature Control

# Web Server / WLAN Control

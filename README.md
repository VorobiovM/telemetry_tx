## xBee - Challanger antenna

On the challanger vehicle, __xBee__ should be located at the top of the windshield.  
Its purpose is to send CAN data to the __escort car antenna__.  
___Optionaly:___ Store CAN raw data in the on-board Micro-SD card.

### Usage

#### Debug (WIP):

1. Install [STM32 Cube IDE](https://www.st.com/en/development-tools/stm32cubeide.html#get-software) on your host machine.
1. Start STM32 Cube IDE.
1. Open project: `File > Open Projects from File System...`.
1. Select `Import source` directory to *<repo_path>/src_tx/CAN_Tranciever.ioc*.
1. Press `Finish`.
> Additional configuring is required.
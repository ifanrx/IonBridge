# IonBridge

This repository contains the open-source power management module for the power adapter developed by ifanr / CANDYSIGN.

## 许可

请参阅 LICENSE 文件以了解更多详细信息。

我们欢迎非商业用途和小批量（单一型号总生产量小于等于一百台）使用。然而，对于大批量或盈利生产，需要获得商业许可。

请注意，虽然这是一个开源软件项目，但这并不意味着我们放弃了对该项目的版权。

## License
Please refer to the LICENSE file for more details.

We welcome non-commercial use and small batch runs. However, for larger batch or for-profit productions, a commercial license is required.

Please note that although this is an open-source software project, it does not mean we are relinquishing copyright to it.

## Contributing
The project is still in its early stages. We are currently sorting out copyright issues with our legal team to ensure we can legally acquire a perpetual, non-exclusive license for contributions while remaining in compliance with the relevant legal framework. 

As a result, we are unable to accept external contributions at this time. Stay tuned for updates! Once the issues are resolved, you will be required to sign a Contributor License Agreement (CLA) to authorize us to use your copyrighted work non-exclusively.

##  What's Missing
The proprietary implementation of power allocation is a patented feature and is not included in the open-source scope of this repository.

## Development Setup
To get started, you need to install the [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/)

### Prerequisites
Install the required dependencies:

```bash
$ brew install cmake ninja dfu-util
# Install Python 3 if it's not already installed
$ brew install python3
```

### Getting ESP-IDF
Clone the ESP-IDF repository and set it up:

```bash
$ mkdir -p ~/esp
$ cd ~/esp
$ git clone --recursive https://github.com/espressif/esp-idf.git
$ cd esp-idf
$ git checkout v5.4
```

### Updating Submodules
Update the ESP-IDF submodules:

```bash
$ cd ~/esp/esp-idf
$ git submodule update --init --recursive
```

### Installing ESP-IDF Tools
Run the installation script to install the ESP-IDF tools:

```bash
$ ./install.sh all
```

### Setting Environment Variables
Set up the required environment variables:

```bash
$ . $HOME/esp/esp-idf/export.sh
```
Alternatively, you can add the following alias to your shell profile for convenience:

```bash
alias get_idf='. $HOME/esp/esp-idf/export.sh'
```

### Configure the ESP32 Target
Navigate to the project root directory and set the ESP32 target:

```bash
$ idf.py set-target esp32c3
```

Copy sdkconfig.develop or sdkconfig.fake to sdkconfig:

```bash
$ cp configs/sdkconfig.develop sdkconfig
```

### Build the Project
Build the project using the following command:

```bash
$ idf.py build
```

## HummingBoard
HummingBoard is a fully functional development board for the power adapter in this project, designed specifically for development and testing.

- **Comprehensive Features**: Provides a complete development environment for power conversion.
- **Schematic Support**: You can find detailed schematics and related documentation in [HummingBoard](/HummingBoard/).

### lis2dw12 driver sample dev in Zephyr environment
----
### Zephyr SDK environment
- Install toolchain
<https://docs.zephyrproject.org/latest/develop/toolchains/zephyr_sdk.html>
```
unzip zephyr-sdk-0.16.0_windows-x86_64.7z
cd zephyr-sdk-0.16.0
setup.cmd
```
- Install west
```
pip3 install -U west
```
- nRF Connect SDK v2.5.2
```
west init -m https://github.com/nrfconnect/sdk-nrf --mr v2.5.2
west update
python -m pip install -r ./zephyr/scripts/requirements.txt
python -m pip install -r ./nrf/scripts/requirements.txt
python -m pip install -r ./bootloader/mcuboot/scripts/requirements.txt
```

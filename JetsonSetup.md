# 

1. Do **not** connect the Jetson to your computer at this stage.
2. Run `sdkmanager` on the host computer, and select both "Host Machine" and "Jetson TX2" to download JetPack to the host.
    ![image](https://user-images.githubusercontent.com/3406269/116788542-c9e12780-aa6f-11eb-94ee-167d64f6a90e.png)
    Click next past Step 02, accepting the license along the way.
3. After Step 03 of the sdkmanager, the Jetpack image will be created, and it will ask you to connect the Jetson to the host computer: 
    ![image](https://user-images.githubusercontent.com/3406269/116788518-a5854b00-aa6f-11eb-9d03-fb108bf09b9e.png)
4. Click on "Skip" and confirm that you want to skip installation to the Jetson:
    ![image](https://user-images.githubusercontent.com/3406269/116788576-03b22e00-aa70-11eb-8b3f-8ebebefb6994.png)
5. At this point, you should see JetPack installed on your host computer at `~/nvidia/nvidia_sdk/`:
    ```bash
    joydeepb@ubuntu:~$ cd nvidia/nvidia_sdk
    joydeepb@ubuntu:~/nvidia/nvidia_sdk$ ls -lthr
    total 8.0K
    drwxrwxr-x 4 joydeepb joydeepb 4.0K May  1 09:16 JetPack_4.5.1_Linux
    drwxrwxr-x 3 joydeepb joydeepb 4.0K May  1 09:19 JetPack_4.5.1_Linux_JETSON_TX2
    ```
6. Go to the [Connect Tech support page](http://connecttech.com/product/orbitty-carrier-for-nvidia-jetson-tx2-tx1/) and download 
    the L4T support package into `<JetPack_install_dir>/JetPack_4.5.1_Linux_JETSON_TX2/Linux_for_Tegra/`. For example:
   ```bash
    joydeepb@ubuntu:~$ cd nvidia/nvidia_sdk/JetPack_4.5.1_Linux_JETSON_TX2/Linux_for_Tegra/
    joydeepb@ubuntu:~/nvidia/nvidia_sdk/JetPack_4.5.1_Linux_JETSON_TX2/Linux_for_Tegra$ wget https://connecttech.com/ftp/Drivers/CTI-L4T-TX2-32.5-V001.tgz
    --2021-05-01 09:14:59--  https://connecttech.com/ftp/Drivers/CTI-L4T-TX2-32.5-V001.tgz
    Resolving connecttech.com (connecttech.com)... 104.26.6.94, 172.67.72.40, 104.26.7.94, ...
    Connecting to connecttech.com (connecttech.com)|104.26.6.94|:443... connected.
    HTTP request sent, awaiting response... 200 OK
    Length: 519331075 (495M) [application/octet-stream]
    Saving to: ‘CTI-L4T-TX2-32.5-V001.tgz’

    CTI-L4T-TX2-32.5-V001.tgz                                 100%[===================================================================================================================================>] 495.27M  15.5MB/s    in 32s     

    2021-05-01 09:15:31 (15.4 MB/s) - ‘CTI-L4T-TX2-32.5-V001.tgz’ saved [519331075/519331075]
   ```
1. Extract the archive, and run the install script in the extracted directory:
    ```
    joydeepb@ubuntu:~/nvidia/nvidia_sdk/JetPack_4.5.1_Linux_JETSON_TX2/Linux_for_Tegra$ tar xzf CTI-L4T-TX2-32.5-V001.tgz
    joydeepb@ubuntu:~/nvidia/nvidia_sdk/JetPack_4.5.1_Linux_JETSON_TX2/Linux_for_Tegra$ cd ./CTI-L4T
    joydeepb@ubuntu:~/nvidia/nvidia_sdk/JetPack_4.5.1_Linux_JETSON_TX2/Linux_for_Tegra/CTI-L4T$ sudo ./install.sh
    ```
1. Connect the Jetson to the host computer and enter firmware update mode:
      1. Connect the USB OTG port
      2. Press and hold the recovery button
      3. Press and release the power button
      4. Release the recovery button
3. Now reflash the Jetson from the `Linux_for_Tegra` directory:
    ```
    joydeepb@ubuntu:~/nvidia/nvidia_sdk/JetPack_4.5.1_Linux_JETSON_TX2/Linux_for_Tegra$ sudo ./cti-flash.sh
    ```

## Installing CUDA and PyTorch

1. Ensure that the Jetson is running, booted into Ubuntu, and accessible via the local network (e.g. WiFi) 
    from the host computer. Let the Jetson's IP address on the local network be `192.168.10.100`
2. On the host computer, run `sdkmanager`
3. Select "Jetson" as the product category, deselect Host Machine, Jetson TX2 as the Target Hardware, and 
    select the correct JetPack version installed on the Jetson.
4. On step 2, **deselect "Jetson OS"** from the list of components and select `CUDA` and additional components:
    ![image](https://user-images.githubusercontent.com/3406269/116788926-dfefe780-aa71-11eb-8396-6ba00247c308.png)
5. Manually enter the IP address of the Jetson computer, and the admin username and password:
    ![image](https://user-images.githubusercontent.com/3406269/116788939-f9912f00-aa71-11eb-980d-24dc96409fd3.png)
6. To install pytorch, follow the instructions here: https://forums.developer.nvidia.com/t/pytorch-for-jetson-version-1-8-0-now-available/72048 


## Optional Tools

To monitor CPU and GPU usage, clone and install [Jetson Stats](https://github.com/rbonghi/jetson_stats)
![image](https://github.com/rbonghi/jetson_stats/wiki/images/jtop.gif)

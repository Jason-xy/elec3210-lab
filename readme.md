# ELEC 3210 Dev Environment Setup

## Supported Platforms

|  | Windows | MacOS | Linux |
| --- | --- | --- | --- |
| amd64 | ✅ | ✅ | ✅ |
| arm64 | ✅ | ✅ | ✅ |

## Tested Platforms

- Windows 11 (Recommend with WSL2)
- MacOS (Apple Silicon)
- Ubuntu

## Prerequisites

- [Docker](https://docs.docker.com/get-docker/)
- [Docker Compose](https://docs.docker.com/compose/install/)

## Start the Dev Environment

Use `ekf_slam_dev` as an example:

### Modify the path in `xxx_dev.sh`

Set the `APP_PATH` and `DATA_PATH` to the correct path on your machine.

```bash
export APP_PATH=$SCRIPT_DIR/ekf_slam
export DATA_PATH=$SCRIPT_DIR/dataset
```

### Start the Dev Environment

```bash
chmod +x ekf_slam_dev.sh
./ekf_slam_dev.sh
```

### Access the Dev Environment (Recommend to use VSCode)

Required Extensions:

| Extension | ScreenShot |
| --- | --- |
| Docker | ![](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/wpcos-1300629776/Gallery20240827175454.png) |
| Remote Development | ![](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/wpcos-1300629776/Gallery20240827175422.png) |
| C/C++ Extension Pack | ![](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/wpcos-1300629776/Gallery20240827175144.png) |

Right-click on the container and click `Attach Visual Studio Code`.
![](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/wpcos-1300629776/Gallery20240827180027.png)

Then click `File -> Open Folder` on the newly popped up VSCode window and select the workspace folder `/ws`.

![](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/wpcos-1300629776/Gallery20240827180329.png)

### Compile and Run (Inside the containered environment)

```bash
cd /ws
catkin_make
source devel/setup.bash
roslaunch ekf_slam ekf_slam.launch
```

### Access GUI Desktop

Open a browser and visit `http://localhost:8080/` to access the GUI desktop.

The default resolution is `1200x800`. You can change it in the `docker/lab/supervisord.conf` file and rebuild the image.

![](https://wpcos-1300629776.cos.ap-chengdu.myqcloud.com/wpcos-1300629776/Gallery20240827200722.png)

## Dataset

Download the dataset from [OneDrive](https://hkustconnect-my.sharepoint.com/:f:/g/personal/yxuew_connect_ust_hk/EvNk5iuMFM1NkEr5jKU8t_sBQ8kD0Upbh8-hUefh_FPxwg?e=eUildp) and put it in your `DATA_PATH` folder.
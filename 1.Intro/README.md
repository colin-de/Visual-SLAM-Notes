# ex01

## Linux Basis

1. Install software

   via source: `sudo apt install *`
   via deb package: `sudo dpkg -i *.deb`
   Default installation path is `/usr/share`

2. linux environment variables

   `export PATH=/path`

3. linux root directory structure

   `/ # The Very Top (Root) of The File Tree. Holds Everything else.
   ├── bin # Stores Common Linux user command binaries 
   ├── boot # Bootable linux Kernel and bootloader config files
   ├── dev # Device files, including hardware
   ├── etc # Configuration file storage directory
   ├── home # Host user directory
   ├─ lib # Library location for system calls
   ├── lib64			
   ├── lost+found
   ├── media 
   ├── mnt
   ├── opt 
   ├── proc
   ├── root 
   ├─ run
   ├─ sbin
   ├─ snap
   ├─ srv
   ├─ swapfile
   ├─ sys
   ├─ tmp
   ├── usr 
   ├── var `

4. Add executable authority to a.sh

   `chmod +x a.sh` 

5. Change the owner of a file a.sh to xiang:xiang

   `sudo chown xiang:xiang a.sh`

## SLAM overview & literature reading

1. SLAM Applications

   Autonomous driving, unmanned logistics vehicles, AR/VR

2. The relationship between location and mapping

   In SLAM system, localization is the process of the robot knowing its position in the environment and map building is the process of generating a map of the surrounding environment. Generally speaking, a robot starts moving from an unknown place and incrementally builds a map based on its own localization to achieve autonomous navigation. The two are interdependent, as accurate positioning requires accurate maps, and accurate maps come from accurate positioning.

3. History of SLAM

   The SLAM problem was first proposed by Smith Self and Cheeseman in 1988 and was considered to be the key to achieving a truly fully autonomous mobile robot. Early Kalman filtering was the mainstream method for SLAM, including KF, EKF, UKF, PF, etc., to perform maximum likelihood estimation of the solution sensor information; in the last decade or so, graph optimization has gradually become the mainstream method for SLAM, introducing BA into SLAM, which is not an iterative process but a least-squares process for historical information, and through optimization, the error is averaged into each observation, which can more intuitive to express the optimization problem.

   classical age (1986-2004)

   Probabilistic approachesnd data association

   algorithmic-analysis age (2004-2015)

   Observability, consistency and convergence

   robust-perception age (2015-future)

   robust performance, high-level understanding, resource awareness, task-driven inference

4. Classical work in SLAM 

   > [1] Georg Klein and David Murray, "Parallel Tracking and Mapping for Small AR Workspaces", Proc. ISMAR 2007
   >
   > [2] Mourikis A, Roumeliotis S. A multi-state constraint Kalman fil-ter for vision-aided inertial navigation[C] //Proceedings of IEEE International Conference on obotics and Automation. Los Alamitos: IEEE Computer Society Press, 2007: 3565-3572
   >
   > [3]  T. Qin, P. Li and S. Shen, "VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator," in IEEE Transactions on Robotics, vol. 34, no. 4, pp. 1004-1020, Aug. 2018, doi: 10.1109/TRO.2018.2853729.

   ## CMake Practice

1. include/hello.h 和 src/hello.c 构成了 libhello.so 库。hello.c 中提供一个函数 sayHello(),调用此函数时往屏幕输出一行“Hello SLAM”。我们已经为你准备了 hello.h 和 hello.c 这两个文件,见“code/”目录下。
2. 文件 useHello.c 中含有一个 main 函数,它可以编译成一个可执行文件,名为“sayhello”。
3. 默认用 Release 模式编译这个工程。
4. 如果用户使用 sudo make install,那么将 hello.h 放至/usr/local/include/下,将 libhello.so 放至/usr/local/lib/下。请按照上述要求组织源代码文件,并书写 CMakeLists.txt。

## Understand ORB-SLAM2 framework

1. git clone https://github.com/raulmur/ORB_SLAM2 

2. 此时我们不着急直接运行 ORB-SLAM2,让我们首先来看它的代码结构。ORB-SLAM2 是一个
    cmake 工程,所以可以从 CMakeLists.txt 上面来了解它的组织方式。阅读 ORB-SLAM2 代码目录下的 CMakeLists.txt,回答问题:
    (a) ORB-SLAM2 将编译出什么结果?有几个库文件和可执行文件?

  The executable files are: rgbd_tum, stereo_
  kitti, stereo_euroc, mono_tum, mono_kitti, mono_euroc

  (b) ORB-SLAM2 中的 include, src, Examples 三个文件夹中都含有什么内容?

  include contains the header files used in the program
  src contains the source files of the program
  examples includes monocular, rgbd, ros, and stereo folders, which are used as program entry points to generate the corresponding executable files.

  (c) ORB-SLAM2 中的可执行文件链接到了哪些库?它们的名字是什么?

  The library liborb_slam2.so is linked to libDBoW2.so, libg2o.so, opencv, eigen3, and pangolin.

## Run ORB-SLAM2 with camera or video



1. 为了实际运行 ORB-SLAM2,你需要安装它的依赖项,并通过它本身的编译

   ![image-20210530223445821](/home/lwh/.config/Typora/typora-user-images/image-20210530223445821.png)

2. 如何将 myslam.cpp或 myvideo.cpp 加入到 ORB-SLAM2 工程中.
   将myvideo拷贝过去并在cmake中添加可执行文件即可，需注意yaml和mp4文件路径
   `add_executable(myvideo src/myvideo.cpp)`
   `target_link_libraries(myvideo ${PROJECT_NAME}`  

3. 现在你的程序应该可以编译出结果了

   ![image-20210530230424831](/home/lwh/.config/Typora/typora-user-images/image-20210530230424831.png)
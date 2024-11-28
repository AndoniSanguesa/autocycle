# Instaling `autocycle` ROS Package on Ubuntu Focal 20.04 #

Note: As of 20/27/2020, ROS does not support Ubuntu Groovy 20.1. You must install Ubuntu Focal 20.04 or earlier to install ROS succesfully.

## 1. Install ROS and Create Catkin Workspace##

   Follow steps [here](http://wiki.ros.org/noetic/Installation/Ubuntu) to install ROS noetic  
   Follow steps [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) to create a catkin workspace

## 2. Clone Repository ##

   Clone the repository under the src/ directory of your catkin workspace

## 3. Clone Livox-SDK Repository ##

   Clone the repository [here](https://github.com/Livox-SDK/Livox-SDK) under the root directory of this pacakge (src/autocycle/)  
   Follow the setup instructions listed in the Livox-SDK's `README.md`

## 4. Configure Package ##

   While in the root directory of your catkin workspace call  
   `catkin_make`  
     
   NOTE: This will generate two directories "build" and "devel" there is no need to delete these between subsequent `catkin_make` calls
   
## 5. Setup Environment ##

   NOTE: This step must be repeated for evey separate terminal open.  
   while in the root directory of your catkin workspace call  
   `./devel/setup.bash`
   
## 6. Start Navigation ##

   Run the command  
   `roslaunch autocycle nav.launch`

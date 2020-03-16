# vins-mono(self-improved)
an improved version of vins-mono

I: OK, now, let me introduce to you a improved version of ...

you: Hey,hey,these seem like, well, how to say, exactly the same as the original masterpiece VINS-Mono, so what have you done man???

I: OK, admittedly, I do not change the ideas, the variables and the functions...

you: hahahahahaha,xixixixixi,233333333...

I: I establish include for each packages and put all the .h into them.

you:oh-hoh?

I: And when I read the original codes, I think the main vio functions, estimator.cpp, they are too long! Maybe it is not easier for new-comer to figure out the strcture.

So I extract codes out of some functions like initialStructure(). optimization()...

And,I put the steps initialization and backend-optimization out of estimator.h/cpp into new files initial.h/cpp and backend.h/cpp and form 2 new classes: class Backend and class Initial.

As you can see, their are much fewer lines in estimator.cpp, from 1200 to 220 lines. There are only main processes in the .cpp file. So the structure is much clear.

The official link is:
https://github.com/HKUST-Aerial-Robotics/VINS-Mono

To run this project is exactly the same as the original one, take it easy. Put all the files above into src, and put src into catkin_ws.

Open terminal, type in "catkin_make" and drink a cup of coffee.

Then, open one terminal, type "roscore", of course you have to install ROS in advance.

And open 2 terminals and in the path of devel,type "source setup.bash", then fill in:

roslaunch vins_estimator euroc.launch 
roslaunch vins_estimator vins_rviz.launch

Then another ternimal with:

rosbag play YOUR_PATH_TO_DATASET/MH_01_easy.bag 


Good time.





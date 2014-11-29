all: robot_programming_manual.pdf html

robot_programming_manual.pdf:
	roseus ../eus/lib/llib/documentation.l "(progn\
	(require :robot-interface \"package://pr2eus/robot-interface.l\") (require \"package://turtleboteus/euslisp/turtlebot-with-sensors-robot.l\") (ros::load-ros-manifest \"kobuki_msgs\") (ros::load-ros-manifest \"dynamixel_msgs\") (ros::load-ros-manifest \"dynamixel_controllers\") (ros::load-ros-manifest \"control_msgs\") (ros::load-ros-manifest \"move_base_msgs\") (ros::load-ros-manifest \"diagnostic_msgs\")(ros::load-ros-manifest \"linux_hardware\")\
	(make-document \"robot_programming_model.l\" \"robot_programming_model-func.tex\")\
	(exit))"
	roseus ../eus/lib/llib/documentation.l "(progn\
	(require :robot-interface \"package://pr2eus/robot-interface.l\") (require \"package://turtleboteus/euslisp/turtlebot-with-sensors-robot.l\") (ros::load-ros-manifest \"kobuki_msgs\") (ros::load-ros-manifest \"dynamixel_msgs\") (ros::load-ros-manifest \"dynamixel_controllers\") (ros::load-ros-manifest \"control_msgs\") (ros::load-ros-manifest \"move_base_msgs\") (ros::load-ros-manifest \"diagnostic_msgs\")(ros::load-ros-manifest \"linux_hardware\") (load \"robot_programming_model.l\")\
	(make-document \"robot_programming_ri.l\" \"robot_programming_ri-func.tex\")\
	(exit))"
	nkf --in-place -e *.tex;
	platex robot_programming_manual;
	dvipdfmx robot_programming_manual.dvi;

open:
	gnome-open robot_programming_manual.pdf
	gnome-open html/index.html

html: robot_programming_manual.pdf
	mkdir -p html && charset="UTF-8" latex2html -dir html -local_icons -auto_prefix -iso_language JP robot_programming_manual

clean:
	rm -f *.log *.out *.toc *.dvi *.idx *.aux *.pdf robot_programming_model-func.tex robot_programming_ri-func.tex
	rm -fr html

TMPDIR := $(shell mktemp -d)

all: robot_programming_manual.pdf html

copy_eus_tex:
	mkdir -p ${TMPDIR}
	cp robot_programming_manual.tex ${TMPDIR}
	cp robot_programming_*.l ${TMPDIR}

robot_programming_manual.pdf: copy_eus_tex
	(cd ${TMPDIR};\
	roseus ../eus/lib/llib/documentation.l "(progn\
	(require :robot-interface \"package://pr2eus/robot-interface.l\") (require \"package://turtleboteus/euslisp/turtlebot-with-sensors-robot.l\") (ros::load-ros-manifest \"kobuki_msgs\") (ros::load-ros-manifest \"dynamixel_msgs\") (ros::load-ros-manifest \"dynamixel_controllers\") (ros::load-ros-manifest \"control_msgs\") (ros::load-ros-manifest \"move_base_msgs\") (ros::load-ros-manifest \"diagnostic_msgs\")(ros::load-ros-manifest \"linux_hardware\")\
	(make-document \"robot_programming_model.l\" \"robot_programming_model-func.tex\")\
	(exit))";\
	roseus ../eus/lib/llib/documentation.l "(progn\
	(require :robot-interface \"package://pr2eus/robot-interface.l\") (require \"package://turtleboteus/euslisp/turtlebot-with-sensors-robot.l\") (ros::load-ros-manifest \"kobuki_msgs\") (ros::load-ros-manifest \"dynamixel_msgs\") (ros::load-ros-manifest \"dynamixel_controllers\") (ros::load-ros-manifest \"control_msgs\") (ros::load-ros-manifest \"move_base_msgs\") (ros::load-ros-manifest \"diagnostic_msgs\")(ros::load-ros-manifest \"linux_hardware\") (load \"robot_programming_model.l\")\
	(make-document \"robot_programming_ri.l\" \"robot_programming_ri-func.tex\")\
	(exit))")
	nkf --in-place -e ${TMPDIR}/*.tex;
	(cd ${TMPDIR}; platex robot_programming_manual; dvipdfmx robot_programming_manual.dvi)

open:
	gnome-open robot_programming_manual.pdf
	gnome-open html/index.html

html: robot_programming_manual.pdf
	nkf --in-place -u ${TMPDIR}/*.tex
	mkdir ${TMPDIR}/html
	sed -i 's@^\(\\\usepackage.*{hyperref}\)@%\1@' ${TMPDIR}/robot_programming_manual.tex
	sed -i 's@\\\documentclass\[\]{jarticle}@\\\documentclass\[\]{article}@' ${TMPDIR}/robot_programming_manual.tex
	sed -i 's@^\\\ifnum 42146@%\\\ifnum 42146@' ${TMPDIR}/robot_programming_manual.tex
	(cd ${TMPDIR}; charset="UTF-8" latex2html -dir ${TMPDIR}/html -local_icons -auto_prefix -iso_language JP robot_programming_manual)
	(cd ${TMPDIR}/html; sed -i 's@</TITLE>@</TITLE>\n<META HTTP-EQUIV="Content-Type" CONTENT="text/html; charset=UTF-8">@' *.html index.html)
	sed -i 's@%\(\\\usepackage.*{hyperref}\)@\1@' ${TMPDIR}/robot_programming_manual.tex
	sed -i 's@\\\documentclass\[\]{article}@\\\documentclass\[\]{jarticle}@' ${TMPDIR}/robot_programming_manual.tex
	sed -i 's@^%\\\ifnum 42146@\\\ifnum 42146@' ${TMPDIR}/robot_programming_manual.tex
	cp -r ${TMPDIR}/html ./
	cp ${TMPDIR}/robot_programming_manual.pdf ./html/
	rm -fr ${TMPDIR}

clean:
	rm -f *.log *.out *.toc *.dvi *.idx *.aux *.pdf robot_programming_model-func.tex robot_programming_ri-func.tex
	rm -fr html ${TMPDIR}

function warn {
	export DISPLAY=:0
	a=$(rostopic echo -n 1 /battery_state |grep lifePercent|cut -f 2 -d ' ')
		c=$(($a*12))
		h=$(($c/60))
		m=$(($c%60))
		b="had_an_accident_or_got_lost"
		from=$(cat $HOME/.batterycheck|grep Fromemail|sed s/Fromemail.//)
		echo From: $from
		to=$(cat $HOME/.batterycheck|grep Toemail|sed s/Toemail.//)
		echo To: $to
		echo ----------------------
		subject="Battery_low"
		if [ $# == 1 ]; then 
		if [ $1 == 'test' ]; then
			subject="Battery_low_TEST_MESSAGE"
		fi
		fi
		cat $HOME/.batterycheck|grep -v Fromemail|grep -v Toemail|sed s/XXX/$a/ |sed s/YYY/$b/|sed s/ZZh/$h/|sed s/ZZm/$m/|tr _ ' ' 
		cat $HOME/.batterycheck|grep -v Fromemail|grep -v Toemail|sed s/XXX/$a/ |sed s/YYY/$b/|sed s/ZZh/$h/|sed s/ZZm/$m/|mail -r $from -s $subject $to
		firefox /localhome/patroller/recovery/index.html&
}

source /opt/strands/strands_catkin_ws/devel/setup.bash
if [ $# == 3 ]; then 
if [ $3 == 'test' ]; then warn test; fi;fi
if [ $(rostopic echo -n 1 /battery_state|grep powerSupplyPresent|grep -c True) == 0 ];then 
if [ $(($(rostopic echo -n 1 /battery_state |grep lifePercent|cut -f 2 -d ' ') < $1)) == 1 ];
then
warn
fi
if [ $(($(rostopic echo -n 1 /battery_state |grep lifePercent|cut -f 2 -d ' ') < $2)) == 1 ];
then
rosrun dynamic_reconfigure dynparam set /EBC Port0_5V_Enabled False
rosrun dynamic_reconfigure dynparam set /EBC Port0_12V_Enabled False
rosrun dynamic_reconfigure dynparam set /EBC Port0_24V_Enabled False
rosrun dynamic_reconfigure dynparam set /EBC Port1_5V_Enabled False
rosrun dynamic_reconfigure dynparam set /EBC Port1_12V_Enabled False 
rosrun dynamic_reconfigure dynparam set /EBC Port1_24V_Enabled False
echo "sudo /sbin/shutdown -P now"
fi
fi



#!/bin/bash

export MVCAM_COMMON_RUNENV=/opt/MVS/lib 

FIND_PATH=${MVCAM_COMMON_RUNENV//\//\\\/}

LD_LIBRARY_PATH=${LD_LIBRARY_PATH//${FIND_PATH}\/64:/}
LD_LIBRARY_PATH=${LD_LIBRARY_PATH//${FIND_PATH}\/32:/}
LD_LIBRARY_PATH=${LD_LIBRARY_PATH//${FIND_PATH}\/armhf:/}
LD_LIBRARY_PATH=${LD_LIBRARY_PATH//${FIND_PATH}\/aarch64:/}

LD_LIBRARY_PATH=${LD_LIBRARY_PATH//${FIND_PATH}\/64/}
LD_LIBRARY_PATH=${LD_LIBRARY_PATH//${FIND_PATH}\/32/}
LD_LIBRARY_PATH=${LD_LIBRARY_PATH//${FIND_PATH}\/armhf/}
LD_LIBRARY_PATH=${LD_LIBRARY_PATH//${FIND_PATH}\/aarch64/}

ADD_TO_LIBRARY_PATH=

# platform array
PLATFORM_ARRAY=("32" "64" "armhf" "aarch64")

# find LD_LIBRARY_PATH, only one param: The platform in PLATFORM_ARRAY
function get_LD_LIBRARY_PATH(){
	echo $1
	if ! echo ${ADD_TO_LIBRARY_PATH} | grep -q ${MVCAM_COMMON_RUNENV}/$1; then
	  if [ "$ADD_TO_LIBRARY_PATH" = "" ]; then
		ADD_TO_LIBRARY_PATH=${MVCAM_COMMON_RUNENV}/$1
	  else
		ADD_TO_LIBRARY_PATH=${MVCAM_COMMON_RUNENV}/$1:${ADD_TO_LIBRARY_PATH}
	  fi
	fi
	
	if ! echo ${LD_LIBRARY_PATH} | grep -q ${MVCAM_COMMON_RUNENV}/$1; then
	  if [ "$LD_LIBRARY_PATH" = "" ]; then
		LD_LIBRARY_PATH=${MVCAM_COMMON_RUNENV}/$1
	  else
		LD_LIBRARY_PATH=${MVCAM_COMMON_RUNENV}/$1:${LD_LIBRARY_PATH}
	  fi
	fi
}

# put dynamic library path to LD_LIBRARY_PATH
for platform in ${PLATFORM_ARRAY[@]} 
do
	if [ -d /opt/MVS/lib/$platform ]; then
		get_LD_LIBRARY_PATH $platform
	fi
done

export LD_LIBRARY_PATH

#echo $LD_LIBRARY_PATH

# first param:  true --> need sudo Permission; false --> no need sudo Permission
# second param: input file dir
function export_LD_LIBRARY_PATH(){
	if [ $1 == true ]; then
		if [ ! -f "$2" ]; then
			break
		fi
		
		if [ ! -w "$2" ]; then
			echo "Permission Denied to write $2"
			continue
		fi
	fi
	
	sed -i '/^export.MVCAM_COMMON_RUNENV/d' $2
	
	sed -i "/^export.LD_LIBRARY_PATH*/s/${FIND_PATH}\/64://" $2
	sed -i "/^export.LD_LIBRARY_PATH*/s/${FIND_PATH}\/32://" $2
	sed -i "/^export.LD_LIBRARY_PATH*/s/${FIND_PATH}\/armhf://" $2
	sed -i "/^export.LD_LIBRARY_PATH*/s/${FIND_PATH}\/aarch64://" $2
	
	sed -i "/^export.LD_LIBRARY_PATH*/s/${FIND_PATH}\/64//" $2
	sed -i "/^export.LD_LIBRARY_PATH*/s/${FIND_PATH}\/32//" $2
	sed -i "/^export.LD_LIBRARY_PATH*/s/${FIND_PATH}\/armhf//" $2
	sed -i "/^export.LD_LIBRARY_PATH*/s/${FIND_PATH}\/aarch64//" $2
	
	sed -i '/^export.LD_LIBRARY_PATH=$/d' $2
	sed -i '/^export.LD_LIBRARY_PATH=\$LD_LIBRARY_PATH$/d' $2
	
	
	if grep --silent ^export.MVCAM_COMMON_RUNENV $2; then
		echo "$2: env path has already been set"    
	else
		# judge the last line is blank Line. if not, add a blank line
		if [ "`tail -1 $2`" != "" ]; then
			echo "" >> $2
		fi
		echo "export MVCAM_COMMON_RUNENV=/opt/MVS/lib" >> $2
	fi
	
	# For example x86 platform, Append the following command to the last line of the file
	# export LD_LIBRARY_PATH=/opt/MVS/lib/32:/opt/MVS/lib/64:$LD_LIBRARY_PATH
	echo "export LD_LIBRARY_PATH=$ADD_TO_LIBRARY_PATH:\$LD_LIBRARY_PATH" >> $2
	
	source $2
}

# for common user
for i in /home/*/.profile; 
do 
	if [ -f $i ]; then
		echo $i; 
		export_LD_LIBRARY_PATH true $i
	fi
done

# for common user
for i in /home/*/.bashrc; 
do 
	if [ -f $i ]; then
		echo $i; 
		export_LD_LIBRARY_PATH true $i
	fi
done

# for common user
for i in /home/*/.bash_profile; 
do 
	if [ -f $i ]; then
		echo $i;
		export_LD_LIBRARY_PATH true $i
	fi
done

if [ -w /etc/profile ]; then
	# for sudo user
	export_LD_LIBRARY_PATH true /etc/profile
fi

if [ -w ~/.bashrc ]; then
	# for current user
	export_LD_LIBRARY_PATH false ~/.bashrc
fi


SETUP_FILE=/etc/rc.local
# 判断是否已经添加了开启启动的脚本
if [ -z "`grep "bash /opt/MVS/bin/set_rp_filter.sh" $SETUP_FILE`" ]; then
	# 将开机启动脚本写到/etc/rc.local中
	exitLine=`grep -n "^exit 0" $SETUP_FILE | awk -F ":" '{print $1}'`
	sed -e "${exitLine}i if [ -f /opt/MVS/bin/set_rp_filter.sh ]; then\n\tbash /opt/MVS/bin/set_rp_filter.sh\nfi\n" -i $SETUP_FILE

fi
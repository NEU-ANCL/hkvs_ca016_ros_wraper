#! /bin/sh
DIRNAME=`dirname $0`
PWD=`pwd`

USER_ID=`id -u`
# Check required priviledge
if [ "$USER_ID" != "0" ]; then
echo "LogServer can only be installed by root user or sudoer"
exit 1
fi

ARCH=$(uname -m | sed 's/x86_//;s/i[3-6]86/32/')

if [ -f /etc/debian_version ]; then
    OS=Debian  # XXX or Ubuntu??
    VER=$(cat /etc/debian_version)
elif [ -f /etc/redhat-release ]; then
    # TODO add code for Red Hat and CentOS here
    OS=Redhat  # XXX or CentOs??
    VER=$(cat /etc/debian_version)
else
    OS=$(uname -s)
    VER=$(uname -r)
fi

if [ "Debian" = "$OS" ]; then
	cp -f /opt/MVS/logserver/Debian/MvLogServerd /etc/init.d/
elif [ "Redhat" = "$OS" ]; then
	cp -f /opt/MVS/logserver/Redhat/MvLogServerd /etc/init.d/
else
	cp -f /opt/MVS/logserver/Debian/MvLogServerd /etc/init.d/
fi

mv -f /etc/init.d/MvLogServerd /etc/init.d/MvLogServer
chmod 777 /etc/init.d/MvLogServer

if [ "Debian" = "$OS" ]; then
	update-rc.d MvLogServer defaults
elif [ "Redhat" = "$OS" ]; then
    chkconfig --add MvLogServer
	chkconfig MvLogServer on
else
	echo "Can not recognize system"
fi

service MvLogServer start

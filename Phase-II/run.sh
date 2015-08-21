#!/bin/bash

export MODULE_NAME=an_rtl8150_usb;
export MODULE_FILE=an-rtl8150-usb.ko;

export INT1=eth0;
export INT2=eth1;

# run if user hits control-c
control_c()
{
  echo -en "\n Exiting .. Removing module .. \n"
  rmmod $MODULE_NAME;
  exit $?
}

# trap keyboard interrupt (control-c)
trap control_c SIGINT

if [[ $(lsmod | grep an_rtl8150_usb) ]]; then
    echo "Removing module ";
    rmmod $MODULE_NAME;
fi

echo "Inserting module..";
insmod $MODULE_FILE;

echo "Bring up ethernet interface"
ifconfig $INT1 up; 
ifconfig $INT2 up; 

echo "Assigning ip address.. ";
#ip ad add 10.0.0.10/24 dev $INT1;
ip ad add 10.0.0.20/24 dev $INT2


#ping -s 1111 -I $INT2 10.0.0.10 




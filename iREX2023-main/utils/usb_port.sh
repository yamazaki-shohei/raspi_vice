#!/bin/bash

DYN_PICK="FTDI_USB-RS422_Cable_FTKVM62P"
ROBOTIQ_2F_140="FTDI_USB_TO_RS-485_DA76NZTY"
DIGITAL_SCALE="Prolific_Technology_Inc._USB-Serial_Controller_D"
DPA_06="FTDI_DPA-06"

find_usb_port () {
  USB_PORT=$1
  RESULT="$USB_PORT unknown"
  # echo $USB_PORT
  ID_SERIAL=$( \
    udevadm info $USB_PORT | grep "ID_SERIAL=" | tr "=" " " \
    )
  # echo $ID_SERIAL
  ARRAY=($ID_SERIAL)
  # echo ${ARRAY[2]}
  if [ "${ARRAY[2]}" = "${DYN_PICK}" ]; then
    RESULT="$USB_PORT DynPick"
  elif [ "${ARRAY[2]}" = "${ROBOTIQ_2F_140}" ]; then
    RESULT="$USB_PORT ROBOTIQ 2F-140"
  elif [ "${ARRAY[2]}" = "${DIGITAL_SCALE}" ]; then
    RESULT="$USB_PORT Digital scale"
  elif [ "${ARRAY[2]}" = "${DPA_06}" ]; then
    RESULT="$USB_PORT DPA-06"
  fi
  echo "$RESULT"
}

for VAR1 in $( ls -1 /dev/ttyUSB* )
do
  find_usb_port $VAR1
done


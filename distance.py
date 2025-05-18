#!./bin/python3
import argparse
import datetime
import os
import paho.mqtt.client as mqtt
import serial
import socket
import sys
import time

SERIAL_PORT = "/dev/serial0"

# Make sure the serial port is enabled using raspi-config and the following lines are added to /boot/firmware/config.txt
#
# [all]
# enable_uart=1
# dtoverlay=pi3-disable-bt 

class DistanceMeasure:
  ## Board status 
  STA_OK = 0x00
  STA_ERR_CHECKSUM = 0x01
  STA_ERR_SERIAL = 0x02
  STA_ERR_CHECK_OUT_LIMIT = 0x03
  STA_ERR_CHECK_LOW_LIMIT = 0x04
  STA_ERR_DATA = 0x05

  ## last operate status, users can use this variable to determine the result of a function call. 
  last_operate_status = STA_ERR_DATA

  distance = 0

  ## Maximum range
  distance_max = 4500
  distance_min = 0
  range_max = 4500

  def __init__(self):
    self._ser = serial.Serial(SERIAL_PORT, 9600)
    if self._ser.isOpen() != True:
      self.last_operate_status = self.STA_ERR_SERIAL

  def set_dis_range(self, min, max):
    self.distance_max = max
    self.distance_min = min

  def getDistance(self):
    self._measure()
    return self.distance
  
  def _check_sum(self, l):
    return (l[0] + l[1] + l[2])&0x00ff

  def _measure(self):
    data = [0]*4
    timenow = time.time()

    # set default error code to be overwritten with a good value
    self.last_operate_status = self.STA_ERR_DATA

    # Wait for 4 characters to be ready and timeout after a second
    while (self._ser.inWaiting() < 4):
      time.sleep(0.01)
      if ((time.time() - timenow) > 1):
        break
   
    # Read the characters that are waiting 
    response = self._ser.read(self._ser.inWaiting())
    print("response",len(response),response)
 
    # This is looking through the characters received for the 0xff. The sequence should be 0xff,DH,DL,CHK.
    if len(response) < 4:
      return self.distance

    index = len(response) - 4
    while True:
      try:
        data[0] = ord(response[index])
      except:
        data[0] = response[index]

      if (data[0] == 0xFF):
        break
      elif (index > 0):
        index -= 1
      else:
        break

    if (data[0] != 0xFF):
      return self.distance

    try:
      data[1] = ord(response[index + 1])
      data[2] = ord(response[index + 2])
      data[3] = ord(response[index + 3])
    except:
      data[1] = response[index + 1]
      data[2] = response[index + 2]
      data[3] = response[index + 3]

    sum = self._check_sum(data)
    if sum != data[3]:
      self.last_operate_status = self.STA_ERR_CHECKSUM

    else:
      self.distance = data[1]*256 + data[2]
      self.last_operate_status = self.STA_OK

    if self.distance > self.distance_max:
      self.last_operate_status = self.STA_ERR_CHECK_OUT_LIMIT
      self.distance = self.distance_max

    elif self.distance < self.distance_min:
      self.last_operate_status = self.STA_ERR_CHECK_LOW_LIMIT
      self.distance = self.distance_min

    return self.distance

STATUS = {
  DistanceMeasure.STA_OK: "OK",
  DistanceMeasure.STA_ERR_CHECKSUM: "Checksum error",
  DistanceMeasure.STA_ERR_SERIAL: "Serial error",
  DistanceMeasure.STA_ERR_CHECK_OUT_LIMIT: "Upper limit error",
  DistanceMeasure.STA_ERR_CHECK_LOW_LIMIT: "Low limit error",
  DistanceMeasure.STA_ERR_DATA: "Data error"
}

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument("--mqttusername", help="MQTT username")
  parser.add_argument("--mqttpassword", help="MQTT password")
  parser.add_argument("--mqttport", type=int, help="MQTT port", default=1883)
  parser.add_argument("--mqtthost", help="MQTT host")
  parser.add_argument("--verbose", action="store_true", help="Print readings to console")
  args = parser.parse_args()

  hostname = socket.gethostname()
  dts = datetime.datetime.now().replace(microsecond=0).isoformat()

  board = DistanceMeasure()
 
  # set reading range from 10mm to 2000mm.
  board.set_dis_range(10, 2000)

  distance = board.getDistance()
  status = STATUS[board.last_operate_status]

  if args.mqtthost:
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=hostname, userdata=None, protocol=mqtt.MQTTv5)
    client.username_pw_set(args.mqttusername,args.mqttpassword)
    client.connect(args.mqtthost,args.mqttport)

    client.publish("oil/distance_status", payload=status, qos=1)

  if board.last_operate_status == board.STA_OK:
    if args.mqtthost:
      client.publish("oil/distance", payload=distance, qos=1)

    if args.verbose:
      print("%s: Distance %d mm, %s" %(dts, distance, status))
  else:
    if args.verbose:
      print("Status ",status)

#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2016 by DTU                             *
 #*   jca@elektro.dtu.dk                                                    *
 #*
 #* Primary class, that holds all the rest
 #*
 #*   This program is free software; you can redistribute it and/or modify  *
 #*   it under the terms of the GNU General Public License as published by  *
 #*   the Free Software Foundation; either version 2 of the License, or     *
 #*   (at your option) any later version.                                   *
 #*                                                                         *
 #*   This program is distributed in the hope that it will be useful,       *
 #*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 #*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 #*   GNU General Public License for more details.                          *
 #*                                                                         *
 #*   You should have received a copy of the GNU General Public License     *
 #*   along with this program; if not, write to the                         *
 #*   Free Software Foundation, Inc.,                                       *
 #*   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 #***************************************************************************/

import sys 
import os
import threading
import numpy as np
#import pyqtgraph as pg
import serial
import socket
import time
#from time import sleep
import ConfigParser
import timeit
from pyqtgraph.Qt import QtGui, QtCore


from ulog import ULog
from uimu import UImu
from umission import UMission, UMissionLine
from uinfo import UInfo
from ulinesensor import ULineSensor
from uirsensor import UIRDistance
from udrive import UDrive
from uservo import UServo
from control_edit import UControlUnit

CLIENT_REV = "$Id: urobot.py 576 2017-02-01 18:57:39Z jcan $"


class URobot(object):
  con = serial.Serial()
  wificlient = socket.socket()
  # status data
  regbotTime = 0
  msgCnt = 0;
  msgCntLast = 0;
  timeLast = 0;
  timeLastCnt = 0;
  #
  mainSetReset = False
  #
  statusRq = 0 # set status from robot
  #
  mainStatus = ""
  mainStatusSet = False
  dataRxCnt = 0
  dataTxCnt = 0
  #
  timerUpdate = False
  timerCnt = 0
  failCnt = 0
  wifiPort = 24001
  wifiFailCnt = 0
  wifiConnected = False
  wifiTxCnt = 0
  wifiRxCnt = 0
  wifiWaiting4reply = False
  #decodeLock = threading.RLock()
  statusUpdateTime = 0
  statusUpdateIdx = 0;
  # lock to allow timer and main thread to send data to robot
  lock = threading.Lock()
  currentTab = "none"
  #
  lastDataRequestTime = time.time()
  nextDataRequest = 0
  #
  def __init__(self, parent, ui):
    self.parent = parent
    self.ui = ui
  def init(self):
    self.mission = UMission(self)
    self.log = ULog(self)
    self.imu = UImu(self, self.ui)
    self.drive = UDrive(self, self.ui) 
    #self.regTurn = URegTurn(self)
    #self.regVel = URegVel(self)
    #self.regPosition = URegPosition(self)
    #self.regBal = URegBal(self)
    self.ctrlVelocity = UControlUnit("cvel", self.parent, "Wheel Velocity (left and right)")
    self.ctrlTurn = UControlUnit("ctrn", self.parent, "Heading")
    self.ctrlWallVel = UControlUnit("cwve", self.parent, "IR forward distance")
    self.ctrlWallTurn = UControlUnit("cwth", self.parent, "Wall distance")
    self.ctrlPos = UControlUnit("cpos", self.parent, "Position (drive distance)")
    self.ctrlEdge = UControlUnit("cedg", self.parent, "Line edge")
    self.ctrlBalance = UControlUnit("cbal", self.parent, "Balance")
    self.ctrlBalVel = UControlUnit("cbav", self.parent, "Balance velocity")
    self.ctrlVelocity.init()
    self.ctrlTurn.init()
    self.ctrlWallVel.init()
    self.ctrlWallTurn.init()
    self.ctrlPos.init()
    self.ctrlEdge.init()
    self.ctrlBalance.init()
    self.ctrlBalVel.init()
    #regSSBal = URegSSBal()
    self.lineSensor = ULineSensor(self, self.parent)
    self.irDist = UIRDistance(self);
    self.info = UInfo(self)
    self.servo = UServo(self)
    self.stop = threading.Event()
    self.doRead = threading.Thread(target=self.readThread, name="regbot_usb_reader")
    self.doRead.start()
    self.doReadWiFi = threading.Thread(target=self.readThreadWiFi, name="regbot_wifi_reader")
    self.doReadWiFi.start()
  def terminate(self):
    self.stop.set()
    self.close()
    self.doRead.join(2)
    self.wificlient.close()
    self.doReadWiFi.join(2)
  def usbopen(self, name):
    if (not self.con.isOpen()):
      self.con.port = str(name)
      self.con.timeout = 0.5
      self.conWriteTimeout = 0.5
      #print("Trying to open:" + self.con.port)
      try:
        self.con.open()
        print("usb - opened OK.")
        self.mainStatus += "USB Connected OK\n"
        self.mainStatusSet = True
        self.failCnt = 0
        self.justConnected()
      except:
        if self.failCnt < 5:
          print("usb failed to open usb-port :" + str(name))
        self.failCnt += 1
        #self.ui.connect_usb.setChecked(False)
        self.mainStatus += "usb Failed to connect to " + str(name) + "\n"
        self.mainStatusSet = True
    if self.con.isOpen():
      #if not self.ui.connect_usb.isChecked():
      #  self.ui.connect_usb.setChecked(True)
      self.con.flushInput()
      self.con.flushOutput()
      #flags = fcntl(self.con, F_GETFL) # get current p.stdout flags
      #fcntl(self.con, F_SETFL, flags & ~O_NONBLOCK)
      #self.requestData()
      #self.getStatus()
    pass
  
  def justConnected(self):
    # inform tabs about connection
    self.info.justConnected()
  
  def close(self):
    if self.con.isOpen():
      print("stopping push S=0")
      self.conWrite("S=0\n")
      self.con.close()
      self.ui.statusbar.showMessage("usb disconnected", 3000)
      self.mainStatus += "Robot is disconnected\n"
      self.mainStatusSet = True
    pass
  
  def wifiOpen(self):
    ## 
    # must be changed to something like:
    # 
    #for res in socket.getaddrinfo(HOST, PORT, socket.AF_UNSPEC, socket.SOCK_STREAM):
    #af, socktype, proto, canonname, sa = res
    #try:
        #s = socket.socket(af, socktype, proto)
    #except socket.error as msg:
        #s = None
        #continue
    #try:
        #s.connect(sa)
    #except socket.error as msg:
        #s.close()
        #s = None
        #continue
    # 
    # and make a new socket whenever it is closed
    # det går ikke at åbne, lukke og så genåbne
    
    if not self.wifiConnected:
      hopo = self.ui.wifi_host_name.text().split(':')
      if False: # old method
        try:
          self.wifiPort = int(str(hopo[1]), 0)
        except:
          self.wifiPort = 24001
          print("wifi no ':' or port number found in '" + hopo[0] + "'")
        try:
          #hopo = self.ui.wifi_host_name.text().split(':')
          self.wificlient.connect((hopo[0], self.wifiPort))
          print("wifi connect did not fail")
          self.wifiFailCnt = 0
          self.wifiConnected = True
          if not self.con.isOpen():
            self.justConnected()
        except:
          #if (self.wifiFailCnt < 3):
          print("wifi Failed to connect to " + hopo[0] + ", port " + str(self.wifiPort))
          self.wifiFailCnt += 1
          pass
      else: # new test
        print("# wifi open: hopo=" + str(hopo[0]) + " " + str(self.wifiPort))
        for res in socket.getaddrinfo(str(hopo[0]), self.wifiPort, socket.AF_UNSPEC, socket.SOCK_STREAM):
          print("# socket res " + str(res))
          af = res[0]
          socktype = res[1] 
          proto = res[2]
          canonname = res[3]
          sa = res[4] # both IP and port number
          try:
              self.wificlient = socket.socket(af, socktype, proto)
              self.wificlient.settimeout(5)
          except OSError as msg:
              self.wificlient = None
              print("# wifi connection timeout - retry")
              continue
          try:
              self.wificlient.connect(sa)
              self.wifiConnected = True
          except OSError as msg:
              self.wificlient.close()
              self.wificlient = None
              continue
          except:
            print("# wifi other except")
          break
        pass
    if self.wifiConnected:
      if not self.con.isOpen():
        self.justConnected()
      print("wifi is open")
      self.ui.statusbar.showMessage("wifi client - connected", 2000)
      self.mainStatus += "wifi is connected\n"
      self.mainStatusSet = True
      self.wifiWaiting4reply = False
      pass
    pass
  
  def wifiClose(self):
    if (self.wifiConnected):
      print("wifi stopping")
      self.wifiConnected = False
      self.wificlient.close()
      self.ui.statusbar.showMessage("wifi client - disconnected", 2000)
      self.mainStatus += "wifi is disconnected\n"
      self.mainStatusSet = True
      self.wifiWaiting4reply = False
    pass

  def connect_usb_changed(self):
    if self.ui.connect_usb.isChecked():
      #print("trying to open connection to regbot") 
      if (not self.isConnected()):
        name = self.ui.usb_device_name.text()
        self.usbopen(name)
        if self.isConnected():
          self.ui.statusbar.showMessage("Robot client - connected", 2000)
        else:
          self.ui.statusbar.showMessage("Robot client starting - not connected", 2000)
      pass
    else:
      if (self.isConnected()):
        print("usb closing connection to regbot") 
        self.close()
    pass

  def connect_wifi_changed(self):
    if self.ui.connect_wifi.isChecked():
      print("trying to open connection to regbot") 
      if not self.wifiConnected:
        self.wifiOpen()
        if self.wifiConnected:
          self.ui.statusbar.showMessage("wifi client - connected", 2000)
        else:
          self.ui.statusbar.showMessage("wifi client starting - not connected", 2000)
      pass
    else:
      if self.wifiConnected:
        print("wifi closing connection to regbot") 
        self.wifiClose()
    pass
  #  
  def isConnected(self):
    return self.con.isOpen()
  #
  # use this in place of con.write
  def conWrite(self, s):
    self.lock.acquire()
    isSend = False
    if (self.con.isOpen()):
      #print("# about to send to USB : " + s)
      self.usbWrite(s)
      #print("# just after to send to USB")
      isSend = self.con.isOpen()
      #print("# justafter isSend() " + str(isSend))
    elif self.wifiConnected:
      # debug
      #print("# sending to wifi " + s)
      # debug end
      self.wifiWrite(s)
      isSend = self.wifiConnected
    if (isSend):
      self.dataTxCnt += 1 #len(s)
      if (self.ui.main_show_all_tx.isChecked()):
        self.mainStatus += str(s)
        self.mainStatusSet = True
    else:
      self.mainStatus += "not connected, could not send " + s
      self.mainStatusSet = True
    self.lock.release();
    pass
  #
  ### send string to USB connection
  def usbWrite(self, s):
    if self.con.isOpen():
      n = len(s)
      #print("# sending " + s)
      if (n > 0):
        try:
          n = self.con.write(s)
          #print("# USB write returned " + str(n) + " bytes send")
          if (n == 0):
            raise Exception("Write error")
        except:
          self.con.close()
          print("URobot conWrite - closed connection")
          self.ui.statusbar.showMessage("Robot usb - connection broken", 2000)
    pass
  #
  ### send string to wifi socket
  def wifiWrite(self, s):
    n = len(s)
    #print("# sending " + s)
    if (n > 0):
      m = 0;
      try:
        while m < n:
          d = self.wificlient.send(s[m:])
          if (d == 0):
            raise Exception("Write error")
          m += d
          self.wifiTxCnt += 1
          self.wifiWaiting4reply = True
      except:
        self.wifiClose()
        print("wifi - closed connection (write fail)")
        self.ui.statusbar.showMessage("wifi connection broken")
    #print("# send " + s)
    pass
  #
  ### request data from robot
  #def requestData(self):
    ##print("trying to send S=" + str(self.ui.main_push_interval.value()))
    #self.conWrite("i=0\r\n") # turn off interactive mode
    #self.conWrite("u0\r\n") # request software version
    #self.conWrite("u4\r\n") # request static robot info
    ## request regular status updates
    #self.conWrite("S=" + str(self.ui.main_push_interval.value()) + "\r\n")
  #
  ### interpret data from robot
  def decodeCommand(self, got, n):
    if n > 0:
      #print(got)
      self.dataRxCnt += n
      if (self.ui.main_show_all_rx.isChecked()):
        self.mainStatus += got
        self.mainStatusSet = True
    #if (got[0] == '<'):
      #print("fik " + str(n) + " bytes: " + got)
    if n > 3:
      self.msgCnt += 1
      gg = got.split()
      if (self.mission.readUserMissionLine(got)):
        # read user defined mission lines
        # print("got mission line - now " + str(len(self.mission.mLines)) + " lines in mLines")
        pass
      elif self.imu.readData(gg):
        pass
      elif gg[0] == "hbt":
        self.regbotTime = float(gg[1])
        if len(gg) > 2:
          # battery value comes here too - else in drive group
          self.drive.battery = float(gg[2])
          self.drive.dataReadBat = True
      elif self.drive.readData(gg):
        pass
      elif self.ctrlVelocity.fromString(gg):
        pass
      elif self.ctrlTurn.fromString(gg):
        pass
      elif self.ctrlWallVel.fromString(gg):
        pass
      elif self.ctrlWallTurn.fromString(gg):
        pass
      elif self.ctrlPos.fromString(gg):
        pass
      elif self.ctrlEdge.fromString(gg):
        pass
      elif self.ctrlBalance.fromString(gg):
        pass
      elif self.ctrlBalVel.fromString(gg):
        pass
      elif self.mission.readData(gg, got):
        pass
      elif self.info.readData(gg):
        pass
      elif self.log.readData(gg, got):
        pass
      elif self.lineSensor.readData(gg):
        pass
      elif self.irDist.readData(gg):
        pass
      elif self.servo.readData(gg, got):
        pass
      else:
        if (not self.ui.main_show_all_rx.isChecked()):
          self.mainStatus += got
          self.mainStatusSet = True
    pass


  def readThread(self):
    count = 0
    m = 0
    n = 0
    c = '\0'
    self.threadRunning = True
    print("thread running")
    got = ""
    while (not self.stop.is_set()):
      if self.con.isOpen():
        m = m + 1
        n = 0
        if (c == '\n'):
          got = ""
        c = '\0'
        try:
          while (c != '\n'):
            c = self.con.read(1)
            if (c >= ' ' or c == '\n'):
              got = got + c
          #print("got (" + str(m) + ")=" + got)
          n = len(got)
        except:
          ##print("read " + str(m) + " returned 0 bytes - " + str(n) + "- closing")
          m = m + 1
          time.sleep(0.01)
        self.decodeCommand(got, n)
      else:
        time.sleep(0.1)
    print("read thread ended")
    self.threadRunning = False
    
  def readThreadWiFi(self):
    count = 0
    m = 0
    n = 0
    c = '\0'
    self.threadRunningWiFi = True
    print("thread wifi running")
    got = ""
    self.wificlient.settimeout(0.5)
    while (not self.stop.is_set()):
      if self.wifiConnected:
        m = m + 1
        n = 0
        if (c == '\n'):
          got = ""
        c = '\0'
        try: 
          while (c != '\n' and self.wifiConnected):
            c = self.wificlient.recv(1)
            if (len(c) > 0):
              if (c >= ' ' or c == '\n'):
                # filter all control characters but newline
                got = got + c
            #else:
              #print("-no data received - OK? (" + c + ")")
          #print("got (" + str(m) + ")=" + got)
          n = len(got)
        except:
          #print("read error " + str(m) + " returned " + str(n) + " bytes")
          m = m + 1
          #self.wifiWaiting4reply = False
          time.sleep(0.01)
        if (n > 0):
          #print("# got (" + str(m) + ", len=" + str(n) + ")=" + got)
          self.wifiRxCnt += 1
          self.wifiWaiting4reply = False
          self.decodeCommand(got, n)
        # tell wifi page that data is coming from wifi source
        self.info.gotAliveMsg();
      else:
        time.sleep(0.1)
    print("read thread ended")
    self.threadRunning = False
    pass
  #
  ### 
  # start a mission
  def mainSettingStart(self):
    self.conWrite("start\r\n")
  #
  ### stop a mission
  def mainSettingStop(self):
    self.conWrite("stop\r\n")
  #
    
  def robotIdApply(self):
    # print("robotIDapply clicked")
    self.conWrite("rid=%d %g %g %d %g %g %g %d %g %d\n" % (
      self.ui.robot_id.value(),
      self.ui.robot_base.value(),
      self.ui.robot_gear.value(),
      self.ui.robot_pulse_per_rev.value(),
      self.ui.robot_wheel_radius_left.value(),
      self.ui.robot_wheel_radius_right.value(),
      self.ui.robot_balance_offset.value(),
      self.ui.robot_on_battery.isChecked(),
      self.ui.robot_battery_idle_volt.value(),
      self.ui.robot_hw_type.value()
      ))
    self.info.inEdit = False
    #self.ui.save_id_on_robot.setEnabled(False)
  def doGyroOffset(self):
    self.ui.imu_gyro_offset_done.setChecked(False)
    self.conWrite("gyroo\n")
    
  def mainStatusClear(self):
    self.mainStatus = ""
    self.mainStatusSet = True;
    self.dataRxCnt = 0
    self.dataTxCnt = 0
  def mainSettingHelp(self):
    print("help clicked")
    self.conWrite("h\n")    
    
  ## timer update to update display
  # called every 50 ms
  def timerUpdate(self):
    self.timerCnt += 1
    if (self.timerCnt % 4 == 0):
      # ensure tab focus is not none
      if self.currentTab == "none":
        self.parent.tabPageFocusChanged()
      #self.conWrite("u1\n")
      pass
      #print("# requested data update using timer function in urobot.py")
    self.timerUpdate = True
    #self.regTurn.showData()
    # request data
    if self.nextDataRequest < 15:
      # get initial data and settings
      if self.nextDataRequest == 1:
        self.conWrite("i=0\n")
      elif self.nextDataRequest == 3:
        self.conWrite("u0\n")
      elif self.nextDataRequest == 5:
        self.conWrite("u2\n")
      elif self.nextDataRequest == 7:
        self.conWrite("S=0\n")
      elif self.nextDataRequest == 9:
        self.conWrite("u4\n")
      self.nextDataRequest += 1
    elif (time.time() - self.lastDataRequestTime) > 0.95:
      if (self.isConnected() or self.wifiConnected):
        self.conWrite("u5\n") # heartbeat
        self.lastDataRequestTime = time.time()
    else:
      self.mission.timerUpdate()
      self.log.timerUpdate()
      self.imu.timerUpdate()
      self.drive.timerUpdate()
      self.lineSensor.timerUpdate()
      self.irDist.timerUpdate()
      self.info.timerUpdate()
      self.servo.timerUpdate()
    #
    self.ctrlVelocity.timerUpdate()
    self.ctrlTurn.timerUpdate()
    self.ctrlWallVel.timerUpdate()
    self.ctrlWallTurn.timerUpdate()
    self.ctrlPos.timerUpdate()
    self.ctrlEdge.timerUpdate()
    self.ctrlBalance.timerUpdate()
    self.ctrlBalVel.timerUpdate()
    #
    #self.requestStatusUpdate()
    #
    # update overall 
    if (self.mainStatusSet):
      self.mainStatusSet = False
      if (len(self.mainStatus) > 17000):
        self.mainStatus = "# status truncated\n"
      self.ui.main_status.setPlainText(str(self.mainStatus))
      self.ui.main_status.verticalScrollBar().setValue(self.ui.main_status.verticalScrollBar().maximum())
      self.ui.Main_status_log.setPlainText(str(self.mainStatus))
      self.ui.Main_status_log.verticalScrollBar().setValue(self.ui.Main_status_log.verticalScrollBar().maximum())
      #self.ui.main_status.ensureCursorVisible()
    self.ui.main_rxCnt.setValue(self.dataRxCnt)
    self.ui.main_txCnt.setValue(self.dataTxCnt)
    if (self.ui.connect_usb.isChecked() and not self.con.isOpen()):
      #self.ui.connect_usb.setChecked(False)
      self.ui.statusbar.showMessage("Connection closed - e.g. no heart beat", 2000)
      #self.ui.usb_port_label.setText('Robot connection')
    if self.mission.missionManually and self.con.isOpen():
      self.conWrite("M=" + str(self.ui.main_mission_2.value()) + "\n")
      self.mission.missionManually = False
    self.ui.main_time.setValue(self.regbotTime)
    self.timerUpdate = False
    if (self.regbotTime != self.timeLast or self.msgCnt != self.msgCntLast):
      # is the connection alive - based on heartbeat or other messages
      self.timeLastCnt = 0
      self.msgCntLast = self.msgCnt
      self.timeLast = self.regbotTime
      if (not self.ui.frame_batt_time.isEnabled()):
        self.ui.frame_batt_time.setEnabled(True)
        print('Got active communication with robot ' + self.ui.robot_id_main.text())
        self.mainStatus += 'Robot ' + self.ui.robot_id_main.text() + " active\n"
        self.mainStatusSet = True;
      self.ui.frame_usb_connect.repaint() # setEnabled(self.isConnected())
      self.ui.frame_wifi_connect.repaint() # Enabled(self.wifiConnected and not self.isConnected())
    elif (self.con.isOpen() or  self.wifiConnected):
      self.timeLastCnt = self.timeLastCnt + 1
      #print("--    connection count=" + str(self.timeLastCnt))
      if (self.timeLastCnt >= 30):
        if (self.ui.frame_batt_time.isEnabled()):
          self.ui.frame_batt_time.setEnabled(False)
          print('No  active communication with robot ' + self.ui.robot_id_main.text())
          self.mainStatus += 'Robot ' + self.ui.robot_id_main.text() + " silent\n"
          self.mainStatusSet = True
        if (self.timeLastCnt >= 40):
          self.con.close()
          self.timeLastCnt = 0;
      self.ui.frame_usb_connect.repaint() # setEnabled(self.isConnected())
      self.ui.frame_wifi_connect.repaint() # Enabled(self.wifiConnected and not self.isConnected())
      pass
    else:
      # no connection, so no timeout
      self.timeLastCnt = self.timeLastCnt + 1
      #print("-- no connection count=" + str(self.timeLastCnt))
      if self.ui.connect_usb.isChecked():
        # try autoconnect
        if self.timeLastCnt >= 100:
          # try if open works now
          self.usbopen(self.ui.usb_device_name.text())
          self.timeLastCnt = 0
      # make sure to show
      if (self.ui.frame_batt_time.isEnabled()):
        self.ui.frame_batt_time.setEnabled(False)
    self.timeLast = self.regbotTime
    pass
        

  def sendCmd(self):
    s = self.ui.main_send_txt.text() + "\n"
    self.conWrite(s.toAscii())
    #print("Urobot: send " + s + str(len(s)))
  def mainMissionChanged2(self):
    self.missionChanged(self.ui.main_mission_2.value())
  def missionChanged(self, m):
    if (self.con.isOpen()):
      self.conWrite("M=" + str(m) + "\n")
  #def mainPushInterval(self):
    #self.conWrite("S=" + str(self.ui.main_push_interval.value()) + "\n")
  def robotPoseReset(self):
    self.conWrite("posec\n")
    self.drive.poseReset()
  def logGet(self):
    self.conWrite("log get\n")
  def setLogFlag_lac(self):
    self.setLogFlag("acc", self.ui.log_lac.isChecked())
  def setLogFlag_lbt(self):
    self.setLogFlag("bat", self.ui.log_lbt.isChecked())
  def setLogFlag_line(self):
    self.setLogFlag("line", self.ui.log_line.isChecked())
  def setLogFlag_dist(self):
    self.setLogFlag("dist", self.ui.log_distance.isChecked())
  def setLogFlag_lct(self):
    self.setLogFlag("ct", self.ui.log_lct.isChecked())
  def setLogFlag_lgy(self):
    self.setLogFlag("gyro", self.ui.log_lgy.isChecked())
  def setLogFlag_lma(self):
    self.setLogFlag("motA", self.ui.log_lma.isChecked())
  def setLogFlag_lme(self):
    self.setLogFlag("enc", self.ui.log_lme.isChecked())
  def setLogFlag_lmr(self):
    self.setLogFlag("mvel", self.ui.log_lmr.isChecked())
  def setLogFlag_lms(self):
    self.setLogFlag("mis", self.ui.log_lms.isChecked())
  def setLogFlag_lmv(self):
    self.setLogFlag("motV", self.ui.log_lmv.isChecked())
  def setLogFlag_lvr(self):
    self.setLogFlag("motR", self.ui.log_lvr.isChecked())
  def setLogFlag_ltr(self):
    self.setLogFlag("tr", self.ui.log_turn_rate.isChecked())
  def setLogFlag_lex(self):
    self.setLogFlag("extra", self.ui.log_lex.isChecked())
  #def setLogFlag_lbc(self):
    #self.setLogFlag("bcl", self.ui.log_lbc.isChecked())
  def setLogFlag_lpo(self):
    self.setLogFlag("pose", self.ui.log_lpo.isChecked())
  def setLogFlag(self, flag, checked):
    if (not self.timerUpdate):
      c = '-'
      if (checked):
        c = '+'
      self.conWrite("log" + c + flag + "\n")
      # request update from robot
      self.conWrite("u3\n")
  def setLogFlagControl(self):
    if (not self.timerUpdate):
      self.conWrite("lcl {} {} {} {} {} {} {} {}\n".format(
                    int(self.ui.log_ctrl_vel.isChecked()),
                    int(self.ui.log_ctrl_turn.isChecked()),
                    int(self.ui.log_ctrl_pos.isChecked()),
                    int(self.ui.log_ctrl_edge.isChecked()),
                    int(self.ui.log_ctrl_wall.isChecked()),
                    int(self.ui.log_ctrl_fwd_dist.isChecked()),
                    int(self.ui.log_ctrl_bal.isChecked()),
                    int(self.ui.log_ctrl_bal_vel.isChecked())
                    ))
      pass
  def setLogInterval(self):
    if (not self.timerUpdate):
      self.conWrite("s=" + str(self.ui.log_interval.value()) + " 1\n")
      self.conWrite("u3\n")
      
  def setLogFileName(self):
    filename = QtGui.QFileDialog.getSaveFileName(self.parent,'logfile to use', '', 'log (*.txt)')
    if (filename is not None and len(filename) > 0):
      print("saved log to '" + filename + "'")
      self.ui.log_filename.setText(filename)
      self.logSave()
  def logSave(self):
    if (self.ui.log_save_config.isChecked()):
      self.configurationFileSave('.regbotConfigTemp.ini', False)
    self.log.logSave()
    
  #def mainStatusStop(self):
    #self.ui.main_push_interval.setValue(0)
    
  #def mainStatusStart(self):
    #self.ui.main_push_interval.setValue(50)
    
  def configurationFileSaveDef(self):
    ret = QtGui.QMessageBox.Save
    if (not (self.con.isOpen() or self.wifiConnected)):
      msgBox = QtGui.QMessageBox(self.parent)
      msgBox.setText("The robot is not connected and some configuration values may have default value!")
      msgBox.setInformativeText("Do you want to save all parameters to regbot.ini anyhow?")
      msgBox.setStandardButtons(QtGui.QMessageBox.Save | QtGui.QMessageBox.Cancel)
      msgBox.setDefaultButton(QtGui.QMessageBox.Save)
      ret = msgBox.exec_()
    if ret == QtGui.QMessageBox.Save:
      # Save was clicked
      if (self.isConnected() or self.wifiConnected):
        # request data - takes some time, if on wifi
        if (self.isConnected()):
          waitTime = 0.05;
        else:
          waitTime = 0.5;
        self.ui.statusbar.showMessage("Getting data .", 500)
        tab = self.ui.tabPages.currentIndex()
        for t in range(0, self.ui.tabPages.count()):
          self.ui.tabPages.setCurrentIndex(t)
          # wait for tab info to be filled
          time.sleep(waitTime)
        # also all the control data
        self.ctrlBalance.requestDataFromRobot()
        self.ui.statusbar.showMessage("Getting data ..", 500)
        time.sleep(waitTime)
        self.ctrlBalVel.requestDataFromRobot()
        self.ui.statusbar.showMessage("Getting data ..", 500)
        time.sleep(waitTime)
        self.ctrlEdge.requestDataFromRobot()
        self.ui.statusbar.showMessage("Getting data ...", 500)
        time.sleep(waitTime)
        self.ctrlPos.requestDataFromRobot()
        self.ui.statusbar.showMessage("Getting data ....", 500)
        time.sleep(waitTime)
        self.ctrlTurn.requestDataFromRobot()
        self.ui.statusbar.showMessage("Getting data .....", 500)
        time.sleep(waitTime)
        self.ctrlVelocity.requestDataFromRobot()
        self.ui.statusbar.showMessage("Getting data .......", 500)
        time.sleep(waitTime)
        self.ctrlWallTurn.requestDataFromRobot()
        self.ui.statusbar.showMessage("Getting data ........", 500)
        time.sleep(waitTime)
        self.ctrlWallVel.requestDataFromRobot()
        self.ui.statusbar.showMessage("Getting data .........", 500)
        time.sleep(waitTime)
        self.mission.clearRxField()
        self.conWrite("<get\n")
        self.ui.statusbar.showMessage("Getting data ..........", 500)
        time.sleep(waitTime)
        self.ui.tabPages.setCurrentIndex(tab)
      # try save to file
      try:
        self.configurationFileSave('regbot.ini', True)    
        self.ui.statusbar.showMessage("Saved configuration to regbot.ini", 3000)
      except:
        self.ui.statusbar.showMessage("Save configuration to regbot.ini - failed", 3000)
  def configurationFileLoadDef(self, bootload):
    try:
      self.configurationFileLoad('regbot.ini', bootload)
      self.ui.statusbar.showMessage("Loaded configuration from regbot.ini", 3000)
    except:
      self.ui.statusbar.showMessage("Load configuration from regbot.ini - failed", 3000)

  def configurationFileSaveAs(self):
    filename = QtGui.QFileDialog.getOpenFileName(self.parent,'save configuration as', os.getcwd(), 'data (*.ini)')
    if (filename is not None  and len(filename) > 0):
      self.configurationFileSave(filename, True)
      print("saved config to " + filename)
  def configurationFileLoadFrom(self):
    filename = QtGui.QFileDialog.getOpenFileName(self.parent,'configuration file to open', '', 'data (*.ini)')
    if (filename is not None and len(filename) > 0):
      self.configurationFileLoad(filename, False)
      print("Loaded configuration from " + filename)
  def configurationFileSave(self, filename, verbose):
    # create configuration to save
    config = ConfigParser.SafeConfigParser()
    config.add_section('main')
    config.set('main', 'version', self.clientVersion())
    config.set('main', 'mission', str(self.ui.main_mission_2.value()))
    #config.set('main', 'statusInterval', str(self.ui.main_push_interval.value()))
    config.set('main', 'usbDeviceName', str(self.ui.usb_device_name.text()))
    config.set('main', 'connect', str(self.ui.connect_usb.isChecked()))
    # wifi connect
    config.set('main', 'wifiHost', str(self.ui.wifi_host_name.text()))
    config.set('main', 'wifiConnect', str(self.ui.connect_wifi.isChecked()))
    config.set('main', 'wifiport', str(self.ui.wifi_port.text()))
    # show tabs
    config.add_section('show')
    config.set('show', 'debug', str(self.ui.actionDebug.isChecked()))
    config.set('show', 'wifi', str(self.ui.actionWifi.isChecked()))
    config.set('show', 'Log', str(self.ui.actionLog.isChecked()))
    config.set('show', 'Robot', str(self.ui.actionRobot.isChecked()))
    config.set('show', 'edge', str(self.ui.actionLine.isChecked()))
    config.set('show', 'distance', str(self.ui.actionDistance.isChecked()))
    config.set('show', 'IMU', str(self.ui.actionIMU.isChecked()))
    config.set('show', 'control', str(self.ui.actionControl.isChecked()))
    config.set('show', 'mission', str(self.ui.actionMission.isChecked()))
    config.set('show', 'servo', str(self.ui.actionServo.isChecked()))
    # log settings
    config.add_section('log')
    config.set('log', 'interval', str(self.ui.log_interval.value()))
    config.set('log', 'mission_state', str(self.ui.log_lms.isChecked()))
    config.set('log', 'Acceleration', str(self.ui.log_lac.isChecked()))
    config.set('log', 'Gyro', str(self.ui.log_lgy.isChecked()))
    config.set('log', 'Encoder', str(self.ui.log_lme.isChecked()))
    config.set('log', 'motorRef', str(self.ui.log_lvr.isChecked()))
    config.set('log', 'motorVoltage', str(self.ui.log_lmv.isChecked()))
    config.set('log', 'motorCurrent', str(self.ui.log_lma.isChecked()))
    config.set('log', 'wheelVelocity', str(self.ui.log_lmr.isChecked()))
    config.set('log', 'robotPose', str(self.ui.log_lpo.isChecked()))
    config.set('log', 'lineSensor', str(self.ui.log_line.isChecked()))
    config.set('log', 'irdist', str(self.ui.log_distance.isChecked()))
    config.set('log', 'turnrate', str(self.ui.log_turn_rate.isChecked()))
    config.set('log', 'batteryVoltage', str(self.ui.log_lbt.isChecked()))
    #
    config.set('log', 'ctrl_vel', str(self.ui.log_ctrl_vel.isChecked()))
    config.set('log', 'ctrl_turn', str(self.ui.log_ctrl_turn.isChecked()))
    config.set('log', 'ctrl_pos', str(self.ui.log_ctrl_pos.isChecked()))
    config.set('log', 'ctrl_edge', str(self.ui.log_ctrl_edge.isChecked()))
    config.set('log', 'ctrl_wall', str(self.ui.log_ctrl_wall.isChecked()))
    config.set('log', 'ctrl_fwd_dist', str(self.ui.log_ctrl_fwd_dist.isChecked()))
    config.set('log', 'ctrl_bal', str(self.ui.log_ctrl_bal.isChecked()))
    config.set('log', 'ctrl_bal_vel', str(self.ui.log_ctrl_bal_vel.isChecked()))
    #config.set('log', 'balanceCtrl', str(self.ui.log_lbc.isChecked()))
    config.set('log', 'controlTime', str(self.ui.log_lct.isChecked()))
    config.set('log', 'extraInfo', str(self.ui.log_lex.isChecked()))
    config.set('log', 'logFilename', str(self.ui.log_filename.text()))
    config.set('log', 'allow', str(True)) # str(self.ui.log_allow.isChecked()))
    # new velocity control
    self.ctrlVelocity.configurationFileSave(config)
    self.ctrlTurn.configurationFileSave(config)
    self.ctrlWallVel.configurationFileSave(config)
    self.ctrlWallTurn.configurationFileSave(config)
    self.ctrlPos.configurationFileSave(config)
    self.ctrlEdge.configurationFileSave(config)
    self.ctrlBalance.configurationFileSave(config)
    self.ctrlBalVel.configurationFileSave(config)
    # mission
    config.add_section('mission')
    if (self.isConnected() or self.wifiConnected):
      config.set('mission', 'mission', self.mission.missionTextMsg)
    else:
      config.set('mission', 'mission', self.mission.missionTextEdit)
    #config.set('mission', 'filename', str(self.ui.mission_filename.text()))
    # line sensor page
    config.add_section('lineSensor')
    config.set('lineSensor', 'maximum', str(self.ui.line_disp_max_value.value()))
    config.set('lineSensor', 'use', str(self.ui.ls_use_sensor.isChecked()))
    config.set('lineSensor', 'white', str(self.ui.ls_line_white.isChecked()))
    config.set('lineSensor', 'LEDpower', str(self.ui.ls_power_high.isChecked()))
    config.set('lineSensor', 'LEDpowerAuto', str(self.ui.ls_power_auto.isChecked()))
    # robot specifics
    for rb in self.info.robots:
      print("Saving data for robot (" + str(rb.robotID) + ") " + rb.name)
    for rb in self.info.robots :
      idstr = "robot" + str(rb.robotID)
      if rb.robotID == int(self.ui.robot_id.value()) and self.ui.save_id_on_robot.isEnabled():
        # some data may have changed
        rb.wheelbase =  self.ui.robot_base.value()
        rb.gear = self.ui.robot_gear.value()
        rb.pulsePerRev = self.ui.robot_pulse_per_rev.value()
        rb.wheelLRadius = self.ui.robot_wheel_radius_left.value()
        rb.wheelRRadius = self.ui.robot_wheel_radius_right.value()
        rb.balanceOffset = self.ui.robot_balance_offset.value()
        rb.batteryUse = self.ui.robot_on_battery.isChecked()
        rb.batteryIdleVolt = self.ui.robot_battery_idle_volt.value()
        rb.wifiIP = self.ui.wifi_ip.text()
        rb.wifiMAC = self.ui.wifi_mac.text()
        #rb.wifiport = self.ui.wifi_port.text()
        #print("save values on screen")
      #
      #print("Adding section for " + idstr + ", wr1: " + str(rb.wheelRRadius))
      try:
        config.add_section(idstr)
      except:
        print("error - exist already: " + idstr)
        pass
      config.set(idstr, 'name', rb.name)
      config.set(idstr, 'version', rb.version)
      config.set(idstr, 'robotBase', str(rb.wheelbase))
      #print("writeing wheelbase = " + str(rb.wheelbase) + " robot " + str(rb.robotID))
      config.set(idstr, 'gear', str(rb.gear))
      config.set(idstr, 'encoderPPR', str(rb.pulsePerRev))
      config.set(idstr, 'wheelRadiusLeft', str(rb.wheelLRadius))
      #print("writeing wheelradius 0 = " + str(rb.wheelLRadius) + " robot " + str(rb.robotID))
      config.set(idstr, 'wheelRadiusRight', str(rb.wheelRRadius))
      #print("writeing wheelradius 1 = " + str(rb.wheelRRadius) + " robot " + str(rb.robotID))
      config.set(idstr, 'balanceOffset', str(rb.balanceOffset))
      config.set(idstr, 'batteryUse', str(rb.batteryUse))
      config.set(idstr, 'batteryIdleVolt', str(rb.batteryIdleVolt))
      config.set(idstr, 'wifiIP', str(rb.wifiIP))
      config.set(idstr, 'wifiMAC', str(rb.wifiMAC))
      #config.set(idstr, 'wifiport', str(rb.wifiport))
      pass
    # override with current robot
    #if (self.ui.robot_id.value() > 0):
      #idstr = "robot" + str(int(self.ui.robot_id.value()))
      #config.add_section(idstr)
      #config.set(idstr, 'version', str(self.info.version))
      #config.set(idstr, 'robotBase', str(self.ui.robot_base.value()))
      #config.set(idstr, 'gear', str(self.ui.robot_gear.value()))
      #config.set(idstr, 'encoderPPR', str(self.ui.robot_pulse_per_rev.value()))
      #config.set(idstr, 'weelRadiusLeft', str(self.ui.robot_wheel_radius_left.value()))
      #config.set(idstr, 'weelRadiusRight', str(self.ui.robot_wheel_radius_right.value()))
      #config.set(idstr, 'balanceOffset', str(self.ui.robot_balance_offset.value()))
    # rename old file first
    if os.path.exists(filename):
      newname = 'regbot_' + time.strftime("%Y%2m%2d_%02H%2M%2S") + '.ini'
      #t = time.gmtime()
      #newname = 'regbot_' + str(t.tm_year) + str(t.tm_mon) + str(t.tm_mday) + "_" + str(t.tm_hour) + str(t.tm_min) + str(t.tm_sec) + '.ini'
      if (verbose):
        print("Renamed " + filename + " to " + newname)
      os.rename(filename, newname)
    # save new file
    with open(filename, 'wb') as configFile:
      config.write(configFile)
      if (verbose):
        print("Created new " + filename)
    #finished
    pass
  #
  # Load configuration from configuration file
  #
  def configurationFileLoad(self, filename, bootload):
    config = ConfigParser.SafeConfigParser()
    config.read(filename)
    print("config - reading " + filename)
    try:
      self.ui.main_mission_2.setValue(config.getint('main', 'mission'))
      #self.ui.main_push_interval.setValue(config.getint('main', 'statusInterval'))
      self.ui.usb_device_name.setText(config.get('main', 'usbDeviceName'))
      self.ui.connect_usb.setChecked(config.getboolean('main', 'connect'))
      self.connect_usb_changed()
      try:
        self.ui.wifi_host_name.setText(config.get('main', 'wifiHost'))
        self.ui.connect_wifi.setChecked(config.getboolean('main', 'wifiConnect'))
        self.ui.wifi_port.setText(config.get('main','wifiport'))
        self.connect_wifi_changed()
      except:
        pass
      #
      if bootload:
        # load visible tab list during boot only
        try:
          self.ui.actionDebug.setChecked(config.getboolean('show', 'debug'))
          self.ui.actionWifi.setChecked(config.getboolean('show', 'wifi'))
          self.ui.actionLog.setChecked(config.getboolean('show', 'Log'))
          self.ui.actionRobot.setChecked(config.getboolean('show', 'Robot'))
          self.ui.actionIMU.setChecked(config.getboolean('show', 'IMU'))
          self.ui.actionLine.setChecked(config.getboolean('show', 'edge'))
          self.ui.actionDistance.setChecked(config.getboolean('show', 'distance'))
          self.ui.actionControl.setChecked(config.getboolean('show', 'control'))
          self.ui.actionServo.setChecked(config.getboolean('show', 'servo'))
          self.ui.actionMission.setChecked(config.getboolean('show', 'mission'))
        except:
          print("# failed to load show/hide tab settings")
        # implement config actions
        self.parent.menuShowDebug()
        self.parent.menuShowWiFi()
        self.parent.menuShowLog()
        self.parent.menuShowRobot()
        self.parent.menuShowIMU()
        self.parent.menuShowLine()
        self.parent.menuShowDist()
        self.parent.menuShowControl()
        self.parent.menuShowMission()
        self.parent.menuShowServo()
        print("menu options set")
      # log options
      try:
        self.ui.log_interval.setValue(config.getint('log', 'interval'))
        self.ui.log_lms.setChecked(config.getboolean('log', 'mission_state'))
        self.ui.log_lac.setChecked(config.getboolean('log', 'Acceleration'))
        self.ui.log_lgy.setChecked(config.getboolean('log', 'gyro'))
        self.ui.log_lme.setChecked(config.getboolean('log', 'encoder'))
        self.ui.log_lvr.setChecked(config.getboolean('log', 'motorRef'))
        self.ui.log_lmv.setChecked(config.getboolean('log', 'motorVoltage'))
        self.ui.log_lma.setChecked(config.getboolean('log', 'motorCurrent'))
        self.ui.log_lmr.setChecked(config.getboolean('log', 'wheelVelocity'))
        self.ui.log_lpo.setChecked(config.getboolean('log', 'robotPose'))
        self.ui.log_line.setChecked(config.getboolean('log', 'lineSensor'))
        self.ui.log_distance.setChecked(config.getboolean('log', 'irdist'))
        self.ui.log_turn_rate.setChecked(config.getboolean('log', 'turnrate'))
        self.ui.log_lbt.setChecked(config.getboolean('log', 'batteryVoltage'))
        #self.ui.log_lbc.setChecked(config.getboolean('log', 'balanceCtrl'))
        self.ui.log_lct.setChecked(config.getboolean('log', 'controlTime'))
        self.ui.log_lex.setChecked(config.getboolean('log', 'extraInfo'))
        self.ui.log_filename.setText(config.get('log', 'logFilename'))
        #
        self.ui.log_ctrl_vel.setChecked(config.getboolean('log','ctrl_vel'))
        self.ui.log_ctrl_turn.setChecked(config.getboolean('log','ctrl_turn'))
        self.ui.log_ctrl_pos.setChecked(config.getboolean('log','ctrl_pos'))
        self.ui.log_ctrl_edge.setChecked(config.getboolean('log','ctrl_edge'))
        self.ui.log_ctrl_wall.setChecked(config.getboolean('log','ctrl_wall'))
        self.ui.log_ctrl_fwd_dist.setChecked(config.getboolean('log','ctrl_fwd_dist'))
        self.ui.log_ctrl_bal.setChecked(config.getboolean('log','ctrl_bal'))
        self.ui.log_ctrl_bal_vel.setChecked(config.getboolean('log','ctrl_bal_vel'))
        #self.ui.log_allow.setChecked(config.getboolean('log', 'allow'))
      except:
         print("# load of logging option failed (mis-spelled?)")
      #print("vel")
      self.ctrlVelocity.configurationFileLoad(config)
      self.ctrlTurn.configurationFileLoad(config)
      self.ctrlWallVel.configurationFileLoad(config)
      self.ctrlWallTurn.configurationFileLoad(config)
      self.ctrlPos.configurationFileLoad(config)
      self.ctrlEdge.configurationFileLoad(config)
      self.ctrlBalance.configurationFileLoad(config)
      self.ctrlBalVel.configurationFileLoad(config)
      #print("mission start")
      # mission
      m = config.get('mission', 'mission')
      indent = ""
      for ml in m.splitlines():
        if ml[0] == '#':
          self.mission.missionTextEdit += indent + ml + '\n'
        elif  "thread" in ml:
          self.mission.missionTextEdit += ml + '\n'
          indent = "    "
        else:
          self.mission.missionTextEdit += indent + ml + '\n'
      self.mission.missionTextEditChaged = True
      #self.ui.mission_help.setPlainText(config.get('mission', 'help'))
      #if self.isConnected():
        #self.mission.sendToRobot()
      #self.ui.mission_filename.setText(config.get('mission', 'filename'))
      print("mission load end")
      #
      # line sensor page
      try:
        self.ui.line_disp_max_value.setValue(config.getfloat('lineSensor', 'maximum'))
        self.ui.ls_use_sensor.setChecked(config.getboolean('lineSensor', 'use'))
        self.ui.ls_line_white.setChecked(config.getboolean('lineSensor', 'white'))
        self.ui.ls_power_high.setChecked(config.getboolean('lineSensor', 'LEDpower'))
        self.ui.ls_power_auto.setChecked(config.getboolean('lineSensor', 'LEDpowerAuto'))
        self.lineSensor.configChanged()
        #if self.isConnected():
          #self.lineSensor.setWhiteLine()
      except:
        print"# load of line sensor failed (missing or mis-spelled keywords)"
      # Wall follow

    except:
      # default, if no regbot.ini file is available
      self.ui.actionDebug.setChecked(True)
      self.ui.actionZ.setChecked(False)
      self.ui.actionWifi.setChecked(False)
      self.ui.actionLog.setChecked(True)
      self.ui.actionRobot.setChecked(True)
      self.ui.actionIMU.setChecked(False)
      self.ui.actionLine.setChecked(False)
      self.ui.actionDistance.setChecked(False)
      self.ui.actionVelocity.setChecked(True)
      self.ui.actionPosition.setChecked(True)
      self.ui.actionTurn.setChecked(True)
      #self.ui.actionBalance.setChecked(False)
      self.ui.actionControl.setChecked(False)
      self.ui.actionFollow_line.setChecked(False)
      self.ui.actionMission.setChecked(True)
      # implement config actions
      self.parent.menuShowDebug()
      self.parent.menuShowZ()
      self.parent.menuShowLog()
      self.parent.menuShowRobot()
      self.parent.menuShowIMU()
      self.parent.menuShowVelocity()
      self.parent.menuShowPosition()
      self.parent.menuShowLine()
      self.parent.menuShowDist()
      self.parent.menuShowTurn()
      self.parent.menuShowBalance()
      self.parent.menuShowControl()
      self.parent.menuShowFollowLine() 
      self.parent.menuShowMission()
      print("Configuration load: failed to find regbot.ini or at least one value in the file")
    for rid in range(1, 64):
      #rb = URobotInfo()
      try:
        idstr = "robot" + str(rid)
        #print("robot '" + idstr + "' loading:")
        rbase = config.getfloat(idstr, "robotBase")
        # if section with this id is found make a robot
        rb = self.info.getRobot(rid)
        rb.name = config.get(idstr, "name")
        rb.wheelbase = rbase
        rb.version = config.get(idstr, "version")
        rb.gear = config.getfloat(idstr, "gear")
        rb.pulsePerRev = config.getfloat(idstr, "encoderPPR")
        rb.wheelLRadius = config.getfloat(idstr, "wheelRadiusLeft")
        rb.wheelRRadius = config.getfloat(idstr, "wheelRadiusRight")
        rb.balanceOffset = config.getfloat(idstr, "balanceOffset")
        rb.batteryUse = config.getboolean(idstr, "batteryUse")
        rb.batteryIdleVolt = config.getfloat(idstr, "batteryIdleVolt")
        try:
          rb.wifiIP = config.get(idstr, "wifiIP")
        except:
          rb.wifiIP = "no_IP"
        try:
          rb.wifiMAC = config.get(idstr, "wifiMAC")
        except:
          rb.wifiMAC = "no_MAC"
        #try:
          #rb.wifiport = config.getint(idstr, "wifiport")
        #except:
          #rb.wifiport = 0
        #print("loaded data for robot " + str(rb.robotID))
        #print("    base: " + str(rb.wheelbase))
        #print("    gear: " + str(rb.gear))
        #print("    ppr : " + str(rb.pulsePerRev))
        #print("    wr 1: " + str(rb.wheelLRadius))
        #print("    wr 2: " + str(rb.wheelRRadius))
        #print("    offs: " + str(rb.balanceOffset))
        #print("RID = '" + str(rid) + " == " + str(int(self.ui.robot_id.value())))
        if rid == int(self.ui.robot_id.value()):
          self.ui.robot_base.setValue(rb.wheelbase)
          self.ui.robot_gear.setValue(rb.gear)
          self.ui.robot_pulse_per_rev.setValue(rb.pulsePerRev)
          self.ui.robot_wheel_radius_left.setValue(rb.wheelLRadius)
          self.ui.robot_wheel_radius_right.setValue(rb.wheelRRadius)
          self.ui.robot_balance_offset.setValue(rb.balanceOffset)
          self.ui.robot_on_battery.setChecked(rb.batteryUse)
          self.ui.robot_battery_idle_volt.setValue(rb.batteryIdleVolt)
          #print("saved to vidgets")
          if self.isConnected() or self.wifiConnected:
            self.robotIdApply()
        #for rb in self.robot.info.robots:
        #print("    loaded robots ID " + str(rb.robotID))
      except:
        pass
        #print("Configuration load: failed to find value in config file for this robot " + str(rid))

    pass    
  def configurationRobotSave(self):
    self.conWrite("eew\n")
  def clientVersion(self):
    gg = CLIENT_REV.split()
    return "3." + gg[2]
  def clientVersionDate(self):
    gg = CLIENT_REV.split()
    return gg[3]
# class URobot end        

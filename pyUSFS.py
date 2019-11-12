#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import smbus
import time
import math
import struct
import select
import sys
import curses
import numpy as np

MAG_DECLINATION = 0.0

# Sensor-specific definitions
# MPU9250
MPU9250_GYRO_LPF_3600HZ     =0x07
MPU9250_GYRO_LPF_250HZ      =0x00
MPU9250_GYRO_LPF_184HZ      =0x01
MPU9250_GYRO_LPF_92HZ       =0x02
MPU9250_GYRO_LPF_41HZ       =0x03
MPU9250_GYRO_LPF_20HZ       =0x04
MPU9250_GYRO_LPF_10HZ       =0x05
MPU9250_GYRO_LPF_5HZ        =0x06
MPU9250_GYRO_DLPF_CFG       =MPU9250_GYRO_LPF_41HZ
MPU9250_ACC_LPF_460HZ       =0x00
MPU9250_ACC_LPF_184HZ       =0x01
MPU9250_ACC_LPF_92HZ        =0x02
MPU9250_ACC_LPF_41HZ        =0x03
MPU9250_ACC_LPF_20HZ        =0x04
MPU9250_ACC_LPF_10HZ        =0x05
MPU9250_ACC_LPF_5HZ         =0x06
MPU9250_ACC_DLPF_CFG        =MPU9250_ACC_LPF_41HZ
ACC_ODR_1000HZ              =0x64
ACC_ODR_500HZ               =0x32
ACC_ODR_200HZ               =0x14
ACC_ODR_100HZ               =0x0A
ACC_ODR                     =ACC_ODR_1000HZ
GYRO_ODR_1000HZ             =0x64
GYRO_ODR_500HZ              =0x32
GYRO_ODR_200HZ              =0x14
GYRO_ODR_100HZ              =0x0A
GYRO_ODR                    =GYRO_ODR_1000HZ
MAG_ODR_100HZ               =0x64
MAG_ODR_50HZ                =0x32
MAG_ODR_20HZ                =0x14
MAG_ODR_10HZ                =0x0A
MAG_ODR                     =MAG_ODR_100HZ
BARO_ODR_83HZ               =0x53
BARO_ODR_50HZ               =0x32
BARO_ODR_25HZ               =0x19
BARO_ODR                    =BARO_ODR_25HZ
QUAT_DIV_10HZ               =0x09
QUAT_DIV_5HZ                =0x04
QUAT_DIV_4HZ                =0x03
QUAT_DIV_2HZ                =0x01
QUAT_DIV                    =QUAT_DIV_10HZ
ACC_SCALE_2G                =0x02
ACC_SCALE_4G                =0x04
ACC_SCALE_8G                =0x08
ACC_SCALE_16G               =0x10
ACC_SCALE                   =ACC_SCALE_8G
GYRO_SCALE_250DPS           =0xFA
GYRO_SCALE_500DPS           =0x1F4
GYRO_SCALE_1000DPS          =0x3E8
GYRO_SCALE_2000DPS          =0x7D0
GYRO_SCALE                  =GYRO_SCALE_2000DPS
MAG_SCALE_1000UT            =0x3E8
MAG_SCALE                   =MAG_SCALE_1000UT
DPS_PER_COUNT               =0.1525878906
SENTRAL_UT_PER_COUNT        =0.0305176
G_PER_COUNT                 =0.0004882813

# EEPROM definitions
EEPROM_DATA_ADDRESS         =0x50

# USFS register definitions
USFS_ADDRESS                =0x28
USFS_QX                     =0x00
USFS_QY                     =0x04
USFS_QZ                     =0x08
USFS_QW                     =0x0C
USFS_QTIME                  =0x10
USFS_MX                     =0x12
USFS_MY                     =0x14
USFS_MZ                     =0x16
USFS_MTIME                  =0x18
USFS_AX                     =0x1A
USFS_AY                     =0x1C
USFS_AZ                     =0x1E
USFS_ATIME                  =0x20
USFS_GX                     =0x22
USFS_GY                     =0x24
USFS_GZ                     =0x26
USFS_GTIME                  =0x28
USFS_Baro                   =0x2A
USFS_BaroTIME               =0x2C
USFS_Temp                   =0x2E
USFS_TempTIME               =0x30
USFS_QRateDivisor           =0x32
USFS_EnableEvents           =0x33
USFS_HostControl            =0x34
USFS_EventStatus            =0x35
USFS_SensorStatus           =0x36
USFS_SentralStatus          =0x37
USFS_AlgorithmStatus        =0x38
USFS_FeatureFlags           =0x39
USFS_ParamAcknowledge       =0x3A
USFS_SavedParamByte0        =0x3B
USFS_SavedParamByte1        =0x3C
USFS_SavedParamByte2        =0x3D
USFS_SavedParamByte3        =0x3E
USFS_ActualMagRate          =0x45
USFS_ActualAccelRate        =0x46
USFS_ActualGyroRate         =0x47
USFS_ErrorRegister          =0x50
USFS_AlgorithmControl       =0x54
USFS_MagRate                =0x55
USFS_AccelRate              =0x56
USFS_GyroRate               =0x57
USFS_BaroRate               =0x58
USFS_TempRate               =0x59
USFS_LoadParamByte0         =0x60
USFS_LoadParamByte1         =0x61
USFS_LoadParamByte2         =0x62
USFS_LoadParamByte3         =0x63
USFS_ParamRequest           =0x64
USFS_ROMVersion1            =0x70
USFS_ROMVersion2            =0x71
USFS_RAMVersion1            =0x72
USFS_RAMVersion2            =0x73
USFS_ProductID              =0x90
USFS_RevisionID             =0x91
USFS_RunStatus              =0x92
USFS_UploadAddress          =0x94
USFS_UploadData             =0x96
USFS_CRCHost                =0x97
USFS_ResetRequest           =0x9B
USFS_PassThruStatus         =0x9E
USFS_PassThruControl        =0xA0
USFS_ACC_LPF_BW             =0x5B
USFS_GYRO_LPF_BW            =0x5C
USFS_BARO_LPF_BW            =0x5D
USFS_GP8                    =0x3F
USFS_GP9                    =0x40
USFS_GP10                   =0x41
USFS_GP11                   =0x42
USFS_GP12                   =0x43
USFS_GP13                   =0x44
USFS_GP20                   =0x4B
USFS_GP21                   =0x4C
USFS_GP22                   =0x4D
USFS_GP23                   =0x4E
USFS_GP24                   =0x4F
USFS_GP36                   =0x5B
USFS_GP37                   =0x5C
USFS_GP38                   =0x5D
USFS_GP39                   =0x5E
USFS_GP40                   =0x5F
USFS_GP50                   =0x69
USFS_GP51                   =0x6A
USFS_GP52                   =0x6B
USFS_GP53                   =0x6C
USFS_GP54                   =0x6D
USFS_GP55                   =0x6E
USFS_GP56                   =0x6F

ACCEL_CAL                   =True
WARM_START                  =True
UPDATE_PERIOD               =100

# initialize i2c bus
def initBus(bus_num):
    return smbus.SMBus(bus_num)

# write a byte to an open i2c bus
def writeByteToBus(bus, address, offset, data_byte):
    bus.write_byte_data(address, offset, data_byte)

# read 'count' bytes from an open i2c bus
def readBytesFromBus(bus, address, offset, count):
    bus.write_byte(address, offset)
    return [bus.read_byte(address) for k in range(count)]

class USFS(object):
    def __init__(self, bus_num, calibrate=False):
        self.bus_num = bus_num
        self.calibrate = calibrate
        self.accel_cal = np.zeros((2,3), dtype=np.int16)
        self.accel_cal_valid = False
        self.ws_params = np.zeros((35,4), dtype=np.uint8)
        self.ws_params_valid = False
        self.event_status = None
        self.calibrating_accel = 0
        self.event_status = 0
        self.quat_flag = False
        self.gyro_flag = False
        self.accel_flag = False
        self.mag_flag = False
        self.baro_flag = False
        self.algo_status = 0
        self.gyro_adc = np.zeros((3,1), dtype=np.int16)
        self.gyro_data = np.zeros((3,1))
        self.accel_adc = np.zeros((3,1), dtype=np.int16)
        self.accel_cal_adc = np.zeros((3,1), dtype=np.int16)
        self.a_accel = np.zeros((3,1), dtype=np.int64)
        self.b_accel = np.zeros((3,1), dtype=np.int64)
        self.accel_data = np.zeros((3,1))
        self.accel_data_linear = np.zeros((3,1))
        self.mag_adc = np.zeros((3,1), dtype=np.int16)
        self.mag_data = np.zeros((3,1))
        self.raw_pressure = 0
        self.pressure = 0
        self.raw_temperature = 0
        self.temperature = 0
        self.qt = np.array([[1.0], [0.0], [0.0], [0.0]])
        self.qt_timestamp = 0
        self.accel_smooth = np.zeros((3,1))
        self.accel_linear = np.zeros((3,1))
        self.buff_roll = 0
        self.buff_pitch = 0
        self.buff_heading = 0
        self.angle = np.zeros((2,1))
        self.yaw = 0
        self.heading = 0
        self.mag_declination = MAG_DECLINATION
        self.start_runtime = 0
        self.timestamp = 0

        # initialize i2c bus
        try:
            self.USFS = initBus(bus_num)
        except:
            print('No i2c device detected on bus{:2d}!'.format(bus_num))
            exit(1)
        print('i2c bus initialized on bus{:2d}.'.format(bus_num))

        # check USFS status, make sure EEPROM upload of firmware was accomplished
        upload = False
        for i in range(100):
            if (self.readRegister(USFS_SentralStatus) & 0x01):
                upload = True
                break
            else:
                # if status is not good, attempt to reset
                self.writeRegister(USFS_ResetRequest, 0x01)
                time.sleep(0.5)
        if (self.readRegister(USFS_SentralStatus) & 0x04):
            upload = False
        if not upload:
            print('Failed to upload firmware to device!')
            exit(1)
        else:
            print('Firmware upload successful.')
        print('Sentral status:{:2d} (should be 3).'.format(self.readRegister(USFS_SentralStatus)))

        # place into pass-through mode
        self.writeRegister(USFS_PassThruControl, 0x01)
        time.sleep(0.1)
        pass_thru_enabled = False
        for i in range(100):
            if(self.readRegister(USFS_PassThruStatus) & 0x01):
                pass_thru_enabled = True
                break
            else:
                pass_thru_enabled = False
                time.sleep(0.1)
        if not pass_thru_enabled:
            print('Failed to place device into pass-through mode!')
            exit(1)
        print('Device placed into pass-through mode.')

        print('Fetching EEPROM acceleration and warm-start calibration parameters...')
        self.readAccelCal()
        self.readSenParams()

        # take out of pass-through mode
        self.writeRegister(USFS_PassThruControl, 0x00)
        time.sleep(0.1)
        for i in range(100):
            if(self.readRegister(USFS_PassThruStatus) & 0x01):
                pass_thru_enabled = True
                time.sleep(0.1)
            else:
                pass_thru_enabled = False
                break
        if pass_thru_enabled:
            print('Failed to take device out of pass-through mode!')
            exit(1)
        print('Done.')

        print('Accelerometer Calibration Data:')
        print('X-acc max: {:4d}'.format(self.accel_cal[0,0]))
        print('Y-acc max: {:4d}'.format(self.accel_cal[0,1]))
        print('Z-acc max: {:4d}'.format(self.accel_cal[0,2]))
        print('X-acc min: {:4d}'.format(self.accel_cal[1,0]))
        print('Y-acc min: {:4d}'.format(self.accel_cal[1,1]))
        print('Z-acc min: {:4d}'.format(self.accel_cal[1,2]))
        print('Checking and loading accelerometer calibration data if good...')

        # set device to idle state
        self.writeRegister(USFS_HostControl, 0x00)

        # check that the accel cal values are valid
        self.accel_cal_valid = True
        for i in range(3):
            if (self.accel_cal[1,i] < -2240 or  self.accel_cal[1,i] > -1800):
                self.accel_cal_valid = False
            if (self.accel_cal[0,i] < 1800 or  self.accel_cal[0,i] > 2240):
                self.accel_cal_valid = False

        self.accelCalUpload()
        if (ACCEL_CAL and self.accel_cal_valid):
            print('Accelerometer calibration data is valid...')
        else:
            print('Accelerometer calibration data is NOT valid. Defaults loaded...')

        # force initialize - reads accel cal data into variable
        self.writeRegister(USFS_HostControl, 0x01)
        print('Done. Loading warm start parameters, modifying sensor ranges and data rates...')
        time.sleep(0.1)

        # apply warm start parameters
        if (WARM_START and self.ws_params_valid):
            self.setWSParameters()
            print('Warm start data loaded...')
        else:
            print('Warm start data NOT loaded.')

        # set sensor low-pass filter bandwidth. MUST BE DONE BEFORE SETTING ODR RATES!
        self.writeRegister(USFS_ACC_LPF_BW, MPU9250_ACC_DLPF_CFG)
        self.writeRegister(USFS_GYRO_LPF_BW, MPU9250_GYRO_DLPF_CFG)

        # set accel/gyro/mag to desired ODR rates
        self.writeRegister(USFS_AccelRate, ACC_ODR)
        self.writeRegister(USFS_GyroRate, GYRO_ODR)
        self.writeRegister(USFS_MagRate, MAG_ODR)
        self.writeRegister(USFS_QRateDivisor, QUAT_DIV)

        # ODR + 10000000b to activate the eventStatus bit for the barometer...
        self.writeRegister(USFS_BaroRate, (0x80 + BARO_ODR))

        # configure operating mode
        # output scaled sensor data (quaternion convention NED)
        self.writeRegister(USFS_AlgorithmControl, 0x00)

        # enable interrupt to host upon certain events
        # choose interrupts when: gyros updated (0x20), sentral error (0x02), or sentral reset (0x01)
        self.writeRegister(USFS_EnableEvents, 0x23)

        print('Done. Starting the Sentral...')

        # start the sentral
        self.writeRegister(USFS_AlgorithmControl, 0x00)

        print('Done. Loading algorithm tuning parameters...')

        # perform final sentral algorithm parameters modifications
        self.setIntegerParam(0x49, 0x00)                    # disable 'stillness' mode
        self.setIntegerParam(0x48, 0x01)                    # set Gbias_mode to 1
        self.setMagAccFs(MAG_SCALE, ACC_SCALE)              # set magnetometer/accelerometer full-scale ranges
        self.setGyroFs(GYRO_SCALE)                          # set gyroscope full-scale range
        self.setFloatParam(0x3B, float(0.0))                # set param 59 mag transient protection off (0.0)
        #self.setFloatParam(0x34, float(4.0))               # set param 52 mag merging rate (0.7 is default)
        #self.setFloatParam(0x35, float(0.3))               # set param 53 accel merging rate (0.6 is default)

        #self.writeRegister(USFS_AlgorithmControl, 0x02)     # diagnostic - reports unscaled sensor data

        # choose interrupt events: gyros updated (0x20), sentral error (0x02), or sentral reset (0x01)
        self.writeRegister(USFS_EnableEvents, 0x23)

        # read event status register
        self.event_status = self.readRegister(USFS_EventStatus)

        print('Sentral initialization complete!')
        print('')

    def run(self):
        self.start_runtime = time.time()
        prev_loop_start = time.time()
        last_refresh = time.time()
        console = curses.initscr()
        curses.noecho()
        curses.cbreak()
        console.keypad(True)
        console.nodelay(True)
        input = None
        while True:
            loop_start = time.time()
            loop_time = loop_start - prev_loop_start
            prev_loop_start = loop_start

            self.fetchEventStatus()
            self.fetchSentralData()

            console.addstr(0,0,'Algorithm Status = {:2d}'.format(self.algo_status))
            console.addstr(1,0,'')
            console.addstr(2,0,'ax_0 = {:10d}, ay_0 = {:10d}, az_0 = {:10d} mg'.format(int(1000.0*self.accel_data[0]), int(1000.0*self.accel_data[1]), int(1000.0*self.accel_data[2])))
            console.addstr(3,0,'gx_0 = {:10.2f}, gy_0 = {:10.2f}, gz_0 = {:10.2f} deg/s'.format(self.gyro_data[0,0], self.gyro_data[1,0], self.gyro_data[2,0]))
            console.addstr(4,0,'')
            console.addstr(5,0,'mx_0 = {:4d}, my_0 = {:4d}, mz_0 = {:4d} uT'.format(int(self.mag_data[0,0]), int(self.mag_data[1,0]), int(self.mag_data[2,0])))
            console.addstr(6,0,'')
            console.addstr(7,0,'Sentral Quaternion (NED):')
            console.addstr(8,0,'Q0_0 = {:6.2f}, Qx_0 = {:6.2f}, Qy_0 = {:6.2f}, Qz_0 = {:6.2f}'.format(self.qt[0,0], self.qt[1,0], self.qt[2,0], self.qt[3,0]))
            console.addstr(9,0,'')
            console.addstr(10,0,'Sentral Yaw, Pitch, Roll: {:6.2f}, {:6.2f}, {:6.2f}'.format(self.heading, self.angle[1,0], self.angle[0,0]))
            console.addstr(11,0,'')
            console.addstr(12,0,'Baro Pressure: {:6.2f} mbar'.format(self.pressure))
            console.addstr(13,0,'Baro Temperature: {:4.2f} deg C'.format(self.temperature))
            console.addstr(14,0,'')
            console.addstr(15,0,'Loop Cycle Time: {:4.2f} ms'.format(loop_time*1e3))
            console.addstr(16,0,'')
            console.addstr(17,0,'Press `1` for accelerometer calibration.')
            console.addstr(18,0,'Ensure that the desired accelerometer axis is aligned with gravity and remains still.')
            console.addstr(19,0,'All three accelerometers must be calibrated to within +/-1g for accurate results.')
            console.addstr(20,0,'Press `2` to save Warm Start parameters.')
            console.addstr(21,0,'Press `3` to reset Warm Start parameters and quit.')
            console.addstr(22,0,'Press `q` to quit.')
            console.addstr(23,0,'Accelerometer calibration countdown: {:4d}'.format(self.calibrating_accel))
            console.refresh()

            try:
                input = console.getkey()
            except:
                pass

            if input == 'q':
                curses.nocbreak()
                console.nodelay(False)
                console.keypad(False)
                curses.echo()
                curses.endwin()
                print('Quitting Sentral program.')
                sys.exit(0)
            elif input == '1':
                input = None
                self.calibrating_accel = 512
            elif input == '2':
                input = None
                self.saveSentralWSParameters()
            elif input == '3':
                input = None
                self.resetSentralWSParameters()
                curses.nocbreak()
                console.nodelay(False)
                console.keypad(False)
                curses.echo()
                curses.endwin()
                print('Quitting Sentral program.')
                sys.exit(0)
            else:
                input = None

    def fetchSentralData(self):
        if self.gyro_flag:
            self.gyroGetADC()
            for i in range(3):
                self.gyro_data[i] = float(self.gyro_adc[i]*DPS_PER_COUNT)
            self.gyro_flag = False
        if self.quat_flag:
            self.computeIMU()
            self.quat_flag = False
        if self.accel_flag:
            self.accelGetADC()
            self.accelCommon()
            for i in range(3):
                self.accel_data[i] = float(self.accel_adc[i]*G_PER_COUNT)
                self.accel_data_linear[i] = float(self.accel_linear[i]*G_PER_COUNT)
            self.accel_flag = False
        if self.mag_flag:
            self.magGetADC()
            for i in range(3):
                self.mag_data[i] = float(self.mag_adc[i]*SENTRAL_UT_PER_COUNT)
            self.mag_flag = False
        if self.baro_flag:
            self.baroGetPressure()
            self.baroGetTemperature()
            self.baro_flag = False

    def saveSentralWSParameters(self):
        self.getWSParameters()
        self.WSPassThroughMode()
        self.writeSenParams()
        self.WSResume()

    def resetSentralWSParameters(self):
        self.ws_params[:] = 255
        self.WSPassThroughMode()
        self.resetSenParams()
        self.WSResume()
        self.setWSParameters()

    def getWSParameters(self):
        self.writeRegister(USFS_ParamRequest, 1)
        time.sleep(0.1)
        self.writeRegister(USFS_AlgorithmControl, 0x80)
        time.sleep(0.1)
        stat = self.readRegister(USFS_ParamAcknowledge)
        while (not stat==1):
            stat = self.readRegister(USFS_ParamAcknowledge)
        self.ws_params[0,0] = self.readRegister(USFS_SavedParamByte0)
        self.ws_params[0,1] = self.readRegister(USFS_SavedParamByte1)
        self.ws_params[0,2] = self.readRegister(USFS_SavedParamByte2)
        self.ws_params[0,3] = self.readRegister(USFS_SavedParamByte3)
        for i in range(35):
            param = i+1
            self.writeRegister(USFS_ParamRequest, param)
            time.sleep(0.1)
            stat = self.readRegister(USFS_ParamAcknowledge)
            while (not stat==param):
                stat = self.readRegister(USFS_ParamAcknowledge)
            self.ws_params[i,0] = self.readRegister(USFS_SavedParamByte0)
            self.ws_params[i,1] = self.readRegister(USFS_SavedParamByte1)
            self.ws_params[i,2] = self.readRegister(USFS_SavedParamByte2)
            self.ws_params[i,3] = self.readRegister(USFS_SavedParamByte3)

        self.writeRegister(USFS_ParamRequest, 0x00)
        time.sleep(0.1)
        self.writeRegister(USFS_AlgorithmControl, 0x00)
        time.sleep(0.1)

    def setWSParameters(self):
        param = 1
        param = param | 0x80
        self.writeRegister(USFS_LoadParamByte0, self.ws_params[0,0])
        self.writeRegister(USFS_LoadParamByte1, self.ws_params[0,1])
        self.writeRegister(USFS_LoadParamByte2, self.ws_params[0,2])
        self.writeRegister(USFS_LoadParamByte3, self.ws_params[0,3])
        self.writeRegister(USFS_ParamRequest, param)
        self.writeRegister(USFS_AlgorithmControl, 0x80)
        stat = self.readRegister(USFS_ParamAcknowledge)
        while (not stat==param):
            stat = self.readRegister(USFS_ParamAcknowledge)
        for i in range(35):
            param = (i+1) | 0x80
            self.writeRegister(USFS_LoadParamByte0, self.ws_params[i,0])
            self.writeRegister(USFS_LoadParamByte1, self.ws_params[i,1])
            self.writeRegister(USFS_LoadParamByte2, self.ws_params[i,2])
            self.writeRegister(USFS_LoadParamByte3, self.ws_params[i,3])
            self.writeRegister(USFS_ParamRequest, param)
            stat = self.readRegister(USFS_ParamAcknowledge)
            while (not stat==param):
                stat = self.readRegister(USFS_ParamAcknowledge)
        self.writeRegister(USFS_ParamRequest, 0x00)
        self.writeRegister(USFS_AlgorithmControl, 0x00)

    def computeIMU(self):
        # pass-through for future experimentation
        self.accel_smooth[0] = self.accel_adc[0]
        self.accel_smooth[1] = self.accel_adc[1]
        self.accel_smooth[2] = self.accel_adc[2]

        self.getQuat()

        a11 = self.qt[0]*self.qt[0] + self.qt[1]*self.qt[1] - self.qt[2]*self.qt[2] - self.qt[3]*self.qt[3]
        a21 = 2.0*(self.qt[0]*self.qt[3] + self.qt[1]*self.qt[2])
        a31 = 2.0*(self.qt[1]*self.qt[3] - self.qt[0]*self.qt[2])
        a32 = 2.0*(self.qt[0]*self.qt[1] + self.qt[2]*self.qt[3])
        a33 = self.qt[0]*self.qt[0] - self.qt[1]*self.qt[1] - self.qt[2]*self.qt[2] + self.qt[3]*self.qt[3]

        # pass-through for future experimentation
        self.buff_roll = (math.atan2(a32, a33))*57.2957795
        if a31 < -1.0:
            a31 = -1.0
        if a31 > 1.0:
            a31 = 1.0
        self.buff_pitch = -(math.asin(a31))*57.2957795
        self.buff_heading = (math.atan2(a21, a11))*57.2957795

        self.angle[0] = self.buff_roll
        self.angle[1] = self.buff_pitch
        self.yaw = self.buff_heading
        self.heading = self.yaw + self.mag_declination
        if (self.heading < 0.0):
            self.heading += 360.0
        self.timestamp = time.time() - self.start_runtime

    def getQuat(self):
        vals = self.readRegisters(USFS_QX, 18)
        qx = self.uint32_reg_to_float(vals[0:4])
        qy = self.uint32_reg_to_float(vals[4:8])
        qz = self.uint32_reg_to_float(vals[8:12])
        qw = self.uint32_reg_to_float(vals[12:16])
        qtime = int((vals[17]<<8) | vals[16]);
        self.qt[1] = qx
        self.qt[2] = qy
        self.qt[3] = qz
        self.qt[0] = qw
        self.qt_timestamp = qtime

    def accelGetADC(self):
        vals = struct.unpack('hhh', bytes(self.readRegisters(USFS_AX, 6)))
        self.accel_adc[0] = +vals[0]
        self.accel_adc[1] = +vals[1]
        self.accel_adc[2] = +vals[2]
        if self.calibrating_accel > 0:
            self.accel_cal_adc[0] = +vals[0]
            self.accel_cal_adc[1] = +vals[1]
            self.accel_cal_adc[2] = +vals[2]

    def accelCommon(self):
        if self.calibrating_accel == 512:
            self.writeRegister(USFS_AlgorithmControl, 0x02)
            time.sleep(0.1)
            self.accelGetADC()
        if self.calibrating_accel > 0:
            for axis in range(3):
                if (self.accel_cal_adc[axis]/(0x10/ACC_SCALE) > 1024):
                    self.a_accel[axis] += np.int64(self.accel_cal_adc[axis]/(0x10/ACC_SCALE))
                if (self.accel_cal_adc[axis]/(0x10/ACC_SCALE) < -1024):
                    self.b_accel[axis] += np.int64(self.accel_cal_adc[axis]/(0x10/ACC_SCALE))
                self.accel_cal_adc[axis] = 0.0
            if self.calibrating_accel == 1:
                for axis in range(3):
                    if (self.a_accel[axis]>>9 > 1024):
                        self.accel_cal[0, axis] = self.a_accel[axis]>>9
                    if (self.b_accel[axis]>>9 < -1024):
                        self.accel_cal[1, axis] = self.b_accel[axis]>>9
                    self.a_accel[axis] = 0
                    self.b_accel[axis] = 0
                self.WSPassThroughMode()
                self.writeAccelCal()
                self.WSResume()
            self.calibrating_accel -= 1

    def gyroGetADC(self):
        vals = struct.unpack('hhh', bytes(self.readRegisters(USFS_GX, 6)))
        self.gyro_adc[0] = +vals[0]
        self.gyro_adc[1] = +vals[1]
        self.gyro_adc[2] = +vals[2]

    def magGetADC(self):
        vals = struct.unpack('hhh', bytes(self.readRegisters(USFS_MX, 6)))
        self.mag_adc[0] = +vals[0]
        self.mag_adc[1] = +vals[1]
        self.mag_adc[2] = +vals[2]

    def baroGetPressure(self):
        self.raw_pressure = struct.unpack('h', bytes(self.readRegisters(USFS_Baro, 2)))[0]
        self.pressure = self.raw_pressure * .01 + 1013.25

    def baroGetTemperature(self):
        self.raw_temperature = struct.unpack('h', bytes(self.readRegisters(USFS_Temp, 2)))[0]
        self.temperature = self.raw_temperature * .01

    def fetchEventStatus(self):
        self.event_status = self.readRegister(USFS_EventStatus)
        if (self.event_status & 0x04): self.quat_flag = True
        if (self.event_status & 0x20): self.gyro_flag = True
        if (self.event_status & 0x10): self.accel_flag = True
        if (self.event_status & 0x08): self.mag_flag = True
        if (self.event_status & 0x40): self.baro_flag = True
        self.algo_status = self.readRegister(USFS_AlgorithmStatus)

    def getEventStatus(self):
        return self.readRegister(USFS_EventStatus)

    def setGyroFs(self, gyro_fs):
        bites = [gyro_fs & (0xFF), (gyro_fs >> 8) & (0xFF), 0x00, 0x00]
        self.writeRegister(USFS_LoadParamByte0, bites[0]) #Gyro LSB
        self.writeRegister(USFS_LoadParamByte1, bites[1]) #Gyro MSB
        self.writeRegister(USFS_LoadParamByte2, bites[2]) #Unused
        self.writeRegister(USFS_LoadParamByte3, bites[3]) #Unused
        self.writeRegister(USFS_ParamRequest, 0xCB) #Parameter 75 0xCB is 75 decimal with the MSB set high to indicate a paramter write processs
        self.writeRegister(USFS_AlgorithmControl, 0x80) #Request parameter transfer procedure
        STAT = self.readRegister(USFS_ParamAcknowledge) #Check the parameter acknowledge register and loop until the result matches parameter request byte
        while(not (STAT==0xCB)):
            STAT = self.readRegister(USFS_ParamAcknowledge)

        self.writeRegister(USFS_ParamRequest, 0x00) #Parameter request = 0 to end parameter transfer process
        self.writeRegister(USFS_AlgorithmControl, 0x00) # Re-start algorithm

    def setMagAccFs(self, mag_fs, acc_fs):
        bites = [mag_fs & (0xFF), (mag_fs >> 8) & (0xFF),acc_fs & (0xFF), (acc_fs >> 8) & (0xFF)]
        self.writeRegister(USFS_LoadParamByte0, bites[0]) #Mag LSB
        self.writeRegister(USFS_LoadParamByte1, bites[1]) #Mag MSB
        self.writeRegister(USFS_LoadParamByte2, bites[2]) #Acc LSB
        self.writeRegister(USFS_LoadParamByte3, bites[3]) #Acc MSB
        self.writeRegister(USFS_ParamRequest, 0xCA) #Parameter 74 0xCA is 74 decimal with the MSB set high to indicate a paramter write processs
        self.writeRegister(USFS_AlgorithmControl, 0x80) #Request parameter transfer procedure
        STAT = self.readRegister(USFS_ParamAcknowledge) #Check the parameter acknowledge register and loop until the result matches parameter request byte
        while(not (STAT==0xCA)):
            STAT = self.readRegister(USFS_ParamAcknowledge)

        self.writeRegister(USFS_ParamRequest, 0x00) #Parameter request = 0 to end parameter transfer process
        self.writeRegister(USFS_AlgorithmControl, 0x00) # Re-start algorithm

    def setIntegerParam(self, param, param_val):
        bites = [param_val & (0xFF),(param_val >> 8) & (0xFF),(param_val >> 16) & (0xFF),(param_val >> 24) & (0xFF)]
        param = param | 0x80 #Parameter is the decimal value with the MSB set high to indicate a paramter write processs
        self.writeRegister(USFS_LoadParamByte0, bites[0]) #Param LSB
        self.writeRegister(USFS_LoadParamByte0, bites[1])
        self.writeRegister(USFS_LoadParamByte0, bites[2])
        self.writeRegister(USFS_LoadParamByte0, bites[3]) #Param MSB
        self.writeRegister(USFS_ParamRequest, param)
        self.writeRegister(USFS_AlgorithmControl, 0x80) #Request parameter transfer procedure
        STAT = self.readRegister(USFS_ParamAcknowledge) #Check the parameter acknowledge register and loop until the result matches parameter request byte
        while (not(STAT==param)):
          STAT = self.readRegister(USFS_ParamAcknowledge)
        self.writeRegister(USFS_ParamRequest, 0x00) #Parameter request = 0 to end parameter transfer process
        self.writeRegister(USFS_AlgorithmControl, 0x00) # Re-start algorithm

    def setFloatParam(self, param, param_val):
        bites = self.floatToBytes(param_val)
        param = param | 0x80 #Parameter is the decimal value with the MSB set high to indicate a paramter write processs
        self.writeRegister(USFS_LoadParamByte0, bites[0]) #Param LSB
        self.writeRegister(USFS_LoadParamByte0, bites[1])
        self.writeRegister(USFS_LoadParamByte0, bites[2])
        self.writeRegister(USFS_LoadParamByte0, bites[3]) #Param MSB
        self.writeRegister(USFS_ParamRequest, param)
        self.writeRegister(USFS_AlgorithmControl, 0x80) #Request parameter transfer procedure
        STAT = self.readRegister(USFS_ParamAcknowledge) #Check the parameter acknowledge register and loop until the result matches parameter request byte
        while (not(STAT==param)):
          STAT = self.readRegister(USFS_ParamAcknowledge)
        self.writeRegister(USFS_ParamRequest, 0x00) #Parameter request = 0 to end parameter transfer process
        self.writeRegister(USFS_AlgorithmControl, 0x00) # Re-start algorithm

    def floatToBytes(self, param_val):
        val = '{:08x}'.format(struct.unpack('<I',struct.pack('<f', param_val))[0])
        val1 = val[0:2]
        val2 = val[2:4]
        val3 = val[4:6]
        val4 = val[6:8]
        return[int('0x'+val1,0),int('0x'+val2,0),int('0x'+val3,0),int('0x'+val4,0)]

    def accelCalUpload(self):
        cal_num_byte = np.zeros((2,1), dtype=np.uint8)
        if (not ACCEL_CAL or not self.accel_cal_valid):
            cal_num_byte[0] = 0
            cal_num_byte[1] = 0
        else:
            # NORTH scale
            big_cal_num = (4096000000/(self.accel_cal[0,0] - self.accel_cal[1,0])) - 1000000
            cal_num = int(big_cal_num)
            cal_num_byte[0] = cal_num & 0xff
            cal_num_byte[1] = cal_num >> 8
        self.writeRegister(USFS_GP36, cal_num_byte[0])
        self.writeRegister(USFS_GP37, cal_num_byte[1])

        if (not ACCEL_CAL or not self.accel_cal_valid):
            cal_num_byte[0] = 0
            cal_num_byte[1] = 0
        else:
            # EAST scale
            big_cal_num = (4096000000/(self.accel_cal[0,1] - self.accel_cal[1,1])) - 1000000
            cal_num = int(big_cal_num)
            cal_num_byte[0] = cal_num & 0xff
            cal_num_byte[1] = cal_num >> 8
        self.writeRegister(USFS_GP38, cal_num_byte[0])
        self.writeRegister(USFS_GP39, cal_num_byte[1])

        if (not ACCEL_CAL or not self.accel_cal_valid):
            cal_num_byte[0] = 0
            cal_num_byte[1] = 0
        else:
            # DOWN scale
            big_cal_num = (4096000000/(self.accel_cal[0,2] - self.accel_cal[1,2])) - 1000000
            cal_num = int(big_cal_num)
            cal_num_byte[0] = cal_num & 0xff
            cal_num_byte[1] = cal_num >> 8
        self.writeRegister(USFS_GP40, cal_num_byte[0])
        self.writeRegister(USFS_GP50, cal_num_byte[1])

        if (not ACCEL_CAL or not self.accel_cal_valid):
            cal_num_byte[0] = 0
            cal_num_byte[1] = 0
        else:
            # NORTH offset
            big_cal_num = (((self.accel_cal[0,0] - 2048) + (self.accel_cal[1,0] + 2048))*100000)/4096;
            cal_num = int(big_cal_num)
            cal_num_byte[0] = cal_num & 0xff
            cal_num_byte[1] = cal_num >> 8
        self.writeRegister(USFS_GP51, cal_num_byte[0])
        self.writeRegister(USFS_GP52, cal_num_byte[1])

        if (not ACCEL_CAL or not self.accel_cal_valid):
            cal_num_byte[0] = 0
            cal_num_byte[1] = 0
        else:
            # EAST offset
            big_cal_num = (((self.accel_cal[0,1] - 2048) + (self.accel_cal[1,1] + 2048))*100000)/4096;
            cal_num = int(big_cal_num)
            cal_num_byte[0] = cal_num & 0xff
            cal_num_byte[1] = cal_num >> 8
        self.writeRegister(USFS_GP53, cal_num_byte[0])
        self.writeRegister(USFS_GP54, cal_num_byte[1])

        if (not ACCEL_CAL or not self.accel_cal_valid):
            cal_num_byte[0] = 0
            cal_num_byte[1] = 0
        else:
            # DOWN offset
            big_cal_num = (((self.accel_cal[0,2] - 2048) + (self.accel_cal[1,2] + 2048))*100000)/4096;
            cal_num = int(big_cal_num)
            cal_num_byte[0] = cal_num & 0xff
            cal_num_byte[1] = cal_num >> 8
        self.writeRegister(USFS_GP55, cal_num_byte[0])
        self.writeRegister(USFS_GP56, cal_num_byte[1])

    def readAccelCal(self):
        #self.USFS.write_i2c_block_data(EEPROM_DATA_ADDRESS, 0x80, [0x8c, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255])
        #time.sleep(1)
        self.USFS.write_i2c_block_data(EEPROM_DATA_ADDRESS, 0x80, [0x8c])
        data = [self.USFS.read_byte(EEPROM_DATA_ADDRESS) for k in range(12)]
        for axis in range(3):
            self.accel_cal[0, axis] = int(data[2*axis + 1]<<8) | data[2*axis]        # maxs
            self.accel_cal[1, axis] = int(data[2*axis + 7]<<8) | data[2*axis + 6]    # mins

    def readSenParams(self):
        data = np.zeros((140))
        for i in range(8):
            self.USFS.write_i2c_block_data(EEPROM_DATA_ADDRESS, 0x80, [0x00 + 16*i])
            data[16*i:16*(i+1)] = np.array([self.USFS.read_byte(EEPROM_DATA_ADDRESS) for k in range(16)])
        self.USFS.write_i2c_block_data(EEPROM_DATA_ADDRESS, 0x80, [0x80])
        data[-12:] = np.array([self.USFS.read_byte(EEPROM_DATA_ADDRESS) for k in range(12)])
        for param_num in range(35):
            for i in range(4):
                self.ws_params[param_num, i] = data[param_num*4 + i]
        self.USFS.write_i2c_block_data(EEPROM_DATA_ADDRESS, 0x80, [0x98])
        self.ws_params_valid = self.USFS.read_byte(EEPROM_DATA_ADDRESS)
        if(self.ws_params_valid == 0xaa):
            self.ws_params_valid = True
        else:
            self.ws_params_valid = False

    def writeSenParams(self):
        data = np.zeros((140), dtype=np.uint8)
        for param_num in range(35):
            for i in range(4):
                data[param_num*4 + i] = self.ws_params[param_num, i]
        for i in range(8):
            data_chunk = data[16*i:16*(i+1)].tolist()
            self.USFS.write_i2c_block_data(EEPROM_DATA_ADDRESS, 0x80, [0x00 + 16*i] + data_chunk)
            time.sleep(0.1)
        data_chunk = data[-12:].tolist()
        self.USFS.write_i2c_block_data(EEPROM_DATA_ADDRESS, 0x80, [0x80] + data_chunk)
        time.sleep(0.1)
        self.ws_params_valid = 0xaa
        self.USFS.write_i2c_block_data(EEPROM_DATA_ADDRESS, 0x80, [0x98, self.ws_params_valid])
        self.ws_params_valid = False

    def resetSenParams(self):
        data = np.zeros((140), dtype=np.uint8)
        for param_num in range(35):
            for i in range(4):
                data[param_num*4 + i] = self.ws_params[param_num, i]
        for i in range(8):
            data_chunk = data[16*i:16*(i+1)].tolist()
            self.USFS.write_i2c_block_data(EEPROM_DATA_ADDRESS, 0x80, [0x00 + 16*i] + data_chunk)
            time.sleep(0.1)
        data_chunk = data[-12:].tolist()
        self.USFS.write_i2c_block_data(EEPROM_DATA_ADDRESS, 0x80, [0x80] + data_chunk)
        time.sleep(0.1)
        self.ws_params_valid = 0x00
        self.USFS.write_i2c_block_data(EEPROM_DATA_ADDRESS, 0x80, [0x98, self.ws_params_valid])
        self.ws_params_valid = False

    def writeRegister(self, offset, data):
        writeByteToBus(self.USFS, USFS_ADDRESS, offset, data)

    def readRegister(self, offset):
        return self.readRegisters(offset, 1)[0]

    def readRegisters(self, offset, count):
        return readBytesFromBus(self.USFS, USFS_ADDRESS, offset, count)

    def uint32_reg_to_float(self, buf):
        return struct.unpack('f', bytes(buf))[0]

    def WSPassThroughMode(self):
        # place into pass-through mode for i2c read/write
        stat = 0
        self.writeRegister(USFS_AlgorithmControl, 0x01)
        time.sleep(0.1)
        self.writeRegister(USFS_PassThruControl, 0x01)
        time.sleep(0.1)
        stat = self.readRegister(USFS_PassThruStatus)
        time.sleep(0.1)
        while (not stat & 0x01):
            stat = self.readRegister(USFS_PassThruStatus)
            time.sleep(0.005)

    def WSResume(self):
        # cancel pass-through mode for i2c read/write
        stat = 0
        self.writeRegister(USFS_PassThruControl, 0x00)
        time.sleep(0.1)
        stat = self.readRegister(USFS_PassThruStatus)
        while (stat & 0x01):
            stat = self.readRegister(USFS_PassThruStatus)
            time.sleep(0.005)

        # restart algorithm
        self.writeRegister(USFS_AlgorithmControl, 0x00)
        time.sleep(0.1)
        stat = self.readRegister(USFS_AlgorithmStatus)
        while (stat & 0x01):
            stat = self.readRegister(USFS_AlgorithmStatus)
            time.sleep(0.005)

        # read event status to clear interrupt
        self.event_status = self.readRegister(USFS_EventStatus)

    def writeAccelCal(self):
        data = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
        for axis in range(3):
            data[2*axis] = int(self.accel_cal[0, axis]) & 0xff
            data[2*axis + 1] = int(self.accel_cal[0, axis]) >> 8
            data[2*axis + 6] = int(self.accel_cal[1, axis]) & 0xff
            data[2*axis + 7] = int(self.accel_cal[1, axis]) >> 8
        data = [0x8c] + data
        self.USFS.write_i2c_block_data(EEPROM_DATA_ADDRESS, 0x80, data)

usfs = USFS(2, calibrate=True)
usfs.run()

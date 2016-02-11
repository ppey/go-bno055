// Package bno055 allows interfacing with the LSM303 magnetometer.
package bno055

import (
	"errors"
	"fmt"
	"i2c"
	"time"
)

// BNO055 absoulute orientation sensor
// LSM303 represents a LSM303 magnetometer.
type BNO055 struct {
	i2cbus      *i2c.I2C
	initialized bool
	//mu          sync.RWMutex
	Name       string
	OppMode    byte
	SensorID   byte
	Address    byte
	Version    int
	DelayTime  time.Duration
	MaxValue   float32
	MinValue   float32
	Resolution float32

	quit chan struct{}
}

// OffsetsRawBNO055  holds 22 bytes of calibration data
type OffsetsRawBNO055 [22]byte

// StatusBNO055 holde the status of a Sensor
// dokument me
type StatusBNO055 struct {

	/* System Status (see section 4.3.58)
	---------------------------------
	0 = Idle
	1 = System Error
	2 = Initializing Peripherals
	3 = System Iniitalization
	4 = Executing Self-Test
	5 = Sensor fusio algorithm running
	6 = System running without fusion algorithms */
	BnoStatus,

	/* Self Test Results (see section )
	   --------------------------------
	   1 = test passed, 0 = test failed

	   Bit 0 = Accelerometer self test
	   Bit 1 = Magnetometer self test
	   Bit 2 = Gyroscope self test
	   Bit 3 = MCU self test

	   0x0F = all good! */
	SelfTestResult,

	/* System Error (see section 4.3.59)
	---------------------------------
	0 = No error
	1 = Peripheral initialization error
	2 = System initialization error
	3 = Self test result failed
	4 = Register map value out of range
	5 = Register map address out of range
	6 = Register map write error
	7 = BNO low power mode not available for selected operat ion mode
	8 = Accelerometer power mode not available
	9 = Fusion algorithm configuration error
	A = Sensor configuration error */
	SystemError uint8
}

// New creates a new LSM303 interface. The bus variable controls
// the I2C bus used to communicate with the device.
func New(bus *i2c.I2C) (*BNO055, error) {
	bno := &BNO055{i2cbus: bus, Name: "BNO055", OppMode: operationModeNdof,
		SensorID: bno055Id, Address: bno055AddressA, Version: 1, DelayTime: time.Millisecond * 100,
		MaxValue: 0.0, MinValue: 0.0, Resolution: 0.01}
	err := bno.Init(ModeNodf)
	return bno, err
}

// Close blaa
func (bno *BNO055) Close() {
	bno.i2cbus.Close()
}

// Init blöaa
func (bno *BNO055) Init(mode uint) error {

	errCout := 0
	for id, _ := bno.read(bno055ChipIDAddr); id != bno055Id; errCout++ {
		if errCout > 4 {
			return fmt.Errorf("Cant find Sensor on 0x%X", bno.Address)
		}
		time.Sleep(time.Second)

	}

	/* Switch to config mode (just in case since this is the default) */
	bno.setMode(operationModeConfig)
	time.Sleep(time.Second)
	// Reset

	bno.write(bno055SysTriggerAddr, 0x20)
	time.Sleep(time.Second)

	errCout = 0
	for id, _ := bno.read(bno055ChipIDAddr); id != bno055Id; errCout++ {
		if errCout > 4 {
			return errors.New("Sensor is gone after reset")
		}
		time.Sleep(time.Second)
	}

	/* Set to normal power mode */
	bno.write(bno055PwrModeAddr, powerModeNormal)
	time.Sleep(time.Second)

	bno.write(bno055PageIDAddr, 0x0)
	time.Sleep(time.Second)

	bno.write(bno055SysTriggerAddr, 0x00)
	time.Sleep(time.Second)

	if mode == 0 {
		mode = operationModeNdof
	}
	bno.setMode(operationModeNdof)

	return nil
}

// Delay halt
func (bno *BNO055) Delay() {
	time.Sleep(bno.DelayTime)
}

func (bno *BNO055) write(reg, value byte) error {
	return bno.i2cbus.WriteRegU8(reg, value)
}

func (bno *BNO055) readHL(reg byte) uint16 {
	value, err := bno.i2cbus.ReadRegU16Le(reg)
	if err != nil {
		fmt.Println("Error in read", err)
	}
	return value
}
func (bno *BNO055) writeHL(reg byte, value uint16) {
	err := bno.i2cbus.WriteRegU16Le(reg, value)
	if err != nil {
		fmt.Println("Error in read", err)
	}
}

func (bno *BNO055) read(reg byte) (byte, error) {
	return bno.i2cbus.ReadRegU8(reg)
}

func (bno *BNO055) readlen(reg byte, buff []byte) error {
	return bno.i2cbus.ReadLen(reg, buff)
}

func (bno *BNO055) writelen(reg byte, buff []byte) error {
	return bno.i2cbus.WriteLen(reg, buff)
}

func (bno *BNO055) setMode(mode byte) error {
	bno.OppMode = mode
	err := bno.write(bno055OprModeAddr, mode)
	if err != nil {
		fmt.Println("Error in setMode", err)
	}
	time.Sleep(time.Second)
	return err
}

// SetExtCrystalUse use the time this un platine kp wie in englisch
// Use the external 32.768KHz crystal0
func (bno *BNO055) SetExtCrystalUse(usextal bool) error {
	return errors.New("Error : IM VERRY BUGGIE")
	/* Switch to config mode (just in case since this is the default) */
	/*
		bno.setMode(operationModeConfig)
		bno.delay()
		bno.write(bno055PageIDAddr, 0)

		if usextal {
			bno.write(bno055SysTriggerAddr, 0x80)
		} else {
			bno.write(bno055SysTriggerAddr, 0x00)
		}
		return nil
	*/
}

// GetSystemStatus   Gets the latest system status info
//  Gets the latest system status info
func (bno *BNO055) GetSystemStatus(stat *StatusBNO055, runSelfTest bool) error {
	bno.write(bno055PageIDAddr, 0x0)

	if runSelfTest {
		lastMode := bno.OppMode
		bno.setMode(operationModeConfig)
		sysTrigger, _ := bno.read(bno055SysTriggerAddr)
		bno.write(bno055SysTriggerAddr, sysTrigger|0x1)
		time.Sleep(time.Second)
		bno.setMode(lastMode)
	}
	stat.BnoStatus, _ = bno.read(bno055SysStatAddr)
	stat.SelfTestResult, _ = bno.read(bno055SelftestResultAddr)
	stat.SystemError, _ = bno.read(bno055SysErrAddr)
	return nil
}

// RevInfoBNO055 struct used by getRevInfo as result
// document me
type RevInfoBNO055 struct {
	accelRev, magRev, gyroRev, blRev uint8
	swRe                             uint16
}

// GetRevInfo returns the revision info
// Gets the chip revision numbers
func (bno *BNO055) GetRevInfo(revInfo *RevInfoBNO055) error {
	var a, b uint8
	var err error
	/* Check the accelerometer revision */
	revInfo.accelRev, err = bno.read(bno055AccelRevIDAddr)
	if err != nil {
		return err
	}
	/* Check the magnetometer revision */
	revInfo.magRev, err = bno.read(bno055MagRevIDAddr)
	if err != nil {
		return err
	}
	/* Check the gyroscope revision */
	revInfo.gyroRev, err = bno.read(bno055GyroRevIDAddr)
	if err != nil {
		return err
	}
	/* Check the SW revision */
	revInfo.blRev, err = bno.read(bno055BLRevIDAddr)
	if err != nil {
		return err
	}
	a, err = bno.read(bno055SWRevIDLsbAddr)
	if err != nil {
		return err
	}
	b, err = bno.read(bno055SWRevIDMsbAddr)
	if err != nil {
		return err
	}
	revInfo.swRe = (uint16(b) << 8) | uint16(a)
	return nil
}

// CalbrBNO055 holde calibration data used by getCalibration
// dokument me
type CalbrBNO055 struct {
	Sys, Gyro, Accel, Mag uint8
}

// GetCalibration Gets current calibration state.
//				bnoCalData.XX will be set to 0 if not calibrated and 3 if
//				fully calibrated.
func (bno *BNO055) GetCalibration(calData *CalbrBNO055) error {
	bnoCalData, err := bno.read(bno055CalibStatAddr)
	if err != nil {
		return err
	}
	calData.Sys = (bnoCalData >> 6) & 0x03
	calData.Gyro = (bnoCalData >> 4) & 0x03
	calData.Accel = (bnoCalData >> 2) & 0x03
	calData.Mag = bnoCalData & 0x03
	return nil
}

// IsFullyCalibrated return true if the sensor is Calibrated
func (bno *BNO055) IsFullyCalibrated(caldata *CalbrBNO055) (bool, error) {
	if err := bno.GetCalibration(caldata); err != nil {
		return false, err
	}
	if caldata.Sys < 3 || caldata.Gyro < 3 || caldata.Accel < 3 || caldata.Mag < 3 {
		return false, nil
	}
	return true, nil
}

// GetTemp return the Temperature
func (bno *BNO055) GetTemp() int8 {
	temp, _ := bno.read(bno055TempAddr)
	return int8(temp)
}

// VecBNO055 holde x y z date measured by the sensor
// document me
type VecBNO055 struct {
	X, Y, Z float32
}

// AhrsHRP Ve
type AhrsHRP struct {
	Heading, Roll, Pitch float32
}

func (v *VecBNO055) toAhrsHRP(ahr *AhrsHRP) {
	ahr.Heading = v.X
	//	if v.Z < 0 && ahr.Roll < 0 {
	//		ahr.Roll = v.Y - 90
	//	} else if v.Z < 0 && ahr.Roll > 0 {
	//		ahr.Roll = v.Y + 90
	//	} else {
	ahr.Roll = v.Y
	//}
	ahr.Pitch = v.Z

}

// GetListenChan blaaa
func (bno *BNO055) GetListenChan() (chan (AhrsHRP), chan (error)) {
	mesureChan := make(chan AhrsHRP, 1)
	errorChan := make(chan error, 1)
	var euler VecBNO055
	var ahr AhrsHRP

	go func() {
		for {
			if err := bno.GetVector(&euler, VectorEuler); err != nil {
				errorChan <- err
				return
			}
			euler.toAhrsHRP(&ahr)
			mesureChan <- ahr
			bno.Delay()
		}
	}()
	return mesureChan, errorChan
}

// Use this constants as vectorType for GetVector
const (
	// ModeNodf sdasd
	ModeNodf = operationModeNdof
	// ModeCofig fgffggfgf
	ModeCofig = operationModeConfig
	// VectorAccelerometer zgou
	VectorAccelerometer = bno055AccelDataXLsbAddr
	// VectorMagnetometer juh
	VectorMagnetometer = bno055MagDataXLsbAddr
	// VectorGyroscope uhu
	VectorGyroscope = bno055GyroDataXLsbAddr
	// VectorEuler kj
	VectorEuler = bno055EulerHLsbAddr
	// VectorLinearaccel juh
	VectorLinearaccel = bno055LinearAccelDataXLsbAddr
	// VectorGravity jk
	VectorGravity = bno055GravityDataXLsbAddr
)

//GetVector Gets a vector reading from the specified source
func (bno *BNO055) GetVector(xyz *VecBNO055, vectorType byte) error {
	var (
		x, y, z int16
		buffer  [6]byte
	)
	if err := bno.readlen(vectorType, buffer[:]); err != nil {
		return err
	}

	x = (int16(buffer[1]) << 8) | int16(buffer[0])
	y = (int16(buffer[3]) << 8) | int16(buffer[2])
	z = (int16(buffer[5]) << 8) | int16(buffer[4])

	switch vectorType {
	case VectorMagnetometer:
		/* 1uT = 16 LSB */
		xyz.X = float32(x) / 16.0
		xyz.Y = float32(y) / 16.0
		xyz.Z = float32(z) / 16.0
		break
	case VectorGyroscope:
		/* 1rps = 900 LSB */
		xyz.X = float32(x) / 900.0
		xyz.Y = float32(y) / 900.0
		xyz.Z = float32(z) / 900.0
		break
	case VectorEuler:
		/* 1 degree = 16 LSB */
		xyz.X = float32(x) / 16.0
		xyz.Y = float32(y) / 16.0
		xyz.Z = float32(z) / 16.0
		break
	case VectorAccelerometer, VectorLinearaccel, VectorGravity:
		/* 1m/s^2 = 100 LSB */
		xyz.X = float32(x) / 100.0
		xyz.Y = float32(y) / 100.0
		xyz.Z = float32(z) / 100.0
		break
	default:
		return errors.New("getVector: unknown vectorType")
	}
	return nil
}

// VecQuadBNO055 struct holding quaternion
type VecQuadBNO055 struct {
	x, y, z, w float32
}

//GetQuat gets a quaternion reading from the specified source
func (bno *BNO055) GetQuat(xyzw *VecQuadBNO055) error {
	var (
		x, y, z, w uint16
		buffer     [8]byte
	)
	const scale float32 = (1.0 / (1 << 14))
	/* Read vector data (6 bytes) */
	if err := bno.readlen(bno055QuaternionDataWLsbAddr, buffer[:]); err != nil {
		return err
	}

	w = (uint16(buffer[1]) << 8) | uint16(buffer[0])
	x = (uint16(buffer[3]) << 8) | uint16(buffer[2])
	y = (uint16(buffer[5]) << 8) | uint16(buffer[4])
	z = (uint16(buffer[7]) << 8) | uint16(buffer[6])
	/* Assign to Quaternion */
	/* See http://ae-bst.resource.bosch.com/media/products/dokumente/bno055/BST_BNO055_DS000_12~1.pdf
	3.6.5.5 Orientation (Quaternion)  */
	xyzw.x = scale * float32(x)
	xyzw.y = scale * float32(y)
	xyzw.z = scale * float32(z)
	xyzw.w = scale * float32(w)
	return nil
}

// GetSensorOffsets asd
func (bno *BNO055) GetSensorOffsets(offsets []byte) (bool, error) {
	var caldata CalbrBNO055
	// 22 Offset-Registers starting at accelOffsetXLsbAddr
	if cap(offsets) < 22 {
		return false, errors.New("GetSensorOffsets : cap(offsets) need to be >=22")
	}
	cali, err := bno.IsFullyCalibrated(&caldata)

	if err != nil {
		return false, err
	}

	if !cali {
		return false, nil
	}
	lastMode := bno.OppMode
	bno.setMode(operationModeConfig)

	if err = bno.readlen(accelOffsetXLsbAddr, offsets); err != nil {
		return false, err
	}
	bno.setMode(lastMode)
	return true, nil
}

// SetSensorOffsets blababla
func (bno *BNO055) SetSensorOffsets(offsets []byte) error {
	lastMode := bno.OppMode
	bno.setMode(operationModeConfig)
	err := bno.writelen(accelOffsetXLsbAddr, offsets)
	bno.setMode(lastMode)
	return err
}

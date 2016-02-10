package bno055

const (
	// BNO055's i2c device adress
	bno055AddressA = 0x28
	bno055AddressB = 0x29

	// Constant device identifier
	bno055Id = 0xA0
	// Page id register definition
	bno055PageIDAddr = 0x07

	// PAGE0 REGISTER DEFINITION START
	bno055ChipIDAddr     = 0x00
	bno055AccelRevIDAddr = 0x01
	bno055MagRevIDAddr   = 0x02
	bno055GyroRevIDAddr  = 0x03
	bno055SWRevIDLsbAddr = 0x04
	bno055SWRevIDMsbAddr = 0x05
	bno055BLRevIDAddr    = 0x06

	// Accel data register
	bno055AccelDataXLsbAddr = 0x08
	bno055AccelDataXMsbAddr = 0x09
	bno055AccelDataYLsbAddr = 0x0A
	bno055AccelDataYMsbAddr = 0x0B
	bno055AccelDataZLsbAddr = 0x0C
	bno055AccelDataZMsbAddr = 0x0D

	// Mag data register
	bno055MagDataXLsbAddr = 0x0E
	bno055MagDataXMsbAddr = 0x0F
	bno055MagDataYLsbAddr = 0x10
	bno055MagDataYMsbAddr = 0x11
	bno055MagDataZLsbAddr = 0x12
	bno055MagDataZMsbAddr = 0x13

	// Gyro data registers
	bno055GyroDataXLsbAddr = 0x14
	bno055GyroDataXMsbAddr = 0x15
	bno055GyroDataYLsbAddr = 0x16
	bno055GyroDataYMsbAddr = 0x17
	bno055GyroDataZLsbAddr = 0x18
	bno055GyroDataZMsbAddr = 0x19

	// Euler data registers
	bno055EulerHLsbAddr = 0x1A
	bno055EulerHMsbAddr = 0x1B
	bno055EulerRLsbAddr = 0x1C
	bno055EulerRMsbAddr = 0x1D
	bno055EulerPLsbAddr = 0x1E
	bno055EulerPMsbAddr = 0x1F

	// Quaternion data registers
	bno055QuaternionDataWLsbAddr = 0x20
	bno055QuaternionDataWMsbAddr = 0x21
	bno055QuaternionDataXLsbAddr = 0x22
	bno055QuaternionDataXMsbAddr = 0x23
	bno055QuaternionDataYLsbAddr = 0x24
	bno055QuaternionDataYMsbAddr = 0x25
	bno055QuaternionDataZLsbAddr = 0x26
	bno055QuaternionDataZMsbAddr = 0x27

	// Linear acceleration data registers
	bno055LinearAccelDataXLsbAddr = 0x28
	bno055LinearAccelDataXMsbAddr = 0x29
	bno055LinearAccelDataYLsbAddr = 0x2A
	bno055LinearAccelDataYMsbAddr = 0x2B
	bno055LinearAccelDataZLsbAddr = 0x2C
	bno055LinearAccelDataZMsbAddr = 0x2D

	// Gravity data registers
	bno055GravityDataXLsbAddr = 0x2E
	bno055GravityDataXMsbAddr = 0x2F
	bno055GravityDataYLsbAddr = 0x30
	bno055GravityDataYMsbAddr = 0x31
	bno055GravityDataZLsbAddr = 0x32
	bno055GravityDataZMsbAddr = 0x33

	// Temperature data register
	bno055TempAddr = 0x34

	// Status registers
	bno055CalibStatAddr      = 0x35
	bno055SelftestResultAddr = 0x36
	bno055IntrStatAddr       = 0x37

	bno055SysClkStatAddr = 0x38
	bno055SysStatAddr    = 0x39
	bno055SysErrAddr     = 0x3A

	// Unit selection register
	bno055UnitSelAddr    = 0x3B
	bno055AataSelectAddr = 0x3C

	// Mode registers
	bno055OprModeAddr = 0x3D
	bno055PwrModeAddr = 0x3E

	bno055SysTriggerAddr = 0x3F
	bno055TempSourceAddr = 0x40

	// Axis remap registers
	bno055AxisMapConfigAddr = 0x41
	bno055AxisMapSignAddr   = 0x42

	// SIC registers
	bno055SicMatrix0LsbAddr = 0x43
	bno055SicMatrix0MsbAddr = 0x44
	bno055SicMatrix1LsbAddr = 0x45
	bno055SicMatrix1MsbAddr = 0x46
	bno055SicMatrix2LsbAddr = 0x47
	bno055SicMatrix2MsbAddr = 0x48
	bno055SicMatrix3LsbAddr = 0x49
	bno055SicMatrix3MsbAddr = 0x4A
	bno055SicMatrix4LsbAddr = 0x4B
	bno055SicMatrix4MsbAddr = 0x4C
	bno055SicMatrix5LsbAddr = 0x4D
	bno055SicMatrix5MsbAddr = 0x4E
	bno055SicMatrix6LsbAddr = 0x4F
	bno055SicMatrix6MsbAddr = 0x50
	bno055SicMatrix7LsbAddr = 0x51
	bno055SicMatrix7MsbAddr = 0x52
	bno055SicMatrix8LsbAddr = 0x53
	bno055SicMatrix8MsbAddr = 0x54

	// Accelerometer Offset registers
	accelOffsetXLsbAddr = 0x55
	accelOffsetXMsbAddr = 0x56
	accelOffsetYLsbAddr = 0x57
	accelOffsetYMsbAddr = 0x58
	accelOffsetZLsbAddr = 0x59
	accelOffsetZMsbAddr = 0x5A

	// Magnetometer Offset registers
	magOffsetXLsbAddr = 0x5B
	magOffsetXMsbAddr = 0x5C
	magOffsetYLsbAddr = 0x5D
	magOffsetYMsbAddr = 0x5E
	magOffsetZLsbAddr = 0x5F
	magOffsetZMsbAddr = 0x60

	// Gyroscope Offset registers
	gyroOffsetXLsbAddr = 0x61
	gyroOffsetXMsbAddr = 0x62
	gyroOffsetYLsbAddr = 0x63
	gyroOffsetYMsbAddr = 0x64
	gyroOffsetZLsbAddr = 0x65
	gyroOffsetZMsbAddr = 0x66

	// Radius registers
	accelRadiusLsbAddr = 0x67
	accelRadiusMsbAddr = 0x68
	magRadiusLsbAddr   = 0x69
	magRadiusMsbAddr   = 0x6A

	powerModeNormal   = 0x00
	powerModeLowpower = 0x01
	powerModeSuspend  = 0x02

	// Operation mode settings
	operationModeConfig     = 0x00
	operationModeAcconly    = 0x01
	operationModeMagonly    = 0x02
	operationModeGyronly    = 0x03
	operationModeAccmag     = 0x04
	operationModeAccgyro    = 0x05
	operationModeMaggyro    = 0x06
	operationModeAmg        = 0x07
	operationModeImuplus    = 0x08
	operationModeCompass    = 0x09
	operationModeM4g        = 0x0A
	operationModeNdofFmcOff = 0x0B
	operationModeNdof       = 0x0C

	remapConfigP0 = 0x21
	remapConfigP1 = 0x24 // default
	remapConfigP2 = 0x24
	remapConfigP3 = 0x21
	remapConfigP4 = 0x24
	remapConfigP5 = 0x21
	remapConfigP6 = 0x21
	remapConfigP7 = 0x24

	remapSignP0 = 0x04
	remapSignP1 = 0x00 // default
	remapSignP2 = 0x06
	remapSignP3 = 0x02
	remapSignP4 = 0x03
	remapSignP5 = 0x01
	remapSignP6 = 0x07
	remapSignP7 = 0x05

)

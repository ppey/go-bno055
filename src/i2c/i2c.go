// Package i2c provides low level control over the linux i2c bus.
//
// Before usage you should load the i2c-dev kenel module
//
//      sudo modprobe i2c-dev
//
// Each i2c bus can address 127 independent i2c devices, and most
// linux systems contain several buses.
package i2c

import (
	"encoding/binary"
	"fmt"
	"os"
	"syscall"
)

const (
	i2cSlave = 0x0703
)

// I2C represents a connection to an i2c device.
type I2C struct {
	rc *os.File
}

// SMBus (System Management Bus) protocol over I2C.
// Read unsigned big endian word (16 bits) from i2c device
// starting from address specified in reg.

// NewI2C opens a connection to an i2c device.
func NewI2C(addr uint8, bus int) (*I2C, error) {

	f, err := os.OpenFile(fmt.Sprintf("/dev/i2c-%d", bus), os.O_RDWR, 0600)
	if err != nil {
		return nil, err
	}
	if err := ioctl(f.Fd(), i2cSlave, uintptr(addr)); err != nil {
		return nil, err
	}

	return &I2C{rc: f}, nil
}

// Write sends buf to the remote i2c device. The interpretation of
// the message is implementation dependant.
func (bus *I2C) Write(buf []byte) (int, error) {
	return bus.rc.Write(buf)
}

// WriteByte blaaa
func (bus *I2C) WriteByte(b byte) error {
	var buf [1]byte
	buf[0] = b
	_, err := bus.rc.Write(buf[:])
	return err
}

func (bus *I2C) Read(p []byte) (int, error) {
	return bus.rc.Read(p)
}

// Close blaa
func (bus *I2C) Close() error {
	return bus.rc.Close()
}

// ReadRegU8  SMBus (System Management Bus) protocol over I2C.
// Read byte from i2c device register specified in reg.
func (bus *I2C) ReadRegU8(reg byte) (byte, error) {
	_, err := bus.Write([]byte{reg})
	if err != nil {
		return 0, err
	}
	buf := make([]byte, 1)
	_, err = bus.Read(buf)
	if err != nil {
		return 0, err
	}
	//	log.Debug("Read U8 %d from reg 0x%0X", buf[0], reg)
	return buf[0], nil
}

// WriteRegU8 SMBus (System Management Bus) protocol over I2C.
// Write byte to i2c device register specified in reg.
func (bus *I2C) WriteRegU8(reg byte, value byte) error {
	buf := []byte{reg, value}
	_, err := bus.Write(buf)
	if err != nil {
		return err
	}
	//	log.Debug("Write U8 %d to reg 0x%0X", value, reg)
	return nil
}

// ReadRegU16Le asd
func (bus *I2C) ReadRegU16Le(reg byte) (uint16, error) {
	_, err := bus.Write([]byte{reg})
	if err != nil {
		return 0, err
	}
	// Why not stack ?!?!
	buf := make([]byte, 2)
	_, err = bus.Read(buf)
	if err != nil {
		return 0, err
	}
	w := binary.LittleEndian.Uint16(buf)
	//	log.Debug("Read U16 %d from reg 0x%0X", w, reg)
	return w, nil
}

// WriteRegU16Le blaaa
func (bus *I2C) WriteRegU16Le(reg byte, value uint16) error {
	bufu16 := make([]byte, 2)
	binary.LittleEndian.PutUint16(bufu16, value)

	buf := []byte{reg, bufu16[0], bufu16[1]}
	_, err := bus.Write(buf)
	if err != nil {
		return err
	}
	//	log.Debug("Write U8 %d to reg 0x%0X", value, reg)
	return nil
}

// ReadLen blaaa
func (bus *I2C) ReadLen(reg byte, buf []byte) error {
	_, err := bus.Write([]byte{reg})
	if err != nil {
		return err
	}
	_, err = bus.Read(buf)
	if err != nil {
		return err
	}
	return nil
}

// WriteLen blaa
func (bus *I2C) WriteLen(reg byte, buf []byte) error {
	_, err := bus.Write(append([]byte{reg}, buf...))
	if err != nil {
		return err
	}
	return nil
}

func ioctl(fd, cmd, arg uintptr) error {
	_, _, err := syscall.Syscall6(syscall.SYS_IOCTL, fd, cmd, arg, 0, 0, 0)
	if err != 0 {
		return err
	}
	return nil
}

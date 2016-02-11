// +build ignore

package main

import (
	"bno055"
	"encoding/hex"
	"fmt"
	"i2c"
	"os"
	"os/signal"
	"syscall"
)

func main() {
	fmt.Println("Orientation Sensor Test1")
	const (
		defaultOffsets = "efffb8ff0a00c400c10055ff000000000100e8038c02"
	)
	var (
		status  bno055.StatusBNO055
		caldata bno055.CalbrBNO055
	)

	rawOffsets, err := hex.DecodeString(defaultOffsets)
	if err != nil {
		fmt.Println(err)
		return
	}

	bus, err := i2c.NewI2C(0x28, 1)
	if err != nil {
		fmt.Println(err)
		return
	}

	bno, err := bno055.New(bus)
	if err != nil {
		fmt.Println(err)
		return
	}
	defer bno.Close()

	bno.SetSensorOffsets(rawOffsets[:])

	//  Displays some basic information on this sensor from the unified
	//  sensor API sensor_t type (see Adafruit_Sensor for more information)
	fmt.Println("\n------------Sensor infos------------")
	fmt.Println("Sensor:       ", bno.Name)
	fmt.Println("Driver Ver:   ", bno.Version)
	fmt.Println("Unique ID:    ", bno.SensorID)
	fmt.Println("Max Value:    ", bno.MaxValue)
	fmt.Println("Min Value:    ", bno.MinValue)
	fmt.Println("Resolution:   ", bno.Resolution)

	fmt.Println("")
	bno.GetSystemStatus(&status, true)
	/* Get the system status values (mostly for debugging purposes) */
	fmt.Println("\n------------Sensor Status------------")
	fmt.Printf("System Status : %b\n", status.BnoStatus)
	fmt.Printf("Self Test     : %b\n", status.SelfTestResult)
	fmt.Printf("System Error  : %b\n", status.SystemError)
	fmt.Println("Current Temperature: ", bno.GetTemp())

	cali, err := bno.IsFullyCalibrated(&caldata)

	fmt.Println("\n------------Calibration Status------------")
	fmt.Println("Calibration status values: 0=uncalibrated, 3=fully calibrated")
	for !cali {
		cali, err = bno.IsFullyCalibrated(&caldata)
		if err != nil {
			return
		}
		fmt.Print("\rS : ", caldata.Sys, " G : ", caldata.Gyro, " A : ",
			caldata.Accel, " M : ", caldata.Mag)
		cali, err = bno.GetSensorOffsets(rawOffsets[:])
		bno.Delay()
	}
	fmt.Print("\n\n")

	switch {
	case err != nil:
		fmt.Println("Calibration failed")
	case cali:
		fmt.Println("Bno055 is Calibrated")
		fmt.Printf("Calibration offsets  :  %#x\n", rawOffsets)
	case !cali:
		fmt.Println("Bno055 is not Calibrated")
	}

	/*
		Rotation angle Range (Android format) Range (Windows format)
		Pitch +180° to -180° (turning
		clockwise decreases values)
		-180° to +180° (turing clockwise
		increases values)
		Roll -90° to +90° (increasing with increasing inclination)
		Heading / Yaw 0° to 360° (turning clockwise increases values)
	*/
	sigs := make(chan os.Signal, 1)
	signal.Notify(sigs, syscall.SIGINT, syscall.SIGTERM)

	mesureChan, errorChan := bno.GetListenChan()

	fmt.Println("\n------------EulerAngles------------")
	for {
		select {
		case <-sigs:
			cali, err = bno.GetSensorOffsets(rawOffsets[:])
			fmt.Printf("\n\nCalibration offsets  :  %#x\n", rawOffsets)
			return
		case v := <-mesureChan:
			fmt.Printf("\rHeading:%8.3f \tRoll: %10.3f \tPitch: %10.3f",
				v.Heading, v.Roll, v.Pitch)
		case err := <-errorChan:
			fmt.Println("\n\nSome Error:")
			fmt.Println(err)
			return

		}
	}
}

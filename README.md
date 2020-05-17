# Go DJI Naza 2 Gps Decoder

A serial decoder for the DJI Naza GPS Module written in go. The package is meant to be user on a Raspberry Pi.
The Tx cable of the Naza 2 GPS module needs to be intercepted and connected with the Raspberry Rx port.
 
## Installation

1. `cd ./go/src`
2. `git clone https://github.com/cy8berpunk/goDjiNazaGpsDecoder/`

### Wiring

The wiring is very basic, just connect the Naza GPS module TX with the Raspberry serial RX gpio.

### Note for Raspberry's with Bluetooth

The bluetooth & console serial interfaces need to be disabled in order to make the GPIO serial ports working.
A detailed explanation and tutorial can be found [here](https://www.raspberrypi.org/documentation/configuration/uart.md)

## Example

```go
var sRead djiNazaGpsDecoder.SerialRead
var decInfo djiNazaGpsDecoder.DecodedInformation

djiNazaGpsDecoder.OpenSerial(&sRead, "/dev/serial0")
for {
  djiNazaGpsDecoder.ReadByte(&sRead, &decInfo)
  fmt.Printf("Sats: %d \n", int(decInfo.Satellites))
  fmt.Printf("Heading: %d \n", int(decInfo.Heading))
  fmt.Printf("Alt: %d \n", int(decInfo.Altitude))
  fmt.Printf("Speed: %d \n", int(decInfo.Speed))
  fmt.Printf("Lat: %e \n", decInfo.Latitude)
  fmt.Printf("Lon: %e \n", decInfo.Longitude)
  fmt.Printf("Time: %d, %d, %d, %d, %d, %d \n", int(decInfo.Year), int(decInfo.Month), int(decInfo.Day), int(decInfo.Hour), int(decInfo.Minute), int(decInfo.Second))
  fmt.Printf("HW Version: %d, SW Version: %d \n", int(decInfo.HardwareVersion.Version), int(decInfo.FirmwareVersion.Version))
}
```

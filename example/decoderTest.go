package main

import (
  "fmt"
  "golang-djiNazaGpsDecoder/djiNazaGpsDecoder"
)

func main() {
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
}

package main

import (
  "golang-djiNazaGpsDecoder/djiNazaGpsDecoder"
)

func main() {
  var sRead djiNazaGpsDecoder.SerialRead
  var decInfo djiNazaGpsDecoder.DecodedInformation

  djiNazaGpsDecoder.OpenSerial(&sRead, "/dev/serial0")
  for {
    djiNazaGpsDecoder.ReadByte(&sRead, &decInfo)
    println(int(decInfo.Satellites))
    println(int(decInfo.Heading))
  }
}

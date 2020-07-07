package djiNazaGpsDecoder

import (
  "log"
  "fmt"
  "math"
  "io"
  "goDjiNazaGpsDecoder/go-serial/serial"
)

// https://github.com/jacobsa/go-serial serial lib

const MESSAGE_HEADER_SIZE byte = 0x04
const NAZA_MESSAGE_MAX_PAYLOAD_LENGTH byte = 0x3a

const RAD_TO_DEG float64 = 57.295779513082321

type SerialRead struct
{
  Port io.ReadWriteCloser

  Payload [NAZA_MESSAGE_MAX_PAYLOAD_LENGTH]byte
  Sequence int
  Count int
  MessageId byte
  MessageLength int
  Checksum1 byte
  Checksum2 byte
}

type DecodedInformation struct
{
  MagXMin int16
  MagXMax int16
  MagYMin int16
  MagYMax int16
  Longitude float32
  Latitude float32
  Altitude float64
  Speed float64
  Fix byte
  Satellites byte
  Heading float64
  CourseOverGround float64
  VerticalSpeedIndicator float64
  HorizontalDilutionOfPrecision float64
  VerticalDilutionOfPrecision float64
  Year byte
  Month byte
  Day byte
  Hour byte
  Minute byte
  Second byte
  LastLock byte
  Locked bool
  HardwareVersion VersionType
  FirmwareVersion VersionType
}

// GPSPayloadPositions
const (
  // date and time
  NAZA_MESSAGE_POS_DT byte = 0x04 - MESSAGE_HEADER_SIZE

  // longitude (x10^7, degree decimal)
  NAZA_MESSAGE_POS_LO byte = 0x08 - MESSAGE_HEADER_SIZE

  // latitude (x10^7, degree decimal)
  NAZA_MESSAGE_POS_LA byte = 0x0c - MESSAGE_HEADER_SIZE

  // altitude (in millimeters)
  NAZA_MESSAGE_POS_AL byte = 0x10 - MESSAGE_HEADER_SIZE

  // horizontal accuracy estimate (see uBlox NAV-POSLLH message for details)
  NAZA_MESSAGE_POS_HA byte = 0x14 - MESSAGE_HEADER_SIZE

  // vertical accuracy estimate (see uBlox NAV-POSLLH message for details)
  NAZA_MESSAGE_POS_VA byte = 0x18 - MESSAGE_HEADER_SIZE

  // NED north velocity (see uBlox NAV-VELNED message for details)
  NAZA_MESSAGE_POS_NV byte = 0x20 - MESSAGE_HEADER_SIZE

  // NED east velocity (see uBlox NAV-VELNED message for details)
  NAZA_MESSAGE_POS_EV byte = 0x24 - MESSAGE_HEADER_SIZE

  // NED down velocity (see uBlox NAV-VELNED message for details)
  NAZA_MESSAGE_POS_DV byte = 0x28 - MESSAGE_HEADER_SIZE

  // position DOP (see uBlox NAV-DOP message for details)
  NAZA_MESSAGE_POS_PD byte = 0x2c - MESSAGE_HEADER_SIZE

  // vertical DOP (see uBlox NAV-DOP message for details)
  NAZA_MESSAGE_POS_VD byte = 0x2e - MESSAGE_HEADER_SIZE

  // northing DOP (see uBlox NAV-DOP message for details)
  NAZA_MESSAGE_POS_ND byte = 0x30 - MESSAGE_HEADER_SIZE

  //easting DOP (see uBlox NAV-DOP message for details)
  NAZA_MESSAGE_POS_ED byte = 0x32 - MESSAGE_HEADER_SIZE

  // number of satellites (not XORed)
  NAZA_MESSAGE_POS_NS byte = 0x34 - MESSAGE_HEADER_SIZE

  // fix type (0 - no lock, 2 - 2D lock, 3 - 3D lock, not sure if other values can be expected - see uBlox NAV-SOL message for details)
  NAZA_MESSAGE_POS_FT byte = 0x36 - MESSAGE_HEADER_SIZE

  // fix status flags (see uBlox NAV-SOL message for details)
  NAZA_MESSAGE_POS_SF byte = 0x38 - MESSAGE_HEADER_SIZE

  // XOR mask
  NAZA_MESSAGE_POS_XM byte = 0x3b - MESSAGE_HEADER_SIZE

  // sequence number (not XORed), once there is a lock - increases with every message. When the lock is lost later LSB and MSB are swapped with every message.
  NAZA_MESSAGE_POS_SN byte = 0x3c - MESSAGE_HEADER_SIZE

  // checksum, calculated the same way as for uBlox binary messages
  NAZA_MESSAGE_POS_CS byte = 0x3e - MESSAGE_HEADER_SIZE
)

// MagnetometerPayloadPosition
const (
  // magnetometer X axis data (signed)
  NAZA_MESSAGE_POS_CX byte = 0x04 - MESSAGE_HEADER_SIZE

  // magnetometer Y axis data (signed)
  NAZA_MESSAGE_POS_CY byte = 0x06 - MESSAGE_HEADER_SIZE

  // magnetometer Z axis data (signed)
  NAZA_MESSAGE_POS_CZ byte = 0x08 - MESSAGE_HEADER_SIZE
)

// ModuleVersionPayloadPosition
const (
  // firmware version
  NAZA_MESSAGE_POS_FW byte = 0x08 - MESSAGE_HEADER_SIZE

  // hardware id
  NAZA_MESSAGE_POS_HW byte = 0x0c - MESSAGE_HEADER_SIZE
)

// MessageType
const (
  NAZA_MESSAGE_NONE_TYPE byte = 0x00
  NAZA_MESSAGE_GPS_TYPE byte = 0x10
  NAZA_MESSAGE_MAGNETOMETER_TYPE byte = 0x20
  NAZA_MESSAGE_MODULE_VERSION_TYPE byte = 0x30
)

// MessageSize
const(
  NAZA_MESSAGE_GPS_SIZE byte = 0x3a
  NAZA_MESSAGE_MAGNETOMETER_SIZE byte = 0x06
  NAZA_MESSAGE_MODULE_VERSION_SIZE byte = 0x0c
)

// FixType
const (
  NO_FIX byte = 0x00
  FIX_2D byte = 0x02
  FIX_3D byte = 0x03
  FIX_DGPS byte = 0x04
)

type VersionSchemeType struct
{
  Revision byte
  Build byte
  Minor byte
  Major byte
}

type VersionType struct
{
    Version byte
    Scheme VersionSchemeType
}

func OpenSerial(serialRead *SerialRead, portName string) {

  // Set up options.
  options := serial.OpenOptions{
    PortName: portName,
    BaudRate: 115200,
    DataBits: 8,
    StopBits: 1,
    MinimumReadSize: 4,
		ParityMode: serial.PARITY_NONE,
  }
  var err error

  // Open the port.
  serialRead.Port, err = serial.Open(options)
  serialRead.MessageLength = 0
  if err != nil {
    log.Fatalf("serial.Open: %v", err)
  }
}

func ReadByte(serialRead *SerialRead, deInfo *DecodedInformation) byte {
  rx_buffer := make([]byte, 128)
  n, err := serialRead.Port.Read(rx_buffer)
  if err != nil {
    if err != io.EOF {
  	   fmt.Println("Error reading from serial port: ", err)
		}
  } else if len(rx_buffer) > 0 {
    rx_buffer = rx_buffer[:n]
    for i := 0; i < n; i++ {
      var decodedMessage = decode(serialRead, deInfo, rx_buffer[i])
      if decodedMessage != NAZA_MESSAGE_NONE_TYPE {
        return decodedMessage
      }
    }
  	fmt.Println("Rx: ", rx_buffer)
  }
  return NAZA_MESSAGE_NONE_TYPE
}

func CloseSerial(serialRead SerialRead) {
  // Make sure to close it later.
  defer serialRead.Port.Close()
}

func decode(serialRead *SerialRead, deInfo *DecodedInformation, input byte) byte {
        // header (part 1 - 0x55)
        if ((serialRead.Sequence == 0) && (byte(input) == 0x55)) {
            serialRead.Sequence++
        // header (part 2 - 0xaa)
        } else if ((serialRead.Sequence == 1) && (byte(input) == 0xaa)) {
            serialRead.Checksum1 = 0
            serialRead.Checksum2 = 0
            serialRead.Sequence++
        } else if (serialRead.Sequence == 2) {
            serialRead.MessageId = input
            updateChecksum(serialRead, input)
            serialRead.Sequence++

        // message id
        // message payload length (should match message id)
        // store payload in buffer
        } else if ((byte(serialRead.Sequence) == 3) && (((byte(serialRead.MessageId) == NAZA_MESSAGE_GPS_TYPE) && (byte(input) == NAZA_MESSAGE_GPS_SIZE)) || ((byte(serialRead.MessageId) == NAZA_MESSAGE_MAGNETOMETER_TYPE) && (byte(input) == NAZA_MESSAGE_MAGNETOMETER_SIZE)) || ((byte(serialRead.MessageId) == NAZA_MESSAGE_MODULE_VERSION_TYPE) && (byte(input) == NAZA_MESSAGE_MODULE_VERSION_SIZE)))) {
            serialRead.MessageLength = int(input)
            serialRead.Count = 0
            updateChecksum(serialRead, input)
            serialRead.Sequence++
        } else if (serialRead.Sequence == 4) {
            serialRead.Payload[serialRead.Count] = input
            serialRead.Count += 1
            updateChecksum(serialRead, input)
            if (serialRead.Count >= serialRead.MessageLength) {
                serialRead.Sequence++
            }
        // verify checksum #1
        } else if ((serialRead.Sequence == 5) && (byte(input) == serialRead.Checksum1)) {
            serialRead.Sequence++
        // verify checksum #2
        } else if ((serialRead.Sequence == 6) && (byte(input) == serialRead.Checksum2)) {
            serialRead.Sequence++
        } else {
            serialRead.Sequence = 0
        }

        // all data in buffer
        if (serialRead.Sequence == 7) {
            serialRead.Sequence = 0

            // Decode GPS data
            if (byte(serialRead.MessageId) == NAZA_MESSAGE_GPS_TYPE) {
                var mask = serialRead.Payload[NAZA_MESSAGE_POS_XM]
                var time = pack4FromPayload(serialRead, NAZA_MESSAGE_POS_DT, mask)
                deInfo.Second = byte(time) & 0x3f
                time >>= 6
                deInfo.Minute = byte(time) & 0x3f
                time >>= 6
                deInfo.Hour = byte(time) & 0x0f
                time >>= 4
                deInfo.Day = byte(time) & 0x1f
                time >>= 5
                if (deInfo.Hour > 7) {
                    deInfo.Day++
                }
                deInfo.Month = byte(time) & 0x0f
                time >>= 4
                deInfo.Year = byte(time) & 0x7f
                deInfo.Longitude = float32(pack4Float32FromPayload(serialRead, NAZA_MESSAGE_POS_LO, mask) / 10000000)
                deInfo.Latitude = float32(pack4Float32FromPayload(serialRead, NAZA_MESSAGE_POS_LA, mask) / 10000000)
                deInfo.Altitude =  float64(pack4FromPayload(serialRead, NAZA_MESSAGE_POS_AL, mask) / 1000.0)
                var northVelocity =  pack4FromPayload(serialRead, NAZA_MESSAGE_POS_NV, mask) / 100.0
                var eastVelocity = pack4FromPayload(serialRead, NAZA_MESSAGE_POS_EV, mask) / 100.0
                deInfo.Speed = math.Sqrt(float64(northVelocity * northVelocity + eastVelocity * eastVelocity))
                deInfo.CourseOverGround = math.Atan2(float64(eastVelocity), float64(northVelocity)) * 180.0 / math.Pi
                if (deInfo.CourseOverGround < 0) {
                    deInfo.CourseOverGround += 360.0
                }
                deInfo.VerticalSpeedIndicator = float64(-pack4FromPayload(serialRead, NAZA_MESSAGE_POS_DV, mask) / 100.0)
                deInfo.VerticalDilutionOfPrecision = float64(pack2FromPayload(serialRead, NAZA_MESSAGE_POS_VD, mask) / 100.0)
                var ndop = pack2FromPayload(serialRead, NAZA_MESSAGE_POS_ND, mask) / 100.0
                var edop = pack2FromPayload(serialRead, NAZA_MESSAGE_POS_ED, mask) / 100.0
                deInfo.HorizontalDilutionOfPrecision = float64(math.Sqrt(float64(ndop) * float64(ndop) + float64(edop) * float64(edop)))
                deInfo.Satellites = serialRead.Payload[NAZA_MESSAGE_POS_NS]
                var fixType = serialRead.Payload[NAZA_MESSAGE_POS_FT] ^ mask
                var fixFlags = serialRead.Payload[NAZA_MESSAGE_POS_SF] ^ mask
                switch (fixType) {
                  case 2:
                      deInfo.Fix = FIX_2D
                      break
                  case 3:
                      deInfo.Fix = FIX_3D
                      break
                  default:
                      deInfo.Fix = NO_FIX
                      break
                  }
                if (deInfo.Fix != NO_FIX) && (fixFlags & 0x02) == 1 {
                    deInfo.Fix = FIX_DGPS
                }
                var lock = pack2FromPayload(serialRead, NAZA_MESSAGE_POS_SN, 0x00)
                // go idiomatic
                if lock == int16(deInfo.LastLock + 1) {deInfo.Locked=true} else {deInfo.Locked=false}
                deInfo.LastLock = byte(lock)
            // Decode magnetometer data (not tilt compensated)
            // To calculate the heading (not tilt compensated) you need to do atan2 on the resulting y any a values, convert radians to degrees and add 360 if the result is negative.
            } else if (serialRead.MessageId == NAZA_MESSAGE_MAGNETOMETER_TYPE) {
                var mask = serialRead.Payload[4]
                mask = (((mask ^ (mask >> 4)) & 0x0F) | ((mask << 3) & 0xF0)) ^ (((mask & 0x01) << 3) | ((mask & 0x01) << 7))
                var x = pack2FromPayload(serialRead, NAZA_MESSAGE_POS_CX, mask)
                var y = pack2FromPayload(serialRead, NAZA_MESSAGE_POS_CY, mask)
                if (x > deInfo.MagXMax) {
                    deInfo.MagXMax = x
                }
                if (x < deInfo.MagXMin) {
                    deInfo.MagXMin = x
                }
                if (y > deInfo.MagYMax) {
                    deInfo.MagYMax = y
                }
                if (y < deInfo.MagYMin) {
                    deInfo.MagYMin = y
                }
                deInfo.Heading = computeVectorAngle(y - ((deInfo.MagYMax + deInfo.MagYMin) / 2), x - ((deInfo.MagXMax + deInfo.MagXMin) / 2))
            } else if (serialRead.MessageId == NAZA_MESSAGE_MODULE_VERSION_TYPE) {
                deInfo.FirmwareVersion.Version = byte(pack4FromPayload(serialRead, NAZA_MESSAGE_POS_FW, 0x00))
                deInfo.HardwareVersion.Version = byte(pack4FromPayload(serialRead, NAZA_MESSAGE_POS_HW, 0x00))
            }
            return serialRead.MessageId
        } else {
            return NAZA_MESSAGE_NONE_TYPE
        }
    }

    func int32Conv(b [4]byte) rune {
        return int32(b[0]) | int32(b[1])<<8 | int32(b[2])<<16 | int32(b[3])<<24
    }


    func float32Conv(b [4]byte) float32 {
        return float32(int32(b[0]) | int32(b[1])<<8 | int32(b[2])<<16 | int32(b[3])<<24)
    }

    func int16Conv(b [2]byte) int16 {
        return int16(b[0]) | int16(b[1])<<8
    }

    func pack4Float32FromPayload(serialRead *SerialRead, i byte, mask byte) float32 {
        var b[4] byte
        for j := 0; j < 4; j++ {
          b[j] = serialRead.Payload[int(i) + j] ^ mask
        }
        return float32Conv(b)
    }

    func pack4FromPayload(serialRead *SerialRead, i byte, mask byte) rune {
        var b[4] byte
        for j := 0; j < 4; j++ {
          b[j] = serialRead.Payload[int(i) + j] ^ mask
        }
        return int32Conv(b)
    }

    func pack2FromPayload(serialRead *SerialRead, i byte, mask byte) int16 {
        var b[2] byte
        for j := 0; j < 2; j++ {
            b[j] = serialRead.Payload[int(i) + j] ^ mask
        }
        return int16Conv(b)
    }

    func updateChecksum(serialRead *SerialRead, input byte) {
        serialRead.Checksum1 += input
        serialRead.Checksum2 += serialRead.Checksum1
    }

    func radiansToDegrees(radians float64) float64 {
        return radians * RAD_TO_DEG
    }

    func computeVectorAngle(y int16, x int16) float64 {
      var degrees = radiansToDegrees(-math.Atan2(float64(y), float64(x)))

      if degrees < 0 {
          degrees += 360.0
      }

      if degrees > 360.0 {
          degrees -= 360.0
      }
      return degrees
    }

package djiNazaGpsDecoder

import (
  "log"
  "fmt"
  "math"
  "io"
  "golang-djiNazaGpsDecoder/go-serial/serial"
)

// https://github.com/jacobsa/go-serial serial lib

const MESSAGE_HEADER_SIZE byte = 0x04
const NAZA_MESSAGE_MAX_PAYLOAD_LENGTH byte = 0x3a

const RAD_TO_DEG float64 = 57.295779513082321

type serialRead struct
{
  port io.ReadWriteCloser

  payload [NAZA_MESSAGE_MAX_PAYLOAD_LENGTH]byte
  sequence int
  count int
  messageId byte
  messageLength int
  checksum1 byte
  checksum2 byte
}

type decodedInformation struct
{
  magXMin int16
  magXMax int16
  magYMin int16
  magYMax int16
  longitude float64
  latitude float64
  altitude float64
  speed float64
  fix byte
  satellites byte
  heading float64
  courseOverGround float64
  verticalSpeedIndicator float64
  horizontalDilutionOfPrecision float64
  verticalDilutionOfPrecision float64
  year byte
  month byte
  day byte
  hour byte
  minute byte
  second byte
  lastLock byte
  locked byte
  hardwareVersion VersionType
  firmwareVersion VersionType
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
  revision byte
  build byte
  minor byte
  major byte
}

type VersionType struct
{
    version byte
    scheme VersionSchemeType
}

func OpenSerial(serialRead serialRead, portName string, baudRate uint) {

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
  serialRead.port, err = serial.Open(options)

  if err != nil {
    log.Fatalf("serial.Open: %v", err)
  }
}

func ReadByte(serialRead serialRead, deInfo decodedInformation) byte {
  rx_buffer := make([]byte, 128)
  n, err := serialRead.port.Read(rx_buffer)
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

func CloseSerial(serialRead serialRead) {
  // Make sure to close it later.
  defer serialRead.port.Close()
}

func decode(serialRead serialRead, deInfo decodedInformation, input byte) byte {
        // header (part 1 - 0x55)
        if ((serialRead.sequence == 0) && (byte(input) == 0x55)) {
            serialRead.sequence++
        // header (part 2 - 0xaa)
        } else if ((serialRead.sequence == 1) && (byte(input) == 0xaa)) {
            serialRead.checksum1 = 0
            serialRead.checksum2 = 0
            serialRead.sequence++
        } else if (serialRead.sequence == 2) {
            serialRead.messageId = input
            updateChecksum(serialRead, input)
            serialRead.sequence++

        // message id
        // message payload length (should match message id)
        // store payload in buffer
        } else if ((byte(serialRead.sequence) == 3) && (((byte(serialRead.messageId) == NAZA_MESSAGE_GPS_TYPE) && (byte(input) == NAZA_MESSAGE_GPS_SIZE)) || ((byte(serialRead.messageId) == NAZA_MESSAGE_MAGNETOMETER_TYPE) && (byte(input) == NAZA_MESSAGE_MAGNETOMETER_SIZE)) || ((byte(serialRead.messageId) == NAZA_MESSAGE_MODULE_VERSION_TYPE) && (byte(input) == NAZA_MESSAGE_MODULE_VERSION_SIZE)))) {
            serialRead.messageLength = int(input)
            serialRead.count = 0
            updateChecksum(serialRead, input)
            serialRead.sequence++
        } else if (serialRead.sequence == 4) {
            serialRead.count += 1
            serialRead.payload[serialRead.count] = input
            updateChecksum(serialRead, input)
            if (serialRead.count >= serialRead.messageLength) {
                serialRead.sequence++
            }
        // verify checksum #1
        } else if ((serialRead.sequence == 5) && (byte(input) == serialRead.checksum1)) {
            serialRead.sequence++
        // verify checksum #2
        } else if ((serialRead.sequence == 6) && (byte(input) == serialRead.checksum2)) {
            serialRead.sequence++
        } else {
            serialRead.sequence = 0
        }

        // all data in buffer
        if (serialRead.sequence == 7) {
            serialRead.sequence = 0

            // Decode GPS data
            if (byte(serialRead.messageId) == NAZA_MESSAGE_GPS_TYPE) {
                var mask = serialRead.payload[NAZA_MESSAGE_POS_XM]
                var time = pack4(serialRead, NAZA_MESSAGE_POS_DT, mask)
                deInfo.second = byte(time) & 0x3f
                time >>= 6
                deInfo.minute = byte(time) & 0x3f
                time >>= 6
                deInfo.hour = byte(time) & 0x0f
                time >>= 4
                deInfo.day = byte(time) & 0x1f
                time >>= 5
                if (deInfo.hour > 7) {
                    deInfo.day++
                }
                deInfo.month = byte(time) & 0x0f
                time >>= 4
                deInfo.year = byte(time) & 0x7f
                deInfo.longitude = float64(pack4(serialRead, NAZA_MESSAGE_POS_LO, mask) / 10000000.0)
                deInfo.latitude = float64(pack4(serialRead, NAZA_MESSAGE_POS_LA, mask) / 10000000.0)
                deInfo.altitude =  float64(pack4(serialRead, NAZA_MESSAGE_POS_AL, mask) / 1000.0)
                var northVelocity =  pack4(serialRead, NAZA_MESSAGE_POS_NV, mask) / 100.0
                var eastVelocity = pack4(serialRead, NAZA_MESSAGE_POS_EV, mask) / 100.0
                deInfo.speed = math.Sqrt(float64(northVelocity * northVelocity + eastVelocity * eastVelocity))
                deInfo.courseOverGround = math.Atan2(float64(eastVelocity), float64(northVelocity)) * 180.0 / math.Pi
                if (deInfo.courseOverGround < 0) {
                    deInfo.courseOverGround += 360.0
                }
                deInfo.verticalSpeedIndicator = float64(-pack4(serialRead, NAZA_MESSAGE_POS_DV, mask) / 100.0)
                deInfo.verticalDilutionOfPrecision = float64(pack2(serialRead, NAZA_MESSAGE_POS_VD, mask) / 100.0)
                var ndop = pack2(serialRead, NAZA_MESSAGE_POS_ND, mask) / 100.0
                var edop = pack2(serialRead, NAZA_MESSAGE_POS_ED, mask) / 100.0
                deInfo.horizontalDilutionOfPrecision = float64(math.Sqrt(float64(ndop) * float64(ndop) + float64(edop) * float64(edop)))
                deInfo.satellites = serialRead.payload[NAZA_MESSAGE_POS_NS]
                var fixType = serialRead.payload[NAZA_MESSAGE_POS_FT] ^ mask
                var fixFlags = serialRead.payload[NAZA_MESSAGE_POS_SF] ^ mask
                switch (fixType) {
                  case 2:
                      deInfo.fix = FIX_2D
                      break
                  case 3:
                      deInfo.fix = FIX_3D
                      break
                  default:
                      deInfo.fix = NO_FIX
                      break
                  }
                // if ((deInfo.fix != NO_FIX) && (fixFlags & 0x02)) {
                //     deInfo.fix = FIX_DGPS
                // }
                var lock = pack2(serialRead, NAZA_MESSAGE_POS_SN, 0x00)
                var locked = (lock == int16(deInfo.lastLock + 1))
                deInfo.lastLock = byte(lock)
            // Decode magnetometer data (not tilt compensated)
            // To calculate the heading (not tilt compensated) you need to do atan2 on the resulting y any a values, convert radians to degrees and add 360 if the result is negative.
            } else if (serialRead.messageId == NAZA_MESSAGE_MAGNETOMETER_TYPE) {
                var mask = serialRead.payload[4]
                mask = (((mask ^ (mask >> 4)) & 0x0F) | ((mask << 3) & 0xF0)) ^ (((mask & 0x01) << 3) | ((mask & 0x01) << 7))
                var x = pack2(serialRead, NAZA_MESSAGE_POS_CX, mask)
                var y = pack2(serialRead, NAZA_MESSAGE_POS_CY, mask)
                if (x > deInfo.magXMax) {
                    deInfo.magXMax = x
                }
                if (x < deInfo.magXMin) {
                    deInfo.magXMin = x
                }
                if (y > deInfo.magYMax) {
                    deInfo.magYMax = y
                }
                if (y < deInfo.magYMin) {
                    deInfo.magYMin = y
                }
                deInfo.heading = computeVectorAngle(y - ((deInfo.magYMax + deInfo.magYMin) / 2), x - ((deInfo.magXMax + deInfo.magXMin) / 2))
            } else if (serialRead.messageId == NAZA_MESSAGE_MODULE_VERSION_TYPE) {
                deInfo.firmwareVersion.version = byte(pack4(serialRead, NAZA_MESSAGE_POS_FW, 0x00))
                deInfo.hardwareVersion.version = byte(pack4(serialRead, NAZA_MESSAGE_POS_HW, 0x00))
            }
            return serialRead.messageId
        } else {
            return NAZA_MESSAGE_NONE_TYPE
        }
    }

    func pack4(serialRead serialRead, i byte, mask byte) int32 {
        var b[4] byte
        for j := 0; j < 4; j++ {
          b[j] = serialRead.payload[int(i) + j] ^ mask
        }
        return int32(b)
    }

    func pack2(serialRead serialRead, i byte, mask byte) int16 {
        var b[2] byte
        for j := 0; j < 2; j++ {
            b[j] = serialRead.payload[int(i) + j] ^ mask
        }
        return int16(b)
    }

    func updateChecksum(serialRead serialRead, input byte) {
        serialRead.checksum1 += input
        serialRead.checksum2 += serialRead.checksum1
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

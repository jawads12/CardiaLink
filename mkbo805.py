import serial
import time

# Open the serial port
with serial.Serial('COM8', 115200, bytesize=serial.EIGHTBITS,
                   parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                   timeout=1) as ser:
    print("Connected to", ser.port)
    print("Press Ctrl + C to stop.\n")

    while True:
        # Send the read command
        ser.write(bytes([0xFD, 0x00, 0x00, 0x00, 0x00, 0x00]))
        rx = ser.read(6)

        if len(rx) != 6 or rx[0] != 0xFD:
            print("Bad response or timeout:", rx.hex(' ').upper())
        else:
            hp, lp, hr = rx[1], rx[2], rx[3]
            print(f"Raw: {rx.hex(' ').upper()}")
            print(f"High pressure: {hp}")
            print(f"Low pressure:  {lp}")
            print(f"Heart rate:    {hr}")
            print("-" * 30)

        # wait a bit before next read
        time.sleep(1)

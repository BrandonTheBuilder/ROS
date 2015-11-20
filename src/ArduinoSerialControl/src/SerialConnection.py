import serial

class SerialControl:
    def SerialControl():
        self.ser = serial.Serial('/dev/ttyACM0',9600)

            

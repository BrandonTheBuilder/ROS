import serial

class Arduino:
    def Arduino(self):
        self.con = serial.Serial('/dev/ttyACM0', 9600)
        self.pins = dict() 

    def initPin(self, pin, io):
        message = 'i'+'i'+ chr(pin)+io
        self.con.write(message)
        return self.getResponse()

    def readdPin(self, pin, ad):
        message = 'a' + 'a' + chr(pin) + 'a'
        self.con.write(message)
        return self.getResponse()

    def writePin(self, pin, hl):
        message = 'a' + 'w' + chr(pin) + hl
        self.con.write(message)
        return self.getResponse()

    def getResponse(self):
        char = ''
        response = ''
        while char is not chr(4):
            response += char
            char = self.con.read()
        return response

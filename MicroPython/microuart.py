import __main__


from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

from pybricks.iodevices import UARTDevice

# u=UARTDevice(Port.A,timeout=1000)
# u.set_baudrate(115200)

PREAMBLE = b'<$MU'

class MicroUART:
    def __init__(self,port,baudrate=115200,wait_recv=1000,uart_timeout=1000):
        self.byte_timeout = 10
        self.wait_recv = wait_recv
        self.uart = UARTDevice(port,timeout=uart_timeout)
        self.uart.set_baudrate(baudrate)
    
    def flush(self):
        while self.uart.waiting()>0:
            self.uart.read_all()

    def send_bytes(self,b):
        b=PREAMBLE+b
        b=bytes([len(b)])+b
        #rint(b)
        self.uart.write(b)

    def receive_bytes(self):
        s=StopWatch()
        while (s.time()<self.wait_recv) and self.uart.waiting()==0:
            pass
        if self.uart.waiting() > 0:
            l=self.uart.read(1)[0]
        else:
            self.flush()
            return b'' #1,None
        #print(l,"u.waiting",self.uart.waiting())
        # if start reading all is fatsre than sending the bytes,
            # we will miss sime bytes.
            # therefore collect bytes during a time window
        payload = bytearray()
        total_sw = StopWatch()     # overall timeout
        byte_sw = StopWatch()      # inter-byte timeout
        i=0
        while len(payload) < l:

            # overall timeout
            if total_sw.time() > self.wait_recv:
                self.flush()
                return b'' #2, None     # total timeout

            if self.uart.waiting():
                b = self.uart.read(1)
                if b:
                    payload.append(b[0])
                    if i<4:
                        if b[0]!=PREAMBLE[i]:
                            self.flush()
                            return b'' #4, None
                    i+=1
                    byte_sw.reset()
            else:
                # inter-byte timeout
                if byte_sw.time() > self.byte_timeout:
                    self.flush()
                    return b'' #3, None
                pass

        return bytes(payload[4:])


    def encode(self, cmd, *argv):
        encoded=bytes([len(cmd)])+bytes(cmd,'utf-8')
        for arg in argv:
            if type(arg)==int:
                s=str(arg)
                encoded+=bytes([78,len(s)])+bytes(s,'utf-8')
            if type(arg)==bytes:
                encoded+=bytes([65,len(arg)])+arg
            if type(arg)==str:
                encoded+=bytes([83,len(arg)])+bytes(arg,'utf-8')
            if type(arg)==bool:
                b= 1 if arg==True else 0
                encoded+=bytes([66,1,b])
        return encoded

    def decode(self, encoded: bytes):
        len_cmd=encoded[0]
        cmd = str(encoded[1:len_cmd+1],'utf-8')
        decoded = []
        p = len_cmd+1
        while p < len(encoded):
            t = encoded[p]
            length = encoded[p + 1]
            p += 2

            payload = encoded[p:p + length]
            p += length
            if t == 78:          # N int
                decoded.append(int(str(payload,'utf-8')))
            elif t == 65:        # A bytes
                decoded.append(payload)
            elif t == 83:        # S str
                decoded.append(str(payload,'utf-8'))
            elif t == 66:        # B bool
                decoded.append(bool(payload[0]))
            else:
                raise ValueError(f"Unknown type code: {t}")

        return cmd,decoded


    def send_command(self,cmd,*data):
        b=self.encode(cmd,*data)
        #print('b encoded',b)
        self.send_bytes(b)
        
    def receive_command(self):
        b=self.receive_bytes()
        #print('recv serial',b)
        if len(b)>0:
            try:
                cmd,data=self.decode(b)
                return cmd,data
            except:
                self.flush()
                return "!ERROR","no bytes received"
        else:
            return "!ERROR","no bytes received"



    def call(self,cmd,*data):
        self.send_command(cmd,*data)
        self.flush()
        return self.receive_command()

    def process(self):
        cmd, data = self.receive_command()
        if cmd != '!ERROR':
            if cmd in dir(__main__):
                funct = eval(cmd)
                resp = funct(*data)
                self.send_command(cmd+'_ack',*resp)
        else:
            self.send_command(cmd+'_err','recv error')
        

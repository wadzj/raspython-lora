# python 3.7

import defs
import spidev
import time
# import ctypes

import RPi.GPIO as GPIO

import pdb



class LoRa:

    def __init__(self, cs, rst=None, dio0=None):
        self._cs = 22
        self._rst = rst
        self._dio0 = dio0
        
       
   

    
        self._frequency = 0
        self._packetIndex = 0
        self._implicitHeaderMode = 0
        self._onReceive = None
        self._onTxDone = None
    #    overide Stream timeout value

    #  frequency is a long, returns an int
    def begin(self, frequency):
        GPIO.setmode(GPIO.BCM) 

       
        if self._rst is not None:
            GPIO.setup(self._rst, GPIO.OUT)
            # perform reset
            GPIO.output(self._rst, GPIO.LOW)
            time.sleep(0.01) 
            GPIO.output(self._rst, GPIO.HIGH)
            time.sleep(0.01) 

        self._spi = spidev.SpiDev(0,0)
        self._spi.max_speed_hz = 976000
        self._spi.mode = 0

        # start SPI
        if self._cs == 0 or self._cs == 1:
            try:
                self._spi.open(0,self._cs)
            except Exception as e:
                print(e)
                self._spi.close() 
                exit()
        else:
            pass
            self._spi.no_cs = True
            GPIO.setup(self._cs, GPIO.OUT) 
            try:
                self._spi.open(0,0)
            except Exception as e:
                print(e)
                self._spi.close() # this is crucial to catch, otherwise we won't be able to start again next time
                exit()

        # The callback will run as soon as it is set up
        if self._dio0 is not None:
            GPIO.setup(self._dio0, GPIO.IN, pull_up_down=GPIO.PUD_UP)
          
    
       
        # check version
        version = self.__readRegister(defs.REG_VERSION) 
        if (version != 0x12):
            return 0 

        # put in sleep mode
        self.sleep() 

        # set frequency
        self.setFrequency(frequency) 

        # set base addresses
        self.__writeRegister(defs.REG_FIFO_TX_BASE_ADDR, 0) 
        self.__writeRegister(defs.REG_FIFO_RX_BASE_ADDR, 0) 

        # set LNA boost
        self.__writeRegister(defs.REG_LNA, self.__readRegister(defs.REG_LNA) | 0x03) 

        # set auto AGC
        self.__writeRegister(defs.REG_MODEM_CONFIG_3, 0x04) 

        # set output power to 17 dBm
        self.setTxPower(17, defs.PA_OUTPUT_PA_BOOST_PIN) 

        # put in standby mode
        self.idle() 
        return 1 

    # Working
    def end(self):
        # put in sleep mode
        self.sleep() 

        # stop SPI
        self._spi.close() 
        GPIO.cleanup()


    def detectCad(self, timeout:int = 1000):
        # clear IRQ's
        self.__writeRegister(defs.REG_IRQ_FLAGS, defs.IRQ_CAD_DONE_MASK | defs.IRQ_CAD_DETECTED_MASK);

        # set mode to CAD
        modeState = self._readRegister(defs.REG_OP_MODE);
        self.__writeRegister(defs.REG_OP_MODE, defs.MODE_STDBY);

        self.__writeRegister(defs.REG_OP_MODE, defs.MODE_CAD);

        # stall till timeout
        timeout_count = 0;
        while not (self.__readRegister(defs.REG_IRQ_FLAGS) & defs.IRQ_CAD_DONE_MASK ) and (timeout_count < timeout):
            time.sleep(0.01)
            timeout_count += 1

        # restore the mode here??
        self.__writeRegister(defs.REG_OP_MODE, modeState);

        if(self.__readRegister(defs.REG_IRQ_FLAGS) & defs.IRQ_CAD_DETECTED_MASK):
            # clear the interrupt
            self.__writeRegister(defs.REG_IRQ_FLAGS, defs.IRQ_CAD_DONE_MASK | defs.IRQ_CAD_DETECTED_MASK);
            # put in rx mode
            self.__writeRegister(defs.REG_OP_MODE, defs.MODE_RX_CONTINUOUS);
            return 1
        # we timed out without finishing cad, go ahead and send anyway?
        elif (timeout_count == timeout):
            return 0

        # no cad detected, free to send
        return 0



    # takes int, returns int
    def beginPacket(self):
        if (self.isTransmitting()):
            return 0 

        # put in standby mode
        self.idle() 

        if (self._implicitHeaderMode == 1):
            self.implicitHeaderMode() 
        else: 
            self.explicitHeaderMode() 

        # reset FIFO address and paload length
        self.__writeRegister(defs.REG_FIFO_ADDR_PTR, 0) 
        self.__writeRegister(defs.REG_PAYLOAD_LENGTH, 0) 

        return 1 

    #int return, bool in
    def endPacket(self,asynch):
        if ((asynch) and (self._onTxDone)):
            self.__writeRegister(defs.REG_DIO_MAPPING_1, 0x40) # DIO0 => TXDONE

        # put in TX mode
        self.__writeRegister(defs.REG_OP_MODE, defs.MODE_LONG_RANGE_MODE | defs.MODE_TX) 

        if (not asynch):
        # wait for TX done
            while ((self.__readRegister(defs.REG_IRQ_FLAGS) & defs.IRQ_TX_DONE_MASK) == 0):
                yield()     # have to figure this out...
            # clear IRQ's
            self.__writeRegister(defs.REG_IRQ_FLAGS, defs.IRQ_TX_DONE_MASK) 

        return 1 

    #bool return
    def isTransmitting(self):
        if ((self.__readRegister(defs.REG_OP_MODE) & defs.MODE_TX) == defs.MODE_TX):
            return True 

        if (self.__readRegister(defs.REG_IRQ_FLAGS) & defs.IRQ_TX_DONE_MASK):
            # clear IRQ's
            self.__writeRegister(defs.REG_IRQ_FLAGS, defs.IRQ_TX_DONE_MASK) 
        return False 

    #int return, int in
    def parsePacket(self, size:int = 0):
        packetLength = 0 
        # self._packetIndex = 0
        irqFlags = self.__readRegister(defs.REG_IRQ_FLAGS) 

        if (size > 0):
            self.implicitHeaderMode() 
            self.__writeRegister(defs.REG_PAYLOAD_LENGTH, size & 0xff) 
        else:
            self.explicitHeaderMode() 


        # clear IRQ's
        self.__writeRegister(defs.REG_IRQ_FLAGS, irqFlags) 

        if ((irqFlags & defs.IRQ_RX_DONE_MASK) and not (irqFlags & defs.IRQ_PAYLOAD_CRC_ERROR_MASK) ):
            # received a packet
            self._packetIndex = 0 

            # read packet length
            if self._implicitHeaderMode:
                packetLength = self.__readRegister(defs.REG_PAYLOAD_LENGTH) 
            else:
                packetLength = self.__readRegister(defs.REG_RX_NB_BYTES) 

            # set FIFO address to current RX address
            self.__writeRegister(defs.REG_FIFO_ADDR_PTR, self.__readRegister(defs.REG_FIFO_RX_CURRENT_ADDR)) 

            # put in standby mode
            self.idle() 
        elif (self.__readRegister(defs.REG_OP_MODE) != (defs.MODE_LONG_RANGE_MODE | defs.MODE_RX_SINGLE) ):
            # not currently in RX mode
            # reset FIFO address
            self.__writeRegister(defs.REG_FIFO_ADDR_PTR, 0) 
            # put in single RX mode
            self.__writeRegister(defs.REG_OP_MODE, defs.MODE_LONG_RANGE_MODE | defs.MODE_RX_SINGLE) 
        return packetLength 

    #int return
    def packetRssi(self):
        return (self.__readRegister(defs.REG_PKT_RSSI_VALUE) - (164 if self._frequency < 868E6 else 157)) 

    #float return 
    def packetSnr(self):
        return (self.__readRegister(defs.REG_PKT_SNR_VALUE)) * 0.25 

    #long 
    def packetFrequencyError(self):
        freqError = 0 
        freqError = (self.__readRegister(defs.REG_FREQ_ERROR_MSB) & B111) 
        freqError <<= 8 
        freqError += (self.__readRegister(defs.REG_FREQ_ERROR_MID)) 
        freqError <<= 8 
        freqError += (self.__readRegister(defs.REG_FREQ_ERROR_LSB)) 

        if (self.__readRegister(defs.REG_FREQ_ERROR_MSB) & B1000 ):# Sign bit is on
            freqError -= 524288 # B1000'0000'0000'0000'0000

        fXtal = 32E6 # FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
        fError = (((freqError) * (1 << 24)) / fXtal) * (self.getSignalBandwidth() / 500000.0) # p. 37

        return (fError) 

    #size_t return, uint8 in
    def write(self, byte):
        return write(byte, sizeof(byte)) 

    #TODO: This should be changed to use bytearray with automatic size calculation
    #size_t return, buffer pointer and int size in
    def write(self, buffer):
        pkt = bytearray(buffer, 'utf-8')
        size = len(pkt)
        currentLength = self.__readRegister(defs.REG_PAYLOAD_LENGTH) 
        # check size
        if ((currentLength + size) > defs.MAX_PKT_LENGTH ):
            size = defs.MAX_PKT_LENGTH - currentLength 
        # write data
        for i in range(0,size):
            self.__writeRegister(defs.REG_FIFO, pkt[i]) 
        # update length
        self.__writeRegister(defs.REG_PAYLOAD_LENGTH, currentLength + size) 

        return size 

    #int 
    def available(self):
        return (self.__readRegister(defs.REG_RX_NB_BYTES) -self._packetIndex) 

    #int 
    def read(self):
        if (not self.available()):
            return -1 
        self._packetIndex = self._packetIndex + 1 
        return self.__readRegister(defs.REG_FIFO) 

    #int 
    def peek():
        if (not self.available()):
            return -1 
        # store current FIFO address
        currentAddress = self.__readRegister(defs.REG_FIFO_ADDR_PTR) 
        # read
        b = self.__readRegister(defs.REG_FIFO) 
        # restore FIFO address
        self.__writeRegister(defs.REG_FIFO_ADDR_PTR, currentAddress) 
        return b 

    #TODO: Figure out how to handle callbacks

    def onReceive(self, callback = None):
        self._onReceive = callback 

        if not self._dio0 or self._onTxDone:
            return

        if callback:
            self.__writeRegister(defs.REG_DIO_MAPPING_1, 0x00) # DIO0 => RXDONE
            GPIO.add_event_detect(self._dio0, GPIO.RISING, callback=self.handleDio0Rise, bouncetime=50)  
        else:
            GPIO.remove_event_detect(self._dio0)

    def onTxDone(self, callback = None):
        self._onTxDone = callback 

        if not self._dio0 or self._onReceive:
            return

        if callback:
            GPIO.add_event_detect(self._dio0, GPIO.RISING, callback=self.handleDio0Rise, bouncetime=50)  
        else: 
            GPIO.remove_event_detect(self._dio0)

    # takes int, returns nothing
    def receive(self, size: int = 0):
        if (size > 0):
            self.implicitHeaderMode() 
            self.__writeRegister(defs.REG_PAYLOAD_LENGTH, size & 0xff) 
        else:
            self.explicitHeaderMode() 

        self.__writeRegister(defs.REG_OP_MODE, defs.MODE_LONG_RANGE_MODE | defs.MODE_RX_CONTINUOUS) 

    # send a packet over the wire
    def transmit(self, packet):
        if self.beginPacket():
            self.write(packet)
            self.endPacket(False)
            return True
        return False

    # returns void 
    def idle(self):
        self.__writeRegister(defs.REG_OP_MODE, defs.MODE_LONG_RANGE_MODE | defs.MODE_STDBY) 

    # returns void 
    def sleep(self):
        self.__writeRegister(defs.REG_OP_MODE, defs.MODE_LONG_RANGE_MODE | defs.MODE_SLEEP) 

    # returns void, takes two ints
    def setTxPower(self, level, outputPin):
        if (defs.PA_OUTPUT_RFO_PIN == outputPin):
        # RFO
            if (level < 0):
                level = 0 
            elif (level > 14):
                level = 14 

            self.__writeRegister(defs.REG_PA_CONFIG, 0x70 | level) 
        else:
        # PA BOOST
            if (level > 17):
                if (level > 20):
                    level = 20 

                # subtract 3 from level, so 18 - 20 maps to 15 - 17
                level -= 3 

                # High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
                self.__writeRegister(defs.REG_PA_DAC, 0x87) 
                self.setOCP(140) 
            else:
                if (level < 2):
                    level = 2 
                #$Default value PA_HF/LF or +17dBm
                self.__writeRegister(defs.REG_PA_DAC, 0x84) 
                self.setOCP(100) 

            self.__writeRegister(defs.REG_PA_CONFIG, defs.PA_BOOST | (level - 2)) 

    # returns void, takes a long
    def setFrequency(self, frequency):
        frequency = int(frequency)
        _frequency = frequency 
        frf = int((frequency << 19) / 32000000)

        self.__writeRegister(defs.REG_FRF_MSB, (frf >> 16)) 
        self.__writeRegister(defs.REG_FRF_MID, (frf >> 8)) 
        self.__writeRegister(defs.REG_FRF_LSB, (frf >> 0)) 

    #int 
    def getSpreadingFactor(self):
        return self.__readRegister(defs.REG_MODEM_CONFIG_2) >> 4 

    # returns void, takes int 
    def setSpreadingFactor(self, sf):
        if (sf < 6):
            sf = 6 
        elif (sf > 12):
            sf = 12 

        if (sf == 6):
            self.__writeRegister(defs.REG_DETECTION_OPTIMIZE, 0xc5) 
            self.__writeRegister(defs.REG_DETECTION_THRESHOLD, 0x0c) 
        else:
            self.__writeRegister(defs.REG_DETECTION_OPTIMIZE, 0xc3) 
            self.__writeRegister(defs.REG_DETECTION_THRESHOLD, 0x0a) 

        self.__writeRegister(defs.REG_MODEM_CONFIG_2, (self.__readRegister(defs.REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0)) 
        self.setLdoFlag() 

    #long 
    def getSignalBandwidth(self):
        bw = (self.__readRegister(defs.REG_MODEM_CONFIG_1) >> 4) 
        lut = [ 7.8E3 , 10.4E3 , 15.6E3 , 20.8E3 , 31.25E3 , 41.7E3 , 62.5E3 , 125E3 , 250E3 , 500E3]
        return lut[bw]

    # returns void, takes long
    def setSignalBandwidth(self, sbw):
        bw = 0 

        if (sbw <= 7.8E3):
            bw = 0 
        elif (sbw <= 10.4E3):
            bw = 1 
        elif (sbw <= 15.6E3):
            bw = 2 
        elif (sbw <= 20.8E3):
            bw = 3 
        elif (sbw <= 31.25E3):
            bw = 4 
        elif (sbw <= 41.7E3):
            bw = 5 
        elif (sbw <= 62.5E3):
            bw = 6 
        elif (sbw <= 125E3):
            bw = 7 
        elif (sbw <= 250E3):
            bw = 8 
        else:
            bw = 9 

        self.__writeRegister(defs.REG_MODEM_CONFIG_1, (self.__readRegister(defs.REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4)) 
        self.setLdoFlag() 

    # returns void 

    #define bitRead(value, bit) (((value) >> (bit)) & 0x01)
    #define bitSet(value, bit) ((value) |= (1UL << (bit)))
    #define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
    def bitWrite(self, value, bit, bitvalue):
        return ((1 << bit) | value) if bitvalue == 1 else ( ~(1 << bit) & value)



    def setLdoFlag(self):
        # Section 4.1.1.5
        symbolDuration = 1000 / ( self.getSignalBandwidth() / (1 << self.getSpreadingFactor()) )  

        # Section 4.1.1.6
        ldoOn = symbolDuration > 16 

        config3 = self.__readRegister(defs.REG_MODEM_CONFIG_3) 
        self.bitWrite(config3, 3, ldoOn) 
        self.__writeRegister(defs.REG_MODEM_CONFIG_3, config3) 

    # returns void, takes int
    def setCodingRate4(self, denominator):
        if (denominator < 5):
            denominator = 5 
        elif (denominator > 8):
            denominator = 8 

        cr = denominator - 4 

        self.__writeRegister(defs.REG_MODEM_CONFIG_1, (self.__readRegister(defs.REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1)) 

    # returns void, takes long
    def setPreambleLength(self, length):
        self.__writeRegister(defs.REG_PREAMBLE_MSB, (length >> 8)) 
        self.__writeRegister(defs.REG_PREAMBLE_LSB, (length >> 0)) 

    # returns void, takes int 
    def setSyncWord(self, sw):
        self.__writeRegister(defs.REG_SYNC_WORD, sw) 

    # returns void 
    def enableCrc(self):
        self.__writeRegister(defs.REG_MODEM_CONFIG_2, self.__readRegister(defs.REG_MODEM_CONFIG_2) | 0x04) 

    # returns void 
    def disableCrc(self):
        self.__writeRegister(defs.REG_MODEM_CONFIG_2, self.__readRegister(defs.REG_MODEM_CONFIG_2) & 0xfb) 

    # returns void 
    def enableInvertIQ(self):
        self.__writeRegister(defs.REG_INVERTIQ,  0x66) 
        self.__writeRegister(defs.REG_INVERTIQ2, 0x19) 

    # returns void 
    def disableInvertIQ(self):
        self.__writeRegister(defs.REG_INVERTIQ,  0x27) 
        self.__writeRegister(defs.REG_INVERTIQ2, 0x1d) 

    # returns void, takes uint8 
    def setOCP(self, mA:int):
        ocpTrim: int = 27 

        if (mA <= 120):
            ocpTrim = int((mA - 45) / 5) 
        elif (mA <=240):
            ocpTrim = int((mA + 30) / 10) 

        self.__writeRegister(defs.REG_OCP, 0x20 | (0x1F & ocpTrim)) 

    def random(self):
        return self.__readRegister(defs.REG_RSSI_WIDEBAND) 

    # returns void 
    def setSPIFrequency(self, frequency):
        self._spi.max_speed_hz = frequency

    # Working
    def dumpRegisters(self):
        for i in range(0,128):
            print(F"0x{hex(i)}: 0x{hex(self.__readRegister(i))}") 

    # returns void 
    def explicitHeaderMode(self):
        _implicitHeaderMode = 0 
        self.__writeRegister(defs.REG_MODEM_CONFIG_1, self.__readRegister(defs.REG_MODEM_CONFIG_1) & 0xfe) 

    # returns void 
    def implicitHeaderMode(self):
        _implicitHeaderMode = 1 
        self.__writeRegister(defs.REG_MODEM_CONFIG_1, self.__readRegister(defs.REG_MODEM_CONFIG_1) | 0x01) 

    # returns void 
    def handleDio0Rise(self, channel):
        irqFlags = self.__readRegister(defs.REG_IRQ_FLAGS) 

        # clear IRQ's
        self.__writeRegister(defs.REG_IRQ_FLAGS, irqFlags) 

        if ((irqFlags & defs.IRQ_PAYLOAD_CRC_ERROR_MASK) == 0):

            if ((irqFlags & defs.IRQ_RX_DONE_MASK) != 0):
                # received a packet
                self._packetIndex = 0 

                # read packet length
                packetLength = self.__readRegister(defs.REG_PAYLOAD_LENGTH) if self._implicitHeaderMode else self.__readRegister(defs.REG_RX_NB_BYTES) 

                # set FIFO address to current RX address
                self.__writeRegister(defs.REG_FIFO_ADDR_PTR, self.__readRegister(defs.REG_FIFO_RX_CURRENT_ADDR)) 

                if (self._onReceive):
                    self._onReceive(packetLength) 

            elif ((irqFlags & defs.IRQ_TX_DONE_MASK) != 0):
                if (self._onTxDone):
                   self._onTxDone() 

    # takes and returns single byte
    def __readRegister(self, address):
        return self.__singleTransfer(address & 0x7f, 0x00) 

    # returns void, takes single byte 
    def __writeRegister(self, address, value):
        self.__singleTransfer(address | 0x80, value) 

    # Working
    def __singleTransfer(self, address:int, value:int):
        response = 00

        if self._cs > 1:
            GPIO.output(self._cs, GPIO.LOW)

        self._spi.xfer2([address]) 
        response = self._spi.xfer2([value]) 

        if self._cs > 1:
            GPIO.output(self._cs, GPIO.HIGH)

        return response[0] 



if __name__ == "__main__":

    def onReceive(size):
        print("Receiving message")
        print("Rssi: " + str(radio.packetRssi()) )
        print("SNR: " + str(radio.packetSnr()))
        msg = ""
        array = bytearray()
        for i in range(0,size):
            temp = radio.read()
            array.append(temp)
            msg += chr(temp)
       	print(array)

        radio.beginPacket()
        radio.write(msg)
        radio.endPacket(False)
        radio.receive()

    def txDone():
        print("Packet Transmitted")

    radio = LoRa(22,27, 17)
    radio.begin(915E6)
    radio.onTxDone(txDone)
    radio.onReceive(onReceive)
    radio.receive()
    try:
        while True:
            time.sleep(1)

    except KeyboardInterrupt as e:
        pass
    finally:
        radio.end()





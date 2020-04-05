# python 3.7

import defs
import spidev
import time

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM) 

# CS on GPIO22


# We only have SPI bus 0 available to us on the Pi
bus = 0

#Device is the chip select pin. Set to 0 or 1, depending on the connections
device = 1

# Enable SPI
spi = spidev.SpiDev()

# Open a connection to a specific bus and device (chip select pin)
spi.open(bus, device)

# Set SPI speed and mode
spi.max_speed_hz = 200000
spi.mode = 0


class LoRaClass:

    def __init__(self, cs, rst, dio0):
        self._cs = cs
        self._rst = rst
        self._dio0 = dio0
        self._spi = spidev.SpiDev()
   

    
        self._frequency = 0
        self._packetIndex = 0
        self._implicitHeaderMode = 0
        self._onReceive = None
        self._onTxDone = None
    #    overide Stream timeout value

    #  frequency is a long, returns an int
    def begin(self, frequency):

        

        GPIO.setup(self._rst, GPIO.OUT) 
        GPIO.setup(self._cs, GPIO.OUT) 
        GPIO.setup(self._dio0, GPIO.IN, pull_up_down=GPIO.PUD_UP) 
        
        # perform reset
        GPIO.output(self._rst, GPIO.LOW)
        time.sleep(0.01) 
        GPIO.output(self._rst, GPIO.HIGH)
        time.sleep(0.01) 

        # start SPI
        if self._cs == 0 or self._cs == 1:
            self._spi.open(0,self._cs)

        else:
            self._spi.no_cs = True
            GPIO.output(self._cs, GPIO.LOW)   # just set low for now, we'll debug deselecting the chip after each transfer
            self._spi.open(0,0)

       
        # check version
        version = self.__readRegister(defs.REG_VERSION) 
        if (version != 0x12):
            return 0 

        # put in sleep mode
        sleep() 

        # set frequency
        setFrequency(frequency) 

        # set base addresses
        self.__writeRegister(defs.REG_FIFO_TX_BASE_ADDR, 0) 
        self.__writeRegister(defs.REG_FIFO_RX_BASE_ADDR, 0) 

        # set LNA boost
        self.__writeRegister(defs.REG_LNA, self.__readRegister(defs.REG_LNA) | 0x03) 

        # set auto AGC
        self.__writeRegister(defs.REG_MODEM_CONFIG_3, 0x04) 

        # set output power to 17 dBm
        setTxPower(17) 

        # put in standby mode
        idle() 
        return 1 

    # returns nothing
    def end(self):
        # put in sleep mode
        sleep() 

        # stop SPI
        self._spi.close() 


    # takes int, returns int
    def beginPacket(self, implicitHeader):
        if (isTransmitting()):
            return 0 

        # put in standby mode
        idle() 

        if (implicitHeader):
            implicitHeaderMode() 
        else: 
            explicitHeaderMode() 

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
            while ((self.__readRegister(defs.REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0):
                yield()     # have to figure this out...
            # clear IRQ's
            self.__writeRegister(defs.REG_IRQ_FLAGS, IRQ_TX_DONE_MASK) 

        return 1 

    #bool return
    def isTransmitting(self):
        if ((self.__readRegister(defs.REG_OP_MODE) & defs.MODE_TX) == defs.MODE_TX):
            return true 

        if (self.__readRegister(defs.REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK):
            # clear IRQ's
            self.__writeRegister(defs.REG_IRQ_FLAGS, IRQ_TX_DONE_MASK) 
        return false 

    #int return, int in
    def parsePacket(self, size):
        packetLength = 0 
        irqFlags = self.__readRegister(defs.REG_IRQ_FLAGS) 

        if (size > 0):
            implicitHeaderMode() 
            self.__writeRegister(defs.REG_PAYLOAD_LENGTH, size & 0xff) 
        else:
            explicitHeaderMode() 

        # clear IRQ's
        self.__writeRegister(defs.REG_IRQ_FLAGS, irqFlags) 

        if ((irqFlags & IRQ_RX_DONE_MASK) and (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0 ):
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
            idle() 
        elif (self.__readRegister(defs.REG_OP_MODE) != (MODE_LONG_RANGE_MODE | defs.MODE_RX_SINGLE) ):
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
        fError = (((freqError) * (1 << 24)) / fXtal) * (getSignalBandwidth() / 500000.0) # p. 37

        return static_cast<long>(fError) 

    #size_t return, uint8 in
    def write(self, byte):
        return write(byte, sizeof(byte)) 

    #TODO: This should be changed to use bytearray with automatic size calculation
    #size_t return, buffer pointer and int size in
    def write(self, buffer, size):
        currentLength = self.__readRegister(defs.REG_PAYLOAD_LENGTH) 
        # check size
        if ((currentLength + size) > defs.MAX_PKT_LENGTH ):
            size = defs.MAX_PKT_LENGTH - currentLength 
        # write data
        for i in range(0,size):
            self.__writeRegister(defs.REG_FIFO, buffer[i]) 
        # update length
        self.__writeRegister(defs.REG_PAYLOAD_LENGTH, currentLength + size) 

        return size 

    #int 
    def available(self):
        return (self.__readRegister(defs.REG_RX_NB_BYTES) -self._packetIndex) 

    #int 
    def read(self):
        if (not available()):
            return -1 
        self._packetIndex = self._packetIndex + 1 
        return self.__readRegister(defs.REG_FIFO) 

    #int 
    def peek():
        if (not available()):
            return -1 
        # store current FIFO address
        currentAddress = self.__readRegister(defs.REG_FIFO_ADDR_PTR) 
        # read
        b = self.__readRegister(defs.REG_FIFO) 
        # restore FIFO address
        self.__writeRegister(defs.REG_FIFO_ADDR_PTR, currentAddress) 
        return b 

    # void def flush()
    # {
    # }

    #TODO: Figure out how to handle callbacks

    # void def onReceive(void(*callback)(int))
    # {
    # _onReceive = callback 

    # if (callback) {
    #     pinMode(_dio0, INPUT) 
    # #ifdef SPI_HAS_NOTUSINGINTERRUPT
    #     SPI.usingInterrupt(digitalPinToInterrupt(_dio0)) 
    # #endif
    #     attachInterrupt(digitalPinToInterrupt(_dio0), def onDio0Rise, RISING) 
    # } else {
    #     detachInterrupt(digitalPinToInterrupt(_dio0)) 
    # #ifdef SPI_HAS_NOTUSINGINTERRUPT
    #     SPI.notUsingInterrupt(digitalPinToInterrupt(_dio0)) 
    # #endif
    # }
    # }

    # void def onTxDone(void(*callback)())
    # {
    # _onTxDone = callback 

    # if (callback) {
    #     pinMode(_dio0, INPUT) 
    # #ifdef SPI_HAS_NOTUSINGINTERRUPT
    #     SPI.usingInterrupt(digitalPinToInterrupt(_dio0)) 
    # #endif
    #     attachInterrupt(digitalPinToInterrupt(_dio0), def onDio0Rise, RISING) 
    # } else {
    #     detachInterrupt(digitalPinToInterrupt(_dio0)) 
    # #ifdef SPI_HAS_NOTUSINGINTERRUPT
    #     SPI.notUsingInterrupt(digitalPinToInterrupt(_dio0)) 
    # #endif
    # }
    # }

    # takes int, returns nothing
    def receive(self, size):
        self.__writeRegister(defs.REG_DIO_MAPPING_1, 0x00) # DIO0 => RXDONE

        if (size > 0):
            self.implicitHeaderMode() 
            self.__writeRegister(defs.REG_PAYLOAD_LENGTH, size & 0xff) 
        else:
            self.explicitHeaderMode() 

        self.__writeRegister(defs.REG_OP_MODE, defs.MODE_LONG_RANGE_MODE | defs.MODE_RX_CONTINUOUS) 

    # returns void 
    def idle(self):
        self.__writeRegister(defs.REG_OP_MODE, defs.MODE_LONG_RANGE_MODE | defs.MODE_STDBY) 

    # returns void 
    def sleep(self):
        self.__writeRegister(defs.REG_OP_MODE, defs.MODE_LONG_RANGE_MODE | defs.MODE_SLEEP) 

    # returns void, takes two ints
    def setTxPower(self, level, outputPin):
        if (PA_OUTPUT_RFO_PIN == outputPin):
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
                setOCP(140) 
            else:
                if (level < 2):
                    level = 2 
                #$Default value PA_HF/LF or +17dBm
                self.__writeRegister(defs.REG_PA_DAC, 0x84) 
                setOCP(100) 

            self.__writeRegister(defs.REG_PA_CONFIG, PA_BOOST | (level - 2)) 

    # returns void, takes a long
    def setFrequency(self, frequency):
        _frequency = frequency 

        frf = (frequency << 19) / 32000000 

        self.__writeRegister(defs.REG_FRF_MSB, (uint8_t)(frf >> 16)) 
        self.__writeRegister(defs.REG_FRF_MID, (uint8_t)(frf >> 8)) 
        self.__writeRegister(defs.REG_FRF_LSB, (uint8_t)(frf >> 0)) 

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
        setLdoFlag() 

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
        setLdoFlag() 

    # returns void 
    def setLdoFlag(self):
        # Section 4.1.1.5
        symbolDuration = 1000 / ( getSignalBandwidth() / (1 << getSpreadingFactor()) )  

        # Section 4.1.1.6
        ldoOn = symbolDuration > 16 

        config3 = self.__readRegister(defs.REG_MODEM_CONFIG_3) 
        bitWrite(config3, 3, ldoOn) 
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
        self.__writeRegister(defs.REG_PREAMBLE_MSB, (uint8_t)(length >> 8)) 
        self.__writeRegister(defs.REG_PREAMBLE_LSB, (uint8_t)(length >> 0)) 

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
    def setOCP(self, mA):
        ocpTrim = 27 

        if (mA <= 120):
            ocpTrim = (mA - 45) / 5 
        elif (mA <=240):
            ocpTrim = (mA + 30) / 10 

        self.__writeRegister(defs.REG_OCP, 0x20 | (0x1F & ocpTrim)) 

    def random(self):
        return self.__readRegister(defs.REG_RSSI_WIDEBAND) 

    # returns void, takes ints 
    def setPins(self, ss, reset, dio0):
        _ss = ss 
        _reset = reset 
        _dio0 = dio0 

    # returns void 
    # def setSPI(SPIClass& spi)
    # {
    # _spi = &spi 
    # }

    # returns void 
    def setSPIFrequency(self, frequency):
        _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0) 

    # returns void 
    def dumpRegisters(self):
        for i in range(0,128):
            print("0x") 
            print(hex(i)) 
            print(": 0x") 
            println(hex(self.__readRegister(i)) ) 

    # returns void 
    def explicitHeaderMode(self):
        _implicitHeaderMode = 0 
        self.__writeRegister(defs.REG_MODEM_CONFIG_1, self.__readRegister(defs.REG_MODEM_CONFIG_1) & 0xfe) 

    # returns void 
    def implicitHeaderMode(self):
        _implicitHeaderMode = 1 
        self.__writeRegister(defs.REG_MODEM_CONFIG_1, self.__readRegister(defs.REG_MODEM_CONFIG_1) | 0x01) 

    # returns void 
    def handleDio0Rise(self):
        irqFlags = self.__readRegister(defs.REG_IRQ_FLAGS) 

        # clear IRQ's
        self.__writeRegister(defs.REG_IRQ_FLAGS, irqFlags) 

        if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0):

            if ((irqFlags & IRQ_RX_DONE_MASK) != 0):
                # received a packet
                self._packetIndex = 0 

                # read packet length
                packetLength = self.__readRegister(defs.REG_PAYLOAD_LENGTH) if self._implicitHeaderMode else self.__readRegister(defs.REG_RX_NB_BYTES) 

                # set FIFO address to current RX address
                self.__writeRegister(defs.REG_FIFO_ADDR_PTR, self.__readRegister(defs.REG_FIFO_RX_CURRENT_ADDR)) 

                if (self._onReceive):
                    self._onReceive(packetLength) 

            elif ((irqFlags & IRQ_TX_DONE_MASK) != 0):
                if (self._onTxDone):
                   self._onTxDone() 

    # takes and returns single byte
    def __readRegister(self, address):
        return self.__singleTransfer(address & 0x7f, 0x00) 

    # returns void, takes single byte 
    def __writeRegister(self, address, value):
        self.__singleTransfer(address | 0x80, value) 

    # takes and returns bytes
    def __singleTransfer(self, address, value):
        response 

        # digitalWrite(_ss, LOW) 

        self._spi.xfer2(address) 
        response = self._spi.xfer2(value) 

        # digitalWrite(_ss, HIGH) 

        return response 

    # ISR_PREFIX # returns void 
    # def onDio0Rise()
    # {
    # LoRa.handleDio0Rise() 
    # }





if __name__ == "__main__":
    radio = LoRaClass(22,3,1)
    radio.begin(915E6)




    GPIO.cleanup()
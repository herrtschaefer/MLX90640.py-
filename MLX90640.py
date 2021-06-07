#MLX90640 python driver ported to python by Tim Schäfer 
#Official c++ driver from Melexis: https://github.com/melexis/mlx90640-library 
#licensed under Apache License 2.0

# version 0.3

# version information:
#  it`s not pretty but it works 
#  Does not fully support everything yet
#  USE AT OWN RISK I DO NOT GUARANTEE THAT EVERYTHING WORKS (AS EXPECTED)

from smbus2 import SMBusWrapper, i2c_msg
from math import sqrt


class MLX90640(object):
    
  
    
    version = 'MLX90640 library version: 0.1' #module Version
    
    
    def MODE(self,mode):
        #select Mode
        if mode == 'interleaved':
            self.mode = 'interleaved'
            self.SetInterleavedMode()
        else:
            if mode != 'chess':
                Print("using default mode : chess") 
            self.mode = 'chess'
            self.SetChessMode()
            
# resolution constants
    MLX16BIT = 0x00
    MLX17BIT = 0x01
    MLX18BIT = 0x02
    MLX19BIT = 0x03
    
# Refresh constants 
    MLXP5Hz     = 0x00 #– 0.5Hz
    MLX01Hz     = 0x01 #– 1Hz
    MLX02Hz     = 0x02 #– 2Hz
    MLX04Hz     = 0x03 #– 4Hz
    MLX08Hz     = 0x04 #– 8Hz
    MLX16Hz     = 0x05 #– 16Hz
    MLX32Hz     = 0x06 #– 32Hz
    MLX64Hz     = 0x07 #– 64Hz
    
        
        
             
    """
    i2c interface functions
    """
    
    #write 16bit word to a register in the sensor
    def write(self,register, data):
        with SMBusWrapper(1) as bus:
            bus.write_word_data(self.address, register,data)
        
    #convert list of bytes into list of 16 bit words [0xEE,0x0A]=> 0xEE0A
    def WORD(self,bytelist): 
        wordList =[]
        for i in range(0,len(bytelist),2):
            value = (bytelist[i] << 8) | bytelist[i+1]
            wordList.append(value)
        return wordList
    
    #read n 16bit words beginning with the register address returns list of words
    def read(self,register, length):
        data = []  #data buffer
        with SMBusWrapper(1) as bus:
        
            regl = [register >> 8,register & 0x00FF]   # split register address into a list of bytes
            write = i2c_msg.write(self.address,regl)   # build message object    
            read = i2c_msg.read(self.address,length*2) # build read request
            bus.i2c_rdwr(write,read) #send i2c command
            data = list(read) # get data from read in byte form
            return(self.WORD(data))# returns the words
    
    #read one 16bit word corresponding to the register
    def read_word(self,register):
        word = self.read(register,1)
        return word[0]
      
      
# CONSTRUCTOR    
    def __init__(self,i2cAddress=0x33,mode='chess',refreshRate= MLX04Hz,resolution=MLX16BIT):
       
        self.address = i2cAddress
              
        
        self.MLXParameters = {
                        "kVdd":0,
                        "vdd25":0,
                        "KvPTAT":0,
                        "KtPTAT":0,
                        "alphaPTAT":0,
                        "gainEE":0,
                        "tgc":0,
                        "cpKv":0,
                        "cpKta":0,
                        "resolutionEE":0,
                        "calibrationModeEE":0,
                        "KsTa":0,
                        "KsTo":0,
                        "ct":0,
                        "alpha":0,
                        "offset":0,
                        "kta":0,
                        "kv":0,
                        "cpAlpha":0,
                        "cpOffset":0,
                        "ilChessC":0,
                        "brokenPixels":0,
                        "outlierPixels":0
                        }
        
        self.eeData    = self.dumpEE()
        self.ExtractParameters()
        self.MODE(mode)
        self.SetRefreshRate(refreshRate)
        self.setResolution(resolution)
        self.frameData = [0.0]*834
        self.getFrameData()
           
    
    """
    API functions
    """
    
    def dumpEE(self): # reads the EEPROM data
        self.eeData  = self.read(0x2400,832)
        return self.eeData

    def ExtractVDDParameters(self):
        
        kVdd = self.eeData[51]
        kVdd = (self.eeData[51] & 0xFF00) >> 8
        
        if kVdd > 127:
            kVdd = kVdd - 256
            
        kVdd  = 32 * kVdd
        vdd25 = self.eeData[51] & 0x00FF
        vdd25 = ((vdd25 - 256) << 5 ) -8192
        
        self.MLXParameters['kVdd']  = kVdd
        self.MLXParameters['vdd25'] = vdd25
 
    def ExtractPTATParameters(self):
        
        KvPTAT = (self.eeData[50] & 0xFC00) >> 10
        
        if KvPTAT > 31:
            KvPTAT = KvPTAT - 64
            
        KvPTAT/=4096
        
        KtPTAT = self.eeData[50] & 0x03FF
        if KtPTAT > 511:
            KtPTAT = KtPTAT - 1024
            
        KtPTAT /=8
        
        vPTAT25 = self.eeData[49]
        
        alphaPTAT = (self.eeData[16] & 0xF000) / pow(2,14) + 8.0
        
        self.MLXParameters['KvPTAT']     = KvPTAT
        self.MLXParameters['KtPTAT']     =KtPTAT
        self.MLXParameters['vPTAT25']    =vPTAT25
        self.MLXParameters['alphaPTAT']  =alphaPTAT    

    def ExtractGainParameters(self):
        gainEE = self.eeData[48]
        
        if gainEE > 32767:
            gainEE =gainEE - 65536
            
        self.MLXParameters['gainEE'] = gainEE
        
    def ExtractTgcParameter(self):
        tgc = self.eeData[60] & 0x00FF
        if tgc > 127:
            tgc = tgc - 256
        tgc /= 32.0
        self.MLXParameters['tgc'] = tgc
        
    def ExtractResolutionParameters(self):
        self.MLXParameters['resolutionEE'] = (self.eeData[56] & 0x3000) >> 12

    def ExtractKsTaParameters(self):
        
        KsTa = (self.eeData[60] & 0xFF00) >>8
        if KsTa >127:
            KsTa -= 256
        KsTa /= 8192.0
        self.MLXParameters['KsTa'] = KsTa

    def ExtractKsToParameters(self):
        ct =[0]*4 #generate list with 4 elements
        KsTo =[0]*4 #generate list with 4 elements
        step = ((self.eeData[63] & 0x3000) >> 12)*10
        
        ct[0] = -40
        ct[1] = 0
        ct[2] = (self.eeData[63]& 0x00F0)>>4
        ct[3] = (self.eeData[63]& 0x0F00)>>8
        
        ct[2]*=step
        ct[3] = ct[2] + ct[3]*step
        
        KsToScale = (self.eeData[63] & 0x000F)+8
        KsToScale = 1 << KsToScale
        
        KsTo[0] = self.eeData[61] & 0x00FF
        KsTo[1] = self.eeData[61] & 0xFF00 >> 8
        
        KsTo[2] = self.eeData[62] & 0x00FF
        KsTo[3] = self.eeData[62] & 0xFF00 >> 8
        
        for i in range(4):
            if KsTo[i] > 127:
                KsTo[i] -= 256
            KsTo[i] /=KsToScale
        
        self.MLXParameters['ct'] = ct
        self.MLXParameters['KsTo'] = KsTo 
        
    def ExtractAlphaParameters(self):
        
        alpha = [0.0]*768
        accRow =[0]*24
        accColumn=[0]*32
        p = 0
        
        accRemScale    = self.eeData[32] & 0x000F
        accColumnScale = (self.eeData[32] & 0x00F0) >> 4
        accRowScale    = (self.eeData[32] & 0x0F00) >> 8
        alphaScale     = ((self.eeData[32] & 0xF000) >> 12) + 30
        alphaRef       = self.eeData[33]
        
        for i in range(6):
            p= i*4
            accRow[p + 0] = (self.eeData[34 + i] & 0x000F)
            accRow[p + 1] = (self.eeData[34 + i] & 0x00F0) >> 4
            accRow[p + 2] = (self.eeData[34 + i] & 0x0F00) >> 8
            accRow[p + 3] = (self.eeData[34 + i] & 0xF000) >> 12
        
        for i in range(24):
            if accRow[i] > 7:
                accRow[i] -= 16
                
        for i in range(8):
            p= i*4
            accColumn[p + 0] = (self.eeData[40 + i] & 0x000F)
            accColumn[p + 1] = (self.eeData[40 + i] & 0x00F0) >> 4
            accColumn[p + 2] = (self.eeData[40 + i] & 0x0F00) >> 8
            accColumn[p + 3] = (self.eeData[40 + i] & 0xF000) >> 12
        
        for i in range(32):
            if accColumn[i] >7:
                accColumn[i] -=16
                
        for i in range(24):
            for j in range(32):
                p= 32*i+j
                alpha[p] = (self.eeData[64 +p] & 0x03F0)>> 4
                if alpha[p] > 31:
                    alpha[p]-= 64
                    
                alpha[p] *=(1<<accRemScale)
                alpha[p] = (alphaRef +(accRow[i]<< accRowScale)+(accColumn[j]<< accColumnScale)+alpha[p])
                alpha[p] /= pow(2,alphaScale)
                
        self.MLXParameters['alpha'] = alpha
                
    def ExtractOffsetParameters(self):
        offset = [0]*768
        occRow =[0]*24
        occColumn =[0]*32
        p =0
        
        occRemScale = (self.eeData[16] & 0x000F)
        occColumnScale = (self.eeData[16] & 0x00F0) >> 4
        occRowScale = (self.eeData[16] & 0x0F00) >> 8
        offsetRef = self.eeData[17]
        if offsetRef > 32767:
            offsetRef -= 65536
            
        for i in range(6):
            p = i * 4
            occRow[p + 0] = (self.eeData[18 + i] & 0x000F)
            occRow[p + 1] = (self.eeData[18 + i] & 0x00F0) >> 4
            occRow[p + 2] = (self.eeData[18 + i] & 0x0F00) >> 8
            occRow[p + 3] = (self.eeData[18 + i] & 0xF000) >> 12
        
        
        for i in range(24):
            if occRow[i] > 7:
                occRow[i] = occRow[i] - 16
            
        
        
        for i in range(8):
            p = i * 4
            occColumn[p + 0] = (self.eeData[24 + i] & 0x000F)
            occColumn[p + 1] = (self.eeData[24 + i] & 0x00F0) >> 4
            occColumn[p + 2] = (self.eeData[24 + i] & 0x0F00) >> 8
            occColumn[p + 3] = (self.eeData[24 + i] & 0xF000) >> 12
        
        for i in range(32):
            if occColumn[i] > 7:
                occColumn[i] = occColumn[i] - 16
                
        for i in range(24):
            for j in range(32):
                p = 32 * i +j
                offset[p] = (self.eeData[64 + p] & 0xFC00) >> 10
                if (offset[p] > 31):
                    offset[p] = offset[p] - 64
                
                offset[p] = offset[p]*(1 << occRemScale)
                offset[p] = (offsetRef + (occRow[i] << occRowScale) + (occColumn[j] << occColumnScale) + offset[p])
        self.MLXParameters['offset'] = offset
        
    def ExtractKtaPixelParameters(self):
        p = 0
        KtaRC = [0]*4
        kta =[0.0]*768
        
        KtaRoCo = (self.eeData[54] & 0xFF00) >> 8
        if KtaRoCo > 127:
            KtaRoCo = KtaRoCo - 256
        
        KtaRC[0] = KtaRoCo
        
        KtaReCo = (self.eeData[54] & 0x00FF)
        if KtaReCo > 127:    
            KtaReCo = KtaReCo - 256
        
        KtaRC[2] = KtaReCo
          
        KtaRoCe = (self.eeData[55] & 0xFF00) >> 8
        if KtaRoCe > 127:
            KtaRoCe = KtaRoCe - 256
        
        KtaRC[1] = KtaRoCe
          
        KtaReCe = (self.eeData[55] & 0x00FF)
        if KtaReCe > 127:
            KtaReCe = KtaReCe - 256
        
        KtaRC[3] = KtaReCe
      
        ktaScale1 = ((self.eeData[56] & 0x00F0) >> 4) + 8
        ktaScale2 = (self.eeData[56] & 0x000F)
        
        for i in range(24):
            for j in range(32):
                p = 32 * i +j
                split = 2*(p/32 - (p/64)*2) + p%2
                kta[p] = (self.eeData[64 + p] & 0x000E) >> 1
                if kta[p] > 3:
                    kta[p] = kta[p] - 8
                
                kta[p] = kta[p] * (1 << ktaScale2)
                kta[p] = KtaRC[int(split)] + kta[p]
                kta[p] = kta[p] / pow(2, ktaScale1)
        
        self.MLXParameters['kta'] = kta

    def ExtractKvPixelParameters(self):
        p = 0
        KvT = [0]*4
        kv = [0.0]*768
        
        KvRoCo = (self.eeData[52] & 0xF000) >> 12
        if KvRoCo > 7:
            KvRoCo = KvRoCo - 16
        
        KvT[0] = KvRoCo
        
        KvReCo = (self.eeData[52] & 0x0F00) >> 8
        if KvReCo > 7:
            KvReCo = KvReCo - 16
        
        KvT[2] = KvReCo
          
        KvRoCe = (self.eeData[52] & 0x00F0) >> 4
        if KvRoCe > 7:
            KvRoCe = KvRoCe - 16
        
        KvT[1] = KvRoCe
          
        KvReCe = (self.eeData[52] & 0x000F)
        if KvReCe > 7:
            KvReCe = KvReCe - 16
        
        KvT[3] = KvReCe
      
        kvScale = (self.eeData[56] & 0x0F00) >> 8


        for i in range(24):
            for j in range(32):
                p = 32 * i +j
                split = 2*(p/32 - (p/64)*2) + p%2
                kv[p] = KvT[int(split)]
                kv[p] = kv[p] / pow(2,kvScale)
        self.MLXParameters['kv'] = kv
     
    def ExtractCPParameters(self):
        alphaSP = [0.0]*2
        offsetSP = [0]*2
        
        alphaScale = ((self.eeData[32] & 0xF000) >> 12) + 27
        
        offsetSP[0] = (self.eeData[58] & 0x03FF)
        if offsetSP[0] > 511:
            offsetSP[0] = offsetSP[0] - 1024
        
        
        offsetSP[1] = (self.eeData[58] & 0xFC00) >> 10
        if offsetSP[1] > 31:
        
            offsetSP[1] = offsetSP[1] - 64
        
        offsetSP[1] = offsetSP[1] + offsetSP[0] 
        
        alphaSP[0] = (self.eeData[57] & 0x03FF)
        if alphaSP[0] > 511:
        
            alphaSP[0] = alphaSP[0] - 1024
        
        alphaSP[0] = alphaSP[0] /  pow(2,alphaScale)
        
        alphaSP[1] = (self.eeData[57] & 0xFC00) >> 10
        if alphaSP[1] > 31:
        
            alphaSP[1] = alphaSP[1] - 64
        
        alphaSP[1] = (1 + alphaSP[1]/128) * alphaSP[0]
        
        cpKta = (self.eeData[59] & 0x00FF)
        if cpKta > 127:
        
            cpKta = cpKta - 256
        
        ktaScale1 = ((self.eeData[56] & 0x00F0) >> 4) + 8    
        cpKta = cpKta / pow(2,ktaScale1)
        
        cpKv = (self.eeData[59] & 0xFF00) >> 8
        if (cpKv > 127):
        
            cpKv = cpKv - 256
        
        kvScale = (self.eeData[56] & 0x0F00) >> 8
        cpKv = cpKv / pow(2,kvScale)
     
        
        self.MLXParameters['cpKta'] = cpKta
        self.MLXParameters['cpKv'] = cpKv
        self.MLXParameters['cpAlpha'] = alphaSP
        self.MLXParameters['cpOffset'] = offsetSP

    def ExtractCILCParameters(self):
        ilChessC = [0.0]*3
        
        calibrationModeEE = (self.eeData[10] & 0x0800) >> 4
        calibrationModeEE = calibrationModeEE ^ 0x80

        ilChessC[0] = (self.eeData[53] & 0x003F)
        if ilChessC[0] > 31:
        
            ilChessC[0] = ilChessC[0] - 64
        
        ilChessC[0] = ilChessC[0] / 16.0
        
        ilChessC[1] = (self.eeData[53] & 0x07C0) >> 6
        if ilChessC[1] > 15:
        
            ilChessC[1] = ilChessC[1] - 32
        
        ilChessC[1] = ilChessC[1] / 2.0
        
        ilChessC[2] = (self.eeData[53] & 0xF800) >> 11
        if ilChessC[2] > 15:
        
            ilChessC[2] = ilChessC[2] - 32
        
        ilChessC[2] = ilChessC[2] / 8.0
        
        self.MLXParameters['calibrationModeEE'] = calibrationModeEE
        self.MLXParameters['ilChessC'] = ilChessC

    def CheckAdjacentPixels(pix1,pix2):
        
        pixPosDif = pix1-pix2
        
        if pixPosDif > -34 and pixPosDif < -30:
            return -6
        if pixPosDif > -2 and pixPosDif < 2:
            return -6
        if pixPosDif > 30 and pixPosDif < 34:
            return -6
        return 0

    def ExtractDeviatingPixels(self):
        pixCnt = 0
        brokenPixCnt = 0
        outlierPixCnt =0
        warn = 0
        brokenPixels = [0]*5
        outlierPixels = [0]*5
        
        for i in range(5):
            brokenPixels[i]  = 0xFFFF
            outlierPixels[i] = 0xFFFF
        self.MLXParameters['brokenPixels'] = brokenPixels
        self.MLXParameters['outlierPixels'] = outlierPixels
            
        while pixCnt < 768 and brokenPixCnt < 5 and outlierPixCnt < 5:
            if self.eeData[pixCnt+64] == 0:
                brokenPixels[brokenPixCnt] = pixCnt
                self.MLXParameters['brokenPixels'] = brokenPixels
                brokenPixCnt +=1
                
            if self.eeData[pixCnt+64]& 0x0001 != 0:
                outlierPixels[outlierPixCnt] = pixCnt
                self.MLXParameters['outlierPixels'] = outlierPixels
                outlierPixCnt+=1
            pixCnt +=1
            
        
        if brokenPixCnt> 4:
             warn = -3
        else:
            if outlierPixCnt > 4:
                warn-4
            else:
                if (brokenPixCnt + outlierPixCnt) > 4 :
                    warn =-5
                else:
                        for pixCnt in range(brokenPixCnt):
                                
                            for i  in range((pixCnt+1),brokenPixCnt):
                                warn = self.CheckAdjacentPixels(brokenPixels[pixCnt],brokenPixels[i])
                                if warn != 0:
                                    return warn
                                    
                        for pixCnt in range(outlierPixCnt):
                                
                            for i  in range(pixCnt +1,outlierPixCnt):
                                warn = self.CheckAdjacentPixels(outlierPixCnt[pixCnt],outlierPixCnt[i])
                                if warn != 0:
                                    return warn
                        for pixCnt in range(brokenPixCnt):
                            for i in range(outlierPixCnt):
                                 warn = self.CheckAdjacentPixels(brokenPixCnt[pixCnt],outlierPixCnt[i])
                                 if warn != 0:
                                     return warn
        return warn        
                                    
    def CheckEEPROMValid(self):
        deviceSelect = self.eeData[10] & 0x0040
        if deviceSelect == 0:
            return 0
        else:
            return -7
                                
    def GetMedian(self,values):
        n = len(values)
        
        for i in range(n-1): # sort array
            for j in range(i+1,n):
                if values[j] <values[i]:
                    temp = values[i]
                    values[i] = values[j]
                    values[j] = temp
        if n%2 ==0:
            return ((values[int(n/2)] + values[int(n/2 - 1)]) / 2.0)
        else:
            return values[n/2]
        
    
        
        
    def IsPixelBad(self,pixel):
        outlier =self.MLXParameters['outlierPixels']
        broken  =self.MLXParameters['brokenPixels' ]
        
        for i in range(5):
            
            if pixel == outlier[i] or pixel ==broken[i]:
                return 1
        return 0
                    
    def getFrameData(self):
        
        dataReady = 0
        cnt = 0
         
        while dataReady == 0:
            statusRegister = self.read_word(0x8000)  #read statusregister
            dataReady = statusRegister & 0x0008
            
        while dataReady != 0 and cnt < 5:
            self.write(0x8000,0x0030)          # manipulate status register to clear data Ready
            frameData =self.read(0x0400,832)  # read data
            
            #print("MLX" )
            #print(frameData)
            statusRegister = self.read_word(0x8000)  #read statusregister
            dataReady = statusRegister & 0x0008
            cnt = cnt +1
            
        controlRegister1 = self.read_word(0x800D)
        frameData.append( controlRegister1)
        frameData.append(statusRegister & 0x0001)
        
        self.frameData = frameData  # store Frame Data in object
        return frameData
       

    def ExtractParameters(self):
        error = self.CheckEEPROMValid()
        if error ==0:
            
            self.ExtractVDDParameters()
            self.ExtractPTATParameters()
            self.ExtractGainParameters()
            self.ExtractTgcParameter()
            self.ExtractResolutionParameters()
            self.ExtractKsTaParameters()
            self.ExtractKsToParameters()
            self.ExtractAlphaParameters()
            self.ExtractOffsetParameters()
            self.ExtractKtaPixelParameters()
            self.ExtractKvPixelParameters()
            self.ExtractCPParameters()
            self.ExtractCILCParameters()
            error = self.ExtractDeviatingPixels()
        return error

    

    def setResolution(self,resolution):
        value = (resolution & 0x03)<<10
        controlRegister1= self.read_word(0x800D)
        value = (controlRegister1 & 0xF3FF) | value
        self.write(0x800D, value)

    def GetCurResolution(self):
        
        controlRegister1= self.read_word(0x800D)
        resolutionRAM = (controlRegister1 & 0xc00) >>10
        return resolutionRAM

    

    def SetRefreshRate(self,refreshRate):
        value = (refreshRate & 0x07) << 7
        controlRegister1 = self.read_word(0x800D)
        value = (controlRegister1 & 0xFC7F) | value
        self.write(0x800D, value)
        
    def GetRefreshRate(self):
        
        controlRegister1 = self.read_word(0x800D)
        refreshRate = (controlRegister1 & 0x0380) >> 7
        return refreshRate

    def SetInterleavedMode(self):
        controlRegister1 = self.read_word(0x800D)
        value = (controlRegister1 & 0xEFFF)
        self.write(0x800D,value)
        
    def SetChessMode(self):
        controlRegister1 = self.read_word(0x800D)
        value = (controlRegister1 | 0x1000)
        self.write(0x800D,value)

    def GetCurMode(self):
        controlRegister1 = self.read_word(0x800D)
        modeRAM = (controlRegister1 &0x1000) >>12
        if modeRAM == 0 :
            print("InterleavedMode")
        else:
            if modeRAM ==1:
                print("ChessMode")
            
        
        return modeRAM

    def GetVdd(self):
        vdd =self.frameData[810]
        if vdd > 32767:
            vdd -= 65536
        resolutionRAM = (self.frameData[832] & 0xc00) >> 10
        resolutionCorrection = pow(2,self.MLXParameters['resolutionEE']) / pow(2,resolutionRAM)
        vdd = (resolutionCorrection * vdd- self.MLXParameters['vdd25'])/self.MLXParameters['kVdd'] +3.3
        return vdd

    def getTa(self):
        
        vdd = self.GetVdd()
        
        ptat =self.frameData[800]
            
        if ptat > 32767:
            ptat = ptat - 65536
        
        ptatArt =self.frameData[768]
        if ptatArt > 32767:
            ptatArt = ptatArt -65536
            
        ptatArt = ( ptat/ (ptat * self.MLXParameters['alphaPTAT'] + ptatArt)) * pow(2,18.0)
        ta = (ptatArt/ (1+ self.MLXParameters['KvPTAT']*(vdd -3.3))- self.MLXParameters['vPTAT25'])
        ta = ta / self.MLXParameters['KtPTAT'] +25
        return ta

    def GetSubPageNumber(self):
        return self.frameData[833]

    def CalculateTo(self,emissivity,reflectedTemperature):
        result =[0.0]*768
        tr = reflectedTemperature
        alphaCorrR = [0.0]*4
        irDataCP   = [0.0]*2
        
        subPage    = self.frameData[833]
        vdd        = self.GetVdd()
        ta = self.getTa()
        ta4 = pow((ta+273.15),4.0)
        tr4 = pow((tr+273.15),4.0)
        taTr =tr4-(tr4-ta4)/emissivity
        
        alphaCorrR[0] = 1 / (1 + self.MLXParameters['KsTo'][0] * 40)
        alphaCorrR[1] = 1 
        alphaCorrR[2] = (1 + self.MLXParameters['KsTo'][2] * self.MLXParameters['ct'][2])
        alphaCorrR[3] = alphaCorrR[2] * (1 + self.MLXParameters['KsTo'][3] * (self.MLXParameters['ct'][3] - self.MLXParameters['ct'][2]))
        
        # GAIN CALCULATION
        gain = self.frameData[778]
        if gain > 32767:
        
            gain = gain - 65536
        
        
        gain = self.MLXParameters['gainEE']/ gain
        
        # To CALCULATION
        
        mode = (self.frameData[832]& 0x1000) >> 5
        
        irDataCP[0] = self.frameData[776]
        irDataCP[1] = self.frameData[808]
        
        for i in range(2):
            if irDataCP[i] > 32767:
                irDataCP[i] = irDataCP[i] - 65536
                
            irDataCP[i] = irDataCP[i] * gain
        
        irDataCP[0] = irDataCP[0] - self.MLXParameters['cpOffset'][0] * (1 + self.MLXParameters['cpKta']* (ta - 25)) * (1 +self.MLXParameters['cpKv'] * (vdd - 3.3))
        
        if mode == self.MLXParameters['calibrationModeEE']:
            irDataCP[1] = irDataCP[1] - self.MLXParameters['cpOffset'][1] * (1 + self.MLXParameters['cpKta'] * (ta - 25)) * (1 + self.MLXParameters['cpKv'] * (vdd - 3.3))
        else:
            irDataCP[1] = irDataCP[1] - (self.MLXParameters['cpOffset'] + self.MLXParameters['ilChessC'][0]) * (1 + self.MLXParameters['cpKta'] * (ta - 25)) * (1 + self.MLXParameters['cpKv'] * (vdd - 3.3))
            
        for pixelNumber in range(768):
            
            ilPattern = int(pixelNumber / 32 - (pixelNumber / 64) * 2)
            
            chessPattern = ilPattern ^ int((pixelNumber - (pixelNumber/2)*2))
            conversionPattern = ((pixelNumber + 2) / 4 - (pixelNumber + 3) / 4 + (pixelNumber + 1) / 4 - pixelNumber / 4) * (1 - 2 * ilPattern)
            
            if mode == 0:
                pattern = ilPattern
            else:
                pattern = chessPattern
                
            if pattern == self.frameData[833]:
                irData = self.frameData[pixelNumber]
                if irData > 32767:
               
                    irData = irData - 65536
               
                irData = irData * gain
                
                irData = irData - self.MLXParameters['offset'][pixelNumber]*(1 + self.MLXParameters['kta'][pixelNumber]*(ta - 25))*(1 + self.MLXParameters['kv'][pixelNumber]*(vdd - 3.3))
                if mode !=  self.MLXParameters['calibrationModeEE']:
                
                  irData = irData + self.MLXParameters['ilChessC'][2] * (2 * ilPattern - 1) - self.MLXParameters['ilChessC'][1] * conversionPattern 
                
                
                irData = irData / emissivity
        
                irData = irData - self.MLXParameters['tgc'] * irDataCP[subPage]
                
                alphaCompensated = (self.MLXParameters['alpha'][pixelNumber] - self.MLXParameters['tgc'] * self.MLXParameters['cpAlpha'][subPage])*(1 + self.MLXParameters['KsTa'] * (ta - 25))
                
                Sx = pow(alphaCompensated, 3) * (irData + alphaCompensated * taTr)
                Sx = sqrt(sqrt(Sx)) * self.MLXParameters['KsTo'][1]
                
                To = sqrt(sqrt(irData/(alphaCompensated * (1 - self.MLXParameters['KsTo'][1] * 273.15) + Sx) + taTr)) - 273.15
                        
                if To < self.MLXParameters['ct'][1]:
                
                    _range = 0
                
                else:
                    if To < self.MLXParameters['ct'][2]:
                       _range = 1
                    else:
                        if To < self.MLXParameters['ct'][3]:
                           _range = 2
                        else:
                            _range = 3
                   
                    
                
                To = sqrt(sqrt(irData / (alphaCompensated * alphaCorrR[_range] * (1 + self.MLXParameters['KsTo'][_range] * (To - self.MLXParameters['ct'][_range]))) + taTr)) - 273.15
                
                result[pixelNumber] = To
        return result
            
    def GetImage(self):
        #to be implemented later
        pass
    
    def BadPixelsCorrection(self,pixelAdresses,TemperatureArray, mode):
        #to be implemented later
        pixels = pixelAdresses
        to = TemperatureArray
        
        ap =[0.0]*4
        
        pix = 0
       
        while pixels[pix] < 65535:
            line = pixels[pix] >> 5
            column = pixels[pix] - line << 5
            
            if (mode == 1):
                if(line == 0):
            
                    if(column == 0):
                           
                        to[pixels[pix]] = to[33]                    
                    
                    else:
                        if(column == 31):
                    
                            to[pixels[pix]] = to[62]                      
                    
                        else:
                    
                            to[pixels[pix]] = (to[pixels[pix]+31] + to[pixels[pix]+33])/2.0                    
                            
                
                else:
                    if(line == 23):
                
                        if(column == 0):
                        
                            to[pixels[pix]] = to[705]                    
                        
                        else:
                            if(column == 31):
                                        
                             to[pixels[pix]] = to[734]                      
                        
                            else:
                        
                             to[pixels[pix]] = (to[pixels[pix]-33] + to[pixels[pix]-31])/2.0                     
                                       
             
                    else :
                       if(column == 0):
                    
                            to[pixels[pix]] = (to[pixels[pix]-31] + to[pixels[pix]+33])/2.0                
                    
                       else:
                            if(column == 31):
                                to[pixels[pix]] = (to[pixels[pix]-33] + to[pixels[pix]+31])/2.0                
                     
                            else:
                                ap[0] = to[pixels[pix]-33]
                                ap[1] = to[pixels[pix]-31]
                                ap[2] = to[pixels[pix]+31]
                                ap[3] = to[pixels[pix]+33]
                                to[pixels[pix]] = self.GetMedian(ap)
                   
            else:
                if(column == 0):
                        
                    to[pixels[pix]] = to[pixels[pix]+1]          
                        
                else:
                    if(column == 1 or column == 30):
                        
                        to[pixels[pix]] = (to[pixels[pix]-1]+to[pixels[pix]+1])/2.0              
                        
                    else:
                        if(column == 31):
                        
                            to[pixels[pix]] = to[pixels[pix]-1]
                         
                        else:
                        
                            if(self.IsPixelBad(pixels[pix]-2) == 0 and self.IsPixelBad(pixels[pix]+2) == 0):
                                                       
                                ap[0] = to[pixels[pix]+1] - to[pixels[pix]+2]
                                ap[1] = to[pixels[pix]-1] - to[pixels[pix]-2]
                                if(abs(ap[0]) > abs(ap[1])):
                                
                                    to[pixels[pix]] = to[pixels[pix]-1] + ap[1]                        
                                
                                else:
                                
                                    to[pixels[pix]] = to[pixels[pix]+1] + ap[0]                       
                                
                            
                            else:
                            
                                to[pixels[pix]] = (to[pixels[pix]-1]+to[pixels[pix]+1])/2.0                   
                                        
                                              
                     
        pix = pix + 1 
                                    
                
            

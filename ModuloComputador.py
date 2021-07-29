import time
import serial
celCarga = serial.Serial('COM3', 115200)
nBytesFromCelCarga = 3
time.sleep(2)
celCarga.write(b"2") #apagar luz
for i in range(1000):
    tic = time.time()
    data = celCarga.read(nBytesFromCelCarga)
    #If byteorder is "big", the most significant byte is at the beginning of the byte array. 
    #If byteorder is "little", the most significant byte is at the end of the byte array.
    data = int.from_bytes(data, byteorder= 'big') 
    altitudeHardware = data & 0x00FFFF
    if(altitudeHardware>32767):
        altitudeEngineering = altitudeHardware - (65535 + 1) #Criando a parte negativa
    else:
        altitudeEngineering = altitudeHardware
    altitudeEngineering = altitudeEngineering/100
    toc = time.time()
    print(str(altitudeEngineering) + "\t" + str(toc-tic))

celCarga.close()
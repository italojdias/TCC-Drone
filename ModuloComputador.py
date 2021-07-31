import time
import serial
celCarga = serial.Serial('COM3', 115200)
drone = serial.Serial('COM4', 115200)
nBytesFromCelCarga = 3
nBytesFromDrone = 3
time.sleep(5)
tic = time.time()
celCarga.write(b"2") #apagar led
drone.write(bytes([0])) #Acao media
for i in range(2000):
    
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

    data = drone.read(nBytesFromDrone)
    data = int.from_bytes(data, byteorder= 'big') 
    anguloHardware = data & 0x00FFFF
    if(anguloHardware>32767):
        anguloEngineering = anguloHardware - (65535 + 1) #Criando a parte negativa
    else:
        anguloEngineering = anguloHardware
    anguloEngineering = anguloEngineering/100

    toc = time.time()
    print(str(altitudeEngineering) + "\t" + str(anguloEngineering) + "\t" + str(i) + "\t" + str(toc-tic))

drone.write(bytes([0])) #Acao media
drone.close()
celCarga.close()
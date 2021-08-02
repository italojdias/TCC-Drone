import time
import serial
celCarga = serial.Serial('COM3', 115200)
drone = serial.Serial('COM4', 115200)
nBytesFromCelCarga = 3
nBytesFromDrone = 3
time.sleep(5)
tic = time.time()
celCarga.write(b"2") #apagar led
drone.write(bytes([255])) #enviando SOP
for i in range(2000):
    ##############RX##############
    dataByteCelCarga = celCarga.read(nBytesFromCelCarga)
    # celCarga.reset_input_buffer()
    #If byteorder is "big", the most significant byte is at the beginning of the byte array. 
    #If byteorder is "little", the most significant byte is at the end of the byte array.
    dataIntCelCarga = int.from_bytes(dataByteCelCarga, byteorder= 'big') 
    altitudeHardware = dataIntCelCarga & 0x00FFFF
    if(altitudeHardware>32767):
        altitudeEngineering = altitudeHardware - (65535 + 1) #Criando a parte negativa
    else:
        altitudeEngineering = altitudeHardware
    altitudeEngineering = altitudeEngineering/100

    dataByteDrone = drone.read(nBytesFromDrone)
    # drone.reset_input_buffer()
    dataIntDrone = int.from_bytes(dataByteDrone, byteorder= 'big') 
    anguloHardware = dataIntDrone & 0x00FFFF
    if(anguloHardware>32767):
        anguloEngineering = anguloHardware - (65535 + 1) #Criando a parte negativa
    else:
        anguloEngineering = anguloHardware
    anguloEngineering = anguloEngineering/100

    ##############TX##############
    drone.write(dataByteCelCarga)

    toc = time.time()
    print(str(altitudeEngineering) + "\t" + str(anguloEngineering) + "\t" + str(i) + "\t" + str(toc-tic))

drone.write(bytes([0])) #Acao media
drone.close()
celCarga.close()
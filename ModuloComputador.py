import time
import serial
import matplotlib.pyplot as plt

def unsignedToSigned(value,sizeOfBytes):
    range = 2**(sizeOfBytes*8)
    if(value > ((range/2)-1)):
        value = value - range
    else:
        value = value
    return value

def gettingSignal(msg, nBytesOfMsg, firstByteOfSignal, sizeByteOfSignal):
    #SOP is allocated on position 0
    mask = (256**sizeByteOfSignal)-1
    msg = msg >> (8*(nBytesOfMsg-firstByteOfSignal-sizeByteOfSignal))
    signal = msg & mask
    return signal

file = open("Data.txt","w")

celCarga = serial.Serial('COM3', 115200)
drone = serial.Serial('COM4', 115200)
celCarga.timeout = 0.010
drone.timeout = 0.010
nBytesFromCelCarga = 5
nBytesFromDrone = 6
altitude = []
setpoint_altitude = []
time.sleep(5)
celCarga.reset_input_buffer()
celCarga.reset_output_buffer()
drone.reset_input_buffer()
drone.reset_output_buffer()
tic = time.time()
celCarga.write(b"2") #apagar led
drone.write(bytes([255])) #enviando SOP
for i in range(3000):
    ##############RX##############
    dataByteCelCarga = celCarga.read(nBytesFromCelCarga)
    celCarga.reset_input_buffer()
    #If byteorder is "big", the most significant byte is at the beginning of the byte array. 
    #If byteorder is "little", the most significant byte is at the end of the byte array.
    dataIntCelCarga = int.from_bytes(dataByteCelCarga, byteorder= 'big') 
    # altitudeHardware = dataIntCelCarga & 0x00FFFF
    altitudeHardware = gettingSignal(dataIntCelCarga, nBytesFromCelCarga, 1, 2)
    if(altitudeHardware>32767):
        altitudeEngineering = altitudeHardware - (65535 + 1) #Criando a parte negativa
    else:
        altitudeEngineering = altitudeHardware
    altitudeEngineering = altitudeEngineering/100
    altitude.append(altitudeEngineering)
    if i<1400:
        setpoint_altitude.append(8)
    else:
        setpoint_altitude.append(5)

    aceleracaoHardware = gettingSignal(dataIntCelCarga, nBytesFromCelCarga, 3, 2)
    aceleracaoHardware = unsignedToSigned(aceleracaoHardware, 2)
    aceleracaoEngineering = aceleracaoHardware/100

    dataByteDrone = drone.read(nBytesFromDrone)
    drone.reset_input_buffer()
    dataIntDrone = int.from_bytes(dataByteDrone, byteorder= 'big') 
    anguloHardware = gettingSignal(dataIntDrone, nBytesFromDrone, 1, 2)
    anguloEngineering = unsignedToSigned(anguloHardware, 2)/100

    acaoP = gettingSignal(dataIntDrone, nBytesFromDrone, 3, 1)
    acaoI = gettingSignal(dataIntDrone, nBytesFromDrone, 4, 1)
    acaoD = gettingSignal(dataIntDrone, nBytesFromDrone, 5, 1)

    acaoP = unsignedToSigned(acaoP, 1)
    acaoI = unsignedToSigned(acaoI, 1)
    acaoD = unsignedToSigned(acaoD, 1)
    ##############TX##############
    drone.reset_output_buffer()
    drone.write(dataByteCelCarga)

    toc = time.time()
    print(altitudeEngineering, "\t", aceleracaoEngineering, "\t", acaoP, "\t", acaoI, "\t", acaoD, "\t", toc-tic)
    # print(str(altitudeEngineering) + "\t" + str(anguloEngineering) + "\t" + str(i) + "\t" + str(toc-tic))

drone.write(bytes([0])) #Acao media
drone.close()
celCarga.close()
file.write("altitude = " + str(altitude))
file.close()

plt.plot(altitude)
plt.plot(setpoint_altitude)
plt.show()
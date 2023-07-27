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

file = open("Resultados\\ControleAngulo\\Data.txt","w")

celCarga = serial.Serial('COM3', 115200)
drone = serial.Serial('COM4', 115200)
celCarga.timeout = 0.010
drone.timeout = 0.010
nBytesFromCelCarga = 5
nBytesFromDrone = 7
altitude = []
angle = []
tempo = []
setpoint_altitude = []
setpoint_angle = []
dataP = []
dataI = []
dataA = []
dataB = []
dataD = []
dataAcao = []
time.sleep(5)
celCarga.reset_input_buffer()
celCarga.reset_output_buffer()
drone.reset_input_buffer()
drone.reset_output_buffer()
tic = time.time()
celCarga.write(b"2") #apagar led
drone.write(bytes([255])) #enviando SOP
MM_counter = 0
SOP_counter = 0
for i in range(2000):
    if i<1000:
        setpoint_angle.append(0)
        setpoint_altitude.append(5)
    elif i<3000:
        setpoint_angle.append(0)
        setpoint_altitude.append(5)
    else:
        setpoint_altitude.append(5)

    ##############RX##############
    dataByteCelCarga = celCarga.read(nBytesFromCelCarga)
    celCarga.reset_input_buffer()
    dataByteDrone = drone.read(nBytesFromDrone)
    drone.reset_input_buffer()
    dataIntCelCarga = int.from_bytes(dataByteCelCarga, byteorder= 'big') 
    dataIntDrone = int.from_bytes(dataByteDrone, byteorder= 'big') 
    #If byteorder is "big", the most significant byte is at the beginning of the byte array. 
    #If byteorder is "little", the most significant byte is at the end of the byte array.
    if dataByteCelCarga != b'' and dataByteDrone != b'' :
        if MM_counter > 0:
            MM_counter -= 1
        if gettingSignal(dataIntCelCarga, nBytesFromCelCarga, 0, 1) == 255 and gettingSignal(dataIntDrone, nBytesFromDrone, 0, 1) == 255:
            if SOP_counter > 0:
                SOP_counter -= 1
            altitudeHardware = gettingSignal(dataIntCelCarga, nBytesFromCelCarga, 1, 2)
            altitudeEngineering = unsignedToSigned(altitudeHardware, 2)/100
            altitude.append(altitudeEngineering)
            aceleracaoHardware = gettingSignal(dataIntCelCarga, nBytesFromCelCarga, 3, 2)
            aceleracaoHardware = unsignedToSigned(aceleracaoHardware, 2)
            aceleracaoEngineering = aceleracaoHardware/100

            anguloHardware = gettingSignal(dataIntDrone, nBytesFromDrone, 1, 2)
            anguloEngineering = unsignedToSigned(anguloHardware, 2)/100
            angle.append(anguloEngineering)
            # acaoP = gettingSignal(dataIntDrone, nBytesFromDrone, 3, 1)
            # acaoI = gettingSignal(dataIntDrone, nBytesFromDrone, 4, 1)
            acaoA = gettingSignal(dataIntDrone, nBytesFromDrone, 3, 1)
            acaoB = gettingSignal(dataIntDrone, nBytesFromDrone, 4, 1)
            acaoD = gettingSignal(dataIntDrone, nBytesFromDrone, 5, 1)
            acao = gettingSignal(dataIntDrone, nBytesFromDrone, 6, 1)
            # acaoP = unsignedToSigned(acaoP, 1)
            # acaoI = unsignedToSigned(acaoI, 1)
            acaoD = unsignedToSigned(acaoD, 1)
            acao = unsignedToSigned(acao, 1)
            # dataP.append(acaoP)
            # dataI.append(acaoI)
            dataA.append(acaoA)
            dataB.append(acaoB)
            dataD.append(acaoD)
            dataAcao.append(acao)
            ##############TX##############
            if len(altitude) > 1:
                if (abs(altitude[-1] - altitude[-2]) < 1) and (abs(aceleracaoEngineering) < 20): # Só passa a msg para o drone se a diferença de altitude for menor que 1m, isso implica em 100m/s e aceleração menor que 20m/s² ou 2g
                    drone.reset_output_buffer()
                    drone.write(dataByteCelCarga)
                    drone.write(bytes([setpoint_altitude[-1]]))
                    drone.write(bytes([setpoint_angle[-1]]))
                else: # Falha: altitude variou muito de uma iteração para outra ou aceleração muito alta
                    altitude[-1] = altitude[-2]
            elif abs(aceleracaoEngineering) < 20:
                drone.reset_output_buffer()
                drone.write(dataByteCelCarga)
                drone.write(bytes([setpoint_altitude[-1]]))
                drone.write(bytes([setpoint_angle[-1]]))
        else:
            SOP_counter += 3
            altitude.append(altitude[-1])
    else:
        MM_counter += 3
        altitude.append(altitude[-1])

    if MM_counter >= 15:
        print("MM FAILED")
        print("Drone was opened: ", drone.is_open)
        print("CelCarga was opened: ", celCarga.is_open)
        break

    if SOP_counter >= 15:
        print("SOP FAILED")
        break


    toc = time.time()
    tempo.append(toc-tic)
    print(altitude[-1], "\t", angle[-1], "\t", setpoint_angle[-1], "\t", acaoA, "\t", acaoB, "\t", toc-tic, "\t", MM_counter, "\t", SOP_counter)
    # print(str(altitudeEngineering) + "\t" + str(anguloEngineering) + "\t" + str(i) + "\t" + str(toc-tic))

drone.write(bytes([0])) #Acao media
drone.close()
celCarga.close()
file.write("altitude = " + str(altitude))
file.write("\nsetpoint_altitude = " + str(setpoint_altitude))
file.write("\nangle = " + str(angle))
file.write("\nsetpoint_angle = " + str(setpoint_angle))
file.write("\ndataA = " + str(dataA))
file.write("\ndataB = " + str(dataB))
# file.write("\ndataAcao = " + str(dataAcao))
# file.write("\ndataP = " + str(dataP))
# file.write("\ndataI = " + str(dataI))
# file.write("\ndataD = " + str(dataD))
file.write("\ntempo = " + str(tempo))
file.close()

# plt.plot(altitude)
# plt.plot(setpoint_altitude)
# plt.plot(angle, label = "angle")
# plt.plot(setpoint_angle, label = "setpoint_angle")
# plt.plot(dataP, label = "dataP")
# plt.plot(dataI, label = "dataI")
# plt.plot(dataD, label = "dataD")
# plt.legend()
# plt.show()

fig = plt.figure()
gs = fig.add_gridspec(2, hspace=0)
axs = gs.subplots(sharex=True, sharey=False)
fig.suptitle('Controle Angulo')
axs[0].plot(angle, label = "angle")
axs[0].plot(setpoint_angle, label = "setpoint_angle")
axs[1].plot(dataA, label = "Acao A")
axs[1].plot(dataB, label = "Acao B")
# axs[1].plot(dataD, label = "dataD")
axs[0].set_ylim([-20,20])
axs[1].set_ylim([0,100])
axs[0].legend()
axs[1].legend()
plt.show()
#aaa
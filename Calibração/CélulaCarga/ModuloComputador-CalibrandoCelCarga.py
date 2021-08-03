import time
import serial
f = open("dadosCelCarga.txt","w")
mediaCelCarga = []
pwm = []
celCarga = serial.Serial('COM3', 115200)
drone = serial.Serial('COM4', 115200)
nBytesFromCelCarga = 3
nBytesFromDrone = 3
time.sleep(5)
celCarga.reset_input_buffer()
drone.reset_input_buffer()
celCarga.reset_output_buffer()
drone.reset_output_buffer()
tic = time.time()
celCarga.write(b"2") #apagar led
drone.write(bytes([255])) #enviando SOP
for acaoMedia in range(10,110,10):
    soma = 0
    dadosCelCarga = []
    for i in range(1500):
        ##############RX##############
        dataByteCelCarga = celCarga.read(nBytesFromCelCarga)
        celCarga.reset_input_buffer()
        # celCarga.reset_input_buffer()
        #If byteorder is "big", the most significant byte is at the beginning of the byte array. 
        #If byteorder is "little", the most significant byte is at the end of the byte array.
        dataIntCelCarga = int.from_bytes(dataByteCelCarga, byteorder= 'big') 
        celCargaFiltradaHardware = dataIntCelCarga & 0x00FFFF
        
        dataByteDrone = drone.read(nBytesFromDrone)
        drone.reset_input_buffer()
        # drone.reset_input_buffer()
        dataIntDrone = int.from_bytes(dataByteDrone, byteorder= 'big') 
        anguloHardware = dataIntDrone & 0x00FFFF
        if(anguloHardware>32767):
            anguloEngineering = anguloHardware - (65535 + 1) #Criando a parte negativa
        else:
            anguloEngineering = anguloHardware
        anguloEngineering = anguloEngineering/100

        ##############TX##############
        drone.reset_output_buffer()
        drone.write(bytes([255,acaoMedia&0xFF00,acaoMedia&0x00FF]))
        
        if i>499:
            soma += celCargaFiltradaHardware
            dadosCelCarga.append(celCargaFiltradaHardware)
    pwm.append(acaoMedia)
    mediaCelCarga.append(soma/1000)
    f.write("acao"+str(acaoMedia)+" = " + str(dadosCelCarga) + "\n")
    print("acaoMedia = " + str(acaoMedia) + "\t mediaCelCarga = " + str(mediaCelCarga[-1]))
        # print(str(altitudeEngineering) + "\t" + str(anguloEngineering) + "\t" + str(i) + "\t" + str(toc-tic))
print("array dos PWMs:")
print(pwm)
print("array da médias da célula de carga:")
print(mediaCelCarga)
f.write("pwm = " + str(pwm) + "\n")
f.write("media = " + str(mediaCelCarga) + "\n")

drone.write(bytes([0])) #Acao media
drone.close()
celCarga.close()
f.close()
import pygame
# To install the pygame, we need: pip install pygame
import serial
# To install the serial, we need: pip install serial
from random import randint
import time
from datetime import datetime
import matplotlib.pyplot as plt

#Constantes paras Cores
white = (255,255,255)
black = (0,0,0)
red = (255,0,0)
green = (0,255,0)
blue = (0,0,255)

pygame.init()

#Constantes Tamanho de tela
largura = 300
altura = 300
tamanho = 20

imagemDrone = pygame.image.load('imagens/drone.png')
imagemDrone = pygame.transform.scale(imagemDrone,(tamanho,tamanho))

#Definindo um relógio para fazer o fps
relogio = pygame.time.Clock()
ts = 0.02
fs = 1/ts

fundo = pygame.display.set_mode(size=(largura,altura))
pygame.display.set_caption("Drone")
font = pygame.font.SysFont(None,15)

def texto(msg, cor,x,y):
    texto1 = font.render(msg,True,cor)
    fundo.blit(texto1,[x,y])

def draw_drone(pos_x,pos_y): #Desenha a posição atual do drone em vermelho
    pygame.draw.rect(fundo, red, [pos_x,pos_y,tamanho,tamanho])
    fundo.blit(imagemDrone,[pos_x,pos_y])

def draw_setpoint(pos_x,pos_y): #Desenha a posição do setpoint em verde
    pygame.draw.rect(fundo, green, [pos_x,pos_y,tamanho,tamanho])
    fundo.blit(imagemDrone,[pos_x,pos_y])

def m2px(metro): #Transformando uma medida de metro para pixel
    #Altura: 300px -> 9
    #Altura: 10m seja em altura-30
    #270px é 9m, logo 30px é 1m
    px = (altura - 30) - metro*30
    return px

def ang2px(ang):  #Transformando uma medida de ângulo para pixel
    px = (largura/2) + (ang*2)
    return px

def RX(celCarga,drone,nBytesFromCelCarga,nBytesFromDrone):
    dataByteCelCarga = celCarga.read(nBytesFromCelCarga)
    celCarga.reset_input_buffer() #limpando o buffer depois de toda leitura para evitar formação de pilha
    dataByteDrone = drone.read(nBytesFromDrone)
    drone.reset_input_buffer() #limpando o buffer depois de toda leitura para evitar formação de pilha
    dataIntCelCarga = int.from_bytes(dataByteCelCarga, byteorder= 'big') 
    dataIntDrone = int.from_bytes(dataByteDrone, byteorder= 'big') 
    #If byteorder is "big", the most significant byte is at the beginning of the byte array. 
    #If byteorder is "little", the most significant byte is at the end of the byte array.
    if dataByteCelCarga != b'' and dataByteDrone != b'' : #Se alguma mensagem for vazia, significa que alguma mensagem não foi transmitida corretamente
        MM_failing = 0
    else:
        MM_failing = 1
    #Se o SOP for diferente de 255, siginifica que a comunicação não está sincronizada
    if gettingSignal(dataIntCelCarga, nBytesFromCelCarga, 0, 1) == 255 and gettingSignal(dataIntDrone, nBytesFromDrone, 0, 1) == 255:
        SOP_failing = 0
    else:
        SOP_failing = 1

    return [dataByteCelCarga,dataByteDrone,dataIntCelCarga,dataIntDrone,MM_failing,SOP_failing]

def TX(drone,dataByteCelCarga,setpoint_altitude,setpoint_angle):
    drone.reset_output_buffer() #Limpando o buffer de output antes da escrita para evitar a formação de pilha
    drone.write(dataByteCelCarga)
    drone.write(bytes([setpoint_altitude]))
    drone.write(bytes([setpoint_angle]))

def unsignedToSigned(value,sizeOfBytes): #Função para transformar um sinal unsigned em signed
    range = 2**(sizeOfBytes*8)
    if(value > ((range/2)-1)):
        value = value - range
    else:
        value = value
    return value

def gettingSignal(msg, nBytesOfMsg, firstByteOfSignal, sizeByteOfSignal): #Função para facilitar ler o buffer da mensagem 
    #SOP is allocated on position 0
    mask = (256**sizeByteOfSignal)-1
    msg = msg >> (8*(nBytesOfMsg-firstByteOfSignal-sizeByteOfSignal))
    signal = msg & mask
    return signal

teste = 0 #Flag para facilitar alguns testes

def jogo(): #Função que cria a janela da interface
    sair = True
    fimdejogo = False
    angulo = []
    altitude = []
    angle = []
    tempo = []
    setpoint_altitude = 0
    setpoint_angle = 0
    data_setpoint_altitude = []
    data_setpoint_angle = []
    dataA = []
    dataB = []
    dataD = []
    dataAcao = []
    MM_counter = 0
    SOP_counter = 0
    if teste == False:
        # Inicializando as portas seriais
        celCarga = serial.Serial('COM3', 115200)
        nBytesFromCelCarga = 5
        drone = serial.Serial('COM4', 115200)
        nBytesFromDrone = 7
        celCarga.timeout = 0.010
        drone.timeout = 0.010

        time.sleep(5) #Delay para dar tempo de configurar a porta serial
        #Resetando os buffers
        celCarga.reset_input_buffer()
        celCarga.reset_output_buffer()
        drone.reset_input_buffer()
        drone.reset_output_buffer()
        tic = time.time()
        #Enviando a primeira mensagem para os arduinos para começar a comunicação
        celCarga.write(b"2")
        drone.write(bytes([255])) #enviando SOP
        file = open("Resultados\\ControleAngulo\\Controle2motores\\Data.txt","w")

    #posicao inicial do drone
    [pos_x,pos_y] = [ang2px(0), m2px(0)]

    i = 0
    while(sair):
        while fimdejogo: #Se o diagnóstico de MM e/ou SOP acusar falha, o usuário pode escolher se repete a simulação ou sai da aplicação
            if teste == False:
                celCarga.close()
                drone.close()

            fundo.fill(white)
            texto("Fim de jogo, para continuar tecle C ou S para sair", red,largura/10,altura/2) #Mensagem de falha
            pygame.display.update()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    sair = False
                    fimdejogo = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_c:
                        jogo()
                    if event.key == pygame.K_s:
                        sair = False
                        fimdejogo = False
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sair = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    setpoint_angle -= 5
                if event.key == pygame.K_RIGHT:
                    setpoint_angle += 5
                if event.key == pygame.K_UP:
                    setpoint_altitude += 2
                if event.key == pygame.K_DOWN:
                    setpoint_altitude -= 2
        #Salvando os dados
        data_setpoint_angle.append(setpoint_angle)
        data_setpoint_altitude.append(setpoint_altitude)

        if teste == False:
            ####Recebendo a mensagem####
            [dataByteCelCarga,dataByteDrone,dataIntCelCarga,dataIntDrone,MM_failing,SOP_failing] = RX(celCarga,drone,nBytesFromCelCarga,nBytesFromDrone)
            if MM_failing == False:
                if MM_counter > 0:
                    MM_counter -= 1
                if SOP_failing == False:
                    if SOP_counter > 0:
                        SOP_counter -= 1
                    #Só atualiza as variáveis se o SOP e o MM não estivere falhando
                    altitudeHardware = gettingSignal(dataIntCelCarga, nBytesFromCelCarga, 1, 2)
                    altitudeEngineering = unsignedToSigned(altitudeHardware, 2)/100
                    altitudeEngineering = setpoint_altitude
                    altitude.append(altitudeEngineering)
                    aceleracaoHardware = gettingSignal(dataIntCelCarga, nBytesFromCelCarga, 3, 2)
                    aceleracaoHardware = unsignedToSigned(aceleracaoHardware, 2)
                    aceleracaoEngineering = aceleracaoHardware/100
                    
                    anguloHardware = gettingSignal(dataIntDrone, nBytesFromDrone, 1, 2)
                    anguloEngineering = unsignedToSigned(anguloHardware, 2)/100
                    angle.append(anguloEngineering)
                    acaoA = gettingSignal(dataIntDrone, nBytesFromDrone, 3, 1)
                    acaoB = gettingSignal(dataIntDrone, nBytesFromDrone, 4, 1)
                    dataA.append(acaoA)
                    dataB.append(acaoB)
                    ##########TX##########
                    if len(altitude) > 1:
                        if (abs(altitude[-1] - altitude[-2]) < 1) and (abs(aceleracaoEngineering) < 20): # Só passa a msg para o drone se a diferença de altitude for menor que 1m, isso implica em 100m/s e aceleração menor que 20m/s² ou 2g
                            TX(drone,dataByteCelCarga,setpoint_altitude,setpoint_angle)
                        else: # Falha: altitude variou muito de uma iteração para outra ou aceleração muito alta
                            altitude[-1] = altitude[-2]
                    elif abs(aceleracaoEngineering) < 20:
                        TX(drone,dataByteCelCarga,setpoint_altitude,setpoint_angle)
                else:
                    SOP_counter += 3
                    altitude.append(altitude[-1])
            else:
                MM_counter += 3
                altitude.append(altitude[-1])
        else:
            altitude.append(0)
            angle.append(0)

        if MM_counter>=15:
            print("MM FAILED")
            fimdejogo = True
        if SOP_counter>=15:
            print("SOP FAILED")
            fimdejogo = True
        
        fundo.fill(white) #Desenhando o fundo
        
        [setpoint_x,setpoint_y] = [ang2px(setpoint_angle), m2px(setpoint_altitude)] #Calculando a posição em pixels do setpoint
        draw_setpoint(setpoint_x,setpoint_y) #Desenhando o sepoint

        [pos_x,pos_y] = [ang2px(angle[-1]), m2px(altitude[-1])] #Calculando a posição em pixels do drone
        pos_x = pos_x%largura #Se passar das bordas laterais, o drone aparece do outro lado
        pos_y = pos_y%altura #Se passar das bordas superior ou inferior, o drone aparece do outro lado
        draw_drone(pos_x, pos_y) #Desenhando o drone
        #Imprimindo os valores atuais
        texto("Altura Real: " +str(int(altitude[-1]*100)/100) +" m",red, 10,10)
        texto("Angulo Real: " +str(angle[-1]) + " º",red, 10,20)
        texto("Setpoint Altura: " +str(setpoint_altitude) +" m",green, 10,30)
        texto("Setpoint Angulo: " +str(setpoint_angle) + " º",green, 10,40)

        pygame.display.update() #O tempo de atualização depende do tempo do recebimento da mensagem
        i = i+1
        toc = time.time()
        tempo.append(toc-tic) #Salvando o tempo passado desde o início
    
    #Quando fechar a tela, executa as seguintes instruções:
    if teste == False:
        drone.write(bytes([17])) #Desliga os motores
        drone.write(bytes([17])) #Desliga os motores
        celCarga.close() #Fecha a comunicação serial
        drone.close() #Fecha a comunicação serial
        plt.figure() #Plota alguns gráficos
        plt.plot(altitude, label = "Altitude")
        plt.plot(data_setpoint_altitude, label = "Setpoint da Altitude")
        plt.legend()
        plt.show()
        file.write("altitude = " + str(altitude)) #Salva os dados num arquivo txr
        file.write("\nsetpoint_altitude = " + str(setpoint_altitude))
        file.write("\nangle = " + str(angle))
        file.write("\nsetpoint_angle = " + str(setpoint_angle))
        file.write("\ndataA = " + str(dataA))
        file.write("\ndataB = " + str(dataB))
        file.write("\ntempo = " + str(tempo))
        file.close()

jogo()


pygame.quit()
from vpython import *
import serial
import time
import os

arduinoData=serial.Serial('com5',115200)
time.sleep(2)
roomX=12
roomY=10
roomZ=16
wallT=.5
wallColor=vector(1,1,1)
wallOpacity=.8
frontOpacity=.1
marbleR=.5
ballColor=vector(0,0,1)

myFloor=box(size=vector(roomX,wallT,roomZ),pos=vector(0,-roomY/2,0),color=wallColor,opacity=wallOpacity)
myCeiling=box(size=vector(roomX,wallT,roomZ),pos=vector(0,roomY/2,0),color=wallColor,opacity=wallOpacity)
leftWall=box(size=vector(wallT,roomY,roomZ),pos=vector(-roomX/2,0,0),color=wallColor,opacity=wallOpacity)
rightWall=box(size=vector(wallT,roomY,roomZ),pos=vector(roomX/2,0,0),color=wallColor,opacity=wallOpacity)
backWall=box(size=vector(roomX,roomY,wallT),pos=vector(0,0,-roomZ/2),color=wallColor,opacity=wallOpacity)
frontWall=box(size=vector(roomX,roomY,wallT),pos=vector(0,0,roomZ/2),color=wallColor,opacity=frontOpacity)
marble=sphere(color=ballColor,radius=marbleR)

paddleX=2
paddleY=2
paddleZ=.2
paddleOpacity=.8
paddleColor=vector(0,.8,.6)
paddle=box(size=vector(paddleX,paddleY,paddleZ),pos=vector(0,0,roomZ/2),color=paddleColor,opacity=paddleOpacity)

marbleX=0
deltaX=.02
 
marbleY=0
deltaY=.02
 
marbleZ=0
deltaZ=.02
point = 0
lb1= label(pos = vector(-9,7,0),
           text = point,
           color = vector(1, 0, 0),
           linecolor = vector(0, 1, 0),
           linewidth = 3,
           border = 10)

def paddle_cal(X):
    if X > 13:
        X = 13
    if X < -13:
        X = -13
    return X

while True:
    
    while arduinoData.inWaiting()==0:
        pass
    dataPacket=arduinoData.readline()
    try:
        dataPacket=str(dataPacket,'utf-8')
        splitPacket=dataPacket.split(',')

        x=float(splitPacket[0])
        y=float(splitPacket[1])
        z=float(splitPacket[2])
        y = -y
        z = -z
        x = paddle_cal(x) * (1023/26) + 511
        y = paddle_cal(y) * (1023/26) + 511
        z = paddle_cal(z) * (1023/26) + 511
        padX=(roomX/1023.)*x - roomX/2
        padY=(-roomY/1023.)*y + roomY/2
        
        marbleX=marbleX+deltaX
        marbleY=marbleY+deltaY
        marbleZ=marbleZ+deltaZ
    
    
        
        if marbleX+marbleR>(roomX/2-wallT/2) or marbleX-marbleR<(-roomX/2+wallT/2):
            deltaX=deltaX*(-1)
            marbleX=marbleX+deltaX
     
        if marbleY+marbleR>(roomY/2-wallT/2) or marbleY-marbleR<(-roomY/2+wallT/2):
            deltaY=deltaY*(-1)
            marbleY=marbleY+deltaY
     
        if marbleZ-marbleR<(-roomZ/2+wallT/2):
            deltaZ=deltaZ*(-1)
            marbleZ=marbleZ+deltaZ
        
        if  marbleZ+marbleR >= (roomZ/2-wallT/2) :
            
            if marbleX > padX-paddleX/2 and marbleX < padX+paddleX/2 and marbleY > padY-paddleY/2 and marbleY<padY+paddleY/2 :
               deltaZ += 0.01
               deltaZ=deltaZ*(-1)
               marbleZ=marbleZ+deltaZ
               point += 1
               #lb1.visible = False
               lb1= label(pos = vector(-9,7,0),
                          text = point,
                          color = vector(1, 0, 0),
                          linecolor = vector(0, 1, 0),
                          linewidth = 3,
                          border = 10) 
             
            else:
                point -= 1
                if point < 0:
               
                    lb=label(pos = vector(0,0,0), text = 'GAME OVER!')
                    time.sleep(2)
                    point = 0
                    lb.visible = False
                        
                    lb2=label(text = "3")
                    time.sleep(1)
                    lb2.visible = False

                    lb2=label(text = "2")
                    time.sleep(1)
                    lb2.visible = False

                    lb2=label(text = "1")
                    time.sleep(1)
                    lb2.visible = False

                    deltaZ=deltaZ*(-1)
                    marbleZ=marbleZ+deltaZ 
                    marbleX=0
                    deltaX=.02
                      
                    marbleY=0
                    deltaY=.02
                      
                    marbleZ=0
                    deltaZ=-.02

                else:
                    
                    lb1= label(pos = vector(-9,7,0),
                               text = point,
                               color = vector(1, 0, 0),
                               linecolor = vector(0, 1, 0),
                               linewidth = 3,
                               border = 10) 
                    deltaZ=deltaZ*(-1)
                    marbleZ=marbleZ+deltaZ
     
        marble.pos=vector(marbleX,marbleY,marbleZ)
        paddle.pos=vector(padX,padY,roomZ/2)
        
    except:
        pass
    
"""
Instituto TecnolÃ³gico de Costa Rica
Computer Engineering
Taller de ProgramaciÃ³n
Ejemplo Consola Cliente
Implementación del módulo NodeMCU
Proyecto 2, semestre 1
2019
Profesor: Milton Villegas Lemus
Autor: Santiago Gamboa RamÃ­rez
Restricciónes: Python3.7 
Ejemplo de como usar el módudo NodeMCU de wifiConnection
"""
#           _____________________________
#__________/BIBLIOTECAS
from tkinter import *               # Tk(), Label, Canvas, Photo
from threading import Thread        # p.start()
import threading                    # 
import os                           # ruta = os.path.join('')
import time                         # time.sleep(x)
from tkinter import messagebox      # AskYesNo ()
import tkinter.scrolledtext as tkscrolled
##### Biblioteca para el Carro
from WiFiClient import NodeMCU


#           ____________________________
#__________/Ventana Principal
root=Tk()
root.title('Proyecto 1')
root.minsize(800,400)
root.resizable(width=NO,height=NO)

#           ______________________________
#__________/Se crea un lienzo para objetos
C_root=Canvas(root, width=800,height=600, bg='white')
C_root.place(x=0,y=0)


#           _____________________________________
#__________/Se titulo de los Cuadros de texto
L_Titulo = Label(C_root,text="Mensajes Enviados",font=('Agency FB',14),bg='white',fg='blue')
L_Titulo.place(x=100,y=10)

L_Titulo = Label(C_root,text="Respuesta Mensaje",font=('Agency FB',14),bg='white',fg='blue')
L_Titulo.place(x=490,y=10)


SentCarScrolledTxt = tkscrolled.ScrolledText(C_root, height=10, width=45)
SentCarScrolledTxt.place(x=10,y=50)

RevCarScrolledTxt = tkscrolled.ScrolledText(C_root, height=10, width=45)
RevCarScrolledTxt.place(x=400,y=50)


#           _____________________________________
#__________/Creando el cliente para NodeMCU
myCar = NodeMCU()
myCar.start()


def get_log():
    """
    Hilo que actualiza los Text cada vez que se agrega un nuevo mensaje al log de myCar
    """
    indice = 0
    while(myCar.loop):
        while(indice < len(myCar.log)):
            mnsSend = "[{0}] cmd: {1}\n".format(indice,myCar.log[indice][0])
            SentCarScrolledTxt.insert(END,mnsSend)
            SentCarScrolledTxt.see("end")

            mnsRecv = "[{0}] result: {1}\n".format(indice,myCar.log[indice][1])
            RevCarScrolledTxt.insert(END, mnsRecv)
            RevCarScrolledTxt.see('end')

            indice+=1
        time.sleep(0.200)
    
p = Thread(target=get_log)
p.start()
           


L_Titulo = Label(C_root,text="Mensaje:",font=('Agency FB',14),bg='white',fg='blue')
L_Titulo.place(x=100,y=250)

E_Command = Entry(C_root,width=30,font=('Agency FB',14))
E_Command.place(x=200,y=250)

L_Titulo = Label(C_root,text="ID mensaje:",font=('Agency FB',14),bg='white',fg='blue')
L_Titulo.place(x=100,y=300)

E_read = Entry(C_root,width=30,font=('Agency FB',14))
E_read.place(x=200,y=300)


def send (event):
    """
    Ejemplo como enviar un mensaje sencillo sin importar la respuesta
    """
    mns = str(E_Command.get())
    if(len(mns)>0 and mns[-1] == ";"):
        E_Command.delete(0, 'end')
        myCar.send(mns)
    else:
        messagebox.showwarning("Error del mensaje", "Mensaje sin caracter de finalización (';')") 


def sendShowID():
    """
    Ejemplo como capturar un ID de un mensaje específico.
    """
    mns = str(E_Command.get())
    if(len(mns)>0 and mns[-1] == ";"):
        E_Command.delete(0, 'end')
        mnsID = myCar.send(mns)
        messagebox.showinfo("Mensaje pendiente", "Intentando enviar mensaje, ID obtenido: {0}\n\
La respuesta definitiva se obtine en un máximo de {1}s".format(mnsID, myCar.timeoutLimit))
        
    else:
        messagebox.showwarning("Error del mensaje", "Mensaje sin caracter de finalización (';')")

def read():
    """
    Ejemplo de como leer un mensaje enviado con un ID específico
    """
    mnsID = str(E_read.get())
    if(len(mnsID)>0 and ":" in mnsID):
        mns = myCar.readById(mnsID)
        if(mns != ""):
            messagebox.showinfo("Resultado Obtenido", "El mensaje con ID:{0}, obtuvo de respuesta:\n{1}".format(mnsID, mns))
            E_read.delete(0, 'end')
        else:
            messagebox.showerror("Error de ID", "No se obtuvo respuesta\n\
El mensaje no ha sido procesado o el ID es invalido\n\
Asegurese que el ID: {0} sea correcto".format(mnsID))

    else:
        messagebox.showwarning("Error en formato", "Recuerde ingresar el separador (':')")

root.bind('<Return>', send) #Vinculando tecla Enter a la función send

#           ____________________________
#__________/Botones de ventana principal

Btn_ConnectControl = Button(C_root,text='Send',command=lambda:send(None),fg='white',bg='blue', font=('Agency FB',12))
Btn_ConnectControl.place(x=450,y=250)

Btn_Controls = Button(C_root,text='Send & Show ID',command=sendShowID,fg='white',bg='blue', font=('Agency FB',12))
Btn_Controls.place(x=500,y=250)

Btn_ConnectControl = Button(C_root,text='Leer Mensaje',command=read,fg='white',bg='blue', font=('Agency FB',12))
Btn_ConnectControl.place(x=450,y=300)

root.mainloop()

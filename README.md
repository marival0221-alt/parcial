# parcial
app.
Es un conjunto de modulos deseñado para analizar una señal proveniente del Raspberry Pi Pico, cada uno independiente de otro, pero trabajando en conjunto en un archivo ejecutable (run.py) que llama todas las clases hechas en cada modulo.

Modulos
Modulo lector.
Contiene el archivo (lector.py), dentro del mismo existe la clase "Lector" compuesta por varias funciones dedicadas al analisis, entre ellas: tomar_lectura, muestrear y muestrear_freq. Se guarda en el Raspberry Pi Pico en donde se conecta al ADC (convertidor analógico-digital) del microcontrolador y convierte las señales analógicas (0–3.3 V) en valores digitales.

#Importa pines en particulas de raspberry pi pico.
from machine import ADC, Pin
import time

#Clase del lector.
class Lector:
    #Constructor: inicializa la clase.
    def __init__(self, pin:int):
       # Inicializa el ADC en el pin indicado
       self.adc = ADC(Pin(pin))
    
    #Toma la Lectura cruda (0–65535) convertida a voltaje (0–3.3 V)   
    def tomar_lectura(self) -> float:
       valor = self.adc.read_u16()
       voltaje = (valor / 65535) * 3.3
       return voltaje
    
    # Toma n cantidad de muestras con un intervalo en segundos
    def muestrear(self, n:int, intervalo:float=0.1) -> list[float]:
        muestra = []
        for _ in range(n):
            l = self.tomar_lectura()
            muestra.append(l)
            time.sleep(intervalo)
        return muestra
    #Determina la frecuencia de muestreo.
    def muestrear_freq(self, n:int, frecuencia:float=50.0) -> list[float]:
        """
        Toma n muestras a una frecuencia dada (Hz).
        Ejemplo: frecuencia=50 → intervalo=0.02 s
        """
        intervalo = 1.0 / frecuencia
        muestra = []
        for _ in range(n):
            l = self.tomar_lectura()
            muestra.append(l)
            time.sleep(intervalo)
        return muestra

# Uso en un bucle infinito

lector = Lector(26)  # GP26 → ADC0

while True:
   # Bloque de muestreo (ejemplo: 5 muestras cada ciclo)
   datos = lector.muestrear_freq(128, frecuencia=100)
   print(datos)

   # Pausa entre ciclos
   time.sleep(2)
Modulo puerto
EL archivo (puertos.py) se encarga de correr el código en la PC. Detecta qué puertos seriales están disponibles, por ejemplo: COM3, para saber dónde está conectado el Raspberry Pi Pico. Su trabajo es listar los puertos activos, permitiendo identificar el puerto correcto para abrir la comunicación. Es un "explorador" de conexiones, ayuda a la PC a encontrar el Pico antes de empezar a leer datos.

#Importaciones para manejar puertos seriales en python.
import serial
import serial.tools.list_ports

#Clase puertos.
class Puertos:
    #Constructor: Inicializa la clase.
    def __init__(self):
        self.lista = serial.tools.list_ports
#Devuelve una lista con cada puerto serial detectado.
    def puertos_disponibles(self) -> list:
        puertos = self.lista.comports()
        return puertos
    
    
Modulo signal
El archivo (analyzer.py) abre el puerto serial y recibe los datos que el Pico envía. Su función es leer líneas completas del puerto (leer_linea) la cual es una de las funciones que conforman a la clase Analyzer. También convierte cadenas con formato de lista (ejemplo:[0.12, 0.34]) en listas de floats (leer_valores) con otra de las funciones de la clase Analyzer.

#Permite abrir y manejar un puerto serial en de la PC para comunicar con dispositivos externos como el Raspberry Pi Pico.
from serial import Serial

class Analyzer: 
    #Constructor. baudrate: velocidad de transmisión (bits por segundo). Por defecto 115200. port: puerto serial a usar en este caso "COM3".

    def __init__(self,baudrate: int = 115200, port: str = "COM3"):
        self.bautrate = baudrate
        self.port = port
        self.ser = Serial(self.port, self.bautrate, timeout=1)

#Lee una línea completa desde el puerto serial (readline()). Decodifica los bytes recibidos a texto UTF-8. Elimina espacios y saltos de línea con .strip(). Devuelve la línea como string.
    def leer_linea(self) -> str:
        raw = self.ser.readline()
        return raw.decode("utf-8").strip()

#Llama a leer_linea() para obtener una cadena y verifica si la cadena está en formato de lista, es decir: Empieza con [ y termina con ]. Si cumple: Quita los corchetes (linea[1:-1]), separa los valores por comas, convierte cada valor a float y devuelve la lista de números. Si no cumple, devuelve una lista vacía.

    def leer_valores(self) -> list[float]:
        linea = self.leer_linea()
        if linea.startswith("[") and linea.endswith("]"):
            valores = linea[1:-1].split(",")
            response = [float(v.strip()) for v in valores]
            return response
        return []
Modulo Analizador
El archivo (analizador.py) aplica la Transformada Rápida de Fourier (FFT) a las muestras recibidas. Convierte los datos crudos provenientes de Raspberry Pi Pico en información de frecuencia. Su trabajo es calcular la FFT con el algoritmo Cooley–Tukey (El algoritmo Cooley–Tukey es el método más usado para calcular la Transformada Rápida de Fourier (FFT). Divide una Transformada Discreta de Fourier (DFT) grande en varias más pequeñas, reduciendo el tiempo de cálculo.)

#Importaciones para usar funciones matematicas simples y complejas requeridad por la FFt.
import math
import cmath

#Clase del analizador, contiene una serie de funciones que ayudan a pasar la señal del dominio del tiempo al dominio de la frecuencia-

class Analizador:

    #Constructor, inicializa la clase.
    def __init__(self, señal: list[float]):
        """
        Inicializa la clase con una señal (lista de números).
        """
        self.señal = señal
        self.N = len(señal)
        self.transformada: list[complex] | None = None

    #Función para calcular la FFT, devuelve una lista de números complejos.

    def fft(self) -> list[complex]:
        """
        Calcula la Transformada Rápida de Fourier (FFT) usando el algoritmo recursivo Cooley–Tukey.
        Requiere que la longitud de la señal sea potencia de 2.
        """
        #Divide la señal en dos partes: índices pares (even) e impares (odd).

        def _fft(x: list[complex]) -> list[complex]:
            n = len(x)
            if n <= 1:
                return x
            even = _fft(x[0::2])
            odd = _fft(x[1::2])
            T = [cmath.exp(-2j * math.pi * k / n) * odd[k] for k in range(n // 2)]
            return [even[k] + T[k] for k in range(n // 2)] + \
                   [even[k] - T[k] for k in range(n // 2)]

        #De señal a complejos.

        señal_c = [complex(x, 0) for x in self.señal]
        X = _fft(señal_c)
        self.transformada = X
        return X

    #Calcula la magnitud de un número complejo devolvendo el resultado como un número complejo cuya parte imaginaria es 0

    def abs_complejo_formula(z: complex) -> complex:
        """
        Calcula el valor absoluto de un número complejo y lo devuelve como complejo.
        """
        if isinstance(z, complex):
            a = z.real
            b = z.imag
            magnitud: float = math.sqrt(a*a + b*b)
            return complex(magnitud, 0)
        try:
            return complex(abs(z), 0)
        except Exception as e:
            raise TypeError(f"No se pudo calcular |z| para tipo {type(z)}: {e}")


    #Recibe un número complejo z en su entrada y devuelve un número real (float) que representa la fase.

    def fase(z: complex) -> float:
        """
        Calcula la fase (argumento) de un número complejo en radianes.
        """
        return cmath.phase(z)
    
   #Devuelve una lista de números reales (list[float]) referente a las magnitudes obtenidas por la fft con un mensaje de advertencia si no se ha calculado la fft.

    def obtener_magnitudes(self) -> list[float]:
        """
        Devuelve la lista de magnitudes de la transformada.
        """
        if self.transformada is None:
            raise ValueError("Primero debes calcular la FFT.")
        return [abs(z) for z in self.transformada]

    #Delvuelve una lista de fases de la transformada fft.
    def obtener_fases(self) -> list[float]:
        """
        Devuelve la lista de fases (argumentos en radianes) de la transformada.
        """
        if self.transformada is None:
            raise ValueError("Primero debes calcular la FFT.")
        return [cmath.phase(z) for z in self.transformada]


#Bloque de prueba que solo se ejecuta solo si corres el archivo directamente
if __name__ == "__main__":
    #Señal de ejemplo
    señal = [0, 1, 0, -1]
    tf = Analizador(señal)

    #Ejecutar la FFT
    resultado = tf.fft()
    print("Resultado FFT:", resultado)

    #Magnitudes y fases
    print("Magnitudes:", tf.obtener_magnitudes())
    print("Fases:", tf.obtener_fases())

    #Ejemplo de uso de métodos estáticos
    print("Fase de (1+1j):", Analizador.fase(complex(1, 1)))
    print("Magnitud de (3+4j):", Analizador.abs_complejo_formula(complex(3, 4)))
Su respuesta son las magnitudes y fases de cada componente de frecuencia.

Modulo: graficador
El archivo "graficador.py" genera gráficas de magnitud y fase de la FFT usando matplotlib a partir de los datos proporcionados por la FFT. Lo cual nos permite visualizar el espectro de la señal.

#Importaciones para poder visualizar una grafica en este caso y tambien se importa del modulo dentro de app al analizador descrito anteriormente.

import matplotlib.pyplot as plt
from app.analizador.analizador import Analizador

#Clase del graficador.
class GraficadorFFT:
   #Constructor: Inicializa la clase. Es mas dinamico con la instroducción de datos.
   def __init__(self, señal, fs=1.0):
       self.tf = Analizador(señal)
       self.tf.fft()
       self.magnitudes = self.tf.obtener_magnitudes()
       self.fases = self.tf.obtener_fases()
       self.fs = fs
       self.N = len(señal)

#- Crea el primer gráfico en una figura con 2 filas y 1 columna. El número 1 indica que es el primer subplot (arriba). plt.title("Magnitud de la Transformada") Asigna el título al primer gráfico; plt.plot(self.magnitudes, marker="o", color="blue") Dibuja la curva de las magnitudes de la FFT. Usa círculos (marker="o") y color azul.

   def graficar(self):
       plt.subplot(2, 1, 1)
       plt.title("Magnitud de la Transformada")
       plt.plot(self.magnitudes, marker="o", color="blue")

       plt.subplot(2, 1, 2)
       plt.title("Fase de la Transformada")
       plt.plot(self.fases, marker="o", color="orange")

       plt.tight_layout()
       plt.show()
Orquestador: run.py
Llama a todas las clases para que trabajen en conjunto y poder analizar la señal proveniente del Raspberry para ello, se llaman a las clases para trabajos en particular. En este caso:

Importa Puertos para listar los puertos disponibles; Importa Analyzer para abrir el puerto y recibir los datos del Raspberry Pi Pico; Los datos obtenidos del Analyzer pasan al Analizador para ser procesados y posteriormente al Graficador donde se hace visible la serie de procedimientos realizados por cada uno de los modulos dentro de app.

#Importa 4 de nuestros 5 modulos.(signal, analizador, puertos y graficador para hacer las mediciones.)
from app.signal.analyzer import Analyzer
from app.analizador.analizador import Analizador
from app.puertos import Puertos
from app.graficador.graficador import GraficadorFFT

#se determina que "puertos: llama a la clase Puertos() y raspberry: llama a la clase Analyzer(). Con el objetivo de implementar sus calculos y listas.
puertos = Puertos()
raspberry = Analyzer()

#Nos muestra puertos disponibles
puertos_disponibles = puertos.puertos_disponibles()
for puerto in puertos_disponibles:
    print(puerto)

#Usa Analyzer.leer_valores() esperando recibir una cadena tipo "[0.12, 1.34, 2.56]" desde el Raspberry Pi Pico. Convierte esa cadena en una lista de floats e imprime cuántas muestras se recibieron.

muestras = raspberry.leer_valores()
print(f"Recibiendo {len(muestras)} muestras")
#Si hay más de una muestra: Analizador(muestras) prepara los datos para aplicar FFT; GraficadorFFT(muestras) instancia el graficador con las muestras; graficos.graficar() genera la gráfica; ff.fft() → calcula la transformada rápida de Fourier y devuelve los valores complejos.

#Si no hay suficientes muestras, imprime un aviso

if len(muestras) > 1:
    ff = Analizador(muestras)
    graficos = GraficadorFFT(muestras)
    graficos.graficar()
    complejos = ff.fft()
else:
    print("No hay muestras que analizar")

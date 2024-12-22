# WaterQuality
Medidor Qualidade de Agua
Autor: Renato F. Fernandes e Maria Beatriz Fernandes.
Data: Nov/2023

Este programa utiliza três sensores para medir a qualidade da agua
1 - Sensor de Turbidez
2 - Sensor de Condutividade
3 - Sensor de temperatura

Ele tambem se comunica serialmente com uma placa ESP32 para mostrar os valores do sensor em uma pagina web
O codigo fonte da ESP32 esta na pasta Examples\Esp32Mariot6.ino
Nesta pagina existe tambem um arquivo html que eh a captura da pagina principal do programa.
Para este caso, preferi fazer a pagina em um editor de CSS e salvei em formato base64 e somente copiei a imagem gerada dentro do codigo fonte (fica grande, mas eh mais 
facil do que passar um link para uma pagina web de algum lugar, caso nao tiver internet disponivel no local que vc vai usar).

Este programa foi feito em uma placa Arduino UNO.



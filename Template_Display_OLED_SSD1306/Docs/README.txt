Este template foi criado por:
	Leandro Poloni Dantas

Este projeto é baseado nas seguintes referências:
	https://youtu.be/KaQrp-uNqBs?si=_75WQSk_Xb2vgHz5
	https://github.com/afiskon/stm32-ssd1306/tree/master
	https://controllerstech.com/oled-display-using-i2c-stm32/
	https://youtu.be/M5ddTjrcvEs?si=rjwZjhCSfjuTwVBV

Usei a biblioteca disponível no Github, porém a biblioteca do site Controllestech
é praticamente igual com acréscimo das funções de scroll.  

Para criação de imagens monocromáticas usei o site:
	https://projedefteri.com/en/tools/lcd-assistant/
	
	Parâmetros para o fantasma_128x64:
		Canvas size(s): 128x64
		Background Color: White
		Scaling: scale to fit, keeping proportions
		Code output format: Arduino Code
		Draw Mode: Horizontal - 1 bit per pixel
		
	Parâmetros para o fantasma_50x50:
		Canvas size(s): 50x50
		Invert image colors: checked
		Background Color: Black
		Scaling: scale to fit, keeping proportions
		Code output format: Arduino Code
		Draw Mode: Horizontal - 1 bit per pixel

Outra opção para criação das imagens é o programa LCD Assistant disponível em:
	https://en.radzio.dxp.pl/bitmap_converter/


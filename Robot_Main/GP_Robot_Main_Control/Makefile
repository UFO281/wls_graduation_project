target := main

relyon := app_gpio.c 
# w_can/wcan.c w_uart/wuart.c 

compile := arm-linux-gnueabihf-gcc
# compile := 
#-I/usr/local/freetype/include/freetype2 -L/usr/local/lib -lm

target :$(relyon)
	$(compile) -o $(target) $(relyon) -lpthread 
# $(compile) -o $(target) $(relyon) -lpthread -I./w_uart./w_can


clean:
	rm -rf $(target)






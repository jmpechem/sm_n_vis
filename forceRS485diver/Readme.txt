
Compile driver : make 

Install driver : insmod xr17v35x.ko

Uninstall driver : rmmod xr17v35x.ko

The follwing code is included in "static int serialxr_startup(struct uart_port *port)" function to enable auto RS485 mode. 

/*Enables Auto RS485 mode for all ports 
	 *This Part of code enables Auto RS485 mode permanantly for all ports
	 */

	value = 0 ;
	value=serial_in(up,0x08);
	value |=1<<5;
	serial_out(up, 0x08,value );

	value=serial_in(up,0x03);
	value |=1<<7;
	serial_out(up, 0x03,value );

	value=serial_in(up,0x02);
	value |=1<<7;
	serial_out(up, 0x02,value );

	value=serial_in(up,0x03);
	value &=~(1<<7);
	serial_out(up, 0x03,value );

	/*Enabling Auto RS485 mode code ends here
	 */


The following code is modified in "static int serialxr_startup(struct uart_port *port)" function to xenomai driver system. (highspeed baudrate control; only using 8x mode)


	if(1)
	{//using the 8x mode
		val_4xmode &=~(1 << port_index);
		val_8xmode |=(1 << port_index);
		quot_coeff = 8;
		printk(KERN_INFO "Using the 8x Mode\n");
	}
	/*
	if(baud < 12500000/16)
	{//using the 16x mode
		val_4xmode &=~(1 << port_index);
		val_8xmode &=~(1 << port_index);	
		quot_coeff = 16;
		printk(KERN_INFO "Using the 16x Mode\n");
	}
	else if((baud >= 12500000/16)&&(baud < 12500000/4))
	{//using the 8x mode
		val_4xmode &=~(1 << port_index);
		val_8xmode |=(1 << port_index);
		quot_coeff = 8;
		printk(KERN_INFO "Using the 8x Mode\n");
	}
	else 
	{//using the 4x mode
		val_4xmode |=(1 << port_index);
		val_8xmode &=~(1 << port_index);
		quot_coeff = 4;
		printk(KERN_INFO "Using the 4x Mode\n");
	}
	*/

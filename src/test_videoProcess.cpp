/*
  Copyright (C) 1997-2016 Sam Lantinga <slouken@libsdl.org>

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely.
*/

#include "osa.h"
#include "osa_thr.h"
#include "osa_buf.h"
#include "osa_sem.h"
#include "sync422_trans.h"

#define MAX_INPUT_STR_SIZE 		( 128 )
char gAPP_mainMenu[] =
{
	"\r\n ========="
	"\r\n MainMenu"
	"\r\n ========="
	"\r\n 0: Stop test send"
	"\r\n 1: Start test send ch0"
	"\r\n 2: Cycle test send ch0"
	"\r\n 1: Start test send ch1"
	"\r\n 2: Cycle test send ch1"
	"\r\n e: Exit"
	"\r\n"
	"\r\n Enter Choice: "
};

char APP_getChar( )
{
    char buffer[MAX_INPUT_STR_SIZE];

    fflush( stdin );
    fgets( buffer, MAX_INPUT_STR_SIZE, stdin );

    return ( buffer[0] );
}

int main(int argc, char **argv)
{
	Bool ui_loop = TRUE;
	char ch = '0';
	int i, uart=0, mode=0;

	for(i=1; i<argc; i++)
	{
		if(strcmp(argv[i], "a") == 0)
			uart = 0;
		else if(strcmp(argv[i], "b") == 0)
			uart = 1;
		else if(strcmp(argv[i], "v") == 0)
			mode = 0;
		else if(strcmp(argv[i], "p") == 0)
			mode = 1;
	}

	OSA_printf("%s:  enter %d   \n", __func__, __LINE__);
	sync422_spi_create(uart, mode);
	sync422_demo_start();

	while (ui_loop)
	{
		printf(gAPP_mainMenu);

		if (ch == '0')
		{
			testSnd(0, 0);
			testSnd(1, 0);
		}
		if (ch == '1')
		{
			testSnd(0, 1);
		}
		
		if (ch == '2')
		{
			testSnd(0, 2);
		}

		if (ch == '3')
		{
			testSnd(1, 1);
		}
		
		if (ch == '4')
		{
			testSnd(1, 2);
		}
		
		if (ch == 'e')
		{
			ui_loop = FALSE;
			break;
		}

		ch = APP_getChar();
	}

	sync422_demo_stop();
	sync422_spi_destory(uart);

	OSA_printf("%s:  exit %d   \n", __func__, __LINE__);
	return 0;
}



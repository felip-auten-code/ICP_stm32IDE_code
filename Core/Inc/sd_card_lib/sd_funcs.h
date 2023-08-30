
#include "fatfs_sd.h"
#include "fatfs.h"
#include "string.h"
#include "main.h"
//#include "../init_variables.h"


void delete_file(FATFS fs, char* path);
//void write_data(FATFS fs, FIL fil, APP_DATA* app);
//void read_data(FATFS fs, FIL fil, char* path, APP_DATA* app);
char* check_sd(FATFS fs, FIL fil, char* path);

void delete_file(FATFS fs, char* path){
	//f_mount(&fs, "", 0);
	f_unlink(path);
}

void write_PoseEstimation(FATFS fs, FIL fil, float* info, int id){
	char buff[40]; // n1[5], n2[5], n3[5];
	FRESULT stat = f_mount(&fs, "", 0);
	HAL_Delay(300);
	int integer_p[3], decimal_p[3];
	if(stat == 0){
		FRESULT open_file = f_open(&fil, "data_out.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_READ );
		FRESULT seek_filesystem = f_lseek(&fil, fs.fsize);
//		gcvt(info[0], 5, n1);
//		gcvt(info[1], 5, n2);
//		gcvt(info[2], 5, n3);
//		sprintf(buff, "[%s,\t %s, \t %s]\t %d\n", n1, n2, n3, id);
		for (int i =0; i< 3; i++){
			integer_p[i]  = (int) info[i];
			decimal_p[i] = abs((int)((float)(info[i]-integer_p[i]) * 10000));
		}
		memset(buff, 0 , sizeof(buff));
		sprintf(buff, "[%d.%d,\t %d.%d, \t %d.%d]\t %d\n", 	integer_p[0], decimal_p[0],
															integer_p[1], decimal_p[1],
															integer_p[2], decimal_p[2],
															id);
		int status = f_puts(buff, &fil);
		f_close(&fil);
	}
}


void Transmit_UART_PoseEstimation(UART_HandleTypeDef *huart, float* info, int id){
	char buff[40]; // n1[5], n2[5], n3[5];
	HAL_Delay(300);
	int integer_p[3], decimal_p[3];

//		gcvt(info[0], 5, n1);
//		gcvt(info[1], 5, n2);
//		gcvt(info[2], 5, n3);
//		sprintf(buff, "[%s,\t %s, \t %s]\t %d\n", n1, n2, n3, id);
	for (int i =0; i < 3; i++){
		integer_p[i]  = (int) info[i];
		decimal_p[i] = abs((int)((float)(info[i]-integer_p[i]) * 10000));
	}
	memset(buff, 0 , sizeof(buff));
	int len = sprintf(buff, "[%d.%d,\t %d.%d, \t %d.%d]\t id=%d\n", 	integer_p[0], decimal_p[0],
																		integer_p[1], decimal_p[1],
																		integer_p[2], decimal_p[2],
																		id);

	if(HAL_UART_Transmit_IT(huart,(uint8_t*) buff, len) == HAL_OK){
		HAL_Delay(100);
	}else{
		if(HAL_UART_Transmit(huart,(uint8_t*) buff, len, 180)==HAL_OK){
			HAL_Delay(100);
		}
	}

}

//void write_data(FATFS fs, FIL fil, APP_DATA* app){
//	char buff[100];
//	int len=0;
////	int PERIODO_BAIXO, PERIODO_MEDIO, PERIODO_ALTO;
////	int DUTY_BAIXO, DUTY_MEDIO, DUTY_ALTO;
////	int MODO_OPERACAO;
////	uint8_t MARGEM_ERRO;
////	uint16_t 	DESLOC_MARGEM_ERRO;
////	uint16_t 	DESLOC_MARGEM_ERRO_QUADRO;
////	uint16_t 	ALTURA_IDEAL;
////	uint16_t 	ALTURA_QUADRO;
////	uint16_t 	ALTURA_INT_MIN;
//	f_mount(&fs, "", 0);
//	delete_file(fs, "data-setup.txt");
//	f_open(&fil, "data-setup.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
//	f_lseek(&fil, fil.fsize);
//	len = sprintf(buff, "%d\n", app[0].DUTY_BAIXO);					// 0
//	f_puts(buff, &fil);
//	len = sprintf(buff, "%d\n", app[0].DUTY_MEDIO);					// 1
//	f_puts(buff, &fil);
//	len = sprintf(buff, "%d\n", app[0].DUTY_ALTO);					// 2
//	f_puts(buff, &fil);
//	len = sprintf(buff, "%d\n", app[0].PERIODO_BAIXO);				// 3
//	f_puts(buff, &fil);
//	len = sprintf(buff, "%d\n", app[0].PERIODO_MEDIO);				// 4
//	f_puts(buff, &fil);
//	len = sprintf(buff, "%d\n", app[0].PERIODO_ALTO);				// 5
//	f_puts(buff, &fil);
//	len = sprintf(buff, "%d\n", app[0].DUTY_BAIXO_);				// 6
//	f_puts(buff, &fil);
//	len = sprintf(buff, "%d\n", app[0].DUTY_MEDIO_);				// 7
//	f_puts(buff, &fil);
//	len = sprintf(buff, "%d\n", app[0].DUTY_ALTO_);					// 8
//	f_puts(buff, &fil);
//	len = sprintf(buff, "%d\n", app[0].PERIODO_BAIXO_);				// 9
//	f_puts(buff, &fil);
//	len = sprintf(buff, "%d\n", app[0].PERIODO_MEDIO_);				// 10
//	f_puts(buff, &fil);
//	len = sprintf(buff, "%d\n", app[0].PERIODO_ALTO_);				// 11
//	f_puts(buff, &fil);
//	len = sprintf(buff, "%d\n", app[0].DESLOC_MARGEM_ERRO);			// 12
//	f_puts(buff, &fil);
//	len = sprintf(buff, "%d\n", app[0].DESLOC_MARGEM_ERRO_QUADRO);	// 13
//	f_puts(buff, &fil);
//	len = sprintf(buff, "%d\n", app[0].ALTURA_IDEAL);				// 14
//	f_puts(buff, &fil);
//	len = sprintf(buff, "%d\n", app[0].ALTURA_QUADRO);				// 15
//	f_puts(buff, &fil);
//	len = sprintf(buff, "%d\n", app[0].ALTURA_INT_MIN);				// 16
//	f_puts(buff, &fil);
//	len = sprintf(buff, "%d\n", app[0].MODO_OPERACAO);				// 17
//	f_puts(buff, &fil);
//	len = sprintf(buff, "%d\n", app[0].MARGEM_ERRO);				// 18
//	f_puts(buff, &fil);
//
//
//
//	f_close(&fil);
//	len = abs(-len);
//}

//void read_data(FATFS fs, FIL fil, char* path, APP_DATA* app){
////	f_mount(&fs, "", 0);
//	char buffer[10];
//	char* buff;
//	f_mount(&fs, "", 0);
//	f_open(&fil, "data-setup.txt", FA_READ);
//	// Reads line by line until the end
//	int ctt =0;
//	while(f_gets(buffer, sizeof(buffer), &fil))
//	{
//	// Can use the buffer for something useful
//	//	  memset(buffer,0,sizeof(buffer));
//		buff = buffer;
//
//		if(ctt==0){
//			app[0].DUTY_BAIXO = atoi(buff);
//		}else if(ctt==1){
//			app[0].DUTY_MEDIO = atoi(buff);
//		}else if(ctt==2){
//			app[0].DUTY_ALTO = atoi(buff);
//		}else if(ctt==3){
//			app[0].PERIODO_BAIXO = atoi(buff);
//		}else if(ctt==4){
//			app[0].PERIODO_MEDIO = atoi(buff);
//		}else if(ctt==5){
//			app[0].PERIODO_ALTO = atoi(buff);
//		}else if(ctt==6){
//			app[0].DUTY_BAIXO_ = atoi(buff);
//		}else if(ctt==7){
//			app[0].DUTY_MEDIO_ = atoi(buff);
//		}else if(ctt==8){
//			app[0].DUTY_ALTO_ = atoi(buff);
//		}else if(ctt==9){
//			app[0].PERIODO_BAIXO_ = atoi(buff);
//		}else if(ctt==10){
//			app[0].PERIODO_MEDIO_ = atoi(buff);
//		}else if(ctt==11){
//			app[0].PERIODO_ALTO_ = atoi(buff);
//		}else if(ctt==12){
//			app[0].DESLOC_MARGEM_ERRO = atoi(buff);
//		}else if(ctt==13){
//			app[0].DESLOC_MARGEM_ERRO_QUADRO = atoi(buff);
//		}else if(ctt==14){
//			app[0].ALTURA_IDEAL = atoi(buff);
//		}else if(ctt==15){
//			app[0].ALTURA_QUADRO= atoi(buff);
//		}else if(ctt==16){
//			app[0].ALTURA_INT_MIN = atoi(buff);
//		}else if(ctt==17){
//			app[0].MODO_OPERACAO = atoi(buff);
//		}else if(ctt==18){
//			app[0].MARGEM_ERRO = atoi(buff);
//		}
//
//		HAL_Delay(1);
//		ctt++;
//	}
//	f_close(&fil);
//}

char* check_sd(FATFS fs, FIL fil, char* path){
	f_mount(&fs, "", 0);
	//f_open(&fil, "data-setup.txt", FA_READ);
	if(f_open(&fil, path, FA_READ) != 0){
		return "not-ok";
	}else{
		return "ok";
	}
	return 0;
}

int SD_test(FATFS fs, FIL fil){
	int out = f_mount(&fs, "", 0);
	f_close(&fil);
	return out;
}



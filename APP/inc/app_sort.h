/**
  *******************************************************************************************************
  * File Name: app_sort.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-1
  * Brief: 本文件提供了排序函数的声明
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date:	2018-3-1
	*			Mod: 建立文件
  *
  *******************************************************************************************************
  */	

# ifndef __APP_SORT_H
# define __APP_SORT_H

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp.h"

void sort_QuickSort(uint16_t Array[], int32_t StartIndex, int32_t EndIndex);
//void sort_MergeSort(int32_t Array[], int32_t StartIndex, int32_t EndIndex);
void sort_SelectSort(int32_t Array[], int32_t ArrayLength);
void sort_InsertSort(int32_t Array[], int32_t ArrayLength);

# endif

/********************************************  END OF FILE  *******************************************/

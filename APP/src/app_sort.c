/**
  *******************************************************************************************************
  * File Name: app_sort.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-1
  * Brief: 本文件提供了排序函数的定义
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date:	2018-3-1
	*			Mod: 建立文件
  *
  *******************************************************************************************************
  */	


/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "app_sort.h"


/*
*********************************************************************************************************
*                       Partition                   
*
* Description: 将数组分为两个部分,pivot左边的小于pivot,右边的大于pivot
*             
* Arguments  : 1> Array[]: 数组
*              2> StartIndex: 数组起始下标
*							 3> EndIndex: 数组结束下标
*
* Reutrn     : 1> pivot在数组中的下标
*
* Note(s)    : 该函数无排序功能,为快速排序中的一部分
*********************************************************************************************************
*/
int Partition(uint16_t Array[], int32_t StartIndex, int32_t EndIndex)
{
	int i = StartIndex, j = EndIndex;
	uint16_t pivot = Array[StartIndex];
	int temp;
	
	while(i < j)		
	{
		/*  从右向左找到第一个大于pivot的数  */
		while(i < j && (Array[j] >= pivot)) j--;
		/*  从左向右查找  */
		while(i < j && (Array[j] <= pivot)) i++;

		if(i < j)		/*  交换两个数  */
		{
			temp = Array[i];
			Array[i] = Array[j];
			Array[j] = temp;
		}
	}
	
	Array[StartIndex] = Array[i];
	Array[i] = pivot;
	return i;
}

/*
*********************************************************************************************************
*                            sort_QuickSort              
*
* Description: 快速排序算法,从小到大排序
*             
* Arguments  : 1> Array[]: 要排序的数组
*              2> StartIndex: 开始数组下标
*              3> EndIndex: 数组结束下标
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void sort_QuickSort(uint16_t Array[], int32_t StartIndex, int32_t EndIndex)
{
	int q = 0;
	if (StartIndex < EndIndex)
	{
		q = Partition(Array, StartIndex , EndIndex);
		sort_QuickSort(Array, StartIndex, q - 1);
		sort_QuickSort(Array, q + 1, EndIndex);
	}
}


/*
*********************************************************************************************************
*                       sort_SelectSort                   
*
* Description: 选择排序算法,从小到大排序
*             
* Arguments  : 1> Array[]: 要排序的数组
*              2> ArrayLength: 数组长度
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void sort_SelectSort(int32_t Array[], int32_t ArrayLength)
{
	int i = 0, j = 0;
	int index = 0;
	int key = 0;

	for (i = 0; i < ArrayLength; i++)
	{
		key = Array[i];
		j = i;
		index = j;
		while (j < ArrayLength)
		{
			if (key > Array[j])		/* 挣到最小的值,然后交换 */
			{
				index = j;				/* 保存最小值的下标 */
				key = Array[j];       /* 更新key */
			}
			j++;
		}

		Array[index] = Array[i];
		Array[i] = key;			/* 插入找到的最小值 */
	}
}

/*
*********************************************************************************************************
*                       sort_InsertSort                   
*
* Description: 插入排序算法,从小到大排序
*             
* Arguments  : 1> Array[]: 要排序的数组
*              2> ArrayLength: 数组长度
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void sort_InsertSort(int32_t Array[], int32_t ArrayLength)
{
	int i = 1, j = 0;
	int32_t key = 0;
	
	for(; i < ArrayLength; i++)
	{
		key = Array[i];
		j = i - 1;
		while(j >= 0 && Array[j] < key)
		{
			Array[j + 1] = Array[j];
			j--;
		}
		Array[j + 1] = key;
	}
}

/********************************************  END OF FILE  *******************************************/


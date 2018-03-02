/**
  *******************************************************************************************************
  * File Name: app_sort.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-1
  * Brief: ���ļ��ṩ���������Ķ���
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date:	2018-3-1
	*			Mod: �����ļ�
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
* Description: �������Ϊ��������,pivot��ߵ�С��pivot,�ұߵĴ���pivot
*             
* Arguments  : 1> Array[]: ����
*              2> StartIndex: ������ʼ�±�
*							 3> EndIndex: ��������±�
*
* Reutrn     : 1> pivot�������е��±�
*
* Note(s)    : �ú�����������,Ϊ���������е�һ����
*********************************************************************************************************
*/
int Partition(uint16_t Array[], int32_t StartIndex, int32_t EndIndex)
{
	int i = StartIndex, j = EndIndex;
	uint16_t pivot = Array[StartIndex];
	int temp;
	
	while(i < j)		
	{
		/*  ���������ҵ���һ������pivot����  */
		while(i < j && (Array[j] >= pivot)) j--;
		/*  �������Ҳ���  */
		while(i < j && (Array[j] <= pivot)) i++;

		if(i < j)		/*  ����������  */
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
* Description: ���������㷨,��С��������
*             
* Arguments  : 1> Array[]: Ҫ���������
*              2> StartIndex: ��ʼ�����±�
*              3> EndIndex: ��������±�
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
* Description: ѡ�������㷨,��С��������
*             
* Arguments  : 1> Array[]: Ҫ���������
*              2> ArrayLength: ���鳤��
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
			if (key > Array[j])		/* ������С��ֵ,Ȼ�󽻻� */
			{
				index = j;				/* ������Сֵ���±� */
				key = Array[j];       /* ����key */
			}
			j++;
		}

		Array[index] = Array[i];
		Array[i] = key;			/* �����ҵ�����Сֵ */
	}
}

/*
*********************************************************************************************************
*                       sort_InsertSort                   
*
* Description: ���������㷨,��С��������
*             
* Arguments  : 1> Array[]: Ҫ���������
*              2> ArrayLength: ���鳤��
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


/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
  * @file    integral_algorithm.c
  * @author  LJY 2250017028@qq.com
  * @brief   Code for integral algorithm.
  * @date    2021-01-30
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author     <th>Description
  * <tr><td>2021-01-31  <td> 1.0     <td>LJY  			<td>Creator
	* <tr><td>2021-02-22  <td> 1.1     <td>LJY  			<td>Change the frame
  * </table>
  *
  ==============================================================================
                              How to use this driver  
  ==============================================================================
    @note
      -# ����ʵ������
	     Integral_Lib inte_lib;		 
        
			-# ��ȡ����ֵ
				 inte_lib.I(ref_x , ref_y, ref_y_dot, n);
		 
    @warning	
			-# ����ʵ��ʱ��Ҫѡ����������,�����㷨�Լ�׼ȷ��, ��ѡ�������Ĭ��ֵ
      -# �ÿ�ο�����ֵ����̿��飬�в�����ϸ�ڿ���ȥ�����Ҵ�
			-# �ÿ�ֳ�����㷨�ͻ����㷨������
			
	  
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2021 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "integral_algorithm.h"
#include <stdio.h>
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/


/********************************************����㷨************************************************
	* @attention
	* -# �������պ�ţ����϶�û���õ�����ĵ���,�������ֻ��Ϊ��ͳһ�ӿڡ����û�еĻ�,����0Ҳ�޷���
	* -# �ڵ�����׼ȷ������£��������ز�ֵ�������ȷ��
	**************************************************************************************************/
	
	
/**
 *  @brief �������ղ�ֵ��Ϸ�  
 *	@note	�����СҪ������ĵ������ƥ��
 * 	@param[in] 	ref_x �ο���ĺ���������
 * 	@param[in] 	ref_y �ο��������������
 * 	@param[in] 	ref_y_dot �ο���ĵ�������(����û���õ�,ֻ��Ϊ��ͳһ�ӿ�)
 *	@param[in] 	n 		�ο���ĸ���
 *  @param[in] 	out_x ����ĵ����������
 *  @param[out] out_y ����������������
 *  @param[in] 	m 		�����ĸ���
 *  @return  void
 */
 void Integral_Lib::Lagrange_Interpolation(const float* ref_x, const float* ref_y, const float* ref_y_dot, uint16_t n, const float* out_x, float* out_y, uint16_t m)
 {
		int i, j, k;
	  float mol, deno;	//��¼ÿ������ʽ���Ӻͷ�ĸ��ֵ
	 
	 /* �Ƚ���������ֵ���� */
	 for(i = 0; i < m; i ++)
	 {
		 out_y[i] = 0;
	 }
	 
	 /* ÿ�������ĺ�������� */
	 for(i = 0; i < m; i ++)
	 {
		 /* (n-1)�ζ���ʽ */
		 for(j = 0; j < n; j ++)
		 {
			 mol = 1.0;
			 deno = 1.0;
			 /* ���ÿ������ʽ��ֵ */
			 for(k = 0; k < n; k ++)
			 {
				 if(j != k)
				 {
						mol *= out_x[i] - ref_x[k];
						deno *= ref_x[j] - ref_x[k];
				 }
			 }
			 
			 /* ����ʽ�ۼ� */
			 out_y[i] += ref_y[j] * mol / deno; 
		 }
	 }
 }
 
/**
 *  @brief ţ�ٲ�ֵ��Ϸ�  
 *	@note	�����СҪ������ĵ������ƥ��
 * 	@param[in] 	ref_x �ο���ĺ���������
 * 	@param[in] 	ref_y �ο��������������
 * 	@param[in] 	ref_y_dot �ο���ĵ�������(����û���õ�,ֻ��Ϊ��ͳһ�ӿ�)
 *	@param[in] 	n 		�ο���ĸ���
 *  @param[in] 	out_x ����ĵ����������
 *  @param[out] out_y ����������������
 *  @param[in] 	m 		�����ĸ���
 *  @return  void
 */
 void Integral_Lib::Newton_Interpolation(const float* ref_x, const float* ref_y, const float* ref_y_dot, uint16_t n, const float* out_x, float* out_y, uint16_t m)
 {
	 int i, j, k, l;
	 float mol, deno;	//��¼ÿ�����̵ķ��Ӻͷ�ĸ��ֵ
	 float diff;			//��¼ÿ������ʽ�Ĳ���
	 float single;		//��¼ÿ������ʽ��ֵ
	
	 /* �Ƚ���������ֵ���� */
	 for(i = 0; i < m; i ++)
	 {
		 out_y[i] = 0;
	 }

	 /* ÿ�������ĺ�������� */
	 for(i = 0; i < m; i ++)
	 {
		 /* (n-1)�ζ���ʽ */
		 for(j = 0; j < n; j ++)
		 { 
			 if(j == 0) out_y[i] += ref_y[j];
			 else
			 {
				 /* ����ÿ������ʽ�Ĳ��� */
				 diff = 0;
				 for(k = 0; k <= j; k ++)
				 { 	 
					 mol = ref_y[k];
					 deno = 1.0;
					 for(l = 0; l <= j; l ++)
					 {
						 if(l != k) deno *= ref_x[k] - ref_x[l];
					 }
					 diff += mol / deno;
				 }
				 
				  /* ����ÿ������ʽ */
				 single = 1.0;
				 for(k = 0; k < j; k ++)
				 {
					 single *= out_x[i] - ref_x[k];
				 }
				 
					/* ����ʽ�ۼ� */
					out_y[i] += single * diff; 				 
			 }		
		 }
	 }		
 }
 
 /**
 *  @brief �������ز�ֵ��Ϸ�(δ����) 
 *	@note	�����СҪ������ĵ������ƥ��
 * 	@param[in] 	ref_x 		�ο���ĺ���������
 * 	@param[in] 	ref_y 		�ο��������������
 * 	@param[in] 	ref_y_dot �ο���ĵ�������
 *	@param[in] 	n 				�ο���ĸ���
 *  @param[in] 	out_x 		����ĵ����������
 *  @param[out] out_y 		����������������
 *  @param[in] 	m 				�����ĸ���
 *  @return  void
 */
 void Integral_Lib::Hermite_Interpolation(const float* ref_x, const float* ref_y, const float* ref_y_dot, uint16_t n, const float* out_x, float* out_y, uint16_t m)
 {
	 int i, j, k;
	 float alpha, beta;							//��¼����ʽ�е�ϵ��
	 float mol, deno;								//��¼�������ղ�ֵϵ���ķ��Ӻͷ�ĸ
	 float temp_1, temp_2, temp_3;	//��¼ϵ���еĹ�����
	 
	 /* �Ƚ���������ֵ���� */
	 for(i = 0; i < m; i ++)
	 {
		 out_y[i] = 0;
	 }
	 
	 /* ÿ�������ĺ�������� */
	 for(i = 0; i < m; i ++)
	 {
		 /* (n-1)�ζ���ʽ */
		 for(j = 0; j < n; j ++)
		 {		 
			 /* �����������ղ�ֵ������ */
			 mol = 1.0;
			 deno = 1.0;
			 for(k = 0; k < n; k ++)
			 {
				 if(j != k)
				 {
						mol *= out_x[i] - ref_x[k];
						deno *= ref_x[j] - ref_x[k];
				 }
			 }
			 temp_1 = mol / deno;
			 temp_1 *= temp_1;
			 
			 /* ����ϵ��alpha */
			 temp_2 = 2 * ( out_x[i] - ref_x[j]);
			 temp_3 = 0;
			 for(k = 0; k < n; k ++) 
			 {
				 if(k != j) temp_3 += 1 / (ref_x[j] - ref_x[k]);
			 }
			 temp_2 *= temp_3;
			 alpha = (1 - temp_2) * temp_1;
			 
			 /* ����ϵ��beta */
			 beta = (out_x[i] - ref_x[j]) * temp_1;
			 
			 /* ����ʽ�ۼ� */
			 out_y[i] += alpha * ref_y[j] + beta * ref_y_dot[j];
		 }
	 }
 }
 
 
 /********************************************�����㷨************************************************
	* @attention
	* -# �����㷨���õ���new����������,�洢��ʱ����,ʹ��ʱaccuracy���ܸ�̫��,����ᱬջ��
	* -# �Ƽ���accuracyֵ(����accuracyԽ������ʱ��Խ��������ʹ�ö���Ҫ�����������):
				Trapezoid_Integral 20
				Parabola_Integral  15
				Vortex_Integral		 10
				Romberg_Integral   5
	**************************************************************************************************/
 
 /**
 *  @brief ������������� 
 *	@note	�����СҪ������ĵ������ƥ��
 * 	@param[in] 	func 			����㷨�ĺ���ָ��
 * 	@param[in] 	ref_x 		�ο���ĺ���������
 * 	@param[in] 	ref_y 		�ο��������������
 * 	@param[in] 	ref_y_dot �ο���ĵ�������
 *	@param[in] 	n 				�ο���ĸ���
 *  @param[in] 	accuracy	��ȷ�ȣ������������仮�ֳɶ��ٸ�С���䣬ȡֵӦ >= 1
 *  @return  		res				����ֵ
 */
 float Integral_Lib::Trapezoid_Integral(fitting_func func, const float* ref_x, const float* ref_y, const float* ref_y_dot, uint16_t n, uint16_t accuracy)
 {
	 float h;				//���ֲ���
	 float Tn = 0;	//����ֵ
	 uint16_t len;	//������Ϻ����м����ֵ�ĸ���
	 
	 /* ���ж�nֵ�Ĵ�С,��ֹ����������� */
	 if(n <= 0) return 0;
	 else if(n == 1) return ref_y[0];
	 else
	 {
		 /* ���㲽�� */
		 h = (ref_x[n-1] - ref_x[0]) / accuracy;
		 
		 /* ������ֵ�ǰ�벿�� */
		 Tn += h / 2 * (ref_y[0] + ref_y[n-1]);
		 
		 /* �жϾ���ֵ��С */
		 if(accuracy == 1) return Tn;
		 else
		 {
			 /* �����������鳤�� */
			 len = accuracy - 1;
			 
			 /* �����������洢Ҫ��������㷨��������Ա��� */
			 float* cal_x = new float[len];
			 for(int i = 0; i < len; i++) cal_x[i] = ref_x[0] + h * (i + 1);
			 
			 /* ��������洢������ */
			 float* result = new float[len];
			 
			 /* ������Ϻ����������� */
			 func(ref_x, ref_y, ref_y_dot, n, cal_x, result, len);
			 
			 /* �ۼ������ */
			 for(int i = 0; i < len; i++)
			 {
				 Tn += h * result[i];
			 }
			 
			 /* ����ڴ� */
			 delete []cal_x;
			 delete []result;
			 return Tn;
		 }
	 }
 }
 
 
 /**
 *  @brief ��������������� 
 *	@note	�����СҪ������ĵ������ƥ��
 * 	@param[in] 	func 			����㷨�ĺ���ָ��
 * 	@param[in] 	ref_x 		�ο���ĺ���������
 * 	@param[in] 	ref_y 		�ο��������������
 * 	@param[in] 	ref_y_dot �ο���ĵ�������
 *	@param[in] 	n 				�ο���ĸ���
 *  @param[in] 	accuracy 	��ȷ�ȣ������������仮�ֳɶ��ٸ�С����
 *  @return  		res				����ֵ
 */
  float Integral_Lib::Parabola_Integral(fitting_func func, const float* ref_x, const float* ref_y, const float* ref_y_dot, uint16_t n, uint16_t accuracy)
 {
	 float Tn, T2n; //���ø������η��ĵ��������
	 float Sn = 0;	//����ֵ
	 Tn = Trapezoid_Integral(func, ref_x, ref_y, ref_y_dot, n, accuracy);
	 T2n = Trapezoid_Integral(func, ref_x, ref_y, ref_y_dot, n, 2*accuracy);
	 Sn = 4 * T2n / 3 - Tn / 3;
	 return Sn;
 }
 
 /**
 *  @brief ���Ͽ���˹�����  
 *	@note	�����СҪ������ĵ������ƥ��
 * 	@param[in] 	func 			����㷨�ĺ���ָ��
 * 	@param[in] 	ref_x 		�ο���ĺ���������
 * 	@param[in] 	ref_y 		�ο��������������
 * 	@param[in] 	ref_y_dot �ο���ĵ�������
 *	@param[in] 	n 				�ο���ĸ���
 *  @param[in] 	accuracy 	��ȷ�ȣ������������仮�ֳɶ��ٸ�С����
 *  @return  		res				����ֵ
 */
 float Integral_Lib::Vortex_Integral(fitting_func func, const float* ref_x, const float* ref_y, const float* ref_y_dot, uint16_t n, uint16_t accuracy)
 {
	 float Sn, S2n; //���ø������η��ĵ��������
	 float Cn = 0;	//����ֵ
	 Sn = Parabola_Integral(func, ref_x, ref_y, ref_y_dot, n, accuracy);
	 S2n = Parabola_Integral(func, ref_x, ref_y, ref_y_dot, n, 2*accuracy);
	 Cn = 16 * S2n / 15 - Sn / 15;
	 return Cn;	 
 }
 
 /**
 *  @brief �����������  
 *	@note	�����СҪ������ĵ������ƥ��
 * 	@param[in] 	func 			����㷨�ĺ���ָ��
 * 	@param[in] 	ref_x 		�ο���ĺ���������
 * 	@param[in] 	ref_y 		�ο��������������
 * 	@param[in] 	ref_y_dot �ο���ĵ�������
 *	@param[in] 	n 				�ο���ĸ���
 *  @param[in] 	accuracy 	��ȷ�ȣ������������仮�ֳɶ��ٸ�С����
 *  @return  		res				����ֵ
 */
 float Integral_Lib::Romberg_Integral(fitting_func func, const float* ref_x, const float* ref_y, const float* ref_y_dot, uint16_t n, uint16_t accuracy)
 {
	 float Cn, C2n; //���ø������η��ĵ��������
	 float Rn = 0;	//����ֵ
	 Cn = Vortex_Integral(func, ref_x, ref_y, ref_y_dot, n, accuracy);
	 C2n = Vortex_Integral(func, ref_x, ref_y, ref_y_dot, n, 2*accuracy);
	 Rn = 64 * C2n / 63 - Cn / 63;
	 return Rn;	 	 
 } 
 
 /**
 *  @brief �����㷨��ʼ��
 *	@note	��ʹ�û��ֺ���ǰһ��Ҫ�ȳ�ʼ��
 * 	@param[in] 	void
 *  @return  		void
 */
 Integral_Lib::Integral_Lib(Interpolation a, Integral b , uint16_t ac)
 {
	 /* ѡ������㷨 */
	 switch(a)
	 {
		 case Lagrange: cal.fit = Lagrange_Interpolation; break;
		 case Newton: 	cal.fit = Newton_Interpolation; 	break;
		 case Hermite: 	cal.fit = Hermite_Interpolation; 	break;
	 }
	 cal.fit = Lagrange_Interpolation;
	 
	 /* ѡ������㷨 */
	 switch(b)
	 {
		 case Trapezoid: 	cal.integ = Trapezoid_Integral; 	break;
		 case Parabola:	 	cal.integ = Parabola_Integral; 		break;
		 case Vortex:			cal.integ = Vortex_Integral; 			break;
		 case Romberg:		cal.integ = Romberg_Integral; 		break;
	 }
	 
	 /* ���þ�ȷ�� */
	 cal.ac = 15;
 }
 
 /**
 *  @brief �����㷨(API����)
 *	@note	�����СҪ������ĵ������ƥ��
 * 	@param[in] 	ref_x 		�ο���ĺ���������
 * 	@param[in] 	ref_y 		�ο��������������
 * 	@param[in] 	ref_y_dot �ο���ĵ�������
 *	@param[in] 	n 				�ο���ĸ���
 *  @return  		I					����ֵ
 */
 float Integral_Lib::I(const float* ref_x, const float* ref_y, const float* ref_y_dot, uint16_t n)
 {
	 float I;
	 I = cal.integ(cal.fit, ref_x, ref_y, ref_y_dot, n, cal.ac);
	 return I;
 }

	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/


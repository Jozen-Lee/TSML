/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    tutu_filter.h
  * @author  buff buffdemail@163.com 
  * @brief   Filter set in general signal process and analysis.
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */
#ifndef TUTU_FILTERS_H
#define TUTU_FILTERS_H

/* Includes ------------------------------------------------------------------*/
#ifdef __cplusplus
#include <algorithm>
#include <string.h>
#include <stdio.h>
#include <math.h>
/* Exported function declarations --------------------------------------------*/

/* �˲��� */
class Filter
{
	public:
	Filter(){};
	~Filter(){};
	virtual	void operator >> (float& num){ num = out();}
  virtual	void operator << (const float& num){in(num);}
	virtual float f(float num)
	{
		in(num);
		return out();
	}		
		
  protected:
	virtual void in(float num){};
	virtual float out(void){ return 0;};	
};


/* ��ͨ�˲� */
class LowPassFilter : public Filter
{
  public:
  /**
    @brief trust (0,1) 
   */
	LowPassFilter(float trust = 1): Trust(trust)
    	{
      	now_num = last_num = 0;
    	} 
  	~LowPassFilter(){};
    float Trust;
		virtual float f(float num){return Filter::f(num);}
  protected:
  	virtual void in(float num);
  	virtual float out();
  private:
    float now_num;
    float last_num;
};


/* ��ֵ�˲�	*/
template<int Length> 	
class MedianFilter : public Filter
{
  /**
    @note �˲�����(1,100)
   */
  public:
	MedianFilter()
	{
		static_assert((Length>0)&&(Length<101),"MedianFilter Length [1,100]");
		flag = Length;
		where_num = 0;
	} 						
  	~MedianFilter(){}; 
		virtual float f(float num){return Filter::f(num);}
  protected:
  	virtual void in(float num)
    {
      now_num = num;
      /* flag=LengthȻ��ݼ���֤�����ڶ�����Ч��ֵ */
      flag > 0? flag-- : 0;										
      buffer_num[where_num++] = num;
      where_num %= Length; 
    }
    
  	virtual float out()
    {
      if(flag>0)
        return now_num;
      else
        {
          /* ׼������ */
          memcpy(sort_num,buffer_num,sizeof(sort_num));	
          std::sort(sort_num,sort_num+Length);
          return sort_num[int(Length/2)];
        }
    }  
  private:
  	float buffer_num[Length];
  	float sort_num[Length];
		float now_num;
		int flag,where_num;
};


/* ��ֵ�˲� */
template<int Length> 	
class MeanFilter : public Filter
{
  public:
  /**
    @note �˲�����(1,100)
   */
	MeanFilter()
	{
		static_assert((Length>0)&&(Length<101),"MedianFilter Length [1,100]");
		for(int x = 0 ; x < Length; x++) buffer_num[x] = 0;
		flag = Length;
		where_num = 0;
		sum = 0;
	} 						
  	~MeanFilter(){}; 
		virtual float f(float num){return Filter::f(num);}
  protected:
  	virtual void in(float num)
    {
      now_num = num;
      sum -= buffer_num[where_num];			  /*<! sum��ȥ��ֵ */
      sum += num;													/*<! sum������ֵ */
      buffer_num[where_num++] = num;
      flag > 0? flag-- : 0;								/*<!flag=LengthȻ��ݼ���֤�����ڶ�����Ч��ֵ */
      where_num %= Length; 
    }
    
  	virtual float out(void)
    {
      if(flag>0)
        return now_num;
      else
        return (sum/Length);
    }
		
		float Get_last_Data(void)
		{
			if(where_num) return buffer_num[where_num - 1];
			else return buffer_num[Length - 1];
		}
		
		int Get_flag(void)
		{
			return flag;
		}
		
  private:
  float buffer_num[Length];
	float now_num;
	float sum; 						/*<! ���Ⱥ����ֺ� */
	int flag,where_num;
};


/* ���޷��ľ�ֵ�˲� */
template<int Length> 	
class Lim_MeanFilter : public MeanFilter<Length>
{
	public:
		Lim_MeanFilter(float lim = 1e-1): limit(lim){}

		~Lim_MeanFilter() { MeanFilter<Length>::~MeanFilter();}
		virtual float f(float num){return MeanFilter<Length>::f(num);}
	protected:
		
		/* ��д���뺯�� */
		virtual void in(float num)
		{
			/* ��ȡ���ֵ */ 	
			deta = num - MeanFilter<Length>::Get_last_Data();
			if(fabs(deta) < limit || MeanFilter<Length>::Get_flag() == Length) 
			{
				MeanFilter<Length>::in(num);
			}			
		}
		
		virtual float out(void)
		{
			return MeanFilter<Length>::out();
		}
		
	private:
		float limit;
		float deta;
};


/* �����˲� */
class DeboundFilter : public Filter
{
  /**
    @note ���ȴ����Ƚϵ�С����ڼ�λ
   */
	public:
		DeboundFilter(unsigned char acc, unsigned short num = 6)
		{
			accuracy = 10 ^ acc;
			N = num;
			data = 0;
		}
		~DeboundFilter(){};
		virtual float f(float num){return Filter::f(num);}
	protected:
		virtual void in(float data_new)
		{
			if((int)data * accuracy == (int)data_new * accuracy) count = 0;
			else 
			{
				count ++;
				if(count == N)
				{
					data = data_new;
					count = 0;
				}
			}			
		}
		
		virtual float out(void)
		{
			return data;
		}
	
	private:
		unsigned char accuracy;	//����
		unsigned short N;				//���Ƚϴ���
		unsigned short count;		//������
		float data;
};


#endif

#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/


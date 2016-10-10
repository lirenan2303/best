#ifndef __SECOND_DATE_H__
#define __SECOND_DATE_H__

#include <stdint.h>

/// \brief  ����ʱ��Ľṹ��.
typedef struct {
	/// \brief  ��.
	/// �����2000��, ��ֵΪ0, 2001���ֵΪ2001, �Դ�����.
	uint8_t year;
	/// \brief  ��.
	/// ȡֵ��ΧΪ1-12.
	uint8_t month;
	/// \brief  ��.
	/// ȡֵ��ΧΪ1-31.
	uint8_t date;
	/// \brief  ����1-7.
	/// ȡֵ��ΧΪ1-7.
	uint8_t week;
	/// \brief  Сʱ.
	/// ȡֵ��ΧΪ0-23.
	uint8_t hour;
	/// \brief  ����.
	/// ȡֵ��ΧΪ0-59.
	uint8_t minute;
	/// \brief  ��.
	/// ȡֵ��ΧΪ0-59.
	uint8_t second;
} DateTime;


/// \brief  ����ת����ʱ��.
/// \param  second[in]     ��2000/1/1-00:00:00��֮��ʼ��ʱ������.
/// \param  dateTime[out]  ���ת���õ�ʱ��.
void SecondToDateTime(DateTime *dateTime, uint32_t second);

/// \brief  ��ʱ��ת������.
/// \param  dateTime[in]  ��Ҫת����ʱ��.
/// \return ��2000/1/1-00:00:00��֮��ʼ��ʱ������.
uint32_t DateTimeToSecond(const DateTime *dateTime);

unsigned int __OffsetNumbOfDay(const DateTime *dateTime);

#endif

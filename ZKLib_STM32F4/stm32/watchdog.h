#ifndef __WATCHDOG_H__
#define __WATCHDOG_H__


/// \brief  ���Ź���ʼ��.
/// ��ʼ��CPUƬ�Ͽ��Ź�, 5���и�λ.
void WatchdogInit(void);

/// \brief  ֹͣι���Ź�.
/// ��ִ���������֮���һ��ʱ��ϵͳ�������Ź�����.
void WatchdogResetSystem(void);

/// \brief  ι���Ź�.
/// �ú���������������ȼ�����ִ��, ÿ��һ����ι���Ź�.
void WatchdogFeed(void);

#endif

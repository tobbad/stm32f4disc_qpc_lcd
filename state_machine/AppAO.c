/*
 * AppAO.c
 *
 *  Created on: 01.01.2019
 *      Author: badi
 */
#include "qpc.h"
#include "bsp.h"
#include "AppSM.h"

typedef struct AppAO_t_ {
	/* protected: */
	QActive super;
	/* private: */
    QTimeEvt timeEvt;
} AppAO_t;

static QState AppAO_initial(AppAO_t * const me, QEvt const * const e);
static QState AppAO_base(AppAO_t * const me, QEvt const * const e);
static QState AppAO_on(AppAO_t * const me, QEvt const * const e);
static QState AppAO_off(AppAO_t * const me, QEvt const * const e);


static AppAO_t	app;

QActive * const AO_App = (QActive *)&app;

void AppAO_ctor(void)
{
	AppAO_t * me = &app;
	QActive_ctor(&me->super, (QStateHandler)&AppAO_initial);
    QTimeEvt_ctorX(&me->timeEvt, &me->super, TIMEOUT_SIG, 0U);
}

static QState AppAO_initial(AppAO_t * const me, QEvt const * const e)
{
	(void)e;
    QActive_subscribe(&me->super, BTNSH_ON);
    QActive_subscribe(&me->super, BTNSH_OFF);
    QTimeEvt_armX(&me->timeEvt, BSP_TICKS_PER_SEC/2, BSP_TICKS_PER_SEC/2);

    QS_SIG_DICTIONARY(TIMEOUT_SIG,         me);
    QS_FUN_DICTIONARY(&AppAO_base);
    QS_FUN_DICTIONARY(&AppAO_on);
    QS_FUN_DICTIONARY(&AppAO_off);

	return Q_TRAN(&AppAO_base);
}

static QState AppAO_base(AppAO_t * const me, QEvt const * const e)
{
	switch (e->sig)
	{
		case Q_INIT_SIG:
		{
			BSP_ledOff(LEDSH_RED);
			return Q_TRAN(&AppAO_off);
		}
		case BTNSH_ON:
		{
			BSP_ledOn(LEDSH_RED);
			return Q_HANDLED();
		}
		case BTNSH_OFF:
		{
			BSP_ledOff(LEDSH_RED);
			return Q_HANDLED();
		}
	}
	return Q_SUPER(&QHsm_top);
}


static QState AppAO_on(AppAO_t * const me, QEvt const * const e)
{
	switch (e->sig)
	{
		case Q_ENTRY_SIG:
		{
			BSP_ledOn(LEDSH_GREEN);
			return Q_HANDLED();
		}
		case TIMEOUT_SIG:
		{
			return Q_TRAN(&AppAO_off);
		}
	}
	return Q_SUPER(&AppAO_base);
}

static QState AppAO_off(AppAO_t * const me, QEvt const * const e)
{
	switch (e->sig)
	{
		case Q_ENTRY_SIG:
		{
			BSP_ledOff(LEDSH_GREEN);
			return Q_HANDLED();
		}
		case (TIMEOUT_SIG):
		{
			return Q_TRAN(&AppAO_on);
		}
	}
	return Q_SUPER(&AppAO_base);
}

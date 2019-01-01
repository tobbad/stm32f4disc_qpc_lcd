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
    QTimeEvt_armX(&me->timeEvt, BSP_TICKS_PER_SEC/2, BSP_TICKS_PER_SEC/2);
	return Q_TRAN(&AppAO_off);
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
	return Q_SUPER(&QHsm_top);
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
	return Q_SUPER(&QHsm_top);
}

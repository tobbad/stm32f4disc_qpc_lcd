/*
 * State machine for application
 *
 *  Created on: 01.01.2019
 *      Author: badi
 */

#ifndef STATE_MACHINE_APPSM_H_
#define STATE_MACHINE_APPSM_H_

/*
 * We need a couple of signals
 */
enum APPSignals
{
	BTNSH_ON  =  Q_USER_SIG,
	BTNSH_OFF,
    MAX_PUB_SIG,    /* the last published signal */

    TIMEOUT_SIG,    /*  */
    MAX_SIG         /* the last signal */
};

typedef struct key_event_t_
{
	/* protected: */
	QActive super;
	/* private: */
	btnsh_t btn;
} key_event_t;
#endif /* STATE_MACHINE_APPSM_H_ */

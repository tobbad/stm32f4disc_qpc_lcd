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
	Btnsh_up_on  =  Q_USER_SIG,
	Btnsh_up_off,
	Btnsh_down_on,
	Btnsh_down_off,
	Btnsh_left_on,
	Btnsh_left_off,
	Btnsh_right_on,
	Btnsh_right_off,
    MAX_PUB_SIG,    /* the last published signal */

    TIMEOUT_SIG,    /*  */
    MAX_SIG         /* the last signal */
};


#endif /* STATE_MACHINE_APPSM_H_ */

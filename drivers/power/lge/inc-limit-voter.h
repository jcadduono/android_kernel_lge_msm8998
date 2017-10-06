#ifndef _INC_LIMIT_VOTER_H_
#define _INC_LIMIT_VOTER_H_

#include <linux/list.h>
#include <linux/kernel.h>

#define INTVAL_GRINDER 10
#define LIMIT_RELEASED ((INT_MAX/INTVAL_GRINDER)*INTVAL_GRINDER)

enum limit_voter_type {
	LIMIT_VOTER_INVALID = -1,

	LIMIT_VOTER_IUSB = 0, LIMIT_VOTER_IBAT, LIMIT_VOTER_IDC,

	/* add 'limit_voter_type's here */

	LIMIT_VOTER_MAX,
};

struct limit_voter_entry {
	struct list_head node;
	int id;

	enum limit_voter_type type;
	const char* name;
	int limit; // in mA

	int (*effected)(void); // called-back when its voted value is effected.
	int (*activated)(void); // called-back when its charger source is attached.
	int (*deactivated)(void); // called-back when its charger source is detached.
};

struct limit_voter_list {
	struct list_head head;
	struct mutex lock;
	enum limit_voter_type type;
};

static inline union power_supply_propval vote_make(
		enum limit_voter_type limit_type, int limit_current) {
	union power_supply_propval vote = { .intval = (limit_current
			/ INTVAL_GRINDER) * INTVAL_GRINDER + limit_type };
	return vote;
}

static inline enum limit_voter_type vote_type(
		const union power_supply_propval* vote) {
	enum limit_voter_type type = LIMIT_VOTER_INVALID;

	switch (vote->intval % INTVAL_GRINDER) {
	case LIMIT_VOTER_IUSB:
		type = LIMIT_VOTER_IUSB;
		break;
	case LIMIT_VOTER_IBAT:
		type = LIMIT_VOTER_IBAT;
		break;
	case LIMIT_VOTER_IDC:
		type = LIMIT_VOTER_IDC;
		break;
	default:
		break;
	}

	return type;
}

static inline int vote_current(const union power_supply_propval* vote) {
	return (vote->intval / INTVAL_GRINDER) * INTVAL_GRINDER;
}

#endif

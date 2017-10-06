#define pr_fmt(fmt) "LIMIT-VOTER: %s: " fmt, __func__
#define pr_voter(fmt, ...) pr_info(fmt, ##__VA_ARGS__)

#include <linux/power_supply.h>
#include "inc-limit-voter.h"

static struct limit_voter_list voter_list_iusb = { .head = LIST_HEAD_INIT(
		voter_list_iusb.head),
		.lock = __MUTEX_INITIALIZER(voter_list_iusb.lock), .type =
				LIMIT_VOTER_IUSB, };
static struct limit_voter_list voter_list_ibat = { .head = LIST_HEAD_INIT(
		voter_list_ibat.head),
		.lock = __MUTEX_INITIALIZER(voter_list_ibat.lock), .type =
				LIMIT_VOTER_IBAT, };
static struct limit_voter_list voter_list_idc = { .head = LIST_HEAD_INIT(
		voter_list_idc.head), .lock = __MUTEX_INITIALIZER(voter_list_idc.lock),
		.type = LIMIT_VOTER_IDC, };

static struct limit_voter_list* voter_list_get_by_type(
		enum limit_voter_type type) {
	switch (type) {
	case LIMIT_VOTER_IUSB:
		return &voter_list_iusb;
	case LIMIT_VOTER_IBAT:
		return &voter_list_ibat;
	case LIMIT_VOTER_IDC:
		return &voter_list_idc;

	default:
		pr_voter("Invalid limit voter type\n");
		return NULL;
	}
}

static struct limit_voter_list* voter_list_get_by_entry(
		struct limit_voter_entry* entry) {
	return voter_list_get_by_type(entry->type);
}

static int atomic_effecting_limit(struct limit_voter_list* list) {
	int effecting_limit = LIMIT_RELEASED;
	struct limit_voter_entry* iter;

	list_for_each_entry(iter, &list->head, node)
	{
		if (iter->limit < effecting_limit) {
			effecting_limit = iter->limit;
		}
	}

	return effecting_limit;
}

static int atomic_voter_id(void) {
	static int voter_id = 0;
	return voter_id++;
}

static void signal_to_veneer(enum limit_voter_type type, int limit) {
	struct power_supply* psy_veneer = power_supply_get_by_name(
			"battery-veneer");
	if (psy_veneer && psy_veneer->set_property) {
		union power_supply_propval value = vote_make(type, limit);
		psy_veneer->set_property(psy_veneer,
				POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &value);
	}
}

void limit_voter_register(struct limit_voter_entry* entry, const char* name,
		enum limit_voter_type type, int (*effected)(void),
		int (*activated)(void), int (*deactivated)(void)) {
	struct limit_voter_list* list_to = voter_list_get_by_type(type);

	entry->type = type;
	entry->name = name;
	entry->limit = LIMIT_RELEASED;

	entry->effected = effected;
	entry->activated = activated;
	entry->deactivated = deactivated;

	mutex_lock(&list_to->lock);
	entry->id = atomic_voter_id();
	list_add(&entry->node, &list_to->head);
	mutex_unlock(&list_to->lock);
}

void limit_voter_unregister(struct limit_voter_entry* entry) {
	struct limit_voter_list* list_from = voter_list_get_by_entry(entry);

	mutex_lock(&list_from->lock);
	list_del(&entry->node);
	mutex_unlock(&list_from->lock);
}

void limit_voter_set(struct limit_voter_entry* voter, int limit) {
	if (voter->limit != limit) {
		struct limit_voter_list* list_of = voter_list_get_by_entry(voter);
		int effecting_limit;

		mutex_lock(&list_of->lock);

		voter->limit = limit;
		effecting_limit = atomic_effecting_limit(list_of);

		if (effecting_limit != LIMIT_RELEASED) {
			struct limit_voter_entry* iter;

			list_for_each_entry(iter, &list_of->head, node)
			{
				if (iter->limit == effecting_limit) {
					iter->effected();
				}
			}
		}

		mutex_unlock(&list_of->lock);

		signal_to_veneer(list_of->type, effecting_limit);
	} else
		pr_voter("voting values are same, %d\n", limit);
}

void limit_voter_release(struct limit_voter_entry* voter) {
	limit_voter_set(voter, LIMIT_RELEASED);
}

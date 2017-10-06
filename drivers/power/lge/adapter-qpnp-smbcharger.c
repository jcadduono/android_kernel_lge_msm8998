/*
 * CAUTION! :
 * 		This file will be included at the end of "qpnp-smbcharger.c".
 * 		So "qpnp-smbcharger.c" should be touched before you start to build.
 * 		If not, your work will not be applied to built image
 * 		because the build system doesn't care the update time of this file.
 */

static enum power_supply_property smbchg_battery_properties_append[] = {
		POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, };

static enum power_supply_property _smbchg_battery_properties_ext[ARRAY_SIZE(smbchg_battery_properties) + ARRAY_SIZE(smbchg_battery_properties_append)] = { 0, };

static int smbchg_battery_get_property_pre(struct power_supply *psy,
		enum power_supply_property prop, union power_supply_propval *val) {
	int rc = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		// TODO
		break;

	default:
		rc = -ENOENT;
	}

	return rc;
}

static int smbchg_battery_get_property_post(struct power_supply *psy,
		enum power_supply_property prop, union power_supply_propval *val,
		int rc) {
	return rc;
}

static int smbchg_battery_set_property_pre(struct power_supply *psy,
		enum power_supply_property prop, const union power_supply_propval *val) {
	int rc = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		break;

	default:
		rc = -ENOENT;
	}

	return rc;
}

static int smbchg_battery_set_property_post(struct power_supply *psy,
		enum power_supply_property prop, const union power_supply_propval *val,
		int rc) {
	return rc;
}

/////////////////////////////////////////////////////////////////////////////////////////////////

int smbchg_battery_get_property_ext(struct power_supply *psy,
		enum power_supply_property prop, union power_supply_propval *val) {

	int rc = smbchg_battery_get_property_pre(psy, prop, val);
	if (rc == -ENOENT || rc == -EAGAIN)
		rc = smbchg_battery_get_property(psy, prop, val);
	rc = smbchg_battery_get_property_post(psy, prop, val, rc);

	return rc;
}

int smbchg_battery_set_property_ext(struct power_supply *psy,
		enum power_supply_property prop, const union power_supply_propval *val) {

	int rc = smbchg_battery_set_property_pre(psy, prop, val);
	if (rc == -ENOENT || rc == -EAGAIN)
		rc = smbchg_battery_set_property(psy, prop, val);
	rc = smbchg_battery_set_property_post(psy, prop, val, rc);

	return rc;
}

int smbchg_battery_is_writeable_ext(struct power_supply *psy,
		enum power_supply_property prop) {
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		rc = 1;
		break;
	default:
		rc = smbchg_battery_is_writeable(psy, prop);
		break;
	}
	return rc;
}

enum power_supply_property* smbchg_battery_properties_ext(void) {
	int size_original = ARRAY_SIZE(smbchg_battery_properties);
	int size_appended = ARRAY_SIZE(smbchg_battery_properties_append);

	memcpy(_smbchg_battery_properties_ext, smbchg_battery_properties,
			size_original*sizeof(enum power_supply_property));
	memcpy(&_smbchg_battery_properties_ext[size_original],
			smbchg_battery_properties_append, size_appended*sizeof(enum power_supply_property));

	pr_smb(PR_STATUS, "show extended properties\n"); {
		int i;
		for( i=0; i<ARRAY_SIZE(_smbchg_battery_properties_ext); ++i ) {
			pr_smb(PR_STATUS, "%d : %d\n", i, _smbchg_battery_properties_ext[i]);
		}
	}

	return _smbchg_battery_properties_ext;
}

size_t smbchg_battery_num_properties_ext(void) {
	return ARRAY_SIZE(_smbchg_battery_properties_ext);
}
